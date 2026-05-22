// =============================================================================
// Beamforming_synth.v  -  DAS Beamformer  (Synthesisable, Multi-Pixel)
// =============================================================================
//
// ── BUG FIXES FROM Beamforming_sim.sv (unchanged) ────────────────────────────
// [BUG 1]  $sformatf in $readmemh unsynthesisable → generate-if literal filenames
// [BUG 2]  current_addr bound "< 2048" discarded batches 1-4 → guard removed
// [BUG 3]  rom_addr "< 0" always false (unsigned) → signed [9:0] arithmetic
// [BUG 4]  dont_touch on norm_full_res blocked DSP48E1 → use_dsp48
//
// ── INTERFACE CHANGES (unchanged from previous synth version) ─────────────────
// [CHANGE 1-4]  Parallel-load interface, center_idx reg, S_LOAD removed,
//               reset_calc propagates system reset.
//
// ── NEW FIX - CORDIC REUSE / BATCH TIMEOUT ───────────────────────────────────
//
// ROOT CAUSE OF UART TIMEOUT:
//   Delay_calc.v (NOTE): "cordic_sqrt never clears its busy flag after
//   completion.  A second start_calc pulse will be silently ignored."
//   The old 1-cycle auto-reset (reset_calc = ~(calc_valid[0] & ~delayed))
//   MAY not deassert busy reliably: reset_calc=0 for only 1 bf_clk cycle,
//   but busy is still 1 at the CORDIC input.  When start_calc fires 5 cycles
//   later, the CORDIC sees busy=1 → ignores the pulse → vld_out never rises
//   → &calc_valid never true → FSM stuck in S_WAIT forever → pixel_valid
//   never fires → wrapper stuck in WS_BF_WAIT → UART sends 0 bytes.
//
// FIX - S_BRST (batch-reset) state:
//   A dedicated 4-cycle reset state replaces the 1-cycle wire pulse.
//   reset_calc = 0 while state == S_BRST, 1 otherwise.
//   4 cycles (80 ns at 50 MHz) is more than enough for any synchronous
//   reset implementation to clear busy.  start_calc fires on the cycle
//   S_BRST exits to S_WAIT, propagating through start_pipe[4:0] (5 stages)
//   to the CORDIC input 5 cycles later, at which point reset has been
//   released for 5 cycles and busy is guaranteed to be 0.
//
//   S_NEXT is removed; its accumulator-timing role is absorbed by S_BRST:
//   acc_valid_pipe[4] fires 4 cycles after &calc_valid, which is exactly
//   at the end of the 4-cycle S_BRST window → accumulation is triggered
//   before the next batch starts. ✓
//
// FIX - pixel_valid timing:
//   Previous: valid_delay shifted whenever last_batch_done && state!=S_IDLE
//             → pixel_valid fired ~42 cycles BEFORE batch 4 completed
//             → final_pixel_val was stale / zero at the time of CDC transfer.
//   Fixed:    valid_delay shifts only when state == S_DONE.
//             Accumulation completes by S_DONE cycle 4, normalisation by
//             cycle 8; pixel_valid fires at cycle 11 - 3 cycles of margin. ✓
//
// ── REVISED FSM ──────────────────────────────────────────────────────────────
//   S_IDLE → S_WAIT (start_calc fired for batch 0)
//   S_WAIT → S_BRST (when &calc_valid, i.e. all Delay_calc done)
//   S_BRST → S_WAIT (if !last_batch_done: batch_idx++, start_calc for next batch)
//   S_BRST → S_DONE (if  last_batch_done)
//   S_DONE → S_IDLE via new start (re-entrant / multi-pixel)
//
//   Accumulator timing from &calc_valid rising:
//     acc_valid_pipe[0] at S_BRST cycle 0
//     BRAM→adder stages 1-4: S_BRST cycles 0-3
//     acc_valid_pipe[4] at S_BRST cycle 4 → end of S_BRST →
//       accumulation triggered on first S_DONE cycle ✓  (or first S_WAIT cycle
//       of the next batch, which is also correct since batch_idx has incremented)
//     acc_valid_pipe[6] at S_DONE cycle 2 → normalisation triggered ✓
//     final_pixel_val stable at S_DONE cycle 3 ✓
//     pixel_valid fires at S_DONE cycle 11 ✓
//
// ── MEMORY FILES ─────────────────────────────────────────────────────────────
//   element_positions.mem          128 entries, Q6.26 hex
//   rf_data_0.mem .. rf_data_9.mem 10240 entries each, 32-bit hex
// =============================================================================

`timescale 1ns / 1ps

module beamforming_synth #(
    parameter DATA_WIDTH    = 32,
    parameter APERTURE_SIZE = 50,
    parameter CALC_INST     = 10,
    parameter ADDR_W        = 11,
    parameter BRST_CYCLES   = 4    // bf_clk cycles reset_calc=0 between batches.
                                   // Must be ≥ 2; 4 gives comfortable margin for
                                   // CORDIC busy to clear after reset.
)(
    input  wire        clk,
    input  wire        reset,           // Active-LOW

    input  wire                          start,
    input  wire signed [DATA_WIDTH-1:0]  x_pixel_in,
    input  wire signed [DATA_WIDTH-1:0]  z_pixel_in,
    input  wire signed [DATA_WIDTH-1:0]  start_depth_in,
    input  wire [7:0]                    center_idx_in,

    output reg  signed [DATA_WIDTH-1:0]  final_pixel_val,
    output reg                           pixel_valid
);

    // =========================================================================
    // Pixel parameter registers (latched on start)
    // =========================================================================
    reg signed [DATA_WIDTH-1:0] x_pixel_reg, z_pixel_reg, start_depth_reg;
    reg [7:0]                   center_idx;

    wire signed [DATA_WIDTH-1:0] x_pixel     = x_pixel_reg;
    wire signed [DATA_WIDTH-1:0] z_pixel     = z_pixel_reg;
    wire signed [DATA_WIDTH-1:0] start_depth = start_depth_reg;

    // =========================================================================
    // Element-position ROM (128 entries, Q6.26, loaded from .mem file)
    // =========================================================================
    reg signed [DATA_WIDTH-1:0] element_x_rom [0:127];
    initial $readmemh("element_positions.mem", element_x_rom);

    // =========================================================================
    // Delay_calc control signals
    // =========================================================================
    wire [CALC_INST-1:0] calc_valid;
    reg                  calc_valid_delayed;

    always @(posedge clk)
        calc_valid_delayed <= calc_valid[0];

    // [CORDIC FIX]  reset_calc = 0 whenever:
    //   a) system reset is asserted (!reset)
    //   b) FSM is in S_BRST (dedicated batch-reset state)
    // This gives the CORDIC busy flag 4 full cycles to clear before the
    // next start_calc pulse is issued.
    wire reset_calc;   // driven below after state declaration

    // =========================================================================
    // RF sample arrays
    // =========================================================================
    wire [ADDR_W-1:0]            rf_addr_temp [0:CALC_INST-1];
    reg  signed [DATA_WIDTH-1:0] sampled_data [0:CALC_INST-1];

    // =========================================================================
    // FSM
    // =========================================================================
    localparam S_IDLE = 3'd0;
    localparam S_WAIT = 3'd1;   // waiting for &calc_valid (batch computing)
    localparam S_BRST = 3'd2;   // reset Delay_calc between batches [NEW]
    localparam S_DONE = 3'd3;   // all batches complete, result stable

    reg [2:0] state;
    reg [2:0] batch_idx;                // 0..4  (5 batches × 10 cores = 50 elements)
    reg [2:0] brst_cnt;                 // counter inside S_BRST
    reg       start_calc;

    wire last_batch_done = (batch_idx == 3'd4);

    // reset_calc: 0 (assert reset) during system reset OR S_BRST state
    assign reset_calc = (!reset) ? 1'b0 : (state != S_BRST);

    // acc_valid_pipe timing from &calc_valid rising (at S_WAIT → S_BRST):
    //   pipe[4] fires at end of S_BRST (cycle 4 of 4) → accumulate
    //   pipe[6] fires at S_DONE cycle 2                → normalise
    (* dont_touch = "yes" *) reg [7:0] acc_valid_pipe;
    reg                                acc_valid_pipe_d;

    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            state           <= S_IDLE;
            batch_idx       <= 3'd0;
            brst_cnt        <= 3'd0;
            start_calc      <= 1'b0;
            x_pixel_reg     <= 32'sd0;
            z_pixel_reg     <= 32'sd0;
            start_depth_reg <= 32'sd0;
            center_idx      <= 8'd0;

        end else begin
            start_calc <= 1'b0;   // default: deasserted every clock

            case (state)
                // ── S_IDLE ───────────────────────────────────────────────────
                S_IDLE: begin
                    if (start) begin
                        x_pixel_reg     <= x_pixel_in;
                        z_pixel_reg     <= z_pixel_in;
                        start_depth_reg <= start_depth_in;
                        center_idx      <= center_idx_in;
                        batch_idx       <= 3'd0;
                        start_calc      <= 1'b1;   // fire batch 0
                        state           <= S_WAIT;
                    end
                end

                // ── S_WAIT ───────────────────────────────────────────────────
                // Wait for all CALC_INST Delay_calc instances to assert vld_out.
                // reset_calc = 1 here (CORDIC computing normally).
                S_WAIT: begin
                    if (&calc_valid) begin
                        brst_cnt <= 3'd0;
                        state    <= S_BRST;
                    end
                end

                // ── S_BRST ───────────────────────────────────────────────────
                // [NEW]  Hold reset_calc = 0 for BRST_CYCLES to ensure the
                // CORDIC's busy flag is fully cleared before the next batch.
                // The adder tree (4 pipeline stages) runs in parallel so
                // final_accumulator_input is stable by the end of this state.
                S_BRST: begin
                    if (brst_cnt == BRST_CYCLES - 1) begin
                        brst_cnt <= 3'd0;
                        if (last_batch_done) begin
                            state <= S_DONE;
                        end else begin
                            batch_idx  <= batch_idx + 3'd1;
                            start_calc <= 1'b1;   // fire next batch
                            state      <= S_WAIT;
                        end
                    end else begin
                        brst_cnt <= brst_cnt + 3'd1;
                    end
                end

                // ── S_DONE ───────────────────────────────────────────────────
                // Re-entrant: a new start immediately restarts for the next pixel.
                S_DONE: begin
                    if (start) begin
                        x_pixel_reg     <= x_pixel_in;
                        z_pixel_reg     <= z_pixel_in;
                        start_depth_reg <= start_depth_in;
                        center_idx      <= center_idx_in;
                        batch_idx       <= 3'd0;
                        start_calc      <= 1'b1;
                        state           <= S_WAIT;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Generate: CALC_INST parallel Delay_calc + BRAM pairs
    // =========================================================================
    genvar i;
    generate
        for (i = 0; i < CALC_INST; i = i + 1) begin : gen_cores

            (* dont_touch = "yes" *) (* ram_style = "block" *)
            reg [DATA_WIDTH-1:0] local_rf_mem [0:10239];

            // [BUG 1 FIX] Literal filenames - $sformatf is not synthesisable.
            if      (i == 0) begin : rf_init initial $readmemh("rf_data_0.mem", local_rf_mem); end
            else if (i == 1) begin : rf_init initial $readmemh("rf_data_1.mem", local_rf_mem); end
            else if (i == 2) begin : rf_init initial $readmemh("rf_data_2.mem", local_rf_mem); end
            else if (i == 3) begin : rf_init initial $readmemh("rf_data_3.mem", local_rf_mem); end
            else if (i == 4) begin : rf_init initial $readmemh("rf_data_4.mem", local_rf_mem); end
            else if (i == 5) begin : rf_init initial $readmemh("rf_data_5.mem", local_rf_mem); end
            else if (i == 6) begin : rf_init initial $readmemh("rf_data_6.mem", local_rf_mem); end
            else if (i == 7) begin : rf_init initial $readmemh("rf_data_7.mem", local_rf_mem); end
            else if (i == 8) begin : rf_init initial $readmemh("rf_data_8.mem", local_rf_mem); end
            else             begin : rf_init initial $readmemh("rf_data_9.mem", local_rf_mem); end

            // [BUG 3 FIX] Signed 10-bit so the < 0 lower clamp actually fires.
            wire signed [9:0] rom_addr_s =
                  $signed({2'b00, center_idx})
                + ($signed({7'b0, batch_idx}) * 10'sd10)
                + $signed(10'd0 + i)
                - 10'sd25;

            wire [6:0] rom_addr =
                (rom_addr_s < $signed(10'd0))   ? 7'd0   :
                (rom_addr_s > $signed(10'd127)) ? 7'd127 :
                 rom_addr_s[6:0];

            // reset_calc = 0 during S_BRST and system reset → CORDIC cleared.
            (* keep_hierarchy = "yes" *)
            Delay_calc delay_unit (
                .clk        (clk),
                .reset      (reset_calc),
                .start_calc (start_calc),
                .x_pixel    (x_pixel),
                .z_pixel    (z_pixel),
                .start_depth(start_depth),
                .x_elem     (element_x_rom[rom_addr]),
                .rf_addr    (rf_addr_temp[i]),
                .vld_out    (calc_valid[i])
            );

            // [BUG 2 FIX] 14-bit address, no guard.  Max = 2047 + 4×2048 = 10239.
            wire [13:0] current_addr =
                {3'b000, rf_addr_temp[i]} + ({11'b0, batch_idx} << 11);

            always @(posedge clk)
                sampled_data[i] <= local_rf_mem[current_addr];

        end
    endgenerate

    // =========================================================================
    // 4-Stage Balanced Adder Tree  (10 inputs → final_accumulator_input)
    // =========================================================================
    (* dont_touch = "yes" *) reg signed [DATA_WIDTH+4:0] batch_sum_stg1 [0:4];
    (* dont_touch = "yes" *) reg signed [DATA_WIDTH+5:0] batch_sum_stg2 [0:2];
    (* dont_touch = "yes" *) reg signed [DATA_WIDTH+6:0] current_batch_final;
    (* dont_touch = "yes" *) reg signed [DATA_WIDTH+7:0] final_accumulator_input;

    always @(posedge clk) begin
        batch_sum_stg1[0] <= $signed(sampled_data[0]) + $signed(sampled_data[1]);
        batch_sum_stg1[1] <= $signed(sampled_data[2]) + $signed(sampled_data[3]);
        batch_sum_stg1[2] <= $signed(sampled_data[4]) + $signed(sampled_data[5]);
        batch_sum_stg1[3] <= $signed(sampled_data[6]) + $signed(sampled_data[7]);
        batch_sum_stg1[4] <= $signed(sampled_data[8]) + $signed(sampled_data[9]);

        batch_sum_stg2[0] <= $signed(batch_sum_stg1[0]) + $signed(batch_sum_stg1[1]);
        batch_sum_stg2[1] <= $signed(batch_sum_stg1[2]) + $signed(batch_sum_stg1[3]);
        batch_sum_stg2[2] <= $signed(batch_sum_stg1[4]);

        current_batch_final     <= $signed(batch_sum_stg2[0]) + $signed(batch_sum_stg2[1]);
        final_accumulator_input <= $signed(current_batch_final) + $signed(batch_sum_stg2[2]);
    end

    // =========================================================================
    // Accumulator + Normalisation
    // =========================================================================
    (* dont_touch = "yes" *) reg signed [DATA_WIDTH+10:0] accumulator;
    (* use_dsp48 = "yes"  *) reg [DATA_WIDTH+31:0]        norm_full_res; // [BUG 4 FIX]

    localparam [DATA_WIDTH-1:0] NORM_FACTOR = 32'd20972;   // ≈ 2^20 / 50

    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            accumulator      <= {(DATA_WIDTH+11){1'b0}};
            final_pixel_val  <= {DATA_WIDTH{1'b0}};
            acc_valid_pipe   <= 8'b0;
            acc_valid_pipe_d <= 1'b0;
            norm_full_res    <= {(DATA_WIDTH+32){1'b0}};

        end else begin
            acc_valid_pipe   <= {acc_valid_pipe[6:0], &calc_valid};
            acc_valid_pipe_d <= acc_valid_pipe[4];

            // acc_valid_pipe[4] fires at the end of S_BRST (4 cycles after
            // &calc_valid), when final_accumulator_input is stable.
            if (acc_valid_pipe[4] && !acc_valid_pipe_d) begin
                if (batch_idx == 3'd0)
                    accumulator <= $signed(final_accumulator_input);
                else
                    accumulator <= $signed(accumulator) + $signed(final_accumulator_input);
            end

            // acc_valid_pipe[6] fires at S_DONE cycle 2.
            if (state == S_DONE && acc_valid_pipe[6])
                norm_full_res <= $signed(accumulator) * $signed(NORM_FACTOR);

            if (state == S_DONE)
                final_pixel_val <= $signed(norm_full_res) >>> 20;
        end
    end

    // =========================================================================
    // pixel_valid  (10-cycle shift-register delay, fires only in S_DONE)
    //
    // [PIXEL_VALID FIX]
    //   Old condition: last_batch_done && state != S_IDLE
    //     → fired ~42 bf_clk cycles before batch 4 completed (during S_WAIT),
    //       sending stale/zero final_pixel_val through the CDC.
    //   New condition: state == S_DONE
    //     → fires only after all batches are accumulated and normalised.
    //     final_pixel_val stable by S_DONE cycle 3.
    //     pixel_valid fires at S_DONE cycle 11.  3 cycles of margin. ✓
    // =========================================================================
    reg [9:0] valid_delay;

    always @(posedge clk) begin
        if (!reset || start)
            valid_delay <= 10'b0;
        else if (state == S_DONE)   // [FIX] was: last_batch_done && state != S_IDLE
            valid_delay <= {valid_delay[8:0], 1'b1};

        pixel_valid <= valid_delay[9];
    end

endmodule