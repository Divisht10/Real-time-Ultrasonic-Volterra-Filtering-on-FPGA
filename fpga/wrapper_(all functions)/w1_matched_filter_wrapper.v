// ============================================================================
// File        : w1_matched_filter_wrapper.v
// Description : W1 - Matched Filter Wrapper
//               Implements MATLAB lines 31-53, 93-98 from volterra_code.m
//
// MATLAB Equivalent Functions:
//   custom_interp1()  → PRECOMPUTED offline → stored in fw1_coeffs.mem / fw2_coeffs.mem
//   custom_conv()     → matched_filter_1d   (convolution.v)
//
// Data Flow (from block diagram):
//   tw1 (1×545 @ 250MHz) → interp → ww1 (1×68 @ Fs)  --→  FW1 = [zeros(68) ww1]
//   tw2 (1×68  @ 250MHz) → interp → ww2 (1×68 @ Fs)  --→  FW2 = [zeros(68) ww2]
//   RData (1792×128) → normalize → x_norm (1792×128)
//   x_norm' (128×1792) → conv(FW1) → x1_conv (128×1927) → x1 (1927×128)
//   x_norm' (128×1792) → conv(FW2) → x2_conv (128×1927) → x2 (1927×128)
//
// 8-Bit Serial Interface:
//   - Each 32-bit RF sample is streamed as 4 bytes, MSB first
//   - RF data sent row-major: [ch0_s0, ch1_s0, ..., ch127_s0, ch0_s1 ...]
//     (all 128 channels for sample 0, then sample 1, etc.)
//
// Precomputed ROM Files Required (generate from MATLAB using gen_fpga_roms.m):
//   fw1_coeffs.mem  : 136 lines × 8 hex chars (Q2.30, FW1 taps)
//   fw2_coeffs.mem  : 136 lines × 8 hex chars (Q2.30, FW2 taps)
//
// Output:
//   Dual-port BRAM accessible by W2 wrapper.
//   Separate x1 (fundamental) and x2 (subharmonic) filtered outputs.
//
// Author      : Auto-generated wrapper
// ============================================================================

`timescale 1ns/1ps

module w1_matched_filter_wrapper #(
    parameter DATA_WIDTH  = 32,    // Q2.30 fixed-point internal width
    parameter ACCUM_WIDTH = 64,    // Accumulator width in matched_filter_1d
    parameter TAPS        = 136,   // FW length = 2 × 68 (68 zeros + 68 chirp samples)
    parameter L           = 1792,  // Axial samples per channel (Receive.endSample in MATLAB)
    parameter PAD_LEN     = 135,   // TAPS-1 leading zeros for causal linear convolution
    parameter TOTAL_IN    = 1927,  // L + PAD_LEN → full convolution length = 1927
    parameter NUM_CH      = 128,   // Number of receive channels (Trans.numelements)
    parameter OUT_SHIFT   = 28     // Shift accumulator to fit Q2.30 output: 64-28=Q2.30
)(
    input  wire        clk,
    input  wire        rst_n,      // Active-low reset

    // -------------------------------------------------------
    // 8-bit Serial Input (from ADC / DMA controller)
    // RF data: row-major order [ch0..ch127 for sample 0], [ch0..ch127 for sample 1]..
    // -------------------------------------------------------
    input  wire [7:0]  data_in,
    input  wire        data_valid,  // High for one cycle when data_in is valid byte
    input  wire        start,       // Pulse high to begin loading RF frame

    // -------------------------------------------------------
    // Status
    // -------------------------------------------------------
    output reg         w1_done,    // Pulses high for 1 cycle when x1, x2 BRAMs are ready
    output reg         w1_busy,    // High while processing

    // -------------------------------------------------------
    // Output BRAM Read Port (W2 reads filtered RF data)
    // Access: addr = channel * L + sample_index
    // Valid after w1_done goes high
    // -------------------------------------------------------
    input  wire [17:0] rd_addr,    // = ch[6:0] * 1792 + sample[10:0]  (max 128*1792=229376)
    output wire [31:0] rd_data_x1, // Fundamental filtered sample
    output wire [31:0] rd_data_x2  // Subharmonic filtered sample (feeds W2 → W3)
);

// ============================================================================
// Internal Parameters
// ============================================================================
localparam BRAM_DEPTH  = NUM_CH * L;   // 229376 entries
localparam IN_BRAM_D   = NUM_CH * L;

// FSM States
localparam S_IDLE       = 3'd0;
localparam S_LOAD_RF    = 3'd1;  // Receive 8-bit bytes → assemble 32-bit words → store in rf_in_bram
localparam S_PAD_ZEROS  = 3'd2;  // Feed PAD_LEN zeros to prime matched filter pipeline
localparam S_FILT_RUN   = 3'd3;  // Feed L RF samples for current channel through both filters
localparam S_FILT_FLUSH = 3'd4;  // Feed zeros until all L valid outputs are captured
localparam S_NEXT_CH    = 3'd5;  // Advance to next channel or finish
localparam S_DONE       = 3'd6;

// ============================================================================
// Byte-to-Word Assembler (8-bit → 32-bit, MSB first)
// ============================================================================
reg [31:0] word_buf;
reg [1:0]  byte_idx;
reg        word_valid;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        word_buf   <= 32'h0;
        byte_idx   <= 2'h0;
        word_valid <= 1'b0;
    end else begin
        word_valid <= 1'b0;
        if (data_valid) begin
            word_buf  <= {word_buf[23:0], data_in};  // Shift in MSB first
            byte_idx  <= byte_idx + 2'h1;
            if (byte_idx == 2'd3)
                word_valid <= 1'b1;                  // Full 32-bit word ready
        end
    end
end

// ============================================================================
// Input BRAM : 128 channels × 1792 samples (raw normalized RF data)
// Written during S_LOAD_RF; read during S_FILT_RUN
// MATLAB: x_norm = x / max(abs(x))  → Q2.30 representation
// ============================================================================
(* ram_style = "block" *) reg [DATA_WIDTH-1:0] rf_in_bram  [0:BRAM_DEPTH-1];

// Input BRAM write (S_LOAD_RF): row-major order ch0_s0, ch1_s0 .. ch127_s0, ch0_s1 ..
// Addressing: sample_index * NUM_CH + channel_index
reg [17:0] in_wr_addr;

always @(posedge clk) begin
    if (word_valid && w1_busy)
        rf_in_bram[in_wr_addr] <= word_buf;
end

// ============================================================================
// Output BRAMs : 128 channels × 1792 samples of filtered data
// rf_out_bram_x1 : fundamental (FW1)
// rf_out_bram_x2 : subharmonic (FW2) → feeds W2 → W3 Volterra
// Addressing: channel_index * L + sample_index
// ============================================================================
(* ram_style = "block" *) reg [DATA_WIDTH-1:0] rf_out_bram_x1 [0:BRAM_DEPTH-1];
(* ram_style = "block" *) reg [DATA_WIDTH-1:0] rf_out_bram_x2 [0:BRAM_DEPTH-1];

// Read ports (W2 accesses these)
assign rd_data_x1 = rf_out_bram_x1[rd_addr];
assign rd_data_x2 = rf_out_bram_x2[rd_addr];

// ============================================================================
// Matched Filter Instances
// Both receive the SAME input stream; differentiated by coefficient ROMs.
//
// IMPORTANT: matched_filter_1d's internal coeffs[] is uninitialised in
// convolution.v. Add these lines at the top of each instance's initial block:
//   For mf_fw1: initial $readmemh("fw1_coeffs.mem", coeffs);
//   For mf_fw2: initial $readmemh("fw2_coeffs.mem", coeffs);
//
// fw1_coeffs.mem / fw2_coeffs.mem are generated from MATLAB (see gen_fpga_roms.m):
//   FW1 = [zeros(1, length(ww1)), ww1];  % 1×136 in Q2.30 hex
//   FW2 = [zeros(1, length(ww2)), ww2];  % 1×136 in Q2.30 hex
// ============================================================================
reg  [DATA_WIDTH-1:0] mf_din;       // Shared input: both filters process same sample
reg                   mf_din_valid;
wire [ACCUM_WIDTH-1:0] mf1_dout, mf2_dout;
wire                   mf1_valid,  mf2_valid;

// FW1 - Fundamental chirp matched filter
// Coefficient ROM: fw1_coeffs.mem (add $readmemh to matched_filter_1d or modify wrapper)
matched_filter_1d_rom #(
    .DATA_WIDTH (DATA_WIDTH),
    .TAPS       (TAPS),
    .ACCUM_WIDTH(ACCUM_WIDTH),
    .ROW_LENGTH (TOTAL_IN), 
    .ROM_FILE   ("fw1_coeffs.mem")         // 1927: resets col_count so filters re-arm per channel
) mf_fw1 (
    .clk       (clk),
    .rst       (~rst_n),            // matched_filter_1d uses active-high rst
    .din       (mf_din),
    .din_valid (mf_din_valid),
    .dout      (mf1_dout),
    .dout_valid(mf1_valid)
);

// FW2 - Subharmonic chirp matched filter
// Coefficient ROM: fw2_coeffs.mem
matched_filter_1d_rom #(
    .DATA_WIDTH (DATA_WIDTH),
    .TAPS       (TAPS),
    .ACCUM_WIDTH(ACCUM_WIDTH),
    .ROW_LENGTH (TOTAL_IN),
    .ROM_FILE   ("fw2_coeffs.mem")
) mf_fw2 (
    .clk       (clk),
    .rst       (~rst_n),
    .din       (mf_din),
    .din_valid (mf_din_valid),
    .dout      (mf2_dout),
    .dout_valid(mf2_valid)
);

// ============================================================================
// Main FSM
// ============================================================================
reg [2:0]  state;
reg [6:0]  cur_ch;       // Current channel being filtered (0 to NUM_CH-1)
reg [10:0] pad_cnt;      // Zero-padding counter (0 to PAD_LEN-1)
reg [10:0] samp_cnt;     // Input sample counter per channel (0 to L-1)
reg [10:0] out_cnt;      // Output write counter per channel (0 to L-1)
reg [17:0] load_cnt;     // Total words loaded so far

// Write address for output BRAM: cur_ch * L + out_cnt
wire [17:0] out_wr_addr = (cur_ch * L[17:0]) + {7'b0, out_cnt};
// Read address for input BRAM: sample * NUM_CH + cur_ch (row-major input)
wire [17:0] in_rd_addr  = ({7'b0, samp_cnt} * NUM_CH[17:0]) + {11'b0, cur_ch};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state        <= S_IDLE;
        w1_done      <= 1'b0;
        w1_busy      <= 1'b0;
        cur_ch       <= 7'b0;
        pad_cnt      <= 11'b0;
        samp_cnt     <= 11'b0;
        out_cnt      <= 11'b0;
        in_wr_addr   <= 18'b0;
        load_cnt     <= 18'b0;
        mf_din       <= {DATA_WIDTH{1'b0}};
        mf_din_valid <= 1'b0;
    end else begin
        mf_din_valid <= 1'b0;   // Default: no new data
        w1_done      <= 1'b0;   // Default: pulse

        case (state)
            // ----------------------------------------------------------
            // IDLE: Wait for start pulse
            // ----------------------------------------------------------
            S_IDLE: begin
                w1_busy   <= 1'b0;
                if (start) begin
                    w1_busy    <= 1'b1;
                    in_wr_addr <= 18'b0;
                    load_cnt   <= 18'b0;
                    state      <= S_LOAD_RF;
                end
            end

            // ----------------------------------------------------------
            // LOAD_RF: Receive RF bytes, assemble 32-bit words, fill BRAM
            // RF is sent row-major: all 128 channels per axial sample
            // Total words = L × NUM_CH = 1792 × 128 = 229376
            // ----------------------------------------------------------
            S_LOAD_RF: begin
                if (word_valid) begin
                    // rf_in_bram written combinatorially by always block above
                    in_wr_addr <= in_wr_addr + 18'h1;
                    load_cnt   <= load_cnt   + 18'h1;
                    if (load_cnt == (L * NUM_CH) - 1) begin
                        // All RF samples loaded - begin filtering channel 0
                        cur_ch   <= 7'b0;
                        pad_cnt  <= 11'b0;
                        samp_cnt <= 11'b0;
                        out_cnt  <= 11'b0;
                        state    <= S_PAD_ZEROS;
                    end
                end
            end

            // ----------------------------------------------------------
            // PAD_ZEROS: Feed PAD_LEN = 135 zeros to prime FIR pipeline
            // Ensures matched filter output aligns to RF sample 0
            // MATLAB: FW2 = [zeros(1,68) ww2] handles this alignment
            // ----------------------------------------------------------
            S_PAD_ZEROS: begin
                mf_din       <= {DATA_WIDTH{1'b0}};
                mf_din_valid <= 1'b1;
                pad_cnt      <= pad_cnt + 11'h1;
                if (pad_cnt == PAD_LEN[10:0] - 1) begin
                    samp_cnt <= 11'b0;
                    out_cnt  <= 11'b0;
                    pad_cnt  <= 11'b0;
                    state    <= S_FILT_RUN;
                end
            end

            // ----------------------------------------------------------
            // FILT_RUN: Feed L RF samples for current channel
            // Reads from rf_in_bram (row-major) → drives both mf_fw1/fw2
            // ----------------------------------------------------------
            S_FILT_RUN: begin
                mf_din       <= rf_in_bram[in_rd_addr];
                mf_din_valid <= 1'b1;
                samp_cnt     <= samp_cnt + 11'h1;

                // Capture valid filter outputs (both filters driven same data,
                // so mf1_valid == mf2_valid in steady state)
                if (mf1_valid) begin
                    // Truncate 64-bit Q accumulator to 32-bit Q2.30:
                    // ACCUM holds Q(2+30).(30) product sum → shift right OUT_SHIFT
                    rf_out_bram_x1[out_wr_addr] <= mf1_dout[OUT_SHIFT +: DATA_WIDTH];
                    rf_out_bram_x2[out_wr_addr] <= mf2_dout[OUT_SHIFT +: DATA_WIDTH];
                    out_cnt <= out_cnt + 11'h1;
                end

                if (samp_cnt == L[10:0] - 1) begin
                    samp_cnt <= 11'b0;
                    state    <= S_FILT_FLUSH;
                end
            end

            // ----------------------------------------------------------
            // FILT_FLUSH: Feed zeros to drain pipeline until L outputs captured
            // Pipeline depth = TAPS stages, so need ~TAPS more cycles
            // ----------------------------------------------------------
            S_FILT_FLUSH: begin
                mf_din       <= {DATA_WIDTH{1'b0}};
                mf_din_valid <= 1'b1;

                if (mf1_valid && out_cnt < L[10:0]) begin
                    rf_out_bram_x1[out_wr_addr] <= mf1_dout[OUT_SHIFT +: DATA_WIDTH];
                    rf_out_bram_x2[out_wr_addr] <= mf2_dout[OUT_SHIFT +: DATA_WIDTH];
                    out_cnt <= out_cnt + 11'h1;
                end

                // All L outputs captured for this channel
                if (out_cnt == L[10:0] - 1)
                    state <= S_NEXT_CH;
            end

            // ----------------------------------------------------------
            // NEXT_CH: Advance to next channel or finish
            // ----------------------------------------------------------
            S_NEXT_CH: begin
                mf_din_valid <= 1'b0;  // Pause filter between channels
                if (cur_ch == NUM_CH[6:0] - 1) begin
                    state <= S_DONE;
                end else begin
                    cur_ch   <= cur_ch + 7'h1;
                    pad_cnt  <= 11'b0;
                    samp_cnt <= 11'b0;
                    out_cnt  <= 11'b0;
                    state    <= S_PAD_ZEROS; // Re-prime filter for next channel
                end
            end

            // ----------------------------------------------------------
            // DONE: Signal completion
            // ----------------------------------------------------------
            S_DONE: begin
                w1_done <= 1'b1;  // One-cycle pulse
                w1_busy <= 1'b0;
                state   <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule

// ============================================================================
// MATLAB ROM Generation Script Snippet (gen_fpga_roms.m):
// ============================================================================
// % -- Generate fw1_coeffs.mem and fw2_coeffs.mem --
// Q = 30;  scale = 2^Q;
// FW1 = [zeros(1,length(ww1)), ww1];   % 1×136
// FW2 = [zeros(1,length(ww2)), ww2];   % 1×136
// fid1 = fopen('fw1_coeffs.mem','w');
// fid2 = fopen('fw2_coeffs.mem','w');
// for k = 1:136
//     fprintf(fid1, '%08X\n', typecast(int32(round(FW1(k)*scale)),'uint32'));
//     fprintf(fid2, '%08X\n', typecast(int32(round(FW2(k)*scale)),'uint32'));
// end
// fclose(fid1); fclose(fid2);
// ============================================================================
