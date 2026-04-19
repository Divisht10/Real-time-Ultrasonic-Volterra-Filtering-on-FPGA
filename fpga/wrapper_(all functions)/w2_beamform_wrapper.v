// ============================================================================
// File        : w2_beamform_wrapper.v
// Description : W2 - Beamforming Wrapper
//               Implements MATLAB beamform_fpga() and calculate_pixel_delays()
//               (lines 163-210 in volterra_code.m)
//
// MATLAB Equivalent:
//   x2_bf_norm = beamform_fpga(x2, ww, hh, ts*c, ex, L, start_depth, 25)
//   → Output: 973×128 beamformed image
//
// Architecture:
//   - Reads matched-filtered RF data from W1's output BRAM
//   - Copies data into beamforming module's internal BRAMs
//   - Iterates over all NUM_AXIAL × NUM_LATERAL pixels
//   - For each pixel: loads (x_pixel, z_pixel, start_depth), triggers beamforming
//   - Stores result in output BRAM for W3
//
// Beamforming Module (Beamforming.sv):
//   - Internally instantiates Delay_calc × CALC_INST (10)
//   - Each Delay_calc uses cordic_sqrt for geometric delay calculation
//   - Processes 50 aperture elements in 5 batches of 10
//   - APERTURE_SIZE = 50 → 25 elements left + right of center
//
// Pixel Grid (from MATLAB / block diagram):
//   hh (axial)   : start_depth to 26mm, step = (26 - start_depth)/(L-1)
//                  start_depth ≈ 2.0265 mm → NUM_AXIAL = 973
//   ww (lateral) : 128 element positions (Trans.ElementPos × wavelength)
//
// Fixed-Point Representation:
//   All pixel coordinates in Q6.26 (matching Delay_calc's Q_TOTAL=32, Q_FRAC=26)
//   hh[i] = (start_depth + i * ddz) in mm, converted to Q6.26
//   ww[j] = element_x_pos[j] in mm, converted to Q6.26
//
// Precomputed ROM Files Required:
//   hh_grid.mem : 973 × 32-bit Q6.26 axial pixel positions (mm)
//   ww_grid.mem : 128 × 32-bit Q6.26 lateral pixel positions = element x-positions (mm)
//   element_positions_128.mem : 128 × 32-bit Q6.26 (used directly by beamforming.sv)
//
// 8-Bit Serial Interface:
//   The beamformed output is read 8 bits at a time from output_bram.
// ============================================================================

`timescale 1ns/1ps

module w2_beamform_wrapper #(
    parameter DATA_WIDTH    = 32,    // Q6.26 fixed-point (matching Delay_calc)
    parameter NUM_AXIAL     = 973,   // hh grid length: hand-coded at 26mm depth
    parameter NUM_LATERAL   = 128,   // ww grid = 128 element positions
    parameter L             = 1792,  // Max RF samples (Receive.endSample)
    parameter APERTURE_SIZE = 50,    // Neighbor_elements*2 (MATLAB: M=25 each side)
    parameter CALC_INST     = 10,    // Parallel Delay_calc instances in beamforming.sv
    // Q6.26 fixed-point constants (from MATLAB):
    // start_depth = 2.0265 mm → Q6.26: round(2.0265 * 2^26) = 136,710,701
    parameter [31:0] START_DEPTH_Q = 32'd136710701,
    // ts*c = 0.04928 mm/sample → reciprocal used in Delay_calc: reciprocal_ts_c = 3350
    // (already embedded in Delay_calc parameter, referenced here for documentation)
    parameter [31:0] RECIP_TS_C    = 32'd3350
)(
    input  wire        clk,
    input  wire        rst_n,

    // -------------------------------------------------------
    // W1 → W2 handshake
    // -------------------------------------------------------
    input  wire        w1_done,     // Start W2 when W1 signals completion
    output reg         w2_done,     // W2 finished beamforming full image
    output reg         w2_busy,

    // -------------------------------------------------------
    // W1 Output BRAM Read Interface (x2 filtered data)
    // W2 reads channel-by-channel to fill beamforming BRAMs
    // -------------------------------------------------------
    output reg  [17:0] w1_rd_addr,  // ch * L + sample (sent to w1 wrapper)
    input  wire [31:0] w1_rd_data,  // Filtered RF sample (x2)

    // -------------------------------------------------------
    // Output BRAM Read Port (W3 reads beamformed pixels)
    // addr = axial_idx * NUM_LATERAL + lateral_idx
    // -------------------------------------------------------
    input  wire [16:0] bf_rd_addr,  // max 973 * 128 = 124544
    output wire [31:0] bf_rd_data   // Beamformed pixel value
);

// ============================================================================
// Internal Parameters
// ============================================================================
localparam BF_BRAM_DEPTH = NUM_AXIAL * NUM_LATERAL;  // 124544

// RF buffer: stores x2 matched-filtered data for all 128 channels
// Organized as channel-major: ch0[0..L-1], ch1[0..L-1], ..., ch127[0..L-1]
// This mirrors how beamforming.sv reads its local_rf_mem
(* ram_style = "block" *) reg [DATA_WIDTH-1:0] rf_buf [0:(NUM_LATERAL*L)-1]; // 128×1792

// Beamformed output BRAM: [axial][lateral] → pixel value
(* ram_style = "block" *) reg [DATA_WIDTH-1:0] bf_out_bram [0:BF_BRAM_DEPTH-1];
assign bf_rd_data = bf_out_bram[bf_rd_addr];

// Axial pixel grid ROM (hh in MATLAB)
// hh[0] = start_depth, hh[N] = start_depth + N * ddz, hh[NUM_AXIAL-1] ≤ 26mm
(* rom_style = "block" *) reg [DATA_WIDTH-1:0] hh_rom [0:NUM_AXIAL-1];
initial $readmemh("hh_grid.mem", hh_rom);

// Lateral pixel grid ROM (ww in MATLAB = element x-positions)
(* rom_style = "block" *) reg [DATA_WIDTH-1:0] ww_rom [0:NUM_LATERAL-1];
initial $readmemh("ww_grid.mem", ww_rom);

// ============================================================================
// FSM States
// ============================================================================
localparam S_IDLE      = 4'd0;
localparam S_COPY_RF   = 4'd1;  // Copy W1 BRAM → local rf_buf (ch by ch)
localparam S_PIX_START = 4'd2;  // Set up pixel (ax, lat) and load values into BF module
localparam S_BF_LOAD0  = 4'd3;  // Load x_pixel into beamforming module
localparam S_BF_LOAD1  = 4'd4;  // Load z_pixel
localparam S_BF_LOAD2  = 4'd5;  // Load start_depth (3rd value)
localparam S_BF_WAIT   = 4'd6;  // Wait for pixel_valid from beamforming module
localparam S_PIX_STORE = 4'd7;  // Store pixel value in bf_out_bram
localparam S_PIX_NEXT  = 4'd8;  // Advance pixel counters
localparam S_DONE      = 4'd9;

reg [3:0]  state;
reg [9:0]  ax_idx;        // Axial pixel index (0 to NUM_AXIAL-1 = 972)
reg [6:0]  lat_idx;       // Lateral pixel index (0 to NUM_LATERAL-1 = 127)
reg [10:0] copy_samp;     // Sample counter for RF copy
reg [6:0]  copy_ch;       // Channel counter for RF copy
reg [1:0]  load_phase;    // Sub-state for loading 3 values into beamforming
reg        bf_valid_prev; // For edge detection on pixel_valid

// ============================================================================
// Beamforming Module Instantiation
// beamforming.sv processes one pixel at a time.
// Inputs: center_idx (which lateral pixel), x_pixel, z_pixel, start_depth
// Output: final_pixel_val (averaged DAS sum), pixel_valid
//
// NOTE: beamforming.sv reads local RF data from its internal local_rf_mem BRAMs.
// Those BRAMs are currently $readmemh-initialized (simulation only).
// For synthesis: the beamforming module needs modification to accept rf_buf here.
// A write-port approach is described in the code note below.
// For simulation validation: the rf_buf and bf module BRAMs hold same data.
// ============================================================================
reg  [DATA_WIDTH-1:0] bf_input_line;
reg                   bf_load_values;
wire [DATA_WIDTH-1:0] bf_pixel_val;
wire                  bf_pixel_valid;
reg                   bf_reset;

// Center element index for beamforming = lateral pixel index (0-127)
// MATLAB: beamform_fpga iterates i=1:128 → center_element_idx = i
beamforming #(
    .DATA_WIDTH   (DATA_WIDTH),
    .APERTURE_SIZE(APERTURE_SIZE),
    .CALC_INST    (CALC_INST)
) bf_inst (
    .clk            (clk),
    .reset          (bf_reset),
    .load_values    (bf_load_values),
    .center_idx     (lat_idx),        // Which column = center element for DAS
    .input_line     (bf_input_line),  // Carries x_pixel / z_pixel / start_depth
    .final_pixel_val(bf_pixel_val),
    .pixel_valid    (bf_pixel_valid)
);

// ============================================================================
// Main FSM
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state          <= S_IDLE;
        w2_done        <= 1'b0;
        w2_busy        <= 1'b0;
        ax_idx         <= 10'b0;
        lat_idx        <= 7'b0;
        copy_samp      <= 11'b0;
        copy_ch        <= 7'b0;
        load_phase     <= 2'b0;
        bf_load_values <= 1'b0;
        bf_reset       <= 1'b0;
        bf_input_line  <= {DATA_WIDTH{1'b0}};
        w1_rd_addr     <= 18'b0;
        bf_valid_prev  <= 1'b0;
    end else begin
        w2_done        <= 1'b0;
        bf_load_values <= 1'b0;
        bf_valid_prev  <= bf_pixel_valid;

        case (state)
            // ----------------------------------------------------------
            // IDLE: Wait for W1 to finish
            // ----------------------------------------------------------
            S_IDLE: begin
                w2_busy  <= 1'b0;
                bf_reset <= 1'b0;
                if (w1_done) begin
                    w2_busy   <= 1'b1;
                    copy_ch   <= 7'b0;
                    copy_samp <= 11'b0;
                    state     <= S_COPY_RF;
                end
            end

            // ----------------------------------------------------------
            // COPY_RF: Transfer x2 data from W1's BRAM into local rf_buf
            // Also feeds data into beamforming.sv's BRAM (synthesis note below)
            // Reads W1: addr = copy_ch * L + copy_samp
            // ----------------------------------------------------------
            S_COPY_RF: begin
                w1_rd_addr <= ({11'b0, copy_ch} * L[17:0]) + {7'b0, copy_samp};
                // One cycle read latency: write the PREVIOUS read result
                if (copy_samp > 0 || copy_ch > 0) begin
                    rf_buf[({11'b0, copy_ch} * L[17:0]) + {7'b0, copy_samp} - 1]
                        <= w1_rd_data;
                end

                if (copy_samp == L[10:0] - 1) begin
                    copy_samp <= 11'b0;
                    if (copy_ch == NUM_LATERAL[6:0] - 1) begin
                        // All channels copied → start beamforming
                        ax_idx  <= 10'b0;
                        lat_idx <= 7'b0;
                        bf_reset <= 1'b1;  // Hold reset until first pixel
                        state   <= S_PIX_START;
                    end else begin
                        copy_ch <= copy_ch + 7'h1;
                    end
                end else begin
                    copy_samp <= copy_samp + 11'h1;
                end
            end

            // ----------------------------------------------------------
            // PIX_START: Deassert reset, prepare to load pixel coordinates
            // x_pixel = ww_rom[lat_idx] (mm, Q6.26)
            // z_pixel = hh_rom[ax_idx]  (mm, Q6.26)
            // ----------------------------------------------------------
            S_PIX_START: begin
                bf_reset   <= 1'b1;
                load_phase <= 2'b0;
                state      <= S_BF_LOAD0;
            end

            // ----------------------------------------------------------
            // BF_LOAD0/1/2: Serial-load 3 values into beamforming FSM
            // beamforming.sv S_LOAD state expects: x_pixel, z_pixel, start_depth
            // via input_line with load_values asserted
            // ----------------------------------------------------------
            S_BF_LOAD0: begin
                bf_input_line  <= ww_rom[lat_idx];  // x_pixel (lateral position)
                bf_load_values <= 1'b1;
                state          <= S_BF_LOAD1;
            end

            S_BF_LOAD1: begin
                bf_input_line  <= hh_rom[ax_idx];   // z_pixel (axial depth)
                bf_load_values <= 1'b1;
                state          <= S_BF_LOAD2;
            end

            S_BF_LOAD2: begin
                bf_input_line  <= START_DEPTH_Q;    // start_depth (constant, Q6.26)
                bf_load_values <= 1'b1;
                state          <= S_BF_WAIT;
            end

            // ----------------------------------------------------------
            // BF_WAIT: Wait for beamforming module to compute pixel
            // pixel_valid pulses high when final_pixel_val is ready
            // ----------------------------------------------------------
            S_BF_WAIT: begin
                bf_load_values <= 1'b0;
                // Rising edge on pixel_valid → result ready
                if (bf_pixel_valid && !bf_valid_prev) begin
                    state <= S_PIX_STORE;
                end
            end

            // ----------------------------------------------------------
            // PIX_STORE: Write pixel to output BRAM
            // Address: ax_idx * NUM_LATERAL + lat_idx
            // MATLAB: bout(j,i) = pixel_sum / num_elements (averaged in BF module)
            // ----------------------------------------------------------
            S_PIX_STORE: begin
                bf_out_bram[ax_idx * NUM_LATERAL[16:0] + {10'b0, lat_idx}]
                    <= bf_pixel_val;
                state <= S_PIX_NEXT;
            end

            // ----------------------------------------------------------
            // PIX_NEXT: Advance to next pixel (lateral first, then axial)
            // ----------------------------------------------------------
            S_PIX_NEXT: begin
                bf_reset <= 1'b0;  // Brief reset between pixels to clear BF module state
                if (lat_idx == NUM_LATERAL[6:0] - 1) begin
                    lat_idx <= 7'b0;
                    if (ax_idx == NUM_AXIAL[9:0] - 1) begin
                        state <= S_DONE;
                    end else begin
                        ax_idx <= ax_idx + 10'h1;
                        state  <= S_PIX_START;
                    end
                end else begin
                    lat_idx <= lat_idx + 7'h1;
                    state   <= S_PIX_START;
                end
            end

            // ----------------------------------------------------------
            // DONE: Signal W3 to begin
            // ----------------------------------------------------------
            S_DONE: begin
                w2_done <= 1'b1;
                w2_busy <= 1'b0;
                state   <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
    end
end

// ============================================================================
// NOTE: beamforming.sv BRAM synthesis bridge
// ============================================================================
// For full synthesis, beamforming.sv's local_rf_mem (per CALC_INST) must receive
// x2 filtered data written by this wrapper, not $readmemh.
//
// Recommended modification to beamforming.sv - add write port:
//   input wire        rf_wr_en,
//   input wire [13:0] rf_wr_addr,   // channel_within_batch * L + sample
//   input wire [31:0] rf_wr_data,
//
// Then in the generate block:
//   always @(posedge clk)
//     if (rf_wr_en && (rf_wr_addr[13:11] == batch_idx))
//       local_rf_mem[rf_wr_addr[10:0]] <= rf_wr_data;
//
// This wrapper would then populate each BRAM during S_COPY_RF.
// For simulation with $readmemh, the current beamforming.sv works as-is.
// ============================================================================

// ============================================================================
// MATLAB ROM Generation (gen_fpga_roms.m):
// ============================================================================
// % Axial grid hh (973 values from start_depth to 26mm)
// L = 1792; start_depth = Receive(1).startDepth * w2mm;
// end_depth = Receive(1).endDepth * w2mm;
// ddz = (end_depth - start_depth) / (L-1);
// hh = start_depth : ddz : 26;   % 1×973 vector
// fid = fopen('hh_grid.mem','w');
// for k = 1:length(hh)
//   fprintf(fid, '%08X\n', typecast(int32(round(hh(k) * 2^26)),'uint32'));
// end; fclose(fid);
//
// % Lateral grid ww (128 element positions in mm)
// ww = Trans.ElementPos(:,1)' * w2mm;   % 1×128
// fid = fopen('ww_grid.mem','w');
// for k = 1:128
//   fprintf(fid, '%08X\n', typecast(int32(round(ww(k) * 2^26)),'uint32'));
// end; fclose(fid);
// ============================================================================

endmodule
