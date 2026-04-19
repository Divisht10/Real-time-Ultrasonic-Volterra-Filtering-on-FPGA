// ============================================================================
// File        : volterra_top.v
// Description : Top-Level FPGA Wrapper — Ultrasound Volterra Beamforming
//               Integrates W1 (Matched Filter), W2 (Beamforming), W3 (Volterra)
//
// Maps completely to MATLAB: volterra_code_with_custom.m
//
// ─── Block Architecture ────────────────────────────────────────────────────
//
//   8-bit serial input (RF data, MSB first, 4 bytes per 32-bit sample)
//         │
//   ┌─────▼───────────────────────────────────────────────────────┐
//   │  W1: w1_matched_filter_wrapper                              │
//   │  • Assembles 32-bit RF words from 8-bit serial bus          │
//   │  • Stores 1792×128 normalized RF in input BRAM              │
//   │  • matched_filter_1d ×2 (FW1 fundamental, FW2 subharmonic) │
//   │  • Precomputed: fw1_coeffs.mem, fw2_coeffs.mem (136 taps)  │
//   │  • Output: x1[128][1792], x2[128][1792] in BRAMs            │
//   └─────────────────────┬───────────────────────────────────────┘
//                         │ w1_done
//   ┌─────────────────────▼───────────────────────────────────────┐
//   │  W2: w2_beamform_wrapper                                    │
//   │  • Copies x2 RF buffer into beamforming module BRAMs        │
//   │  • Iterates 973 axial × 128 lateral pixels                  │
//   │  • beamforming.sv → Delay_calc × 10 → cordic_sqrt           │
//   │  • Precomputed: hh_grid.mem (973), ww_grid.mem (128)        │
//   │  • Output: bf_out[973][128] in BRAM                         │
//   └─────────────────────┬───────────────────────────────────────┘
//                         │ w2_done
//   ┌─────────────────────▼───────────────────────────────────────┐
//   │  W3: w3_volterra_wrapper                                    │
//   │  • custom_flipud_ff sliding window (M=15 rows)              │
//   │  • volterra_quad  → 120 pair-products                       │
//   │  • dot_product_seq → z2 = prod2' * h2 (N=120)              │
//   │  • volterra_cubic → 680 triple-products                     │
//   │  • dot_product_seq → z3 = prod3' * h3 (N=680)              │
//   │  • Output: env22[973][128], env23[973][128] (|z2|, |z3|)   │
//   └─────────────────────┬───────────────────────────────────────┘
//                         │ w3_done
//   8-bit serial output read back (env22 / env23 pixels)
//
// ─── Signal Naming vs. MATLAB ──────────────────────────────────────────────
//
//  FPGA Signal          MATLAB Variable       Notes
//  ───────────────────────────────────────────────────────────────────────
//  data_in[7:0]         RData (raw)           4 bytes per Q2.30 sample
//  w1 x1 BRAM          x1 (1927×128)         After FW1 matched filter
//  w1 x2 BRAM          x2 (1927×128)         After FW2 matched filter
//  w2 bf_out BRAM      x2_bf_norm (973×128)  After delay-and-sum beamforming
//  w3 env22_bram       env22 (973×128)       Quadratic Volterra envelope
//  w3 env23_bram       env23 (973×128)       Cubic Volterra envelope
//  norm_factor         max(abs(x(:)))        Applied externally before data_in
//  H_par_scaled.h2/h3  Precomputed & stored  volterra_h2.mem, volterra_h3.mem
//
// ─── Precomputed ROM Files (generate with gen_fpga_roms.m) ─────────────────
//  fw1_coeffs.mem           136 × Q2.30 hex  FW1 matched filter taps
//  fw2_coeffs.mem           136 × Q2.30 hex  FW2 matched filter taps
//  hh_grid.mem              973 × Q6.26 hex  Axial pixel depths (mm)
//  ww_grid.mem              128 × Q6.26 hex  Lateral positions = element x-pos (mm)
//  element_positions_128.mem 128 × Q6.26 hex  Same as ww_grid (for beamforming.sv)
//  volterra_h2.mem          120 × Q2.30 hex  Scaled quadratic kernel
//  volterra_h3.mem          680 × Q2.30 hex  Scaled cubic kernel
//  volterra_d2_a/b.mem      120 × 4-bit hex  d{2} index pairs
//  volterra_d3_a/b/c.mem    680 × 4-bit hex  d{3} index triples
//  rf_ch_N.mem (×10)        2048 × 32-bit    RF data per beamforming batch (sim only)
//
// ─── 8-bit Input Protocol ──────────────────────────────────────────────────
//  Phase 1 — RF Data Load:
//    Assert start=1 for one cycle.
//    Stream 1792×128×4 = 917,504 bytes of normalized RF data (MSB first per word).
//    Data sent row-major: all 128 channels for sample 0, then sample 1, etc.
//    Assert data_valid=1 each byte.
//
//  Phase 2 — Pipeline runs autonomously (W1 → W2 → W3).
//    Monitor w3_done for completion.
//
//  Phase 3 — Read Results:
//    Assert out_rd_en=1, set out_rd_addr[16:0] = ax*128 + lat.
//    env22_out and env23_out valid on next cycle.
//
// ─── Timing Estimates (at 100 MHz) ─────────────────────────────────────────
//  W1: ~1792×128 = 229,376 filter cycles ≈ 2.3 ms
//  W2: ~973×128 × ~50 cycles per pixel ≈ 6.2 ms
//  W3: ~958×128 × ~800 cycles (120 + 680 dot product) ≈ 98 ms
//  Total ≈ 107 ms per frame (vs MATLAB: much longer on CPU)
// ============================================================================

`timescale 1ns/1ps

module volterra_top #(
    parameter DATA_WIDTH   = 32,
    parameter NUM_CH       = 128,
    parameter L            = 1792,
    parameter NUM_AXIAL    = 973,
    parameter NUM_LATERAL  = 128,
    parameter TAPS         = 136,
    parameter M_VOLTERRA   = 15,
    parameter NTERMS_QUAD  = 120,
    parameter NTERMS_CUBIC = 680,
    parameter ACCUM_WIDTH  = 64
)(
    input  wire        clk,
    input  wire        rst_n,         // Active-low global reset

    // -------------------------------------------------------
    // 8-bit RF Data Input
    // -------------------------------------------------------
    input  wire [7:0]  data_in,       // Serial RF byte (MSB of 32-bit word first)
    input  wire        data_valid,    // Byte valid strobe
    input  wire        start,         // Pulse high to begin processing a new frame

    // -------------------------------------------------------
    // Pipeline Status
    // -------------------------------------------------------
    output wire        pipeline_busy, // High during entire pipeline execution
    output wire        w1_done,       // Matched filtering complete
    output wire        w2_done,       // Beamforming complete
    output wire        w3_done,       // Volterra filtering complete (results ready)

    // -------------------------------------------------------
    // 32-bit Result Readback Port
    // Address: ax_idx * 128 + lat_idx (max 973*128-1 = 124543)
    // env22_out: quadratic Volterra envelope  (MATLAB: env22)
    // env23_out: cubic     Volterra envelope  (MATLAB: env23)
    // -------------------------------------------------------
    input  wire        out_rd_en,
    input  wire [16:0] out_rd_addr,   // {ax_idx[9:0], lat_idx[6:0]}
    output wire [31:0] env22_out,
    output wire [31:0] env23_out,

    // -------------------------------------------------------
    // Optional: Read fundamental/subharmonic envelopes from W1
    // (x1 and x2 after matched filtering, before beamforming)
    // addr = ch[6:0] * 1792 + sample[10:0]
    // -------------------------------------------------------
    input  wire [17:0] mf_rd_addr,
    output wire [31:0] mf_x1_out,
    output wire [31:0] mf_x2_out
);

// ============================================================================
// Internal Wires
// ============================================================================

// W1 → W2 data path
wire [17:0] w1_rd_addr_from_w2; // W2 requests specific address from W1's output BRAM
wire [31:0] w1_rd_data_x2;      // x2 filtered sample returned to W2
wire [31:0] w1_rd_data_x1;      // x1 filtered sample (for optional readback)

// W2 → W3 data path
wire [16:0] bf_rd_addr_from_w3; // W3 requests specific beamformed pixel
wire [31:0] bf_rd_data_to_w3;   // Beamformed pixel value returned to W3

// Internal done signals (some re-exported as outputs)
wire w1_done_int, w2_done_int, w3_done_int;
assign w1_done = w1_done_int;
assign w2_done = w2_done_int;
assign w3_done = w3_done_int;

// Busy signals
wire w1_busy, w2_busy, w3_busy;
assign pipeline_busy = w1_busy | w2_busy | w3_busy;

// ============================================================================
// W1: Matched Filter Wrapper
// MATLAB: x1_conv = custom_conv(x_norm', FW1)  [lines 95-98]
//         x2_conv = custom_conv(x_norm', FW2)  [lines 102-103]
// Interpolation (custom_interp1) is precomputed into fw1/fw2_coeffs.mem
// ============================================================================
w1_matched_filter_wrapper #(
    .DATA_WIDTH (DATA_WIDTH),
    .ACCUM_WIDTH(ACCUM_WIDTH),
    .TAPS       (TAPS),
    .L          (L),
    .PAD_LEN    (TAPS - 1),           // 135
    .TOTAL_IN   (L + TAPS - 1),       // 1927
    .NUM_CH     (NUM_CH)
) w1_inst (
    .clk         (clk),
    .rst_n       (rst_n),
    .data_in     (data_in),
    .data_valid  (data_valid),
    .start       (start),
    .w1_done     (w1_done_int),
    .w1_busy     (w1_busy),
    // Output BRAM port (read by W2 and external readback)
    .rd_addr     (w2_done_int ? mf_rd_addr : w1_rd_addr_from_w2),
    .rd_data_x1  (w1_rd_data_x1),
    .rd_data_x2  (w1_rd_data_x2)
);

// External readback mux for fundamental/subharmonic matched filter output
assign mf_x1_out = w1_rd_data_x1;
assign mf_x2_out = w1_rd_data_x2;

// ============================================================================
// W2: Beamforming Wrapper
// MATLAB: x2_bf_norm = beamform_fpga(x2, ww, hh, ts*c, ex, L, start_depth, M)
//         [line 109-110, beamform_fpga function lines 163-193]
// Uses beamforming.sv → Delay_calc.v → cordic_sqrt.v (CLZ.v)
// ============================================================================
w2_beamform_wrapper #(
    .DATA_WIDTH    (DATA_WIDTH),
    .NUM_AXIAL     (NUM_AXIAL),
    .NUM_LATERAL   (NUM_LATERAL),
    .L             (L),
    .APERTURE_SIZE (50),              // 2 × MATLAB's M=25 neighbor elements
    .CALC_INST     (10)               // Parallel delay calculators
) w2_inst (
    .clk         (clk),
    .rst_n       (rst_n),
    .w1_done     (w1_done_int),
    .w2_done     (w2_done_int),
    .w2_busy     (w2_busy),
    // Read W1's x2 output BRAM
    .w1_rd_addr  (w1_rd_addr_from_w2),
    .w1_rd_data  (w1_rd_data_x2),
    // Output port (W3 reads beamformed image)
    .bf_rd_addr  (bf_rd_addr_from_w3),
    .bf_rd_data  (bf_rd_data_to_w3)
);

// ============================================================================
// W3: Volterra Filter Wrapper
// MATLAB: z2_norm = V_out(x2_bf_norm, H_par_scaled)   [lines 124-131]
//         env22   = abs(z2_norm(:,:,2))               [line 133]
//         env23   = abs(z2_norm(:,:,3))               [line 137]
// Uses: flip_matrix.v, Quadraticproduct.v, Cubicproduct.v, Dotproduct.v
// ============================================================================
w3_volterra_wrapper #(
    .DATA_WIDTH   (DATA_WIDTH),
    .M            (M_VOLTERRA),
    .NTERMS_QUAD  (NTERMS_QUAD),
    .NTERMS_CUBIC (NTERMS_CUBIC),
    .NUM_AXIAL    (NUM_AXIAL),
    .NUM_LATERAL  (NUM_LATERAL)
) w3_inst (
    .clk         (clk),
    .rst_n       (rst_n),
    .w2_done     (w2_done_int),
    .w3_done     (w3_done_int),
    .w3_busy     (w3_busy),
    // Read W2's beamformed output BRAM
    .bf_rd_addr  (bf_rd_addr_from_w3),
    .bf_rd_data  (bf_rd_data_to_w3),
    // Output readback
    .out_rd_en   (out_rd_en),
    .out_rd_addr (out_rd_addr),
    .env22_out   (env22_out),
    .env23_out   (env23_out)
);

endmodule


// ============================================================================
// ============================================================================
//  COMPLETE MATLAB ROM GENERATION SCRIPT: gen_fpga_roms.m
//  Run this after the main volterra_code script to produce all .mem files.
// ============================================================================
// ============================================================================
//
// % ======================================================================
// % gen_fpga_roms.m — Generates all precomputed ROM .mem files for FPGA
// % Run AFTER volterra_code_with_custom.m has loaded Trans, Receive, etc.
// % ======================================================================
// function gen_fpga_roms()
//
// Q26 = 2^26; Q30 = 2^30;
// to_hex32 = @(x) dec2hex(typecast(int32(round(x)), 'uint32'), 8);
// to_hex4  = @(x) dec2hex(uint8(x), 1);
//
// %% ---- 1. Chirp coefficients: FW1 and FW2 ----
// % These replace custom_interp1 + normalization on FPGA
// % tw1/tw2 already loaded; ww1/ww2 computed from interpolation
//
// tFs = 250e6; Fs_fpga = 32e6;  % Adjust Fs_fpga to match Trans.frequency*4
// t_orig1 = (1:length(tw1))/tFs;
// t_new1  = (1/Fs_fpga):(1/Fs_fpga):(length(tw1)/tFs);
// ww1 = interp1(t_orig1, tw1, t_new1, 'linear', 0)'; ww1 = ww1/max(ww1);
// FW1 = [zeros(length(ww1),1); ww1];  % 136×1 (68 zeros + 68 chirp)
//
// t_orig2 = (1:length(tw2))/tFs;
// t_new2  = (1/Fs_fpga):(1/Fs_fpga):(length(tw2)/tFs);
// ww2 = interp1(t_orig2, tw2, t_new2, 'linear', 0)'; ww2 = ww2/max(ww2);
// FW2 = [zeros(length(ww2),1); ww2];  % 136×1
//
// fid = fopen('fw1_coeffs.mem','w');
// for k=1:length(FW1), fprintf(fid,'%s\n',to_hex32(FW1(k)*Q30)); end; fclose(fid);
// fid = fopen('fw2_coeffs.mem','w');
// for k=1:length(FW2), fprintf(fid,'%s\n',to_hex32(FW2(k)*Q30)); end; fclose(fid);
// fprintf('Written: fw1_coeffs.mem, fw2_coeffs.mem (%d taps each)\n', length(FW1));
//
// %% ---- 2. Pixel grids: hh and ww ----
// c = 1540; w2mm = 1e-3*c/Trans.frequency;
// L_val = Receive(1).endSample;
// start_d = Receive(1).startDepth * w2mm;
// end_d   = Receive(1).endDepth   * w2mm;
// ddz = (end_d - start_d) / (L_val - 1);
// hh  = start_d : ddz : 26;   % axial grid (mm) - 973 values
// ww  = Trans.ElementPos(:,1)' * w2mm;  % lateral grid (mm) - 128 values
//
// fid = fopen('hh_grid.mem','w');
// for k=1:length(hh), fprintf(fid,'%s\n',to_hex32(hh(k)*Q26)); end; fclose(fid);
// fid = fopen('ww_grid.mem','w');
// for k=1:length(ww), fprintf(fid,'%s\n',to_hex32(ww(k)*Q26)); end; fclose(fid);
// % element_positions_128.mem is identical to ww_grid.mem (for beamforming.sv)
// copyfile('ww_grid.mem','element_positions_128.mem');
// fprintf('Written: hh_grid.mem (%d pts), ww_grid.mem (128 pts)\n', length(hh));
//
// %% ---- 3. Volterra kernels h2, h3 ----
// % Run V_tune on representative ROI data first (e.g., from first frame)
// % Requires: roi_data_orig, reborn already computed in main script
// H_par      = V_tune(roi_data_orig, reborn);
// H_max      = max([max(abs(H_par.h2)), max(abs(H_par.h3))]);
// h2_scaled  = H_par.h2 / H_max;   % 120×1, in [-1, 1]
// h3_scaled  = H_par.h3 / H_max;   % 680×1, in [-1, 1]
//
// fid = fopen('volterra_h2.mem','w');
// for k=1:120, fprintf(fid,'%s\n',to_hex32(h2_scaled(k)*Q30)); end; fclose(fid);
// fid = fopen('volterra_h3.mem','w');
// for k=1:680, fprintf(fid,'%s\n',to_hex32(h3_scaled(k)*Q30)); end; fclose(fid);
// fprintf('Written: volterra_h2.mem (120), volterra_h3.mem (680)\n');
//
// %% ---- 4. Volterra index pairs/triples ----
// reborn = Volterra_begins(15);
// d2 = reborn.d{2};   % 120×2 (1-based MATLAB indices)
// d3 = reborn.d{3};   % 680×3
//
// fid_a = fopen('volterra_d2_a.mem','w'); fid_b = fopen('volterra_d2_b.mem','w');
// for k=1:120
//   fprintf(fid_a,'%01X\n', d2(k,1)-1);   % Convert to 0-based
//   fprintf(fid_b,'%01X\n', d2(k,2)-1);
// end; fclose(fid_a); fclose(fid_b);
//
// fid_a=fopen('volterra_d3_a.mem','w'); fid_b=fopen('volterra_d3_b.mem','w');
// fid_c=fopen('volterra_d3_c.mem','w');
// for k=1:680
//   fprintf(fid_a,'%01X\n', d3(k,1)-1);
//   fprintf(fid_b,'%01X\n', d3(k,2)-1);
//   fprintf(fid_c,'%01X\n', d3(k,3)-1);
// end; fclose(fid_a); fclose(fid_b); fclose(fid_c);
// fprintf('Written: volterra_d2_a/b.mem, volterra_d3_a/b/c.mem\n');
//
// %% ---- 5. RF channel data for simulation (beamforming.sv $readmemh) ----
// % Write matched-filtered x2 data for first frame into rf_ch_N.mem files
// % (Only needed for RTL simulation; synthesis uses W1 BRAM directly)
// % Organize as batches of 10 channels:
// % rf_ch_0.mem through rf_ch_9.mem, each 10240 deep (5 batches × 2048)
// x2_bf_input = x2_orig;   % 1927×128 matched-filtered subharmonic data
// norm_f = max(abs(RData(:)));
// x2_norm_full = x2_bf_input / norm_f;
// for ch_inst = 0:9
//   fid = fopen(sprintf('rf_ch_%d.mem', ch_inst), 'w');
//   for batch = 0:4
//     ch_base = ch_inst + batch*10;  % Which actual channel for this slot
//     if ch_base < 128
//       col_data = x2_norm_full(1:2048, ch_base+1);
//     else
//       col_data = zeros(2048,1);
//     end
//     for s=1:2048
//       fprintf(fid,'%s\n',to_hex32(col_data(s)*Q30));
//     end
//   end
//   fclose(fid);
// end
// fprintf('Written: rf_ch_0.mem through rf_ch_9.mem\n');
//
// fprintf('\n=== All ROM files generated. Copy to Vivado project source dir. ===\n');
// end  % gen_fpga_roms
