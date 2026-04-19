================================================================================
  FPGA Synthesis Guide — Volterra Ultrasound Pipeline
  volterra_top.v + W1/W2/W3 Wrappers
================================================================================

1. FILE LIST
------------
  Source Files (add to Vivado project in this order):
    CLZ.v                        - Count Leading Zeros (used by cordic_sqrt)
    Cordic_sqrt.v                - CORDIC square root (used by Delay_calc)
    Delay_calc.v                 - Per-element delay calculation
    Beamforming.sv               - Delay-and-sum beamforming core
    flip_matrix.v                - custom_flipud_ff sliding window buffer
    Quadraticproduct.v           - Volterra 2nd-order pair products
    Cubicproduct.v               - Volterra 3rd-order triple products
    Dotproduct.v                 - Sequential dot product engine
    matched_filter_1d_rom.v      - Patched matched filter (replaces convolution.v)
    interpolation_decimation.v   - Not used directly (coeffs precomputed into .mem)
    w1_matched_filter_wrapper.v  - W1 top
    w2_beamform_wrapper.v        - W2 top
    w3_volterra_wrapper.v        - W3 top
    volterra_top.v               - System top-level

  IMPORTANT: Replace matched_filter_1d instantiations in w1_matched_filter_wrapper.v
  with matched_filter_1d_rom. Update port names accordingly.


2. PRECOMPUTED ROM FILES (place in Vivado project source directory)
--------------------------------------------------------------------
  File                      Size      Description
  fw1_coeffs.mem            136 lines  FW1 matched filter taps (Q2.30 hex)
  fw2_coeffs.mem            136 lines  FW2 matched filter taps (Q2.30 hex)
  hh_grid.mem               973 lines  Axial pixel depths mm (Q6.26 hex)
  ww_grid.mem               128 lines  Lateral element positions mm (Q6.26 hex)
  element_positions_128.mem 128 lines  Copy of ww_grid.mem (for Beamforming.sv)
  volterra_h2.mem           120 lines  Scaled h2 kernel (Q2.30 hex)
  volterra_h3.mem           680 lines  Scaled h3 kernel (Q2.30 hex)
  volterra_d2_a.mem         120 lines  d{2} first  index (1 hex nibble per line)
  volterra_d2_b.mem         120 lines  d{2} second index
  volterra_d3_a.mem         680 lines  d{3} first  index
  volterra_d3_b.mem         680 lines  d{3} second index
  volterra_d3_c.mem         680 lines  d{3} third  index
  rf_ch_0.mem .. rf_ch_9.mem 10240 lines each  RF data for beamforming simulation

  Generate all .mem files by running gen_fpga_roms() (code at bottom of volterra_top.v).


3. BEAMFORMING.SV MODIFICATION REQUIRED FOR SYNTHESIS
-------------------------------------------------------
  Problem:
    beamforming.sv uses $readmemh to load rf data:
      initial $readmemh($sformatf("rf_ch_%0d.mem", i), local_rf_mem);
    This works for simulation but NOT for actual hardware data from W1.

  Solution — Add write port to beamforming.sv:
    In the module port list, add:
      input wire        rf_wr_en,
      input wire [13:0] rf_wr_addr,  // batch_id[2:0] * 2048 + sample[10:0]
      input wire [31:0] rf_wr_data,

    In the generate block (inside gen_cores for i=0..CALC_INST-1):
      always @(posedge clk) begin
        if (rf_wr_en && (rf_wr_addr[13:11] == (center_idx - 25 + batch_idx*CALC_INST + i) / 2048))
          local_rf_mem[rf_wr_addr[10:0]] <= rf_wr_data;
      end

    Then in w2_beamform_wrapper.v S_COPY_RF state, drive these ports from rf_buf.

  For SIMULATION ONLY: Keep $readmemh and ensure rf_ch_N.mem matches W1 output.


4. FIXED-POINT FORMATS
----------------------
  Module / Signal          Format    Notes
  RF data (raw)            Q1.31     After x / max(abs(x)) normalization in MATLAB
  matched filter taps      Q2.30     FW1/FW2 chirp templates
  matched filter output    Q4.60     64-bit accumulator; shift >> 28 → Q2.30
  pixel coordinates        Q6.26     hh, ww grids in mm (matches Delay_calc)
  Volterra kernels h2/h3   Q2.30     H_par_scaled.h2 / .h3 from V_tune
  Volterra products (quad) Q4.60     yy[a]*yy[b]; shift >> 28 → Q2.30
  Volterra products (cubic)Q6.90     yy[a]*yy[b]*yy[c]; shift >> 58 → Q2.30
  Dot product result        Q4+24    89-bit; bits [58:27] → Q2.30 output
  env22 / env23 output      Q2.30    abs(z2), abs(z3)


5. MATLAB PARAMETER MAPPING TABLE
-----------------------------------
  MATLAB Variable           FPGA Parameter / Signal       Value
  m = 15                    M_VOLTERRA                    15
  Neighbor_elements = 25    APERTURE_SIZE/2               25
  L = 1792                  L                             1792
  length(hh) = 973          NUM_AXIAL                     973
  length(ww) = 128          NUM_LATERAL                   128
  length(FW2) = 136         TAPS                          136
  l(2) = 120                NTERMS_QUAD                   120
  l(3) = 680                NTERMS_CUBIC                  680
  ts*c = 0.04928            reciprocal_ts_c = 3350        in Delay_calc.v
  start_depth ≈ 2.0265mm    START_DEPTH_Q = 136710701     Q6.26 value
  NORM_FACTOR = 20972        NORM_FACTOR                   in Beamforming.sv
  Fs ≈ 32 MHz               STEP_SIZE = 524288            in custom_interp1.v


6. VIVADO SYNTHESIS SETTINGS
-----------------------------
  Target: Xilinx UltraScale+ or 7-series (Artix-7/Kintex-7 recommended)
  Clock: 100 MHz (10ns period)
  Language: Verilog 2001 + SystemVerilog (for Beamforming.sv)

  XDC Constraints snippet:
    create_clock -period 10.000 -name clk [get_ports clk]
    set_input_delay  -clock clk 2.000 [get_ports {data_in data_valid start}]
    set_output_delay -clock clk 2.000 [get_ports {env22_out env23_out}]

  Resource estimates (rough):
    BRAMs : ~40 (RF BRAMs + output BRAMs)
    DSPs  : ~320 (136 FIR taps × 2 + 120 quad + 680 cubic products)
    LUTs  : ~15,000
    FFs   : ~8,000


7. 8-BIT INPUT PROTOCOL EXAMPLE (Verilog testbench snippet)
------------------------------------------------------------
  // Load one frame of RF data
  task load_rf_frame;
    input [31:0] rf_data [0:1791][0:127]; // [sample][channel]
    integer s, ch, b;
    begin
      @(posedge clk); start = 1; @(posedge clk); start = 0;
      // Stream bytes: all 128 channels for sample 0, then sample 1, ...
      for (s = 0; s < 1792; s = s+1) begin
        for (ch = 0; ch < 128; ch = ch+1) begin
          for (b = 3; b >= 0; b = b-1) begin  // MSB first
            @(posedge clk);
            data_in    = rf_data[s][ch][b*8 +: 8];
            data_valid = 1;
          end
        end
      end
      @(posedge clk); data_valid = 0;
    end
  endtask

  // Read result pixel
  task read_pixel;
    input [9:0] ax; input [6:0] lat;
    begin
      out_rd_addr = {ax, lat};
      out_rd_en   = 1;
      @(posedge clk); // 1 cycle read latency
      $display("env22[%0d][%0d] = %0d", ax, lat, env22_out);
      $display("env23[%0d][%0d] = %0d", ax, lat, env23_out);
    end
  endtask
================================================================================
