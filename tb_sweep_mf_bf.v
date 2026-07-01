// ============================================================================
// tb_mf_beamform_sweep.v  -  full-grid sweep testbench for mf_beamform_top.
//
//   Drives the complete W1 (matched filter) -> bank_a -> W2 (DAS beamformer)
//   pipeline over the pixel grid defined by the uploaded coordinate files and
//   dumps every beamformed pixel to a text file for off-line viewing.
//
//   MEM FILES (must be on the xsim run path):
//     hh.mem            : AXIAL   axial   pixel coords (Q6.26 mm)   [uploaded: 10]
//     ww.mem            : LATERAL lateral pixel coords (Q6.26 mm)   [uploaded: 128]
//     fw2_coeffs.mem    : TAPS    matched-filter taps (Q2.30)       [uploaded: 139]
//     element_positions.mem : 128 element x-positions (Q6.26 mm)
//                             - loaded by beamforming_synth, NOT optional with
//                               the real Delay_calc.  Provide your array geometry.
//
//   IMPORTANT PARAMETER NOTES
//   -------------------------
//   * AXIAL = 10 to match the uploaded hh.mem (10 lines), NOT 973.  For a true
//     973-row sweep, replace hh.mem with 973 entries and set AXIAL = 973 (that
//     run is ~70x longer - see runtime note at the bottom of this file).
//   * TAPS = 139 / PAD_LEN = 138 to match the uploaded 139-tap coeff files.
//   * START_DEPTH_Q626 = 2.0 mm.  hh.mem depths are 2.0..2.9 mm, so start_depth
//     MUST be <= the shallowest pixel or sum_dist (= z + dd_n - 2*start_depth)
//     goes negative and every rf_addr underflows.  SET THIS to your real
//     Receive(1).startDepth * w2mm.
//   * rf_din is a CONSTANT placeholder (0x40000000).  This validates the sweep
//     mechanics and that Delay_calc produces sane in-range addresses across the
//     whole grid; it is NOT real echo data, so the pixels are not a real B-mode.
//     To get a real image, stream actual RF samples in MF_RUN instead.
// ============================================================================
`timescale 1ns/1ps

module tb_mf_beamform_sweep;

    // ---- full-size parameters (match the uploaded mem files) ----
    localparam DATA_WIDTH       = 32;
    localparam ACCUM_WIDTH      = 64;
    localparam TAPS             = 139;          // fw2_coeffs.mem line count
    localparam PAD_LEN          = TAPS-1;       // 138
    localparam OUT_SHIFT        = 28;
    localparam L                = 1792;         // RF samples per channel
    localparam NUM_CH           = 128;
    localparam AXIAL            = 10;           // hh.mem line count (NOT 973)
    localparam LATERAL          = 128;          // ww.mem line count
    localparam SAMP_STRIDE_LOG2 = 11;
    localparam ADDR_W           = 18;
    localparam PIX_LAT_W        = 7;            // ceil(log2(LATERAL=128))
    localparam PIX_AX_W         = 4;            // ceil(log2(AXIAL=10))

    // start_depth: SET TO YOUR REAL VALUE.  2.0 mm = 0x08000000 (Q6.26).
    localparam signed [DATA_WIDTH-1:0] START_DEPTH_Q626 = 32'sh08000000;

    // RF placeholder (constant).  Replace with a real RF stream for a true image.
    localparam [DATA_WIDTH-1:0] DIN_CONST = 32'h40000000;

    localparam integer NPIX = AXIAL*LATERAL;    // 1280

    reg clk = 0, rst_n = 0, start = 0, enable_sweep = 1;
    reg signed [DATA_WIDTH-1:0] rf_din = 0;
    reg rf_din_valid = 0;

    wire rf_ready;
    wire signed [DATA_WIDTH-1:0] pixel_out;
    wire pixel_out_valid;
    wire [PIX_LAT_W-1:0] pixel_lat_idx;
    wire [PIX_AX_W-1:0]  pixel_ax_idx;
    wire mf_done, done;

    // ---- DUT ----
    mf_beamform_top #(
        .DATA_WIDTH(DATA_WIDTH), .ACCUM_WIDTH(ACCUM_WIDTH),
        .TAPS(TAPS), .PAD_LEN(PAD_LEN), .OUT_SHIFT(OUT_SHIFT),
        .L(L), .NUM_CH(NUM_CH), .AXIAL(AXIAL), .LATERAL(LATERAL),
        .SAMP_STRIDE_LOG2(SAMP_STRIDE_LOG2), .ADDR_W(ADDR_W),
        .PIX_LAT_W(PIX_LAT_W), .PIX_AX_W(PIX_AX_W),
        .START_DEPTH_Q626(START_DEPTH_Q626),
        .HH_FILE("hh.mem"), .WW_FILE("ww.mem"), .FW2_FILE("fw2_coeffs.mem")
    ) dut (
        .clk(clk), .rst_n(rst_n), .start(start), .enable_sweep(enable_sweep),
        .rf_din(rf_din), .rf_din_valid(rf_din_valid), .rf_ready(rf_ready),
        .pixel_out(pixel_out), .pixel_out_valid(pixel_out_valid),
        .pixel_lat_idx(pixel_lat_idx), .pixel_ax_idx(pixel_ax_idx),
        .mf_done(mf_done), .done(done)
    );

    // ---- clock : 10 ns ----
    always #5 clk = ~clk;

    // ---- pixel capture + dump ----
    integer pix [0:NPIX-1];
    integer pix_seen;
    integer pmin, pmax;
    integer anyx;
    integer fout;
    integer i;

    always @(posedge clk) begin
        if (rst_n && pixel_out_valid) begin
            pix[pixel_lat_idx*AXIAL + pixel_ax_idx] = pixel_out;
            pix_seen = pix_seen + 1;
            if (^pixel_out === 1'bx) anyx = anyx + 1;
            else begin
                if ($signed(pixel_out) < pmin) pmin = $signed(pixel_out);
                if ($signed(pixel_out) > pmax) pmax = $signed(pixel_out);
            end
            // one line per pixel: ax_idx  lat_idx  signed_value  hex
            $fwrite(fout, "%0d %0d %0d %08h\n",
                    pixel_ax_idx, pixel_lat_idx, $signed(pixel_out), pixel_out);
        end
    end

    integer watchdog;

    // 'done' is a 1-cycle pulse (T_DONE asserts it then returns to idle), so
    // latch it rather than polling the level after the fact.
    reg done_seen = 1'b0;
    always @(posedge clk) if (rst_n && done) done_seen <= 1'b1;

    initial begin
        pix_seen = 0; anyx = 0; pmin = 2147483647; pmax = -2147483648;
        for (i=0;i<NPIX;i=i+1) pix[i] = -1;

        fout = $fopen("sweep_pixels.txt", "w");
        $fwrite(fout, "# ax_idx lat_idx value(signed) value(hex)  grid=%0dx%0d (AXIAL x LATERAL)\n",
                AXIAL, LATERAL);

        // reset
        rst_n = 0; repeat (5) @(posedge clk);
        rst_n = 1; @(posedge clk);

        // launch
        start = 1; @(posedge clk); start = 0;

        // constant RF, held for the whole run (wrapper self-paces in MF_RUN)
        rf_din = DIN_CONST; rf_din_valid = 1'b1;

        // wait for completion, with a generous watchdog + progress prints
        watchdog = 0;
        while (!done_seen && watchdog < 8000000) begin
            @(posedge clk); watchdog = watchdog + 1;
            if (watchdog % 200000 == 0)
                $display("[%0t] alive: cyc=%0d  mf_ch=%0d mf_state=%0d t_state=%0d mf_done=%b pix_seen=%0d",
                         $time, watchdog, dut.mf_ch, dut.mf_state, dut.t_state, mf_done, pix_seen);
        end

        repeat (8) @(posedge clk);     // flush last pixel capture
        $fclose(fout);

        // ---------------- report ----------------
        $display("\n========================================");
        if (!done_seen)
            $display("RESULT: TIMEOUT - 'done' never asserted in %0d cycles", watchdog);
        else
            $display("RESULT: sweep complete - 'done' at cycle %0d (%0t)", watchdog, $time);
        $display("  pixels captured : %0d  (expected %0d)", pix_seen, NPIX);
        $display("  pixel value min : %0d", pmin);
        $display("  pixel value max : %0d", pmax);
        if (anyx != 0)
            $display("  *** %0d pixels had X bits (check Delay_calc / mem files)", anyx);
        else
            $display("  no X pixels (all numeric)");
        $display("  -> pixel dump written to sweep_pixels.txt (%0d rows)", pix_seen);
        $display("========================================\n");
        $finish;
    end

endmodule