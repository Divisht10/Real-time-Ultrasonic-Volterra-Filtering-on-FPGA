// ============================================================================
// mf_beamform_eth_uart_top.v
//   AC701 top: RF in over RGMII Ethernet -> W1 matched filter + W2 beamform
//   -> beamformed pixels out over UART.  Phase 3 (Volterra) NOT included yet.
//
//   Data path:
//     phy_rxc/rxd --eth_rx_rgmii--> rx_data(32b) --rf_fifo(BRAM)--> rf_din
//        --mf_beamform_top(W1+W2)--> pixel_out(32b) --pix_fifo--> uart 4 bytes/px
//
//   - Coefficients (fw2) load on startup via $readmemh inside mf_beamform_top.
//   - RF must be streamed CHANNEL-MAJOR (ch0 s0..sL-1, ch1 ...) by the host;
//     the matched filter consumes it in that order and stalls when the FIFO is
//     empty (rf_din_valid low), so the laptop sets the pace.
//   - The RF FIFO is a BRAM FIFO (rate buffer) rather than a full frame store:
//     a full 128x1792x32b RF buffer (~204 BRAMs) plus bank_a (~256) overflows
//     the XC7A200T's 365 BRAMs, so we stream + stall instead.
//
//   eth_rx_rgmii and uart_tx are your modules (in eth_uart_ac701.v).
// ============================================================================
`timescale 1ns/1ps
module mf_beamform_eth_uart_top #(
    // mf_beamform_top config (set to your real grid)
    parameter L         = 1792,
    parameter NUM_CH    = 128,
    parameter AXIAL     = 973,
    parameter LATERAL   = 128,
    parameter PIX_LAT_W = 7,
    parameter PIX_AX_W  = 10,
    parameter signed [31:0] START_DEPTH_Q626 = 32'sh08000000,
    parameter CLKS_PER_BIT = 434,      // 50 MHz / 115200
    parameter RF_FIFO_AW   = 9,        // 512-word RF rate buffer (LUTRAM FWFT)
    parameter PIX_FIFO_AW  = 9         // 512-word pixel buffer (LUTRAM FWFT)
)(
    input  wire        sys_clk_p,
    input  wire        sys_clk_n,
    input  wire        reset,          // SW7, active high
    output wire        phy_reset_n,
    input  wire        phy_rxc,
    input  wire        phy_rx_ctl,
    input  wire [3:0]  phy_rxd,
    output wire        uart_txd,
    output wire [3:0]  led
);
    // ---------------- clocking: 200 MHz -> 50 MHz core via FF /4 divider --------
    // No MMCM for the core (avoids any MMCM-lock dependency; your echo design
    // used the same FF-divider style and worked). 200/4 = 50 MHz.
    wire sys_clk_ibuf, clk_core;
    IBUFGDS u_ibuf (.I(sys_clk_p), .IB(sys_clk_n), .O(sys_clk_ibuf));
    (* keep = "true" *) reg [1:0] clk_div = 2'b00;          // XDC: clk_div_reg[1]/Q
    always @(posedge sys_clk_ibuf) clk_div <= clk_div + 1'b1;
    BUFG u_bufg (.I(clk_div[1]), .O(clk_core));             // 50 MHz core clock

    // SW7 reset (async, active-high) synchronized into the core domain
    reg rst_meta, rst_sync_r;
    always @(posedge clk_core) begin rst_meta <= reset; rst_sync_r <= rst_meta; end
    wire rst_sync = rst_sync_r;
    wire rst_n    = ~rst_sync_r;

    // heartbeat: proves the core clock is alive (led[0] blinks ~1.5 Hz @50 MHz)
    reg [24:0] hb;
    always @(posedge clk_core) hb <= hb + 1'b1;

    // ---------------- Ethernet RX (your module) ----------------
    wire [31:0] rx_data;  wire rx_valid, more_out;  wire [3:0] rx_state;
    eth_rx_rgmii u_eth (
        .sys_clk(clk_core), .reset(rst_sync),
        .phy_rxc(phy_rxc), .phy_rx_ctl(phy_rx_ctl), .phy_rxd(phy_rxd),
        .phy_reset_n(phy_reset_n),
        .rx_data(rx_data), .rx_valid(rx_valid), .more_out(more_out),
        .rx_state(rx_state)
    );

    // ---------------- RF rate FIFO (eth -> matched filter) ----------------
    wire        rf_fifo_empty, rf_fifo_full;
    wire [31:0] rf_din;
    wire        rf_ready;                            // mf_beamform_top asks for a sample
    wire        rf_rd = rf_ready & ~rf_fifo_empty;
    sync_fifo #(.DW(32), .AW(RF_FIFO_AW)) u_rf_fifo (
        .clk(clk_core), .rst(rst_sync),
        .wr(rx_valid & ~rf_fifo_full), .din(rx_data),
        .rd(rf_rd), .dout(rf_din),
        .empty(rf_fifo_empty), .full(rf_fifo_full)
    );
    wire rf_din_valid = ~rf_fifo_empty;

    // ---------------- one start pulse after reset/lock ----------------
    reg [3:0] start_dly; reg started;
    always @(posedge clk_core) begin
        if (rst_sync) begin start_dly<=0; started<=0; end
        else if (!started) begin start_dly<=start_dly+1'b1; if (&start_dly) started<=1; end
    end
    wire mf_start = (start_dly==4'hE) & ~started;     // single pulse

    // ---------------- W1 + W2 ----------------
    wire signed [31:0] pixel_out;  wire pixel_out_valid;
    wire [PIX_LAT_W-1:0] pixel_lat_idx;  wire [PIX_AX_W-1:0] pixel_ax_idx;
    wire mf_done, all_done;
    mf_beamform_top #(
        .L(L), .NUM_CH(NUM_CH), .AXIAL(AXIAL), .LATERAL(LATERAL),
        .PIX_LAT_W(PIX_LAT_W), .PIX_AX_W(PIX_AX_W),
        .START_DEPTH_Q626(START_DEPTH_Q626)
    ) u_mfbf (
        .clk(clk_core), .rst_n(rst_n), .start(mf_start), .enable_sweep(1'b1),
        .pixel_ready(~pix_fifo_full),   // throttle sweep to UART rate (no FIFO overflow)
        .rf_din(rf_din), .rf_din_valid(rf_din_valid), .rf_ready(rf_ready),
        .pixel_out(pixel_out), .pixel_out_valid(pixel_out_valid),
        .pixel_lat_idx(pixel_lat_idx), .pixel_ax_idx(pixel_ax_idx),
        .mf_done(mf_done), .done(all_done)
    );

    // ---------------- pixel -> UART (4 bytes/pixel, MSB first) ----------------
    wire pix_fifo_full, pix_overflow;
    pixel_uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT), .AW(PIX_FIFO_AW)) u_pxuart (
        .clk(clk_core), .rst(rst_sync),
        .pixel(pixel_out), .pixel_valid(pixel_out_valid),
        .full(pix_fifo_full), .overflow(pix_overflow),
        .uart_txd(uart_txd)
    );

    // ---------------- status LEDs ----------------
    // ---------------- status LEDs (stretched/latched so they're readable) ----
    reg [22:0] rx_act;          // RX-activity monostable (~0.17 s @50 MHz)
    reg        done_lat, pix_lat;
    always @(posedge clk_core) begin
        if (rst_sync) begin rx_act<=0; done_lat<=0; pix_lat<=0; end
        else begin
            if (rx_valid)         rx_act   <= {23{1'b1}};   // re-trigger on any word
            else if (rx_act != 0) rx_act   <= rx_act - 1'b1;
            if (all_done)         done_lat <= 1'b1;         // latch sweep done
            if (pixel_out_valid)  pix_lat  <= 1'b1;         // latch first pixel out
        end
    end
    assign led[0] = hb[24];             // HEARTBEAT - blinks if core clock alive
    assign led[1] = (rx_act != 0);      // RF arriving over Ethernet (stretched -> visible)
    assign led[2] = done_lat;           // beamform sweep finished (latched)
    assign led[3] = pix_lat;            // pipeline produced >=1 pixel (latched)
endmodule

// ============================================================================
// ==========================================================================
// sync_fifo - first-word-fall-through FIFO (dout = current head, combinational).
// LUTRAM (distributed); fine for modest depth. For large buffers use a BRAM
// FWFT FIFO IP instead.
// ============================================================================
module sync_fifo #(parameter DW=32, parameter AW=9)(
    input  wire clk, rst,
    input  wire wr, input wire [DW-1:0] din,
    input  wire rd, output wire [DW-1:0] dout,
    output wire empty, full
);
    localparam DEPTH=(1<<AW);
    reg [DW-1:0] mem [0:DEPTH-1];
    reg [AW:0] wptr, rptr;
    assign empty = (wptr==rptr);
    assign full  = (wptr[AW]!=rptr[AW]) && (wptr[AW-1:0]==rptr[AW-1:0]);
    assign dout  = mem[rptr[AW-1:0]];          // FWFT: head always visible
    always @(posedge clk) begin
        if (rst) begin wptr<=0; rptr<=0; end
        else begin
            if (wr && !full)  begin mem[wptr[AW-1:0]]<=din; wptr<=wptr+1'b1; end
            if (rd && !empty) rptr<=rptr+1'b1;
        end
    end
endmodule

// pixel_uart_tx - buffers 32-bit pixels and ships them as 4 UART bytes (MSB
// first), instantiating your uart_tx.  Drops (flags) on overflow.
// ============================================================================
module pixel_uart_tx #(parameter CLKS_PER_BIT=868, parameter AW=11)(
    input  wire        clk, rst,
    input  wire [31:0] pixel, input wire pixel_valid,
    output wire        full, output reg overflow,
    output wire        uart_txd
);
    // pixel FIFO
    wire pf_empty, pf_full;  wire [31:0] pf_dout;  reg pf_rd;
    sync_fifo #(.DW(32), .AW(AW)) u_pf (
        .clk(clk), .rst(rst),
        .wr(pixel_valid & ~pf_full), .din(pixel),
        .rd(pf_rd), .dout(pf_dout), .empty(pf_empty), .full(pf_full));
    assign full = pf_full;
    always @(posedge clk) if (rst) overflow<=0; else if (pixel_valid & pf_full) overflow<=1;

    // uart
    reg        tx_start; reg [7:0] tx_data; wire tx_busy;
    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_uart (
        .clk(clk), .reset(rst), .tx_start(tx_start), .tx_data(tx_data),
        .tx(uart_txd), .tx_busy(tx_busy));

    // serializer: with FWFT, pf_dout is the head; capture it and advance.
    localparam S_IDLE=2'd0, S_SEND=2'd1, S_WAIT=2'd2;
    reg [1:0]  st; reg [1:0] byte_idx; reg [31:0] word;
    always @(posedge clk) begin
        if (rst) begin st<=S_IDLE; pf_rd<=0; tx_start<=0; byte_idx<=0; word<=0; end
        else begin
            pf_rd<=0; tx_start<=0;
            case (st)
            S_IDLE:  if (!pf_empty) begin
                         word<=pf_dout; pf_rd<=1'b1;     // grab head, advance
                         byte_idx<=2'd3; st<=S_SEND;
                     end
            S_SEND:  if (!tx_busy) begin
                         tx_data<=word[byte_idx*8 +: 8]; // MSB byte first
                         tx_start<=1'b1; st<=S_WAIT;
                     end
            S_WAIT:  if (tx_busy) begin                  // byte accepted
                         if (byte_idx==0) st<=S_IDLE;
                         else begin byte_idx<=byte_idx-1'b1; st<=S_SEND; end
                     end
            default: st<=S_IDLE;
            endcase
        end
    end
endmodule