// ============================================================================
// mf_poc_top.v  --  Matched-filter PROOF OF CONCEPT (per-channel buffered)
//
// Proves W1 (FW2 matched filter) runs in HARDWARE, replacing x2_to_bank.py.
// Produces a bank_a-layout, 0.5-FS Q2.30 output bit-compatible with the .coe
// the beamformer consumes.
//
// FITS: one-channel input buffer (~7 RAMB36) + output frame BRAM (~233) ~= 240
// of 365 RAMB36. (The old full-frame-input version needed ~437 and overflowed.)
//
// PROTOCOL (per channel, x128):
//   host streams L=1792 Q2.30 words (that channel's RF) -> in_buf
//   FPGA filters via matched_filter_1d_rom, scales 0.5 FS, writes out_bram[ch]
//   -> next channel
// After all 128 channels: host sends 'S', FPGA streams back preamble + full
//   filtered frame (128 x 1654 valid words, channel-major).
//
// Reuses proven uart_rx / uart_tx.  50 MHz, CLKS_PER_BIT=434 (115200).
// Datapath VERIFIED BIT-EXACT in simulation (mf_perch_core / tb_perch).
// ============================================================================
`timescale 1ns / 1ps
module mf_poc_top #(
    parameter DATA_WIDTH   = 32,
    parameter TAPS         = 139,
    parameter ACCUM_WIDTH  = 64,
    parameter NUM_CH       = 128,
    parameter L            = 1792,
    parameter STRIDE       = 2048,
    parameter VALID_LEN    = 1654,        // L-(TAPS-1) outputs per channel
    parameter CLKS_PER_BIT = 434,
    parameter ROM_FILE     = "fw2_coeffs.mem",
    parameter [31:0] SCALE_MULT  = 32'd11571513,
    parameter integer SCALE_SHIFT = 46
)(
    input  wire        sys_clk_p,
    input  wire        sys_clk_n,
    input  wire        reset,
    input  wire        uart_rxd,
    output wire        uart_txd,
    output wire [3:0]  led
);
    // ---- clocks ----
    wire clk_200, clk;
    IBUFGDS u_ibuf (.I(sys_clk_p), .IB(sys_clk_n), .O(clk_200));
    reg [1:0] div = 2'd0; always @(posedge clk_200) div <= div + 1'b1;
    BUFG u_bufg (.I(div[1]), .O(clk));
    // ---- reset sync ----
    (* ASYNC_REG="TRUE" *) reg [1:0] rst_sync = 2'b11;
    always @(posedge clk or posedge reset)
        if (reset) rst_sync <= 2'b11; else rst_sync <= {rst_sync[0],1'b0};
    wire rst = rst_sync[1];
    reg [24:0] hb=0; always @(posedge clk) hb<=hb+1'b1;

    // ---- UART ----
    wire rx_valid; wire [7:0] rx_byte;
    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx
      (.clk(clk), .reset(rst), .rx(uart_rxd), .rx_data(rx_byte), .rx_valid(rx_valid));
    reg tx_start; reg [7:0] tx_data; wire tx_busy;
    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx
      (.clk(clk), .reset(rst), .tx_start(tx_start), .tx_data(tx_data),
       .tx(uart_txd), .tx_busy(tx_busy));

    // ---- RX word assembly (4 bytes BE -> Q2.30 word) ----
    reg [1:0] rx_bcnt; reg [31:0] rx_word; reg rx_word_valid;
    always @(posedge clk) begin
        rx_word_valid<=1'b0;
        if (rst) rx_bcnt<=0;
        else if (rx_valid) begin
            rx_word<={rx_word[23:0],rx_byte};
            if (rx_bcnt==2'd3) begin rx_bcnt<=0; rx_word_valid<=1'b1; end
            else rx_bcnt<=rx_bcnt+1'b1;
        end
    end

    // ---- per-channel core ----
    reg  [11:0] inbuf_waddr; reg inbuf_we; reg [31:0] inbuf_wdata;
    reg         go; reg [6:0] ch_in; wire ch_done;
    reg  [17:0] out_raddr; wire [31:0] out_rdata;
    mf_perch_core #(.DATA_WIDTH(DATA_WIDTH),.TAPS(TAPS),.ACCUM_WIDTH(ACCUM_WIDTH),
        .NUM_CH(NUM_CH),.L(L),.STRIDE(STRIDE),.ROM_FILE(ROM_FILE),
        .SCALE_MULT(SCALE_MULT),.SCALE_SHIFT(SCALE_SHIFT)) u_core
      (.clk(clk),.rst(rst),
       .inbuf_waddr(inbuf_waddr),.inbuf_we(inbuf_we),.inbuf_wdata(inbuf_wdata),
       .go(go),.ch_in(ch_in),.ch_done(ch_done),
       .out_raddr(out_raddr),.out_rdata(out_rdata));

    // ---- top FSM ----
    localparam S_FILL=4'd0,   // receive L words for current channel into in_buf
               S_GO  =4'd1,   // pulse go
               S_WAIT=4'd2,   // wait ch_done
               S_NEXT=4'd3,   // advance channel or move to readback
               S_WAITS=4'd4,  // wait 'S' to start readback
               S_PRE =4'd5,   // preamble
               S_TX  =4'd6,   // stream frame back
               S_DONE=4'd7;
    reg [3:0] state;
    reg [11:0] fill_cnt; reg [6:0] ch;
    reg [2:0] pre_idx; reg [17:0] tx_word_idx; reg [1:0] tx_bcnt, rd_lat;
    localparam [7:0] START_CMD=8'h53;
    function [7:0] pre_byte(input [2:0] i);
        case(i) 0:pre_byte=8'hAA;1:pre_byte=8'h55;2:pre_byte=8'hAA;3:pre_byte=8'h55;
                default:pre_byte=8'h00; endcase
    endfunction

    always @(posedge clk) begin
        inbuf_we<=0; go<=0; tx_start<=0;
        if (rst) begin
            state<=S_FILL; fill_cnt<=0; ch<=0; pre_idx<=0; tx_word_idx<=0;
            tx_bcnt<=0; rd_lat<=0;
        end else case(state)
          S_FILL: if (rx_word_valid) begin
                    inbuf_we<=1'b1; inbuf_waddr<=fill_cnt; inbuf_wdata<=rx_word;
                    if (fill_cnt==L-1) begin fill_cnt<=0; state<=S_GO; end
                    else fill_cnt<=fill_cnt+1'b1;
                  end
          S_GO:  begin ch_in<=ch; go<=1'b1; state<=S_WAIT; end
          S_WAIT: if (ch_done) state<=S_NEXT;
          S_NEXT: if (ch==NUM_CH-1) state<=S_WAITS;
                  else begin ch<=ch+1'b1; state<=S_FILL; end
          S_WAITS: if (rx_valid && rx_byte==START_CMD) begin
                     pre_idx<=0; state<=S_PRE;
                   end
          S_PRE: if (!tx_busy && !tx_start) begin
                   tx_data<=pre_byte(pre_idx); tx_start<=1'b1;
                   if (pre_idx==3'd3) begin
                     pre_idx<=0; tx_word_idx<=0; tx_bcnt<=0; out_raddr<=0; rd_lat<=0;
                     state<=S_TX;
                   end else pre_idx<=pre_idx+1'b1;
                 end
          // stream VALID_LEN words per channel, channel-major (skip stride tail)
          S_TX: begin
             out_raddr <= (tx_word_idx/VALID_LEN)*STRIDE + (tx_word_idx%VALID_LEN);
             if (rd_lat<2) rd_lat<=rd_lat+1'b1;
             else if (!tx_busy && !tx_start) begin
               tx_data <= (tx_bcnt==0)?out_rdata[31:24]:(tx_bcnt==1)?out_rdata[23:16]:
                          (tx_bcnt==2)?out_rdata[15:8]:out_rdata[7:0];
               tx_start<=1'b1;
               if (tx_bcnt==2'd3) begin
                 tx_bcnt<=0;
                 if (tx_word_idx==NUM_CH*VALID_LEN-1) state<=S_DONE;
                 else begin tx_word_idx<=tx_word_idx+1'b1; rd_lat<=0; end
               end else tx_bcnt<=tx_bcnt+1'b1;
             end
          end
          S_DONE: ;
        endcase
    end
    assign led[0]=hb[24];
    assign led[1]=(state==S_FILL)||(state==S_GO)||(state==S_WAIT); // filtering
    assign led[2]=(state>=S_PRE);                                   // streaming
    assign led[3]=(state==S_DONE);
endmodule