// Per-channel-buffer datapath core (fits BRAM: ~7 in + 233 out RAMB36).
// Reuses the VERIFIED primed-read feed/capture FSM. One channel is buffered at a
// time in in_buf (1792 words); host fills it, we filter it, store output, repeat.
// UART-free for sim: the TB preloads in_buf per channel and pulses go.
`timescale 1ns/1ps
module mf_perch_core #(
    parameter DATA_WIDTH=32, TAPS=139, ACCUM_WIDTH=64,
    parameter NUM_CH=128, L=1792, STRIDE=2048,
    parameter ROM_FILE="fw2_coeffs.mem",
    parameter [31:0] SCALE_MULT=32'd11571513, parameter integer SCALE_SHIFT=46
)(
    input  wire clk, input wire rst,
    // per-channel input buffer write port (host fills this before pulsing go)
    input  wire [11:0] inbuf_waddr,
    input  wire        inbuf_we,
    input  wire [DATA_WIDTH-1:0] inbuf_wdata,
    input  wire        go,          // pulse: filter the buffered channel `ch_in`
    input  wire [6:0]  ch_in,       // which output channel this buffer is
    output reg         ch_done,     // pulse when this channel's outputs are written
    // output frame read port (host reads out_bram after all channels)
    input  wire [17:0] out_raddr,
    output reg  [DATA_WIDTH-1:0] out_rdata
);
    // ---- one-channel input buffer ----
    (* ram_style="block" *) reg [DATA_WIDTH-1:0] in_buf [0:L-1];
    reg [11:0] in_raddr; reg [DATA_WIDTH-1:0] in_rdata;
    always @(posedge clk) begin
        if (inbuf_we) in_buf[inbuf_waddr] <= inbuf_wdata;
        in_rdata <= in_buf[in_raddr];
    end
    // ---- output frame BRAM (bank_a layout) ----
    localparam OUT_WORDS=NUM_CH*STRIDE;
    (* ram_style="block" *) reg [DATA_WIDTH-1:0] out_bram [0:OUT_WORDS-1];
    reg [17:0] out_waddr; reg out_we; reg [DATA_WIDTH-1:0] out_wdata;
    always @(posedge clk) begin
        if (out_we) out_bram[out_waddr] <= out_wdata;
        out_rdata <= out_bram[out_raddr];
    end
    // ---- FIR ----
    reg mf_rst; reg signed [DATA_WIDTH-1:0] mf_din; reg mf_din_valid;
    wire signed [ACCUM_WIDTH-1:0] mf_dout; wire mf_dout_valid;
    matched_filter_1d_rom #(.DATA_WIDTH(DATA_WIDTH),.TAPS(TAPS),
        .ACCUM_WIDTH(ACCUM_WIDTH),.ROW_LENGTH(L),.ROM_FILE(ROM_FILE)) u_mf
      (.clk(clk),.rst(mf_rst),.din(mf_din),.din_valid(mf_din_valid),
       .dout(mf_dout),.dout_valid(mf_dout_valid));
    wire signed [ACCUM_WIDTH+31:0] sc_w = mf_dout*$signed({1'b0,SCALE_MULT});
    wire signed [DATA_WIDTH-1:0]   sc_q = (sc_w >>> SCALE_SHIFT);

    // ---- VERIFIED primed-read feed/capture FSM (from mf_frame_core) ----
    localparam S_IDLE=0,S_CHINIT=1,S_PRIME=2,S_FEED=3,S_DRAIN=4,S_FIN=5;
    reg [2:0] st; reg [11:0] fs, os; reg [4:0] dr; reg [6:0] ch;
    always @(posedge clk) begin
        out_we<=0; mf_din_valid<=0; mf_rst<=0; ch_done<=0;
        if (rst) begin st<=S_IDLE; end
        else case(st)
          S_IDLE: if (go) begin ch<=ch_in; st<=S_CHINIT; end
          S_CHINIT: begin mf_rst<=1; fs<=0; os<=0; in_raddr<=0; st<=S_PRIME; end
          S_PRIME:  begin in_raddr<=1; fs<=1; st<=S_FEED; end
          S_FEED: begin
             mf_din<=in_rdata; mf_din_valid<=1'b1;    // in_rdata holds sample (fs-1)
             if (fs<L) begin in_raddr<=fs+1'b1; fs<=fs+1'b1; end
             else begin dr<=0; st<=S_DRAIN; end
             if (mf_dout_valid) begin out_we<=1; out_waddr<=ch*STRIDE+os; out_wdata<=sc_q; os<=os+1'b1; end
          end
          S_DRAIN: begin
             if (mf_dout_valid) begin out_we<=1; out_waddr<=ch*STRIDE+os; out_wdata<=sc_q; os<=os+1'b1; end
             dr<=dr+1'b1; if (dr==5'd12) st<=S_FIN;
          end
          S_FIN: begin ch_done<=1'b1; st<=S_IDLE; end
        endcase
    end
    integer fo,k;
    task dump; begin
        fo=$fopen("frame_out.hex","w");
        for(k=0;k<OUT_WORDS;k=k+1) $fwrite(fo,"%08X\n",out_bram[k]);
        $fclose(fo);
    end endtask
endmodule
