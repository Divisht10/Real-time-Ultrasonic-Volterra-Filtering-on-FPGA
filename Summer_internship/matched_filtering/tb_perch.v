`timescale 1ns/1ps
module tb_perch;
  localparam NUM_CH=128, L=1792, STRIDE=2048, DW=32, TESTCH=4;
  reg clk=0; always #5 clk=~clk;
  reg rst=1;
  reg [11:0] wa; reg we; reg [DW-1:0] wd;
  reg go; reg [6:0] ch_in; wire ch_done;
  reg [17:0] ra; wire [DW-1:0] rd;
  mf_perch_core #(.NUM_CH(NUM_CH),.L(L),.STRIDE(STRIDE)) dut
    (.clk(clk),.rst(rst),.inbuf_waddr(wa),.inbuf_we(we),.inbuf_wdata(wd),
     .go(go),.ch_in(ch_in),.ch_done(ch_done),.out_raddr(ra),.out_rdata(rd));
  reg [DW-1:0] frame [0:NUM_CH*L-1];
  integer c,i;
  initial begin
    $readmemh("frame_in.hex", frame);
    we=0; go=0; ra=0;
    repeat(4) @(negedge clk); rst=0;
    for (c=0;c<TESTCH;c=c+1) begin
      // fill in_buf with channel c
      for (i=0;i<L;i=i+1) begin @(negedge clk); wa=i; wd=frame[c*L+i]; we=1; end
      @(negedge clk); we=0;
      // pulse go
      @(negedge clk); ch_in=c; go=1; @(negedge clk); go=0;
      wait(ch_done); @(negedge clk);
      $display("channel %0d done", c);
    end
    dut.dump(); $display("ALL DONE"); $finish;
  end
  initial begin #2000000000; $display("TIMEOUT"); $finish; end
endmodule
