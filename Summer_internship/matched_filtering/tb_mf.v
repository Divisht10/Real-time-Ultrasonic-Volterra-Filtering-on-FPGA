`timescale 1ns/1ps
module tb_mf;
  localparam TAPS=139, L=1792, DW=32, AW=64;
  reg clk=0; always #5 clk=~clk;
  reg rst=1;
  reg signed [DW-1:0] din; reg din_valid=0;
  wire signed [AW-1:0] dout; wire dout_valid;
  matched_filter_1d_rom #(.DATA_WIDTH(DW),.TAPS(TAPS),.ACCUM_WIDTH(AW),
      .ROW_LENGTH(L),.ROM_FILE("fw2_coeffs.mem")) dut
    (.clk(clk),.rst(rst),.din(din),.din_valid(din_valid),.dout(dout),.dout_valid(dout_valid));
  reg [DW-1:0] rf [0:L-1];
  integer i, fo;
  initial begin
    $readmemh("ch0_rf.hex", rf);
    fo=$fopen("ch0_hw.txt","w");
    @(negedge clk); rst=0;
    for (i=0;i<L;i=i+1) begin
      @(negedge clk); din=rf[i]; din_valid=1;
    end
    @(negedge clk); din_valid=0;
    repeat(10) @(negedge clk);
    $fclose(fo); $finish;
  end
  // log every valid output with a running index
  always @(posedge clk) if (dout_valid) $fwrite(fo, "%0d\n", dout);
endmodule
