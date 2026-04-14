`timescale 1ns / 1ps

module cordic_sqrt #(

    parameter N = 64,

    parameter I = 12,

    parameter F = N - I,

    parameter LFB = 6,

    parameter iterations = 30,

    parameter logiter = 6

)(
    
    input  [N-1:0] n,

    output [(N/2)-1:0] sqrt,

    input clk, start,reset,

    output reg done

);

wire [N-1:0] n_norm;
wire [(N/2)-1:0] root_norm;
reg busy = 0;
wire signed [LFB:0] m, lead_pos,lz;
clz #(.N(N)) clz_inst (.x(n), .count(lz));

localparam signed [LFB:0] F_s = F;

assign lead_pos = (n == 0) ? 0 : (N - lz -1 );

assign m = ($signed(lead_pos - F_s + 1 )) >>> 1; // Used arithmetic shift for signed m

assign n_norm = ($signed(m) >= 0) ? (n >> (2*m)) : (n << (-2*m));
wire[N/2-1:0] trial_subtrahend;

reg [N/2-1:0] y;


reg signed [N/2-1:0] r;

reg signed [logiter:0] i;
reg [32:0] test_bit;
    assign    trial_subtrahend = ( 2*y+test_bit)>>i;

always @(posedge clk) begin
    if (!reset) begin

        i <= 0;
        y <= 32'd0;
        r <= 0;
        test_bit <= 32'h00000000;
        busy <= 0;
        done <= 0;
    end
    if (start && !busy) begin

        i <= 1;
        y <= 32'd0;
        r <= n_norm>>>26;
        test_bit <= 32'h80000000;
        busy <= 1;
        done <= 0;


    end

    else if (busy) begin


        if (i <= iterations) begin
        test_bit<=test_bit>>1;

            if (r >= trial_subtrahend) begin
                r <= r - trial_subtrahend;

                y <= y+test_bit;


            end
            i <= i + 1;
            done <= (i == iterations);

        end

    end

end


assign root_norm = y>>3;

assign sqrt = (m >= 0) ? (root_norm << m) : (root_norm >> (-m));


endmodule 
