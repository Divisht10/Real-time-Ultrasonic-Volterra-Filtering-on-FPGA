`timescale 1ns / 1ps

module clz #(
    parameter N = 64,
    parameter logN = 6
)(
    input  [N-1:0] x,
    output reg [logN:0] count
);
    integer i;
    reg found;

    always @(*) begin
        count = N;
        found = 0;

        for (i = N-1; i >= 0; i = i - 1) begin
            if (!found && x[i]) begin
                count = N-1-i;
                found = 1;
            end
        end
    end
endmodule
