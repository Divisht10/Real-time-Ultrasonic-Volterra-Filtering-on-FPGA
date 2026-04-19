// ============================================================================
// File        : matched_filter_1d_rom.v
// Description : Patched version of convolution.v (matched_filter_1d)
//               Adds parameter-driven coefficient ROM initialization.
//               The original convolution.v declares coeffs[] but never loads it;
//               this version adds $readmemh with a configurable ROM_FILE parameter.
//
// CHANGE vs original convolution.v:
//   Added parameter ROM_FILE = "coeffs.mem"
//   Added: initial $readmemh(ROM_FILE, coeffs);
//
// In w1_matched_filter_wrapper.v, instantiate this module instead of
// matched_filter_1d, using:
//   .ROM_FILE("fw1_coeffs.mem")  for FW1 (fundamental)
//   .ROM_FILE("fw2_coeffs.mem")  for FW2 (subharmonic)
// ============================================================================

`timescale 1ns / 1ps

module matched_filter_1d_rom #(
    parameter DATA_WIDTH  = 32,
    parameter TAPS        = 136,
    parameter ACCUM_WIDTH = 64,
    parameter ROW_LENGTH  = 1927,
    parameter ROM_FILE    = "coeffs.mem"  // NEW: coefficient ROM file path
)(
    input  wire clk,
    input  wire rst,      // Active-high reset

    input  wire signed [DATA_WIDTH-1:0] din,
    input  wire din_valid,

    output reg  signed [ACCUM_WIDTH-1:0] dout,
    output reg  dout_valid
);

    // Coefficient ROM: loaded from ROM_FILE at synthesis/simulation
    reg signed [DATA_WIDTH-1:0] coeffs [0:TAPS-1];
    initial $readmemh(ROM_FILE, coeffs);   // <<< KEY ADDITION

    reg signed [DATA_WIDTH-1:0] shift_reg [0:TAPS-1];
    reg signed [(2*DATA_WIDTH)-1:0] mult_out [0:TAPS-1];

    reg signed [ACCUM_WIDTH-1:0] add_tree_out;
    reg signed [ACCUM_WIDTH-1:0] sum;

    reg pipe_valid_1, pipe_valid_2, pipe_valid_3;
    reg [15:0] col_count;
    integer i;

    always @(posedge clk) begin
        if (rst) begin
            dout          <= 0;
            dout_valid    <= 0;
            pipe_valid_1  <= 0;
            pipe_valid_2  <= 0;
            pipe_valid_3  <= 0;
            add_tree_out  <= 0;
            col_count     <= 0;
            for (i = 0; i < TAPS; i = i + 1) begin
                shift_reg[i] <= 0;
                mult_out[i]  <= 0;
            end
        end else begin

            // Stage 1: Shift Register
            if (din_valid) begin
                shift_reg[0] <= din;
                for (i = 1; i < TAPS; i = i + 1)
                    shift_reg[i] <= shift_reg[i-1];

                if (col_count == ROW_LENGTH - 1)
                    col_count <= 0;
                else
                    col_count <= col_count + 1;

                if (col_count >= TAPS - 1)
                    pipe_valid_1 <= 1'b1;
                else
                    pipe_valid_1 <= 1'b0;
            end else begin
                pipe_valid_1 <= 1'b0;
            end

            // Stage 2: Parallel Multipliers
            for (i = 0; i < TAPS; i = i + 1)
                mult_out[i] <= shift_reg[TAPS-1-i] * coeffs[i];
            pipe_valid_2 <= pipe_valid_1;

            // Stage 3: Accumulation Tree
            sum = 0;
            for (i = 0; i < TAPS; i = i + 1)
                sum = sum + mult_out[i];
            add_tree_out  <= sum;
            pipe_valid_3  <= pipe_valid_2;

            // Stage 4: Output
            dout       <= add_tree_out;
            dout_valid <= pipe_valid_3;
        end
    end

endmodule
