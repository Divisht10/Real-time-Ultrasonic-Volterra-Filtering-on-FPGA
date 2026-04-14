`timescale 1ns / 1ps

module custom_flipud_ff #(
    parameter DW = 16,     
    parameter ROWS = 15,
    parameter COLS = 128
)(
    input  wire                 clk,
    input  wire                 rst_n,
    
    // Write Interface
    input  wire                 write_en,
    input  wire [3:0]           write_row, 
    input  wire [6:0]           write_col, 
    input  wire signed [DW-1:0] data_in,
    
    // Read Interface
    input  wire                 read_en,
    input  wire [3:0]           read_row,  
    input  wire [6:0]           read_col,  
    output reg signed [DW-1:0]  data_out
);

    reg signed [DW-1:0] frame_buffer [0:(ROWS*COLS)-1];
    
    wire [10:0] write_addr = (write_row * COLS) + write_col;
    wire [10:0] read_addr  = ((ROWS - 1 - read_row) * COLS) + read_col;
    
    integer i;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < ROWS*COLS; i = i + 1) begin
                frame_buffer[i] <= {DW{1'b0}};
            end
            data_out <= {DW{1'b0}};
        end else begin
            if (write_en) begin
                frame_buffer[write_addr] <= data_in;
            end
            
            if (read_en) begin
                data_out <= frame_buffer[read_addr];
            end
        end
    end

endmodule
