
`timescale 1ns / 1ps

module tb_matched_filter_1d;

    parameter DATA_WIDTH  = 32;
    parameter TAPS        = 136;
    parameter ACCUM_WIDTH = 64;
    
    // Matrix dimensions
    parameter N_ROWS_X = 1792;
    parameter N_COLS_X = 128;
    
    // Transposed streaming dimensions
    parameter PAD_LEFT = 68;
    parameter PAD_RIGHT = 67;
    parameter ROW_LENGTH = 1927; // 68 + 1792 + 67

    reg clk, rst, din_valid;
    reg signed [DATA_WIDTH-1:0] din;
    wire signed [ACCUM_WIDTH-1:0] dout;
    wire dout_valid;

    // Memories for .mem files
    reg signed [DATA_WIDTH-1:0] x_mem [0:(N_ROWS_X * N_COLS_X)-1]; 
    reg [63:0] expected_hex_mem [0:(N_COLS_X * N_ROWS_X)-1]; 
    real expected_real [0:(N_COLS_X * N_ROWS_X)-1];          
    
    integer curr_row, curr_col, out_idx, i;
    real scale_out, rtl_float_val, error, max_error;

    // Instantiate UUT
    matched_filter_1d uut (
        .clk(clk), .rst(rst),
        .din(din), .din_valid(din_valid),
        .dout(dout), .dout_valid(dout_valid)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 0; rst = 1; din = 0; din_valid = 0;
        out_idx = 0; max_error = 0.0;
        scale_out = 1073741824.0; // 32768.0 * 32768.0 (Q16.15 * Q16.15)
        
        $display("Loading memory files...");
        // Vivado will locate these automatically if added as Simulation Sources
        $readmemh("fw2_data.mem", uut.coeffs); 
        $readmemh("x_data.mem", x_mem);
        $readmemh("expected_data.mem", expected_hex_mem);
        
        // Convert the 64-bit IEEE 754 Hex into standard Verilog 'real' floats
        for (i = 0; i < N_COLS_X * N_ROWS_X; i = i + 1) begin
            expected_real[i] = $bitstoreal(expected_hex_mem[i]);
        end

        #100; rst = 0; #20;
        $display("Streaming transposed data into pipeline...");
        
        for (curr_row = 0; curr_row < N_COLS_X; curr_row = curr_row + 1) begin
            for (curr_col = 0; curr_col < ROW_LENGTH; curr_col = curr_col + 1) begin
                @(posedge clk);
                din_valid <= 1'b1;
                
                if (curr_col < PAD_LEFT) begin
                    din <= 0; 
                end else if (curr_col < PAD_LEFT + N_ROWS_X) begin
                    // Feed directly from 1D flattened hex memory (Hardware Transpose)
                    din <= x_mem[(curr_col - PAD_LEFT) * N_COLS_X + curr_row];
                end else begin
                    din <= 0; 
                end
            end
        end
        
        @(posedge clk); din_valid <= 1'b0;
        
        // Wait for final pixels to drain out
        #200;
        
        $display("==========================================================");
        $display("SIMULATION COMPLETE");
        $display("Total Pixels Verified: %0d", out_idx);
        $display("max error: %f", max_error);
        $display("==========================================================");
        $finish;
    end

    always @(posedge clk) begin
        if (dout_valid) begin
            
            // Reconstruct float from fixed-point hardware accumulator
            rtl_float_val = dout; 
            rtl_float_val = rtl_float_val / scale_out; 
            
            // Calculate error against MATLAB
            error = rtl_float_val - expected_real[out_idx];
            if (error < 0) error = -error;
            if (error > max_error) max_error = error;
            
            // Debug print for the first 20 pixels to PROVE non-zero matching
            if (out_idx < 20) begin
                $display("Idx: %6d | RTL: %10.5f | MATLAB: %10.5f | Err: %f", 
                         out_idx, rtl_float_val, expected_real[out_idx], error);
            end
            
// display every 10k pixels display
            if (out_idx % 10000 == 0 && out_idx > 0) begin
                $display("Verified %0d pixels...", out_idx);
            end
            
            out_idx = out_idx + 1;
        end
    end

endmodule
