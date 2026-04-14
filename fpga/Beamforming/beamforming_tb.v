    `timescale 1ns / 1ps
    
    module beamform_tb();
    
        // Parameters
        parameter DATA_WIDTH = 32;
        parameter APERTURE_SIZE = 50;
        parameter CLK_PERIOD = 10; // 100MHz
    
        // Signals
        reg clk;
        reg reset;
        reg load_values;
        reg [7:0] center_idx;
        reg [DATA_WIDTH-1:0] input_line;
        
        wire [DATA_WIDTH-1:0] final_pixel_val;
        wire pixel_valid;
    
        // Instantiate the Unit Under Test (UUT)
        beamforming #(
            .DATA_WIDTH(DATA_WIDTH),
            .APERTURE_SIZE(APERTURE_SIZE)
        ) uut (
            .clk(clk),
            .reset(reset),
            .load_values(load_values),
            .center_idx(center_idx),
            .input_line(input_line),
            .final_pixel_val(final_pixel_val),
            .pixel_valid(pixel_valid)
        );
    
        // Clock Generation
        initial clk = 0;
        always #(CLK_PERIOD/2) clk = ~clk;
    
        // Test Sequence
        // Test Sequence matching MATLAB settings
        initial begin
            reset = 0;
            load_values = 0;
            center_idx = 8'd64; 
            input_line = 0;
    
            #(CLK_PERIOD * 10);
            reset = 1;
            #(CLK_PERIOD * 5);
    
            // Cycle 1: x_pixel (5.0)
            load_values = 1;
            input_line = 32'h04000000; 
            #(CLK_PERIOD);
            
            // Cycle 2: z_pixel (15.0)
            input_line = 32'h3c000000;
            #(CLK_PERIOD);
                  
            // Cycle 3: start_depth (2.026316)
            input_line = 32'h081B10CE;
            #(CLK_PERIOD);
            #(CLK_PERIOD);
            
            load_values = 0;
            // ... rest of testbench
    
            // --- STEP 2: Wait for Pipeline ---
            // Your pipeline has roughly 30 (CORDIC) + 6 (Adder Tree) + 1 (Norm) cycles.
            // We'll wait 50 cycles to see the valid signal toggle.
            wait(pixel_valid == 1);
            $display("Result Received: final_pixel_val = %d", final_pixel_val);
            
            #(CLK_PERIOD * 500);
            
            $display("Simulation Finished");
            $finish;
        end
    
        // Optional: Monitor changes
        initial begin
            $monitor("Time: %t | Valid: %b | Val: %h", $time, pixel_valid, final_pixel_val);
        end
    
    endmodule
