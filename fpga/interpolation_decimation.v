module custom_interp1 (
    input  wire clk,
    input  wire rst,
    input  wire signed [DATA_WIDTH-1:0] y_in,                   
    input  wire y_in_valid,
    output reg  signed [DATA_WIDTH-1:0] vq_out,               
    output reg  vq_out_valid
);

    parameter DATA_WIDTH  = 32;   // Q2.30
    parameter PHASE_WIDTH = 16;
    
    // Step Size = T_new / T_orig = 32ns / 4ns = 8.0
    // In Q16 format: 8.0 * 65536 = 524288
    parameter STEP_SIZE   = 32'd524288;
    
    // First t_new point is 32ns, which corresponds to the 8th t_orig sample
    // In Q16 format: 8.0 * 65536 = 524288
    parameter START_TIME  = 32'd524288;

    reg signed [DATA_WIDTH-1:0] y_curr;
    
    // Time trackers in RTL representation
    reg [31:0] current_time; 
    reg [31:0] target_time;  

    wire [16:0] frac_32 = target_time - (current_time - 32'd65536);

    //33 bit subtraction
    wire signed [DATA_WIDTH:0] y_in_ext   = {y_in[DATA_WIDTH-1], y_in};
    wire signed [DATA_WIDTH:0] y_curr_ext = {y_curr[DATA_WIDTH-1], y_curr};
    wire signed [DATA_WIDTH:0] diff       = y_in_ext - y_curr_ext;
    
    // Multiplication: diff * Frac
    // 33 bits * 18 bits = 51 bits. We use 64 bits to guarantee safety.
    wire signed [63:0] mult = diff * $signed({1'b0, frac_32});


    always @(posedge clk) begin
        if (rst) begin
            y_curr       <= 0;
            current_time <= 32'd65536;    // Input grid starts at index 1.0
            target_time  <= START_TIME;   // Target grid starts at index 8.0
            vq_out       <= 0;
            vq_out_valid <= 0;
        end 
        else begin
            vq_out_valid <= 0; // Default

            if (y_in_valid) begin
                // Store previous sample safely
                y_curr <= y_in;

                // Check if the current input bounds the target grid point
                if (current_time >= target_time) begin
                    
                    // Linear Interpolation: Y_old + (Diff * Frac)
                    vq_out       <= y_curr + (mult >>> PHASE_WIDTH); // to adjust to q2.30 format
                    vq_out_valid <= 1;

                    // Advance target to the next query point
                    target_time  <= target_time + STEP_SIZE;
                end

                // Advance the input time by exactly 1.0
                current_time <= current_time + 32'd65536;
            end
        end
    end

endmodule
