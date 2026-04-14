module beamforming #(
    parameter DATA_WIDTH = 32,
    parameter APERTURE_SIZE = 50,
    parameter CALC_INST = 10
)(
    input  wire clk,
    input  wire reset,load_values, 
    input  wire [7:0] center_idx,
    input wire signed [DATA_WIDTH-1:0] input_line,
    output reg signed [DATA_WIDTH-1:0] final_pixel_val,
    output reg pixel_valid
);
wire signed [DATA_WIDTH-1:0] x_pixel;
wire signed [DATA_WIDTH-1:0] z_pixel;
wire signed [DATA_WIDTH-1:0] start_depth; 

// Input reception sampled
reg [1:0]  input_count = 0;
reg [31:0] x_pixel_reg, z_pixel_reg, start_depth_reg;

// Assign the internal wires to the registers
assign x_pixel     = x_pixel_reg;
assign z_pixel     = z_pixel_reg;
assign start_depth = start_depth_reg;
wire [CALC_INST-1:0] calc_valid;

    reg signed [DATA_WIDTH-1:0] element_x_rom [0:127];
    // synthesis translate off
    initial $readmemh("element_positions_128.mem", element_x_rom);
    // synthesis translate on
    reg [DATA_WIDTH-1:0] rf_addr_temp [0:CALC_INST-1];
    reg  signed [DATA_WIDTH-1:0]  sampled_data [0:CALC_INST-1];
wire reset_calc;
reg start_calc;
reg calc_valid_delayed;
reg [7:0] acc_valid_pipe;

always @(posedge clk) begin
    calc_valid_delayed <= calc_valid[0];
end

// This triggers on the rising edge

reg [2:0] state;
reg [2:0] batch_idx;  // 0 to 4
reg load_done;
wire last_batch_done = (batch_idx == 3'd4);
assign reset_calc = (~(calc_valid[0] & ~calc_valid_delayed));

localparam S_IDLE   = 0;
localparam S_LOAD   = 1;
localparam S_WAIT   = 2;
localparam S_NEXT   = 3;
localparam S_DONE   = 4;



always @(posedge clk or negedge reset) begin
    if (!reset) begin
        state <= S_IDLE;
        batch_idx <= 0;
        start_calc <= 0;
        load_done <= 0;
        x_pixel_reg <= 0;
        z_pixel_reg <= 0;
        start_depth_reg <= 0;
    end else begin
        case (state)
            S_IDLE: begin
                if (!load_done && !last_batch_done) begin
                    state <= S_LOAD;
                end else begin
                    start_calc <= 1;
                    state <= S_WAIT;
                end
            end

            S_LOAD: begin
            if (load_values) begin
                // Increment every time the testbench provides a new value
                input_count <= input_count + 1;

                case (input_count)
                    2'd0: x_pixel_reg     <= input_line;
                    2'd1: z_pixel_reg     <= input_line;
                    2'd2: start_depth_reg <= input_line;
                    2'd3: begin 
                        load_done <= 1; 
                        input_count <= 0;
                        state     <= S_IDLE; 
                    end
                endcase
            end
        end

            S_WAIT: begin
                start_calc <= 0;
                if (&calc_valid) begin
                    state <= S_NEXT;
                end
            end

            S_NEXT: begin
                if (last_batch_done) begin
                    state <= S_DONE;
                end else if(acc_valid_pipe[4]) begin
                    batch_idx <= batch_idx + 1;
                    start_calc <= 1;
                    state <= S_WAIT;
                end
            end

            S_DONE: begin

            end
            
            default: state <= S_IDLE;
        endcase
    end
end


genvar i;
    generate
        for (i = 0; i < CALC_INST; i = i + 1) begin : gen_cores
            
            (* ram_style = "block" *) reg [DATA_WIDTH-1:0] local_rf_mem [0:10239];
/*            initial begin
        // Fill the first few entries with garbage to "trick" the synthesizer/
       for (int j = 0; j < 1024; j++) begin
            local_rf_mem[j] = 16'hAAAA; 
        end
    end*/
            initial begin
               // synthesis translate_off
                $readmemh($sformatf("rf_ch_%0d.mem", i), local_rf_mem);
               // synthesis translate_on
            end
            wire [6:0] rom_addr = ((center_idx - 25 + batch_idx*CALC_INST + i) < 0)   ? 0 : 
                                  ((center_idx - 25 + batch_idx*CALC_INST + i) > 127) ? 127 : (center_idx - 25 + batch_idx*CALC_INST + i);
    
            Delay_calc delay_unit (
                .clk(clk),
                .reset(reset_calc), 
                .start_calc(start_calc), // From FSM
                .x_pixel(x_pixel),
                .z_pixel(z_pixel),
                .start_depth(start_depth),
                .x_elem(element_x_rom[rom_addr]),
                .rf_addr(rf_addr_temp[i]),
                .vld_out(calc_valid[i])
            );
            wire [DATA_WIDTH-1:0] current_addr = rf_addr_temp[i];

        // 2. Feed the BRAM immediately (Lowest Latency)
        always @(posedge clk) begin
            if (current_addr < 2048)
                // Use batch_idx * 4096 to point to the correct sensor's data in the 20k RAM
                sampled_data[i] <= local_rf_mem[current_addr + (batch_idx * 2048)];
            else
                sampled_data[i] <= 0;
        end
    end
endgenerate


    localparam [DATA_WIDTH-1:0] NORM_FACTOR = 32'd20972;
  reg [DATA_WIDTH+21:0] norm_result;


 (* dont_touch = "yes" *)   reg [DATA_WIDTH+31:0] norm_full_res; 

(* dont_touch = "yes" *)reg signed [DATA_WIDTH+4:0] batch_sum_stg1 [0:4];

(* dont_touch = "yes" *)reg signed [DATA_WIDTH+5:0] batch_sum_stg2 [0:2];

(* dont_touch = "yes" *)reg signed [DATA_WIDTH+6:0] current_batch_final; 

// --- 1. Balanced Adder Tree (3 Stages) ---
always @(posedge clk) begin
    // Stage 1: 1 cycle delay
    for (integer m=0; m<5; m=m+1) 
        batch_sum_stg1[m] <= $signed(sampled_data[2*m]) + $signed(sampled_data[2*m+1]);

    // Stage 2: 1 cycle delay
    batch_sum_stg2[0] <= $signed(batch_sum_stg1[0]) + $signed(batch_sum_stg1[1]);
    batch_sum_stg2[1] <= $signed(batch_sum_stg1[2]) + $signed(batch_sum_stg1[3]);
    // Added register here to balance the pipeline depth with the other two sums
    batch_sum_stg2[2] <= $signed(batch_sum_stg1[4]); 

    // Stage 3: 1 cycle delay
    current_batch_final <= $signed(batch_sum_stg2[0]) + $signed(batch_sum_stg2[1]) + $signed(batch_sum_stg2[2]);
end

// --- 2. The Accumulator with Edge Detection ---
(* dont_touch = "yes" *) reg signed [DATA_WIDTH+10:0] accumulator;
reg acc_valid_pipe_d; // Delayed bit for rising edge detection

always @(posedge clk or negedge reset) begin
    if (!reset) begin
        accumulator <= 0;
        final_pixel_val <= 0;
        acc_valid_pipe <= 0;
        acc_valid_pipe_d <= 0;
        norm_full_res <= 0;
    end else begin
        // Shift pipe: [0] is address valid, [1] is RAM out, [2-4] are adder stages
        acc_valid_pipe <= {acc_valid_pipe[6:0], &calc_valid};
        
        // Capture the state of the trigger bit from the previous cycle
        acc_valid_pipe_d <= acc_valid_pipe[3];

        // RISING EDGE DETECTOR: Trigger only when bit [4] goes from 0 to 1
        if (acc_valid_pipe[3] && !acc_valid_pipe_d) begin 
            if (batch_idx == 0) 
                accumulator <= $signed(current_batch_final); 
            else
                accumulator <= $signed(accumulator) + $signed(current_batch_final);
        end
     
        if (state == S_DONE && acc_valid_pipe[5]) begin
            norm_full_res <= $signed(accumulator) * $signed(NORM_FACTOR);
        end
        
        // Final output assignment
        if (state == S_DONE) begin
            final_pixel_val <= norm_full_res >>> 20;
        end
    end
end

// Validation signal - 8 clock cycles for adder to finish
    reg [7:0] valid_delay; 
    always @(posedge clk) begin
        if (!reset) 
            valid_delay <= 8'b0;
        else if (last_batch_done) 
            valid_delay <= {valid_delay[6:0], 1'b1};
            
        pixel_valid <= valid_delay[7];
    end
endmodule