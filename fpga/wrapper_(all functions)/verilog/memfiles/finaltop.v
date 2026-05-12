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
        test_bit <= test_bit >> 1;
        if (r >= trial_subtrahend) begin
            r <= r - trial_subtrahend;
            y <= y + test_bit;
        end
        i    <= i + 1;
        done <= (i == iterations);
    end else begin
        done <= 1'b0;   // ? NEW: clears vld_out after 1 clock
        busy <= 1'b0;   // ? NEW: allows CORDIC to restart for next query
    end
end

end


assign root_norm = y>>3;

assign sqrt = (m >= 0) ? (root_norm << m) : (root_norm >> (-m));


endmodule 


module Delay_calc #(
    parameter Q_TOTAL = 32,
    parameter Q_FRAC  = 26,
    parameter ADDR_W  = 11,
    parameter SQRT_LATENCY = 40,
    parameter MAX_ADDR = 2047,
    parameter reciprocal_ts_c = 3350
)(
    input  wire                   clk,start_calc,
    input  wire                   reset,
    input wire signed [31:0] x_pixel, z_pixel, x_elem, start_depth,   
    output reg  [ADDR_W-1:0]      rf_addr,
    output reg                   vld_out
);

    localparam D_WIDTH = (Q_TOTAL * 2);

    // --- STAGE 1 & 2 & 3: Pipelined Math ---
    reg signed [Q_TOTAL-1:0] dx;
    reg signed [D_WIDTH-1:0] dx_sq, dz_sq, sum_sq;
    
    reg [4:0] start_pipe; 

always @(posedge clk) begin
    if (!reset) begin
        start_pipe <= 5'b0;
    end else begin
        start_pipe <= {start_pipe[3:0], start_calc}; 
        
        // Data path calculations
        dx     <= $signed(x_elem) - $signed(x_pixel);
        dx_sq  <= dx * dx;
        dz_sq  <= z_pixel * z_pixel;
        sum_sq <= dx_sq + dz_sq;
    end
end

    // --- STAGE 4: CORDIC SQRT ---
    wire [Q_TOTAL-1:0] dist_q;

    cordic_sqrt #(
        .iterations(SQRT_LATENCY) 
    ) sqrt_unit (
        .clk(clk),          
        .start(start_pipe[4]),
        .reset(reset),
        .n(sum_sq),
        .sqrt(dist_q),
        .done(sqrt_vld)
    );

    reg signed [Q_TOTAL:0] sum_dist;

    always @(posedge clk) begin
        sum_dist <= $signed(z_pixel) + $signed(dist_q) - $signed(2*start_depth);
    end

    // --- STAGE 6: Final Multiply (Scale to Sample Index) ---
    reg [63:0] k_n_long;

    always @(posedge clk) begin
    if (!reset) begin
    k_n_long <= 0;
    end else begin
        k_n_long <= sum_dist * reciprocal_ts_c;
    end 
    end
    wire [63:0] k_n_rounded = k_n_long + (1 << (Q_FRAC));
    wire [10:0] raw_index;
    assign raw_index = (k_n_rounded>>Q_FRAC*2-19);

    

    always @(posedge clk) begin
        if (!reset) begin
            rf_addr <= 0;
        end else begin
            if (raw_index > MAX_ADDR) begin
                rf_addr <= MAX_ADDR;
            end
            else if (raw_index < 1) begin
                rf_addr <= 11'd1;
            end
            else begin
                rf_addr <= raw_index[ADDR_W-1:0];
            end
        end
    end

    // --- STAGE 8: Valid Signal Alignment ---
    reg [2:0] vld_delay_pipe;

    always @(posedge clk) begin
        if (!reset) vld_delay_pipe <= 3'b0;
        else        vld_delay_pipe <= {vld_delay_pipe[1:0], sqrt_vld};
                    vld_out <= vld_delay_pipe[2];
    end



endmodule


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
    initial $readmemh("element_positions.mem", element_x_rom);
    // synthesis translate on
    wire [DATA_WIDTH-1:0] rf_addr_temp [0:CALC_INST-1];
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
           
           (* dont_touch = "yes" *) (* ram_style = "block" *) reg [DATA_WIDTH-1:0] local_rf_mem [0:10239];
/*            initial begin
        // Fill the first few entries with garbage to "trick" the synthesizer/
       for (int j = 0; j < 1024; j++) begin
            local_rf_mem[j] = 16'hAAAA;
        end
    end*/
            initial begin
               // synthesis translate_off
                $readmemh($sformatf("rf_data_%0d.mem", i), local_rf_mem);
               // synthesis translate_on
            end
            wire [6:0] rom_addr = ((center_idx - (APERTURE_SIZE/2 + 1) + batch_idx*CALC_INST + i) < 0)   ? 0 :
                                  ((center_idx - (APERTURE_SIZE/2 + 1) + batch_idx*CALC_INST + i) > 127) ? 127 : (center_idx - (APERTURE_SIZE/2 + 1) + batch_idx*CALC_INST + i);
   
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
            wire [DATA_WIDTH-1:0] current_addr = (rf_addr_temp[i]);

        // 2. Feed the BRAM immediately (Lowest Latency)
        always @(posedge clk) begin
            if (current_addr < 2048)
                // Use batch_idx * 4096 to point to the correct sensor's data in the 20k RAM
                    sampled_data[i] <= local_rf_mem[(current_addr) + (batch_idx * 2048)];
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
    // Stage 1: 1 cycle delay (Loop explicitly unrolled)
    batch_sum_stg1[0] <= $signed(sampled_data[0]) + $signed(sampled_data[1]);
    batch_sum_stg1[1] <= $signed(sampled_data[2]) + $signed(sampled_data[3]);
    batch_sum_stg1[2] <= $signed(sampled_data[4]) + $signed(sampled_data[5]);
    batch_sum_stg1[3] <= $signed(sampled_data[6]) + $signed(sampled_data[7]);
    batch_sum_stg1[4] <= $signed(sampled_data[8]) + $signed(sampled_data[9]);

    // Stage 2: 1 cycle delay
    batch_sum_stg2[0] <= $signed(batch_sum_stg1[0]) + $signed(batch_sum_stg1[1]);
    batch_sum_stg2[1] <= $signed(batch_sum_stg1[2]) + $signed(batch_sum_stg1[3]);
   
    // Added register here to balance the pipeline depth with the other two sums
    batch_sum_stg2[2] <= $signed(batch_sum_stg1[4]);

    // Stage 3: 1 cycle delay
    // Note: This adds 3 operands, inferring two cascaded adders in this stage.
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


module volterra_quad #(
  parameter M = 15, //yy size
  parameter W = 32, //bitwidth of data in yy
  parameter IDX_W = 4, //bitwidth of indices in d{2}
  parameter NTERMS = 10 //no of terms in d{2}
) (
  input signed [M*W-1:0] yy_flat,   // Flattened input array
  input [NTERMS*IDX_W-1:0] d2_a_flat, //first column of d{2}
  input [NTERMS*IDX_W-1:0] d2_b_flat, //second column of d{2}
  output signed [NTERMS*(2*W)-1:0] prod_flat  // Flattened output array
);

  //internal wires which act as "unpacked" arrays
  wire signed [W-1:0] yy [0:M-1];
  wire [IDX_W-1:0] d2_a [0:NTERMS-1];
  wire [IDX_W-1:0] d2_b [0:NTERMS-1];
  
    // Force the use of DSP slices
    (* use_dsp = "yes" *) reg signed [2*W-1:0] prod2_terms [0:NTERMS-1];

    genvar j;
    generate
        //unpack the inputs
        for (j = 0; j < M; j = j + 1) begin : unpack_yy
            assign yy[j] = yy_flat[j*W +: W];
        end
        for (j = 0; j < NTERMS; j = j + 1) begin : unpack_indices
            assign d2_a[j] = d2_a_flat[j*IDX_W +: IDX_W];
            assign d2_b[j] = d2_b_flat[j*IDX_W +: IDX_W];
        end
        // Pack outputs
        for (j = 0; j < NTERMS; j = j + 1) begin : pack_out
            assign prod_flat[j*(2*W) +: (2*W)] = prod2_terms[j];
        end
    endgenerate

    integer i;
    always @(*) begin
        for (i = 0; i < NTERMS; i = i + 1) begin
            prod2_terms[i] = yy[d2_a[i]] * yy[d2_b[i]];
        end
    end

endmodule


module volterra_quad_wrapper #(
    parameter M = 15, 
    parameter W = 32, 
    parameter IDX_W = 4, 
    parameter NTERMS = 20 
)(
    input  wire clk,
    input  wire rst_n,
    
    // Control signals
    input wire start_load,
    input wire [W-1:0] data_in,
    output reg  ready,
    output reg  done,
    
    // Serialized output
    output reg [2*W-1:0] data_out
);

    reg signed [W-1:0] yy_mem [0:M-1];
    reg [IDX_W-1:0] d2_a_mem [0:NTERMS-1];
    reg [IDX_W-1:0] d2_b_mem [0:NTERMS-1];

    reg [M*W-1:0] yy_flat;
    reg [NTERMS*IDX_W-1:0] d2_a_flat;
    reg [NTERMS*IDX_W-1:0] d2_b_flat;

    wire [NTERMS*(2*W)-1:0] prod_flat_out;

    // FSM States
    localparam IDLE = 3'd0, LOAD_YY = 3'd1, LOAD_D2 = 3'd2, CALC = 3'd3, READOUT = 3'd4;

    reg [2:0] state;
    integer count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            ready <= 1'b0;
            done  <= 1'b0;
            count <= 0;
        end else begin
            case (state)
                IDLE: begin
                    done  <= 1'b0;
                    ready <= 1'b1;
                    if (start_load) begin
                        state <= LOAD_YY;
                        count <= 0;
                    end
                end

                LOAD_YY: begin
                    yy_mem[count] <= data_in;
                    if (count == M-1) begin
                        count <= 0;
                        state <= LOAD_D2;
                    end else begin
                        count <= count + 1;
                    end
                end

                LOAD_D2: begin
                    // Pack two indices into one data_in
                    d2_a_mem[count] <= data_in[IDX_W-1:0];
                    d2_b_mem[count] <= data_in[2*IDX_W-1:IDX_W];

                    if (count == NTERMS-1) begin
                        state <= CALC;
                        ready <= 1'b0;
                        count <= 0;
                    end else begin
                        count <= count + 1;
                    end
                end

                CALC: begin
                    state <= READOUT;
                    count <= 0;
                end

                READOUT: begin
                    data_out <= prod_flat_out[count*(2*W) +: (2*W)];

                    if (count == NTERMS-1) begin
                        done  <= 1'b1;
                        state <= IDLE;
                    end else begin
                        count <= count + 1;
                    end
                end

            endcase
        end
    end

    // Flattening 
    integer i;
    always @(*) begin
        for (i = 0; i < M; i = i + 1)
            yy_flat[i*W +: W] = yy_mem[i];

        for (i = 0; i < NTERMS; i = i + 1) begin
            d2_a_flat[i*IDX_W +: IDX_W] = d2_a_mem[i];
            d2_b_flat[i*IDX_W +: IDX_W] = d2_b_mem[i];
        end
    end

    // main module instantiation
    volterra_quad #(M, W, IDX_W, NTERMS) core_inst (
        .yy_flat(yy_flat),
        .d2_a_flat(d2_a_flat),
        .d2_b_flat(d2_b_flat),
        .prod_flat(prod_flat_out)
    );

endmodule


module volterra_cubic # (
    parameter M = 15, 
    parameter W = 32, 
    parameter IDX_W = 4, 
    parameter NTERMS = 10 
) (
    input clk,                       
    input signed [M*W-1:0] yy_flat,      
    input [NTERMS*IDX_W-1:0] d3_a_flat, 
    input [NTERMS*IDX_W-1:0] d3_b_flat, 
    input [NTERMS*IDX_W-1:0] d3_c_flat, 
    output reg signed [NTERMS*(3*W)-1:0] prod_flat
);

    wire signed [W-1:0] yy [0:M-1];
    wire [IDX_W-1:0] d3_a [0:NTERMS-1];
    wire [IDX_W-1:0] d3_b [0:NTERMS-1];
    wire [IDX_W-1:0] d3_c [0:NTERMS-1];
    
    // Force the use of DSP slices
    (* use_dsp = "yes" *) reg signed [3*W-1:0] prod3_terms [0:NTERMS-1];

    genvar i, j;
    generate
        for (i = 0; i < M; i = i + 1) begin : unpack_yy
            assign yy[i] = yy_flat[i*W +: W];
        end
        for (i = 0; i < NTERMS; i = i + 1) begin : unpack_indices
            assign d3_a[i] = d3_a_flat[i*IDX_W +: IDX_W];
            assign d3_b[i] = d3_b_flat[i*IDX_W +: IDX_W];
            assign d3_c[i] = d3_c_flat[i*IDX_W +: IDX_W];
        end
    endgenerate

    integer k;
    always @(posedge clk) begin
        for (k = 0; k < NTERMS; k = k + 1) begin
            prod3_terms[k] <= yy[d3_a[k]] * yy[d3_b[k]] * yy[d3_c[k]];
        end
    end

    integer p;
    always @(*) begin
        for (p = 0; p < NTERMS; p = p + 1) begin
            prod_flat[p*(3*W) +: (3*W)] = prod3_terms[p];
        end
    end

endmodule


module volterra_wrapper # (
    parameter M = 15, 
    parameter W = 32, 
    parameter IDX_W = 4, 
    parameter NTERMS = 10 
) (
    input  wire clk,
    input  wire rst_n,
    
    // Control Signals
    input  wire start_load,      // Pulse to start loading data
    input  wire [W-1:0] data_in, // Single port for all data
    output reg  ready,           // High when ready for next data_in
    output reg  done,            // High when calculation is finished
    
    // Output Port
    output reg [3*W-1:0] data_out
);

    // Internal storage to replace the huge flat inputs
    reg signed [W-1:0] yy_mem [0:M-1];
    reg [IDX_W-1:0] d3_a_mem [0:NTERMS-1];
    reg [IDX_W-1:0] d3_b_mem [0:NTERMS-1];
    reg [IDX_W-1:0] d3_c_mem [0:NTERMS-1];
    
    // Internal flat signals to connect to the original module
    reg [M*W-1:0] yy_flat;
    reg [NTERMS*IDX_W-1:0] d3_a_flat;
    reg [NTERMS*IDX_W-1:0] d3_b_flat;
    reg [NTERMS*IDX_W-1:0] d3_c_flat;
    wire [NTERMS*(3*W)-1:0] prod_flat_out;

    // FSM States
    localparam IDLE = 3'd0, LOAD_YY = 3'd1, LOAD_D3 = 3'd2, CALC = 3'd3, READOUT = 3'd4;

    reg [2:0] state;
    integer count;

    // FSM Logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            ready <= 1'b0;
            done  <= 1'b0;
            count <= 0;
        end else begin
            case (state)
                IDLE: begin
                    done  <= 1'b0;
                    ready <= 1'b1;
                    if (start_load) begin
                        state <= LOAD_YY;
                        count <= 0;
                    end
                end

                LOAD_YY: begin
                    yy_mem[count] <= data_in;
                    if (count == M-1) begin
                        count <= 0;
                        state <= LOAD_D3;
                    end else begin
                        count <= count + 1;
                    end
                end

                LOAD_D3: begin
                    // Here we pack the indices.
                    d3_a_mem[count] <= data_in[IDX_W-1:0];
                    d3_b_mem[count] <= data_in[2*IDX_W-1:IDX_W];
                    d3_c_mem[count] <= data_in[3*IDX_W-1:2*IDX_W];
                    
                    if (count == NTERMS-1) begin
                        state <= CALC;
                        ready <= 1'b0;
                        count <= 0;
                    end else begin
                        count <= count + 1;
                    end
                end

                CALC: begin
                    // Wait for the pipelined Volterra module to finish
                    if (count == 2) begin 
                        state <= READOUT;
                        count <= 0;
                    end else begin
                        count <= count + 1;
                    end
                end

                READOUT: begin
                    // Stream the results out one by one
                    data_out <= prod_flat_out[count*(3*W) +: (3*W)];
                    if (count == NTERMS-1) begin
                        done  <= 1'b1;
                        state <= IDLE;
                    end else begin
                        count <= count + 1;
                    end
                end
            endcase
        end
    end

    integer i;
    always @(*) begin
        for (i=0; i<M; i=i+1) yy_flat[i*W +: W] = yy_mem[i];
        for (i=0; i<NTERMS; i=i+1) begin
            d3_a_flat[i*IDX_W +: IDX_W] = d3_a_mem[i];
            d3_b_flat[i*IDX_W +: IDX_W] = d3_b_mem[i];
            d3_c_flat[i*IDX_W +: IDX_W] = d3_c_mem[i];
        end
    end

    // Instantiate Original Module
    volterra_cubic #(M, W, IDX_W, NTERMS) core_inst (
        .clk(clk),
        .yy_flat(yy_flat),
        .d3_a_flat(d3_a_flat),
        .d3_b_flat(d3_b_flat),
        .d3_c_flat(d3_c_flat),
        .prod_flat(prod_flat_out)
    );

endmodule


module dot_product_seq #(
    parameter N = 120,
    parameter W = 32
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,

    input  wire signed [N*W-1:0] a_flat,
    input  wire signed [N*W-1:0] b_flat,

    output reg signed [2*W+24:0] y,
    output reg done
);

    // Internal unpacked arrays
    wire signed [W-1:0] a [0:N-1];
    wire signed [W-1:0] b [0:N-1];

    genvar j;
    generate
        for (j = 0; j < N; j = j + 1) begin
            assign a[j] = a_flat[j*W +: W];
            assign b[j] = b_flat[j*W +: W];
        end
    endgenerate

    localparam IDLE = 2'd0,  // FSM states
               RUN  = 2'd1,
               DONE = 2'd2;

    reg [1:0] state;
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            y     <= 0;
            done  <= 0;
            i     <= 0;
        end else begin
            case (state)

                IDLE: begin
                    done <= 0;
                    if (start) begin
                        y <= 0;
                        i <= 0;
                        state <= RUN;
                    end
                end

                RUN: begin
                    y <= y + (a[i] * b[i]);

                    if (i == N-1) begin
                        state <= DONE;
                    end else begin
                        i <= i + 1;
                    end
                end

                DONE: begin
                    done <= 1;
                    state <= IDLE;
                end

            endcase
        end
    end

endmodule


// Wrapper Module
module dot_product_wrapper_seq #(
    parameter N = 120,
    parameter W = 32
)(
    input  wire clk,
    input  wire rst_n,

    input  wire start_load,
    input  wire [W-1:0] data_in,

    output reg  ready,
    output reg  done,
    output reg signed [2*W+24:0] data_out
);

    // Memory
    reg signed [W-1:0] a_mem [0:N-1];
    reg signed [W-1:0] b_mem [0:N-1];

    reg [N*W-1:0] a_flat;
    reg [N*W-1:0] b_flat;

    wire signed [2*W+16:0] y_out;
    wire core_done;

    reg core_start;

    localparam IDLE = 3'd0, LOAD_A = 3'd1, LOAD_B = 3'd2, START = 3'd3, WAIT = 3'd4;

    reg [2:0] state;
    integer count;

    // FSM
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            ready <= 0;
            done  <= 0;
            core_start <= 0;
            count <= 0;
        end else begin
            case (state)

                IDLE: begin
                    ready <= 1;
                    done  <= 0;

                    if (start_load) begin
                        count <= 0;
                        state <= LOAD_A;
                    end
                end

                LOAD_A: begin
                    a_mem[count] <= data_in;

                    if (count == N-1) begin
                        count <= 0;
                        state <= LOAD_B;
                    end else begin
                        count <= count + 1;
                    end
                end

                LOAD_B: begin
                    b_mem[count] <= data_in;

                    if (count == N-1) begin
                        state <= START;
                    end else begin
                        count <= count + 1;
                    end
                end

                START: begin
                    core_start <= 1;
                    ready <= 0;
                    state <= WAIT;
                end

                WAIT: begin
                    core_start <= 0;

                    if (core_done) begin
                        data_out <= y_out;
                        done <= 1;
                        state <= IDLE;
                    end
                end

            endcase
        end
    end

    // Flatten
    integer i;
    always @(*) begin
        for (i = 0; i < N; i = i + 1) begin
            a_flat[i*W +: W] = a_mem[i];
            b_flat[i*W +: W] = b_mem[i];
        end
    end

// calling main module
    dot_product_seq #(N, W) inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(core_start),
        .a_flat(a_flat),
        .b_flat(b_flat),
        .y(y_out),
        .done(core_done)
    );

endmodule


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



module custom_interpl #(
    parameter DATA_WIDTH = 32,   // Q2.30
    parameter PHASE_WIDTH = 16,
    parameter START_TIME  = 32'd524288,
    parameter STEP_SIZE   = 32'd524288
)
 (
    input  wire clk,
    input  wire rst,
    input  wire signed [DATA_WIDTH-1:0] y_in,
    input  wire y_in_valid,
    output reg  signed [DATA_WIDTH-1:0] vq_out,
    output reg  vq_out_valid
);
    // Step Size = T_new / T_orig = 32ns / 4ns = 8.0
    // In Q16 format: 8.0 * 65536 = 524288

    
    // First t_new point is 32ns, which corresponds to the 8th t_orig sample
    // In Q16 format: 8.0 * 65536 = 524288


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


module w1_matched_filter_wrapper #(
    parameter DATA_WIDTH  = 32,    // Q2.30 fixed-point internal width
    parameter ACCUM_WIDTH = 64,    // Accumulator width in matched_filter_1d
    parameter TAPS        = 136,   // FW length = 2 × 68 (68 zeros + 68 chirp samples)
    parameter L           = 1792,  // Axial samples per channel (Receive.endSample in MATLAB)
    parameter PAD_LEN     = 135,   // TAPS-1 leading zeros for causal linear convolution
    parameter TOTAL_IN    = 1927,  // L + PAD_LEN → full convolution length = 1927
    parameter NUM_CH      = 128,   // Number of receive channels (Trans.numelements)
    parameter OUT_SHIFT   = 28     // Shift accumulator to fit Q2.30 output: 64-28=Q2.30
)(
    input  wire        clk,
    input  wire        rst_n,      // Active-low reset

    // -------------------------------------------------------
    // 8-bit Serial Input (from ADC / DMA controller)
    // RF data: row-major order [ch0..ch127 for sample 0], [ch0..ch127 for sample 1]..
    // -------------------------------------------------------
    input  wire [7:0]  data_in,
    input  wire        data_valid,  // High for one cycle when data_in is valid byte
    input  wire        start,       // Pulse high to begin loading RF frame

    // -------------------------------------------------------
    // Status
    // -------------------------------------------------------
    output reg         w1_done,    // Pulses high for 1 cycle when x1, x2 BRAMs are ready
    output reg         w1_busy,    // High while processing

    // -------------------------------------------------------
    // Output BRAM Read Port (W2 reads filtered RF data)
    // Access: addr = channel * L + sample_index
    // Valid after w1_done goes high
    // -------------------------------------------------------
    input  wire [17:0] rd_addr,    // = ch[6:0] * 1792 + sample[10:0]  (max 128*1792=229376)
    output wire [31:0] rd_data_x1, // Fundamental filtered sample
    output wire [31:0] rd_data_x2  // Subharmonic filtered sample (feeds W2 → W3)
);

// ============================================================================
// Internal Parameters
// ============================================================================
localparam BRAM_DEPTH  = NUM_CH * L;   // 229376 entries
localparam IN_BRAM_D   = NUM_CH * L;

// FSM States
localparam S_IDLE       = 3'd0;
localparam S_LOAD_RF    = 3'd1;  // Receive 8-bit bytes → assemble 32-bit words → store in rf_in_bram
localparam S_PAD_ZEROS  = 3'd2;  // Feed PAD_LEN zeros to prime matched filter pipeline
localparam S_FILT_RUN   = 3'd3;  // Feed L RF samples for current channel through both filters
localparam S_FILT_FLUSH = 3'd4;  // Feed zeros until all L valid outputs are captured
localparam S_NEXT_CH    = 3'd5;  // Advance to next channel or finish
localparam S_DONE       = 3'd6;

// ============================================================================
// Byte-to-Word Assembler (8-bit → 32-bit, MSB first)
// ============================================================================
reg [31:0] word_buf;
reg [1:0]  byte_idx;
reg        word_valid;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        word_buf   <= 32'h0;
        byte_idx   <= 2'h0;
        word_valid <= 1'b0;
    end else begin
        word_valid <= 1'b0;
        if (data_valid) begin
            word_buf  <= {word_buf[23:0], data_in};  // Shift in MSB first
            byte_idx  <= byte_idx + 2'h1;
            if (byte_idx == 2'd3)
                word_valid <= 1'b1;                  // Full 32-bit word ready
        end
    end
end

// ============================================================================
// Input BRAM : 128 channels × 1792 samples (raw normalized RF data)
// Written during S_LOAD_RF; read during S_FILT_RUN
// MATLAB: x_norm = x / max(abs(x))  → Q2.30 representation
// ============================================================================
(* ram_style = "block" *) reg [DATA_WIDTH-1:0] rf_in_bram  [0:BRAM_DEPTH-1];

// Input BRAM write (S_LOAD_RF): row-major order ch0_s0, ch1_s0 .. ch127_s0, ch0_s1 ..
// Addressing: sample_index * NUM_CH + channel_index
reg [17:0] in_wr_addr;

always @(posedge clk) begin
    if (word_valid && w1_busy)
        rf_in_bram[in_wr_addr] <= word_buf;
end

// ============================================================================
// Output BRAMs : 128 channels × 1792 samples of filtered data
// rf_out_bram_x1 : fundamental (FW1)
// rf_out_bram_x2 : subharmonic (FW2) → feeds W2 → W3 Volterra
// Addressing: channel_index * L + sample_index
// ============================================================================
(* ram_style = "block" *) reg [DATA_WIDTH-1:0] rf_out_bram_x1 [0:BRAM_DEPTH-1];
(* ram_style = "block" *) reg [DATA_WIDTH-1:0] rf_out_bram_x2 [0:BRAM_DEPTH-1];

// Read ports (W2 accesses these)
assign rd_data_x1 = rf_out_bram_x1[rd_addr];
assign rd_data_x2 = rf_out_bram_x2[rd_addr];

// ============================================================================
// Matched Filter Instances
// Uses matched_filter_1d_rom (matched_filter_1d_rom.v) — the patched version
// of convolution.v that adds a ROM_FILE parameter and calls
//   initial $readmemh(ROM_FILE, coeffs);
// so that the coefficient array is actually loaded at elaboration time.
//
// The original matched_filter_1d (convolution.v) declares coeffs[] but never
// initialises it, leaving all taps at zero and producing zero output.
// Do NOT instantiate matched_filter_1d here; always use matched_filter_1d_rom.
//
// Both instances receive the SAME din / din_valid stream so a single RF sample
// is filtered through FW1 and FW2 in parallel each cycle.
//
// ROM files (generated by gen_fpga_roms.m):
//   fw1_coeffs.mem : 136 lines of 8 hex chars — FW1 = [zeros(68,1); ww1]  Q2.30
//   fw2_coeffs.mem : 136 lines of 8 hex chars — FW2 = [zeros(68,1); ww2]  Q2.30
// ============================================================================
reg  [DATA_WIDTH-1:0] mf_din;       // Shared input: both filters process same sample
reg                   mf_din_valid;
wire [ACCUM_WIDTH-1:0] mf1_dout, mf2_dout;
wire                   mf1_valid,   mf2_valid;

// FW1 — Fundamental chirp matched filter
// ROM_FILE resolves at elaboration; Vivado finds it in the project source dir.
matched_filter_1d_rom #(
    .DATA_WIDTH (DATA_WIDTH),
    .TAPS       (TAPS),
    .ACCUM_WIDTH(ACCUM_WIDTH),
    .ROW_LENGTH (TOTAL_IN),         // 1927 resets col_count so filter re-arms per channel
    .ROM_FILE   ("fw1_coeffs.mem")  // << FW1 taps loaded here
) mf_fw1 (
    .clk       (clk),
    .rst       (~rst_n),            // matched_filter_1d_rom uses active-high rst
    .din       (mf_din),
    .din_valid (mf_din_valid),
    .dout      (mf1_dout),
    .dout_valid(mf1_valid)
);

// FW2 — Subharmonic chirp matched filter
matched_filter_1d_rom #(
    .DATA_WIDTH (DATA_WIDTH),
    .TAPS       (TAPS),
    .ACCUM_WIDTH(ACCUM_WIDTH),
    .ROW_LENGTH (TOTAL_IN),
    .ROM_FILE   ("fw2_coeffs.mem")  // << FW2 taps loaded here
) mf_fw2 (
    .clk       (clk),
    .rst       (~rst_n),
    .din       (mf_din),
    .din_valid (mf_din_valid),
    .dout      (mf2_dout),
    .dout_valid(mf2_valid)
);

// ============================================================================
// Main FSM
// ============================================================================
reg [2:0]  state;
reg [6:0]  cur_ch;       // Current channel being filtered (0 to NUM_CH-1)
reg [10:0] pad_cnt;      // Zero-padding counter (0 to PAD_LEN-1)
reg [10:0] samp_cnt;     // Input sample counter per channel (0 to L-1)
reg [10:0] out_cnt;      // Output write counter per channel (0 to L-1)
reg [17:0] load_cnt;     // Total words loaded so far

// Write address for output BRAM: cur_ch * L + out_cnt
wire [17:0] out_wr_addr = (cur_ch * L[17:0]) + {7'b0, out_cnt};
// Read address for input BRAM: sample * NUM_CH + cur_ch (row-major input)
wire [17:0] in_rd_addr  = ({7'b0, samp_cnt} * NUM_CH[17:0]) + {11'b0, cur_ch};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state        <= S_IDLE;
        w1_done      <= 1'b0;
        w1_busy      <= 1'b0;
        cur_ch       <= 7'b0;
        pad_cnt      <= 11'b0;
        samp_cnt     <= 11'b0;
        out_cnt      <= 11'b0;
        in_wr_addr   <= 18'b0;
        load_cnt     <= 18'b0;
        mf_din       <= {DATA_WIDTH{1'b0}};
        mf_din_valid <= 1'b0;
    end else begin
        mf_din_valid <= 1'b0;   // Default: no new data
        w1_done      <= 1'b0;   // Default: pulse

        case (state)
            // ----------------------------------------------------------
            // IDLE: Wait for start pulse
            // ----------------------------------------------------------
            S_IDLE: begin
                w1_busy   <= 1'b0;
                if (start) begin
                    w1_busy    <= 1'b1;
                    in_wr_addr <= 18'b0;
                    load_cnt   <= 18'b0;
                    state      <= S_LOAD_RF;
                end
            end

            // ----------------------------------------------------------
            // LOAD_RF: Receive RF bytes, assemble 32-bit words, fill BRAM
            // RF is sent row-major: all 128 channels per axial sample
            // Total words = L × NUM_CH = 1792 × 128 = 229376
            // ----------------------------------------------------------
            S_LOAD_RF: begin
                if (word_valid) begin
                    // rf_in_bram written combinatorially by always block above
                    in_wr_addr <= in_wr_addr + 18'h1;
                    load_cnt   <= load_cnt   + 18'h1;
                    if (load_cnt == (L * NUM_CH) - 1) begin
                        // All RF samples loaded - begin filtering channel 0
                        cur_ch   <= 7'b0;
                        pad_cnt  <= 11'b0;
                        samp_cnt <= 11'b0;
                        out_cnt  <= 11'b0;
                        state    <= S_PAD_ZEROS;
                    end
                end
            end

            // ----------------------------------------------------------
            // PAD_ZEROS: Feed PAD_LEN = 135 zeros to prime FIR pipeline
            // Ensures matched filter output aligns to RF sample 0
            // MATLAB: FW2 = [zeros(1,68) ww2] handles this alignment
            // ----------------------------------------------------------
            S_PAD_ZEROS: begin
                mf_din       <= {DATA_WIDTH{1'b0}};
                mf_din_valid <= 1'b1;
                pad_cnt      <= pad_cnt + 11'h1;
                if (pad_cnt == PAD_LEN[10:0] - 1) begin
                    samp_cnt <= 11'b0;
                    out_cnt  <= 11'b0;
                    pad_cnt  <= 11'b0;
                    state    <= S_FILT_RUN;
                end
            end

            // ----------------------------------------------------------
            // FILT_RUN: Feed L RF samples for current channel
            // Reads from rf_in_bram (row-major) → drives both mf_fw1/fw2
            // ----------------------------------------------------------
            S_FILT_RUN: begin
                mf_din       <= rf_in_bram[in_rd_addr];
                mf_din_valid <= 1'b1;
                samp_cnt     <= samp_cnt + 11'h1;

                // Capture valid filter outputs (both filters driven same data,
                // so mf1_valid == mf2_valid in steady state)
                if (mf1_valid) begin
                    // Truncate 64-bit Q accumulator to 32-bit Q2.30:
                    // ACCUM holds Q(2+30).(30) product sum → shift right OUT_SHIFT
                    rf_out_bram_x1[out_wr_addr] <= mf1_dout[OUT_SHIFT +: DATA_WIDTH];
                    rf_out_bram_x2[out_wr_addr] <= mf2_dout[OUT_SHIFT +: DATA_WIDTH];
                    out_cnt <= out_cnt + 11'h1;
                end

                if (samp_cnt == L[10:0] - 1) begin
                    samp_cnt <= 11'b0;
                    state    <= S_FILT_FLUSH;
                end
            end

            // ----------------------------------------------------------
            // FILT_FLUSH: Feed zeros to drain pipeline until L outputs captured
            // Pipeline depth = TAPS stages, so need ~TAPS more cycles
            // ----------------------------------------------------------
            S_FILT_FLUSH: begin
                mf_din       <= {DATA_WIDTH{1'b0}};
                mf_din_valid <= 1'b1;

                if (mf1_valid && out_cnt < L[10:0]) begin
                    rf_out_bram_x1[out_wr_addr] <= mf1_dout[OUT_SHIFT +: DATA_WIDTH];
                    rf_out_bram_x2[out_wr_addr] <= mf2_dout[OUT_SHIFT +: DATA_WIDTH];
                    out_cnt <= out_cnt + 11'h1;
                end

                // All L outputs captured for this channel
                if (out_cnt == L[10:0] - 1)
                    state <= S_NEXT_CH;
            end

            // ----------------------------------------------------------
            // NEXT_CH: Advance to next channel or finish
            // ----------------------------------------------------------
            S_NEXT_CH: begin
                mf_din_valid <= 1'b0;  // Pause filter between channels
                if (cur_ch == NUM_CH[6:0] - 1) begin
                    state <= S_DONE;
                end else begin
                    cur_ch   <= cur_ch + 7'h1;
                    pad_cnt  <= 11'b0;
                    samp_cnt <= 11'b0;
                    out_cnt  <= 11'b0;
                    state    <= S_PAD_ZEROS; // Re-prime filter for next channel
                end
            end

            // ----------------------------------------------------------
            // DONE: Signal completion
            // ----------------------------------------------------------
            S_DONE: begin
                w1_done <= 1'b1;  // One-cycle pulse
                w1_busy <= 1'b0;
                state   <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule


module w2_beamform_wrapper #(
    parameter DATA_WIDTH    = 32,    // Q6.26 fixed-point (matching Delay_calc)
    parameter NUM_AXIAL     = 973,   // hh grid length: hand-coded at 26mm depth
    parameter NUM_LATERAL   = 128,   // ww grid = 128 element positions
    parameter L             = 1792,  // Max RF samples (Receive.endSample)
    parameter APERTURE_SIZE = 50,    // Neighbor_elements*2 (MATLAB: M=25 each side)
    parameter CALC_INST     = 10,    // Parallel Delay_calc instances in beamforming.sv
    // Q6.26 fixed-point constants (from MATLAB):
    // start_depth = 2.0265 mm → Q6.26: round(2.0265 * 2^26) = 136,710,701
    parameter [31:0] START_DEPTH_Q = 32'd136710701,
    // ts*c = 0.04928 mm/sample → reciprocal used in Delay_calc: reciprocal_ts_c = 3350
    // (already embedded in Delay_calc parameter, referenced here for documentation)
    parameter [31:0] RECIP_TS_C    = 32'd3350
)(
    input  wire        clk,
    input  wire        rst_n,

    // -------------------------------------------------------
    // W1 → W2 handshake
    // -------------------------------------------------------
    input  wire        w1_done,     // Start W2 when W1 signals completion
    output reg         w2_done,     // W2 finished beamforming full image
    output reg         w2_busy,

    // -------------------------------------------------------
    // W1 Output BRAM Read Interface (x2 filtered data)
    // W2 reads channel-by-channel to fill beamforming BRAMs
    // -------------------------------------------------------
    output reg  [17:0] w1_rd_addr,  // ch * L + sample (sent to w1 wrapper)
    input  wire [31:0] w1_rd_data,  // Filtered RF sample (x2)

    // -------------------------------------------------------
    // Output BRAM Read Port (W3 reads beamformed pixels)
    // addr = axial_idx * NUM_LATERAL + lateral_idx
    // -------------------------------------------------------
    input  wire [16:0] bf_rd_addr,  // max 973 * 128 = 124544
    output wire [31:0] bf_rd_data   // Beamformed pixel value
);

// ============================================================================
// Internal Parameters
// ============================================================================
localparam BF_BRAM_DEPTH = NUM_AXIAL * NUM_LATERAL;  // 124544

// RF buffer: stores x2 matched-filtered data for all 128 channels
// Organized as channel-major: ch0[0..L-1], ch1[0..L-1], ..., ch127[0..L-1]
// This mirrors how beamforming.sv reads its local_rf_mem
(* ram_style = "block" *) reg [DATA_WIDTH-1:0] rf_buf [0:(NUM_LATERAL*L)-1]; // 128×1792

// Beamformed output BRAM: [axial][lateral] → pixel value
(* ram_style = "block" *) reg [DATA_WIDTH-1:0] bf_out_bram [0:BF_BRAM_DEPTH-1];
assign bf_rd_data = bf_out_bram[bf_rd_addr];

// Axial pixel grid ROM (hh in MATLAB)
// hh[0] = start_depth, hh[N] = start_depth + N * ddz, hh[NUM_AXIAL-1] ≤ 26mm
(* rom_style = "block" *) reg [DATA_WIDTH-1:0] hh_rom [0:NUM_AXIAL-1];
initial $readmemh("hh_grid.mem", hh_rom);

// Lateral pixel grid ROM (ww in MATLAB = element x-positions)
(* rom_style = "block" *) reg [DATA_WIDTH-1:0] ww_rom [0:NUM_LATERAL-1];
initial $readmemh("ww_grid.mem", ww_rom);

// ============================================================================
// FSM States
// ============================================================================
localparam S_IDLE      = 4'd0;
localparam S_COPY_RF   = 4'd1;  // Copy W1 BRAM → local rf_buf (ch by ch)
localparam S_PIX_START = 4'd2;  // Set up pixel (ax, lat) and load values into BF module
localparam S_BF_LOAD0  = 4'd3;  // Load x_pixel into beamforming module
localparam S_BF_LOAD1  = 4'd4;  // Load z_pixel
localparam S_BF_LOAD2  = 4'd5;  // Load start_depth (3rd value)
localparam S_BF_WAIT   = 4'd6;  // Wait for pixel_valid from beamforming module
localparam S_PIX_STORE = 4'd7;  // Store pixel value in bf_out_bram
localparam S_PIX_NEXT  = 4'd8;  // Advance pixel counters
localparam S_DONE      = 4'd9;

reg [3:0]  state;
reg [9:0]  ax_idx;        // Axial pixel index (0 to NUM_AXIAL-1 = 972)
reg [6:0]  lat_idx;       // Lateral pixel index (0 to NUM_LATERAL-1 = 127)
reg [10:0] copy_samp;     // Sample counter for RF copy
reg [6:0]  copy_ch;       // Channel counter for RF copy
reg [1:0]  load_phase;    // Sub-state for loading 3 values into beamforming
reg        bf_valid_prev; // For edge detection on pixel_valid

// ============================================================================
// Beamforming Module Instantiation
// beamforming.sv processes one pixel at a time.
// Inputs: center_idx (which lateral pixel), x_pixel, z_pixel, start_depth
// Output: final_pixel_val (averaged DAS sum), pixel_valid
//
// NOTE: beamforming.sv reads local RF data from its internal local_rf_mem BRAMs.
// Those BRAMs are currently $readmemh-initialized (simulation only).
// For synthesis: the beamforming module needs modification to accept rf_buf here.
// A write-port approach is described in the code note below.
// For simulation validation: the rf_buf and bf module BRAMs hold same data.
// ============================================================================
reg  [DATA_WIDTH-1:0] bf_input_line;
reg                   bf_load_values;
wire [DATA_WIDTH-1:0] bf_pixel_val;
wire                  bf_pixel_valid;
reg                   bf_reset;

// Center element index for beamforming = lateral pixel index (0-127)
// MATLAB: beamform_fpga iterates i=1:128 → center_element_idx = i
beamforming #(
    .DATA_WIDTH   (DATA_WIDTH),
    .APERTURE_SIZE(APERTURE_SIZE),
    .CALC_INST    (CALC_INST)
) bf_inst (
    .clk            (clk),
    .reset          (bf_reset),
    .load_values    (bf_load_values),
    .center_idx     (lat_idx),        // Which column = center element for DAS
    .input_line     (bf_input_line),  // Carries x_pixel / z_pixel / start_depth
    .final_pixel_val(bf_pixel_val),
    .pixel_valid    (bf_pixel_valid)
);

// ============================================================================
// Main FSM
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state          <= S_IDLE;
        w2_done        <= 1'b0;
        w2_busy        <= 1'b0;
        ax_idx         <= 10'b0;
        lat_idx        <= 7'b0;
        copy_samp      <= 11'b0;
        copy_ch        <= 7'b0;
        load_phase     <= 2'b0;
        bf_load_values <= 1'b0;
        bf_reset       <= 1'b0;
        bf_input_line  <= {DATA_WIDTH{1'b0}};
        w1_rd_addr     <= 18'b0;
        bf_valid_prev  <= 1'b0;
    end else begin
        w2_done        <= 1'b0;
        bf_load_values <= 1'b0;
        bf_valid_prev  <= bf_pixel_valid;

        case (state)
            // ----------------------------------------------------------
            // IDLE: Wait for W1 to finish
            // ----------------------------------------------------------
            S_IDLE: begin
                w2_busy  <= 1'b0;
                bf_reset <= 1'b0;
                if (w1_done) begin
                    w2_busy   <= 1'b1;
                    copy_ch   <= 7'b0;
                    copy_samp <= 11'b0;
                    state     <= S_COPY_RF;
                end
            end

            // ----------------------------------------------------------
            // COPY_RF: Transfer x2 data from W1's BRAM into local rf_buf
            // Also feeds data into beamforming.sv's BRAM (synthesis note below)
            // Reads W1: addr = copy_ch * L + copy_samp
            // ----------------------------------------------------------
            S_COPY_RF: begin
                w1_rd_addr <= ({11'b0, copy_ch} * L[17:0]) + {7'b0, copy_samp};
                // One cycle read latency: write the PREVIOUS read result
                if (copy_samp > 0 || copy_ch > 0) begin
                    rf_buf[({11'b0, copy_ch} * L[17:0]) + {7'b0, copy_samp} - 1]
                        <= w1_rd_data;
                end

                if (copy_samp == L[10:0] - 1) begin
                    copy_samp <= 11'b0;
                    if (copy_ch == NUM_LATERAL[6:0] - 1) begin
                        // All channels copied → start beamforming
                        ax_idx  <= 10'b0;
                        lat_idx <= 7'b0;
                        bf_reset <= 1'b1;  // Hold reset until first pixel
                        state   <= S_PIX_START;
                    end else begin
                        copy_ch <= copy_ch + 7'h1;
                    end
                end else begin
                    copy_samp <= copy_samp + 11'h1;
                end
            end

            // ----------------------------------------------------------
            // PIX_START: Deassert reset, prepare to load pixel coordinates
            // x_pixel = ww_rom[lat_idx] (mm, Q6.26)
            // z_pixel = hh_rom[ax_idx]  (mm, Q6.26)
            // ----------------------------------------------------------
            S_PIX_START: begin
                bf_reset   <= 1'b1;
                load_phase <= 2'b0;
                state      <= S_BF_LOAD0;
            end

            // ----------------------------------------------------------
            // BF_LOAD0/1/2: Serial-load 3 values into beamforming FSM
            // beamforming.sv S_LOAD state expects: x_pixel, z_pixel, start_depth
            // via input_line with load_values asserted
            // ----------------------------------------------------------
            S_BF_LOAD0: begin
                bf_input_line  <= ww_rom[lat_idx];  // x_pixel (lateral position)
                bf_load_values <= 1'b1;
                state          <= S_BF_LOAD1;
            end

            S_BF_LOAD1: begin
                bf_input_line  <= hh_rom[ax_idx];   // z_pixel (axial depth)
                bf_load_values <= 1'b1;
                state          <= S_BF_LOAD2;
            end

            S_BF_LOAD2: begin
                bf_input_line  <= START_DEPTH_Q;    // start_depth (constant, Q6.26)
                bf_load_values <= 1'b1;
                state          <= S_BF_WAIT;
            end

            // ----------------------------------------------------------
            // BF_WAIT: Wait for beamforming module to compute pixel
            // pixel_valid pulses high when final_pixel_val is ready
            // ----------------------------------------------------------
            S_BF_WAIT: begin
                bf_load_values <= 1'b0;
                // Rising edge on pixel_valid → result ready
                if (bf_pixel_valid && !bf_valid_prev) begin
                    state <= S_PIX_STORE;
                end
            end

            // ----------------------------------------------------------
            // PIX_STORE: Write pixel to output BRAM
            // Address: ax_idx * NUM_LATERAL + lat_idx
            // MATLAB: bout(j,i) = pixel_sum / num_elements (averaged in BF module)
            // ----------------------------------------------------------
            S_PIX_STORE: begin
                bf_out_bram[ax_idx * NUM_LATERAL[16:0] + {10'b0, lat_idx}]
                    <= bf_pixel_val;
                state <= S_PIX_NEXT;
            end

            // ----------------------------------------------------------
            // PIX_NEXT: Advance to next pixel (lateral first, then axial)
            // ----------------------------------------------------------
            S_PIX_NEXT: begin
                bf_reset <= 1'b0;  // Brief reset between pixels to clear BF module state
                if (lat_idx == NUM_LATERAL[6:0] - 1) begin
                    lat_idx <= 7'b0;
                    if (ax_idx == NUM_AXIAL[9:0] - 1) begin
                        state <= S_DONE;
                    end else begin
                        ax_idx <= ax_idx + 10'h1;
                        state  <= S_PIX_START;
                    end
                end else begin
                    lat_idx <= lat_idx + 7'h1;
                    state   <= S_PIX_START;
                end
            end

            // ----------------------------------------------------------
            // DONE: Signal W3 to begin
            // ----------------------------------------------------------
            S_DONE: begin
                w2_done <= 1'b1;
                w2_busy <= 1'b0;
                state   <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
    end
end

// ============================================================================
// NOTE: beamforming.sv BRAM synthesis bridge
// ============================================================================
// For full synthesis, beamforming.sv's local_rf_mem (per CALC_INST) must receive
// x2 filtered data written by this wrapper, not $readmemh.
//
// Recommended modification to beamforming.sv - add write port:
//   input wire        rf_wr_en,
//   input wire [13:0] rf_wr_addr,   // channel_within_batch * L + sample
//   input wire [31:0] rf_wr_data,
//
// Then in the generate block:
//   always @(posedge clk)
//     if (rf_wr_en && (rf_wr_addr[13:11] == batch_idx))
//       local_rf_mem[rf_wr_addr[10:0]] <= rf_wr_data;
//
// This wrapper would then populate each BRAM during S_COPY_RF.
// For simulation with $readmemh, the current beamforming.sv works as-is.
// ============================================================================

// ============================================================================
// MATLAB ROM Generation (gen_fpga_roms.m):
// ============================================================================
// % Axial grid hh (973 values from start_depth to 26mm)
// L = 1792; start_depth = Receive(1).startDepth * w2mm;
// end_depth = Receive(1).endDepth * w2mm;
// ddz = (end_depth - start_depth) / (L-1);
// hh = start_depth : ddz : 26;   % 1×973 vector
// fid = fopen('hh_grid.mem','w');
// for k = 1:length(hh)
//   fprintf(fid, '%08X\n', typecast(int32(round(hh(k) * 2^26)),'uint32'));
// end; fclose(fid);
//
// % Lateral grid ww (128 element positions in mm)
// ww = Trans.ElementPos(:,1)' * w2mm;   % 1×128
// fid = fopen('ww_grid.mem','w');
// for k = 1:128
//   fprintf(fid, '%08X\n', typecast(int32(round(ww(k) * 2^26)),'uint32'));
// end; fclose(fid);
// ============================================================================

endmodule



// ============================================================================
// File        : w3_volterra_wrapper.v
// Description : W3 - Volterra Filter Wrapper
//               Implements MATLAB V_out() function (lines 290-317)
//
// MATLAB Equivalent:
//   z2_norm = V_out(x2_bf_norm, H_par_scaled)
//   env22 = abs(z2_norm(:,:,2))  ← quadratic Volterra output
//   env23 = abs(z2_norm(:,:,3))  ← cubic Volterra output
//
// Algorithm (per pixel, starting at axial index m=15):
//   1. xy  = flipud(x[i-m+1:i, :])          → custom_flipud_ff (flip_matrix.v)
//   2. yy  = xy[:, j]                        → m=15 element column vector
//   3. prod2 = prod(yy[d{2}], 2)             → volterra_quad (120 terms)
//   4. z2   = prod2' * h2                    → dot_product_seq (N=120)
//   5. prod3 = prod(yy[d{3}], 2)             → volterra_cubic (680 terms)
//   6. z3   = prod3' * h3                    → dot_product_seq (N=680)
//   7. env22[i,j] = |z2|, env23[i,j] = |z3|
//
// Kernel Indices (from Volterra_begins, m=15):
//   d{2}: 120 pairs   (nchoosek(1:15,2) = 105 + 15 diagonal = 120)
//   d{3}: 680 triples (nchoosek(1:15,3) = 455 + 210 cross + 15 diagonal = 680)
//
// Precomputed ROM Files Required (from MATLAB H_par_scaled after V_tune):
//   volterra_h2.mem     : 120 × 32-bit Q2.30 scaled kernel values
//   volterra_h3.mem     : 680 × 32-bit Q2.30 scaled kernel values
//   volterra_d2_a.mem   : 120 × 4-bit first index  of d{2} (values 0-14)
//   volterra_d2_b.mem   : 120 × 4-bit second index of d{2}
//   volterra_d3_a.mem   : 680 × 4-bit first  index of d{3}
//   volterra_d3_b.mem   : 680 × 4-bit second index of d{3}
//   volterra_d3_c.mem   : 680 × 4-bit third  index of d{3}
//
// Processing:
//   Sequential pixel-by-pixel (lat inner loop, ax outer loop)
//   For each pixel (ax, lat): run quad → dot2, cubic → dot3 in sequence
//   Total pixels = (NUM_AXIAL - M) × NUM_LATERAL = 958 × 128 = 122624
// ============================================================================

`timescale 1ns/1ps

module w3_volterra_wrapper #(
    parameter DATA_WIDTH    = 32,    // Q2.30 fixed-point
    parameter M             = 15,    // Volterra memory length (MATLAB: m = 15)
    parameter NTERMS_QUAD   = 120,   // l(2) in Volterra_begins(m=15)
    parameter NTERMS_CUBIC  = 680,   // l(3) in Volterra_begins(m=15)
    parameter IDX_W         = 4,     // Bit width for d{} indices (0-14 fits in 4 bits)
    parameter NUM_AXIAL     = 973,
    parameter NUM_LATERAL   = 128,
    parameter OUT_BRAM_D    = 973 * 128  // 124544
)(
    input  wire        clk,
    input  wire        rst_n,

    // -------------------------------------------------------
    // W2 → W3 handshake
    // -------------------------------------------------------
    input  wire        w2_done,
    output reg         w3_done,
    output reg         w3_busy,

    // -------------------------------------------------------
    // W2 Beamformed Output BRAM Read Interface
    // -------------------------------------------------------
    output reg  [16:0] bf_rd_addr,   // ax * NUM_LATERAL + lat
    input  wire [31:0] bf_rd_data,   // x2_bf[ax][lat]

    // -------------------------------------------------------
    // 8-bit Serial Output (Volterra envelope results)
    // Addresses valid after w3_done; streamed MSB first
    // -------------------------------------------------------
    input  wire        out_rd_en,
    input  wire [16:0] out_rd_addr,
    output wire [31:0] env22_out,    // Quadratic Volterra envelope
    output wire [31:0] env23_out     // Cubic Volterra envelope
);

// ============================================================================
// Precomputed Volterra Kernel ROMs
// Loaded once at initialization from MATLAB-generated .mem files
// ============================================================================

// h2 kernel: 120 × Q2.30 values (H_par_scaled.h2 from V_tune)
(* rom_style = "block" *) reg signed [DATA_WIDTH-1:0] h2_rom [0:NTERMS_QUAD-1];
initial $readmemh("volterra_h2.mem", h2_rom);

// h3 kernel: 680 × Q2.30 values (H_par_scaled.h3 from V_tune)
(* rom_style = "block" *) reg signed [DATA_WIDTH-1:0] h3_rom [0:NTERMS_CUBIC-1];
initial $readmemh("volterra_h3.mem", h3_rom);

// d{2} index pairs: 120 pairs of 4-bit indices (0-based, MATLAB uses 1-based)
(* rom_style = "distributed" *) reg [IDX_W-1:0] d2_a_rom [0:NTERMS_QUAD-1];
(* rom_style = "distributed" *) reg [IDX_W-1:0] d2_b_rom [0:NTERMS_QUAD-1];
initial $readmemh("volterra_d2_a.mem", d2_a_rom);
initial $readmemh("volterra_d2_b.mem", d2_b_rom);

// d{3} index triples: 680 triples of 4-bit indices
(* rom_style = "distributed" *) reg [IDX_W-1:0] d3_a_rom [0:NTERMS_CUBIC-1];
(* rom_style = "distributed" *) reg [IDX_W-1:0] d3_b_rom [0:NTERMS_CUBIC-1];
(* rom_style = "distributed" *) reg [IDX_W-1:0] d3_c_rom [0:NTERMS_CUBIC-1];
initial $readmemh("volterra_d3_a.mem", d3_a_rom);
initial $readmemh("volterra_d3_b.mem", d3_b_rom);
initial $readmemh("volterra_d3_c.mem", d3_c_rom);

// ============================================================================
// Output BRAMs: env22 and env23
// ============================================================================
(* ram_style = "block" *) reg [DATA_WIDTH-1:0] env22_bram [0:OUT_BRAM_D-1];
(* ram_style = "block" *) reg [DATA_WIDTH-1:0] env23_bram [0:OUT_BRAM_D-1];
assign env22_out = env22_bram[out_rd_addr];
assign env23_out = env23_bram[out_rd_addr];

// ============================================================================
// Sliding Window Buffer: custom_flipud_ff
// Stores the last M=15 axial samples for current lateral column
// Written as new beamformed rows arrive; read as yy vector for Volterra
// Implements MATLAB: xy = custom_flipud(x(i-(m:-1:1)+1,:))
// ============================================================================
reg        flip_wr_en;
reg [3:0]  flip_wr_row;   // Row within window (0 to M-1)
reg [6:0]  flip_wr_col;   // Current lateral column
reg signed [DATA_WIDTH-1:0] flip_data_in;

reg        flip_rd_en;
reg [3:0]  flip_rd_row;
reg [6:0]  flip_rd_col;
wire signed [DATA_WIDTH-1:0] flip_data_out;

// custom_flipud_ff: 15-row × 128-col flip buffer
// Stores a sliding window; read_addr reverses row order (flipud)
custom_flipud_ff #(
    .DW  (DATA_WIDTH),
    .ROWS(M),
    .COLS(NUM_LATERAL)
) flip_buf (
    .clk      (clk),
    .rst_n    (rst_n),
    .write_en (flip_wr_en),
    .write_row(flip_wr_row),
    .write_col(flip_wr_col),
    .data_in  (flip_data_in),
    .read_en  (flip_rd_en),
    .read_row (flip_rd_row),
    .read_col (flip_rd_col),
    .data_out (flip_data_out)
);

// ============================================================================
// yy Vector Buffer: holds M=15 samples extracted from flip buffer for one column
// Loaded element-by-element before Volterra computation begins
// ============================================================================
reg signed [DATA_WIDTH-1:0] yy_mem [0:M-1];
reg [3:0] yy_load_cnt;   // Load counter (0 to M-1)

// Flattened yy for Volterra product modules
reg [M*DATA_WIDTH-1:0] yy_flat;
always @(*) begin : pack_yy
    integer k;
    for (k = 0; k < M; k = k + 1)
        yy_flat[k*DATA_WIDTH +: DATA_WIDTH] = yy_mem[k];
end

// ============================================================================
// d{2} and d{3} Index Flattened Buses
// Pre-computed from ROM, assembled once during INIT state
// ============================================================================
reg [NTERMS_QUAD*IDX_W-1:0]  d2_a_flat, d2_b_flat;
reg [NTERMS_CUBIC*IDX_W-1:0] d3_a_flat, d3_b_flat, d3_c_flat;

// Assemble flat buses from ROM arrays (synthesis: these are just wire concatenations)
always @(*) begin : pack_d
    integer k;
    for (k = 0; k < NTERMS_QUAD;  k = k + 1) begin
        d2_a_flat[k*IDX_W +: IDX_W] = d2_a_rom[k];
        d2_b_flat[k*IDX_W +: IDX_W] = d2_b_rom[k];
    end
    for (k = 0; k < NTERMS_CUBIC; k = k + 1) begin
        d3_a_flat[k*IDX_W +: IDX_W] = d3_a_rom[k];
        d3_b_flat[k*IDX_W +: IDX_W] = d3_b_rom[k];
        d3_c_flat[k*IDX_W +: IDX_W] = d3_c_rom[k];
    end
end

// ============================================================================
// Volterra Quadratic Products Module (volterra_quad)
// Computes: prod2_terms[k] = yy[d2_a[k]] * yy[d2_b[k]] for k=0..119
// 120 pair-products, purely combinational
// ============================================================================
wire [NTERMS_QUAD*(2*DATA_WIDTH)-1:0] prod2_flat;

volterra_quad #(
    .M     (M),
    .W     (DATA_WIDTH),
    .IDX_W (IDX_W),
    .NTERMS(NTERMS_QUAD)
) quad_inst (
    .yy_flat  (yy_flat),
    .d2_a_flat(d2_a_flat),
    .d2_b_flat(d2_b_flat),
    .prod_flat(prod2_flat)
);

// ============================================================================
// Volterra Cubic Products Module (volterra_cubic)
// Computes: prod3_terms[k] = yy[d3_a[k]] * yy[d3_b[k]] * yy[d3_c[k]] for k=0..679
// 680 triple-products, registered (1 cycle latency)
// ============================================================================
wire [NTERMS_CUBIC*(3*DATA_WIDTH)-1:0] prod3_flat;

volterra_cubic #(
    .M     (M),
    .W     (DATA_WIDTH),
    .IDX_W (IDX_W),
    .NTERMS(NTERMS_CUBIC)
) cubic_inst (
    .clk      (clk),
    .yy_flat  (yy_flat),
    .d3_a_flat(d3_a_flat),
    .d3_b_flat(d3_b_flat),
    .d3_c_flat(d3_c_flat),
    .prod_flat(prod3_flat)
);

// ============================================================================
// Dot Product Engines
// z2 = prod2_terms' * h2  (N=120, 64-bit result)
// z3 = prod3_terms' * h3  (N=680, 64-bit result)
//
// We drive them via a shared data_in bus, reusing dot_product_wrapper_seq.
// a_mem = product terms (loaded from Volterra output)
// b_mem = kernel coefficients h2/h3 (loaded from ROM once per pixel)
//
// Because N=680 is large, we use dot_product_seq directly for efficiency.
// The dot product accumulates sequentially (1 multiply-add per cycle).
// ============================================================================

// --- h2 flat bus: 120 × 32-bit (from ROM) ---
reg [NTERMS_QUAD*DATA_WIDTH-1:0] h2_flat;
always @(*) begin : pack_h2
    integer k;
    for (k = 0; k < NTERMS_QUAD; k = k + 1)
        h2_flat[k*DATA_WIDTH +: DATA_WIDTH] = h2_rom[k];
end

// --- h3 flat bus: 680 × 32-bit (from ROM) ---
reg [NTERMS_CUBIC*DATA_WIDTH-1:0] h3_flat;
always @(*) begin : pack_h3
    integer k;
    for (k = 0; k < NTERMS_CUBIC; k = k + 1)
        h3_flat[k*DATA_WIDTH +: DATA_WIDTH] = h3_rom[k];
end

// prod2 flat as 32-bit (take upper 32 of 64-bit products, Q-aligned)
// prod2_flat has NTERMS_QUAD × 64-bit entries → take upper 32 bits of each
reg [NTERMS_QUAD*DATA_WIDTH-1:0]  prod2_32;
reg [NTERMS_CUBIC*DATA_WIDTH-1:0] prod3_32;

always @(*) begin : truncate_products
    integer k;
    // Truncate 64-bit Q4.60 products to Q2.30 by taking bits [59:28]
    for (k = 0; k < NTERMS_QUAD; k = k + 1)
        prod2_32[k*DATA_WIDTH +: DATA_WIDTH] = prod2_flat[k*(2*DATA_WIDTH)+28 +: DATA_WIDTH];
    // Truncate 96-bit Q6.90 products to Q2.30 by taking bits [89:58]
    for (k = 0; k < NTERMS_CUBIC; k = k + 1)
        prod3_32[k*DATA_WIDTH +: DATA_WIDTH] = prod3_flat[k*(3*DATA_WIDTH)+58 +: DATA_WIDTH];
end

// Dot product: z2 = prod2_32' * h2_flat (N=120)
reg  dp2_start;
wire [(2*DATA_WIDTH+24):0] dp2_result;
wire dp2_done;

dot_product_seq #(
    .N(NTERMS_QUAD),
    .W(DATA_WIDTH)
) dp2_inst (
    .clk   (clk),
    .rst_n (rst_n),
    .start (dp2_start),
    .a_flat(prod2_32),
    .b_flat(h2_flat),
    .y     (dp2_result),
    .done  (dp2_done)
);

// Dot product: z3 = prod3_32' * h3_flat (N=680)
reg  dp3_start;
wire [(2*DATA_WIDTH+24):0] dp3_result;
wire dp3_done;

dot_product_seq #(
    .N(NTERMS_CUBIC),
    .W(DATA_WIDTH)
) dp3_inst (
    .clk   (clk),
    .rst_n (rst_n),
    .start (dp3_start),
    .a_flat(prod3_32),
    .b_flat(h3_flat),
    .y     (dp3_result),
    .done  (dp3_done)
);

// ============================================================================
// FSM States
// ============================================================================
localparam S_IDLE        = 4'd0;
localparam S_FILL_WIN    = 4'd1;  // Fill initial window with first M rows
localparam S_LOAD_YY     = 4'd2;  // Read M values from flip buffer → yy_mem
localparam S_START_QUAD  = 4'd3;  // Trigger quad products + dp2
localparam S_WAIT_QUAD   = 4'd4;  // Wait for dp2 (N=120 cycles)
localparam S_START_CUBIC = 4'd5;  // Trigger cubic products + dp3
localparam S_WAIT_CUBIC  = 4'd6;  // Wait for dp3 (N=680 cycles + 1 cubic latency)
localparam S_STORE_PIX   = 4'd7;  // Store abs(z2), abs(z3)
localparam S_NEXT_LAT    = 4'd8;  // Advance lateral; if done, advance axial
localparam S_ADVANCE_AX  = 4'd9;  // Add new beamformed row to flip buffer
localparam S_DONE        = 4'd10;

reg [3:0]  state;
reg [9:0]  ax_idx;        // Axial pixel index (M to NUM_AXIAL-1)
reg [6:0]  lat_idx;       // Lateral pixel index (0 to NUM_LATERAL-1)
reg [3:0]  win_row;       // Current write row in flip buffer (circular, 0..M-1)
reg [3:0]  yy_rd_cnt;     // Counter for reading yy from flip buffer
reg        cubic_latency; // Extra cycle for cubic module registered output
reg signed [DATA_WIDTH-1:0] z2_val, z3_val;

// Circular row index in flip buffer (overwrites oldest row)
reg [3:0]  circ_row;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state        <= S_IDLE;
        w3_done      <= 1'b0;
        w3_busy      <= 1'b0;
        ax_idx       <= 10'b0;
        lat_idx      <= 7'b0;
        win_row      <= 4'b0;
        yy_rd_cnt    <= 4'b0;
        circ_row     <= 4'b0;
        dp2_start    <= 1'b0;
        dp3_start    <= 1'b0;
        flip_wr_en   <= 1'b0;
        flip_rd_en   <= 1'b0;
        yy_load_cnt  <= 4'b0;
        cubic_latency<= 1'b0;
        z2_val       <= {DATA_WIDTH{1'b0}};
        z3_val       <= {DATA_WIDTH{1'b0}};
        bf_rd_addr   <= 17'b0;
    end else begin
        w3_done    <= 1'b0;
        dp2_start  <= 1'b0;
        dp3_start  <= 1'b0;
        flip_wr_en <= 1'b0;
        flip_rd_en <= 1'b0;

        case (state)
            // ----------------------------------------------------------
            // IDLE: Wait for W2 to finish
            // ----------------------------------------------------------
            S_IDLE: begin
                w3_busy <= 1'b0;
                if (w2_done) begin
                    w3_busy  <= 1'b1;
                    ax_idx   <= 10'd0;
                    lat_idx  <= 7'd0;
                    circ_row <= 4'd0;
                    state    <= S_FILL_WIN;
                end
            end

            // ----------------------------------------------------------
            // FILL_WIN: Load first M rows into the flip buffer
            // For each of the M rows, write all NUM_LATERAL columns
            // MATLAB: xy = x(i-m+1:i, :) for i=m=15 (first valid pixel)
            // ----------------------------------------------------------
            S_FILL_WIN: begin
                // Read from BF BRAM: ax_idx * NUM_LATERAL + lat_idx
                bf_rd_addr <= {ax_idx, 7'b0} + {10'b0, lat_idx};

                // One cycle read latency: write previous result
                if (ax_idx > 0 || lat_idx > 0) begin
                    flip_wr_en   <= 1'b1;
                    flip_wr_row  <= ax_idx[3:0];   // row within M-window
                    flip_wr_col  <= lat_idx - (lat_idx > 0 ? 7'd0 : 7'd0);
                    // Correct addressing: use previous (ax, lat)
                    flip_data_in <= bf_rd_data;
                end

                if (lat_idx == NUM_LATERAL[6:0] - 1) begin
                    lat_idx <= 7'b0;
                    if (ax_idx == M[9:0] - 1) begin
                        // Window filled with M rows, start processing
                        ax_idx  <= M[9:0];          // First valid Volterra output at ax=M
                        lat_idx <= 7'b0;
                        state   <= S_LOAD_YY;
                    end else begin
                        ax_idx <= ax_idx + 10'h1;
                    end
                end else begin
                    lat_idx <= lat_idx + 7'h1;
                end
            end

            // ----------------------------------------------------------
            // LOAD_YY: Extract column lat_idx from flip buffer → yy_mem[0..M-1]
            // flip_buf reverses row order automatically (flipud)
            // MATLAB: yy = xy(:,j) where xy = custom_flipud(x[i-M+1:i,:])
            // ----------------------------------------------------------
            S_LOAD_YY: begin
                flip_rd_en  <= 1'b1;
                flip_rd_row <= yy_rd_cnt;
                flip_rd_col <= lat_idx;

                // One cycle read latency: store previous result
                if (yy_rd_cnt > 0)
                    yy_mem[yy_rd_cnt - 1] <= flip_data_out;

                if (yy_rd_cnt == M[3:0]) begin
                    // Last entry (yy_rd_cnt rolled over or reached M)
                    yy_mem[M-1] <= flip_data_out;
                    yy_rd_cnt   <= 4'b0;
                    state       <= S_START_QUAD;
                end else begin
                    yy_rd_cnt <= yy_rd_cnt + 4'h1;
                end
            end

            // ----------------------------------------------------------
            // START_QUAD: Trigger quadratic products and dot product
            // volterra_quad is combinational, dp2 starts accumulating
            // MATLAB: prod2_terms = prod(yy(d{2}),2); z2 = prod2_terms' * h2
            // ----------------------------------------------------------
            S_START_QUAD: begin
                dp2_start <= 1'b1;  // Start dot_product_seq for z2
                state     <= S_WAIT_QUAD;
            end

            // ----------------------------------------------------------
            // WAIT_QUAD: Wait N=120 cycles for dp2 to finish
            // ----------------------------------------------------------
            S_WAIT_QUAD: begin
                if (dp2_done) begin
                    // Capture z2: take Q2.30 portion of the large accumulator
                    // dp2_result is (2*32+24+1)=89 bits; shift to recover Q2.30
                    z2_val <= dp2_result[58:27];  // bits [58:27] = Q2.30 range
                    state  <= S_START_CUBIC;
                end
            end

            // ----------------------------------------------------------
            // START_CUBIC: Trigger cubic products and dot product
            // volterra_cubic has 1-cycle registered output; dp3 follows
            // MATLAB: prod3_terms = prod(yy(d{3}),2); z3 = prod3_terms' * h3
            // ----------------------------------------------------------
            S_START_CUBIC: begin
                cubic_latency <= 1'b1;   // Wait one extra cycle for cubic reg output
                state         <= S_WAIT_CUBIC;
            end

            S_WAIT_CUBIC: begin
                if (cubic_latency) begin
                    cubic_latency <= 1'b0;
                    dp3_start     <= 1'b1;  // Start dot_product_seq for z3 after cubic settles
                end
                if (dp3_done) begin
                    z3_val <= dp3_result[58:27];  // Q2.30 portion
                    state  <= S_STORE_PIX;
                end
            end

            // ----------------------------------------------------------
            // STORE_PIX: Write abs(z2) and abs(z3) to output BRAMs
            // MATLAB: env22 = abs(z2_norm); env23 = abs(z3_norm)
            // addr = ax_idx * NUM_LATERAL + lat_idx
            // ----------------------------------------------------------
            S_STORE_PIX: begin
                begin
                    reg [16:0] out_addr;
                    out_addr = {ax_idx[9:0], 7'b0} + {10'b0, lat_idx};
                    // abs() in two's complement: if sign bit set, negate
                    env22_bram[out_addr] <= z2_val[DATA_WIDTH-1] ? (-z2_val) : z2_val;
                    env23_bram[out_addr] <= z3_val[DATA_WIDTH-1] ? (-z3_val) : z3_val;
                end
                state <= S_NEXT_LAT;
            end

            // ----------------------------------------------------------
            // NEXT_LAT: Advance to next lateral pixel
            // When all lateral pixels done, advance axial + update flip buffer
            // ----------------------------------------------------------
            S_NEXT_LAT: begin
                if (lat_idx == NUM_LATERAL[6:0] - 1) begin
                    lat_idx <= 7'b0;
                    state   <= S_ADVANCE_AX;
                end else begin
                    lat_idx  <= lat_idx + 7'h1;
                    yy_rd_cnt <= 4'b0;
                    state    <= S_LOAD_YY;
                end
            end

            // ----------------------------------------------------------
            // ADVANCE_AX: Load new beamformed row into flip buffer
            // Overwrites oldest row (circular buffer, circ_row)
            // Then move to next axial depth, or finish
            // ----------------------------------------------------------
            S_ADVANCE_AX: begin
                // Read new row from BF BRAM and write into flip buffer
                bf_rd_addr <= {ax_idx, 7'b0} + {10'b0, lat_idx};

                if (lat_idx < NUM_LATERAL[6:0]) begin
                    if (lat_idx > 0) begin
                        flip_wr_en   <= 1'b1;
                        flip_wr_row  <= circ_row;
                        flip_wr_col  <= lat_idx - 7'h1;
                        flip_data_in <= bf_rd_data;
                    end
                    lat_idx <= lat_idx + 7'h1;
                end else begin
                    // Finish writing last column
                    flip_wr_en   <= 1'b1;
                    flip_wr_row  <= circ_row;
                    flip_wr_col  <= NUM_LATERAL[6:0] - 7'h1;
                    flip_data_in <= bf_rd_data;
                    // Advance circular buffer pointer
                    circ_row <= (circ_row == M[3:0]-1) ? 4'b0 : circ_row + 4'h1;
                    lat_idx  <= 7'b0;
                    ax_idx   <= ax_idx + 10'h1;

                    if (ax_idx == NUM_AXIAL[9:0] - 1) begin
                        state <= S_DONE;
                    end else begin
                        yy_rd_cnt <= 4'b0;
                        state     <= S_LOAD_YY;
                    end
                end
            end

            // ----------------------------------------------------------
            // DONE
            // ----------------------------------------------------------
            S_DONE: begin
                w3_done <= 1'b1;
                w3_busy <= 1'b0;
                state   <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule

// ============================================================================
// MATLAB ROM Generation Snippet (append to gen_fpga_roms.m):
// ============================================================================
// % ---- Generate Volterra kernel and index ROM files ----
// % After running V_tune() to get H_par, H_max, H_par_scaled:
//
// Q = 30; scale = 2^Q;
// reborn = Volterra_begins(15);   % m=15
//
// % Kernel files
// h2s = H_par_scaled.h2;  h3s = H_par_scaled.h3;
// fid = fopen('volterra_h2.mem','w');
// for k=1:120, fprintf(fid,'%08X\n',typecast(int32(round(h2s(k)*scale)),'uint32')); end
// fclose(fid);
// fid = fopen('volterra_h3.mem','w');
// for k=1:680, fprintf(fid,'%08X\n',typecast(int32(round(h3s(k)*scale)),'uint32')); end
// fclose(fid);
//
// % Index files (convert 1-based MATLAB to 0-based Verilog)
// d2 = reborn.d{2};  d3 = reborn.d{3};
// fid_a = fopen('volterra_d2_a.mem','w'); fid_b = fopen('volterra_d2_b.mem','w');
// for k=1:120
//   fprintf(fid_a,'%01X\n', d2(k,1)-1);
//   fprintf(fid_b,'%01X\n', d2(k,2)-1);
// end; fclose(fid_a); fclose(fid_b);
//
// fid_a=fopen('volterra_d3_a.mem','w'); fid_b=fopen('volterra_d3_b.mem','w');
// fid_c=fopen('volterra_d3_c.mem','w');
// for k=1:680
//   fprintf(fid_a,'%01X\n', d3(k,1)-1);
//   fprintf(fid_b,'%01X\n', d3(k,2)-1);
//   fprintf(fid_c,'%01X\n', d3(k,3)-1);
// end; fclose(fid_a); fclose(fid_b); fclose(fid_c);
// ============================================================================



module volterra_top #(
    parameter DATA_WIDTH   = 32,
    parameter NUM_CH       = 128,
    parameter L            = 1792,
    parameter NUM_AXIAL    = 973,
    parameter NUM_LATERAL  = 128,
    parameter TAPS         = 136,
    parameter M_VOLTERRA   = 15,
    parameter NTERMS_QUAD  = 120,
    parameter NTERMS_CUBIC = 680,
    parameter ACCUM_WIDTH  = 64
)(
    input  wire        clk,
    input  wire        rst_n,         // Active-low global reset

    // -------------------------------------------------------
    // 8-bit RF Data Input
    // -------------------------------------------------------
    input  wire [7:0]  data_in,       // Serial RF byte (MSB of 32-bit word first)
    input  wire        data_valid,    // Byte valid strobe
    input  wire        start,         // Pulse high to begin processing a new frame

    // -------------------------------------------------------
    // Pipeline Status
    // -------------------------------------------------------
    output wire        pipeline_busy, // High during entire pipeline execution
    output wire        w1_done,       // Matched filtering complete
    output wire        w2_done,       // Beamforming complete
    output wire        w3_done,       // Volterra filtering complete (results ready)

    // -------------------------------------------------------
    // 32-bit Result Readback Port
    // Address: ax_idx * 128 + lat_idx (max 973*128-1 = 124543)
    // env22_out: quadratic Volterra envelope  (MATLAB: env22)
    // env23_out: cubic     Volterra envelope  (MATLAB: env23)
    // -------------------------------------------------------
    input  wire        out_rd_en,
    input  wire [16:0] out_rd_addr,   // {ax_idx[9:0], lat_idx[6:0]}
    output wire [31:0] env22_out,
    output wire [31:0] env23_out,

    // -------------------------------------------------------
    // Optional: Read fundamental/subharmonic envelopes from W1
    // (x1 and x2 after matched filtering, before beamforming)
    // addr = ch[6:0] * 1792 + sample[10:0]
    // -------------------------------------------------------
    input  wire [17:0] mf_rd_addr,
    output wire [31:0] mf_x1_out,
    output wire [31:0] mf_x2_out
);

// ============================================================================
// Internal Wires
// ============================================================================

// W1 → W2 data path
wire [17:0] w1_rd_addr_from_w2; // W2 requests specific address from W1's output BRAM
wire [31:0] w1_rd_data_x2;      // x2 filtered sample returned to W2
wire [31:0] w1_rd_data_x1;      // x1 filtered sample (for optional readback)

// W2 → W3 data path
wire [16:0] bf_rd_addr_from_w3; // W3 requests specific beamformed pixel
wire [31:0] bf_rd_data_to_w3;   // Beamformed pixel value returned to W3

// Internal done signals (some re-exported as outputs)
wire w1_done_int, w2_done_int, w3_done_int;
assign w1_done = w1_done_int;
assign w2_done = w2_done_int;
assign w3_done = w3_done_int;

// Busy signals
wire w1_busy, w2_busy, w3_busy;
assign pipeline_busy = w1_busy | w2_busy | w3_busy;

// ============================================================================
// W1: Matched Filter Wrapper
// MATLAB: x1_conv = custom_conv(x_norm', FW1)  [lines 95-98]
//         x2_conv = custom_conv(x_norm', FW2)  [lines 102-103]
// Interpolation (custom_interp1) is precomputed into fw1/fw2_coeffs.mem
// ============================================================================
w1_matched_filter_wrapper #(
    .DATA_WIDTH (DATA_WIDTH),
    .ACCUM_WIDTH(ACCUM_WIDTH),
    .TAPS       (TAPS),
    .L          (L),
    .PAD_LEN    (TAPS - 1),           // 135
    .TOTAL_IN   (L + TAPS - 1),       // 1927
    .NUM_CH     (NUM_CH)
) w1_inst (
    .clk         (clk),
    .rst_n       (rst_n),
    .data_in     (data_in),
    .data_valid  (data_valid),
    .start       (start),
    .w1_done     (w1_done_int),
    .w1_busy     (w1_busy),
    // Output BRAM port (read by W2 and external readback)
    .rd_addr     (w2_done_int ? mf_rd_addr : w1_rd_addr_from_w2),
    .rd_data_x1  (w1_rd_data_x1),
    .rd_data_x2  (w1_rd_data_x2)
);

// External readback mux for fundamental/subharmonic matched filter output
assign mf_x1_out = w1_rd_data_x1;
assign mf_x2_out = w1_rd_data_x2;

// ============================================================================
// W2: Beamforming Wrapper
// MATLAB: x2_bf_norm = beamform_fpga(x2, ww, hh, ts*c, ex, L, start_depth, M)
//         [line 109-110, beamform_fpga function lines 163-193]
// Uses beamforming.sv → Delay_calc.v → cordic_sqrt.v (CLZ.v)
// ============================================================================
w2_beamform_wrapper #(
    .DATA_WIDTH    (DATA_WIDTH),
    .NUM_AXIAL     (NUM_AXIAL),
    .NUM_LATERAL   (NUM_LATERAL),
    .L             (L),
    .APERTURE_SIZE (50),              // 2 × MATLAB's M=25 neighbor elements
    .CALC_INST     (10)               // Parallel delay calculators
) w2_inst (
    .clk         (clk),
    .rst_n       (rst_n),
    .w1_done     (w1_done_int),
    .w2_done     (w2_done_int),
    .w2_busy     (w2_busy),
    // Read W1's x2 output BRAM
    .w1_rd_addr  (w1_rd_addr_from_w2),
    .w1_rd_data  (w1_rd_data_x2),
    // Output port (W3 reads beamformed image)
    .bf_rd_addr  (bf_rd_addr_from_w3),
    .bf_rd_data  (bf_rd_data_to_w3)
);

// ============================================================================
// W3: Volterra Filter Wrapper
// MATLAB: z2_norm = V_out(x2_bf_norm, H_par_scaled)   [lines 124-131]
//         env22   = abs(z2_norm(:,:,2))               [line 133]
//         env23   = abs(z2_norm(:,:,3))               [line 137]
// Uses: flip_matrix.v, Quadraticproduct.v, Cubicproduct.v, Dotproduct.v
// ============================================================================
w3_volterra_wrapper #(
    .DATA_WIDTH   (DATA_WIDTH),
    .M            (M_VOLTERRA),
    .NTERMS_QUAD  (NTERMS_QUAD),
    .NTERMS_CUBIC (NTERMS_CUBIC),
    .NUM_AXIAL    (NUM_AXIAL),
    .NUM_LATERAL  (NUM_LATERAL)
) w3_inst (
    .clk         (clk),
    .rst_n       (rst_n),
    .w2_done     (w2_done_int),
    .w3_done     (w3_done_int),
    .w3_busy     (w3_busy),
    // Read W2's beamformed output BRAM
    .bf_rd_addr  (bf_rd_addr_from_w3),
    .bf_rd_data  (bf_rd_data_to_w3),
    // Output readback
    .out_rd_en   (out_rd_en),
    .out_rd_addr (out_rd_addr),
    .env22_out   (env22_out),
    .env23_out   (env23_out)
);

endmodule