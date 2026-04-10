`timescale 1ns / 1ps

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
