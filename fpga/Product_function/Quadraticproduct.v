`timescale 1ns / 1ps

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
