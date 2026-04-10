`timescale 1ns / 1ps

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
