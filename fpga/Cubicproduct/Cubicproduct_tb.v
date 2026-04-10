`timescale 1ns / 1ps

module volterra_cubic_tb;

    // Parameters
    parameter M = 15;
    parameter W = 32;
    parameter IDX_W = 4;
    parameter NTERMS = 10; 

    reg clk;
    reg rst_n;
    reg start_load;
    reg [W-1:0] data_in;
    
    wire ready;
    wire done;
    wire [3*W-1:0] data_out;

    // Variables for Verification
    real golden_real [0:NTERMS-1];
    real yy_real [0:M-1];
    real rtl_real;
    real scale = 2.0**31; 

    volterra_wrapper # (
        .M(M), .W(W), .IDX_W(IDX_W), .NTERMS(NTERMS)
    ) uut (
        .clk(clk),
        .rst_n(rst_n),
        .start_load(start_load),
        .data_in(data_in),
        .ready(ready),
        .done(done),
        .data_out(data_out)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    integer i, k;
    
    initial begin
        
        rst_n = 0;
        start_load = 0;
        data_in = 0;
        
        // Define Test Data
        yy_real[0]  = 0.003788879;  yy_real[1]  = 0.026653082;  yy_real[2]  = 0.030207293;
        yy_real[3]  = -0.00887535;  yy_real[4]  = -0.037677948; yy_real[5]  = -0.018201157;
        yy_real[6]  = 0.011609129;  yy_real[7]  = 0.021253069;  yy_real[8]  = 0.008482087;
        yy_real[9]  = 0.000615561;  yy_real[10] = -0.000248243; yy_real[11] = 0.010842782;
        yy_real[12] = 0.000833651;  yy_real[13] = -0.009041418; yy_real[14] = -0.021143747;

        golden_real[0] = 0.00000305; golden_real[1] = -0.000000896; golden_real[2] = -0.0000038;
        golden_real[3] = -0.00000184; golden_real[4] = 0.00000117; golden_real[5] = 0.00000215;
        golden_real[6] = 0.000000857; golden_real[7] = 0.0000000622; golden_real[8] = -0.0000000251;
        golden_real[9] = 0.00000109;

        #20 rst_n = 1;
        #20;

        // Start Loading Phase
        @(posedge clk);
        start_load <= 1;
        @(posedge clk);
        start_load <= 0;

        // Stream YY data into the wrapper
        $display("Loading YY values...");
        for (i = 0; i < M; i = i + 1) begin
            data_in <= $rtoi(yy_real[i] * scale);
            @(posedge clk);
        end

        // Stream Indices 
        $display("Loading Indices...");
        for (i = 0; i < NTERMS; i = i + 1) begin
            data_in = 0;
            // Packing logic must match the wrapper's LOAD_D3 state
            data_in[IDX_W-1:0]          = 0;     // d3_a (always 0 in this tb)
            data_in[2*IDX_W-1:IDX_W]    = 1;     // d3_b (always 1 in this tb)
            data_in[3*IDX_W-1:2*IDX_W]  = (2+i); // d3_c (2, 3, 4..)
            @(posedge clk);
        end

        $display("Waiting for Calculation...");
        
        k = 0;
        while (!done) begin
            @(posedge clk);
            if (uut.state == 3'd4) begin // Checking if FSM is in READOUT state (4)
                rtl_real = $itor($signed(data_out)) / (2.0**93);
                $display(" Term %0d | RTL: %.10f | Expected: %.10f", k, rtl_real, golden_real[k]);
                k = k + 1;
            end
        end

        $display("Verification Complete.");
        #100;
        $finish;
    end

endmodule
