`timescale 1ns / 1ps

module volterra_quad_tb;

    // Parameters
    parameter M = 15;
    parameter W = 32;
    parameter IDX_W = 4;
    parameter NTERMS = 20; 

    reg clk;
    reg rst_n;
    reg start_load;
    reg [W-1:0] data_in;
    
    wire ready;
    wire done;
    wire [2*W-1:0] data_out;

    real golden_real [0:NTERMS-1];
    real yy_real [0:M-1];
    real rtl_real;
    real scale = 2.0**31;

    volterra_quad_wrapper # (
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

        // Input YY values
        yy_real[0]  = 0.003788879;  yy_real[1]  = 0.026653082;  yy_real[2]  = 0.030207293;
        yy_real[3]  = -0.00887535;  yy_real[4]  = -0.037677948; yy_real[5]  = -0.018201157;
        yy_real[6]  = 0.011609129;  yy_real[7]  = 0.021253069;  yy_real[8]  = 0.008482087;
        yy_real[9]  = 0.000615561;  yy_real[10] = -0.000248243; yy_real[11] = 0.010842782;
        yy_real[12] = 0.000833651;  yy_real[13] = -0.009041418; yy_real[14] = -0.021143747;

        // actual output values from MATLAB
        golden_real[0] = 0.000100985000;
        golden_real[1] = 0.000114452000;
        golden_real[2] = -0.000033600000;
        golden_real[3] = -0.000142757000;
        golden_real[4] = -0.000069000000;
        golden_real[5] = 0.000044000000;
        golden_real[6] = 0.000080500000;
        golden_real[7] = 0.000032100000;
        golden_real[8] = 0.000002330000;
        golden_real[9] = -0.000000941000;
        golden_real[10] = 0.000041100000;
        golden_real[11] = 0.000003160000;
        golden_real[12] = -0.000034300000;
        golden_real[13] = -0.000080100000;
        golden_real[14] = 0.000805117000;
        golden_real[15] = -0.000236555000;
        golden_real[16] = -0.001004233000;
        golden_real[17] = -0.000485117000;
        golden_real[18] = 0.000309419000;
        golden_real[19] = 0.000566460000;

        #20 rst_n = 1;
        #20;

        // Start loading
        @(posedge clk);
        start_load <= 1;
        @(posedge clk);
        start_load <= 0;

        // Load YY
        $display("Loading YY values...");
        for (i = 0; i < M; i = i + 1) begin
            data_in <= $rtoi(yy_real[i] * scale);
            @(posedge clk);
        end

        // Load D2 indices
        $display("Loading Indices...");
        for (i = 0; i < NTERMS; i = i + 1) begin
            data_in = 0;

            data_in[IDX_W-1:0]       = (i < 14) ? 0 : 1;   // a
            data_in[2*IDX_W-1:IDX_W] = (i < 14) ? (i+1) : (i-12); // b

            @(posedge clk);
        end

        // Read Output
        $display("Waiting for Calculation...");

        k = 0;
        while (!done) begin
            @(posedge clk);

            if (uut.state == 3'd4) begin // READOUT stage
                rtl_real = $itor($signed(data_out)) / (2.0**62);

                $display("Term %0d | RTL: %.10f | Expected: %.10f",
                          k, rtl_real, golden_real[k]);

                k = k + 1;
            end
        end

        $display("Verification Complete.");
        #50;
        $finish;
    end

endmodule
