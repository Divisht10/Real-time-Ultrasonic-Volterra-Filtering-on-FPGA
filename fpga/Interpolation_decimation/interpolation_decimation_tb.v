`timescale 1ns/1ps

module tb_custom_interp1;

    parameter DATA_WIDTH = 32;
    parameter SCALE      = 1073741824.0; // 2^30
    parameter TOL        = 1e-3;

    reg clk = 0;
    always #2 clk = ~clk;   // 250 MHz

    reg rst;
    reg signed [DATA_WIDTH-1:0] y_in;
    reg y_in_valid;

    wire signed [DATA_WIDTH-1:0] vq_out;
    wire vq_out_valid;

    // DUT
    custom_interp1 dut (
        .clk(clk),
        .rst(rst),
        .y_in(y_in),
        .y_in_valid(y_in_valid),
        .vq_out(vq_out),
        .vq_out_valid(vq_out_valid)
    );

    // Arrays
    real y_input_real [0:543];
    real vq_expected_real [0:67];
    real rtl_outputs [0:67];

    integer input_index;
    integer output_index;
    integer k;

    integer fd_in;
    integer fd_out;

    real rtl_max;
    real error;
    real max_error;

    // ===============================
    // LOAD FILES
    // ===============================
    initial begin
        $display("Loading input data...");

        // Load input samples
        fd_in = $fopen("y_input_real.mem", "r");
        if (fd_in == 0) begin
            $display("ERROR: Cannot open y_input_real.mem");
            $finish;
        end

        for (k = 0; k < 544; k = k + 1) begin
            $fscanf(fd_in, "%f\n", y_input_real[k]);
        end
        $fclose(fd_in);

        // Load expected outputs
        fd_out = $fopen("vq_expected_real.mem", "r");
        if (fd_out == 0) begin
            $display("ERROR: Cannot open vq_expected_real.mem");
            $finish;
        end

        for (k = 0; k < 68; k = k + 1) begin
            $fscanf(fd_out, "%f\n", vq_expected_real[k]);
        end
        $fclose(fd_out);

        $display("File loading DONE");
    end

    // ===============================
    // MAIN TEST
    // ===============================
    initial begin
        $display("TESTBENCH STARTED");

        rst = 1;
        y_in_valid = 0;
        y_in = 0;
        input_index = 0;
        output_index = 0;

        #25;
        rst = 0;

        @(negedge clk);
        y_in_valid <= 1;

        // Feed input samples
        for (input_index = 0; input_index < 544; input_index = input_index + 1) begin
            y_in <= $rtoi(y_input_real[input_index] * SCALE);
            @(negedge clk);
        end

        y_in_valid <= 0;

        // Wait for pipeline
        #500;

        // Find max value
        rtl_max = 0.0;
        for (k = 0; k < 68; k = k + 1) begin
            if (rtl_outputs[k] > rtl_max)
                rtl_max = rtl_outputs[k];
        end

        // Normalize
        if (rtl_max > 0.0) begin
            for (k = 0; k < 68; k = k + 1) begin
                rtl_outputs[k] = rtl_outputs[k] / rtl_max;
            end
        end

        max_error = 0.0;

        $display("\n====================================");
        $display("Comparing RTL vs MATLAB");
        $display("====================================");

        for (k = 0; k < 68; k = k + 1) begin
            error = rtl_outputs[k] - vq_expected_real[k];

            if (error < 0)
                error = -error;

            if (error > max_error)
                max_error = error;

            $display("xq=%0d RTL=%.10f MATLAB=%.10f ERROR=%.10f",
                     k,
                     rtl_outputs[k],
                     vq_expected_real[k],
                     error);
        end

        $display("\n====================================");
        $display("MAX ABS ERROR = %f", max_error);

        if (max_error < TOL)
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL");

        $display("====================================\n");

        $finish;
    end

    // ===============================
    // OUTPUT CAPTURE
    // ===============================
    always @(posedge clk) begin
        if (vq_out_valid && output_index < 68) begin
            rtl_outputs[output_index] = vq_out / SCALE;
            output_index = output_index + 1;
        end
    end

endmodule