`timescale 1ns/1ps

module tb_custom_flipud_mem;

    parameter DW = 16;
    parameter SCALE = 32768.0;
    parameter TOL = 1e-3;
    parameter ROWS = 15;
    parameter COLS = 128;
    parameter TOTAL_SAMPLES = ROWS * COLS;

    reg clk = 0;
    always #5 clk = ~clk;

    reg rst_n;
    reg write_en;
    reg [3:0] write_row;
    reg [6:0] write_col;
    reg signed [DW-1:0] data_in;

    reg read_en;
    reg [3:0] read_row;
    reg [6:0] read_col;
    wire signed [DW-1:0] data_out;

    // DUT
    custom_flipud_ff dut (
        .clk(clk),
        .rst_n(rst_n),
        .write_en(write_en),
        .write_row(write_row),
        .write_col(write_col),
        .data_in(data_in),
        .read_en(read_en),
        .read_row(read_row),
        .read_col(read_col),
        .data_out(data_out)
    );

    // REAL arrays (same as your TB)
    real y_input_real [0:TOTAL_SAMPLES-1];
    real vq_expected_real [0:TOTAL_SAMPLES-1];
    real rtl_outputs [0:TOTAL_SAMPLES-1];

    integer input_index;
    integer output_index;
    integer k, r, c;

    real error;
    real max_error;

    integer f_in, f_exp;
    integer status;

    initial begin
        $display("Loading FLOAT mem files...");

        // Open files
        f_in  = $fopen("y_input_real.mem", "r");
        f_exp = $fopen("vq_expected_real.mem", "r");

        if (f_in == 0 || f_exp == 0) begin
            $display("ERROR: Could not open mem files");
            $finish;
        end

        // Read floating values
        for (k = 0; k < TOTAL_SAMPLES; k = k + 1) begin
            status = $fscanf(f_in, "%f\n", y_input_real[k]);
            status = $fscanf(f_exp, "%f\n", vq_expected_real[k]);
        end

        $fclose(f_in);
        $fclose(f_exp);

        // RESET
        rst_n = 0;
        write_en = 0;
        read_en = 0;
        #20;
        rst_n = 1;

        // WRITE PHASE
        input_index = 0;
        for (r = 0; r < ROWS; r = r + 1) begin
            for (c = 0; c < COLS; c = c + 1) begin
                @(posedge clk);
                write_en = 1;
                write_row = r;
                write_col = c;

                data_in = $rtoi(y_input_real[input_index] * SCALE);
                input_index = input_index + 1;
            end
        end
        write_en = 0;

        // READ + STORE
        output_index = 0;
        for (r = 0; r < ROWS; r = r + 1) begin
            for (c = 0; c < COLS; c = c + 1) begin
                @(posedge clk);
                read_en = 1;
                read_row = r;
                read_col = c;

                @(posedge clk);
                rtl_outputs[output_index] = data_out / SCALE;
                output_index = output_index + 1;
            end
        end

        // ERROR CHECK
        max_error = 0;
        for (k = 0; k < TOTAL_SAMPLES; k = k + 1) begin
            error = rtl_outputs[k] - vq_expected_real[k];
            if (error < 0) error = -error;

            if (error > max_error)
                max_error = error;
        end

        $display("MAX ERROR = %f", max_error);
        $finish;
    end

endmodule