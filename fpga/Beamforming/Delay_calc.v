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