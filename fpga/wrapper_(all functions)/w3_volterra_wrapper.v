// ============================================================================
// File        : w3_volterra_wrapper.v
// Description : W3 - Volterra Filter Wrapper
//               Implements MATLAB V_out() function (lines 290-317)
//
// MATLAB Equivalent:
//   z2_norm = V_out(x2_bf_norm, H_par_scaled)
//   env22 = abs(z2_norm(:,:,2))  �? quadratic Volterra output
//   env23 = abs(z2_norm(:,:,3))  �? cubic Volterra output
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
reg [16:0] out_addr;
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
