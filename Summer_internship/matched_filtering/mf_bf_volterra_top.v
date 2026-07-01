// ============================================================================
// File        : mf_bf_volterra_top.v
// Description : W1 (matched filter) + W2 (DAS beamformer) + W3 (Volterra)
//
//   FIX: W2->W3 start handshake is now req/ack. Previously w2_done_r was a
//   single-cycle pulse; if the wrapper ever failed to sample it in S_IDLE the
//   pipeline would deadlock in T_VOLT. Now the top HOLDS w2_done_r (request)
//   high until the wrapper raises w3_busy (ack), then drops it. Pairs with the
//   wrapper's S_IDLE which latches the level and leaves S_IDLE the same cycle.
// ============================================================================
`timescale 1ns/1ps

module mf_bf_volterra_top #(
    parameter DATA_WIDTH        = 32,
    parameter ACCUM_WIDTH       = 64,
    parameter TAPS              = 136,
    parameter PAD_LEN           = 135,
    parameter OUT_SHIFT         = 28,
    parameter L                 = 1792,
    parameter NUM_CH            = 128,
    parameter AXIAL             = 973,
    parameter LATERAL           = 128,
    parameter SAMP_STRIDE_LOG2  = 11,
    parameter ADDR_W            = 18,
    parameter PIX_LAT_W         = 7,
    parameter PIX_AX_W          = 10,
    parameter BF_ADDR_W         = 17,
    // SKIP_W1=1 : bank_a is preloaded with golden W1 output via the BMG .coe;
    //             skip the matched-filter phase and go straight to the W2 sweep.
    parameter SKIP_W1           = 0,
    parameter signed [DATA_WIDTH-1:0] START_DEPTH_Q626 = 32'sd335544320,
    parameter HH_FILE           = "hh.mem",
    parameter WW_FILE           = "ex.mem",
    parameter FW2_FILE          = "fw2_coeffs.mem"
)(
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        start,
    input  wire                        enable_sweep,
    input  wire                        enable_volterra,
    input  wire                        pixel_ready,
    input  wire signed [DATA_WIDTH-1:0] rf_din,
    input  wire                         rf_din_valid,
    output wire                         rf_ready,
    output reg  signed [DATA_WIDTH-1:0] pixel_out,
    output reg                          pixel_out_valid,
    output reg  [PIX_LAT_W-1:0]         pixel_lat_idx,
    output reg  [PIX_AX_W-1:0]          pixel_ax_idx,
    // ---- live Volterra pixel stream (replaces env read-back interface) ----
    output wire                        pix_valid,
    output wire [9:0]                  pix_ax,
    output wire [6:0]                  pix_lat,
    output wire [DATA_WIDTH-1:0]       pix_env22,
    output wire [DATA_WIDTH-1:0]       pix_env23,
    input  wire                        pix_ack,
    output reg                          mf_done,
    output reg                          done
);

    localparam BANK_DEPTH = NUM_CH << SAMP_STRIDE_LOG2;
    localparam BF_BRAM_D  = AXIAL * LATERAL;

    function integer clog2;
        input integer value;
        integer v;
        begin
            v = value - 1;
            for (clog2 = 0; v > 0; clog2 = clog2 + 1)
                v = v >> 1;
        end
    endfunction

    localparam CH_W   = clog2(NUM_CH);
    localparam PAD_W  = clog2(PAD_LEN + 1);
    localparam SAMP_W = clog2(L + 1);
    localparam LAT_W  = clog2(LATERAL);
    localparam AX_W   = clog2(AXIAL);

    (* ram_style = "block" *) reg signed [DATA_WIDTH-1:0] hh_rom [0:AXIAL-1];
    (* ram_style = "block" *) reg signed [DATA_WIDTH-1:0] ww_rom [0:LATERAL-1];
    initial begin
        $readmemh(HH_FILE, hh_rom);
        $readmemh(WW_FILE, ww_rom);
    end

    // ---- W1 ----
    reg  signed [DATA_WIDTH-1:0]  mf_din;
    reg                           mf_din_valid;
    reg                           mf_rst;
    wire signed [ACCUM_WIDTH-1:0] mf_dout;
    wire                          mf_dout_valid;

    matched_filter_1d_rom #(
        .DATA_WIDTH (DATA_WIDTH), .TAPS(TAPS), .ACCUM_WIDTH(ACCUM_WIDTH),
        .ROW_LENGTH (PAD_LEN + L), .ROM_FILE(FW2_FILE)
    ) u_mf_fw2 (
        .clk(clk), .rst(mf_rst | ~rst_n),
        .din(mf_din), .din_valid(mf_din_valid),
        .dout(mf_dout), .dout_valid(mf_dout_valid)
    );

    // ---- bank_a ----
    reg  [ADDR_W-1:0]            bank_waddr;
    reg                          bank_we;
    reg  signed [DATA_WIDTH-1:0] bank_wdata;
    wire [ADDR_W-1:0]            bank_raddr;
    wire [DATA_WIDTH/2-1:0]        bank_rdata;
    // bank_a stores bits [24:9] of the Q2.30 W1 output (16-bit signed).
    // Rebuild full 32-bit Q2.30: sign-extend 7, then <<9.
    wire signed [DATA_WIDTH-1:0] bank_rdata_q230 = { bank_rdata, 16'b0 };
        
            
    bank_a_bram u_bank_a (
        .clka(clk), .ena(1'b1), .wea({4{bank_we}}),
        .addra(bank_waddr), .dina(bank_wdata),
        .clkb(clk), .enb(1'b1),
        .addrb(bank_raddr), .doutb(bank_rdata));

    // ---- W2 ----
    reg  signed [DATA_WIDTH-1:0] bf_x_pixel, bf_z_pixel;
    reg  [7:0]                   bf_center_idx;
    reg                          bf_start;
    wire signed [DATA_WIDTH-1:0] bf_pixel_val;
    wire                         bf_pixel_valid;

    beamforming_synth #(
        .DATA_WIDTH(DATA_WIDTH), .APERTURE_SIZE(50), .ADDR_W(SAMP_STRIDE_LOG2)
    ) u_bf (
        .clk(clk), .reset(rst_n), .start(bf_start),
        .x_pixel_in(bf_x_pixel), .z_pixel_in(bf_z_pixel),
        .start_depth_in(START_DEPTH_Q626), .center_idx_in(bf_center_idx),
        .rf_rd_addr(bank_raddr), .rf_rd_data(bank_rdata_q230),
        .final_pixel_val(bf_pixel_val), .pixel_valid(bf_pixel_valid)
    );

    // ---- bf_bram : 16-bit STORAGE, 32-bit (Q2.30) interface to W3 ----
    // Stores the most-significant 16 bits of each beamformed pixel and rebuilds
    // a 32-bit word (low 16 zeroed) on read. Halves this BRAM (~123 -> ~62
    // RAMB36). PW=32 arithmetic in W3 is unchanged. Slice [31:16] is adjustable
    // if your beamformed magnitudes are small (shift the window down).
    localparam BF_STORE_W = 16;
   // ---- bf_bram : full 32-bit (Q2.30). bank_a is 16-bit instead, so the
    //      Volterra input stays bit-exact (no slicing/guesswork here). ----
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] bf_bram [0:BF_BRAM_D-1];
    reg  [BF_ADDR_W-1:0]  bf_waddr;
    reg                   bf_we;
    reg  [DATA_WIDTH-1:0] bf_wdata;
    reg  [DATA_WIDTH-1:0] w3_bf_rd_data_r;
    wire [BF_ADDR_W-1:0]  w3_bf_rd_addr;

    always @(posedge clk) begin
        if (bf_we)
            bf_bram[bf_waddr] <= bf_wdata;
        w3_bf_rd_data_r <= bf_bram[w3_bf_rd_addr];
    end
        
    wire [BF_ADDR_W-1:0] bf_wr_addr =
        ( {{(BF_ADDR_W-AX_W){1'b0}}, ax_idx}  << LAT_W ) |
          {{(BF_ADDR_W-LAT_W){1'b0}}, lat_idx};

    // ---- W3 ----
    reg  w2_done_r;     // REQ (level, held until w3_busy ack)
    wire w3_done_int;
    wire w3_busy_int;

    w3_volterra_wrapper #(
        .DATA_WIDTH(DATA_WIDTH), .M(15), .NTERMS_QUAD(120), .NTERMS_CUBIC(680),
        .IDX_W(4), .NUM_AXIAL(AXIAL), .NUM_LATERAL(LATERAL), .OUT_BRAM_D(AXIAL*LATERAL)
    ) u_w3 (
        .clk(clk), .rst_n(rst_n),
        .w2_done(w2_done_r), .w3_done(w3_done_int), .w3_busy(w3_busy_int),
        .bf_rd_addr(w3_bf_rd_addr), .bf_rd_data(w3_bf_rd_data_r),
        .pix_valid(pix_valid), .pix_ax(pix_ax), .pix_lat(pix_lat),
        .pix_env22(pix_env22), .pix_env23(pix_env23), .pix_ack(pix_ack)
    );

    // ---- MF back-end ----
    localparam MF_IDLE=3'd0, MF_RST=3'd1, MF_PAD=3'd2, MF_RUN=3'd3,
               MF_FLUSH=3'd4, MF_NEXT=3'd5, MF_FIN=3'd6;

    reg [2:0]        mf_state;
    reg [CH_W-1:0]   mf_ch;
    reg [PAD_W-1:0]  pad_cnt;
    reg [SAMP_W-1:0] samp_cnt;
    reg [SAMP_W-1:0] out_cnt;

    assign rf_ready = (mf_state == MF_RUN);

    wire [ADDR_W-1:0] mf_ch_ext   = mf_ch;
    wire [ADDR_W-1:0] out_cnt_ext = out_cnt;
    wire [ADDR_W-1:0] mf_wr_addr  = (mf_ch_ext << SAMP_STRIDE_LOG2) | out_cnt_ext;
    wire mf_capture = mf_dout_valid && (out_cnt < L);

    // ---- top FSM ----
    localparam T_IDLE=3'd0, T_MF=3'd1, T_SW_SET=3'd2, T_SW_RD=3'd3,
               T_SW_LATCH=3'd4, T_SW_WAIT=3'd5, T_VOLT=3'd6, T_DONE=3'd7;

    reg [2:0]        t_state;
    reg [LAT_W-1:0]  lat_idx;
    reg [AX_W-1:0]   ax_idx;
    reg              bf_seen_low;
    reg              w3_started;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t_state<=T_IDLE; lat_idx<=0; ax_idx<=0; bf_seen_low<=1'b0; w3_started<=1'b0;
            mf_state<=MF_IDLE; mf_ch<=0; pad_cnt<=0; samp_cnt<=0; out_cnt<=0;
            mf_din<=0; mf_din_valid<=1'b0; mf_rst<=1'b0;
            bank_we<=1'b0; bank_waddr<=0; bank_wdata<=0;
            bf_we<=1'b0; bf_waddr<=0; bf_wdata<=0;
            bf_start<=1'b0; bf_x_pixel<=0; bf_z_pixel<=0; bf_center_idx<=0;
            pixel_out<=0; pixel_out_valid<=1'b0; pixel_lat_idx<=0; pixel_ax_idx<=0;
            mf_done<=1'b0; done<=1'b0;
            w2_done_r<=1'b0;
        end else begin
            // one-cycle-pulse defaults
            mf_din_valid    <= 1'b0;
            mf_rst          <= 1'b0;
            bank_we         <= 1'b0;
            bf_we           <= 1'b0;
            bf_start        <= 1'b0;
            pixel_out_valid <= 1'b0;
            mf_done         <= 1'b0;
            done            <= 1'b0;
            // NOTE: w2_done_r is NOT defaulted here anymore -- it is a held REQ
            // line driven explicitly in T_VOLT / T_DONE (req/ack handshake).

            if (mf_capture) begin
                bank_we    <= 1'b1;
                bank_waddr <= mf_wr_addr;
                bank_wdata <= mf_dout[OUT_SHIFT +: DATA_WIDTH];
                out_cnt    <= out_cnt + 1'b1;
            end

            case (t_state)

            T_IDLE: begin
                if (start) begin
                    w3_started <= 1'b0;
                    w2_done_r  <= 1'b0;     // ensure req is low before a new run
                    if (SKIP_W1) begin
                        // bank_a already holds golden W1 output (BMG .coe init);
                        // jump straight into the beamformer sweep.
                        lat_idx  <= 0;
                        ax_idx   <= 0;
                        mf_done  <= 1'b1;   // signal "W1 stage complete"
                        t_state  <= enable_sweep ? T_SW_SET : T_DONE;
                    end else begin
                        mf_state <= MF_RST;
                        mf_ch    <= 0;
                        t_state  <= T_MF;
                    end
                end
            end

            T_MF: begin
                case (mf_state)
                MF_RST: begin
                    mf_rst<=1'b1; pad_cnt<=0; samp_cnt<=0; out_cnt<=0; mf_state<=MF_PAD;
                end
                MF_PAD: begin
                    mf_din<=0; mf_din_valid<=1'b1;
                    if (pad_cnt==PAD_LEN-1) begin pad_cnt<=0; mf_state<=MF_RUN; end
                    else pad_cnt<=pad_cnt+1'b1;
                end
                MF_RUN: begin
                    mf_din<=rf_din; mf_din_valid<=rf_din_valid;
                    if (rf_din_valid) begin
                        if (samp_cnt==L-1) mf_state<=MF_FLUSH;
                        else samp_cnt<=samp_cnt+1'b1;
                    end
                end
                MF_FLUSH: begin
                    mf_din<=0; mf_din_valid<=1'b1;
                    if (out_cnt==L-1 && mf_capture) mf_state<=MF_NEXT;
                    else if (out_cnt==L) mf_state<=MF_NEXT;
                end
                MF_NEXT: begin
                    mf_din_valid<=1'b0;
                    if (mf_ch==NUM_CH-1) mf_state<=MF_FIN;
                    else begin mf_ch<=mf_ch+1'b1; mf_state<=MF_RST; end
                end
                MF_FIN: begin
                    mf_done<=1'b1;
                    if (enable_sweep) begin lat_idx<=0; ax_idx<=0; t_state<=T_SW_SET; end
                    else t_state<=T_DONE;
                end
                default: mf_state<=MF_RST;
                endcase
            end

            T_SW_SET: begin
                bf_center_idx <= lat_idx;
                t_state       <= T_SW_RD;
            end

            T_SW_RD: begin
                bf_x_pixel <= ww_rom[lat_idx];
                bf_z_pixel <= hh_rom[ax_idx];
                t_state    <= T_SW_LATCH;
            end

            T_SW_LATCH: begin
                bf_start    <= 1'b1;
                bf_seen_low <= 1'b0;
                t_state     <= T_SW_WAIT;
            end

            T_SW_WAIT: begin
                if (!bf_pixel_valid)
                    bf_seen_low <= 1'b1;
                if (bf_pixel_valid && bf_seen_low && pixel_ready) begin
                    pixel_out       <= bf_pixel_val;
                    pixel_out_valid <= 1'b1;
                    pixel_lat_idx   <= lat_idx;
                    pixel_ax_idx    <= ax_idx;
                    bf_we    <= 1'b1;
                    bf_waddr <= bf_wr_addr;
                    bf_wdata <= bf_pixel_val;
                    if (ax_idx == AXIAL-1) begin
                        ax_idx <= 0;
                        if (lat_idx == LATERAL-1) begin
                            t_state <= enable_volterra ? T_VOLT : T_DONE;
                        end else begin
                            lat_idx <= lat_idx + 1'b1;
                            t_state <= T_SW_SET;
                        end
                    end else begin
                        ax_idx  <= ax_idx + 1'b1;
                        t_state <= T_SW_SET;
                    end
                end
            end

            // ---- W3 : req/ack start handshake ----
            T_VOLT: begin
                if (!w3_started) begin
                    w2_done_r <= 1'b1;          // assert REQ (held)
                    if (w3_busy_int) begin       // wrapper ACK
                        w2_done_r  <= 1'b0;
                        w3_started <= 1'b1;
                    end
                end
                if (w3_done_int) begin
                    w2_done_r <= 1'b0;
                    t_state   <= T_DONE;
                end
            end

            T_DONE: begin
                done      <= 1'b1;
                w2_done_r <= 1'b0;
                t_state   <= T_IDLE;
            end

            default: t_state <= T_IDLE;
            endcase
        end
    end

endmodule