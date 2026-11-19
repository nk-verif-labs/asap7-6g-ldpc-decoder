// ============================================================================
// Module: ldpc_6g_decoder_top
// Description: Pipelined (12, 6) Systematic QC-LDPC Normalized Min-Sum Decoder
//              Targeting ASAP7 (7nm FinFET) @ 1.0 GHz (1000 ps period).
//              - 12 Variable Nodes, 6 Check Nodes (H = [P | I_6])
//              - 6-bit signed LLRs (Q4.2, saturating [-31, +31])
//              - Normalized Min-Sum (alpha = 0.75: (m >> 1) + (m >> 2))
//              - Early syndrome termination + AXI4-Lite CSR telemetry
// ============================================================================
`timescale 1ns / 1ps

module ldpc_6g_decoder_top (
    input  wire        clk,
    input  wire        rst_n,

    // AXI4-Lite Control & Status Slave
    input  wire [3:0]  s_axil_awaddr,
    input  wire        s_axil_awvalid,
    output reg         s_axil_awready,
    input  wire [31:0] s_axil_wdata,
    input  wire [3:0]  s_axil_wstrb,
    input  wire        s_axil_wvalid,
    output reg         s_axil_wready,
    output reg  [1:0]  s_axil_bresp,
    output reg         s_axil_bvalid,
    input  wire        s_axil_bready,
    input  wire [3:0]  s_axil_araddr,
    input  wire        s_axil_arvalid,
    output reg         s_axil_arready,
    output reg  [31:0] s_axil_rdata,
    output reg  [1:0]  s_axil_rresp,
    output reg         s_axil_rvalid,
    input  wire        s_axil_rready,

    // AXI4-Stream LLR Input (12 x 6-bit signed LLRs = 72 bits)
    input  wire [71:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,

    // AXI4-Stream Decoded Codeword Output (12-bit hard decision)
    output reg  [11:0] m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready
);

    // ------------------------------------------------------------------------
    // Helper: Saturate 9-bit signed value to 6-bit signed [-31, +31]
    // ------------------------------------------------------------------------
    function signed [5:0] sat6;
        input signed [8:0] val;
        begin
            if (val > 9'sd31)       sat6 = 6'sd31;
            else if (val < -9'sd31) sat6 = -6'sd31;
            else                    sat6 = val[5:0];
        end
    endfunction

    // ------------------------------------------------------------------------
    // Helper: 6-bit signed magnitude (unsigned 5-bit)
    // ------------------------------------------------------------------------
    function [4:0] abs6;
        input signed [5:0] val;
        reg signed [5:0] neg;
        begin
            if (val < 0) begin
                neg  = -val;
                abs6 = (neg > 6'sd31) ? 5'd31 : neg[4:0];
            end else begin
                abs6 = val[4:0];
            end
        end
    endfunction

    // ------------------------------------------------------------------------
    // Helper: 4-input Normalized Min-Sum CNU (returns C2V for one target edge)
    // ------------------------------------------------------------------------
    function signed [5:0] cnu_edge;
        input signed [5:0] v0;
        input signed [5:0] v1;
        input signed [5:0] v2;
        reg [4:0] a0, a1, a2, m_min, m_scaled;
        reg       sgn;
        begin
            a0 = abs6(v0);
            a1 = abs6(v1);
            a2 = abs6(v2);
            m_min = (a0 < a1) ? ((a0 < a2) ? a0 : a2) : ((a1 < a2) ? a1 : a2);
            // alpha = 0.75: (m >> 1) + (m >> 2)
            m_scaled = (m_min >> 1) + (m_min >> 2);
            sgn = v0[5] ^ v1[5] ^ v2[5];
            cnu_edge = sgn ? -$signed({1'b0, m_scaled}) : $signed({1'b0, m_scaled});
        end
    endfunction

    // ------------------------------------------------------------------------
    // AXI4-Lite CSRs
    // 0x00 CTRL      : [3:0] max_iter (default 8)
    // 0x04 STATUS    : [0] busy, [9:4] last_syndrome, [15:12] last_iters
    // 0x08 DECODE_CNT: 32-bit decoded frame counter
    // 0x0C SYND_ZERO : 32-bit syndrome-zero pass counter
    // ------------------------------------------------------------------------
    reg [3:0]  cfg_max_iter;
    reg [5:0]  stat_last_synd;
    reg [3:0]  stat_last_iters;
    reg [31:0] stat_decode_cnt;
    reg [31:0] stat_synd_zero_cnt;

    reg        aw_en, aw_cap, w_cap;
    reg [3:0]  awaddr_q;
    reg [31:0] wdata_q;

    // ------------------------------------------------------------------------
    // Decoder FSM & Storage
    // ------------------------------------------------------------------------
    localparam ST_IDLE   = 2'd0;
    localparam ST_ITER   = 2'd1;
    localparam ST_OUTPUT = 2'd2;

    reg [1:0]         state;
    reg [3:0]         iter_cnt;
    reg signed [5:0]  llr_ch [0:11];

    // Variable-to-Check (v2c) and Check-to-Variable (c2v) edge messages
    // H rows (each row has 4 ones):
    //   r0: {0, 1, 2, 6}   r1: {1, 2, 3, 7}   r2: {2, 3, 4, 8}
    //   r3: {3, 4, 5, 9}   r4: {0, 4, 5, 10}  r5: {0, 1, 5, 11}
    reg signed [5:0] v2c_r0_0, v2c_r0_1, v2c_r0_2, v2c_r0_6;
    reg signed [5:0] v2c_r1_1, v2c_r1_2, v2c_r1_3, v2c_r1_7;
    reg signed [5:0] v2c_r2_2, v2c_r2_3, v2c_r2_4, v2c_r2_8;
    reg signed [5:0] v2c_r3_3, v2c_r3_4, v2c_r3_5, v2c_r3_9;
    reg signed [5:0] v2c_r4_0, v2c_r4_4, v2c_r4_5, v2c_r4_10;
    reg signed [5:0] v2c_r5_0, v2c_r5_1, v2c_r5_5, v2c_r5_11;

    reg signed [5:0] c2v_r0_0, c2v_r0_1, c2v_r0_2, c2v_r0_6;
    reg signed [5:0] c2v_r1_1, c2v_r1_2, c2v_r1_3, c2v_r1_7;
    reg signed [5:0] c2v_r2_2, c2v_r2_3, c2v_r2_4, c2v_r2_8;
    reg signed [5:0] c2v_r3_3, c2v_r3_4, c2v_r3_5, c2v_r3_9;
    reg signed [5:0] c2v_r4_0, c2v_r4_4, c2v_r4_5, c2v_r4_10;
    reg signed [5:0] c2v_r5_0, c2v_r5_1, c2v_r5_5, c2v_r5_11;

    assign s_axis_tready = (state == ST_IDLE);

    // Combinational CNU outputs
    wire signed [5:0] nc2v_r0_0 = cnu_edge(v2c_r0_1, v2c_r0_2, v2c_r0_6);
    wire signed [5:0] nc2v_r0_1 = cnu_edge(v2c_r0_0, v2c_r0_2, v2c_r0_6);
    wire signed [5:0] nc2v_r0_2 = cnu_edge(v2c_r0_0, v2c_r0_1, v2c_r0_6);
    wire signed [5:0] nc2v_r0_6 = cnu_edge(v2c_r0_0, v2c_r0_1, v2c_r0_2);

    wire signed [5:0] nc2v_r1_1 = cnu_edge(v2c_r1_2, v2c_r1_3, v2c_r1_7);
    wire signed [5:0] nc2v_r1_2 = cnu_edge(v2c_r1_1, v2c_r1_3, v2c_r1_7);
    wire signed [5:0] nc2v_r1_3 = cnu_edge(v2c_r1_1, v2c_r1_2, v2c_r1_7);
    wire signed [5:0] nc2v_r1_7 = cnu_edge(v2c_r1_1, v2c_r1_2, v2c_r1_3);

    wire signed [5:0] nc2v_r2_2 = cnu_edge(v2c_r2_3, v2c_r2_4, v2c_r2_8);
    wire signed [5:0] nc2v_r2_3 = cnu_edge(v2c_r2_2, v2c_r2_4, v2c_r2_8);
    wire signed [5:0] nc2v_r2_4 = cnu_edge(v2c_r2_2, v2c_r2_3, v2c_r2_8);
    wire signed [5:0] nc2v_r2_8 = cnu_edge(v2c_r2_2, v2c_r2_3, v2c_r2_4);

    wire signed [5:0] nc2v_r3_3 = cnu_edge(v2c_r3_4, v2c_r3_5, v2c_r3_9);
    wire signed [5:0] nc2v_r3_4 = cnu_edge(v2c_r3_3, v2c_r3_5, v2c_r3_9);
    wire signed [5:0] nc2v_r3_5 = cnu_edge(v2c_r3_3, v2c_r3_4, v2c_r3_9);
    wire signed [5:0] nc2v_r3_9 = cnu_edge(v2c_r3_3, v2c_r3_4, v2c_r3_5);

    wire signed [5:0] nc2v_r4_0  = cnu_edge(v2c_r4_4, v2c_r4_5, v2c_r4_10);
    wire signed [5:0] nc2v_r4_4  = cnu_edge(v2c_r4_0, v2c_r4_5, v2c_r4_10);
    wire signed [5:0] nc2v_r4_5  = cnu_edge(v2c_r4_0, v2c_r4_4, v2c_r4_10);
    wire signed [5:0] nc2v_r4_10 = cnu_edge(v2c_r4_0, v2c_r4_4, v2c_r4_5);

    wire signed [5:0] nc2v_r5_0  = cnu_edge(v2c_r5_1, v2c_r5_5, v2c_r5_11);
    wire signed [5:0] nc2v_r5_1  = cnu_edge(v2c_r5_0, v2c_r5_5, v2c_r5_11);
    wire signed [5:0] nc2v_r5_5  = cnu_edge(v2c_r5_0, v2c_r5_1, v2c_r5_11);
    wire signed [5:0] nc2v_r5_11 = cnu_edge(v2c_r5_0, v2c_r5_1, v2c_r5_5);

    // Total posteriori LLRs (using current c2v)
    wire signed [8:0] tot0  = $signed({{3{llr_ch[0][5]}}, llr_ch[0]})   + $signed({{3{c2v_r0_0[5]}}, c2v_r0_0}) + $signed({{3{c2v_r4_0[5]}}, c2v_r4_0}) + $signed({{3{c2v_r5_0[5]}}, c2v_r5_0});
    wire signed [8:0] tot1  = $signed({{3{llr_ch[1][5]}}, llr_ch[1]})   + $signed({{3{c2v_r0_1[5]}}, c2v_r0_1}) + $signed({{3{c2v_r1_1[5]}}, c2v_r1_1}) + $signed({{3{c2v_r5_1[5]}}, c2v_r5_1});
    wire signed [8:0] tot2  = $signed({{3{llr_ch[2][5]}}, llr_ch[2]})   + $signed({{3{c2v_r0_2[5]}}, c2v_r0_2}) + $signed({{3{c2v_r1_2[5]}}, c2v_r1_2}) + $signed({{3{c2v_r2_2[5]}}, c2v_r2_2});
    wire signed [8:0] tot3  = $signed({{3{llr_ch[3][5]}}, llr_ch[3]})   + $signed({{3{c2v_r1_3[5]}}, c2v_r1_3}) + $signed({{3{c2v_r2_3[5]}}, c2v_r2_3}) + $signed({{3{c2v_r3_3[5]}}, c2v_r3_3});
    wire signed [8:0] tot4  = $signed({{3{llr_ch[4][5]}}, llr_ch[4]})   + $signed({{3{c2v_r2_4[5]}}, c2v_r2_4}) + $signed({{3{c2v_r3_4[5]}}, c2v_r3_4}) + $signed({{3{c2v_r4_4[5]}}, c2v_r4_4});
    wire signed [8:0] tot5  = $signed({{3{llr_ch[5][5]}}, llr_ch[5]})   + $signed({{3{c2v_r3_5[5]}}, c2v_r3_5}) + $signed({{3{c2v_r4_5[5]}}, c2v_r4_5}) + $signed({{3{c2v_r5_5[5]}}, c2v_r5_5});
    wire signed [8:0] tot6  = $signed({{3{llr_ch[6][5]}}, llr_ch[6]})   + $signed({{3{c2v_r0_6[5]}}, c2v_r0_6});
    wire signed [8:0] tot7  = $signed({{3{llr_ch[7][5]}}, llr_ch[7]})   + $signed({{3{c2v_r1_7[5]}}, c2v_r1_7});
    wire signed [8:0] tot8  = $signed({{3{llr_ch[8][5]}}, llr_ch[8]})   + $signed({{3{c2v_r2_8[5]}}, c2v_r2_8});
    wire signed [8:0] tot9  = $signed({{3{llr_ch[9][5]}}, llr_ch[9]})   + $signed({{3{c2v_r3_9[5]}}, c2v_r3_9});
    wire signed [8:0] tot10 = $signed({{3{llr_ch[10][5]}}, llr_ch[10]}) + $signed({{3{c2v_r4_10[5]}}, c2v_r4_10});
    wire signed [8:0] tot11 = $signed({{3{llr_ch[11][5]}}, llr_ch[11]}) + $signed({{3{c2v_r5_11[5]}}, c2v_r5_11});

    wire [11:0] hard_bits = {
        tot11[8], tot10[8], tot9[8], tot8[8], tot7[8], tot6[8],
        tot5[8],  tot4[8],  tot3[8], tot2[8], tot1[8], tot0[8]
    };

    wire [5:0] syndrome = {
        hard_bits[0] ^ hard_bits[1] ^ hard_bits[5] ^ hard_bits[11],
        hard_bits[0] ^ hard_bits[4] ^ hard_bits[5] ^ hard_bits[10],
        hard_bits[3] ^ hard_bits[4] ^ hard_bits[5] ^ hard_bits[9],
        hard_bits[2] ^ hard_bits[3] ^ hard_bits[4] ^ hard_bits[8],
        hard_bits[1] ^ hard_bits[2] ^ hard_bits[3] ^ hard_bits[7],
        hard_bits[0] ^ hard_bits[1] ^ hard_bits[2] ^ hard_bits[6]
    };

    // AXI4-Lite CSR Logic
    always @(posedge clk) begin
        if (!rst_n) begin
            s_axil_awready <= 1'b0; s_axil_wready <= 1'b0;
            s_axil_bvalid  <= 1'b0; s_axil_bresp  <= 2'b00;
            s_axil_arready <= 1'b0; s_axil_rvalid <= 1'b0;
            s_axil_rresp   <= 2'b00; s_axil_rdata <= 32'd0;
            aw_en <= 1'b1; aw_cap <= 1'b0; w_cap <= 1'b0;
            awaddr_q <= 4'd0; wdata_q <= 32'd0;
            cfg_max_iter <= 4'd8;
        end else begin
            if (!s_axil_awready && s_axil_awvalid && !aw_cap && aw_en) begin
                s_axil_awready <= 1'b1; awaddr_q <= s_axil_awaddr; aw_cap <= 1'b1;
            end else s_axil_awready <= 1'b0;

            if (!s_axil_wready && s_axil_wvalid && !w_cap && aw_en) begin
                s_axil_wready <= 1'b1; wdata_q <= s_axil_wdata; w_cap <= 1'b1;
            end else s_axil_wready <= 1'b0;

            if (aw_cap && w_cap && !s_axil_bvalid) begin
                aw_cap <= 1'b0; w_cap <= 1'b0; aw_en <= 1'b0;
                s_axil_bvalid <= 1'b1; s_axil_bresp <= 2'b00;
                if (awaddr_q[3:2] == 2'b00 && s_axil_wstrb[0]) begin
                    cfg_max_iter <= wdata_q[3:0];
                end
            end else if (s_axil_bvalid && s_axil_bready) begin
                s_axil_bvalid <= 1'b0; aw_en <= 1'b1;
            end

            if (!s_axil_arready && s_axil_arvalid && !s_axil_rvalid) begin
                s_axil_arready <= 1'b1; s_axil_rvalid <= 1'b1; s_axil_rresp <= 2'b00;
                case (s_axil_araddr[3:2])
                    2'b00: s_axil_rdata <= {28'd0, cfg_max_iter};
                    2'b01: s_axil_rdata <= {16'd0, stat_last_iters, 2'b00, stat_last_synd, 3'b000, (state != ST_IDLE)};
                    2'b10: s_axil_rdata <= stat_decode_cnt;
                    2'b11: s_axil_rdata <= stat_synd_zero_cnt;
                endcase
            end else begin
                s_axil_arready <= 1'b0;
                if (s_axil_rvalid && s_axil_rready) s_axil_rvalid <= 1'b0;
            end
        end
    end

    // Iterative Decoder FSM
    integer k;
    always @(posedge clk) begin
        if (!rst_n) begin
            state              <= ST_IDLE;
            iter_cnt           <= 4'd0;
            m_axis_tvalid      <= 1'b0;
            m_axis_tdata       <= 12'd0;
            stat_last_synd     <= 6'd0;
            stat_last_iters    <= 4'd0;
            stat_decode_cnt    <= 32'd0;
            stat_synd_zero_cnt <= 32'd0;
            for (k = 0; k < 12; k = k + 1) llr_ch[k] <= 6'sd0;
            c2v_r0_0 <= 0; c2v_r0_1 <= 0; c2v_r0_2 <= 0; c2v_r0_6 <= 0;
            c2v_r1_1 <= 0; c2v_r1_2 <= 0; c2v_r1_3 <= 0; c2v_r1_7 <= 0;
            c2v_r2_2 <= 0; c2v_r2_3 <= 0; c2v_r2_4 <= 0; c2v_r2_8 <= 0;
            c2v_r3_3 <= 0; c2v_r3_4 <= 0; c2v_r3_5 <= 0; c2v_r3_9 <= 0;
            c2v_r4_0 <= 0; c2v_r4_4 <= 0; c2v_r4_5 <= 0; c2v_r4_10 <= 0;
            c2v_r5_0 <= 0; c2v_r5_1 <= 0; c2v_r5_5 <= 0; c2v_r5_11 <= 0;
            v2c_r0_0 <= 0; v2c_r0_1 <= 0; v2c_r0_2 <= 0; v2c_r0_6 <= 0;
            v2c_r1_1 <= 0; v2c_r1_2 <= 0; v2c_r1_3 <= 0; v2c_r1_7 <= 0;
            v2c_r2_2 <= 0; v2c_r2_3 <= 0; v2c_r2_4 <= 0; v2c_r2_8 <= 0;
            v2c_r3_3 <= 0; v2c_r3_4 <= 0; v2c_r3_5 <= 0; v2c_r3_9 <= 0;
            v2c_r4_0 <= 0; v2c_r4_4 <= 0; v2c_r4_5 <= 0; v2c_r4_10 <= 0;
            v2c_r5_0 <= 0; v2c_r5_1 <= 0; v2c_r5_5 <= 0; v2c_r5_11 <= 0;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (s_axis_tvalid && s_axis_tready) begin
                        for (k = 0; k < 12; k = k + 1) begin
                            llr_ch[k] <= s_axis_tdata[k*6 +: 6];
                        end
                        // Initialize v2c = channel LLRs, c2v = 0
                        v2c_r0_0 <= s_axis_tdata[0*6 +: 6]; v2c_r0_1 <= s_axis_tdata[1*6 +: 6];
                        v2c_r0_2 <= s_axis_tdata[2*6 +: 6]; v2c_r0_6 <= s_axis_tdata[6*6 +: 6];
                        v2c_r1_1 <= s_axis_tdata[1*6 +: 6]; v2c_r1_2 <= s_axis_tdata[2*6 +: 6];
                        v2c_r1_3 <= s_axis_tdata[3*6 +: 6]; v2c_r1_7 <= s_axis_tdata[7*6 +: 6];
                        v2c_r2_2 <= s_axis_tdata[2*6 +: 6]; v2c_r2_3 <= s_axis_tdata[3*6 +: 6];
                        v2c_r2_4 <= s_axis_tdata[4*6 +: 6]; v2c_r2_8 <= s_axis_tdata[8*6 +: 6];
                        v2c_r3_3 <= s_axis_tdata[3*6 +: 6]; v2c_r3_4 <= s_axis_tdata[4*6 +: 6];
                        v2c_r3_5 <= s_axis_tdata[5*6 +: 6]; v2c_r3_9 <= s_axis_tdata[9*6 +: 6];
                        v2c_r4_0 <= s_axis_tdata[0*6 +: 6]; v2c_r4_4 <= s_axis_tdata[4*6 +: 6];
                        v2c_r4_5 <= s_axis_tdata[5*6 +: 6]; v2c_r4_10 <= s_axis_tdata[10*6 +: 6];
                        v2c_r5_0 <= s_axis_tdata[0*6 +: 6]; v2c_r5_1 <= s_axis_tdata[1*6 +: 6];
                        v2c_r5_5 <= s_axis_tdata[5*6 +: 6]; v2c_r5_11 <= s_axis_tdata[11*6 +: 6];

                        c2v_r0_0 <= 0; c2v_r0_1 <= 0; c2v_r0_2 <= 0; c2v_r0_6 <= 0;
                        c2v_r1_1 <= 0; c2v_r1_2 <= 0; c2v_r1_3 <= 0; c2v_r1_7 <= 0;
                        c2v_r2_2 <= 0; c2v_r2_3 <= 0; c2v_r2_4 <= 0; c2v_r2_8 <= 0;
                        c2v_r3_3 <= 0; c2v_r3_4 <= 0; c2v_r3_5 <= 0; c2v_r3_9 <= 0;
                        c2v_r4_0 <= 0; c2v_r4_4 <= 0; c2v_r4_5 <= 0; c2v_r4_10 <= 0;
                        c2v_r5_0 <= 0; c2v_r5_1 <= 0; c2v_r5_5 <= 0; c2v_r5_11 <= 0;

                        iter_cnt <= 4'd0;
                        state    <= ST_ITER;
                    end
                end

                ST_ITER: begin
                    if (syndrome == 6'd0 || iter_cnt >= cfg_max_iter) begin
                        m_axis_tdata    <= hard_bits;
                        m_axis_tvalid   <= 1'b1;
                        stat_last_synd  <= syndrome;
                        stat_last_iters <= iter_cnt;
                        stat_decode_cnt <= stat_decode_cnt + 1'b1;
                        if (syndrome == 6'd0) stat_synd_zero_cnt <= stat_synd_zero_cnt + 1'b1;
                        state           <= ST_OUTPUT;
                    end else begin
                        // Latch CNU -> c2v
                        c2v_r0_0 <= nc2v_r0_0; c2v_r0_1 <= nc2v_r0_1; c2v_r0_2 <= nc2v_r0_2; c2v_r0_6 <= nc2v_r0_6;
                        c2v_r1_1 <= nc2v_r1_1; c2v_r1_2 <= nc2v_r1_2; c2v_r1_3 <= nc2v_r1_3; c2v_r1_7 <= nc2v_r1_7;
                        c2v_r2_2 <= nc2v_r2_2; c2v_r2_3 <= nc2v_r2_3; c2v_r2_4 <= nc2v_r2_4; c2v_r2_8 <= nc2v_r2_8;
                        c2v_r3_3 <= nc2v_r3_3; c2v_r3_4 <= nc2v_r3_4; c2v_r3_5 <= nc2v_r3_5; c2v_r3_9 <= nc2v_r3_9;
                        c2v_r4_0 <= nc2v_r4_0; c2v_r4_4 <= nc2v_r4_4; c2v_r4_5 <= nc2v_r4_5; c2v_r4_10 <= nc2v_r4_10;
                        c2v_r5_0 <= nc2v_r5_0; c2v_r5_1 <= nc2v_r5_1; c2v_r5_5 <= nc2v_r5_5; c2v_r5_11 <= nc2v_r5_11;

                        // Update VNU -> v2c using new c2v (intrinsic + extrinsic sum)
                        v2c_r0_0 <= sat6($signed({{3{llr_ch[0][5]}}, llr_ch[0]}) + $signed({{3{nc2v_r4_0[5]}}, nc2v_r4_0}) + $signed({{3{nc2v_r5_0[5]}}, nc2v_r5_0}));
                        v2c_r4_0 <= sat6($signed({{3{llr_ch[0][5]}}, llr_ch[0]}) + $signed({{3{nc2v_r0_0[5]}}, nc2v_r0_0}) + $signed({{3{nc2v_r5_0[5]}}, nc2v_r5_0}));
                        v2c_r5_0 <= sat6($signed({{3{llr_ch[0][5]}}, llr_ch[0]}) + $signed({{3{nc2v_r0_0[5]}}, nc2v_r0_0}) + $signed({{3{nc2v_r4_0[5]}}, nc2v_r4_0}));

                        v2c_r0_1 <= sat6($signed({{3{llr_ch[1][5]}}, llr_ch[1]}) + $signed({{3{nc2v_r1_1[5]}}, nc2v_r1_1}) + $signed({{3{nc2v_r5_1[5]}}, nc2v_r5_1}));
                        v2c_r1_1 <= sat6($signed({{3{llr_ch[1][5]}}, llr_ch[1]}) + $signed({{3{nc2v_r0_1[5]}}, nc2v_r0_1}) + $signed({{3{nc2v_r5_1[5]}}, nc2v_r5_1}));
                        v2c_r5_1 <= sat6($signed({{3{llr_ch[1][5]}}, llr_ch[1]}) + $signed({{3{nc2v_r0_1[5]}}, nc2v_r0_1}) + $signed({{3{nc2v_r1_1[5]}}, nc2v_r1_1}));

                        v2c_r0_2 <= sat6($signed({{3{llr_ch[2][5]}}, llr_ch[2]}) + $signed({{3{nc2v_r1_2[5]}}, nc2v_r1_2}) + $signed({{3{nc2v_r2_2[5]}}, nc2v_r2_2}));
                        v2c_r1_2 <= sat6($signed({{3{llr_ch[2][5]}}, llr_ch[2]}) + $signed({{3{nc2v_r0_2[5]}}, nc2v_r0_2}) + $signed({{3{nc2v_r2_2[5]}}, nc2v_r2_2}));
                        v2c_r2_2 <= sat6($signed({{3{llr_ch[2][5]}}, llr_ch[2]}) + $signed({{3{nc2v_r0_2[5]}}, nc2v_r0_2}) + $signed({{3{nc2v_r1_2[5]}}, nc2v_r1_2}));

                        v2c_r1_3 <= sat6($signed({{3{llr_ch[3][5]}}, llr_ch[3]}) + $signed({{3{nc2v_r2_3[5]}}, nc2v_r2_3}) + $signed({{3{nc2v_r3_3[5]}}, nc2v_r3_3}));
                        v2c_r2_3 <= sat6($signed({{3{llr_ch[3][5]}}, llr_ch[3]}) + $signed({{3{nc2v_r1_3[5]}}, nc2v_r1_3}) + $signed({{3{nc2v_r3_3[5]}}, nc2v_r3_3}));
                        v2c_r3_3 <= sat6($signed({{3{llr_ch[3][5]}}, llr_ch[3]}) + $signed({{3{nc2v_r1_3[5]}}, nc2v_r1_3}) + $signed({{3{nc2v_r2_3[5]}}, nc2v_r2_3}));

                        v2c_r2_4 <= sat6($signed({{3{llr_ch[4][5]}}, llr_ch[4]}) + $signed({{3{nc2v_r3_4[5]}}, nc2v_r3_4}) + $signed({{3{nc2v_r4_4[5]}}, nc2v_r4_4}));
                        v2c_r3_4 <= sat6($signed({{3{llr_ch[4][5]}}, llr_ch[4]}) + $signed({{3{nc2v_r2_4[5]}}, nc2v_r2_4}) + $signed({{3{nc2v_r4_4[5]}}, nc2v_r4_4}));
                        v2c_r4_4 <= sat6($signed({{3{llr_ch[4][5]}}, llr_ch[4]}) + $signed({{3{nc2v_r2_4[5]}}, nc2v_r2_4}) + $signed({{3{nc2v_r3_4[5]}}, nc2v_r3_4}));

                        v2c_r3_5 <= sat6($signed({{3{llr_ch[5][5]}}, llr_ch[5]}) + $signed({{3{nc2v_r4_5[5]}}, nc2v_r4_5}) + $signed({{3{nc2v_r5_5[5]}}, nc2v_r5_5}));
                        v2c_r4_5 <= sat6($signed({{3{llr_ch[5][5]}}, llr_ch[5]}) + $signed({{3{nc2v_r3_5[5]}}, nc2v_r3_5}) + $signed({{3{nc2v_r5_5[5]}}, nc2v_r5_5}));
                        v2c_r5_5 <= sat6($signed({{3{llr_ch[5][5]}}, llr_ch[5]}) + $signed({{3{nc2v_r3_5[5]}}, nc2v_r3_5}) + $signed({{3{nc2v_r4_5[5]}}, nc2v_r4_5}));

                        v2c_r0_6  <= llr_ch[6];
                        v2c_r1_7  <= llr_ch[7];
                        v2c_r2_8  <= llr_ch[8];
                        v2c_r3_9  <= llr_ch[9];
                        v2c_r4_10 <= llr_ch[10];
                        v2c_r5_11 <= llr_ch[11];

                        iter_cnt <= iter_cnt + 1'b1;
                    end
                end

                ST_OUTPUT: begin
                    if (m_axis_tvalid && m_axis_tready) begin
                        m_axis_tvalid <= 1'b0;
                        state         <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
