//==============================================================================
// shape_ink.v -- 墨迹分割 + 连通域填充率分类
//
// 思路: 不用 Sobel 边缘, 直接找"比周围更暗的笔迹".
//   1. 4x4 取最暗降采样        -> 480x270, 细线不被平均掉
//   2. 3x3 局部均值 - 像素      -> 各向同性暗线响应, 对光照渐变免疫
//   3. 因果 3x3 膨胀            -> 连上 1px 断口
//   4. 游程 + 并查集连通域      -> 与 shape_rec 相同
//   5. 填充率 fill = span/(h*maxspan), 两个阈值分类
//        < 0.66 三角形 (理论 0.50) | 0.66~0.86 圆 (0.785) | >0.86 矩形 (1.0)
//==============================================================================
`timescale 1ns/1ps

module shape_ink #(
    parameter integer H_ACTIVE  = 1920,
    parameter integer V_ACTIVE  = 1080,
    parameter integer LAB       = 32,
    parameter integer RMAX      = 64,
    parameter integer MIN_SPAN  = 400,
    parameter integer MIN_WH    = 14,
    parameter integer THR_DEF   = 18,
    parameter integer Y_OFS     = 2      // 绘制时补偿流水延迟(单位: 超行)
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        i_de,
    input  wire        i_hs,
    input  wire        i_vs,
    input  wire [23:0] i_rgb,
    input  wire        i_edge,
    input  wire [3:0]  i_dir16,
    input  wire [10:0] i_row,

    input  wire        i_en,
    input  wire [11:0] i_draw_x0,
    input  wire [7:0]  i_thr,

    output reg         o_de,
    output reg         o_hs,
    output reg         o_vs,
    output reg  [23:0] o_rgb,
    output reg         o_shape_hit,
    output wire [3:0]  o_dbg
);

    localparam integer SW = H_ACTIVE / 4;
    localparam integer SH = V_ACTIVE / 4;
    localparam [9:0]   SW_LAST = SW[9:0] - 10'd1;
    localparam [5:0]   LAB_Q    = LAB;
    localparam [8:0]   SH_M1    = SH - 1;
    localparam [19:0]  SPAN_MIN = MIN_SPAN;
    localparam [9:0]   WH_MIN   = MIN_WH;
    localparam [7:0]   THR_Q    = THR_DEF;
    localparam [8:0]   YOFS_Q   = Y_OFS;

    integer q;

    wire [7:0] gray = i_rgb[15:8];   // 亮度取 G 通道

    //=========================================================================
    // 1) 4x4 降采样: 取块中心像素 (1,1)
    //    不取块内最小值 -- 实测最小值会把纸面噪点放大, 反而让外接框变大,
    //    中心点采样效果更好, 而且完全不需要行缓冲.
    //=========================================================================
    reg [1:0] colsub, subrow;
    reg [9:0] x4;
    reg       g4_ce;
    reg [7:0] g4_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            colsub <= 2'd0; subrow <= 2'd0; x4 <= 10'd0;
            g4_ce <= 1'b0; g4_d <= 8'd0;
        end else if (i_vs) begin
            colsub <= 2'd0; subrow <= 2'd0; x4 <= 10'd0;
            g4_ce <= 1'b0; g4_d <= 8'd0;
        end else begin
            g4_ce <= 1'b0;
            if (i_de) begin
                if ((colsub == 2'd1) && (subrow == 2'd1)) g4_d <= gray;
                if (colsub == 2'd3) begin
                    if ((subrow == 2'd1)) g4_ce <= 1'b1;   // 本块中心已采到
                    x4 <= (x4 == SW_LAST) ? 10'd0 : (x4 + 10'd1);
                end
                colsub <= colsub + 2'd1;
            end else begin
                colsub <= 2'd0;
                if (subrow == 2'd3) subrow <= 2'd0;
                else                subrow <= subrow + 2'd1;
            end
        end
    end

    //=========================================================================
    // 2) 3x3 局部均值 (在 g4 域), 输出对齐到 (x-1, y-1)
    //=========================================================================
    reg [7:0] gb0 [0:SW-1];
    reg [7:0] gb1 [0:SW-1];
    reg [7:0] g4_p1, g4_p2, g4_r1;
    reg [9:0] vs, vs_a, vs_b;
    reg [7:0] g4_ref;
    reg       mean_ce;

    wire [9:0] vs_now = {2'd0, g4_d} + {2'd0, g4_p1} + {2'd0, g4_p2};
    wire [12:0] hs    = {3'd0, vs} + {3'd0, vs_a} + {3'd0, vs_b};   // 最大 9*255=2295
    wire [18:0] hs_q  = hs * 13'd57;              // /9 ≈ *57/512
    wire [7:0]  mean9 = (hs_q[18:17] != 2'd0) ? 8'hFF : hs_q[16:9];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            g4_p1 <= 8'd0; g4_p2 <= 8'd0; g4_r1 <= 8'd0;
            vs <= 10'd0; vs_a <= 10'd0; vs_b <= 10'd0;
            g4_ref <= 8'd0; mean_ce <= 1'b0;
        end else if (i_vs) begin
            g4_p1 <= 8'd0; g4_p2 <= 8'd0; g4_r1 <= 8'd0;
            vs <= 10'd0; vs_a <= 10'd0; vs_b <= 10'd0;
            g4_ref <= 8'd0; mean_ce <= 1'b0;
        end else begin
            mean_ce <= 1'b0;
            if (g4_ce) begin
                g4_p1 <= gb0[x4];
                gb0[x4] <= g4_d;
                g4_p2 <= gb1[x4];
                gb1[x4] <= gb0[x4];
                vs    <= vs_now;
                vs_a  <= vs;
                vs_b  <= vs_a;
                g4_r1  <= g4_p1;            // 一级
                g4_ref <= g4_r1;            // 两级 -> (x-1, y-1)
                mean_ce <= 1'b1;
            end
        end
    end

    //=========================================================================
    // 3) 掩码: 局部均值 - 像素 > 阈值, 再做因果 3x3 膨胀
    //=========================================================================
    reg [7:0] thr;
    reg [7:0] mb0 [0:SW-1];
    reg       m_prev, m_up, m_upr;
    reg       mask_bit, mask_vld;

    wire [7:0] diff  = (mean9 > g4_ref) ? (mean9 - g4_ref) : 8'd0;
    wire       raw_m = (diff > thr);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            thr <= THR_Q;
            m_prev <= 1'b0; m_up <= 1'b0; m_upr <= 1'b0;
            mask_bit <= 1'b0; mask_vld <= 1'b0;
        end else begin
            thr <= (i_thr != 8'd0) ? i_thr : THR_Q;
            mask_vld <= 1'b0;
            if (i_vs) begin
                m_prev <= 1'b0; m_up <= 1'b0; m_upr <= 1'b0;
            end else if (mean_ce) begin
                m_up    <= mb0[x4];
                m_upr   <= mb0[(x4 == SW_LAST) ? 10'd0 : (x4 + 10'd1)];
                mb0[x4] <= raw_m;
                m_prev  <= raw_m;                // 仅延迟一拍(左邻), 不能自锁
                mask_bit <= raw_m | m_prev | m_up | m_upr;
                mask_vld <= 1'b1;
            end
        end
    end

    //=========================================================================
    // 4) 掩码行 -> 位图, 一整行填满后交给状态机
    //=========================================================================
    reg [SW-1:0] cur_row, sup;
    reg          sup_ready;
    reg [8:0]    y_sup;
    reg [8:0]    y_g4;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_row <= {SW{1'b0}}; sup <= {SW{1'b0}};
            sup_ready <= 1'b0; y_sup <= 9'd0; y_g4 <= 9'd0;
        end else if (i_vs) begin
            cur_row <= {SW{1'b0}}; sup_ready <= 1'b0; y_sup <= 9'd0; y_g4 <= 9'd0;
        end else begin
            if (mask_vld) begin
                if (x4 == SW_LAST) begin
                    sup       <= {cur_row[SW-2:0], mask_bit};
                    sup_ready <= 1'b1;
                    cur_row   <= {SW{1'b0}};
                    y_sup     <= y_g4;          // 该行 mask 对应的超行号
                    y_g4      <= y_g4 + 9'd1;
                end else begin
                    cur_row <= {cur_row[SW-2:0], mask_bit};
                end
            end
            if ((st != S_IDLE) && !mask_vld) sup_ready <= 1'b0;
        end
    end

    //=========================================================================
    // 5) 连通域: 游程 + 并查集
    //=========================================================================
    reg [12:0] L_n    [0:LAB-1];
    reg [9:0]  L_x0   [0:LAB-1];
    reg [9:0]  L_x1   [0:LAB-1];
    reg [8:0]  L_y0   [0:LAB-1];
    reg [8:0]  L_y1   [0:LAB-1];
    reg [19:0] L_span [0:LAB-1];
    reg [9:0]  L_ms   [0:LAB-1];
    reg [9:0]  L_rmin [0:LAB-1];
    reg [9:0]  L_rmax [0:LAB-1];
    reg        L_seen [0:LAB-1];
    reg        L_used [0:LAB-1];
    reg [5:0]  par    [0:LAB-1];

    reg [9:0] pa [0:RMAX-1];
    reg [9:0] pb [0:RMAX-1];
    reg [5:0] pl [0:RMAX-1];
    reg [9:0] ca [0:RMAX-1];
    reg [9:0] cb [0:RMAX-1];
    reg [5:0] cl [0:RMAX-1];
    reg [6:0] nprev, ncur;

    reg  [5:0]  idx;
    reg [12:0] s_n;
    reg [9:0]  s_x0, s_x1, s_ms, s_rmin, s_rmax;
    reg [8:0]  s_y0, s_y1;
    reg [19:0] s_span;
    reg        s_seen, s_used;

    always @(*) begin
        s_n = 13'd0; s_x0 = 10'h3FF; s_x1 = 10'd0;
        s_y0 = 9'h1FF; s_y1 = 9'd0; s_span = 20'd0; s_ms = 10'd1;
        s_rmin = 10'h3FF; s_rmax = 10'd0; s_seen = 1'b0; s_used = 1'b0;
        for (q = 0; q < LAB; q = q + 1) begin
            if (idx == q[5:0]) begin
                s_n    = L_n[q];
                s_x0   = L_x0[q];   s_x1 = L_x1[q];
                s_y0   = L_y0[q];   s_y1 = L_y1[q];
                s_span = L_span[q]; s_ms = L_ms[q];
                s_rmin = L_rmin[q]; s_rmax = L_rmax[q];
                s_seen = L_seen[q]; s_used = L_used[q];
            end
        end
    end

    reg [5:0] free_lab;
    reg       free_any;

    always @(*) begin
        free_lab = 6'd0;
        free_any = 1'b0;
        for (q = LAB-1; q >= 0; q = q - 1) begin
            if (!L_used[q]) begin
                free_lab = q[5:0];
                free_any = 1'b1;
            end
        end
    end

    localparam [2:0] S_IDLE  = 3'd0,
                     S_SCAN  = 3'd1,
                     S_LOAD  = 3'd2,
                     S_MERGE = 3'd3,
                     S_UPD   = 3'd4,
                     S_EOR   = 3'd5,
                     S_NEXT  = 3'd6;

    reg [2:0] st;
    reg [9:0] scan_k;
    reg       prev_bit;
    reg [9:0] run_s;
    reg [6:0] cur_idx, scan_j;
    reg [5:0] best_lab;
    reg       have_lab;
    reg [5:0] row_cnt;
    reg [8:0] y_now;
    reg       fe_req;

    wire [9:0] ca_i = ca[cur_idx[5:0]];
    wire [9:0] cb_i = cb[cur_idx[5:0]];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; scan_k <= 10'd0; prev_bit <= 1'b0;
            run_s <= 10'd0; ncur <= 7'd0; nprev <= 7'd0;
            cur_idx <= 7'd0; scan_j <= 7'd0; best_lab <= 6'd0;
            have_lab <= 1'b0; row_cnt <= 6'd0; idx <= 6'd0; y_now <= 9'd0;
            fe_req <= 1'b0;
            for (q = 0; q < LAB; q = q + 1) begin
                L_n[q] <= 13'd0; L_x0[q] <= 10'h3FF; L_x1[q] <= 10'd0;
                L_y0[q] <= 9'h1FF; L_y1[q] <= 9'd0; L_span[q] <= 20'd0;
                L_ms[q] <= 10'd1; L_rmin[q] <= 10'h3FF; L_rmax[q] <= 10'd0;
                L_seen[q] <= 1'b0; L_used[q] <= 1'b0; par[q] <= q[5:0];
            end
        end else if (i_vs) begin
            for (q = 0; q < LAB; q = q + 1) begin
                L_n[q] <= 13'd0; L_x0[q] <= 10'h3FF; L_x1[q] <= 10'd0;
                L_y0[q] <= 9'h1FF; L_y1[q] <= 9'd0; L_span[q] <= 20'd0;
                L_ms[q] <= 10'd1; L_rmin[q] <= 10'h3FF; L_rmax[q] <= 10'd0;
                L_seen[q] <= 1'b0; L_used[q] <= 1'b0; par[q] <= q[5:0];
            end
            nprev <= 7'd0; ncur <= 7'd0; fe_req <= 1'b0; st <= S_IDLE;
        end else begin
            case (st)
            S_IDLE: if (sup_ready && i_en) begin
                scan_k <= 10'd0; ncur <= 7'd0; prev_bit <= 1'b0; run_s <= 10'd0;
                y_now <= y_sup;
                st <= S_SCAN;
            end
            S_SCAN: begin
                if (sup[SW_LAST - scan_k]) begin
                    if (!prev_bit) run_s <= scan_k;
                    prev_bit <= 1'b1;
                    if (scan_k == SW_LAST) begin
                        if (ncur < RMAX) begin
                            ca[ncur[5:0]] <= run_s;
                            cb[ncur[5:0]] <= scan_k;
                            cl[ncur[5:0]] <= 6'd0;
                            ncur <= ncur + 7'd1;
                        end
                        cur_idx <= 7'd0; st <= S_LOAD;
                    end else scan_k <= scan_k + 10'd1;
                end else begin
                    if (prev_bit) begin
                        if (ncur < RMAX) begin
                            ca[ncur[5:0]] <= run_s;
                            cb[ncur[5:0]] <= scan_k - 10'd1;
                            cl[ncur[5:0]] <= 6'd0;
                            ncur <= ncur + 7'd1;
                        end
                        prev_bit <= 1'b0;
                    end
                    if (scan_k == SW_LAST) begin
                        cur_idx <= 7'd0; st <= S_LOAD;
                    end else scan_k <= scan_k + 10'd1;
                end
            end
            S_LOAD: begin
                if (cur_idx >= ncur) begin
                    row_cnt <= 6'd0; st <= S_EOR;
                end else begin
                    scan_j <= 7'd0; have_lab <= 1'b0; best_lab <= 6'd63;
                    st <= S_MERGE;
                end
            end
            S_MERGE: begin
                if (scan_j < nprev) begin
                    if ((pa[scan_j[5:0]] <= cb_i + 10'd1) &&
                        (pb[scan_j[5:0]] + 10'd1 >= ca_i)) begin
                        if (!have_lab) begin
                            best_lab <= pl[scan_j[5:0]];
                            have_lab <= 1'b1;
                        end else if (pl[scan_j[5:0]] < best_lab) begin
                            par[best_lab] <= pl[scan_j[5:0]];
                            best_lab <= pl[scan_j[5:0]];
                        end else if (pl[scan_j[5:0]] != best_lab) begin
                            par[pl[scan_j[5:0]]] <= best_lab;
                        end
                    end
                    scan_j <= scan_j + 7'd1;
                end else if (!have_lab) begin
                    if (!free_any) begin
                        cl[cur_idx[5:0]] <= 6'd63;
                        cur_idx <= cur_idx + 7'd1;
                        st <= S_LOAD;
                    end else begin
                        idx <= free_lab; st <= S_UPD;
                    end
                end else begin
                    idx <= best_lab; st <= S_UPD;
                end
            end
            S_UPD: begin
                cl[cur_idx[5:0]] <= idx;
                L_used[idx] <= 1'b1;
                L_n[idx]    <= (s_used ? s_n : 13'd0) + {3'd0, cb_i - ca_i + 10'd1};
                L_x0[idx]   <= (ca_i < s_x0) ? ca_i : s_x0;
                L_x1[idx]   <= (cb_i > s_x1) ? cb_i : s_x1;
                L_y0[idx]   <= s_used ? s_y0 : {1'b0, y_now};
                L_y1[idx]   <= {1'b0, y_now};
                L_rmin[idx] <= (ca_i < s_rmin) ? ca_i : s_rmin;
                L_rmax[idx] <= (cb_i > s_rmax) ? cb_i : s_rmax;
                L_ms[idx]   <= s_ms;
                L_seen[idx] <= 1'b1;
                cur_idx <= cur_idx + 7'd1;
                st <= S_LOAD;
            end
            S_EOR: begin
                if (row_cnt >= LAB_Q) begin
                    row_cnt <= 6'd0; st <= S_NEXT;
                end else begin
                    if (L_seen[row_cnt] && L_used[row_cnt]) begin
                        L_span[row_cnt] <= L_span[row_cnt] +
                                           (L_rmax[row_cnt] - L_rmin[row_cnt] + 10'd1);
                        if ((L_rmax[row_cnt] - L_rmin[row_cnt] + 10'd1) > L_ms[row_cnt])
                            L_ms[row_cnt] <= L_rmax[row_cnt] - L_rmin[row_cnt] + 10'd1;
                    end
                    L_rmin[row_cnt] <= 10'h3FF;
                    L_rmax[row_cnt] <= 10'd0;
                    L_seen[row_cnt] <= 1'b0;
                    row_cnt <= row_cnt + 6'd1;
                end
            end
            S_NEXT: begin
                nprev <= ncur;
                for (q = 0; q < RMAX; q = q + 1) begin
                    pa[q] <= ca[q]; pb[q] <= cb[q]; pl[q] <= cl[q];
                end
                for (q = 0; q < LAB; q = q + 1)
                    par[q] <= par[par[q]];
                if (y_now >= SH_M1) fe_req <= 1'b1;
                st <= S_IDLE;
            end
            default: st <= S_IDLE;
            endcase
        end
    end

    //=========================================================================
    // 6) 帧末: fill = span/(h*maxspan), 两阈值分类
    //=========================================================================
    localparam [1:0] F_IDLE = 2'd0, F_DIV = 2'd1, F_DONE = 2'd2;

    reg [1:0]  fst;
    reg        fe_done;
    reg [5:0]  f_lab;
    reg [2:0]  bcls [0:LAB-1];

    reg [31:0] dv_num, dv_den, dv_rem, dv_q, dv_quot;
    reg [5:0]  dv_i;
    reg        dv_start, dv_busy, dv_done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dv_rem <= 32'd0; dv_q <= 32'd0; dv_quot <= 32'd0;
            dv_i <= 6'd0; dv_busy <= 1'b0; dv_done <= 1'b0;
        end else begin
            dv_done <= 1'b0;
            if (dv_start) begin
                dv_rem <= 32'd0; dv_q <= 32'd0; dv_i <= 6'd31; dv_busy <= 1'b1;
            end else if (dv_busy) begin
                if ({dv_rem[30:0], dv_num[dv_i[4:0]]} >= dv_den) begin
                    dv_rem <= {dv_rem[30:0], dv_num[dv_i[4:0]]} - dv_den;
                    dv_q   <= {dv_q[30:0], 1'b1};
                end else begin
                    dv_rem <= {dv_rem[30:0], dv_num[dv_i[4:0]]};
                    dv_q   <= {dv_q[30:0], 1'b0};
                end
                if (dv_i == 6'd0) begin
                    dv_busy <= 1'b0; dv_done <= 1'b1;
                    dv_quot <= {dv_q[30:0], 1'b0};
                end else dv_i <= dv_i - 6'd1;
            end
        end
    end

    wire [9:0] lab_w = L_x1[f_lab] - L_x0[f_lab] + 10'd1;
    wire [9:0] lab_h = L_y1[f_lab] - L_y0[f_lab] + 10'd1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fst <= F_IDLE; f_lab <= 6'd0; fe_done <= 1'b0; dv_start <= 1'b0;
            for (q = 0; q < LAB; q = q + 1) bcls[q] <= 3'd0;
        end else if (i_vs) begin
            fst <= F_IDLE; f_lab <= 6'd0; fe_done <= 1'b0;
            for (q = 0; q < LAB; q = q + 1) bcls[q] <= 3'd0;
        end else begin
            dv_start <= 1'b0;
            case (fst)
            F_IDLE: begin
                if (i_en && fe_req && !fe_done) begin
                    f_lab <= 6'd0; fst <= F_DIV;
                end
            end
            F_DIV: begin
                if (f_lab >= LAB_Q) begin
                    fst <= F_DONE;
                end else if (!L_used[f_lab] || (L_span[f_lab] < SPAN_MIN) ||
                             (lab_w < WH_MIN) || (lab_h < WH_MIN)) begin
                    bcls[f_lab] <= 3'd0;
                    f_lab <= f_lab + 6'd1;
                end else if (dv_start || dv_busy) begin
                    // 等除法
                end else if (!dv_done) begin
                    dv_num <= {12'd0, L_span[f_lab]} << 8;
                    dv_den <= {22'd0, lab_h} * {22'd0, L_ms[f_lab]};
                    dv_start <= 1'b1;
                end else begin
                    // 溢出(>1.0) 说明 maxspan 估计偏小, 按矩形算
                    bcls[f_lab] <= (dv_quot[31:8] != 24'd0) ? 3'd2 :
                                   (dv_quot[7:0] < 8'd169)  ? 3'd3 :
                                   (dv_quot[7:0] < 8'd220)  ? 3'd1 : 3'd2;
                    f_lab <= f_lab + 6'd1;
                end
            end
            F_DONE: begin fst <= F_IDLE; fe_done <= 1'b1; end
            default: fst <= F_IDLE;
            endcase
        end
    end

    //=========================================================================
    // 7) 绘制
    //=========================================================================
    reg [9:0] bx0 [0:LAB-1], bx1 [0:LAB-1];
    reg [8:0] by0 [0:LAB-1], by1 [0:LAB-1];
    reg       bhit[0:LAB-1];
    reg       draw_arm;
    reg [9:0] px_cnt;
    reg [10:0] py_cnt;
    reg        de_z;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (q = 0; q < LAB; q = q + 1) bhit[q] <= 1'b0;
            draw_arm <= 1'b0; px_cnt <= 10'd0;
        end else if (i_vs) begin
            bhit[0] <= 1'b0;   // 逐帧由下一行覆盖, 这里先清第一个避免多驱动
            draw_arm <= 1'b0; px_cnt <= 10'd0;
        end else begin
            if (i_de) px_cnt <= (px_cnt == SW_LAST) ? 10'd0 : (px_cnt + 10'd1);
            if (fst == F_DONE) begin
                for (q = 0; q < LAB; q = q + 1) begin
                    bx0[q]  <= L_x0[q];
                    bx1[q]  <= L_x1[q];
                    by0[q]  <= (L_y0[q] > YOFS_Q) ? (L_y0[q] - YOFS_Q) : 9'd0;
                    by1[q]  <= (L_y1[q] > YOFS_Q) ? (L_y1[q] - YOFS_Q) : 9'd0;
                    bhit[q] <= bcls[q][2:0] != 3'd0;
                end
                draw_arm <= 1'b1;
            end
        end
    end

    wire [8:0] y_pix = py_cnt[10:2];

    reg [LAB-1:0] on_edge;
    reg           any_edge;

    always @(*) begin
        on_edge = {LAB{1'b0}};
        if (draw_arm && i_en && i_de) begin
            for (q = 0; q < LAB; q = q + 1) begin
                if (bhit[q] &&
                    (px_cnt >= bx0[q]) && (px_cnt <= bx1[q]) &&
                    (y_pix >= by0[q]) && (y_pix <= by1[q]) &&
                    ((px_cnt <= bx0[q] + 10'd1) ||
                     (px_cnt + 10'd1 >= bx1[q]) ||
                     (y_pix  <= by0[q] + 9'd1)  ||
                     (y_pix  + 9'd1  >= by1[q])))
                    on_edge[q] = 1'b1;
            end
        end
        any_edge = |on_edge;
    end

    function [23:0] box_color;
        input [2:0] c;
        begin
            case (c)
            3'd1: box_color = 24'h00FF00;   // 圆 -> 绿
            3'd2: box_color = 24'hFF8000;   // 矩形 -> 橙
            3'd3: box_color = 24'hFF0000;   // 三角形 -> 红
            default: box_color = 24'h00FFFF;
            endcase
        end
    endfunction

    reg [23:0] col_sel;
    always @(*) begin
        col_sel = 24'h00FFFF;
        for (q = 0; q < LAB; q = q + 1)
            if (on_edge[q]) col_sel = box_color(bcls[q]);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_de <= 1'b0; o_hs <= 1'b0; o_vs <= 1'b0; o_rgb <= 24'd0;
            o_shape_hit <= 1'b0;
        end else begin
            o_de <= i_de; o_hs <= i_hs; o_vs <= i_vs;
            o_rgb <= (i_en && any_edge) ? col_sel : i_rgb;
            if (i_vs) o_shape_hit <= 1'b0;
            else if (fst == F_DONE) o_shape_hit <= 1'b1;
        end
    end

    assign o_dbg = {1'b0, bcls[0]};

endmodule
