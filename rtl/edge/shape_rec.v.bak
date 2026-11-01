//=============================================================================
// shape_rec.v  —— 连通域 + 归一化几何特征 + 决策树 形状识别
//
// 为什么换算法
// ---------------------------------------------------------------------------
// 旧 shape_proc 是"固定半径 Hough 投票 + 固定长度水平行程 + 5 种长宽比假设",
// 参数按靶纸尺寸手工凑出(半径 40..184 共 10 档、长边 160..360 像素、三角形
// 必须正立). 目标一大一小、转一点角度或换一张靶纸, 参数就全部失配; 而且一帧
// 只验证一个假设, 需要多帧轮转, 响应慢、误检多.
//
// 这里改成尺度/位置无关的做法:
//   1) 边缘图 4x4 降采样到 480x270, 逐行抽取水平游程(run);
//   2) 游程按 x 重叠 + 相邻行做并查集合并 -> 连通域(最多 8 个);
//   3) 每个连通域累计 轮廓像素数 n / 外接框 / 行跨度积分 span / 最大行跨度 ms;
//   4) 归一化成 4 个无量纲特征, 送离线训练好的决策树(shape_tree.vh).
//
// 四个特征(整数, 与 tools/recog_study.py 逐位一致):
//   f1 extent      = 256*span/(w*h)    椭圆 193 / 方 158 / 三角 121 / 负样本 58
//   f2 compactness = 16*n*n/span       圆 174 / 方 212 / 三角 275 / 负样本 970
//   f3 aspect      = 256*w/h
//   f4 solidity    = 256*span/(ms*ms)
//
// 离线对比(同一批 1600 样本、同一划分):
//   几何特征 + 决策树   acc=0.925   31 节点, 无乘法器阵列
//   32x32 图块 CNN      acc=0.922   163k MAC, 6484 参数
// 精度相当而资源差三个数量级, 故选前者(详见 docs/recognition.md).
//
// 时序: 一个超行(4 行原始像素, 约 7680 周期)处理一次, 状态机约 1300 周期,
//       余量充足; 帧末做 8 个连通域 x 4 次除法 + 树判决, 约 1200 周期.
//=============================================================================
`timescale 1ns/1ps

module shape_rec #(
    parameter integer H_ACTIVE = 1920,
    parameter integer V_ACTIVE = 1080,
    parameter integer MIN_SPAN = 24,      // 1/4 尺度最小面积
    parameter integer MAX_SPAN = 12000,   // 超过视为板框/桌面
    parameter integer LAB      = 8,       // 连通域个数
    parameter integer RMAX     = 24       // 每行最多游程数
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

    output reg         o_de,
    output reg         o_hs,
    output reg         o_vs,
    output reg  [23:0] o_rgb,
    output reg         o_shape_hit,
    output wire [3:0]  o_dbg
);

    localparam integer SW = H_ACTIVE / 4;
    localparam [9:0]   SW_LAST = SW[9:0] - 10'd1;

    integer q;

    //=========================================================================
    // 1) 4x4 降采样
    //=========================================================================
    reg  [1:0]    subrow, colsub;
    reg           acc_e;
    reg  [SW-1:0] cur_row, v_acc, sup;
    reg           sup_ready, de_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            subrow <= 2'd0; colsub <= 2'd0; acc_e <= 1'b0;
            cur_row <= {SW{1'b0}}; v_acc <= {SW{1'b0}};
            sup <= {SW{1'b0}}; sup_ready <= 1'b0; de_d <= 1'b0;
        end else begin
            de_d <= i_de;
            if (i_de) begin
                colsub <= colsub + 2'd1;
                acc_e  <= acc_e | i_edge;
                if (colsub == 2'd3) begin
                    cur_row <= {cur_row[SW-2:0], acc_e | i_edge};
                    acc_e   <= 1'b0;
                end
            end else begin
                colsub <= 2'd0;
                acc_e  <= 1'b0;
                if (de_d) begin
                    if (subrow == 2'd3) begin
                        sup       <= v_acc | cur_row;
                        sup_ready <= 1'b1;
                        v_acc     <= {SW{1'b0}};
                        subrow    <= 2'd0;
                    end else begin
                        v_acc  <= v_acc | cur_row;
                        subrow <= subrow + 2'd1;
                    end
                    cur_row <= {SW{1'b0}};
                end
            end
            if (i_vs) sup_ready <= 1'b0;
            else if ((st != S_IDLE) && !((!i_de) && de_d && (subrow == 2'd3)))
                sup_ready <= 1'b0;          // 状态机已取走本行
        end
    end

    //=========================================================================
    // 2) 连通域状态
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
    reg [2:0]  par    [0:LAB-1];

    reg [9:0] pa [0:RMAX-1];
    reg [9:0] pb [0:RMAX-1];
    reg [2:0] pl [0:RMAX-1];
    reg [9:0] ca [0:RMAX-1];
    reg [9:0] cb [0:RMAX-1];
    reg [2:0] cl [0:RMAX-1];
    reg [5:0] nprev, ncur;

    // 8 选 1 读
    reg  [2:0]  idx;
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
            if (idx == q[2:0]) begin
                s_n    = L_n[q];
                s_x0   = L_x0[q];   s_x1 = L_x1[q];
                s_y0   = L_y0[q];   s_y1 = L_y1[q];
                s_span = L_span[q]; s_ms = L_ms[q];
                s_rmin = L_rmin[q]; s_rmax = L_rmax[q];
                s_seen = L_seen[q]; s_used = L_used[q];
            end
        end
    end

    always @(*) begin
        free_lab = 3'd7;
        free_any = 1'b0;
        for (q = LAB-1; q >= 0; q = q - 1) begin
            if (!L_used[q]) begin
                free_lab = q[2:0];
                free_any = 1'b1;
            end
        end
    end

    //=========================================================================
    // 3) 行处理状态机
    //=========================================================================
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
    reg [5:0] cur_idx, scan_j;
    reg [2:0] best_lab;
    reg       have_lab;
    // 组合选空闲连通域(取最小空闲编号), 避免在时序块里用非阻塞赋值互相打架
    reg [2:0] free_lab;
    reg       free_any;
    reg [3:0] row_cnt;
    reg [8:0] y_now;

    wire [9:0] ca_i = ca[cur_idx[4:0]];
    wire [9:0] cb_i = cb[cur_idx[4:0]];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; scan_k <= 10'd0; prev_bit <= 1'b0;
            run_s <= 10'd0; ncur <= 6'd0; nprev <= 6'd0;
            cur_idx <= 6'd0; scan_j <= 6'd0; best_lab <= 3'd0;
            have_lab <= 1'b0;
            row_cnt <= 4'd0; idx <= 3'd0; y_now <= 9'd0;
            for (q = 0; q < LAB; q = q + 1) begin
                L_n[q] <= 13'd0; L_x0[q] <= 10'h3FF; L_x1[q] <= 10'd0;
                L_y0[q] <= 9'h1FF; L_y1[q] <= 9'd0; L_span[q] <= 20'd0;
                L_ms[q] <= 10'd1; L_rmin[q] <= 10'h3FF; L_rmax[q] <= 10'd0;
                L_seen[q] <= 1'b0; L_used[q] <= 1'b0; par[q] <= q[2:0];
            end
        end else if (i_vs) begin
            for (q = 0; q < LAB; q = q + 1) begin
                L_n[q] <= 13'd0; L_x0[q] <= 10'h3FF; L_x1[q] <= 10'd0;
                L_y0[q] <= 9'h1FF; L_y1[q] <= 9'd0; L_span[q] <= 20'd0;
                L_ms[q] <= 10'd1; L_rmin[q] <= 10'h3FF; L_rmax[q] <= 10'd0;
                L_seen[q] <= 1'b0; L_used[q] <= 1'b0; par[q] <= q[2:0];
            end
            nprev <= 6'd0; ncur <= 6'd0;
            fe_req <= 1'b0;
            st <= S_IDLE;
        end else begin
            case (st)
            //-----------------------------------------------------------------
            S_IDLE: if (sup_ready && i_en) begin
                scan_k <= 10'd0; ncur <= 6'd0; prev_bit <= 1'b0; run_s <= 10'd0;
                y_now <= i_row[10:2];   // 超行号 = 行号/4
                st <= S_SCAN;
            end
            //-----------------------------------------------------------------
            S_SCAN: begin
                if (sup[SW_LAST - scan_k]) begin
                    if (!prev_bit) run_s <= scan_k;
                    prev_bit <= 1'b1;
                    if (scan_k == SW_LAST) begin
                        // 行尾仍在游程内
                        if (ncur < RMAX) begin
                            ca[ncur[4:0]] <= run_s;
                            cb[ncur[4:0]] <= scan_k;
                            cl[ncur[4:0]] <= 3'd7;
                            ncur <= ncur + 6'd1;
                        end
                        cur_idx <= 6'd0; st <= S_LOAD;
                    end else scan_k <= scan_k + 10'd1;
                end else begin
                    if (prev_bit) begin
                        if (ncur < RMAX) begin
                            ca[ncur[4:0]] <= run_s;
                            cb[ncur[4:0]] <= scan_k - 10'd1;
                            cl[ncur[4:0]] <= 3'd7;
                            ncur <= ncur + 6'd1;
                        end
                        prev_bit <= 1'b0;
                    end
                    if (scan_k == SW_LAST) begin
                        cur_idx <= 6'd0; st <= S_LOAD;
                    end else scan_k <= scan_k + 10'd1;
                end
            end
            //-----------------------------------------------------------------
            S_LOAD: begin
                if (cur_idx >= ncur) begin
                    row_cnt <= 4'd0;
                    st <= S_EOR;
                end else begin
                    scan_j <= 6'd0; have_lab <= 1'b0; best_lab <= 3'd7;
                    st <= S_MERGE;
                end
            end
            //-----------------------------------------------------------------
            S_MERGE: begin
                if (scan_j < nprev) begin
                    if ((pa[scan_j[4:0]] <= cb_i + 10'd1) &&
                        (pb[scan_j[4:0]] + 10'd1 >= ca_i)) begin
                        if (!have_lab) begin
                            best_lab <= pl[scan_j[4:0]];
                            have_lab <= 1'b1;
                        end else if (pl[scan_j[4:0]] < best_lab) begin
                            par[best_lab] <= pl[scan_j[4:0]];
                            best_lab <= pl[scan_j[4:0]];
                        end else if (pl[scan_j[4:0]] != best_lab) begin
                            par[pl[scan_j[4:0]]] <= best_lab;
                        end
                    end
                    scan_j <= scan_j + 6'd1;
                end else if (!have_lab) begin
                    if (!free_any) begin
                        // 没匹配也没空位: 丢弃
                        cl[cur_idx[4:0]] <= 3'd7;
                        cur_idx <= cur_idx + 6'd1;
                        st <= S_LOAD;
                    end else begin
                        idx <= free_lab;
                        st <= S_UPD;
                    end
                end else begin
                    idx <= best_lab;
                    st <= S_UPD;
                end
            end
            //-----------------------------------------------------------------
            S_UPD: begin
                cl[cur_idx[4:0]] <= idx;
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
                cur_idx <= cur_idx + 6'd1;
                st <= S_LOAD;
            end
            //-----------------------------------------------------------------
            S_EOR: begin
                if (row_cnt == LAB[3:0]) begin
                    row_cnt <= 4'd0;
                    st <= S_NEXT;
                end else begin
                    if (L_seen[row_cnt] && L_used[row_cnt]) begin
                        L_span[row_cnt] <= L_span[row_cnt] + (L_rmax[row_cnt] - L_rmin[row_cnt] + 10'd1);
                        if ((L_rmax[row_cnt] - L_rmin[row_cnt] + 10'd1) > L_ms[row_cnt])
                            L_ms[row_cnt] <= L_rmax[row_cnt] - L_rmin[row_cnt] + 10'd1;
                    end
                    L_rmin[row_cnt] <= 10'h3FF;
                    L_rmax[row_cnt] <= 10'd0;
                    L_seen[row_cnt] <= 1'b0;
                    row_cnt <= row_cnt + 4'd1;
                end
            end
            //-----------------------------------------------------------------
            S_NEXT: begin
                nprev <= ncur;
                for (q = 0; q < RMAX; q = q + 1) begin
                    pa[q] <= ca[q];
                    pb[q] <= cb[q];
                    pl[q] <= cl[q];
                end
                for (q = 0; q < LAB; q = q + 1)
                    par[q] <= par[par[q]];
                if (y_now >= (V_ACTIVE/4 - 1)) fe_req <= 1'b1;
                st <= S_IDLE;
            end
            default: st <= S_IDLE;
            endcase
        end
    end

    //=========================================================================
    // 4) 帧末: 归一化特征 + 决策树
    //=========================================================================
    localparam [1:0] F_IDLE = 2'd0, F_DIV = 2'd1, F_TREE = 2'd2, F_DONE = 2'd3;

    reg [1:0]  fst;
    reg        fe_done;
    reg        fe_req;   // 最后一个超行处理完才请求帧末判决
    reg [3:0]  f_lab;
    reg [2:0]  f_step;
    reg [15:0] fv [0:3];
    reg [19:0] fspan;
    reg [5:0]  tnode;

    // 顺序除法器: restoring, 32 次迭代, start/busy/done 握手
    reg  [31:0] dv_num, dv_den, dv_rem, dv_q, dv_quot;
    reg  [5:0]  dv_i;
    reg         dv_start, dv_busy, dv_done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dv_rem <= 32'd0; dv_q <= 32'd0; dv_quot <= 32'd0;
            dv_i <= 6'd0; dv_busy <= 1'b0; dv_done <= 1'b0;
        end else begin
            dv_done <= 1'b0;
            if (dv_start) begin
                dv_rem  <= 32'd0;
                dv_q    <= 32'd0;
                dv_i    <= 6'd31;
                dv_busy <= 1'b1;
            end else if (dv_busy) begin
                if ({dv_rem[30:0], dv_num[dv_i[4:0]]} >= dv_den) begin
                    dv_rem <= {dv_rem[30:0], dv_num[dv_i[4:0]]} - dv_den;
                    dv_q   <= {dv_q[30:0], 1'b1};
                end else begin
                    dv_rem <= {dv_rem[30:0], dv_num[dv_i[4:0]]};
                    dv_q   <= {dv_q[30:0], 1'b0};
                end
                if (dv_i == 6'd0) begin
                    dv_busy <= 1'b0;
                    dv_done <= 1'b1;
                    dv_quot <= {dv_q[30:0], (1'b0)};
                end else begin
                    dv_i <= dv_i - 6'd1;
                end
            end
        end
    end

    // 每个连通域派生的宽高
    function [9:0] lab_w;
        input [2:0] l;
        begin lab_w = L_x1[l] - L_x0[l] + 10'd1; end
    endfunction
    function [9:0] lab_h;
        input [2:0] l;
        begin lab_h = {1'b0, L_y1[l]} - {1'b0, L_y0[l]} + 10'd1; end
    endfunction

    // 三类最优结果(按 span 取大)
    reg        v_circ, v_rect, v_tri;
    reg [9:0]  bx0 [0:2], by0 [0:2], bx1 [0:2], by1 [0:2];
    reg [19:0] bspan [0:2];

    wire [39:0] rec = tree_node(tnode);
    wire [1:0]  t_cls = rec[38:37];
    wire [15:0] t_sel = (rec[36:34] == 3'd0) ? fv[0] :
                        (rec[36:34] == 3'd1) ? fv[1] :
                        (rec[36:34] == 3'd2) ? fv[2] : fv[3];
    wire [5:0]  t_nxt = (t_sel <= rec[33:18]) ? rec[17:12] : rec[11:6];

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fst <= F_IDLE; f_lab <= 4'd0; f_step <= 3'd0; tnode <= 6'd0;
            fe_done <= 1'b0;
            dv_start <= 1'b0; dv_num <= 32'd0; dv_den <= 32'd1;
            v_circ <= 1'b0; v_rect <= 1'b0; v_tri <= 1'b0;
            fv[0] <= 16'd0; fv[1] <= 16'd0; fv[2] <= 16'd0; fv[3] <= 16'd0;
            fspan <= 20'd0;
            for (k = 0; k < 3; k = k + 1) begin
                bx0[k] <= 10'd0; by0[k] <= 10'd0; bx1[k] <= 10'd0; by1[k] <= 10'd0;
                bspan[k] <= 20'd0;
            end
        end else if (i_vs) begin
            fst <= F_IDLE; f_lab <= 4'd0; f_step <= 3'd0; fe_done <= 1'b0;
            v_circ <= 1'b0; v_rect <= 1'b0; v_tri <= 1'b0;
            dv_start <= 1'b0;
        end else begin
            dv_start <= 1'b0;
            case (fst)
            F_IDLE: begin
                if (i_en && fe_req && !fe_done) begin
                    f_lab  <= 4'd0;
                    fst    <= F_DIV;
                end
            end
            F_DIV: begin
                if (f_lab >= LAB[3:0]) begin
                    fst <= F_DONE;
                end else if (f_step == 3'd0) begin
                    if (!L_used[f_lab] ||
                        (L_span[f_lab] < MIN_SPAN) || (L_span[f_lab] > MAX_SPAN) ||
                        (lab_w(f_lab) < 10'd3) || (lab_h(f_lab) < 10'd3)) begin
                        f_lab <= f_lab + 4'd1;
                    end else begin
                        fspan  <= L_span[f_lab];
                        dv_num <= {12'd0, L_span[f_lab]} << 8;
                        dv_den <= {22'd0, lab_w(f_lab)} * {22'd0, lab_h(f_lab)};
                        dv_start <= 1'b1;
                        f_step <= 3'd1;
                    end
                end else if (!dv_done) begin
                    // 等除法完成
                end else begin
                    case (f_step)
                    3'd1: begin
                        fv[0] <= dv_q[31:16] ? 16'hFFFF : dv_q[15:0];
                        dv_num <= ({19'd0, L_n[f_lab]} * {19'd0, L_n[f_lab]}) << 4;
                        dv_den <= {12'd0, L_span[f_lab]};
                        dv_start <= 1'b1;
                        f_step <= 3'd2;
                    end
                    3'd2: begin
                        fv[1] <= dv_q[31:16] ? 16'hFFFF : dv_q[15:0];
                        dv_num <= {22'd0, lab_w(f_lab)} << 8;
                        dv_den <= {22'd0, lab_h(f_lab)};
                        dv_start <= 1'b1;
                        f_step <= 3'd3;
                    end
                    3'd3: begin
                        fv[2] <= dv_q[31:16] ? 16'hFFFF : dv_q[15:0];
                        dv_num <= {12'd0, L_span[f_lab]} << 8;
                        dv_den <= {22'd0, L_ms[f_lab]} * {22'd0, L_ms[f_lab]};
                        dv_start <= 1'b1;
                        f_step <= 3'd4;
                    end
                    default: begin
                        fv[3]  <= dv_q[31:16] ? 16'hFFFF : dv_q[15:0];
                        tnode  <= 6'd0;
                        f_step <= 3'd0;
                        fst    <= F_TREE;
                    end
                    endcase
                end
            end
            F_TREE: begin
                if (rec[39]) begin
                    // 叶子: 记录该类的最优框
                    case (t_cls)
                    2'd1: if (!v_circ || (fspan > bspan[0])) begin
                        v_circ <= 1'b1; bspan[0] <= fspan;
                        bx0[0] <= L_x0[f_lab]; bx1[0] <= L_x1[f_lab];
                        by0[0] <= {1'b0, L_y0[f_lab]}; by1[0] <= {1'b0, L_y1[f_lab]};
                    end
                    2'd2: if (!v_rect || (fspan > bspan[1])) begin
                        v_rect <= 1'b1; bspan[1] <= fspan;
                        bx0[1] <= L_x0[f_lab]; bx1[1] <= L_x1[f_lab];
                        by0[1] <= {1'b0, L_y0[f_lab]}; by1[1] <= {1'b0, L_y1[f_lab]};
                    end
                    2'd3: if (!v_tri || (fspan > bspan[2])) begin
                        v_tri <= 1'b1; bspan[2] <= fspan;
                        bx0[2] <= L_x0[f_lab]; bx1[2] <= L_x1[f_lab];
                        by0[2] <= {1'b0, L_y0[f_lab]}; by1[2] <= {1'b0, L_y1[f_lab]};
                    end
                    default: ;
                    endcase
                    f_lab <= f_lab + 4'd1;
                    fst   <= F_DIV;
                end else begin
                    tnode <= t_nxt;
                end
            end
            F_DONE: begin fst <= F_IDLE; fe_done <= 1'b1; end
            default: fst <= F_IDLE;
            endcase
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) o_shape_hit <= 1'b0;
        else        o_shape_hit <= v_circ | v_rect | v_tri;
    end

    //=== TREE_BEGIN ===
    // === 由 tools/recog_study.py 自动生成, 请勿手改 ===
    // [39]=leaf [38:37]=cls [36:34]=feat [33:18]=thr [17:12]=left [11:6]=right
    function [39:0] tree_node;
        input [5:0] idx;
        begin
            tree_node = 40'd0;
            case (idx)
                6'd00: tree_node = 40'h0001881580;
                6'd01: tree_node = 40'h0803A822C0;
                6'd02: tree_node = 40'h0000D03180;
                6'd03: tree_node = 40'h0000944140;
                6'd04: tree_node = 40'h8000000000;
                6'd05: tree_node = 40'h8000000000;
                6'd06: tree_node = 40'h0C02607280;
                6'd07: tree_node = 40'h0C01F48240;
                6'd08: tree_node = 40'hE000000000;
                6'd09: tree_node = 40'hE000000000;
                6'd10: tree_node = 40'hA000000000;
                6'd11: tree_node = 40'h080424C440;
                6'd12: tree_node = 40'h000138D400;
                6'd13: tree_node = 40'h000078E3C0;
                6'd14: tree_node = 40'h8000000000;
                6'd15: tree_node = 40'h8000000000;
                6'd16: tree_node = 40'h8000000000;
                6'd17: tree_node = 40'h0000D92540;
                6'd18: tree_node = 40'h0806D93500;
                6'd19: tree_node = 40'h8000000000;
                6'd20: tree_node = 40'h8000000000;
                6'd21: tree_node = 40'hE000000000;
                6'd22: tree_node = 40'h0C02797A80;
                6'd23: tree_node = 40'h00020188C0;
                6'd24: tree_node = 40'h0001F99780;
                6'd25: tree_node = 40'h0001E1A6C0;
                6'd26: tree_node = 40'hE000000000;
                6'd27: tree_node = 40'h0C0269C740;
                6'd28: tree_node = 40'hE000000000;
                6'd29: tree_node = 40'hE000000000;
                6'd30: tree_node = 40'h0803C9F800;
                6'd31: tree_node = 40'hE000000000;
                6'd32: tree_node = 40'h08043A1880;
                6'd33: tree_node = 40'hC000000000;
                6'd34: tree_node = 40'hE000000000;
                6'd35: tree_node = 40'h0803BA4940;
                6'd36: tree_node = 40'hE000000000;
                6'd37: tree_node = 40'h0C01F26A40;
                6'd38: tree_node = 40'h0C01C27A00;
                6'd39: tree_node = 40'hE000000000;
                6'd40: tree_node = 40'hE000000000;
                6'd41: tree_node = 40'hC000000000;
                6'd42: tree_node = 40'h0002F2BC80;
                6'd43: tree_node = 40'h08034ECB40;
                6'd44: tree_node = 40'hA000000000;
                6'd45: tree_node = 40'h0804AAEC40;
                6'd46: tree_node = 40'h080452FC00;
                6'd47: tree_node = 40'hC000000000;
                6'd48: tree_node = 40'hC000000000;
                6'd49: tree_node = 40'hA000000000;
                6'd50: tree_node = 40'h00032F3E00;
                6'd51: tree_node = 40'h0C03D34DC0;
                6'd52: tree_node = 40'h0C03AB5D80;
                6'd53: tree_node = 40'hA000000000;
                6'd54: tree_node = 40'hA000000000;
                6'd55: tree_node = 40'hC000000000;
                6'd56: tree_node = 40'hC000000000;
                default: tree_node = 40'h8000000000;
            endcase
        end
    endfunction
    //=== TREE_END ===

    //=========================================================================
    // 5) 叠加绘制
    //=========================================================================
    reg        d_de, d_hs, d_vs;
    reg [23:0] d_rgb;
    reg [11:0] d_col;
    reg        d_vs_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d_de <= 1'b0; d_hs <= 1'b0; d_vs <= 1'b0; d_rgb <= 24'd0;
            d_col <= 12'd0; d_vs_d <= 1'b0;
        end else begin
            d_de <= i_de; d_hs <= i_hs; d_vs <= i_vs; d_rgb <= i_rgb;
            d_vs_d <= d_vs;
            if (!d_de) d_col <= 12'd0;
            else       d_col <= d_col + 12'd1;
        end
    end

    wire [11:0] X0C = {bx0[0], 2'b00}, X1C = {bx1[0], 2'b00};
    wire [11:0] Y0C = {by0[0], 2'b00}, Y1C = {by1[0], 2'b00};
    wire [11:0] X0R = {bx0[1], 2'b00}, X1R = {bx1[1], 2'b00};
    wire [11:0] Y0R = {by0[1], 2'b00}, Y1R = {by1[1], 2'b00};
    wire [11:0] X0T = {bx0[2], 2'b00}, X1T = {bx1[2], 2'b00};
    wire [11:0] Y0T = {by0[2], 2'b00}, Y1T = {by1[2], 2'b00};

    function on_box;
        input [11:0] x, y, x0, y0, x1, y1;
        begin
            on_box = (x >= x0) && (x <= x1) && (y >= y0) && (y <= y1) &&
                     (((x - x0) < 2) || ((x1 - x) < 2) ||
                      ((y - y0) < 2) || ((y1 - y) < 2));
        end
    endfunction

    wire on_c = i_en && v_circ && on_box(d_col, {1'b0, i_row[10:0]}, X0C, Y0C, X1C, Y1C);
    wire on_r = i_en && v_rect && on_box(d_col, {1'b0, i_row[10:0]}, X0R, Y0R, X1R, Y1R);
    wire on_t = i_en && v_tri  && on_box(d_col, {1'b0, i_row[10:0]}, X0T, Y0T, X1T, Y1T);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_de <= 1'b0; o_hs <= 1'b0; o_vs <= 1'b0; o_rgb <= 24'd0;
        end else begin
            o_de <= d_de; o_hs <= d_hs; o_vs <= d_vs;
            if (on_c || on_r || on_t) o_rgb <= 24'h00FFFF;   // 青色框
            else                      o_rgb <= d_rgb;
        end
    end

    assign o_dbg = {v_circ, v_rect, v_tri, i_en};

endmodule
