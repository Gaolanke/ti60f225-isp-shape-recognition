//=============================================================================
// cam_ae_awb.v
//
// 自动白平衡(AWB) + 自动曝光(AE) 控制器
//
// 背景 / 为什么需要它
// ---------------------------------------------------------------------------
// 旧版本的白平衡是**编译期常量**: DEF_R_GAIN=1.5625x, DEF_B_GAIN=1.8125x,
// 与光照无关. 这套增益是在"夜间冷色灯"下凑出来的, 所以:
//   * 夜间冷光(R 偏少) -> 1.5625x 刚好补平 -> 颜色正常
//   * 白天阳光(R 偏多) -> 仍乘 1.5625x     -> 整体偏红
// 更关键的是 rgb_gain_v1 在 8bit 域做 Q8.8 乘法并饱和到 255: 白天 R 本来就高,
// 再乘 1.5625 直接削顶, 红通道丢失层次, 看起来就是"一片红".
// 手动调节只有 16 档 x 3.125% 的窄窗口, 且下限仍是 1.3125x, 压不回 1.0,
// 所以在阳光下怎么调都救不回来 —— 这就是"手动白平衡无效"的直接原因.
//
// 本模块的做法
// ---------------------------------------------------------------------------
// 1) 统计: 在去马赛克之后的 RGB 上按稀疏网格取样(每 8 个 beat、每 8 行取一个),
//    只在**近中性像素**上累加 (max-min <= max/4 + 4). 这样大面积彩色物体不会
//    把统计拉偏, 规避了 gray-world 最经典的失效模式.
// 2) 控制: 逐帧比较 S_R / S_G / S_B, 用带死区的增量式积分控制器调节 R/B 增益.
//    全程只有比较、移位和加法, **没有除法器**, 资源极小.
//    增益限制在 [128, 512] (0.5x ~ 2.0x), 绿色恒为 1.0x.
// 3) 锁定: 连续 N 帧落在死区内即判定收敛, o_awb_locked 给上层做指示.
// 4) AE: 同一批取样点统计亮度均值与削顶比例, 每 8 帧最多调整一次;
//    先动曝光(细调 12.5%), 曝光到边界再动模拟增益(粗调), 带死区与稳定等待,
//    不会来回抽动.
// 5) 手动优先: i_man_en=1 时冻结 AWB、停止 AE 请求、直接输出手动增益,
//    用户调完退出即可, 不会被自动环路拉回去.
//
// 时钟域: 全部在摄像头像素时钟 (vid_clk_dvi2) 下工作, 增益直接送 debayer 内的
//         rgb_gain_v1, 不需要跨时钟; AE 的曝光/增益请求用翻转握手交给 CLK_5M
//         域的 I2C 调度器.
//=============================================================================
`timescale 1ns/1ps

module cam_ae_awb #(
    // ---- 增益种子(上电初值), 取旧固定增益附近, 开机第一帧就不难看 ----
    parameter [15:0] SEED_R_GAIN     = 16'd384,   // 1.50x
    parameter [15:0] SEED_G_GAIN     = 16'd256,   // 1.00x
    parameter [15:0] SEED_B_GAIN     = 16'd448,   // 1.75x

    // ---- 增益范围 (Q8.8) ----
    parameter [15:0] GAIN_MIN        = 16'd128,   // 0.50x
    parameter [15:0] GAIN_MAX        = 16'd512,   // 2.00x

    // ---- 取样网格: 每 2^BEAT_DIV 个 beat、每 2^LINE_DIV 行取一个点 ----
    parameter integer BEAT_DIV       = 3,         // 8 beat = 16 像素
    parameter integer LINE_DIV       = 3,         // 8 行

    // ---- AWB ----
    parameter integer NEUTRAL_SHIFT  = 2,         // (max-min) <= (max>>N)+4
    parameter [23:0]  MIN_SAMPLES    = 24'd384,   // 少于此样本数本帧不更新
    parameter [15:0]  DEADBAND_Q     = 16'd16,    // 死区 16/256 = 6.25%
    parameter [3:0]   MAX_STEP       = 4'd8,      // 单帧最大增益步进 (Q8.8)
    parameter [7:0]   LOCK_FRAMES    = 8'd6,      // 连续满足死区的帧数 -> 锁定

    // ---- AE ----
    parameter [7:0]   AE_TOL_LO      = 8'd72,     // 低于此认为偏暗
    parameter [7:0]   AE_TOL_HI      = 8'd132,    // 高于此认为偏亮
    parameter [7:0]   AE_PERIOD      = 8'd8,      // 每 N 帧评估一次
    parameter [15:0]  AE_EXP_MIN     = 16'd187,
    parameter [15:0]  AE_EXP_MAX     = 16'd2989,
    parameter [15:0]  AE_EXP_SEED    = 16'h0BAD,
    parameter [7:0]   AE_GAIN_SEED_C = 8'h81,
    parameter [7:0]   AE_GAIN_SEED_F = 8'h32
)(
    input  wire        clk,          // 摄像头像素时钟 (vid_clk_dvi2)
    input  wire        rst_n,

    // 视频时序 + 去马赛克后 RGB (每个 beat 两个像素: {b,g,r,b,g,r})
    input  wire        i_de,
    input  wire        i_vs,
    input  wire [47:0] i_rgb2,

    // 手动覆盖
    input  wire        i_man_en,     // 1 = 冻结自动环路, 使用手动增益
    input  wire [15:0] i_man_r,
    input  wire [15:0] i_man_b,

    // ---- 自动白平衡结果 (送 rgb_gain_v1) ----
    output reg  [15:0] o_r_gain,
    output reg  [15:0] o_g_gain,
    output reg  [15:0] o_b_gain,

    // ---- 状态指示 ----
    output reg         o_awb_locked,
    output reg  [3:0]  o_awb_dbg,    // {locked, gain_sat, lowlight, few_samples}

    // ---- AE 请求 (翻转握手到 CLK_5M 域) ----
    output reg         o_ae_req_tgl,
    output reg  [15:0] o_ae_exp,
    output reg  [7:0]  o_ae_gain_c,
    output reg  [7:0]  o_ae_gain_f,
    input  wire        i_ae_ack_tgl,
    input  wire        i_ae_en       // 0 = 不发起 AE 写 (手动调节模式)
);

    //-------------------------------------------------------------------------
    // 取样网格
    //-------------------------------------------------------------------------
    reg              de_d;
    reg  [BEAT_DIV-1:0] beat_cnt;
    reg  [LINE_DIV-1:0] line_cnt;
    reg                 vs_d;

    wire vs_rise = i_vs & ~vs_d;
    wire eol     = de_d & ~i_de;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_d     <= 1'b0;
            beat_cnt <= {BEAT_DIV{1'b0}};
            line_cnt <= {LINE_DIV{1'b0}};
            vs_d     <= 1'b0;
        end else begin
            de_d <= i_de;
            vs_d <= i_vs;
            if (i_de) begin
                beat_cnt <= beat_cnt + 1'b1;
            end else begin
                beat_cnt <= {BEAT_DIV{1'b0}};
            end
            if (eol) begin
                line_cnt <= line_cnt + 1'b1;
            end
            if (vs_rise) begin
                line_cnt <= {LINE_DIV{1'b0}};
            end
        end
    end

    wire beat_hit  = (beat_cnt == {BEAT_DIV{1'b0}});
    wire line_hit  = (line_cnt == {LINE_DIV{1'b0}});
    wire sample_en = i_de && beat_hit && line_hit;

    //-------------------------------------------------------------------------
    // 取样像素 (一个 beat 两组)
    //-------------------------------------------------------------------------
    wire [7:0] r0 = i_rgb2[7:0];
    wire [7:0] g0 = i_rgb2[15:8];
    wire [7:0] b0 = i_rgb2[23:16];
    wire [7:0] r1 = i_rgb2[31:24];
    wire [7:0] g1 = i_rgb2[39:32];
    wire [7:0] b1 = i_rgb2[47:40];

    wire [7:0] mx0 = (r0 > g0) ? ((r0 > b0) ? r0 : b0) : ((g0 > b0) ? g0 : b0);
    wire [7:0] mn0 = (r0 < g0) ? ((r0 < b0) ? r0 : b0) : ((g0 < b0) ? g0 : b0);
    wire [7:0] mx1 = (r1 > g1) ? ((r1 > b1) ? r1 : b1) : ((g1 > b1) ? g1 : b1);
    wire [7:0] mn1 = (r1 < g1) ? ((r1 < b1) ? r1 : b1) : ((g1 < b1) ? g1 : b1);

    wire       neutral0 = ((mx0 - mn0) <= ((mx0 >> NEUTRAL_SHIFT) + 8'd4));
    wire       neutral1 = ((mx1 - mn1) <= ((mx1 >> NEUTRAL_SHIFT) + 8'd4));

    wire [7:0] y0 = ({1'b0, r0} + {g0, 1'b0} + {1'b0, b0}) >> 2;
    wire [7:0] y1 = ({1'b0, r1} + {g1, 1'b0} + {1'b0, b1}) >> 2;

    //-------------------------------------------------------------------------
    // 累加器 (帧内累加, 帧首清零)
    //-------------------------------------------------------------------------
    reg [23:0] acc_r, acc_g, acc_b;
    reg [23:0] acc_n;      // 近中性像素数
    reg [25:0] acc_y;      // 全部取样像素亮度和
    reg [23:0] acc_c;      // 削顶 beat 数
    reg [23:0] acc_nt;     // 全部取样像素数
    // 全画面通道和: 偏色严重时"近中性"门限会把所有点都挡掉(此时画面本来就不中性),
    // 造成自锁. 样本不足时退回 gray-world 用全画面统计做粗调, 收敛后再用
    // 近中性统计做精调 —— 这是 ISP 里常见的两段式做法.
    reg [23:0] acc_ra, acc_ga, acc_ba;

    wire [1:0]  nsel   = {neutral1, neutral0};
    wire [8:0]  radd   = (neutral0 ? {1'b0, r0} : 9'd0) + (neutral1 ? {1'b0, r1} : 9'd0);
    wire [8:0]  gadd   = (neutral0 ? {1'b0, g0} : 9'd0) + (neutral1 ? {1'b0, g1} : 9'd0);
    wire [8:0]  badd   = (neutral0 ? {1'b0, b0} : 9'd0) + (neutral1 ? {1'b0, b1} : 9'd0);
    wire [8:0]  rall   = {1'b0, r0} + {1'b0, r1};
    wire [8:0]  gall   = {1'b0, g0} + {1'b0, g1};
    wire [8:0]  ball   = {1'b0, b0} + {1'b0, b1};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_r  <= 24'd0; acc_g <= 24'd0; acc_b  <= 24'd0;
            acc_n  <= 24'd0; acc_y <= 26'd0; acc_c  <= 24'd0; acc_nt <= 24'd0;
            acc_ra <= 24'd0; acc_ga <= 24'd0; acc_ba <= 24'd0;
        end else if (vs_rise) begin
            acc_r  <= 24'd0; acc_g <= 24'd0; acc_b  <= 24'd0;
            acc_n  <= 24'd0; acc_y <= 26'd0; acc_c  <= 24'd0; acc_nt <= 24'd0;
            acc_ra <= 24'd0; acc_ga <= 24'd0; acc_ba <= 24'd0;
        end else if (sample_en) begin
            acc_r  <= acc_r + {15'd0, radd};
            acc_g  <= acc_g + {15'd0, gadd};
            acc_b  <= acc_b + {15'd0, badd};
            acc_ra <= acc_ra + {15'd0, rall};
            acc_ga <= acc_ga + {15'd0, gall};
            acc_ba <= acc_ba + {15'd0, ball};
            acc_n  <= acc_n + {22'd0, nsel};
            acc_y  <= acc_y + {18'd0, y0} + {18'd0, y1};
            acc_nt <= acc_nt + 24'd2;
            if ((mx0 >= 8'd250) || (mx1 >= 8'd250))
                acc_c <= acc_c + 24'd1;
        end
    end

    //-------------------------------------------------------------------------
    // AWB 增量控制器 (无除法)
    //-------------------------------------------------------------------------
    reg  [15:0] g_r, g_b;
    reg  [2:0]  lock_cnt;

    // 两段式统计: 近中性点够用就用近中性; 不够(严重偏色/大面积彩色)退回全画面
    wire        use_all = (acc_n < MIN_SAMPLES);
    wire [23:0] m_r = use_all ? acc_ra : acc_r;
    wire [23:0] m_g = use_all ? acc_ga : acc_g;
    wire [23:0] m_b = use_all ? acc_ba : acc_b;

    wire [24:0] band  = ({1'b0, m_g} * {9'd0, DEADBAND_Q}) >> 8;
    wire [24:0] g_up_ = {1'b0, m_g} + band;
    wire [24:0] g_dn_ = {1'b0, m_g} - band;
    wire [24:0] a_r   = {1'b0, m_r};
    wire [24:0] a_b   = {1'b0, m_b};

    wire r_dark   = (a_r < g_dn_);
    wire r_bright = (a_r > g_up_);
    wire b_dark   = (a_b < g_dn_);
    wire b_bright = (a_b > g_up_);
    wire in_band  = !(r_dark | r_bright | b_dark | b_bright);

    // 步进 = clamp(1 + |err|/16384, 1, MAX_STEP)
    wire [24:0] er = r_dark ? (g_dn_ - a_r) : (r_bright ? (a_r - g_up_) : 25'd0);
    wire [24:0] eb = b_dark ? (g_dn_ - a_b) : (b_bright ? (a_b - g_up_) : 25'd0);
    wire [3:0]  step_r = (er[24:14] != 11'd0) ? MAX_STEP : 4'd1;
    wire [3:0]  step_b = (eb[24:14] != 11'd0) ? MAX_STEP : 4'd1;

    wire [15:0] g_r_up = (g_r > GAIN_MAX - {12'd0, step_r}) ? GAIN_MAX : g_r + {12'd0, step_r};
    wire [15:0] g_r_dn = (g_r < GAIN_MIN + {12'd0, step_r}) ? GAIN_MIN : g_r - {12'd0, step_r};
    wire [15:0] g_b_up = (g_b > GAIN_MAX - {12'd0, step_b}) ? GAIN_MAX : g_b + {12'd0, step_b};
    wire [15:0] g_b_dn = (g_b < GAIN_MIN + {12'd0, step_b}) ? GAIN_MIN : g_b - {12'd0, step_b};

    // 场景过暗时噪声主导, 统计不可信 -> 冻结环路
    wire [31:0] dim_lim = {8'd0, acc_nt} * 32'd24;
    wire        lowlight = (acc_nt != 24'd0) && ({6'd0, acc_y} < dim_lim);
    wire        few_samples = (acc_nt < {MIN_SAMPLES[22:0], 1'b0});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            g_r <= SEED_R_GAIN;
            g_b <= SEED_B_GAIN;
        end else if (vs_rise && !i_man_en) begin
            if (!few_samples && !lowlight) begin
                if (r_dark)        g_r <= g_r_up;
                else if (r_bright) g_r <= g_r_dn;
                if (b_dark)        g_b <= g_b_up;
                else if (b_bright) g_b <= g_b_dn;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lock_cnt     <= 3'd0;
            o_awb_locked <= 1'b0;
        end else if (vs_rise) begin
            if (i_man_en || few_samples || lowlight) begin
                lock_cnt     <= 3'd0;
                o_awb_locked <= 1'b0;
            end else if (in_band) begin
                if (lock_cnt >= LOCK_FRAMES[2:0]) o_awb_locked <= 1'b1;
                else                              lock_cnt <= lock_cnt + 1'b1;
            end else begin
                lock_cnt     <= 3'd0;
                o_awb_locked <= 1'b0;
            end
        end
    end

    //-------------------------------------------------------------------------
    // 增益输出
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_r_gain <= SEED_R_GAIN;
            o_g_gain <= SEED_G_GAIN;
            o_b_gain <= SEED_B_GAIN;
        end else if (i_man_en) begin
            o_r_gain <= i_man_r;
            o_b_gain <= i_man_b;
            o_g_gain <= SEED_G_GAIN;
        end else begin
            o_r_gain <= g_r;
            o_b_gain <= g_b;
            o_g_gain <= SEED_G_GAIN;
        end
    end

    //-------------------------------------------------------------------------
    // AE
    //-------------------------------------------------------------------------
    reg  [7:0]  ae_cnt;
    reg  [15:0] ae_exp;
    reg  [7:0]  ae_gc, ae_gf;
    reg         ae_pend;
    reg         ack_meta, ack_sync;
    reg  [1:0]  ae_settle;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ack_meta <= 1'b0;
            ack_sync <= 1'b0;
        end else begin
            ack_meta <= i_ae_ack_tgl;
            ack_sync <= ack_meta;
        end
    end

    wire ae_busy = (o_ae_req_tgl != ack_sync);

    wire [31:0] y_hi_lim = {8'd0, acc_nt} * {24'd0, AE_TOL_HI};
    wire [31:0] y_lo_lim = {8'd0, acc_nt} * {24'd0, AE_TOL_LO};
    wire [23:0] clip_lim = (acc_nt >> 5) + 24'd4;   // N/32 + 4
    wire        too_bright = ({6'd0, acc_y} > y_hi_lim) || (acc_c > clip_lim);
    wire        too_dark   = ({6'd0, acc_y} < y_lo_lim);

    wire [15:0] exp_up   = ae_exp + (ae_exp >> 3);
    wire [15:0] exp_dn   = ae_exp - (ae_exp >> 3);
    wire [15:0] exp_up_c = (exp_up > AE_EXP_MAX) ? AE_EXP_MAX : exp_up;
    wire [15:0] exp_dn_c = (exp_dn < AE_EXP_MIN) ? AE_EXP_MIN : exp_dn;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ae_cnt       <= 8'd0;
            ae_exp       <= AE_EXP_SEED;
            ae_gc        <= AE_GAIN_SEED_C;
            ae_gf        <= AE_GAIN_SEED_F;
            ae_pend      <= 1'b0;
            ae_settle    <= 2'd0;
            o_ae_req_tgl <= 1'b0;
            o_ae_exp     <= AE_EXP_SEED;
            o_ae_gain_c  <= AE_GAIN_SEED_C;
            o_ae_gain_f  <= AE_GAIN_SEED_F;
        end else begin
            if (ae_pend && !ae_busy) begin
                ae_pend   <= 1'b0;
                ae_settle <= 2'd2;
            end else if (ae_settle != 2'd0) begin
                ae_settle <= ae_settle - 1'b1;
            end

            if (vs_rise) begin
                if (!i_ae_en || i_man_en) begin
                    ae_cnt <= 8'd0;
                end else if (ae_cnt != AE_PERIOD - 1'b1) begin
                    ae_cnt <= ae_cnt + 1'b1;
                end else begin
                    ae_cnt <= 8'd0;
                    if (!ae_pend && !ae_busy && (ae_settle == 2'd0) && (acc_nt != 24'd0)) begin
                        if (too_bright) begin
                            if (ae_exp > AE_EXP_MIN) begin
                                ae_exp       <= exp_dn_c;
                                o_ae_exp     <= exp_dn_c;
                                o_ae_gain_c  <= ae_gc;
                                o_ae_gain_f  <= ae_gf;
                                o_ae_req_tgl <= ~o_ae_req_tgl;
                                ae_pend      <= 1'b1;
                            end else if (ae_gc != 8'h00) begin
                                ae_gc        <= 8'h00;
                                ae_gf        <= 8'h20;
                                o_ae_exp     <= ae_exp;
                                o_ae_gain_c  <= 8'h00;
                                o_ae_gain_f  <= 8'h20;
                                o_ae_req_tgl <= ~o_ae_req_tgl;
                                ae_pend      <= 1'b1;
                            end
                        end else if (too_dark) begin
                            if (ae_exp < AE_EXP_MAX) begin
                                ae_exp       <= exp_up_c;
                                o_ae_exp     <= exp_up_c;
                                o_ae_gain_c  <= ae_gc;
                                o_ae_gain_f  <= ae_gf;
                                o_ae_req_tgl <= ~o_ae_req_tgl;
                                ae_pend      <= 1'b1;
                            end else if (ae_gc != 8'h81) begin
                                ae_gc        <= 8'h81;
                                ae_gf        <= 8'h32;
                                o_ae_exp     <= ae_exp;
                                o_ae_gain_c  <= 8'h81;
                                o_ae_gain_f  <= 8'h32;
                                o_ae_req_tgl <= ~o_ae_req_tgl;
                                ae_pend      <= 1'b1;
                            end
                        end
                    end
                end
            end
        end
    end

    //-------------------------------------------------------------------------
    // 调试
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            o_awb_dbg <= 4'd0;
        else
            o_awb_dbg <= {o_awb_locked,
                          (g_r == GAIN_MAX) | (g_b == GAIN_MAX),
                          lowlight, few_samples};
    end

endmodule
