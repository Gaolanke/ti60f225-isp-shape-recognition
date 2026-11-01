//=============================================================================
// edge_proc.v  (v6.1)
//   可切换前置滤波(中值/高斯/直通) -> Sobel -> NMS -> 双阈值滞回 -> 分屏/叠加
//
//   参考开源工程 "Hardware Implementation of the Canny Edge Detection
//   Algorithm" 的流水线思路, 并结合另外 3 个开源 FPGA Canny 工程的实现:
//     1) 前置滤波(两级级联, 按键可切)
//                            0=3x3 中值(排序网络, 抑制椒盐噪点)
//                            1=3x3 高斯 [1 2 1]x[1 2 1]/16 (抑制随机噪点)
//                            2=直通(原始灰度)
//                            3=中值 -> 高斯 级联(默认, 真实摄像头图像上最干净:
//                              实测边缘像素 -20%, 孤立噪点 -63%, 见 work_dbg/edge_model)
//     2) 灰度       (R+2G+B)>>2, 或 77/150/29 加权(移位加法, 无乘法器)
//     3) Sobel 3x3  幅值 = max + min/2 (各向同性 sqrt 近似, MAG_ISO=0 时回到 |Gx|+|Gy|)
//                   方向 = 22.5/45/67.5 度四分带 + 符号位 (纯移位加法, 无 CORDIC/乘法器)
//     4) NMS        沿梯度方向与相邻两点比较(斜带用 1/2 加权插值), 非极大值置 0
//     5) 双阈值滞回 强边缘(>TH_HIGH)直接保留; 弱边缘(>TH_LOW)满足以下任一即保留:
//                   8 邻域内有强边缘 / 已判决邻域(左,上左,上,上右)已连通到边缘 /
//                   非零邻居数 >= ISOLATE_MIN
//     6) 输出       左半屏 = 原灰度图;
//                   右半屏 = 二值边缘图(白边黑底) 或 彩色原图+方向彩边(i_overlay)
//                   分屏列 i_split_col 运行时可调: 0 = 全屏彩色原图+彩边(不分屏)
//                            方向彩边: 竖直边红 / 水平边绿 / 斜边黄、青 (EDGE_COLOR_DIR)
//                   帧首 TOP_BLANK 行 / 行首行尾 COL_BLANK 列 不出边缘
//
//   时钟: hdmi_tx_slow_clk (148.5MHz), 每时钟 1 像素, 像素格式 {B,G,R}
//   流水线总延迟 28 级; de/hs/vs/彩色数据 与灰度数据全程对齐延后
//   输出附带的 o_edge/o_dir16/o_row 供 shape_proc 做圆/矩形/三角形识别
//=============================================================================

module edge_proc #(
    parameter integer H_ACTIVE       = 1920,   // 每行有效像素数
    parameter integer SPLIT_COL      = 960,    // 分屏列(左<SPLIT_COL<=右)
    parameter [7:0]   TH_LOW         = 8'd2,   // 弱阈值(G>>3 的 0~255 尺度)
    parameter [7:0]   TH_HIGH        = 8'd5,   // 强阈值(需 > TH_LOW)
    parameter integer GRAY_SHIFT_ADD = 1,      // 1=(R+2G+B)>>2; 0=77/150/29 加权
    parameter integer EDGE_DILATE    = 0,      // 0 = 边缘 1 像素细线(默认); 1 = 2 像素水平加粗
    parameter integer ISOLATE_MIN    = 4,      // 弱边缘保留所需的最小非零邻居数(越大越干净)
    parameter integer MAG_ISO        = 1,      // 1=max+min/2 各向同性幅值; 0=|Gx|+|Gy|(L1)
    parameter integer NMS_INTERP     = 1,      // 1=22.5 度分带+1/2 插值 NMS; 0=旧 2:1 硬切
    parameter integer HYST_CONN      = 1,      // 1=增加已判决邻域连通性传播; 0=仅 v4.1 规则
    parameter integer EDGE_COLOR_DIR = 1,      // 1=叠加模式按梯度方向着色; 0=统一红色
    parameter integer TOP_BLANK      = 8,      // 帧首屏蔽行数(消除行缓存跨帧残留白线)
    parameter integer COL_BLANK      = 2,      // 行首/行尾屏蔽列数(消除窗口填充伪影)
    parameter integer ISO_EN         = 0,      // 1=启用"非零邻居数>=ISOLATE_MIN"兜底
                                               //   0(默认)=只用连通性, 实测毛刺少 ~13%
    parameter integer V_ACTIVE       = 1080,   // 每帧有效行数(画中画窗口定位用)
    parameter integer PIP_W          = 640,    // 画中画窗口宽
    parameter integer PIP_H          = 480     // 画中画窗口高
)(
    input  wire        clk,
    input  wire        rst_n,

    // 视频输入(每时钟 1 像素)
    input  wire        i_de,
    input  wire        i_hs,
    input  wire        i_vs,
    input  wire [23:0] i_rgb,       // {B,G,R}

    // 阈值(可外接按键/VIO 在线调节)
    input  wire [7:0]  i_th_low,
    input  wire [7:0]  i_th_high,

    // 运行配置
    input  wire [1:0]  i_prefilt,   // 0=中值 1=高斯 2=直通 3=中值+高斯(默认)
    input  wire        i_overlay,   // 1 = 右半屏彩色原图 + 彩色边缘
    input  wire [10:0] i_split_col, // 运行时分屏列: 0=全屏叠加(不分屏); 其它=左<该列显示灰度
    input  wire [1:0]  i_color_mode,// 0=按方向着色 1=统一红 2=统一白 3=统一绿
    input  wire        i_pip,       // 1 = 画中画: 全屏彩色原图 + 右下角小窗显示二值边缘
    input  wire        i_shape_mode,// 1 = 原始彩色图 + 原始 Sobel 红边(模式4识别用)

    // 视频输出
    output reg         o_de,
    output reg         o_hs,
    output reg         o_vs,
    output reg  [23:0] o_rgb,

    // 诊断: 本帧右半屏出现过边缘
    output reg         o_edge_flag,

    // 给 shape_proc 的随路数据(o_rgb 对齐)
    output reg         o_edge,      // 最终边缘标志(已含帧首/行首屏蔽)
    output reg  [3:0]  o_dir16,     // 16 方向量化: {band8[2:0], quad}
    output reg  [10:0] o_row        // 当前输出行号(帧内)
);

    //-------------------------------------------------------------------------
    localparam integer AW       = 11;
    localparam integer LB_DEPTH = 2048;
    localparam integer LB_DELAY = H_ACTIVE;
    localparam [AW-1:0] LB_INIT = LB_DEPTH - LB_DELAY;
    localparam integer GSHIFT   = (GRAY_SHIFT_ADD != 0) ? 2 : 8;
    localparam [7:0]   ISO_MIN  = ISOLATE_MIN[7:0];
    // A bright two-pixel red contour is required for visibility on the 1080p
    // panel.  Keep the display threshold well above the recogniser threshold
    // so skin and fabric texture still stay unpainted.
    localparam integer SOBEL_DISP_EXTRA = 4;
    localparam integer SOBEL_DISP_MIN   = 16;
    localparam [AW:0]  PIP_X0   = H_ACTIVE - PIP_W - 16;
    localparam [AW:0]  PIP_X1   = H_ACTIVE - 16;
    localparam [AW:0]  PIP_Y0   = V_ACTIVE - PIP_H - 16;
    localparam [AW:0]  PIP_Y1   = V_ACTIVE - 16;

    // 8bit 比较单元(中值排序网络用)
    function [7:0] f_max;
        input [7:0] a;
        input [7:0] b;
        begin f_max = (a > b) ? a : b; end
    endfunction
    function [7:0] f_min;
        input [7:0] a;
        input [7:0] b;
        begin f_min = (a < b) ? a : b; end
    endfunction
    function [7:0] f_med3;          // 3 数中值 = max(min(a,b), min(max(a,b),c))
        input [7:0] a;
        input [7:0] b;
        input [7:0] c;
        begin f_med3 = f_max(f_min(a,b), f_min(f_max(a,b), c)); end
    endfunction
    //=========================================================================
    // s1: 灰度加权(移位加法 或 77/150/29) + 彩色随路
    //=========================================================================
    wire [7:0] r_in = i_rgb[7:0];
    wire [7:0] g_in = i_rgb[15:8];
    wire [7:0] b_in = i_rgb[23:16];

    reg  [15:0] p_r, p_g, p_b;
    reg  [23:0] col_s1;
    reg         s1_de, s1_hs, s1_vs;

    generate
        if (GRAY_SHIFT_ADD != 0) begin : g_shiftadd
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    p_r <= 16'd0; p_g <= 16'd0; p_b <= 16'd0;
                end else begin
                    p_r <= {8'd0 , r_in};        // r * 1
                    p_g <= {7'd0 , g_in, 1'b0};  // g * 2
                    p_b <= {8'd0 , b_in};        // b * 1
                end
            end
        end else begin : g_weighted
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    p_r <= 16'd0; p_g <= 16'd0; p_b <= 16'd0;
                end else begin
                    p_r <= r_in * 8'd77;         // 0.299*256
                    p_g <= g_in * 8'd150;        // 0.587*256
                    p_b <= b_in * 8'd29;         // 0.114*256
                end
            end
        end
    endgenerate

    // Keep faint texture out of the red overlay, but do not let the user
    // threshold table make strong outlines disappear on the panel.
    wire [8:0] sobel_disp_sum = {1'b0, i_th_high} + SOBEL_DISP_EXTRA;
    wire [7:0] sobel_disp_th  = (sobel_disp_sum < SOBEL_DISP_MIN)
                                ? 8'd24 : sobel_disp_sum[7:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_s1 <= 24'd0;
            s1_de <= 1'b0; s1_hs <= 1'b0; s1_vs <= 1'b0;
        end else begin
            col_s1 <= i_rgb;
            s1_de <= i_de; s1_hs <= i_hs; s1_vs <= i_vs;
        end
    end

    //=========================================================================
    // s2: 灰度 = (p_r + p_g + p_b) >> GSHIFT
    //=========================================================================
    wire [16:0] gray_sum = {1'b0, p_r} + {1'b0, p_g} + {1'b0, p_b};

    reg  [7:0]  gray;
    reg  [23:0] col_s2;
    reg         s2_de, s2_hs, s2_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gray <= 8'd0; col_s2 <= 24'd0;
            s2_de <= 1'b0; s2_hs <= 1'b0; s2_vs <= 1'b0;
        end else begin
            gray   <= gray_sum[GSHIFT +: 8];
            col_s2 <= col_s1;
            s2_de <= s1_de; s2_hs <= s1_hs; s2_vs <= s1_vs;
        end
    end

    //=========================================================================
    // s3: 灰度行缓存0(延迟 1 行)
    //=========================================================================
    reg  [7:0]    lb0_mem [0:LB_DEPTH-1];
    reg  [AW-1:0] wr0, rd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr0 <= {AW{1'b0}};
            rd0 <= LB_INIT;
        end else if (s2_de) begin
            wr0 <= (wr0 == LB_DEPTH-1) ? {AW{1'b0}} : wr0 + 1'b1;
            rd0 <= (rd0 == LB_DEPTH-1) ? {AW{1'b0}} : rd0 + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (s2_de) lb0_mem[wr0] <= gray;
    end

    reg  [7:0]  row1_raw;      // 上一行原始灰度
    reg  [7:0]  s3_gray;       // 当前行原始灰度
    reg  [23:0] col_s3;
    reg         s3_de, s3_hs, s3_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row1_raw <= 8'd0; s3_gray <= 8'd0; col_s3 <= 24'd0;
            s3_de <= 1'b0; s3_hs <= 1'b0; s3_vs <= 1'b0;
        end else begin
            row1_raw <= lb0_mem[rd0];
            s3_gray  <= gray;
            col_s3   <= col_s2;
            s3_de <= s2_de; s3_hs <= s2_hs; s3_vs <= s2_vs;
        end
    end

    //=========================================================================
    // s4: 灰度行缓存1(延迟 2 行) + 三行对齐
    //=========================================================================
    reg  [7:0]    lb1_mem [0:LB_DEPTH-1];
    reg  [AW-1:0] wr1, rd1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr1 <= {AW{1'b0}};
            rd1 <= LB_INIT;
        end else if (s3_de) begin
            wr1 <= (wr1 == LB_DEPTH-1) ? {AW{1'b0}} : wr1 + 1'b1;
            rd1 <= (rd1 == LB_DEPTH-1) ? {AW{1'b0}} : rd1 + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (s3_de) lb1_mem[wr1] <= row1_raw;
    end

    reg  [7:0]  row2_raw;      // 上上行(3x3 顶行)
    reg  [7:0]  row1_dly;      // 上一行(3x3 中行)
    reg  [7:0]  row0_dly;      // 当前行(3x3 底行)
    reg  [23:0] col_s4;
    reg         s4_de, s4_hs, s4_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row2_raw <= 8'd0; row1_dly <= 8'd0; row0_dly <= 8'd0;
            col_s4 <= 24'd0;
            s4_de <= 1'b0; s4_hs <= 1'b0; s4_vs <= 1'b0;
        end else begin
            row2_raw <= lb1_mem[rd1];
            row1_dly <= row1_raw;
            row0_dly <= s3_gray;
            col_s4   <= col_s3;
            s4_de <= s3_de; s4_hs <= s3_hs; s4_vs <= s3_vs;
        end
    end

    //=========================================================================
    // s5: 灰度 3x3 窗口(de 门控, 行间清零)
    //=========================================================================
    reg  [23:0] win_t, win_m, win_b;
    reg  [23:0] col_s5;
    reg         s4_de_d, s4_hs_d, s4_vs_d;
    reg  [23:0] col_s4_d;
    reg         s5_de, s5_hs, s5_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_s5 <= 24'd0;
            s4_de_d <= 1'b0; s4_hs_d <= 1'b0; s4_vs_d <= 1'b0;
            col_s4_d <= 24'd0;
            s5_de <= 1'b0; s5_hs <= 1'b0; s5_vs <= 1'b0;
        end else begin
            col_s4_d <= col_s4;
            col_s5 <= col_s4_d;
            // Shift-and-use window: the centre is one pixel behind s4_de.
            s4_de_d <= s4_de; s4_hs_d <= s4_hs; s4_vs_d <= s4_vs;
            s5_de <= s4_de_d; s5_hs <= s4_hs_d; s5_vs <= s4_vs_d;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            win_t <= 24'd0; win_m <= 24'd0; win_b <= 24'd0;
        end else if (s4_de) begin
            win_t <= {win_t[15:0], row2_raw};
            win_m <= {win_m[15:0], row1_dly};
            win_b <= {win_b[15:0], row0_dly};
        end else begin
            win_t <= 24'd0; win_m <= 24'd0; win_b <= 24'd0;
        end
    end

    wire [7:0] w_tl = win_t[23:16], w_tm = win_t[15:8], w_tr = win_t[7:0];
    wire [7:0] w_ml = win_m[23:16], w_mm = win_m[15:8], w_mr = win_m[7:0];
    wire [7:0] w_bl = win_b[23:16], w_bm = win_b[15:8], w_br = win_b[7:0];
    //=========================================================================
    // s6: 前置滤波 第 1 步
    //   中值: 3 行各自排序(取 min/mid/max); 高斯在 s11a(对中值输出再做一次)
    //   同时寄存原始灰度(直通用, 也供左半屏显示)
    //=========================================================================
    wire [7:0] r0_lo = f_min(w_tl, f_min(w_tm, w_tr));
    wire [7:0] r0_hi = f_max(w_tl, f_max(w_tm, w_tr));
    wire [7:0] r0_md = f_med3(w_tl, w_tm, w_tr);

    wire [7:0] r1_lo = f_min(w_ml, f_min(w_mm, w_mr));
    wire [7:0] r1_hi = f_max(w_ml, f_max(w_mm, w_mr));
    wire [7:0] r1_md = f_med3(w_ml, w_mm, w_mr);

    wire [7:0] r2_lo = f_min(w_bl, f_min(w_bm, w_br));
    wire [7:0] r2_hi = f_max(w_bl, f_max(w_bm, w_br));
    wire [7:0] r2_md = f_med3(w_bl, w_bm, w_br);

    reg  [7:0]  s6_lo0, s6_md0, s6_hi0;
    reg  [7:0]  s6_lo1, s6_md1, s6_hi1;
    reg  [7:0]  s6_lo2, s6_md2, s6_hi2;
    reg  [7:0]  s6_gc;         // 原始灰度(显示 / 直通)
    reg  [23:0] col_s6;
    reg         s6_de, s6_hs, s6_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s6_lo0 <= 8'd0; s6_md0 <= 8'd0; s6_hi0 <= 8'd0;
            s6_lo1 <= 8'd0; s6_md1 <= 8'd0; s6_hi1 <= 8'd0;
            s6_lo2 <= 8'd0; s6_md2 <= 8'd0; s6_hi2 <= 8'd0;
            s6_gc <= 8'd0; col_s6 <= 24'd0;
            s6_de <= 1'b0; s6_hs <= 1'b0; s6_vs <= 1'b0;
        end else begin
            s6_lo0 <= r0_lo; s6_md0 <= r0_md; s6_hi0 <= r0_hi;
            s6_lo1 <= r1_lo; s6_md1 <= r1_md; s6_hi1 <= r1_hi;
            s6_lo2 <= r2_lo; s6_md2 <= r2_md; s6_hi2 <= r2_hi;
                s6_gc    <= w_mm;
            col_s6   <= col_s5;
            s6_de <= s5_de; s6_hs <= s5_hs; s6_vs <= s5_vs;
        end
    end

    //=========================================================================
    // s7: 前置滤波 第 2 步(中值: 取出 3 个候选值)
    //=========================================================================
    wire [7:0] m_a = f_max(s6_lo0, f_max(s6_lo1, s6_lo2));   // 行最小者中的最大
    wire [7:0] m_b = f_med3(s6_md0, s6_md1, s6_md2);         // 行中值的中值
    wire [7:0] m_c = f_min(s6_hi0, f_min(s6_hi1, s6_hi2));   // 行最大者中的最小

    reg  [7:0]  s7_a, s7_b, s7_c;
    reg  [7:0]  s7_gc;
    reg  [23:0] col_s7;
    reg         s7_de, s7_hs, s7_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s7_a <= 8'd0; s7_b <= 8'd0; s7_c <= 8'd0;
            s7_gc <= 8'd0; col_s7 <= 24'd0;
            s7_de <= 1'b0; s7_hs <= 1'b0; s7_vs <= 1'b0;
        end else begin
            s7_a <= m_a; s7_b <= m_b; s7_c <= m_c;
            s7_gc    <= s6_gc;
            col_s7   <= col_s6;
            s7_de <= s6_de; s7_hs <= s6_hs; s7_vs <= s6_vs;
        end
    end

    //=========================================================================
    // s8: 前置滤波 第 1 级输出(中值 / 直通)
    //=========================================================================
    wire [7:0] med_now  = f_med3(s7_a, s7_b, s7_c);
    // 第 1 级: 只在 0(中值) / 3(中值+高斯) 时启用中值, 其余直接透传灰度
    wire       use_med_a = (i_prefilt == 2'd0) || (i_prefilt == 2'd3);
    wire [7:0] filt_now  = use_med_a ? med_now : s7_gc;

    reg  [7:0]  s8_filt;
    reg  [7:0]  s8_gc;
    reg  [23:0] col_s8;
    reg         s8_de, s8_hs, s8_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s8_filt <= 8'd0; s8_gc <= 8'd0; col_s8 <= 24'd0;
            s8_de <= 1'b0; s8_hs <= 1'b0; s8_vs <= 1'b0;
        end else begin
            s8_filt <= filt_now;
            s8_gc   <= s7_gc;
            col_s8  <= col_s7;
            s8_de <= s7_de; s8_hs <= s7_hs; s8_vs <= s7_vs;
        end
    end

    //=========================================================================
    // s9: 滤波图行缓存0(延迟 1 行)
    //=========================================================================
    reg  [7:0]    flb0_mem [0:LB_DEPTH-1];
    reg  [AW-1:0] wr0f, rd0f;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr0f <= {AW{1'b0}};
            rd0f <= LB_INIT;
        end else if (s8_de) begin
            wr0f <= (wr0f == LB_DEPTH-1) ? {AW{1'b0}} : wr0f + 1'b1;
            rd0f <= (rd0f == LB_DEPTH-1) ? {AW{1'b0}} : rd0f + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (s8_de) flb0_mem[wr0f] <= s8_filt;
    end

    reg  [7:0]  row1_flt;
    reg  [7:0]  s9_filt;
    reg  [7:0]  s9_gc;
    reg  [23:0] col_s9;
    reg         s9_de, s9_hs, s9_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row1_flt <= 8'd0; s9_filt <= 8'd0; s9_gc <= 8'd0; col_s9 <= 24'd0;
            s9_de <= 1'b0; s9_hs <= 1'b0; s9_vs <= 1'b0;
        end else begin
            row1_flt <= flb0_mem[rd0f];
            s9_filt  <= s8_filt;
            s9_gc    <= s8_gc;
            col_s9   <= col_s8;
            s9_de <= s8_de; s9_hs <= s8_hs; s9_vs <= s8_vs;
        end
    end

    //=========================================================================
    // s10: 滤波图行缓存1(延迟 2 行) + 三行对齐
    //=========================================================================
    reg  [7:0]    flb1_mem [0:LB_DEPTH-1];
    reg  [AW-1:0] wr1f, rd1f;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr1f <= {AW{1'b0}};
            rd1f <= LB_INIT;
        end else if (s9_de) begin
            wr1f <= (wr1f == LB_DEPTH-1) ? {AW{1'b0}} : wr1f + 1'b1;
            rd1f <= (rd1f == LB_DEPTH-1) ? {AW{1'b0}} : rd1f + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (s9_de) flb1_mem[wr1f] <= row1_flt;
    end

    reg  [7:0]  flt_row2;
    reg  [7:0]  flt_row1_dly;
    reg  [7:0]  flt_row0;
    reg  [7:0]  s10_gc;
    reg  [23:0] col_s10;
    reg         s10_de, s10_hs, s10_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            flt_row2 <= 8'd0; flt_row1_dly <= 8'd0; flt_row0 <= 8'd0;
            s10_gc <= 8'd0; col_s10 <= 24'd0;
            s10_de <= 1'b0; s10_hs <= 1'b0; s10_vs <= 1'b0;
        end else begin
            flt_row2     <= flb1_mem[rd1f];
            flt_row1_dly <= row1_flt;
            flt_row0     <= s9_filt;
            s10_gc  <= s9_gc;
            col_s10 <= col_s9;
            s10_de <= s9_de; s10_hs <= s9_hs; s10_vs <= s9_vs;
        end
    end

    //=========================================================================
    // s11: 滤波图 3x3 窗口(de 门控, 行间清零)
    //=========================================================================
    reg  [23:0] fwin_t, fwin_m, fwin_b;
    reg  [7:0]  s11_gc;
    reg  [23:0] col_s11;
    reg         s10_de_d, s10_hs_d, s10_vs_d;
    reg  [7:0]  s10_gc_d;
    reg  [23:0] col_s10_d;
    reg         s11_de, s11_hs, s11_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s11_de <= 1'b0; s11_hs <= 1'b0; s11_vs <= 1'b0;
            s11_gc <= 8'd0; col_s11 <= 24'd0;
            s10_de_d <= 1'b0; s10_hs_d <= 1'b0; s10_vs_d <= 1'b0;
            s10_gc_d <= 8'd0; col_s10_d <= 24'd0;
        end else begin
            // Match the valid/colour bus to the centre of the shifted window.
            s10_de_d <= s10_de; s10_hs_d <= s10_hs; s10_vs_d <= s10_vs;
            s10_gc_d <= s10_gc; col_s10_d <= col_s10;
            s11_de <= s10_de_d; s11_hs <= s10_hs_d; s11_vs <= s10_vs_d;
            s11_gc <= s10_gc_d; col_s11 <= col_s10_d;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fwin_t <= 24'd0; fwin_m <= 24'd0; fwin_b <= 24'd0;
        end else if (s10_de) begin
            fwin_t <= {fwin_t[15:0], flt_row2};
            fwin_m <= {fwin_m[15:0], flt_row1_dly};
            fwin_b <= {fwin_b[15:0], flt_row0};
        end else begin
            fwin_t <= 24'd0; fwin_m <= 24'd0; fwin_b <= 24'd0;
        end
    end

    wire [7:0] f_tl = fwin_t[23:16], f_tm = fwin_t[15:8], f_tr = fwin_t[7:0];
    wire [7:0] f_ml = fwin_m[23:16], f_mm = fwin_m[15:8], f_mr = fwin_m[7:0];
    wire [7:0] f_bl = fwin_b[23:16], f_bm = fwin_b[15:8], f_br = fwin_b[7:0];

    //=========================================================================
    // s11a: 前置滤波 第 2 级 —— 3x3 高斯(对第 1 级输出再做一次 [1 2 1]x[1 2 1]/16)
    //   中值->高斯 级联在真实摄像头图像上明显更干净(边缘 -20%, 孤立噪点 -63%)
    //   i_prefilt: 0=中值 1=高斯 2=直通 3=中值+高斯(默认)
    //=========================================================================
    wire [11:0] gauss2_sum = f_tl + {f_tm, 1'b0} + f_tr
                           + {f_ml, 1'b0} + {f_mm, 2'b0} + {f_mr, 1'b0}
                           + f_bl + {f_bm, 1'b0} + f_br;
    wire        use_gauss_b = (i_prefilt == 2'd1) || (i_prefilt == 2'd3);
    wire [7:0]  s11a_now    = use_gauss_b ? gauss2_sum[11:4] : f_mm;

    reg  [7:0]  s11a_filt;
    reg  [7:0]  s11a_gc;
    reg  [23:0] col_s11a;
    reg         s11a_de, s11a_hs, s11a_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s11a_filt <= 8'd0; s11a_gc <= 8'd0; col_s11a <= 24'd0;
            s11a_de <= 1'b0; s11a_hs <= 1'b0; s11a_vs <= 1'b0;
        end else begin
            s11a_filt <= s11a_now;
            s11a_gc   <= s11_gc;
            col_s11a  <= col_s11;
            s11a_de <= s11_de; s11a_hs <= s11_hs; s11a_vs <= s11_vs;
        end
    end

    //=========================================================================
    // s11b/s11c: 第 2 级滤波图行缓存 x2 + 三行对齐
    //=========================================================================
    reg  [7:0]    flb2_mem [0:LB_DEPTH-1];
    reg  [AW-1:0] wr2f, rd2f;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr2f <= {AW{1'b0}};
            rd2f <= LB_INIT;
        end else if (s11a_de) begin
            wr2f <= (wr2f == LB_DEPTH-1) ? {AW{1'b0}} : wr2f + 1'b1;
            rd2f <= (rd2f == LB_DEPTH-1) ? {AW{1'b0}} : rd2f + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (s11a_de) flb2_mem[wr2f] <= s11a_filt;
    end

    reg  [7:0]  row1_x;
    reg  [7:0]  s11b_filt;
    reg  [7:0]  s11b_gc;
    reg  [23:0] col_s11b;
    reg         s11b_de, s11b_hs, s11b_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row1_x <= 8'd0; s11b_filt <= 8'd0; s11b_gc <= 8'd0; col_s11b <= 24'd0;
            s11b_de <= 1'b0; s11b_hs <= 1'b0; s11b_vs <= 1'b0;
        end else begin
            row1_x    <= flb2_mem[rd2f];
            s11b_filt <= s11a_filt;
            s11b_gc   <= s11a_gc;
            col_s11b  <= col_s11a;
            s11b_de <= s11a_de; s11b_hs <= s11a_hs; s11b_vs <= s11a_vs;
        end
    end

    reg  [7:0]    flb3_mem [0:LB_DEPTH-1];
    reg  [AW-1:0] wr3f, rd3f;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr3f <= {AW{1'b0}};
            rd3f <= LB_INIT;
        end else if (s11b_de) begin
            wr3f <= (wr3f == LB_DEPTH-1) ? {AW{1'b0}} : wr3f + 1'b1;
            rd3f <= (rd3f == LB_DEPTH-1) ? {AW{1'b0}} : rd3f + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (s11b_de) flb3_mem[wr3f] <= row1_x;
    end

    reg  [7:0]  x_row2;
    reg  [7:0]  x_row1_dly;
    reg  [7:0]  x_row0;
    reg  [7:0]  s11c_gc;
    reg  [23:0] col_s11c;
    reg         s11c_de, s11c_hs, s11c_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_row2 <= 8'd0; x_row1_dly <= 8'd0; x_row0 <= 8'd0;
            s11c_gc <= 8'd0; col_s11c <= 24'd0;
            s11c_de <= 1'b0; s11c_hs <= 1'b0; s11c_vs <= 1'b0;
        end else begin
            x_row2     <= flb3_mem[rd3f];
            x_row1_dly <= row1_x;
            x_row0     <= s11b_filt;
            s11c_gc  <= s11b_gc;
            col_s11c <= col_s11b;
            s11c_de <= s11b_de; s11c_hs <= s11b_hs; s11c_vs <= s11b_vs;
        end
    end

    //=========================================================================
    // s11d: 第 2 级滤波图 3x3 窗口(de 门控, 行间清零) -> 送 Sobel
    //=========================================================================
    reg  [23:0] xwin_t, xwin_m, xwin_b;
    reg  [7:0]  s11d_gc;
    reg  [23:0] col_s11d;
    reg         s11c_de_d, s11c_hs_d, s11c_vs_d;
    reg  [7:0]  s11c_gc_d;
    reg  [23:0] col_s11c_d;
    reg         s11d_de, s11d_hs, s11d_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s11d_de <= 1'b0; s11d_hs <= 1'b0; s11d_vs <= 1'b0;
            s11d_gc <= 8'd0; col_s11d <= 24'd0;
            s11c_de_d <= 1'b0; s11c_hs_d <= 1'b0; s11c_vs_d <= 1'b0;
            s11c_gc_d <= 8'd0; col_s11c_d <= 24'd0;
        end else begin
            s11c_de_d <= s11c_de; s11c_hs_d <= s11c_hs; s11c_vs_d <= s11c_vs;
            s11c_gc_d <= s11c_gc; col_s11c_d <= col_s11c;
            s11d_de <= s11c_de_d; s11d_hs <= s11c_hs_d; s11d_vs <= s11c_vs_d;
            s11d_gc <= s11c_gc_d; col_s11d <= col_s11c_d;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            xwin_t <= 24'd0; xwin_m <= 24'd0; xwin_b <= 24'd0;
        end else if (s11c_de) begin
            xwin_t <= {xwin_t[15:0], x_row2};
            xwin_m <= {xwin_m[15:0], x_row1_dly};
            xwin_b <= {xwin_b[15:0], x_row0};
        end else begin
            xwin_t <= 24'd0; xwin_m <= 24'd0; xwin_b <= 24'd0;
        end
    end

    wire [7:0] x_tl = xwin_t[23:16], x_tm = xwin_t[15:8], x_tr = xwin_t[7:0];
    wire [7:0] x_ml = xwin_m[23:16], x_mm = xwin_m[15:8], x_mr = xwin_m[7:0];
    wire [7:0] x_bl = xwin_b[23:16], x_bm = xwin_b[15:8], x_br = xwin_b[7:0];
    //=========================================================================
    // s12: Sobel 行列部分和(×2 用左移, 不用乘法器)
    //   Gx = (tr + 2*mr + br) - (tl + 2*ml + bl)
    //   Gy = (bl + 2*bm + br) - (tl + 2*tm + tr)
    //=========================================================================
    wire [9:0] sum_gx_r = x_tr + {x_mr, 1'b0} + x_br;
    wire [9:0] sum_gx_l = x_tl + {x_ml, 1'b0} + x_bl;
    wire [9:0] sum_gy_b = x_bl + {x_bm, 1'b0} + x_br;
    wire [9:0] sum_gy_t = x_tl + {x_tm, 1'b0} + x_tr;

    reg  [9:0]  s12_gx_r, s12_gx_l, s12_gy_b, s12_gy_t;
    reg  [7:0]  s12_gc;
    reg  [23:0] col_s12;
    reg         s12_de, s12_hs, s12_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s12_gx_r <= 10'd0; s12_gx_l <= 10'd0;
            s12_gy_b <= 10'd0; s12_gy_t <= 10'd0;
            s12_gc <= 8'd0; col_s12 <= 24'd0;
            s12_de <= 1'b0; s12_hs <= 1'b0; s12_vs <= 1'b0;
        end else begin
            s12_gx_r <= sum_gx_r;
            s12_gx_l <= sum_gx_l;
            s12_gy_b <= sum_gy_b;
            s12_gy_t <= sum_gy_t;
            s12_gc   <= s11d_gc;
            col_s12  <= col_s11d;
            s12_de <= s11d_de; s12_hs <= s11d_hs; s12_vs <= s11d_vs;
        end
    end

    //=========================================================================
    // s13: Gx / Gy(有符号)
    //=========================================================================
    reg signed [10:0] gx, gy;
    reg  [7:0]  s13_gc;
    reg  [23:0] col_s13;
    reg         s13_de, s13_hs, s13_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gx <= 11'sd0; gy <= 11'sd0; s13_gc <= 8'd0; col_s13 <= 24'd0;
            s13_de <= 1'b0; s13_hs <= 1'b0; s13_vs <= 1'b0;
        end else begin
            gx <= {1'b0, s12_gx_r} - {1'b0, s12_gx_l};
            gy <= {1'b0, s12_gy_b} - {1'b0, s12_gy_t};
            s13_gc <= s12_gc;
            col_s13 <= col_s12;
            s13_de <= s12_de; s13_hs <= s12_hs; s13_vs <= s12_vs;
        end
    end

    //=========================================================================
    // s14: 幅值(各向同性) + 方向分带 dir3 = {band[1:0], quad}
    //   幅值 MAG_ISO=1: max + min/2, 误差 <6% 且与角度无关 (|Gx|+|Gy| 在对角方向
    //                   会偏亮 41%, 导致斜边阈值/极大值判定不一致, 是"斜边变粗、
    //                   噪点成片"的来源之一; 参考工程 DOUDIU 用 CORDIC 开方做精确
    //                   L2, 本工程用 1 比较器 + 1 加法 + 1 移位做便宜近似)
    //   方向: tan22.5 = 0.4142 = 1/4+1/8+1/32+1/128 (纯移位加法)
    //     band0: |Gy| < 0.414|Gx|  梯度近水平 -> 边缘近竖直
    //     band1: 0.414|Gx| <= |Gy| < |Gx|        (斜带, 近水平侧)
    //     band2: |Gx| <= |Gy| < 2.414|Gx|        (斜带, 近竖直侧)
    //     band3: |Gy| >= 2.414|Gx| 梯度近竖直 -> 边缘近水平
    //     quad = 1 -> gx/gy 异号("/" 对角); 0 -> 同号("\" 对角)
    //   NMS_INTERP=0 时退回 v4.1 的 4 扇区(|Gx|>2|Gy| / |Gy|>2|Gx| / 符号)编码
    //=========================================================================
    wire [10:0] gx_u = gx[10:0];
    wire [10:0] gy_u = gy[10:0];
    wire [10:0] abs_gx = gx_u[10] ? (~gx_u + 11'd1) : gx_u;
    wire [10:0] abs_gy = gy_u[10] ? (~gy_u + 11'd1) : gy_u;

    // (a) 各向同性幅值: max + min/2 (max+min <= 2040, 故结果仍 <= 2040)
    wire [10:0] mag_max = (abs_gx > abs_gy) ? abs_gx : abs_gy;
    wire [10:0] mag_min = (abs_gx > abs_gy) ? abs_gy : abs_gx;
    wire [10:0] mag     = (MAG_ISO != 0) ? (mag_max + {1'b0, mag_min[10:1]})
                                         : (abs_gx + abs_gy);

    // (b) 方向分带: 22.5 度边界用 0.414*|Gx|, 67.5 度边界用对称的 0.414*|Gy|
    //     |Gy| >= 2.414|Gx|  <==>  |Gx| < 0.414|Gy|  (换轴比较, 省掉宽位加法)
    wire [10:0] gx_h    = abs_gx;
    wire [10:0] ay_h    = abs_gy;
    wire [11:0] q22_ax  = {3'b0, gx_h[10:2]} + {4'b0, gx_h[10:3]} +
                          {6'b0, gx_h[10:5]} + {8'b0, gx_h[10:7]};   // 0.414*|Gx|
    wire [11:0] q22_ay  = {3'b0, ay_h[10:2]} + {4'b0, ay_h[10:3]} +
                          {6'b0, ay_h[10:5]} + {8'b0, ay_h[10:7]};   // 0.414*|Gy|
    wire [11:0] ax12    = {1'b0, gx_h};
    wire [11:0] ay12    = {1'b0, ay_h};

    wire [1:0]  band_old  = (abs_gx > {abs_gy[9:0], 1'b0}) ? 2'd0 :
                            (abs_gy > {abs_gx[9:0], 1'b0}) ? 2'd3 :
                            (gx[10] ^ gy[10])              ? 2'd2 : 2'd1;
    wire [1:0]  band_fine = (ay12 < q22_ax) ? 2'd0 :   // |Gy| < 0.414|Gx|  -> <22.5 度
                            (ay12 < ax12)   ? 2'd1 :   // 22.5 ~ 45 度
                            (ax12 < q22_ay) ? 2'd3 :   // |Gx| < 0.414|Gy|  -> >67.5 度
                                              2'd2;    // 45 ~ 67.5 度
    wire [1:0]  band_now  = (NMS_INTERP != 0) ? band_fine : band_old;
    wire [2:0]  dir_now   = {band_now, (gx[10] ^ gy[10])};

    // (c) 16 方向量化(给形状识别的 Hough 圆周投票用): 8 个 22.5 度分带 x 符号
    //     分带边界 tan11.25/33.75/45/56.25/67.5/78.75 = 0.199/0.668/1.0/1.497/2.414/5.027
    //     全部用移位加法实现, 不用乘法器
    wire [13:0] ax14 = {3'b0, gx_h};
    wire [13:0] ay14 = {3'b0, ay_h};
    wire [13:0] q22x14 = {2'b0, q22_ax};
    wire [13:0] tb0 = {5'b0, gx_h[10:2]} - {7'b0, gx_h[10:4]} + {9'b0, gx_h[10:6]};   // 0.203|Gx|
    wire [13:0] tb1 = q22x14;                                                        // 0.414|Gx|
    wire [13:0] tb2 = {4'b0, gx_h[10:1]} + {6'b0, gx_h[10:3]} + {8'b0, gx_h[10:5]}
                    + {10'b0, gx_h[10:7]} + {11'b0, gx_h[10:8]};                     // 0.668|Gx|
    wire [13:0] tb3 = ax14;                                                          // 1.000|Gx|
    wire [13:0] tb4 = ax14 + {4'b0, gx_h[10:1]} - {11'b0, gx_h[10:8]};                // 1.496|Gx|
    wire [13:0] tb5 = {ax14[12:0], 1'b0} + q22x14;                                    // 2.414|Gx|
    wire [13:0] tb6 = {ax14[11:0], 2'b0} + ax14 + {11'b0, gx_h[10:5]};                // 5.031|Gx|
    wire [2:0]  band8 = (ay14 < tb0) ? 3'd0 :
                        (ay14 < tb1) ? 3'd1 :
                        (ay14 < tb2) ? 3'd2 :
                        (ay14 < tb3) ? 3'd3 :
                        (ay14 < tb4) ? 3'd4 :
                        (ay14 < tb5) ? 3'd5 :
                        (ay14 < tb6) ? 3'd6 : 3'd7;
    wire [3:0]  dir16_now = {band8, (gx[10] ^ gy[10])};

    reg  [10:0] s14_mag;
    reg  [2:0]  s14_dir;
    reg  [3:0]  s14_dir16;
    reg  [7:0]  s14_gc;
    reg  [23:0] col_s14;
    reg         s14_de, s14_hs, s14_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s14_mag <= 11'd0; s14_dir <= 3'd0; s14_dir16 <= 4'd0;
            s14_gc <= 8'd0; col_s14 <= 24'd0;
            s14_de <= 1'b0; s14_hs <= 1'b0; s14_vs <= 1'b0;
        end else begin
            s14_mag <= mag;
            s14_dir <= dir_now;
            s14_dir16 <= dir16_now;
            s14_gc  <= s13_gc;
            col_s14 <= col_s13;
            s14_de <= s13_de; s14_hs <= s13_hs; s14_vs <= s13_vs;
        end
    end

    //=========================================================================
    // s15: 幅值行缓存0(延迟 1 行), 存储 {dir, mag}
    //=========================================================================
    reg  [17:0]   mb0_mem [0:LB_DEPTH-1];
    reg  [AW-1:0] wr0m, rd0m;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr0m <= {AW{1'b0}};
            rd0m <= LB_INIT;
        end else if (s14_de) begin
            wr0m <= (wr0m == LB_DEPTH-1) ? {AW{1'b0}} : wr0m + 1'b1;
            rd0m <= (rd0m == LB_DEPTH-1) ? {AW{1'b0}} : rd0m + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (s14_de) mb0_mem[wr0m] <= {s14_dir16, s14_dir, s14_mag};
    end

    reg  [17:0] row1_mag;
    reg  [17:0] mag_c1;
    reg  [7:0]  s15_gc;
    reg  [23:0] col_s15;
    reg         s15_de, s15_hs, s15_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row1_mag <= 18'd0; mag_c1 <= 18'd0; s15_gc <= 8'd0; col_s15 <= 24'd0;
            s15_de <= 1'b0; s15_hs <= 1'b0; s15_vs <= 1'b0;
        end else begin
            row1_mag <= mb0_mem[rd0m];
            mag_c1   <= {s14_dir16, s14_dir, s14_mag};
            s15_gc   <= s14_gc;
            col_s15  <= col_s14;
            s15_de <= s14_de; s15_hs <= s14_hs; s15_vs <= s14_vs;
        end
    end

    //=========================================================================
    // s16: 幅值行缓存1(延迟 2 行) + 三行对齐
    //=========================================================================
    reg  [17:0]   mb1_mem [0:LB_DEPTH-1];
    reg  [AW-1:0] wr1m, rd1m;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr1m <= {AW{1'b0}};
            rd1m <= LB_INIT;
        end else if (s15_de) begin
            wr1m <= (wr1m == LB_DEPTH-1) ? {AW{1'b0}} : wr1m + 1'b1;
            rd1m <= (rd1m == LB_DEPTH-1) ? {AW{1'b0}} : rd1m + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (s15_de) mb1_mem[wr1m] <= row1_mag;
    end

    reg  [17:0] row2_mag;
    reg  [17:0] row1_mag_dly;
    reg  [17:0] row0_mag;
    reg  [7:0]  s16_gc;
    reg  [23:0] col_s16;
    reg         s16_de, s16_hs, s16_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row2_mag <= 18'd0; row1_mag_dly <= 18'd0; row0_mag <= 18'd0;
            s16_gc <= 8'd0; col_s16 <= 24'd0;
            s16_de <= 1'b0; s16_hs <= 1'b0; s16_vs <= 1'b0;
        end else begin
            row2_mag     <= mb1_mem[rd1m];
            row1_mag_dly <= row1_mag;
            row0_mag     <= mag_c1;
            s16_gc  <= s15_gc;
            col_s16 <= col_s15;
            s16_de <= s15_de; s16_hs <= s15_hs; s16_vs <= s15_vs;
        end
    end

    //=========================================================================
    // s17: 幅值 3x3 窗口(de 门控, 行间清零)
    //=========================================================================
    reg  [53:0] win2_t, win2_m, win2_b;
    reg  [7:0]  s17_gc;
    reg  [23:0] col_s17;
    reg         s16_de_d, s16_hs_d, s16_vs_d;
    reg  [7:0]  s16_gc_d;
    reg  [23:0] col_s16_d;
    reg         s17_de, s17_hs, s17_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s17_de <= 1'b0; s17_hs <= 1'b0; s17_vs <= 1'b0;
            s17_gc <= 8'd0; col_s17 <= 24'd0;
            s16_de_d <= 1'b0; s16_hs_d <= 1'b0; s16_vs_d <= 1'b0;
            s16_gc_d <= 8'd0; col_s16_d <= 24'd0;
        end else begin
            // NMS must see the same centre pixel as its valid and colour buses.
            s16_de_d <= s16_de; s16_hs_d <= s16_hs; s16_vs_d <= s16_vs;
            s16_gc_d <= s16_gc; col_s16_d <= col_s16;
            s17_de <= s16_de_d; s17_hs <= s16_hs_d; s17_vs <= s16_vs_d;
            s17_gc <= s16_gc_d; col_s17 <= col_s16_d;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            win2_t <= 54'd0; win2_m <= 54'd0; win2_b <= 54'd0;
        end else if (s16_de) begin
            win2_t <= {win2_t[35:0], row2_mag};
            win2_m <= {win2_m[35:0], row1_mag_dly};
            win2_b <= {win2_b[35:0], row0_mag};
        end else begin
            win2_t <= 54'd0; win2_m <= 54'd0; win2_b <= 54'd0;
        end
    end

    wire [17:0] m_tl = win2_t[53:36], m_tm = win2_t[35:18], m_tr = win2_t[17:0];
    wire [17:0] m_ml = win2_m[53:36], m_mm = win2_m[35:18], m_mr = win2_m[17:0];
    wire [17:0] m_bl = win2_b[53:36], m_bm = win2_b[35:18], m_br = win2_b[17:0];

    //=========================================================================
    // s18: 非极大值抑制(沿梯度方向与相邻两点比较)
    //   负方向侧用 >=, 正方向侧用 >, 避免等值平台被整体抹掉
    //   NMS_INTERP=1: 两个斜带按 1/2 权重混合相邻两点 (如 (左+上左)/2), 逼近
    //                 OpenCV 的线性插值 NMS: 22.5~67.5 度之间的斜边不再因为
    //                 "硬切到某一对邻域"而漏检或变双线
    //   NMS_INTERP=0: 退回 v4.1 的 4 邻域硬切
    //=========================================================================
    wire [2:0]  n_dir   = m_mm[13:11];
    wire [3:0]  n_dir16 = m_mm[17:14];
    wire [10:0] n_mag   = m_mm[10:0];
    wire [1:0]  n_band  = n_dir[2:1];
    wire        n_quad  = n_dir[0];

    wire [10:0] nms_a_old = (n_band == 2'd0) ? m_ml[10:0] :
                            (n_band == 2'd3) ? m_tm[10:0] :
                            n_quad           ? m_bl[10:0] : m_tl[10:0];
    wire [10:0] nms_b_old = (n_band == 2'd0) ? m_mr[10:0] :
                            (n_band == 2'd3) ? m_bm[10:0] :
                            n_quad           ? m_tr[10:0] : m_br[10:0];

    wire [10:0] nms_a_int = (n_band == 2'd0) ? m_ml[10:0] :
                            (n_band == 2'd3) ? m_tm[10:0] :
                            (n_band == 2'd1) ? (n_quad ? ((m_ml[10:0] + m_bl[10:0]) >> 1)
                                                       : ((m_ml[10:0] + m_tl[10:0]) >> 1))
                                             : (n_quad ? ((m_tm[10:0] + m_tr[10:0]) >> 1)
                                                       : ((m_tm[10:0] + m_tl[10:0]) >> 1));
    wire [10:0] nms_b_int = (n_band == 2'd0) ? m_mr[10:0] :
                            (n_band == 2'd3) ? m_bm[10:0] :
                            (n_band == 2'd1) ? (n_quad ? ((m_mr[10:0] + m_tr[10:0]) >> 1)
                                                       : ((m_mr[10:0] + m_br[10:0]) >> 1))
                                             : (n_quad ? ((m_bm[10:0] + m_bl[10:0]) >> 1)
                                                       : ((m_bm[10:0] + m_br[10:0]) >> 1));

    wire [10:0] nms_a = (NMS_INTERP != 0) ? nms_a_int : nms_a_old;
    wire [10:0] nms_b = (NMS_INTERP != 0) ? nms_b_int : nms_b_old;

    reg  [10:0] s18_mag;
    reg         s18_soft;
    reg         s18_soft_disp;
    reg  [2:0]  s18_dir;
    reg  [3:0]  s18_dir16;
    reg  [7:0]  s18_gc;
    reg  [23:0] col_s18;
    reg         s18_de, s18_hs, s18_vs;

    // Form a two-pixel-thick red contour without another line buffer.  The
    // current 3x3 magnitude window already contains the left and upper
    // neighbours, so OR-ing those strong flags dilates the one-pixel Sobel
    // line down/right by exactly one pixel.  The same dilated flag feeds the
    // recogniser, which keeps every painted red pixel a tracked edge pixel.
    wire sobel_disp_strong_c = (n_mag[10:3] > sobel_disp_th);
    wire sobel_disp_strong_l = (m_ml[10:3] > sobel_disp_th);
    wire sobel_disp_strong_u = (m_tm[10:3] > sobel_disp_th);
    wire sobel_disp_next     = sobel_disp_strong_c |
                               sobel_disp_strong_l |
                               sobel_disp_strong_u;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s18_mag <= 11'd0; s18_dir <= 3'd0; s18_dir16 <= 4'd0;
            s18_soft <= 1'b0; s18_soft_disp <= 1'b0;
            s18_gc <= 8'd0; col_s18 <= 24'd0;
            s18_de <= 1'b0; s18_hs <= 1'b0; s18_vs <= 1'b0;
        end else begin
            s18_mag <= ((n_mag >= nms_a) && (n_mag > nms_b)) ? n_mag : 11'd0;
            // Direct, non-max-suppressed Sobel magnitude for mode 4.  NMS
            // produces the thin Canny display used by the other modes, but the
            // recognizer benefits from the thicker, continuous Sobel contour.
            s18_soft <= (n_mag[10:3] > i_th_low) || sobel_disp_next;
            s18_soft_disp <= sobel_disp_next;
            s18_dir <= n_dir;
            s18_dir16 <= n_dir16;
            s18_gc  <= s17_gc;
            col_s18 <= col_s17;
            s18_de <= s17_de; s18_hs <= s17_hs; s18_vs <= s17_vs;
        end
    end
    //=========================================================================
    // s19: 双阈值分类 + 状态行缓存0(延迟 1 行)
    //   state: 2'b10=强, 2'b01=弱, 2'b00=无
    //=========================================================================
    wire [7:0] s18_s = s18_mag[10:3];      // 0~255

    reg  [1:0]  s19_state;
    reg         s19_soft;
    reg         s19_soft_disp;
    reg  [2:0]  s19_dir;
    reg  [3:0]  s19_dir16;
    reg  [7:0]  s19_gc;
    reg  [23:0] col_s19;
    reg         s19_de, s19_hs, s19_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s19_state <= 2'd0; s19_soft <= 1'b0; s19_soft_disp <= 1'b0;
            s19_dir <= 3'd0; s19_dir16 <= 4'd0;
            s19_gc <= 8'd0; col_s19 <= 24'd0;
            s19_de <= 1'b0; s19_hs <= 1'b0; s19_vs <= 1'b0;
        end else begin
            s19_state <= (s18_s > i_th_high) ? 2'b10 :
                         (s18_s > i_th_low ) ? 2'b01 : 2'b00;
            s19_soft <= s18_soft;
            s19_soft_disp <= s18_soft_disp;
            s19_dir <= s18_dir;
            s19_dir16 <= s18_dir16;
            s19_gc  <= s18_gc;
            col_s19 <= col_s18;
            s19_de <= s18_de; s19_hs <= s18_hs; s19_vs <= s18_vs;
        end
    end

    reg  [1:0]    sb0_mem [0:LB_DEPTH-1];
    reg  [AW-1:0] wr0s, rd0s;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr0s <= {AW{1'b0}};
            rd0s <= LB_INIT;
        end else if (s19_de) begin
            wr0s <= (wr0s == LB_DEPTH-1) ? {AW{1'b0}} : wr0s + 1'b1;
            rd0s <= (rd0s == LB_DEPTH-1) ? {AW{1'b0}} : rd0s + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (s19_de) sb0_mem[wr0s] <= s19_state;
    end

    //=========================================================================
    // s20: 状态行缓存0 读出 + 当前列状态
    //=========================================================================
    reg  [1:0]  row1_st;
    reg  [1:0]  st_c1;
    reg         s20_soft;
    reg         s20_soft_disp;
    reg  [2:0]  s20_dir;
    reg  [3:0]  s20_dir16;
    reg  [7:0]  s20_gc;
    reg  [23:0] col_s20;
    reg         s20_de, s20_hs, s20_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row1_st <= 2'd0; st_c1 <= 2'd0; s20_soft <= 1'b0; s20_soft_disp <= 1'b0;
            s20_dir <= 3'd0; s20_dir16 <= 4'd0;
            s20_gc <= 8'd0; col_s20 <= 24'd0;
            s20_de <= 1'b0; s20_hs <= 1'b0; s20_vs <= 1'b0;
        end else begin
            row1_st <= sb0_mem[rd0s];
            st_c1   <= s19_state;
            s20_soft <= s19_soft;
            s20_soft_disp <= s19_soft_disp;
            s20_dir <= s19_dir;
            s20_dir16 <= s19_dir16;
            s20_gc  <= s19_gc;
            col_s20 <= col_s19;
            s20_de <= s19_de; s20_hs <= s19_hs; s20_vs <= s19_vs;
        end
    end

    //=========================================================================
    // s21: 状态行缓存1(延迟 2 行) + 三行对齐
    //=========================================================================
    reg  [1:0]    sb1_mem [0:LB_DEPTH-1];
    reg  [AW-1:0] wr1s, rd1s;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr1s <= {AW{1'b0}};
            rd1s <= LB_INIT;
        end else if (s20_de) begin
            wr1s <= (wr1s == LB_DEPTH-1) ? {AW{1'b0}} : wr1s + 1'b1;
            rd1s <= (rd1s == LB_DEPTH-1) ? {AW{1'b0}} : rd1s + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (s20_de) sb1_mem[wr1s] <= row1_st;
    end

    reg  [1:0]  row2_st;
    reg  [1:0]  row1_st_dly;
    reg  [1:0]  row0_st;
    reg         s21_soft;
    reg         s21_soft_disp;
    reg  [2:0]  s21_dir;
    reg  [3:0]  s21_dir16;
    reg  [7:0]  s21_gc;
    reg  [23:0] col_s21;
    reg         s21_de, s21_hs, s21_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row2_st <= 2'd0; row1_st_dly <= 2'd0; row0_st <= 2'd0;
            s21_soft <= 1'b0; s21_soft_disp <= 1'b0;
            s21_dir <= 3'd0; s21_dir16 <= 4'd0; s21_gc <= 8'd0; col_s21 <= 24'd0;
            s21_de <= 1'b0; s21_hs <= 1'b0; s21_vs <= 1'b0;
        end else begin
            row2_st     <= sb1_mem[rd1s];
            row1_st_dly <= row1_st;
            row0_st     <= st_c1;
            s21_soft <= s20_soft;
            s21_soft_disp <= s20_soft_disp;
            s21_dir <= s20_dir;
            s21_dir16 <= s20_dir16;
            s21_gc  <= s20_gc;
            col_s21 <= col_s20;
            s21_de <= s20_de; s21_hs <= s20_hs; s21_vs <= s20_vs;
        end
    end

    //=========================================================================
    // s22: 状态 3x3 窗口(de 门控, 行间清零)
    //=========================================================================
    reg  [5:0]  win3_t, win3_m, win3_b;
    reg         s22_soft;
    reg         s22_soft_disp;
    reg  [2:0]  s22_dir;
    reg  [3:0]  s22_dir16;
    reg  [7:0]  s22_gc;
    reg  [23:0] col_s22;
    reg         s21_de_d, s21_hs_d, s21_vs_d;
    reg  [2:0]  s21_dir_d;
    reg  [3:0]  s21_dir16_d;
    reg         s21_soft_d;
    reg         s21_soft_disp_d;
    reg  [7:0]  s21_gc_d;
    reg  [23:0] col_s21_d;
    reg         s22_de, s22_hs, s22_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s22_de <= 1'b0; s22_hs <= 1'b0; s22_vs <= 1'b0;
            s22_soft <= 1'b0; s22_soft_disp <= 1'b0;
            s22_dir <= 3'd0; s22_dir16 <= 4'd0; s22_gc <= 8'd0; col_s22 <= 24'd0;
            s21_de_d <= 1'b0; s21_hs_d <= 1'b0; s21_vs_d <= 1'b0;
            s21_dir_d <= 3'd0; s21_dir16_d <= 4'd0; s21_soft_d <= 1'b0;
            s21_soft_disp_d <= 1'b0;
            s21_gc_d <= 8'd0;
            col_s21_d <= 24'd0;
        end else begin
            // win3_m is shifted on s21_de; delay the direction/valid bus so the
            // hysteresis decision and its colour describe the same centre pixel.
            s21_de_d <= s21_de; s21_hs_d <= s21_hs; s21_vs_d <= s21_vs;
            s21_dir_d <= s21_dir; s21_dir16_d <= s21_dir16;
            s21_gc_d <= s21_gc; s21_soft_d <= s21_soft;
            s21_soft_disp_d <= s21_soft_disp;
            col_s21_d <= col_s21;
            s22_de <= s21_de_d; s22_hs <= s21_hs_d; s22_vs <= s21_vs_d;
            s22_dir <= s21_dir_d; s22_dir16 <= s21_dir16_d;
            s22_gc <= s21_gc_d; s22_soft <= s21_soft_d;
            s22_soft_disp <= s21_soft_disp_d;
            col_s22 <= col_s21_d;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            win3_t <= 6'd0; win3_m <= 6'd0; win3_b <= 6'd0;
        end else if (s21_de) begin
            win3_t <= {win3_t[3:0], row2_st};
            win3_m <= {win3_m[3:0], row1_st_dly};
            win3_b <= {win3_b[3:0], row0_st};
        end else begin
            win3_t <= 6'd0; win3_m <= 6'd0; win3_b <= 6'd0;
        end
    end

    //=========================================================================
    // s23: 双阈值滞回(连通性传播) + 孤立点剔除
    //   弱边缘保留条件(三者之一):
    //     1) 8 邻域内有强边缘                                (v4 起)
    //     2) 已判决的因果邻域(左/上左/上/上右)已连通到边缘   (v5.0, HYST_CONN)
    //        —— 单遍扫描的连通性传播: 强边缘种子可以沿边缘"传染"给相邻的弱
    //           边缘像素, 于是细而连贯的边缘能整条保留, 而孤立噪点仍然被剔除
    //           (参考开源工程 CannyFPGA / Nitheesh 的连通跟踪做法)
    //     3) 非零邻居数 >= ISOLATE_MIN                        (仅 ISO_EN=1 时启用)
    //=========================================================================
    // The hysteresis centre is taken from win3_m, i.e. one active row behind
    // the direction/colour stream.  Delay those side bands through one real
    // line buffer so the final edge flag, its colour and the source image all
    // describe the same pixel.  A synchronous read also maps to EBR/RAM10.
    localparam integer OUT_DLY_W = 4 + 3 + 8 + 24 + 2;
    reg  [OUT_DLY_W-1:0] out_line_mem [0:LB_DEPTH-1];
    reg  [AW-1:0] wr_out, rd_out;
    reg  [OUT_DLY_W-1:0] out_line_q;

    // Prefetch the next synchronous-RAM address while the active stream is
    // running.  At cycle t the RAM output register already contains
    // out_line_mem[rd_out(t)], so s23 sees exactly the same side-band value
    // as the old asynchronous read.  During a DE gap the address holds the
    // current pointer, which also makes the first pixel of the next run
    // immediately valid.
    wire [AW-1:0] rd_out_next = (rd_out == LB_DEPTH-1) ? {AW{1'b0}} :
                                 rd_out + 1'b1;
    wire [AW-1:0] out_rd_addr = s22_de ? rd_out_next : rd_out;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_out <= {AW{1'b0}};
            rd_out <= LB_INIT;
        end else if (s22_de) begin
            wr_out <= (wr_out == LB_DEPTH-1) ? {AW{1'b0}} : wr_out + 1'b1;
            rd_out <= (rd_out == LB_DEPTH-1) ? {AW{1'b0}} : rd_out + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (s22_de)
            out_line_mem[wr_out] <= {s22_dir16, s22_dir, s22_gc, col_s22,
                                     s22_soft_disp, s22_soft};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            out_line_q <= {OUT_DLY_W{1'b0}};
        else
            out_line_q <= out_line_mem[out_rd_addr];
    end

    wire [1:0] st_c  = win3_m[3:2];
    wire st_tl = win3_t[5], st_tm = win3_t[3], st_tr = win3_t[1];
    wire st_ml = win3_m[5],                      st_mr = win3_m[1];
    wire st_bl = win3_b[5], st_bm = win3_b[3], st_br = win3_b[1];

    wire strong_near = st_tl | st_tm | st_tr | st_ml | st_mr |
                       st_bl | st_bm | st_br;

    wire nz_tl = |win3_t[5:4], nz_tm = |win3_t[3:2], nz_tr = |win3_t[1:0];
    wire nz_ml = |win3_m[5:4],                      nz_mr = |win3_m[1:0];
    wire nz_bl = |win3_b[5:4], nz_bm = |win3_b[3:2], nz_br = |win3_b[1:0];

    wire [3:0] nz_cnt = nz_tl + nz_tm + nz_tr + nz_ml +
                        nz_mr + nz_bl + nz_bm + nz_br;

    // ---- 单遍扫描的因果连通性 (HYST_CONN) ----------------------------------
    //   hb_mem 存上一行的"最终判决"位; 读写指针差 LB_INIT -> 读到的是上一行同列
    //   hp1/hp2/hp3 延迟链 -> 上一行的 列+1 / 列 / 列-1 (上右/上/上左)
    //   s23_final -> 左邻; 这 4 个邻域都是"已经判决"的, 所以可以在一条流水线里
    //   边判决边传播, 不需要第二遍扫描
    reg          hb_mem [0:LB_DEPTH-1];
    reg  [AW-1:0] wrh, rdh;
    reg          hp1, hp2, hp3;
    reg          s22_de_d;
    reg          s23_final;

    wire [AW-1:0] rdh_p1 = (rdh == LB_DEPTH-1) ? {AW{1'b0}} : rdh + 1'b1;
    // hp1_g 等价于原来"消隐期把 hp1 清零"的行为:
    //   hp1 现在是 hb_mem 的纯同步读寄存器(可推断 RAM10), 读口不能带 else 清零,
    //   所以把清零挪到这一级: hp1_g(t) = s22_de(t-1) ? mem[rdh_p1(t-1)] : 0
    //   = 原 hp1(t), 逐拍完全等价.
    wire          hp1_g  = s22_de_d ? hp1 : 1'b0;
    wire          conn   = s23_final | hp3 | hp2 | hp1_g;
    wire weak_ok   = strong_near | ((ISO_EN != 0) & (nz_cnt >= ISO_MIN[3:0])) |
                     ((HYST_CONN != 0) & conn);
    wire final_now = st_c[1] | (st_c[0] & weak_ok);

    reg  [2:0]  s23_dir;
    reg  [3:0]  s23_dir16;
    reg         s23_soft;
    reg         s23_soft_disp;
    reg  [7:0]  s23_gc;
    reg  [23:0] col_s23;
    reg         s23_de, s23_hs, s23_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wrh <= {AW{1'b0}};
            rdh <= LB_INIT;
        end else if (s22_de) begin
            wrh <= (wrh == LB_DEPTH-1) ? {AW{1'b0}} : wrh + 1'b1;
            rdh <= (rdh == LB_DEPTH-1) ? {AW{1'b0}} : rdh + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (s22_de) hb_mem[wrh] <= final_now;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) s22_de_d <= 1'b0;
        else        s22_de_d <= s22_de;
    end

    // hb_mem 读口: 无条件同步读 -> Efinity 可推断 RAM10
    //   (原来写成 if (s22_de) hp1<=mem[..] else hp1<=0, 读口被判"非同步",
    //    2048x1 被降级成 logic memory = 2048 FF + 3343 LUT4)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) hp1 <= 1'b0;
        else        hp1 <= hb_mem[rdh_p1];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hp2 <= 1'b0; hp3 <= 1'b0;
        end else if (s22_de) begin
            hp2 <= hp1_g;
            hp3 <= hp2;
        end else begin
            hp2 <= 1'b0; hp3 <= 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s23_final <= 1'b0; s23_dir <= 3'd0; s23_dir16 <= 4'd0;
            s23_soft <= 1'b0; s23_soft_disp <= 1'b0;
            s23_gc <= 8'd0; col_s23 <= 24'd0;
            s23_de <= 1'b0; s23_hs <= 1'b0; s23_vs <= 1'b0;
        end else begin
            s23_final <= final_now;
            if (s22_de) begin
                {s23_dir16, s23_dir, s23_gc, col_s23,
                 s23_soft_disp, s23_soft} <= out_line_q;
            end else begin
                s23_dir <= 3'd0; s23_dir16 <= 4'd0;
                s23_gc <= 8'd0; col_s23 <= 24'd0;
                s23_soft <= 1'b0; s23_soft_disp <= 1'b0;
            end
            s23_de <= s22_de; s23_hs <= s22_hs; s23_vs <= s22_vs;
        end
    end

    //=========================================================================
    // s24: 输出(分屏 / 彩色叠加 / 画中画 / 边缘着色 / 帧首行首屏蔽 / 加粗)
    //   i_color_mode: 0=按梯度方向着色(竖直边红/水平边绿/斜边黄青) 1=统一红
    //                 2=统一白 3=统一绿   —— 赛题"红色边缘线叠加原始彩色图"= 模式1
    //   i_pip = 1: 全屏彩色原图, 右下角 PIP_W x PIP_H 小窗显示二值边缘(画中画)
    //   edge_white 同时做了两件事:
    //     (a) 帧首 TOP_BLANK 行不出边缘 -> 消除行缓存跨帧残留的满行白线
    //     (b) 行首行尾 COL_BLANK 列不出边缘 -> 消除 3x3 窗口填充的满列白线
    //=========================================================================
    reg  [AW:0] o_col;
    reg  [AW:0] o_rowc;
    reg         s23_vs_d;
    reg         final_d;       // 边缘 2 像素水平加粗用
    reg         sobel_disp_d;  // 上一像素强红边, 只扩展一像素

    wire edge_raw   = (EDGE_DILATE != 0) ? (s23_final | final_d) : s23_final;
    // Blank by the actual displayed row counter.  A counter derived from de
    // gaps can disagree with o_row after a frame transition and leave warm-up
    // edges visible on row 0/5 of a new frame.
    wire frame_blank = (o_rowc < TOP_BLANK);
    wire edge_white = edge_raw &&
                      !frame_blank &&
                      (o_col >= COL_BLANK) &&
                      (o_col <= (H_ACTIVE - 1 - COL_BLANK));

    // Raw Sobel contour for display mode 4.  It is carried through the same
    // line delay as the Canny side-band, so it lands on the exact pixel whose
    // original colour is being drawn.  shape_proc must receive this same map:
    // using the sparser Canny map here made the recognizer operate on a
    // different edge image from the one visible on the monitor.
    // Mode 4 uses the low threshold Sobel map for geometry recognition, but a
    // stricter Sobel threshold for the red overlay.  Using the raw 2/255 map
    // for display painted skin/fabric texture almost solid red while smooth
    // white areas stayed neutral, which looked exactly like a white-balance
    // cast.  The recognizer keeps full sensitivity; only the overlay is sparse.
    wire sobel_shape = s23_soft &&
                       !frame_blank &&
                       (o_col >= COL_BLANK) &&
                       (o_col <= (H_ACTIVE - 1 - COL_BLANK));
    wire sobel_display = s23_soft_disp &&
                         !frame_blank &&
                         (o_col >= COL_BLANK) &&
                         (o_col <= (H_ACTIVE - 1 - COL_BLANK));
    // s18_soft_disp already contains a one-pixel 2-D dilation.  Do not expand
    // it again here, otherwise the red line becomes an asymmetric 3-pixel
    // staircase and drifts relative to the recogniser map.
    wire sobel_display_wide = sobel_display;
    wire sobel_recognition_wide = sobel_shape;
    wire display_edge = i_shape_mode ? sobel_display_wide : edge_white;
    wire recognition_edge = i_shape_mode ? sobel_recognition_wide : edge_white;

    // 方向彩色边缘: band0 梯度近水平 -> 边缘近竖直 = 红; band3 -> 边缘近水平 = 绿;
    //               band1 = 黄; band2 = 青 (像素格式 {B,G,R})
    wire [23:0] dir_col = (s23_dir[2:1] == 2'd0) ? 24'h0000FF :
                          (s23_dir[2:1] == 2'd3) ? 24'h00FF00 :
                          (s23_dir[2:1] == 2'd1) ? 24'h00FFFF : 24'hFFFF00;
    wire [23:0] edge_col = (EDGE_COLOR_DIR == 0) ? 24'h0000FF :
                           (i_color_mode == 2'd1) ? 24'h0000FF :
                           (i_color_mode == 2'd2) ? 24'hFFFFFF :
                           (i_color_mode == 2'd3) ? 24'h00FF00 : dir_col;

    // 画中画窗口(右下角)
    wire in_pip = (o_col >= PIP_X0) && (o_col < PIP_X1) &&
                  (o_rowc >= PIP_Y0) && (o_rowc < PIP_Y1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_col <= {(AW+1){1'b0}};
            o_rowc <= {(AW+1){1'b0}};
            o_de <= 1'b0; o_hs <= 1'b0; o_vs <= 1'b0;
            o_rgb <= 24'd0;
            s23_vs_d <= 1'b0;
            final_d  <= 1'b0;
            sobel_disp_d <= 1'b0;
            o_edge_flag <= 1'b0;
            o_edge <= 1'b0; o_dir16 <= 4'd0; o_row <= 11'd0;
        end else begin
            final_d  <= s23_final;
            sobel_disp_d <= sobel_display;
            s23_vs_d <= s23_vs;

            if (s23_de) begin
                if (o_col == H_ACTIVE-1) begin
                    o_col <= {(AW+1){1'b0}};
                    o_rowc <= o_rowc + 1'b1;
                end else
                    o_col <= o_col + 1'b1;
            end else
                o_col <= {(AW+1){1'b0}};

            if (s23_vs && !s23_vs_d)
                o_rowc <= {(AW+1){1'b0}};

            o_de <= s23_de;
            o_hs <= s23_hs;
            o_vs <= s23_vs;

            // The first few active lines still contain RGB/gray data from the
            // previous frame while the 3x3 line buffers roll over.  Mask the
            // whole pixel bus there, not just the edge flag, otherwise a
            // previous-frame white row appears as a bright line at the top.
            if (frame_blank)
                o_rgb <= 24'h000000;
            else if (i_shape_mode)
                o_rgb <= sobel_display_wide ? 24'h0000FF : col_s23;
            else if (i_pip) begin
                // 画中画: 全屏彩色原图, 右下角小窗显示二值边缘
                o_rgb <= (in_pip && edge_white) ? 24'hFFFFFF :
                         (in_pip ? 24'h000000 : col_s23);
            end else if (o_col < i_split_col)
                o_rgb <= {s23_gc, s23_gc, s23_gc};              // 左半屏: 灰度图
            else if (i_overlay)
                o_rgb <= edge_white ? edge_col : col_s23;       // 右半屏: 彩色 + 边缘
            else
                o_rgb <= edge_white ? 24'hFFFFFF : 24'h000000;  // 右半屏: 二值边缘

            // 边缘活动标志(新帧清零)
            if (s23_vs && !s23_vs_d)
                o_edge_flag <= 1'b0;
            else if (s23_de && display_edge)
                o_edge_flag <= 1'b1;

            // 随路给 shape_proc.  Mode 4 recognition uses the low-threshold Sobel
            // contour; the red overlay uses a higher Sobel threshold, so faint
            // texture is tracked but not painted.  Other modes keep Canny.
            o_edge  <= recognition_edge;
            o_dir16 <= s23_dir16;
            o_row <= o_rowc[AW-1:0];
        end
    end

endmodule


//=============================================================================
// shape_proc : 简易目标识别(圆/矩形/三角形) —— v6.2 起并入本文件
//   原因: Efinity GUI 保存工程时会把 rtl/edge/shape_proc.v 从设计文件列表里删掉,
//         导致 "instantiating unknown module 'shape_proc'" 编不过.
//         放进 edge_proc.v(GUI 一定保留)后, 工程不再依赖额外的文件登记.
//   rtl/edge/shape_proc.v 现在只剩一段注释占位, 万一被 GUI 加回文件列表也无害.
//=============================================================================
module shape_proc #(
    parameter integer H_ACTIVE     = 1920,
    parameter integer V_ACTIVE     = 1080,
    parameter integer BIN_SHIFT    = 3,        // 累加器降采样(1/8 分辨率)
    parameter integer ACC_W        = 256,      // 累加器行跨度(2 的幂, 免乘法)
    parameter integer RECT_LEN_MIN = 160,      // "长边"最小长度(像素)
    parameter integer RECT_LEN_MAX = 360,      // "长边"最大长度(过滤板框/桌面长线)
    parameter integer RECT_GAP_MIN = 64,       // 上下长边最小间距(像素)
    parameter integer RECT_SIDE_MIN= 64,       // 左右最小间距(像素)
    parameter integer RUN_GAP      = 3,        // 行程间隙容差
    parameter integer TOL_PX       = 2,        // 矩形竖边验证的列容差
    parameter integer R0           = 40,       // 起始半径
    parameter integer R_STEP       = 16,       // 半径步进
    parameter integer R_NUM        = 10,       // 半径档数
    parameter integer HOUGH_TH     = 45,       // 圆投票峰值阈值
    parameter integer CIRC_RATIO_SHIFT = 4,    // 圆峰值需 >= 帧内边缘数/2^N
    parameter integer CIRC_MIN_EDGE    = 24,   // 圆检测所需最小帧内边缘数
    parameter integer CIRC_CORNER_MAX  = 64,   // 候选框四角允许的边缘像素数(纹理容差)
    parameter integer CIRC_ARC_TOL     = 8,    // 椭圆弧采样窗口半宽(像素)
    parameter integer CIRC_ARC_MIN     = 6,    // 8 个弧采样点至少命中数
    parameter integer CIRC_DIAG_MIN    = 16,   // 45度弧窗内的斜向边缘像素下限
    parameter integer CIRC_HOLD    = 90,       // 圆结果保持帧数
    parameter integer RECT_HOLD    = 90,       // 矩形结果保持帧数
    parameter integer TRI_W_MIN    = 72,       // 三角形最小底边宽度
    parameter integer TRI_TOL_PX   = 12,       // 三角形斜边验证列容差
    parameter integer TRI_HOLD     = 90        // 三角形结果保持帧数
)(
    input  wire        clk,
    input  wire        rst_n,

    // 视频输入(来自 edge_proc)
    input  wire        i_de,
    input  wire        i_hs,
    input  wire        i_vs,
    input  wire [23:0] i_rgb,
    input  wire        i_edge,     // 最终边缘标志
    input  wire [3:0]  i_dir16,    // {band8[2:0], quad}
    input  wire [10:0] i_row,      // 当前行号(帧内)

    input  wire        i_en,       // 1 = 使能形状识别(显示模式 4)

    input  wire [11:0] i_draw_x0,  // 识别/绘制 ROI 起始列(模式4=0, 全屏识别)

    // 视频输出(叠加识别结果)
    output reg         o_de,
    output reg         o_hs,
    output reg         o_vs,
    output reg  [23:0] o_rgb,

    output reg         o_shape_hit, // 当前锁定了矩形或圆(LED 指示)
    output wire [3:0]  o_dbg        // 调试: {rect_valid, circ_valid, clr_done, fifo_drop}
);

    localparam integer ACC_H    = (V_ACTIVE >> BIN_SHIFT) + 1;
    localparam integer ACC_SIZE = ACC_W * ACC_H;
    localparam integer YLIM     = ACC_H << BIN_SHIFT;

    localparam [11:0] RLEN_C = RECT_LEN_MIN;
    localparam [11:0] RLMAX_C = RECT_LEN_MAX;
    localparam [11:0] RGAP_C = RECT_GAP_MIN;
    localparam [11:0] RSID_C = RECT_SIDE_MIN;
    localparam [11:0] TOL_C  = TOL_PX;
    localparam [3:0]  RUNG_C = RUN_GAP;
    localparam [7:0]  RTH_C  = HOUGH_TH;
    localparam integer CRSH_C = CIRC_RATIO_SHIFT;
    localparam [15:0] CMIN_C = CIRC_MIN_EDGE;
    localparam [15:0] CMAX_C = CIRC_CORNER_MAX;
    localparam [12:0] CATOL_C = CIRC_ARC_TOL;
    localparam [3:0]  CAMIN_C = CIRC_ARC_MIN;
    localparam [15:0] CDGM_C  = CIRC_DIAG_MIN;
    localparam [7:0]  CHLD_C = CIRC_HOLD;
    localparam [7:0]  RHLD_C = RECT_HOLD;
    localparam [3:0]  RNUM_C = R_NUM;
    localparam [11:0] TWMN_C = TRI_W_MIN;
    localparam [12:0] TOL3_C = TRI_TOL_PX;
    localparam [7:0]  THLD_C = TRI_HOLD;

    //-------------------------------------------------------------------------
    // 行/列计数
    //-------------------------------------------------------------------------
    reg  [11:0] col;
    reg  [11:0] col_d;
    reg         vs_d;
    wire        vs_rise = i_vs & ~vs_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col <= 12'd0; col_d <= 12'd0; vs_d <= 1'b0;
        end else begin
            vs_d  <= i_vs;
            col_d <= col;
            if (!i_de)                  col <= 12'd0;
            else if (col == H_ACTIVE-1) col <= 12'd0;
            else                        col <= col + 1'b1;
        end
    end

    wire rst_frame = vs_rise;
    wire end_line  = (!i_de) && (col_d != 12'd0);

    // Per-frame edge evidence.  A fixed Hough threshold alone is vulnerable to
    // dense textures (checkerboards, foliage): straight lines can accidentally
    // pile enough votes into one 1/8-resolution bin.  At the frame boundary the
    // Hough peak must also reach 5/64 of all ROI edge votes.  The 5/64 value
    // sits between the clean-checker peak (~1/16) and a real ellipse on the same
    // checker (~1/10), while using only shifts and one add.
    reg [15:0] frame_edge_count;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            frame_edge_count <= 16'd0;
        else if (rst_frame)
            frame_edge_count <= 16'd0;
        else if (i_en && i_de && i_edge && (col >= i_draw_x0) &&
                 (frame_edge_count != 16'hFFFF))
            frame_edge_count <= frame_edge_count + 1'b1;
    end

    //-------------------------------------------------------------------------
    // 半径扫描档位
    //-------------------------------------------------------------------------
    reg  [3:0]  frame_k;
    reg  [2:0]  frame_asp;
    wire [11:0] cur_r = R0 + frame_k * R_STEP;
    // The demo sheet can be viewed upright or rotated by roughly 90 degrees.
    // Scan four fixed aspect ratios instead of assuming a tall ellipse:
    //   0: 21/16 = 1.3125 (mild perspective)
    //   1: 25/16 = 1.5625 (portrait target / tall ellipse)
    //   2:  5/8  = 0.625  (rotated target / wide ellipse)
    //   3:  1/2  = 0.5    (very wide printed ellipse on the target sheet)
    //   4:  1    = 1.0    (true circle; scanned last because a tall ellipse
    //                       can otherwise lock on one of its near-circular arcs)
    // Keep the three perspective hypotheses first: a tall ellipse contains
    // large near-circular arc segments that can lock early if 1.0 is scanned
    // before the correct 1.5625 hypothesis.
    // Each complete radius list is scanned at one aspect, then repeated at the
    // next.  The candidate keeps its own rx/ry, so the arc verifier and the
    // final bounding box always use the hypothesis that produced the peak.
    wire [11:0] cur_ry = (frame_asp == 3'd0) ?
                         (cur_r + (cur_r >> 2) + (cur_r >> 4)) :
                         (frame_asp == 3'd1) ?
                         (cur_r + (cur_r >> 1) + (cur_r >> 4)) :
                         (frame_asp == 3'd2) ?
                         ((cur_r >> 1) + (cur_r >> 3)) :
                         (frame_asp == 3'd3) ? (cur_r >> 1) : cur_r;

    //-------------------------------------------------------------------------
    // 边缘/方向/坐标 打拍(检测域, 统一延后 1 拍)
    //-------------------------------------------------------------------------
    reg         d_edge;
    reg  [3:0]  d_dir;
    reg  [11:0] d_col, d_row;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d_edge <= 1'b0; d_dir <= 4'd0; d_col <= 12'd0; d_row <= 12'd0;
        end else begin
            d_edge <= i_edge;
            d_dir  <= i_dir16;
            d_col  <= col;
            d_row  <= {1'b0, i_row};
        end
    end

    //-------------------------------------------------------------------------
    // 1) 行程统计 + 全帧 top-3 水平长行程 (单 always, 避免多驱动)
    //    带间隙容差的行程: 间隙 <= RUN_GAP 视为同一行程(抗边缘虚线化)
    //    注意: i_row 在 edge_proc 里是行尾自增的, 行末(end_line)时已经是下一行,
    //          所以登记 y 坐标用 row_cur = i_row-1, 否则候选框整体下移一行
    //
    //    v6.4: 由 top-2 扩为 top-3. 目标图上三角形底边比矩形宽, 矩形上下边
    //          会占据第 2/3 名; 只保留 top-2 时矩形下边永远被丢掉.
    //          三条线两两组合 + 每帧轮换验证, 3 帧覆盖全部组合.
    //-------------------------------------------------------------------------
    reg  [11:0] run_len, run_x0, run_x1;
    reg  [3:0]  run_gap;
    reg  [11:0] rb_len, rb_x0, rb_x1;      // 本行已收尾的最长行程

    reg  [11:0] h1_len, h1_y, h1_x0, h1_x1;
    reg  [11:0] h2_len, h2_y, h2_x0, h2_x1;
    reg  [11:0] h3_len, h3_y, h3_x0, h3_x1;
    reg         h1_v, h2_v, h3_v;

    wire [11:0] row_cur = {1'b0, i_row} - 12'd1;

    // 行末: 把刚结束的行程(run_*)与本行已登记的最长行程(rb_*)合成该行代表行程
    // The board border/table edge can be much longer than a demo shape.  If
    // those lines are allowed into the top-3 list they occupy every slot and
    // the rectangle/triangle never get validated.  Only keep plausible shape
    // widths; this is the main sensitivity fix for a full-frame camera view.
    wire        run_ok    = (run_len >= RLEN_C) && (run_len <= RLMAX_C);
    wire        run_win   = run_ok && (run_len > rb_len);
    wire [11:0] cur_len   = run_win ? run_len : rb_len;
    wire [11:0] cur_x0    = run_win ? run_x0  : rb_x0;
    wire [11:0] cur_x1    = run_win ? run_x1  : rb_x1;
    wire        cur_valid = run_ok || (rb_len >= RLEN_C);

    // 同一行上可能同时出现两个图形(例如三角形底边和矩形下边). 只有 y 接近
    // 且 x 区间重叠才算同一条线; 左右分开的同高度线段会占据不同的槽位.
    wire [11:0] h1_dy = (row_cur > h1_y) ? (row_cur - h1_y) : (h1_y - row_cur);
    wire [11:0] h2_dy = (row_cur > h2_y) ? (row_cur - h2_y) : (h2_y - row_cur);
    wire [11:0] h3_dy = (row_cur > h3_y) ? (row_cur - h3_y) : (h3_y - row_cur);
    wire [12:0] cx0e = {1'b0, cur_x0};
    wire [12:0] cx1e = {1'b0, cur_x1};
    wire h1_xov = (cx0e <= ({1'b0,h1_x1} + 13'd4)) && ((cx1e + 13'd4) >= {1'b0,h1_x0});
    wire h2_xov = (cx0e <= ({1'b0,h2_x1} + 13'd4)) && ((cx1e + 13'd4) >= {1'b0,h2_x0});
    wire h3_xov = (cx0e <= ({1'b0,h3_x1} + 13'd4)) && ((cx1e + 13'd4) >= {1'b0,h3_x0});
    wire same1 = h1_v && (h1_dy < RGAP_C) && h1_xov;
    wire same2 = h2_v && (h2_dy < RGAP_C) && h2_xov;
    wire same3 = h3_v && (h3_dy < RGAP_C) && h3_xov;

    wire h1_min = (h1_len <= h2_len) && (h1_len <= h3_len);
    wire h2_min = !h1_min && (h2_len <= h3_len);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            run_len <= 12'd0; run_x0 <= 12'd0; run_x1 <= 12'd0; run_gap <= 4'd0;
            rb_len <= 12'd0; rb_x0 <= 12'd0; rb_x1 <= 12'd0;
            h1_len <= 12'd0; h1_y <= 12'd0; h1_x0 <= 12'd0; h1_x1 <= 12'd0;
            h2_len <= 12'd0; h2_y <= 12'd0; h2_x0 <= 12'd0; h2_x1 <= 12'd0;
            h3_len <= 12'd0; h3_y <= 12'd0; h3_x0 <= 12'd0; h3_x1 <= 12'd0;
            h1_v <= 1'b0; h2_v <= 1'b0; h3_v <= 1'b0;
        end else if (rst_frame) begin
            run_len <= 12'd0; run_gap <= 4'd0; rb_len <= 12'd0;
            h1_v <= 1'b0; h2_v <= 1'b0; h3_v <= 1'b0;
        end else if (end_line) begin
            if (cur_valid) begin
                if (same1) begin
                    if (cur_len > h1_len) begin
                        h1_len <= cur_len; h1_y <= row_cur;
                        h1_x0 <= cur_x0; h1_x1 <= cur_x1;
                    end
                end else if (same2) begin
                    if (cur_len > h2_len) begin
                        h2_len <= cur_len; h2_y <= row_cur;
                        h2_x0 <= cur_x0; h2_x1 <= cur_x1;
                    end
                end else if (same3) begin
                    if (cur_len > h3_len) begin
                        h3_len <= cur_len; h3_y <= row_cur;
                        h3_x0 <= cur_x0; h3_x1 <= cur_x1;
                    end
                end else if (!h1_v) begin
                    h1_v <= 1'b1; h1_len <= cur_len; h1_y <= row_cur;
                    h1_x0 <= cur_x0; h1_x1 <= cur_x1;
                end else if (!h2_v) begin
                    h2_v <= 1'b1; h2_len <= cur_len; h2_y <= row_cur;
                    h2_x0 <= cur_x0; h2_x1 <= cur_x1;
                end else if (!h3_v) begin
                    h3_v <= 1'b1; h3_len <= cur_len; h3_y <= row_cur;
                    h3_x0 <= cur_x0; h3_x1 <= cur_x1;
                end else if ((cur_len > h1_len) || (cur_len > h2_len) || (cur_len > h3_len)) begin
                    if (h1_min) begin
                        h1_len <= cur_len; h1_y <= row_cur;
                        h1_x0 <= cur_x0; h1_x1 <= cur_x1;
                    end else if (h2_min) begin
                        h2_len <= cur_len; h2_y <= row_cur;
                        h2_x0 <= cur_x0; h2_x1 <= cur_x1;
                    end else begin
                        h3_len <= cur_len; h3_y <= row_cur;
                        h3_x0 <= cur_x0; h3_x1 <= cur_x1;
                    end
                end
            end
            run_len <= 12'd0; run_gap <= 4'd0; rb_len <= 12'd0;
        end else if (i_edge && (col >= i_draw_x0)) begin
            run_gap <= 4'd0;
            if (run_len == 12'd0) run_x0 <= col;
            run_x1  <= col;
            run_len <= run_len + 1'b1;
        end else if (run_len != 12'd0) begin
            if (run_gap >= RUNG_C) begin
                if (run_ok && (run_len > rb_len)) begin
                    rb_len <= run_len; rb_x0 <= run_x0; rb_x1 <= run_x1;
                end
                run_len <= 12'd0; run_gap <= 4'd0;
            end else
                run_gap <= run_gap + 1'b1;
        end
    end

    //-------------------------------------------------------------------------
    // 2) 候选框 + 下一帧四边覆盖率验证
    //    rot 每帧 +1, 三帧轮换:
    //      rot=0: 矩形(h1,h2), 三角形底边 h1
    //      rot=1: 矩形(h2,h3), 三角形底边 h2
    //      rot=2: 矩形(h1,h3), 三角形底边 h3
    //    每帧只做一组, 省掉并行三套覆盖率计数; 静态目标 3 帧内必被验证一次.
    //-------------------------------------------------------------------------
    reg  [11:0] cand_x0, cand_y0, cand_x1, cand_y1;
    reg         cand_ready;
    reg  [11:0] rect_x0, rect_y0, rect_x1, rect_y1;
    reg         rect_valid;
    reg  [7:0]  rect_hold;
    reg  [1:0]  rot;

    wire [1:0] rot_n = (rot == 2'd2) ? 2'd0 : (rot + 1'b1);

    // 轮换选取矩形的两条水平边
    reg  [11:0] pa_y, pa_x0, pa_x1;
    reg  [11:0] pb_y, pb_x0, pb_x1;
    reg         pair_v;
    always @(*) begin
        case (rot_n)
            2'd0: begin
                pa_y=h1_y; pa_x0=h1_x0; pa_x1=h1_x1;
                pb_y=h2_y; pb_x0=h2_x0; pb_x1=h2_x1;
                pair_v = h1_v & h2_v;
            end
            2'd1: begin
                pa_y=h2_y; pa_x0=h2_x0; pa_x1=h2_x1;
                pb_y=h3_y; pb_x0=h3_x0; pb_x1=h3_x1;
                pair_v = h2_v & h3_v;
            end
            default: begin
                pa_y=h1_y; pa_x0=h1_x0; pa_x1=h1_x1;
                pb_y=h3_y; pb_x0=h3_x0; pb_x1=h3_x1;
                pair_v = h1_v & h3_v;
            end
        endcase
    end

    wire [11:0] ov_x0 = (pa_x0 > pb_x0) ? pa_x0 : pb_x0;
    wire [11:0] ov_x1 = (pa_x1 < pb_x1) ? pa_x1 : pb_x1;
    wire [11:0] ov_y0 = (pa_y  < pb_y ) ? pa_y  : pb_y;
    wire [11:0] ov_y1 = (pa_y  > pb_y ) ? pa_y  : pb_y;

    // 轮换选取三角形底边
    reg  [11:0] tb_len, tb_y, tb_x0, tb_x1;
    reg         tb_v;
    always @(*) begin
        case (rot_n)
            2'd0: begin tb_len=h1_len; tb_y=h1_y; tb_x0=h1_x0; tb_x1=h1_x1; tb_v=h1_v; end
            2'd1: begin tb_len=h2_len; tb_y=h2_y; tb_x0=h2_x0; tb_x1=h2_x1; tb_v=h2_v; end
            default: begin tb_len=h3_len; tb_y=h3_y; tb_x0=h3_x0; tb_x1=h3_x1; tb_v=h3_v; end
        endcase
    end

    // 竖边用"每行命中标志"而不是逐像素计数: 一条竖边每行只加 1,
    // 避免抗锯齿/2 像素线宽把覆盖率放大 4 倍而产生误框.
    reg  [15:0] vl_cnt, vr_cnt, hh_cnt0, hh_cnt1;
    reg  [11:0] vl_xmin, vr_xmax;
    reg         rect_row_l, rect_row_r;

    wire [12:0] col13  = {1'b0, col};
    // Rounded corners shorten the straight top/bottom run by roughly one
    // corner radius on each side (the web regression icon loses ~38 px), so a
    // +-10 px vertical window never sees the real side edges and vert_ok stays
    // low.  Allow a wider margin for reasonably wide boxes; the four-side
    // coverage test still rejects isolated lines.
    wire [12:0] cand_w = {1'b0, cand_x1} - {1'b0, cand_x0};
    wire [12:0] vtol   = (cand_w >= 13'd120) ?
                         ({1'b0, TOL_C} + 13'd40) : {1'b0, TOL_C};
    wire [12:0] lx_lo  = ({1'b0, cand_x0} > vtol) ?
                         ({1'b0, cand_x0} - vtol) : 13'd0;
    wire [12:0] lx_hi  = {1'b0, cand_x0} + vtol;
    wire [12:0] rx_lo  = ({1'b0, cand_x1} > vtol) ?
                         ({1'b0, cand_x1} - vtol) : 13'd0;
    wire [12:0] rx_hi  = {1'b0, cand_x1} + vtol;
    // Only near-vertical edges may count as a rectangle side.  Without this,
    // the wider corner window can pick up a triangle's 60-degree side (its
    // direction band is ~2-3) and fabricate a rectangle around it.  Bands 0/1
    // are within 22.5 degrees of vertical, which still tolerates camera tilt.
    wire side_vert = (i_dir16[3:1] <= 3'd1);
    wire hit_lx   = (col13 >= lx_lo) && (col13 <= lx_hi);
    wire hit_rx   = (col13 >= rx_lo) && (col13 <= rx_hi);
    wire in_yspan = (i_row > cand_y0) && (i_row < cand_y1);
    wire in_xspan = (col   > cand_x0) && (col   < cand_x1);
    wire hit_hy0  = (i_row >= cand_y0) && (i_row <= (cand_y0 + 12'd1));
    wire hit_hy1  = (i_row >= (cand_y1 - 12'd1)) && (i_row <= cand_y1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vl_cnt <= 16'd0; vr_cnt <= 16'd0; hh_cnt0 <= 16'd0; hh_cnt1 <= 16'd0;
            vl_xmin <= 12'hFFF; vr_xmax <= 12'd0;
            rect_row_l <= 1'b0; rect_row_r <= 1'b0;
        end else if (rst_frame) begin
            vl_cnt <= 16'd0; vr_cnt <= 16'd0; hh_cnt0 <= 16'd0; hh_cnt1 <= 16'd0;
            vl_xmin <= 12'hFFF; vr_xmax <= 12'd0;
            rect_row_l <= 1'b0; rect_row_r <= 1'b0;
        end else if (end_line) begin
            if (cand_ready && i_en && (row_cur > cand_y0) && (row_cur < cand_y1)) begin
                if (rect_row_l) vl_cnt <= vl_cnt + 1'b1;
                if (rect_row_r) vr_cnt <= vr_cnt + 1'b1;
            end
            rect_row_l <= 1'b0; rect_row_r <= 1'b0;
        end else if (cand_ready && i_en && i_de && (col >= i_draw_x0)) begin
            if (in_yspan && i_edge && side_vert) begin
                if (hit_lx) begin
                    rect_row_l <= 1'b1;
                    if (col < vl_xmin) vl_xmin <= col;
                end
                if (hit_rx) begin
                    rect_row_r <= 1'b1;
                    if (col > vr_xmax) vr_xmax <= col;
                end
            end
            if (i_edge && in_xspan) begin
                if (hit_hy0) hh_cnt0 <= hh_cnt0 + 1'b1;
                if (hit_hy1) hh_cnt1 <= hh_cnt1 + 1'b1;
            end
        end
    end

    wire [11:0] vh_trg = (cand_y1 > cand_y0) ? (cand_y1 - cand_y0 - 1'b1) : 12'd0;
    wire [11:0] hh_trg = (cand_x1 > cand_x0) ? (cand_x1 - cand_x0 - 1'b1) : 12'd0;
    wire [15:0] hh_min = (hh_cnt0 < hh_cnt1) ? hh_cnt0 : hh_cnt1;
    // Relaxed coverage for real-camera edges: 6/16 (~38%) on each vertical
    // side and 2/16 (~13%) on both horizontal sides.  This still
    // requires all four sides to be present, but tolerates one short break.
    wire [15:0] vl_need = {5'b0, vh_trg[11:1]} - {7'b0, vh_trg[11:3]};
    wire [15:0] vr_need = {5'b0, vh_trg[11:1]} - {7'b0, vh_trg[11:3]};
    wire [15:0] hh_need = {7'b0, hh_trg[11:3]};
    wire        vert_ok = (vl_cnt >= vl_need) && (vr_cnt >= vr_need);
    wire        horz_ok = (hh_min >= hh_need);
    reg         vert_ok_r, horz_ok_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vert_ok_r <= 1'b0; horz_ok_r <= 1'b0;
        end else begin
            vert_ok_r <= vert_ok; horz_ok_r <= horz_ok;
        end
    end

    //-------------------------------------------------------------------------
    // 3) 圆形: 半径扫描 Hough
    //-------------------------------------------------------------------------
    //   v6.2: 原实现用单块 vf_mem(1024x16) 同时做 2 个写地址(vf_wp / vf_wp+1),
    //         Efinity 无法推断成块 RAM, 退化为 logic memory = 16384 FF + 31726 LUT4,
    //         把工程资源顶爆. 现拆成两块独立 FIFO(每块 1 写口 + 1 读口) -> 各 1 块 RAM10;
    //         消费端(累加器写回)同步提速到 1 票/拍, 突发也不会溢出.
    localparam integer FF_AW = 10;                 // 每个子 FIFO 深度 1024
    localparam integer FF_D  = 1 << FF_AW;

    reg  [15:0]      vf_a [0:FF_D-1];
    reg  [15:0]      vf_b [0:FF_D-1];
    reg  [FF_AW-1:0] wa_a, ra_a, wa_b, ra_b;
    reg  [15:0]      q_a, q_b;
    reg              rr;               // 双 FIFO 轮转仲裁
    reg              sel_d;            // 0 -> 取 q_a, 1 -> 取 q_b (已与 q 对齐)
    reg              pop_d;            // 弹票有效标志(与 vote_d 同时有效)
    reg              fifo_drop;

    // 椭圆参数角查表(8.8 定点)。
    //
    // 原来的投票沿"边缘法向"偏移 (rx*cos(phi), ry*sin(phi))，phi 是梯度法向。
    // 对椭圆来说圆心并不在法向上: 设 x=rx*cos(t), y=ry*sin(t)，法向角满足
    //     tan(phi) = (rx/ry) * tan(t)   =>   tan(t) = tan(phi) / (rx/ry)
    // 直接用法向角投票会把一个宽椭圆拆成左右两个"端部小圆"峰值(宽椭圆端点
    // 曲率半径 = ry^2/rx，恰好落进扫描半径范围)，导致锁到错误的中心。
    // 这里按 {frame_asp, v_band} 预先算出 (cos t, sin t)；aspect=1(圆)时与
    // 旧表完全一致，因此圆检测行为不变。
    reg  signed [9:0] par_x, par_y;
    reg  [2:0]        v_band;
    reg               v_quad;
    reg  [11:0]       v_col, v_rowv;
    reg               v_valid;

    always @(*) begin
        case (frame_asp)
            3'd0: case (v_band)   // 高 1.3125 : 1
                3'd0: begin par_x = 10'sd256; par_y = 10'sd0;   end
                3'd1: begin par_x = 10'sd249; par_y = 10'sd60;  end
                3'd2: begin par_x = 10'sd225; par_y = 10'sd122; end
                3'd3: begin par_x = 10'sd194; par_y = 10'sd167; end
                3'd4: begin par_x = 10'sd155; par_y = 10'sd204; end
                3'd5: begin par_x = 10'sd115; par_y = 10'sd229; end
                3'd6: begin par_x = 10'sd77;  par_y = 10'sd244; end
                default: begin par_x = 10'sd35; par_y = 10'sd254; end
            endcase
            3'd1: case (v_band)   // 高 1.5625 : 1
                3'd0: begin par_x = 10'sd256; par_y = 10'sd0;   end
                3'd1: begin par_x = 10'sd246; par_y = 10'sd70;  end
                3'd2: begin par_x = 10'sd215; par_y = 10'sd139; end
                3'd3: begin par_x = 10'sd178; par_y = 10'sd184; end
                3'd4: begin par_x = 10'sd138; par_y = 10'sd216; end
                3'd5: begin par_x = 10'sd99;  par_y = 10'sd236; end
                3'd6: begin par_x = 10'sd65;  par_y = 10'sd247; end
                default: begin par_x = 10'sd30; par_y = 10'sd254; end
            endcase
            3'd2: case (v_band)   // 宽 0.625 : 1
                3'd0: begin par_x = 10'sd256; par_y = 10'sd0;   end
                3'd1: begin par_x = 10'sd254; par_y = 10'sd29;  end
                3'd2: begin par_x = 10'sd248; par_y = 10'sd64;  end
                3'd3: begin par_x = 10'sd237; par_y = 10'sd97;  end
                3'd4: begin par_x = 10'sd217; par_y = 10'sd136; end
                3'd5: begin par_x = 10'sd186; par_y = 10'sd176; end
                3'd6: begin par_x = 10'sd141; par_y = 10'sd214; end
                default: begin par_x = 10'sd72; par_y = 10'sd246; end
            endcase
            3'd3: case (v_band)   // 宽 0.5 : 1
                3'd0: begin par_x = 10'sd256; par_y = 10'sd0;   end
                3'd1: begin par_x = 10'sd255; par_y = 10'sd23;  end
                3'd2: begin par_x = 10'sd251; par_y = 10'sd52;  end
                3'd3: begin par_x = 10'sd243; par_y = 10'sd80;  end
                3'd4: begin par_x = 10'sd229; par_y = 10'sd114; end
                3'd5: begin par_x = 10'sd204; par_y = 10'sd155; end
                3'd6: begin par_x = 10'sd163; par_y = 10'sd197; end
                default: begin par_x = 10'sd88; par_y = 10'sd240; end
            endcase
            default: case (v_band) // 正圆: 与旧表逐项一致
                3'd0: begin par_x = 10'sd256; par_y = 10'sd0;   end
                3'd1: begin par_x = 10'sd251; par_y = 10'sd46;  end
                3'd2: begin par_x = 10'sd237; par_y = 10'sd98;  end
                3'd3: begin par_x = 10'sd217; par_y = 10'sd143; end
                3'd4: begin par_x = 10'sd181; par_y = 10'sd181; end
                3'd5: begin par_x = 10'sd143; par_y = 10'sd217; end
                3'd6: begin par_x = 10'sd98;  par_y = 10'sd237; end
                default: begin par_x = 10'sd46; par_y = 10'sd251; end
            endcase
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_band <= 3'd0; v_quad <= 1'b0;
            v_col <= 12'd0; v_rowv <= 12'd0; v_valid <= 1'b0;
        end else begin
            v_band  <= d_dir[3:1];
            v_quad  <= d_dir[0];
            v_col   <= d_col;
            v_rowv  <= d_row;
            v_valid <= d_edge && (d_col >= i_draw_x0);
        end
    end

    reg  signed [17:0] vx_prod, vy_prod;
    reg  [11:0] p_col, p_row;
    reg         p_valid;

    wire signed [9:0] par_yq = v_quad ? (~par_y + 10'sd1) : par_y;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vx_prod <= 18'sd0; vy_prod <= 18'sd0;
            p_col <= 12'd0; p_row <= 12'd0; p_valid <= 1'b0;
        end else begin
            vx_prod <= par_x  * $signed({4'b0, cur_r});
            vy_prod <= par_yq * $signed({4'b0, cur_ry});
            p_col   <= v_col;  p_row <= v_rowv;  p_valid <= v_valid;
        end
    end

    // A Verilog part-select is unsigned even when the source vector is
    // signed.  Without the explicit $signed conversion, negative vertical
    // Hough offsets are zero-extended into a huge positive row and all votes
    // from two quadrants miss the accumulator.
    wire signed [13:0] dxv = $signed(vx_prod[17:8]);
    wire signed [13:0] dyv = $signed(vy_prod[17:8]);
    wire signed [14:0] a1x = $signed({3'b0, p_col}) + dxv;
    wire signed [14:0] a1y = $signed({3'b0, p_row}) + dyv;
    wire signed [14:0] a2x = $signed({3'b0, p_col}) - dxv;
    wire signed [14:0] a2y = $signed({3'b0, p_row}) - dyv;

    wire signed [14:0] roi_x = $signed({3'b0, i_draw_x0});
    wire ok1 = p_valid && (a1x >= roi_x) && (a1x < H_ACTIVE) && (a1y >= 0) && (a1y < YLIM);
    wire ok2 = p_valid && (a2x >= roi_x) && (a2x < H_ACTIVE) && (a2y >= 0) && (a2y < YLIM);

    wire [7:0] bx1 = a1x[10:BIN_SHIFT];
    wire [7:0] by1 = a1y[10:BIN_SHIFT];
    wire [7:0] bx2 = a2x[10:BIN_SHIFT];
    wire [7:0] by2 = a2y[10:BIN_SHIFT];

    // 累加器 + 峰值 + 逐帧清空 的寄存器(先声明, 便于下面的连续赋值引用)
    (* syn_ramstyle = "block_ram" *) reg [7:0] acc_mem [0:ACC_SIZE-1];
    reg  [7:0]  acc_rdata;
    reg  [7:0]  acc_mem_q;
    reg  [7:0]  acc_wr_d;
    reg         acc_we_d;
    reg         acc_same_d;
    reg  [15:0] acc_raddr, acc_waddr;
    reg  [1:0]  pv;
    reg  [7:0]  peak_val;
    reg  [15:0] peak_addr;
    reg         clr_done;
    reg  [16:0] clr_ptr;

    wire           a_empty = (wa_a == ra_a);
    wire           b_empty = (wa_b == ra_b);
    wire           pop_en  = clr_done && i_en;
    wire           pop_a   = pop_en && !a_empty && (b_empty |  rr);
    wire           pop_b   = pop_en && !b_empty && (a_empty | ~rr);
    wire           pop_now = pop_a | pop_b;
    wire [15:0]    vote_d  = sel_d ? q_b : q_a;

    wire [FF_AW:0] used_a  = {1'b0, wa_a} - {1'b0, ra_a};
    wire [FF_AW:0] used_b  = {1'b0, wa_b} - {1'b0, ra_b};
    wire           room_a  = (used_a < (FF_D - 1));
    wire           room_b  = (used_b < (FF_D - 1));
    wire           hough_poly_mask;

    // Edges already explained by a locked rectangle/triangle must not vote for
    // an ellipse.  Otherwise a long polygon can own the global Hough peak even
    // though the arc verifier would correctly reject its candidate, starving
    // a simultaneous ellipse elsewhere in the same scene.
    wire push1 = i_en && clr_done && ok1 &&  room_a && !hough_poly_mask;
    wire push2 = i_en && clr_done && ok2 &&  room_b && !hough_poly_mask;
    wire drop1 = i_en && clr_done && ok1 && !room_a;
    wire drop2 = i_en && clr_done && ok2 && !room_b;

    // 每块 FIFO: 1 个写口 + 1 个读口 -> simple dual port RAM(EBR/RAM10)
    always @(posedge clk) if (push1) vf_a[wa_a] <= {by1, bx1};
    always @(posedge clk) if (push2) vf_b[wa_b] <= {by2, bx2};
    always @(posedge clk) q_a <= vf_a[ra_a];
    always @(posedge clk) q_b <= vf_b[ra_b];

    //-------------------------------------------------------------------------
    // 累加器(1/8 分辨率投票计数) + 流水写回 + 峰值跟踪 + 逐帧清空
    //   v6.2: 每拍处理 1 票(原实现 4 拍/票), 同址冲突由 same_d 补偿
    //     拍0  FIFO 弹出   : ra+1, sel_d/pop_d 打拍
    //     拍1  出票         : vote_d = q_a/q_b, 与 pop_d 严格同拍
    //     拍2  发读地址     : acc_raddr <= vote_d
    //     拍3  读数据回     : acc_rdata = mem[acc_raddr], acc_waddr <= acc_raddr
    //                          若本拍读写同址, 直接把正在写回的 +1 数据旁路给读口
    //     拍4  写回提交     : acc_wdata = acc_rdata + 1, 由 pv[1] 选通
    //   读地址(acc_raddr)与写地址(acc_waddr)是两组独立寄存器, 因此读口/写口
    //   同拍各自工作, 不需要 vgap 空拍.
    //-------------------------------------------------------------------------
    wire        clr_sweep = !clr_done && !i_de;
    wire [7:0]  acc_wdata = acc_rdata + 8'd1;
    wire        ram_we    = clr_sweep ? 1'b1 : pv[1];
    wire [15:0] ram_wa    = clr_sweep ? clr_ptr[15:0] : acc_waddr;
    wire [7:0]  ram_wd    = clr_sweep ? 8'd0 : acc_wdata;

    always @(posedge clk) begin
        if (ram_we) acc_mem[ram_wa] <= ram_wd;
    end

    always @(posedge clk) begin
        acc_mem_q <= acc_mem[acc_raddr];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_wr_d   <= 8'd0;
            acc_we_d   <= 1'b0;
            acc_same_d <= 1'b0;
        end else begin
            acc_wr_d   <= ram_wd;
            acc_we_d   <= ram_we && !clr_sweep;
            acc_same_d <= (acc_raddr == ram_wa);
        end
    end

    always @(*) begin
        if (acc_we_d && acc_same_d)
            acc_rdata = acc_wr_d;
        else
            acc_rdata = acc_mem_q;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wa_a <= {FF_AW{1'b0}}; ra_a <= {FF_AW{1'b0}};
            wa_b <= {FF_AW{1'b0}}; ra_b <= {FF_AW{1'b0}};
            rr <= 1'b0; sel_d <= 1'b0; pop_d <= 1'b0; fifo_drop <= 1'b0;
            pv <= 2'd0;
            acc_raddr <= 16'd0; acc_waddr <= 16'd0;
            peak_val <= 8'd0; peak_addr <= 16'd0;
            clr_done <= 1'b0; clr_ptr <= 17'd0;
        end else if (rst_frame) begin
            wa_a <= {FF_AW{1'b0}}; ra_a <= {FF_AW{1'b0}};
            wa_b <= {FF_AW{1'b0}}; ra_b <= {FF_AW{1'b0}};
            rr <= 1'b0; sel_d <= 1'b0; pop_d <= 1'b0; fifo_drop <= 1'b0;
            pv <= 2'd0;
            peak_val <= 8'd0; peak_addr <= 16'd0;
            clr_done <= 1'b0; clr_ptr <= 17'd0;
        end else if (clr_sweep) begin
            // 行/场消隐期把累加器逐格清 0, 清完(clr_done=1)才接受投票
            pv <= 2'd0; sel_d <= 1'b0; pop_d <= 1'b0; rr <= 1'b0;
            wa_a <= {FF_AW{1'b0}}; ra_a <= {FF_AW{1'b0}};
            wa_b <= {FF_AW{1'b0}}; ra_b <= {FF_AW{1'b0}};
            if (clr_ptr == ACC_SIZE-1) clr_done <= 1'b1;
            else                       clr_ptr <= clr_ptr + 1'b1;
        end else begin
            // ---- FIFO 指针 + 轮转仲裁 ----
            if (pop_now) rr <= ~rr;
            sel_d <= ~pop_a;                  // 下一拍取哪块 FIFO 的输出
            pop_d <= pop_now;
            if (pop_a) ra_a <= ra_a + 1'b1;
            if (pop_b) ra_b <= ra_b + 1'b1;
            if (push1) wa_a <= wa_a + 1'b1;
            if (push2) wa_b <= wa_b + 1'b1;
            if (drop1 || drop2) fifo_drop <= 1'b1;

            // ---- 累加器写回流水 ----
            pv        <= {pv[0], pop_d};
            acc_raddr <= vote_d;
            acc_waddr <= acc_raddr;
            if (pv[1] && (acc_wdata > peak_val)) begin
                peak_val  <= acc_wdata;
                peak_addr <= acc_waddr;
            end
        end
    end

    // 帧末判决: 圆 + 矩形
    reg  [11:0] circ_cx, circ_cy;
    reg  [11:0] circ_rr;
    reg  [11:0] circ_ry;
    reg         circ_valid;
    reg  [7:0]  circ_hold;

    // Circle candidate verification.  Hough proposes a centre/radius from the
    // current frame; the next frame checks that the four corners of its
    // bounding ellipse are empty.  Real ellipses have curved sides and leave
    // those corners blank, while a rectangle fills all four and a triangle
    // fills the two bottom corners.  This rejects the common false peaks from
    // dense textures without another large frame buffer.
    reg  [11:0] circ_cand_cx, circ_cand_cy;
    reg  [11:0] circ_cand_rx, circ_cand_ry;
    reg         circ_cand_active;
    reg  [15:0] circ_corner_hits;
    reg  [7:0]  circ_arc_mask;
    reg  [15:0] circ_diag_cnt;

    // The Hough peak is quantized to an 8-pixel bin and can also be biased by
    // perspective.  While the candidate is verified on the next frame, measure
    // the real edge extents around it.  The accepted box then follows the
    // target instead of the coarse bin/aspect hypothesis.
    localparam [11:0] CIRC_FIT_MARGIN = 12'd44;
    localparam [12:0] CIRC_FIT_ARC_TOL = 13'd44;
    localparam [12:0] CIRC_FIT_GUARD = 13'd4;
    reg  [11:0] circ_meas_x0, circ_meas_x1;
    reg  [11:0] circ_meas_y0, circ_meas_y1;
    reg         circ_meas_valid;

    wire [11:0] peak_cx = {peak_addr[7:0], {BIN_SHIFT{1'b0}}} + 12'd4;
    wire [11:0] peak_cy = {1'b0, peak_addr[15:8], {BIN_SHIFT{1'b0}}} + 12'd4;
    wire [12:0] cc_l = ({1'b0, circ_cand_cx} > {1'b0, circ_cand_rx}) ?
                       ({1'b0, circ_cand_cx} - {1'b0, circ_cand_rx}) : 13'd0;
    wire [12:0] cc_r = {1'b0, circ_cand_cx} + {1'b0, circ_cand_rx};
    wire [12:0] cc_t = ({1'b0, circ_cand_cy} > {1'b0, circ_cand_ry}) ?
                       ({1'b0, circ_cand_cy} - {1'b0, circ_cand_ry}) : 13'd0;
    wire [12:0] cc_b = {1'b0, circ_cand_cy} + {1'b0, circ_cand_ry};
    wire [11:0] cc_qx = circ_cand_rx >> 2;
    wire [11:0] cc_qy = circ_cand_ry >> 2;
    wire cc_x_corner = (col13 <= (cc_l + {1'b0, cc_qx})) ||
                       (col13 >= (cc_r - {1'b0, cc_qx}));
    wire cc_y_corner = ({1'b0, i_row} <= (cc_t + {1'b0, cc_qy})) ||
                       ({1'b0, i_row} >= (cc_b - {1'b0, cc_qy}));
    wire cc_inside = (col13 >= cc_l) && (col13 <= cc_r) &&
                     ({1'b0, i_row} >= cc_t) && ({1'b0, i_row} <= cc_b);
    // The Hough centre/radius is quantised to an 8-pixel accumulator bin and
    // a 16-pixel radius step.  The true outline can therefore lie just outside
    // the candidate box, especially at the left/right and top/bottom extremes.
    // Allow a small guard band for the arc sampler so a slightly undersized
    // candidate can still see the real extremities.  The corner test stays
    // strictly inside the candidate box.
    localparam [12:0] CIRC_ARC_GUARD = 13'd16;
    wire [12:0] cc_arc_l = (cc_l > CIRC_ARC_GUARD) ? (cc_l - CIRC_ARC_GUARD) : 13'd0;
    wire [12:0] cc_arc_r = cc_r + CIRC_ARC_GUARD;
    wire [12:0] cc_arc_t = (cc_t > CIRC_ARC_GUARD) ? (cc_t - CIRC_ARC_GUARD) : 13'd0;
    wire [12:0] cc_arc_b = cc_b + CIRC_ARC_GUARD;
    wire cc_arc_inside = (col13 >= cc_arc_l) && (col13 <= cc_arc_r) &&
                         ({1'b0, i_row} >= cc_arc_t) && ({1'b0, i_row} <= cc_arc_b);
    wire cc_hit = circ_cand_active && i_en && i_de && i_edge &&
                  (col >= i_draw_x0) && cc_inside && cc_x_corner && cc_y_corner;

    wire [12:0] fit_l = (cc_l > {1'b0, CIRC_FIT_MARGIN}) ?
                        (cc_l - {1'b0, CIRC_FIT_MARGIN}) : 13'd0;
    wire [12:0] fit_r = cc_r + {1'b0, CIRC_FIT_MARGIN};
    wire [12:0] fit_t = (cc_t > {1'b0, CIRC_FIT_MARGIN}) ?
                        (cc_t - {1'b0, CIRC_FIT_MARGIN}) : 13'd0;
    wire [12:0] fit_b = cc_b + {1'b0, CIRC_FIT_MARGIN};
    wire [12:0] meas_cx = ({1'b0, circ_meas_x0} + {1'b0, circ_meas_x1} + 13'd1) >> 1;
    wire [12:0] meas_cy = ({1'b0, circ_meas_y0} + {1'b0, circ_meas_y1} + 13'd1) >> 1;
    wire [12:0] meas_rx = (({1'b0, circ_meas_x1} - {1'b0, circ_meas_x0}) + 13'd1) >> 1;
    wire [12:0] meas_ry = (({1'b0, circ_meas_y1} - {1'b0, circ_meas_y0}) + 13'd1) >> 1;

    // A partially covered ellipse (for example when the candidate centre is
    // biased by one 8-pixel Hough bin) can make one measured pair of extents
    // collapse onto a single arc.  If any extent reaches the outer measurement
    // boundary, the real boundary may lie beyond that search window, so reject
    // that axis and keep the Hough candidate instead.  Complete measurements
    // inside the window are allowed to move away from a biased candidate.
    wire [12:0] meas_x0_13 = {1'b0, circ_meas_x0};
    wire [12:0] meas_x1_13 = {1'b0, circ_meas_x1};
    wire [12:0] meas_y0_13 = {1'b0, circ_meas_y0};
    wire [12:0] meas_y1_13 = {1'b0, circ_meas_y1};
    wire [12:0] fit_l_guard = fit_l + CIRC_FIT_GUARD;
    wire [12:0] fit_r_guard = (fit_r > CIRC_FIT_GUARD) ? (fit_r - CIRC_FIT_GUARD) : 13'd0;
    wire [12:0] fit_t_guard = fit_t + CIRC_FIT_GUARD;
    wire [12:0] fit_b_guard = (fit_b > CIRC_FIT_GUARD) ? (fit_b - CIRC_FIT_GUARD) : 13'd0;
    wire meas_x_ok = circ_meas_valid && (circ_meas_x1 > circ_meas_x0) &&
                     (meas_x0_13 >= fit_l_guard) && (meas_x1_13 <= fit_r_guard);
    wire meas_y_ok = circ_meas_valid && (circ_meas_y1 > circ_meas_y0) &&
                     (meas_y0_13 >= fit_t_guard) && (meas_y1_13 <= fit_b_guard);
    // A candidate biased onto one end-cap of a wide ellipse can still light
    // six arc windows, but its measured box then collapses to that end-cap
    // (for example rx=81 against a candidate rx=136).  A real outline can
    // shrink by at most part of one radius step, so reject a valid measurement
    // that covers less than 75% of the candidate radius.
    wire [12:0] meas_rx_floor = {1'b0, circ_cand_rx} - {3'b0, circ_cand_rx[11:2]};
    wire [12:0] meas_ry_floor = {1'b0, circ_cand_ry} - {3'b0, circ_cand_ry[11:2]};
    wire meas_size_ok = (!meas_x_ok || (meas_rx >= meas_rx_floor)) &&
                        (!meas_y_ok || (meas_ry >= meas_ry_floor));

    // Eight points on the candidate ellipse (cardinal + approximate 45 deg).
    // A real ellipse lights up most of these windows; straight edges from a
    // rectangle or triangle only light a few.  This is a second geometric
    // check independent from the Hough vote and the empty-corner test.
    wire [12:0] cc_row13 = {2'b0, i_row};
    wire [12:0] cc_rx3 = {1'b0, circ_cand_rx} - {1'b0, (circ_cand_rx >> 2)};
    wire [12:0] cc_ry3 = {1'b0, circ_cand_ry} - {1'b0, (circ_cand_ry >> 2)};
    wire [12:0] cc_xr = {1'b0, circ_cand_cx} + {1'b0, circ_cand_rx};
    wire [12:0] cc_xqr = {1'b0, circ_cand_cx} + cc_rx3;
    wire [12:0] cc_xl = ({1'b0, circ_cand_cx} > {1'b0, circ_cand_rx}) ?
                        ({1'b0, circ_cand_cx} - {1'b0, circ_cand_rx}) : 13'd0;
    wire [12:0] cc_xql = ({1'b0, circ_cand_cx} > cc_rx3) ?
                        ({1'b0, circ_cand_cx} - cc_rx3) : 13'd0;
    wire [12:0] cc_yb = {1'b0, circ_cand_cy} + {1'b0, circ_cand_ry};
    wire [12:0] cc_yqb = {1'b0, circ_cand_cy} + cc_ry3;
    wire [12:0] cc_yt = ({1'b0, circ_cand_cy} > {1'b0, circ_cand_ry}) ?
                        ({1'b0, circ_cand_cy} - {1'b0, circ_cand_ry}) : 13'd0;
    wire [12:0] cc_yqt = ({1'b0, circ_cand_cy} > cc_ry3) ?
                        ({1'b0, circ_cand_cy} - cc_ry3) : 13'd0;
    wire [12:0] cc_x0 = {1'b0, circ_cand_cx};
    wire [12:0] cc_dxr = (col13 > cc_xr) ? (col13 - cc_xr) : (cc_xr - col13);
    wire [12:0] cc_dxqr = (col13 > cc_xqr) ? (col13 - cc_xqr) : (cc_xqr - col13);
    wire [12:0] cc_dxl = (col13 > cc_xl) ? (col13 - cc_xl) : (cc_xl - col13);
    wire [12:0] cc_dxql = (col13 > cc_xql) ? (col13 - cc_xql) : (cc_xql - col13);
    wire [12:0] cc_dx0 = (col13 > cc_x0) ? (col13 - cc_x0) : (cc_x0 - col13);
    wire [12:0] cc_dyb = (cc_row13 > cc_yb) ? (cc_row13 - cc_yb) : (cc_yb - cc_row13);
    wire [12:0] cc_dyqb = (cc_row13 > cc_yqb) ? (cc_row13 - cc_yqb) : (cc_yqb - cc_row13);
    wire [12:0] cc_dyt = (cc_row13 > cc_yt) ? (cc_row13 - cc_yt) : (cc_yt - cc_row13);
    wire [12:0] cc_dyqt = (cc_row13 > cc_yqt) ? (cc_row13 - cc_yqt) : (cc_yqt - cc_row13);
    wire [12:0] cc_dy0 = (cc_row13 > {1'b0, circ_cand_cy}) ?
                         (cc_row13 - {1'b0, circ_cand_cy}) : ({1'b0, circ_cand_cy} - cc_row13);
    wire [7:0] cc_arc_now = {
        (cc_dxr  <= CATOL_C) && (cc_dy0 <= CATOL_C),
        (cc_dxqr <= CATOL_C) && (cc_dyqt <= CATOL_C),
        (cc_dx0  <= CATOL_C) && (cc_dyt <= CATOL_C),
        (cc_dxql <= CATOL_C) && (cc_dyqt <= CATOL_C),
        (cc_dxl  <= CATOL_C) && (cc_dy0 <= CATOL_C),
        (cc_dxql <= CATOL_C) && (cc_dyqb <= CATOL_C),
        (cc_dx0  <= CATOL_C) && (cc_dyb <= CATOL_C),
        (cc_dxqr <= CATOL_C) && (cc_dyqb <= CATOL_C)
    };

    function [3:0] circ_pop8;
        input [7:0] v;
        integer j;
        begin
            circ_pop8 = 4'd0;
            for (j = 0; j < 8; j = j + 1)
                circ_pop8 = circ_pop8 + {3'b0, v[j]};
        end
    endfunction
    wire [3:0] circ_arc_hits = circ_pop8(circ_arc_mask);
    // Count diagonal-direction edge pixels in the four 45-degree arc windows.
    // A coarse radius can make a grid intersection light every positional arc
    // window, but a grid only contributes a handful of diagonal Sobel pixels
    // at its corners.  A real ellipse contributes a long diagonal run in each
    // window.  This count separates the two without tightening the positional
    // tolerance needed for coarse Hough radii.
    wire [7:0] cc_arc_diag_now = cc_arc_now & 8'b0101_0101;
    wire       cc_band_diag = (i_dir16[3:1] >= 3'd2) &&
                              (i_dir16[3:1] <= 3'd5);
    wire cc_arc_in = circ_cand_active && i_en && i_de && i_edge &&
                     (col >= i_draw_x0) && cc_arc_inside && (|cc_arc_now);

    // Measure from four wide cardinal windows around the candidate extremes.
    // The narrow eight-point windows above remain the strict acceptance test;
    // these wider windows only refine an already accepted candidate.  They can
    // recover a side missed by one coarse Hough bin, while the fit box and the
    // saturation guard stop distant checker/table edges from stretching the
    // displayed rectangle.
    wire [12:0] fit_dx_center = (col13 > {1'b0, circ_cand_cx}) ?
                                (col13 - {1'b0, circ_cand_cx}) :
                                ({1'b0, circ_cand_cx} - col13);
    wire [12:0] fit_dy_center = (cc_row13 > {1'b0, circ_cand_cy}) ?
                                (cc_row13 - {1'b0, circ_cand_cy}) :
                                ({1'b0, circ_cand_cy} - cc_row13);
    wire [12:0] fit_dx_right = (col13 > cc_r) ? (col13 - cc_r) : (cc_r - col13);
    wire [12:0] fit_dx_left = (col13 > cc_l) ? (col13 - cc_l) : (cc_l - col13);
    wire [12:0] fit_dy_bottom = (cc_row13 > cc_b) ? (cc_row13 - cc_b) : (cc_b - cc_row13);
    wire [12:0] fit_dy_top = (cc_row13 > cc_t) ? (cc_row13 - cc_t) : (cc_t - cc_row13);
    wire fit_right = (fit_dx_right <= CIRC_FIT_ARC_TOL) &&
                     (fit_dy_center <= CIRC_FIT_ARC_TOL);
    wire fit_left = (fit_dx_left <= CIRC_FIT_ARC_TOL) &&
                    (fit_dy_center <= CIRC_FIT_ARC_TOL);
    wire fit_bottom = (fit_dx_center <= CIRC_FIT_ARC_TOL) &&
                      (fit_dy_bottom <= CIRC_FIT_ARC_TOL);
    wire fit_top = (fit_dx_center <= CIRC_FIT_ARC_TOL) &&
                   (fit_dy_top <= CIRC_FIT_ARC_TOL);
    wire cc_fit_hit = circ_cand_active && i_en && i_de && i_edge &&
                      (col >= i_draw_x0) &&
                      (fit_right || fit_left || fit_bottom || fit_top) &&
                      (col13 >= fit_l) && (col13 <= fit_r) &&
                      ({1'b0, i_row} >= fit_t) && ({1'b0, i_row} <= fit_b);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            circ_meas_valid <= 1'b0;
            circ_meas_x0 <= 12'hFFF; circ_meas_x1 <= 12'd0;
            circ_meas_y0 <= 12'hFFF; circ_meas_y1 <= 12'd0;
        end else if (rst_frame) begin
            circ_meas_valid <= 1'b0;
            circ_meas_x0 <= 12'hFFF; circ_meas_x1 <= 12'd0;
            circ_meas_y0 <= 12'hFFF; circ_meas_y1 <= 12'd0;
        end else if (cc_fit_hit) begin
            circ_meas_valid <= 1'b1;
            if (col < circ_meas_x0) circ_meas_x0 <= col;
            if (col > circ_meas_x1) circ_meas_x1 <= col;
            if (i_row < circ_meas_y0) circ_meas_y0 <= {1'b0, i_row};
            if (i_row > circ_meas_y1) circ_meas_y1 <= {1'b0, i_row};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            circ_corner_hits <= 16'd0;
        else if (rst_frame)
            circ_corner_hits <= 16'd0;
        else if (cc_hit && (circ_corner_hits != 16'hFFFF))
            circ_corner_hits <= circ_corner_hits + 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            circ_arc_mask <= 8'd0;
            circ_diag_cnt <= 16'd0;
        end else if (rst_frame) begin
            circ_arc_mask <= 8'd0;
            circ_diag_cnt <= 16'd0;
        end else begin
            if (cc_arc_in)
            circ_arc_mask <= circ_arc_mask | cc_arc_now;
            if (cc_arc_in && (|cc_arc_diag_now) && cc_band_diag &&
                (circ_diag_cnt != 16'hFFFF))
                circ_diag_cnt <= circ_diag_cnt + 1'b1;
        end
    end

    wire new_cand = i_en && pair_v && (ov_x1 > ov_x0) && (ov_y1 > ov_y0) &&
                    ((ov_x1 - ov_x0) >= RSID_C) && ((ov_y1 - ov_y0) >= RGAP_C);
    // Require at least 4/64 of the frame edge votes, but cap the requirement
    // at 64.  peak_val is only 8 bits wide; without the cap a busy 1920-wide
    // scene can demand more than 255 votes and make circular detection
    // mathematically impossible.  The corner + eight-arc tests remain strict.
    wire [15:0] circ_ratio_raw  = (frame_edge_count >> CRSH_C);
    // In a busy frame the old 64-vote cap was stricter than the configured
    // Hough threshold and could reject a valid wide ellipse forever.  Keep
    // the density ratio for sparse frames, but never require more than the
    // explicit Hough threshold once the frame is large enough to saturate it.
    wire [15:0] circ_ratio_need = (circ_ratio_raw > {8'd0, RTH_C}) ?
                                  {8'd0, RTH_C} : circ_ratio_raw;
    wire        circ_evidence_ok = (frame_edge_count >= CMIN_C) &&
                                   ({8'd0, peak_val} >= circ_ratio_need);

    // Do not accept a circle whose centre sits on top of a rectangle or
    // triangle candidate.  Long straight sides can form a narrow Hough peak
    // at small radii; these geometric exclusions remove that cross-class hit.
    // Current-frame triangle hypothesis used to exclude a circle from a
    // triangle candidate.  The scan ratio rotates over three heights.
    reg [1:0] tri_ratio_scan;
    wire [11:0] tri_height_scan = (tri_ratio_scan == 2'd0) ?
                                  (tb_len - (tb_len >> 3)) :
                                  (tri_ratio_scan == 2'd1) ?
                                  (tb_len + (tb_len >> 2)) :
                                  (tb_len + (tb_len >> 1));
    wire        tri_cand_now = i_en && tb_v && (tb_len >= TWMN_C) &&
                               (tb_y >= (tri_height_scan + 12'd12));
    wire [12:0] circ_cx13 = {1'b0, circ_cand_cx};
    wire [12:0] circ_cy13 = {1'b0, circ_cand_cy};
    wire [12:0] circ_margin = {1'b0, (circ_cand_rx >> 1)};
    wire [12:0] tn_x0 = {1'b0, tb_x0};
    wire [12:0] tn_x1 = {1'b0, tb_x1};
    wire [12:0] tn_y0 = {1'b0, tb_y} - {1'b0, tri_height_scan};
    wire [12:0] tn_y1 = {1'b0, tb_y};
    wire [12:0] tc_x0 = {1'b0, tri_cand_x0};
    wire [12:0] tc_x1 = {1'b0, tri_cand_x1};
    wire [12:0] tc_y0 = {1'b0, tri_cand_y0};
    wire [12:0] tc_y1 = {1'b0, tri_cand_y1};
    wire circ_in_tri_now = tri_cand_now &&
        ((circ_cx13 + circ_margin) >= tn_x0) &&
        (circ_cx13 <= (tn_x1 + circ_margin)) &&
        ((circ_cy13 + circ_margin) >= tn_y0) &&
        (circ_cy13 <= (tn_y1 + circ_margin));
    wire circ_in_tri_cur = tri_cand_active &&
        ((circ_cx13 + circ_margin) >= tc_x0) &&
        (circ_cx13 <= (tc_x1 + circ_margin)) &&
        ((circ_cy13 + circ_margin) >= tc_y0) &&
        (circ_cy13 <= (tc_y1 + circ_margin));

    // Rotated-triangle state is also used to veto a circular lock on a
    // polygon.  Declare it before the circle decision so the geometric
    // exclusion is evaluated in the same frame as the other candidates.
    reg  [11:0] tri_rot_cand_x, tri_rot_cand_y0, tri_rot_cand_y1;
    reg  [11:0] tri_rot_cand_depth;
    reg  [1:0]  tri_rot_cand_ratio;
    reg         tri_rot_cand_active;
    reg  [11:0] tri_rot_l_hits, tri_rot_r_hits;
    reg  [15:0] tri_rot_ir_hits, tri_rot_il_hits;
    reg         tri_rot_row_base, tri_rot_row_l, tri_rot_row_r;
    reg         tri_rot_row_ir, tri_rot_row_il;
    reg  [11:0] tri_rot_x0, tri_rot_y0, tri_rot_x1, tri_rot_y1;
    reg         tri_rot_valid;
    reg  [7:0]  tri_rot_hold;
    reg  [1:0]  tri_rot_ratio_scan;

    wire [12:0] rt_cand_x13 = {1'b0, tri_rot_cand_x};
    wire [12:0] rt_cand_y0_13 = {1'b0, tri_rot_cand_y0};
    wire [12:0] rt_cand_y1_13 = {1'b0, tri_rot_cand_y1};
    wire [12:0] rt_cand_depth13 = {1'b0, tri_rot_cand_depth};
    wire [12:0] rt_cand_x0 = (rt_cand_x13 > rt_cand_depth13) ?
                              (rt_cand_x13 - rt_cand_depth13) : 13'd0;
    wire [12:0] rt_cand_x1 = rt_cand_x13 + rt_cand_depth13;
    wire circ_in_tri_rot_cand = tri_rot_cand_active &&
        ((circ_cx13 + circ_margin) >= rt_cand_x0) &&
        (circ_cx13 <= (rt_cand_x1 + circ_margin)) &&
        ((circ_cy13 + circ_margin) >= rt_cand_y0_13) &&
        (circ_cy13 <= (rt_cand_y1_13 + circ_margin));
    wire [12:0] rt_val_x0 = {1'b0, tri_rot_x0};
    wire [12:0] rt_val_x1 = {1'b0, tri_rot_x1};
    wire [12:0] rt_val_y0 = {1'b0, tri_rot_y0};
    wire [12:0] rt_val_y1 = {1'b0, tri_rot_y1};
    wire circ_in_tri_rot_valid = tri_rot_valid &&
        ((circ_cx13 + circ_margin) >= rt_val_x0) &&
        (circ_cx13 <= (rt_val_x1 + circ_margin)) &&
        ((circ_cy13 + circ_margin) >= rt_val_y0) &&
        (circ_cy13 <= (rt_val_y1 + circ_margin));
    wire circ_tri_exclude = circ_cand_active &&
                            (circ_in_tri_now || circ_in_tri_cur ||
                             circ_in_tri_rot_cand || circ_in_tri_rot_valid);

    // Rectangle/circle arbitration.  A filled rectangle can create an
    // inscribed Hough peak and hit two of the four corner windows.  Veto the
    // circle only when a rectangle candidate tightly matches the SAME box as
    // the circle hypothesis (within one Hough-bin).  A circle sitting inside a
    // much larger background cell is not vetoed.
    localparam [12:0] CRECT_MATCH_TOL = 13'd8;
    wire [12:0] circ_cand_box_l = (circ_cx13 > circ_margin) ?
                                  (circ_cx13 - circ_margin) : 13'd0;
    wire [12:0] circ_cand_box_r = circ_cx13 + circ_margin;
    wire [12:0] circ_cand_box_t = (circ_cy13 > circ_margin) ?
                                  (circ_cy13 - circ_margin) : 13'd0;
    wire [12:0] circ_cand_box_b = circ_cy13 + circ_margin;
    wire [12:0] circ_meas_box_l = (meas_cx > meas_rx) ?
                                  (meas_cx - meas_rx) : 13'd0;
    wire [12:0] circ_meas_box_r = meas_cx + meas_rx;
    wire [12:0] circ_meas_box_t = (meas_cy > meas_ry) ?
                                  (meas_cy - meas_ry) : 13'd0;
    wire [12:0] circ_meas_box_b = meas_cy + meas_ry;
    wire [12:0] circ_box_l = meas_x_ok ? circ_meas_box_l : circ_cand_box_l;
    wire [12:0] circ_box_r = meas_x_ok ? circ_meas_box_r : circ_cand_box_r;
    wire [12:0] circ_box_t = meas_y_ok ? circ_meas_box_t : circ_cand_box_t;
    wire [12:0] circ_box_b = meas_y_ok ? circ_meas_box_b : circ_cand_box_b;
    wire [12:0] circ_box_l_g = (circ_box_l > CRECT_MATCH_TOL) ?
                               (circ_box_l - CRECT_MATCH_TOL) : 13'd0;
    wire [12:0] circ_box_r_g = circ_box_r + CRECT_MATCH_TOL;
    wire [12:0] circ_box_t_g = (circ_box_t > CRECT_MATCH_TOL) ?
                               (circ_box_t - CRECT_MATCH_TOL) : 13'd0;
    wire [12:0] circ_box_b_g = circ_box_b + CRECT_MATCH_TOL;
    wire [12:0] rect_new_x0 = {1'b0, ov_x0};
    wire [12:0] rect_new_x1 = {1'b0, ov_x1};
    wire [12:0] rect_new_y0 = {1'b0, ov_y0};
    wire [12:0] rect_new_y1 = {1'b0, ov_y1};
    wire [12:0] rect_cur_x0 = {1'b0, cand_x0};
    wire [12:0] rect_cur_x1 = {1'b0, cand_x1};
    wire [12:0] rect_cur_y0 = {1'b0, cand_y0};
    wire [12:0] rect_cur_y1 = {1'b0, cand_y1};
    wire [12:0] rect_val_x0 = {1'b0, rect_x0};
    wire [12:0] rect_val_x1 = {1'b0, rect_x1};
    wire [12:0] rect_val_y0 = {1'b0, rect_y0};
    wire [12:0] rect_val_y1 = {1'b0, rect_y1};
    wire circ_rect_match_new = new_cand &&
        (rect_new_x0 >= circ_box_l_g) && (rect_new_x1 <= circ_box_r_g) &&
        (rect_new_y0 >= circ_box_t_g) && (rect_new_y1 <= circ_box_b_g);
    wire circ_rect_match_cur = cand_ready &&
        (rect_cur_x0 >= circ_box_l_g) && (rect_cur_x1 <= circ_box_r_g) &&
        (rect_cur_y0 >= circ_box_t_g) && (rect_cur_y1 <= circ_box_b_g);
    wire circ_rect_match_val = rect_valid &&
        (rect_val_x0 >= circ_box_l_g) && (rect_val_x1 <= circ_box_r_g) &&
        (rect_val_y0 >= circ_box_t_g) && (rect_val_y1 <= circ_box_b_g);
    wire circ_rect_exclude = circ_cand_active &&
                             (circ_rect_match_new || circ_rect_match_cur ||
                              circ_rect_match_val);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            circ_valid <= 1'b0; circ_hold <= 8'd0;
            circ_cx <= 12'd0;   circ_cy <= 12'd0;
            circ_rr <= R0;
            circ_ry <= R0 + (R0 >> 2) + (R0 >> 4);
            circ_cand_cx <= 12'd0; circ_cand_cy <= 12'd0;
            circ_cand_rx <= R0;    circ_cand_ry <= R0 + (R0 >> 2) + (R0 >> 4);
            circ_cand_active <= 1'b0;
            frame_k <= 4'd0;
            frame_asp <= 3'd0;
            rot <= 2'd0;
            cand_ready <= 1'b0; rect_valid <= 1'b0; rect_hold <= 8'd0;
            cand_x0 <= 12'd0; cand_y0 <= 12'd0; cand_x1 <= 12'd0; cand_y1 <= 12'd0;
            rect_x0 <= 12'd0; rect_y0 <= 12'd0; rect_x1 <= 12'd0; rect_y1 <= 12'd0;
        end else if (rst_frame) begin
            if (frame_k == (RNUM_C - 1'b1)) begin
                frame_k   <= 4'd0;
            frame_asp <= (frame_asp == 3'd4) ? 3'd0 : (frame_asp + 1'b1);
            end else begin
                frame_k <= frame_k + 1'b1;
            end
            rot <= rot_n;
            // --- 圆: validate the candidate accumulated in the previous frame ---
            if (i_en && circ_cand_active && (frame_edge_count >= CMIN_C) &&
                (circ_corner_hits <= CMAX_C) && (circ_arc_hits >= CAMIN_C) &&
                (circ_diag_cnt >= CDGM_C) &&
                meas_size_ok &&
                !circ_tri_exclude && !circ_rect_exclude) begin
                circ_valid <= 1'b1;
                circ_hold  <= CHLD_C;
                if (meas_x_ok) begin
                    circ_cx <= meas_cx[11:0];
                    circ_rr <= meas_rx[11:0];
                end else begin
                    circ_cx <= circ_cand_cx;
                    circ_rr <= circ_cand_rx;
                end
                if (meas_y_ok) begin
                    circ_cy <= meas_cy[11:0];
                    circ_ry <= meas_ry[11:0];
                end else begin
                    circ_cy <= circ_cand_cy;
                    circ_ry <= circ_cand_ry;
                end
            end else if (circ_hold != 8'd0) begin
                circ_hold <= circ_hold - 1'b1;
                if (circ_hold == 8'd1) circ_valid <= 1'b0;
            end
            // Latch the next Hough candidate.  It is only accepted after the
            // following frame's corner test.
            if (i_en && (peak_val >= RTH_C) && circ_evidence_ok) begin
                circ_cand_active <= 1'b1;
                circ_cand_cx <= peak_cx;
                circ_cand_cy <= peak_cy;
                circ_cand_rx <= cur_r;
                circ_cand_ry <= cur_ry;
            end else begin
                circ_cand_active <= 1'b0;
            end
            // --- 矩形: 用上一帧候选做覆盖率判决 ---
            if (cand_ready) begin
                if (i_en && vert_ok_r && horz_ok_r) begin
                    rect_valid <= 1'b1;
                    rect_hold  <= RHLD_C;
                    // Report the measured side extremes instead of the
                    // shortened straight-run ends, so rounded-corner boxes
                    // still get a box that covers the whole object.
                    rect_x0 <= (vl_xmin != 12'hFFF) ? vl_xmin : cand_x0;
                    rect_y0 <= cand_y0;
                    rect_x1 <= (vr_xmax != 12'd0) ? vr_xmax : cand_x1;
                    rect_y1 <= cand_y1;
                end else if (rect_hold != 8'd0) begin
                    // 轮换到不匹配的组合时不要立刻丢框, 由 hold 维持
                    rect_hold <= rect_hold - 1'b1;
                    if (rect_hold == 8'd1) rect_valid <= 1'b0;
                end
                cand_ready <= 1'b0;
            end else if (rect_hold != 8'd0) begin
                rect_hold <= rect_hold - 1'b1;
                if (rect_hold == 8'd1) rect_valid <= 1'b0;
            end
            // --- 产生新候选 ---
            if (new_cand) begin
                cand_ready <= 1'b1;
                cand_x0 <= ov_x0; cand_y0 <= ov_y0;
                cand_x1 <= ov_x1; cand_y1 <= ov_y1;
            end
        end
    end

    //-------------------------------------------------------------------------
    // 3b) 三角形: 轮换水平底边 + 两侧斜率扫描
    //     目标板上的三角形是正立、底边水平. 每帧从 top-3 中轮换取一条水平
    //     行程作为底边, 并轮换 7/8、5/4、3/2 三种高宽比; 下一帧沿两条从
    //     底角收敛到中点的斜线统计命中行. 候选保存自己的 y0/y1/slope 档位,
    //     不拿当前行重新假设高度. 不使用除法器/连通域, 资源很小.
    //-------------------------------------------------------------------------
    reg  [11:0] tri_cand_x0, tri_cand_y0, tri_cand_x1, tri_cand_y1;
    reg  [1:0]  tri_cand_ratio;
    reg         tri_cand_active;
    reg  [11:0] tri_l_hits, tri_r_hits;
    reg  [15:0] tri_inner_hits;
    reg         tri_row_l, tri_row_r;
    reg  [11:0] tri_x0, tri_y0, tri_x1, tri_y1;
    reg         tri_valid;

    // Polygon-owned Hough-vote exclusion.  p_col/p_row are the edge-pixel
    // coordinates in the vote pipeline; a one-pixel pad covers the second
    // stroke of a thick printed boundary.
    localparam [12:0] HOUGH_POLY_PAD = 13'd2;
    wire [12:0] hough_px13 = {1'b0, p_col};
    wire [12:0] hough_py13 = {1'b0, p_row};
    wire [12:0] hough_rect_l = ({1'b0, rect_x0} > HOUGH_POLY_PAD) ?
                               ({1'b0, rect_x0} - HOUGH_POLY_PAD) : 13'd0;
    wire [12:0] hough_rect_r = {1'b0, rect_x1} + HOUGH_POLY_PAD;
    wire [12:0] hough_rect_t = ({1'b0, rect_y0} > HOUGH_POLY_PAD) ?
                               ({1'b0, rect_y0} - HOUGH_POLY_PAD) : 13'd0;
    wire [12:0] hough_rect_b = {1'b0, rect_y1} + HOUGH_POLY_PAD;
    wire        hough_tri_valid = tri_valid | tri_rot_valid;
    wire [11:0] hough_tri_x0 = tri_rot_valid ? tri_rot_x0 : tri_x0;
    wire [11:0] hough_tri_y0 = tri_rot_valid ? tri_rot_y0 : tri_y0;
    wire [11:0] hough_tri_x1 = tri_rot_valid ? tri_rot_x1 : tri_x1;
    wire [11:0] hough_tri_y1 = tri_rot_valid ? tri_rot_y1 : tri_y1;
    wire [12:0] hough_tri_l = ({1'b0, hough_tri_x0} > HOUGH_POLY_PAD) ?
                              ({1'b0, hough_tri_x0} - HOUGH_POLY_PAD) : 13'd0;
    wire [12:0] hough_tri_r = {1'b0, hough_tri_x1} + HOUGH_POLY_PAD;
    wire [12:0] hough_tri_t = ({1'b0, hough_tri_y0} > HOUGH_POLY_PAD) ?
                              ({1'b0, hough_tri_y0} - HOUGH_POLY_PAD) : 13'd0;
    wire [12:0] hough_tri_b = {1'b0, hough_tri_y1} + HOUGH_POLY_PAD;
    wire hough_in_rect = rect_valid &&
                         (hough_px13 >= hough_rect_l) && (hough_px13 <= hough_rect_r) &&
                         (hough_py13 >= hough_rect_t) && (hough_py13 <= hough_rect_b);
    wire hough_in_tri  = hough_tri_valid &&
                         (hough_px13 >= hough_tri_l) && (hough_px13 <= hough_tri_r) &&
                         (hough_py13 >= hough_tri_t) && (hough_py13 <= hough_tri_b);
    assign hough_poly_mask = hough_in_rect | hough_in_tri;
    reg  [7:0]  tri_hold;

    wire [11:0] tri_h_pix = (tri_cand_y1 >= i_row) ? (tri_cand_y1 - i_row) : 12'd0;
    // offset = height_from_base / (2 * height_ratio)
    //   7/8 base: 4/7  ~= 73/128
    //   5/4 base: 2/5  ~= 51/128
    //   3/2 base: 1/3  ~= 43/128
    // Constant shift-adds avoid a divider and keep the geometry tied to the
    // candidate that was actually accepted on the previous frame.
    wire [17:0] tri_h_m73 = {tri_h_pix, 6'b0} + {tri_h_pix, 3'b0} +
                            {6'b0, tri_h_pix};
    wire [17:0] tri_h_m51 = {tri_h_pix, 5'b0} + {tri_h_pix, 4'b0} +
                            {tri_h_pix, 1'b0} + {6'b0, tri_h_pix};
    wire [17:0] tri_h_m43 = {tri_h_pix, 5'b0} + {tri_h_pix, 3'b0} +
                            {tri_h_pix, 1'b0} + {6'b0, tri_h_pix};
    wire [17:0] tri_h_weighted = (tri_cand_ratio == 2'd0) ? tri_h_m73 :
                                 (tri_cand_ratio == 2'd1) ? tri_h_m51 :
                                 tri_h_m43;
    wire [11:0] tri_off_pix = tri_h_weighted[17:7];
    wire [12:0] tri_xl_pix = {1'b0, tri_cand_x0} + {1'b0, tri_off_pix};
    wire [12:0] tri_xr_pix = {1'b0, tri_cand_x1} - {1'b0, tri_off_pix};
    wire [12:0] tri_l_lo = (tri_xl_pix > TOL3_C) ? (tri_xl_pix - TOL3_C) : 13'd0;
    wire [12:0] tri_l_hi = tri_xl_pix + TOL3_C;
    wire [12:0] tri_r_lo = (tri_xr_pix > TOL3_C) ? (tri_xr_pix - TOL3_C) : 13'd0;
    wire [12:0] tri_r_hi = tri_xr_pix + TOL3_C;
    wire        tri_y_ok_pix = tri_cand_active && (i_row >= tri_cand_y0) &&
                               (i_row <= tri_cand_y1);
    wire [11:0] tri_need = ((tri_cand_y1 - tri_cand_y0) >> 2) + 12'd4;

    // A solid outlined triangle has a mostly empty interior.  Dense texture
    // (a checkerboard or busy tabletop) lights up the interior and is rejected
    // even when enough pixels accidentally line up with the two side slopes.
    wire [12:0] tri_inner_margin = {1'b0, TOL3_C};
    wire [12:0] tri_inner_l = tri_xl_pix + tri_inner_margin;
    wire [12:0] tri_inner_r = (tri_xr_pix > tri_inner_margin) ?
                               (tri_xr_pix - tri_inner_margin) : 13'd0;
    wire        tri_body_y_ok = (i_row + 12'd2 < tri_cand_y1);
    wire        tri_inner_window_ok = (tri_inner_r > tri_inner_l);
    wire [15:0] tri_inner_limit = {4'b0, tri_need};

    // The triangle geometry depends only on the candidate and the current row.
    // Calculating it for every pixel put a 42-level row-subtract/shift-add/
    // compare chain directly in front of the hit-counter enables at 148.5 MHz.
    // i_row is already the next row during the line-end blank, so evaluate the
    // complete row window once there and leave only registered comparisons in
    // the pixel path.  This also keeps the verified hit counts unchanged.
    reg  [12:0] tri_l_lo_r, tri_l_hi_r, tri_r_lo_r, tri_r_hi_r;
    reg  [12:0] tri_inner_l_r, tri_inner_r_r;
    reg         tri_y_ok_r, tri_body_y_ok_r, tri_inner_window_ok_r;

    wire        tri_hit_l = tri_y_ok_r && i_edge &&
                            (col13 >= tri_l_lo_r) && (col13 <= tri_l_hi_r);
    wire        tri_hit_r = tri_y_ok_r && i_edge &&
                            (col13 >= tri_r_lo_r) && (col13 <= tri_r_hi_r);
    wire        tri_inner_hit = tri_y_ok_r && tri_body_y_ok_r && i_edge &&
                                tri_inner_window_ok_r &&
                                (col13 >= tri_inner_l_r) &&
                                (col13 <= tri_inner_r_r);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tri_l_lo_r <= 13'd0; tri_l_hi_r <= 13'd0;
            tri_r_lo_r <= 13'd0; tri_r_hi_r <= 13'd0;
            tri_inner_l_r <= 13'd0; tri_inner_r_r <= 13'd0;
            tri_y_ok_r <= 1'b0; tri_body_y_ok_r <= 1'b0;
            tri_inner_window_ok_r <= 1'b0;
        end else if (end_line || rst_frame) begin
            tri_l_lo_r <= tri_l_lo;
            tri_l_hi_r <= tri_l_hi;
            tri_r_lo_r <= tri_r_lo;
            tri_r_hi_r <= tri_r_hi;
            tri_inner_l_r <= tri_inner_l;
            tri_inner_r_r <= tri_inner_r;
            tri_y_ok_r <= tri_y_ok_pix;
            tri_body_y_ok_r <= tri_body_y_ok;
            tri_inner_window_ok_r <= tri_inner_window_ok;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tri_cand_x0 <= 12'd0; tri_cand_y0 <= 12'd0;
            tri_cand_x1 <= 12'd0; tri_cand_y1 <= 12'd0;
            tri_cand_ratio <= 2'd0; tri_ratio_scan <= 2'd0;
            tri_cand_active <= 1'b0;
            tri_l_hits <= 12'd0; tri_r_hits <= 12'd0;
            tri_inner_hits <= 16'd0;
            tri_row_l <= 1'b0; tri_row_r <= 1'b0;
            tri_x0 <= 12'd0; tri_y0 <= 12'd0;
            tri_x1 <= 12'd0; tri_y1 <= 12'd0;
            tri_valid <= 1'b0; tri_hold <= 8'd0;
        end else if (rst_frame) begin
            // 用上一帧累积的候选命中数做判决
            if (tri_cand_active && i_en &&
                (tri_l_hits >= tri_need) && (tri_r_hits >= tri_need) &&
                (tri_inner_hits <= tri_inner_limit)) begin
                tri_valid <= 1'b1;
                tri_hold  <= THLD_C;
                tri_x0 <= tri_cand_x0; tri_y0 <= tri_cand_y0;
                tri_x1 <= tri_cand_x1; tri_y1 <= tri_cand_y1;
            end else if (tri_hold != 8'd0) begin
                tri_hold <= tri_hold - 1'b1;
                if (tri_hold == 8'd1) tri_valid <= 1'b0;
            end

            // 轮换选取一条水平长线作为下一帧三角形底边候选，同时轮换高度
            if (i_en && tb_v && (tb_len >= TWMN_C) &&
                (tb_y >= (tri_height_scan + 12'd12))) begin
                tri_cand_active <= 1'b1;
                tri_cand_x0 <= tb_x0;
                tri_cand_x1 <= tb_x1;
                tri_cand_y1 <= tb_y;
                tri_cand_y0 <= tb_y - tri_height_scan;
                tri_cand_ratio <= tri_ratio_scan;
            end else begin
                tri_cand_active <= 1'b0;
            end
            // Advance the height hypothesis only when a usable horizontal
            // base was actually seen.  Advancing every frame synchronised with
            // the three-frame rot schedule in the common case, so the same
            // (often too-tall) hypothesis was retried forever.
            if (i_en && tb_v && (tb_len >= TWMN_C)) begin
                tri_ratio_scan <= (tri_ratio_scan == 2'd2) ?
                                  2'd0 : (tri_ratio_scan + 1'b1);
            end
            tri_l_hits <= 12'd0; tri_r_hits <= 12'd0;
            tri_inner_hits <= 16'd0;
            tri_row_l <= 1'b0; tri_row_r <= 1'b0;
        end else if (end_line) begin
            if (tri_cand_active && i_en &&
                (row_cur >= tri_cand_y0) && (row_cur <= tri_cand_y1)) begin
                if (tri_row_l) tri_l_hits <= tri_l_hits + 1'b1;
                if (tri_row_r) tri_r_hits <= tri_r_hits + 1'b1;
            end
            tri_row_l <= 1'b0; tri_row_r <= 1'b0;
        end else if (i_de) begin
            if (tri_hit_l) tri_row_l <= 1'b1;
            if (tri_hit_r) tri_row_r <= 1'b1;
            if (tri_inner_hit && (tri_inner_hits != 16'hFFFF))
                tri_inner_hits <= tri_inner_hits + 1'b1;
        end
    end

    //-------------------------------------------------------------------------
    // 3c) Rotated triangle: vertical base on the left/right + horizontal apex
    //     The printed demo target may be rotated by about 90 degrees.  In that
    //     orientation the usual horizontal-base verifier cannot fire, so build
    //     a small 64-bin histogram of near-vertical edge columns.  The top
    //     three columns become candidate bases; the next frame measures their
    //     row extent, then verifies the two slanted sides against a vertical
    //     base.  Only 64x8 histogram bits are added (no frame buffer).
    //-------------------------------------------------------------------------
    // Cover the full 1920-pixel active line.  The previous 64 bins x 16 px
    // only counted x=0..1023; in mode 4 (i_draw_x0=0) a rotated triangle whose
    // vertical base was on the right half could therefore never become a
    // candidate at all.
    localparam integer VHIST_N   = 128;
    localparam [7:0]   VH_CNT_MIN = TRI_W_MIN >> 1;
    localparam [12:0]  VTOL_C    = TOL3_C + (TOL3_C >> 1);

    reg  [7:0]  vhist_mem [0:VHIST_N-1];
    reg  [1:0]  vrank_state;
    reg  [6:0]  vrank_ptr;
    reg  [7:0]  vrank_c1, vrank_c2, vrank_c3;
    reg  [6:0]  vrank_i1, vrank_i2, vrank_i3;
    reg         vhist_clear;
    integer     vhi;

    wire [11:0] vhist_rel = (col >= i_draw_x0) ? (col - i_draw_x0) : 12'd0;
    wire [6:0]  vhist_bin = vhist_rel[10:4];
    wire        vhist_roi = (col >= i_draw_x0) && (vhist_rel < 12'd2048);
    wire [2:0]  vhist_band = i_dir16[3:1];
    wire        vhist_near_vertical = (vhist_band <= 3'd1);
    wire        vhist_inc = (vrank_state == 2'd0) && i_en && i_de && i_edge &&
                            vhist_roi && vhist_near_vertical;

    wire [6:0]  vrank_d12 = (vrank_i1 > vrank_i2) ?
                            (vrank_i1 - vrank_i2) : (vrank_i2 - vrank_i1);
    wire [6:0]  vrank_d13 = (vrank_i1 > vrank_i3) ?
                            (vrank_i1 - vrank_i3) : (vrank_i3 - vrank_i1);
    wire [6:0]  vrank_d23 = (vrank_i2 > vrank_i3) ?
                            (vrank_i2 - vrank_i3) : (vrank_i3 - vrank_i2);
    wire        vrank_sep12 = (vrank_d12 >= 6'd2);
    wire        vrank_sep13 = (vrank_d13 >= 6'd2);
    wire        vrank_sep23 = (vrank_d23 >= 6'd2);

    reg  [11:0] vb_meas_x0, vb_meas_x1, vb_meas_x2;
    reg         vb_meas_v0, vb_meas_v1, vb_meas_v2;
    reg  [11:0] vby0_0, vby1_0, vby0_1, vby1_1, vby0_2, vby1_2;
    reg         vb_meas_on;
    reg  [1:0]  vb_slot;

    wire [12:0] vcol13 = {1'b0, col};
    wire [12:0] vb_dx0 = (vcol13 > {1'b0, vb_meas_x0}) ?
                         (vcol13 - {1'b0, vb_meas_x0}) : ({1'b0, vb_meas_x0} - vcol13);
    wire [12:0] vb_dx1 = (vcol13 > {1'b0, vb_meas_x1}) ?
                         (vcol13 - {1'b0, vb_meas_x1}) : ({1'b0, vb_meas_x1} - vcol13);
    wire [12:0] vb_dx2 = (vcol13 > {1'b0, vb_meas_x2}) ?
                         (vcol13 - {1'b0, vb_meas_x2}) : ({1'b0, vb_meas_x2} - vcol13);
    wire vb_hit0 = vb_meas_on && vb_meas_v0 && i_en && i_de && i_edge &&
                   vhist_near_vertical && (vb_dx0 <= VTOL_C);
    wire vb_hit1 = vb_meas_on && vb_meas_v1 && i_en && i_de && i_edge &&
                   vhist_near_vertical && (vb_dx1 <= VTOL_C);
    wire vb_hit2 = vb_meas_on && vb_meas_v2 && i_en && i_de && i_edge &&
                   vhist_near_vertical && (vb_dx2 <= VTOL_C);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (vhi = 0; vhi < VHIST_N; vhi = vhi + 1)
                vhist_mem[vhi] <= 8'd0;
            vrank_state <= 2'd0;
            vrank_ptr   <= 7'd0;
            vrank_c1 <= 8'd0; vrank_c2 <= 8'd0; vrank_c3 <= 8'd0;
            vrank_i1 <= 7'd0; vrank_i2 <= 7'd0; vrank_i3 <= 7'd0;
            vhist_clear <= 1'b0;
            vb_meas_x0 <= 12'd0; vb_meas_x1 <= 12'd0; vb_meas_x2 <= 12'd0;
            vb_meas_v0 <= 1'b0; vb_meas_v1 <= 1'b0; vb_meas_v2 <= 1'b0;
            vby0_0 <= 12'hFFF; vby1_0 <= 12'd0;
            vby0_1 <= 12'hFFF; vby1_1 <= 12'd0;
            vby0_2 <= 12'hFFF; vby1_2 <= 12'd0;
            vb_meas_on <= 1'b0;
        end else begin
            vhist_clear <= 1'b0;

            if (vhist_clear) begin
                // Fresh per-frame vertical-edge histogram.  Without this the
                // bins accumulate over the whole run and the top-3 "base"
                // columns drift onto arbitrary scene texture.
                for (vhi = 0; vhi < VHIST_N; vhi = vhi + 1)
                    vhist_mem[vhi] <= 8'd0;
            end else if (rst_frame) begin
                // Keep the completed frame's histogram, rank it during the
                // vertical blank, then start a fresh histogram.
                vrank_state <= 2'd1;
                vrank_ptr   <= 7'd0;
                vrank_c1 <= 8'd0; vrank_c2 <= 8'd0; vrank_c3 <= 8'd0;
                vrank_i1 <= 7'd0; vrank_i2 <= 7'd0; vrank_i3 <= 7'd0;
            end else if (vrank_state == 2'd1) begin
                if (vhist_mem[vrank_ptr] > vrank_c1) begin
                    vrank_c3 <= vrank_c2; vrank_i3 <= vrank_i2;
                    vrank_c2 <= vrank_c1; vrank_i2 <= vrank_i1;
                    vrank_c1 <= vhist_mem[vrank_ptr]; vrank_i1 <= vrank_ptr;
                end else if (vhist_mem[vrank_ptr] > vrank_c2) begin
                    vrank_c3 <= vrank_c2; vrank_i3 <= vrank_i2;
                    vrank_c2 <= vhist_mem[vrank_ptr]; vrank_i2 <= vrank_ptr;
                end else if (vhist_mem[vrank_ptr] > vrank_c3) begin
                    vrank_c3 <= vhist_mem[vrank_ptr]; vrank_i3 <= vrank_ptr;
                end
                if (vrank_ptr == (VHIST_N-1))
                    vrank_state <= 2'd2;
                else
                    vrank_ptr <= vrank_ptr + 1'b1;
            end else if (vrank_state == 2'd2) begin
                vb_meas_x0 <= i_draw_x0 + {1'b0, vrank_i1, 4'b0} + 12'd8;
                vb_meas_x1 <= i_draw_x0 + {1'b0, vrank_i2, 4'b0} + 12'd8;
                vb_meas_x2 <= i_draw_x0 + {1'b0, vrank_i3, 4'b0} + 12'd8;
                vb_meas_v0 <= (vrank_c1 >= VH_CNT_MIN);
                vb_meas_v1 <= (vrank_c2 >= VH_CNT_MIN) && vrank_sep12;
                vb_meas_v2 <= (vrank_c3 >= VH_CNT_MIN) &&
                              vrank_sep13 && vrank_sep23;
                vby0_0 <= 12'hFFF; vby1_0 <= 12'd0;
                vby0_1 <= 12'hFFF; vby1_1 <= 12'd0;
                vby0_2 <= 12'hFFF; vby1_2 <= 12'd0;
                vb_meas_on <= 1'b1;
                vhist_clear <= 1'b1;
                vrank_state <= 2'd0;
            end else begin
                if (vhist_inc) begin
                    if (vhist_mem[vhist_bin] != 8'hFF)
                        vhist_mem[vhist_bin] <= vhist_mem[vhist_bin] + 1'b1;
                end
                if (vb_hit0) begin
                    if (i_row < vby0_0) vby0_0 <= {1'b0, i_row};
                    if (i_row > vby1_0) vby1_0 <= {1'b0, i_row};
                end
                if (vb_hit1) begin
                    if (i_row < vby0_1) vby0_1 <= {1'b0, i_row};
                    if (i_row > vby1_1) vby1_1 <= {1'b0, i_row};
                end
                if (vb_hit2) begin
                    if (i_row < vby0_2) vby0_2 <= {1'b0, i_row};
                    if (i_row > vby1_2) vby1_2 <= {1'b0, i_row};
                end
            end
        end
    end

    wire [11:0] vb_sel_x = (vb_slot == 2'd0) ? vb_meas_x0 :
                           (vb_slot == 2'd1) ? vb_meas_x1 : vb_meas_x2;
    wire [11:0] vb_sel_y0 = (vb_slot == 2'd0) ? vby0_0 :
                            (vb_slot == 2'd1) ? vby0_1 : vby0_2;
    wire [11:0] vb_sel_y1 = (vb_slot == 2'd0) ? vby1_0 :
                            (vb_slot == 2'd1) ? vby1_1 : vby1_2;
    wire        vb_sel_valid = (vb_slot == 2'd0) ? vb_meas_v0 :
                               (vb_slot == 2'd1) ? vb_meas_v1 : vb_meas_v2;
    wire [11:0] vb_sel_h = (vb_sel_y1 > vb_sel_y0) ?
                           (vb_sel_y1 - vb_sel_y0) : 12'd0;
    wire        vb_sel_ok = vb_meas_on && vb_sel_valid &&
                            (vb_sel_y1 > vb_sel_y0) &&
                            (vb_sel_h >= TWMN_C) && (vb_sel_h <= RLMAX_C);

    // Try four depth ratios while keeping one vertical-base candidate for
    // four frames.  This covers printed equal-sided triangles and the older
    // tall reference sheet without four parallel per-pixel comparators.
    wire [11:0] tri_rot_ratio_depth =
        (tri_rot_ratio_scan == 2'd0) ? (vb_sel_h - (vb_sel_h >> 3)) :
        (tri_rot_ratio_scan == 2'd1) ? vb_sel_h :
        (tri_rot_ratio_scan == 2'd2) ? (vb_sel_h + (vb_sel_h >> 2)) :
                                       (vb_sel_h + (vb_sel_h >> 1) +
                                        (vb_sel_h >> 3));

    wire [11:0] tri_rot_h = (tri_rot_cand_y1 > tri_rot_cand_y0) ?
                            (tri_rot_cand_y1 - tri_rot_cand_y0) : 12'd0;
    // A rotated triangle must cover more than one quarter of its vertical
    // base.  The extra eighth rejects a rectangle corner that a single
    // slanted hypothesis can intersect for a short band, while a real printed
    // triangle still accumulates well over half of the base rows.
    wire [11:0] tri_rot_need = (tri_rot_h >> 2) + (tri_rot_h >> 3) + 12'd4;
    wire [11:0] tri_rot_sum_y = tri_rot_cand_y0 + tri_rot_cand_y1;
    wire [11:0] tri_rot_two_row = {1'b0, i_row, 1'b0};
    wire [11:0] tri_rot_dy2 = (tri_rot_sum_y > tri_rot_two_row) ?
                              (tri_rot_sum_y - tri_rot_two_row) :
                              (tri_rot_two_row - tri_rot_sum_y);
    wire [11:0] tri_rot_rem = (tri_rot_h > tri_rot_dy2) ?
                              (tri_rot_h - tri_rot_dy2) : 12'd0;
    wire [14:0] tri_rot_m13 = {tri_rot_rem, 3'b0} +
                              {tri_rot_rem, 2'b0} +
                              {3'b0, tri_rot_rem};
    wire [11:0] tri_rot_off =
        (tri_rot_cand_ratio == 2'd0) ? (tri_rot_rem - (tri_rot_rem >> 3)) :
        (tri_rot_cand_ratio == 2'd1) ? tri_rot_rem :
        (tri_rot_cand_ratio == 2'd2) ? (tri_rot_rem + (tri_rot_rem >> 2)) :
                                       tri_rot_m13[14:3];
    wire [12:0] tri_rot_xr = {1'b0, tri_rot_cand_x} + {1'b0, tri_rot_off};
    wire [12:0] tri_rot_xl_ex = {1'b0, tri_rot_cand_x} - {1'b0, tri_rot_off};
    wire [11:0] tri_rot_xl = (tri_rot_xl_ex < {1'b0, i_draw_x0}) ?
                             i_draw_x0 : tri_rot_xl_ex[11:0];
    wire [12:0] tri_rot_based = (vcol13 > {1'b0, tri_rot_cand_x}) ?
                                (vcol13 - {1'b0, tri_rot_cand_x}) :
                                ({1'b0, tri_rot_cand_x} - vcol13);
    wire [12:0] tri_rot_dxr = (vcol13 > tri_rot_xr) ?
                              (vcol13 - tri_rot_xr) : (tri_rot_xr - vcol13);
    wire [12:0] tri_rot_dxl = (vcol13 > {1'b0, tri_rot_xl}) ?
                              (vcol13 - {1'b0, tri_rot_xl}) :
                              ({1'b0, tri_rot_xl} - vcol13);
    wire tri_rot_y_ok = tri_rot_cand_active && i_en && i_de &&
                        (i_row >= tri_rot_cand_y0) && (i_row <= tri_rot_cand_y1);
    wire tri_rot_base_hit = tri_rot_y_ok && i_edge && vhist_near_vertical &&
                            (tri_rot_based <= VTOL_C);
    // A slanted side needs diagonal direction evidence as well as distance.
    // Without this, a checkerboard's vertical grid can be counted as a
    // triangle slant merely because several columns fall inside the tolerance
    // window.  Both 45-degree triangle sides land in bands 2..5.
    wire tri_rot_slant_dir_ok = (i_dir16[3:1] >= 3'd2) &&
                                (i_dir16[3:1] <= 3'd5);
    wire tri_rot_slant_r = tri_rot_y_ok && i_edge && tri_rot_slant_dir_ok &&
                           (tri_rot_dxr <= VTOL_C);
    wire tri_rot_slant_l = tri_rot_y_ok && i_edge && tri_rot_slant_dir_ok &&
                           (tri_rot_dxl <= VTOL_C);

    // A hollow printed triangle has a mostly empty interior.  Dense texture
    // (checkerboard, cluttered tabletop) fills it with edges; reject those
    // candidates instead of relying on the histogram alone.
    // The interior test must exclude the slanted stroke itself.  With an
    // 18-pixel slant tolerance, an 8-pixel inset still counted the real edge
    // as interior texture and rejected genuine triangles.
    localparam [12:0] RTRI_INNER_MARGIN = 13'd24;
    wire [12:0] tri_rot_in_r_lo = {1'b0, tri_rot_cand_x} + RTRI_INNER_MARGIN;
    wire [12:0] tri_rot_in_r_hi = (tri_rot_xr > RTRI_INNER_MARGIN) ?
                                   (tri_rot_xr - RTRI_INNER_MARGIN) : 13'd0;
    wire [12:0] tri_rot_in_l_lo = tri_rot_xl + RTRI_INNER_MARGIN;
    wire [12:0] tri_rot_in_l_hi =
        ({1'b0, tri_rot_cand_x} > RTRI_INNER_MARGIN) ?
        ({1'b0, tri_rot_cand_x} - RTRI_INNER_MARGIN) : 13'd0;
    wire tri_rot_interior_r = tri_rot_y_ok && i_edge &&
                              (tri_rot_in_r_hi > tri_rot_in_r_lo) &&
                              (vcol13 >= tri_rot_in_r_lo) &&
                              (vcol13 <= tri_rot_in_r_hi);
    wire tri_rot_interior_l = tri_rot_y_ok && i_edge &&
                              (tri_rot_in_l_hi > tri_rot_in_l_lo) &&
                              (vcol13 >= tri_rot_in_l_lo) &&
                              (vcol13 <= tri_rot_in_l_hi);
    wire [15:0] tri_rot_inner_limit = {4'b0, tri_rot_need};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tri_rot_cand_x <= 12'd0; tri_rot_cand_y0 <= 12'd0;
            tri_rot_cand_y1 <= 12'd0; tri_rot_cand_depth <= 12'd0;
            tri_rot_cand_ratio <= 2'd0;
            tri_rot_cand_active <= 1'b0;
            tri_rot_l_hits <= 12'd0; tri_rot_r_hits <= 12'd0;
            tri_rot_ir_hits <= 16'd0; tri_rot_il_hits <= 16'd0;
            tri_rot_row_base <= 1'b0; tri_rot_row_l <= 1'b0; tri_rot_row_r <= 1'b0;
            tri_rot_row_ir <= 1'b0; tri_rot_row_il <= 1'b0;
            tri_rot_x0 <= 12'd0; tri_rot_y0 <= 12'd0;
            tri_rot_x1 <= 12'd0; tri_rot_y1 <= 12'd0;
            tri_rot_valid <= 1'b0; tri_rot_hold <= 8'd0;
            tri_rot_ratio_scan <= 2'd0;
            vb_slot <= 2'd0;
        end else if (rst_frame) begin
            if (tri_rot_cand_active && i_en &&
                ((tri_rot_l_hits >= tri_rot_need) ||
                 (tri_rot_r_hits >= tri_rot_need)) &&
                (((tri_rot_r_hits >= tri_rot_l_hits) &&
                  (tri_rot_ir_hits <= tri_rot_inner_limit)) ||
                 ((tri_rot_r_hits < tri_rot_l_hits) &&
                  (tri_rot_il_hits <= tri_rot_inner_limit)))) begin
                tri_rot_valid <= 1'b1;
                tri_rot_hold  <= THLD_C;
                tri_rot_y0 <= tri_rot_cand_y0;
                tri_rot_y1 <= tri_rot_cand_y1;
                if (tri_rot_r_hits >= tri_rot_l_hits) begin
                    tri_rot_x0 <= tri_rot_cand_x;
                    tri_rot_x1 <= tri_rot_cand_x + tri_rot_cand_depth;
                end else begin
                    tri_rot_x1 <= tri_rot_cand_x;
                    tri_rot_x0 <= (tri_rot_cand_x > tri_rot_cand_depth) ?
                                  (tri_rot_cand_x - tri_rot_cand_depth) : i_draw_x0;
                end
            end else if (tri_rot_hold != 8'd0) begin
                tri_rot_hold <= tri_rot_hold - 1'b1;
                if (tri_rot_hold == 8'd1) tri_rot_valid <= 1'b0;
            end

            if (i_en && vb_sel_ok) begin
                tri_rot_cand_active <= 1'b1;
                tri_rot_cand_x <= vb_sel_x;
                tri_rot_cand_y0 <= vb_sel_y0;
                tri_rot_cand_y1 <= vb_sel_y1;
                tri_rot_cand_depth <= tri_rot_ratio_depth;
                tri_rot_cand_ratio <= tri_rot_ratio_scan;
            end else begin
                tri_rot_cand_active <= 1'b0;
            end
            tri_rot_ratio_scan <= (tri_rot_ratio_scan == 2'd3) ?
                                  2'd0 : (tri_rot_ratio_scan + 1'b1);
            if (tri_rot_ratio_scan == 2'd3)
                vb_slot <= (vb_slot == 2'd2) ? 2'd0 : (vb_slot + 1'b1);
            tri_rot_l_hits <= 12'd0; tri_rot_r_hits <= 12'd0;
            tri_rot_ir_hits <= 16'd0; tri_rot_il_hits <= 16'd0;
            tri_rot_row_base <= 1'b0; tri_rot_row_l <= 1'b0; tri_rot_row_r <= 1'b0;
            tri_rot_row_ir <= 1'b0; tri_rot_row_il <= 1'b0;
        end else if (end_line) begin
            if (tri_rot_cand_active && i_en &&
                (row_cur >= tri_rot_cand_y0) && (row_cur <= tri_rot_cand_y1)) begin
                if (tri_rot_row_base && tri_rot_row_l)
                    tri_rot_l_hits <= tri_rot_l_hits + 1'b1;
                if (tri_rot_row_base && tri_rot_row_r)
                    tri_rot_r_hits <= tri_rot_r_hits + 1'b1;
                if (tri_rot_row_ir)
                    tri_rot_ir_hits <= tri_rot_ir_hits + 1'b1;
                if (tri_rot_row_il)
                    tri_rot_il_hits <= tri_rot_il_hits + 1'b1;
            end
            tri_rot_row_base <= 1'b0; tri_rot_row_l <= 1'b0; tri_rot_row_r <= 1'b0;
            tri_rot_row_ir <= 1'b0; tri_rot_row_il <= 1'b0;
        end else if (i_de) begin
            if (tri_rot_base_hit) tri_rot_row_base <= 1'b1;
            if (tri_rot_slant_r) tri_rot_row_r <= 1'b1;
            if (tri_rot_slant_l) tri_rot_row_l <= 1'b1;
            if (tri_rot_interior_r) tri_rot_row_ir <= 1'b1;
            if (tri_rot_interior_l) tri_rot_row_il <= 1'b1;
        end
    end

    //-------------------------------------------------------------------------
    // 4) 绘制: 三种目标统一为细框 + 上方中文小标签   {B,G,R}
    //-------------------------------------------------------------------------
    // 2 级随路延迟; 框和标签都用 b_col/b_row 绘制, 与 b_rgb 同拍
    reg         a_de, a_hs, a_vs;
    reg         b_de, b_hs, b_vs;
    reg  [23:0] a_rgb, b_rgb;

    reg [11:0] a_col_d, a_row_d, b_col, b_row;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_rgb <= 24'd0;
            a_de <= 1'b0; a_hs <= 1'b0; a_vs <= 1'b0;
            a_col_d <= 12'd0; a_row_d <= 12'd0;
        end else begin
            a_rgb   <= i_rgb;
            a_de    <= i_de; a_hs <= i_hs; a_vs <= i_vs;
            a_col_d <= col;  a_row_d <= {1'b0, i_row};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            b_rgb <= 24'd0; b_de <= 1'b0; b_hs <= 1'b0; b_vs <= 1'b0;
            b_col <= 12'd0; b_row <= 12'd0;
        end else begin
            b_rgb <= a_rgb; b_de <= a_de; b_hs <= a_hs; b_vs <= a_vs;
            b_col <= a_col_d; b_row <= a_row_d;
        end
    end

    // 5x7 ASCII 字库: 0=C, 1=I, 2=R, 3=L, 4=E, 5=T; bit4 为最左像素.
    function [4:0] font5;
        input [2:0] ch;
        input [2:0] row;
        begin
            font5 = 5'd0;
            case (ch)
                3'd0: case (row) // C
                    3'd0: font5 = 5'b01110; 3'd1: font5 = 5'b10001;
                    3'd2: font5 = 5'b10000; 3'd3: font5 = 5'b10000;
                    3'd4: font5 = 5'b10000; 3'd5: font5 = 5'b10001;
                    3'd6: font5 = 5'b01110; default: font5 = 5'd0;
                endcase
                3'd1: case (row) // I
                    3'd0: font5 = 5'b11111; 3'd1: font5 = 5'b00100;
                    3'd2: font5 = 5'b00100; 3'd3: font5 = 5'b00100;
                    3'd4: font5 = 5'b00100; 3'd5: font5 = 5'b00100;
                    3'd6: font5 = 5'b11111; default: font5 = 5'd0;
                endcase
                3'd2: case (row) // R
                    3'd0: font5 = 5'b11110; 3'd1: font5 = 5'b10001;
                    3'd2: font5 = 5'b10001; 3'd3: font5 = 5'b11110;
                    3'd4: font5 = 5'b10100; 3'd5: font5 = 5'b10010;
                    3'd6: font5 = 5'b10001; default: font5 = 5'd0;
                endcase
                3'd3: case (row) // L
                    3'd0: font5 = 5'b10000; 3'd1: font5 = 5'b10000;
                    3'd2: font5 = 5'b10000; 3'd3: font5 = 5'b10000;
                    3'd4: font5 = 5'b10000; 3'd5: font5 = 5'b10000;
                    3'd6: font5 = 5'b11111; default: font5 = 5'd0;
                endcase
                3'd4: case (row) // E
                    3'd0: font5 = 5'b11111; 3'd1: font5 = 5'b10000;
                    3'd2: font5 = 5'b10000; 3'd3: font5 = 5'b11110;
                    3'd4: font5 = 5'b10000; 3'd5: font5 = 5'b10000;
                    3'd6: font5 = 5'b11111; default: font5 = 5'd0;
                endcase
                3'd5: case (row) // T
                    3'd0: font5 = 5'b11111; 3'd1: font5 = 5'b00100;
                    3'd2: font5 = 5'b00100; 3'd3: font5 = 5'b00100;
                    3'd4: font5 = 5'b00100; 3'd5: font5 = 5'b00100;
                    3'd6: font5 = 5'b00100; default: font5 = 5'd0;
                endcase
                default: font5 = 5'd0;
            endcase
        end
    endfunction

    function [2:0] circ_char;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: circ_char = 3'd0; // C
                3'd1: circ_char = 3'd1; // I
                3'd2: circ_char = 3'd2; // R
                default: circ_char = 3'd7;
            endcase
        end
    endfunction

    function [2:0] rect_char;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: rect_char = 3'd2; // R
                3'd1: rect_char = 3'd4; // E
                3'd2: rect_char = 3'd0; // C
                default: rect_char = 3'd7;
            endcase
        end
    endfunction

    function [2:0] tri_char;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: tri_char = 3'd5; // T
                3'd1: tri_char = 3'd2; // R
                3'd2: tri_char = 3'd1; // I
                default: tri_char = 3'd7;
            endcase
        end
    endfunction

    // 16x16 Chinese bitmap labels used by mode 4.  The reference capture shows
    // 圆 / 三角形 / 矩形 rather than Latin abbreviations.  bit15 is leftmost.
    function [15:0] font16_cn;
        input [2:0] ch;
        input [3:0] row;
        begin
            font16_cn = 16'h0000;
            case (ch)
                3'd0: case (row) // 圆
                    4'd0: font16_cn=16'h0000; 4'd1: font16_cn=16'h3FFC;
                    4'd2: font16_cn=16'h7FFE; 4'd3: font16_cn=16'h27E4;
                    4'd4: font16_cn=16'h27E4; 4'd5: font16_cn=16'h27E4;
                    4'd6: font16_cn=16'h2004; 4'd7: font16_cn=16'h27E4;
                    4'd8: font16_cn=16'h2424; 4'd9: font16_cn=16'h25A4;
                    4'd10:font16_cn=16'h2D24; 4'd11:font16_cn=16'h2364;
                    4'd12:font16_cn=16'h2E34; 4'd13:font16_cn=16'h6816;
                    4'd14:font16_cn=16'h7FFE; default:font16_cn=16'h2004;
                endcase
                3'd1: case (row) // 三
                    4'd0: font16_cn=16'h0000; 4'd1: font16_cn=16'h1FF8;
                    4'd2: font16_cn=16'h3FFC; 4'd3: font16_cn=16'h0000;
                    4'd4: font16_cn=16'h0000; 4'd5: font16_cn=16'h0000;
                    4'd6: font16_cn=16'h1FF0; 4'd7: font16_cn=16'h1FF8;
                    4'd8: font16_cn=16'h0000; 4'd9: font16_cn=16'h0000;
                    4'd10:font16_cn=16'h0000; 4'd11:font16_cn=16'h0000;
                    4'd12:font16_cn=16'h7FFE; 4'd13:font16_cn=16'h7FFE;
                    4'd14:font16_cn=16'h0000; default:font16_cn=16'h0000;
                endcase
                3'd2: case (row) // 角
                    4'd0: font16_cn=16'h0000; 4'd1: font16_cn=16'h0600;
                    4'd2: font16_cn=16'h0600; 4'd3: font16_cn=16'h0FF0;
                    4'd4: font16_cn=16'h1860; 4'd5: font16_cn=16'h7FFC;
                    4'd6: font16_cn=16'h198C; 4'd7: font16_cn=16'h1884;
                    4'd8: font16_cn=16'h1FFC; 4'd9: font16_cn=16'h1084;
                    4'd10:font16_cn=16'h188C; 4'd11:font16_cn=16'h1FFC;
                    4'd12:font16_cn=16'h3084; 4'd13:font16_cn=16'h3084;
                    4'd14:font16_cn=16'h609C; default:font16_cn=16'h0000;
                endcase
                3'd3: case (row) // 形
                    4'd0: font16_cn=16'h0000; 4'd1: font16_cn=16'h7F8C;
                    4'd2: font16_cn=16'h1218; 4'd3: font16_cn=16'h1230;
                    4'd4: font16_cn=16'h1260; 4'd5: font16_cn=16'h1204;
                    4'd6: font16_cn=16'h7F8C; 4'd7: font16_cn=16'h1318;
                    4'd8: font16_cn=16'h1270; 4'd9: font16_cn=16'h1266;
                    4'd10:font16_cn=16'h320E; 4'd11:font16_cn=16'h3218;
                    4'd12:font16_cn=16'h6230; 4'd13:font16_cn=16'h6260;
                    4'd14:font16_cn=16'h0000; default:font16_cn=16'h0000;
                endcase
                3'd4: case (row) // 矩
                    4'd0: font16_cn=16'h0000; 4'd1: font16_cn=16'h3000;
                    4'd2: font16_cn=16'h30FE; 4'd3: font16_cn=16'h3E80;
                    4'd4: font16_cn=16'h7E80; 4'd5: font16_cn=16'h4880;
                    4'd6: font16_cn=16'h08FC; 4'd7: font16_cn=16'h0884;
                    4'd8: font16_cn=16'h7E84; 4'd9: font16_cn=16'h0884;
                    4'd10:font16_cn=16'h18FC; 4'd11:font16_cn=16'h1C80;
                    4'd12:font16_cn=16'h3680; 4'd13:font16_cn=16'h3380;
                    4'd14:font16_cn=16'h60FE; default:font16_cn=16'h0000;
                endcase
                default: font16_cn = 16'h0000;
            endcase
        end
    endfunction

    function [2:0] circ_char_cn;
        input [2:0] idx;
        begin
            circ_char_cn = (idx == 3'd0) ? 3'd0 : 3'd7; // 圆
        end
    endfunction

    function [2:0] rect_char_cn;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: rect_char_cn = 3'd4; // 矩
                3'd1: rect_char_cn = 3'd3; // 形
                default: rect_char_cn = 3'd7;
            endcase
        end
    endfunction

    function [2:0] tri_char_cn;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: tri_char_cn = 3'd1; // 三
                3'd1: tri_char_cn = 3'd2; // 角
                3'd2: tri_char_cn = 3'd3; // 形
                default: tri_char_cn = 3'd7;
            endcase
        end
    endfunction

    // Reference capture: every detected target gets a cyan bounding box and a
    // black label plate with white Chinese glyphs.  Keeping one box colour
    // also avoids confusing the cyan annotation with the red Sobel contour.
    localparam [23:0] COL_CIRC = 24'h00FFFF; // cyan {B,G,R}
    localparam [23:0] COL_RECT = 24'h00FFFF; // cyan
    localparam [23:0] COL_TRI  = 24'h00FFFF; // cyan
    localparam [23:0] COL_LBL  = 24'h000000; // black label plate
    localparam [23:0] COL_TXT  = 24'hFFFFFF; // white glyph
    localparam [11:0] DRAW_M   = 12'd6;      // 框向外扩，避免压在白色边缘上

    wire [11:0] circ_rr_d = circ_rr + DRAW_M;
    wire [11:0] circ_ry_d = circ_ry + DRAW_M;
    wire [11:0] circ_bx0_raw = (circ_cx > circ_rr_d) ? (circ_cx - circ_rr_d) : 12'd0;
    wire [11:0] circ_bx0 = (circ_bx0_raw < i_draw_x0) ? i_draw_x0 : circ_bx0_raw;
    wire [12:0] circ_bx1_ex = {1'b0, circ_cx} + {1'b0, circ_rr_d};
    wire [11:0] circ_bx1 = (circ_bx1_ex >= H_ACTIVE) ? (H_ACTIVE-1) : circ_bx1_ex[11:0];
    wire [11:0] circ_by0 = (circ_cy > circ_ry_d) ? (circ_cy - circ_ry_d) : 12'd0;
    wire [12:0] circ_by1_ex = {1'b0, circ_cy} + {1'b0, circ_ry_d};
    wire [11:0] circ_by1 = (circ_by1_ex >= V_ACTIVE) ? (V_ACTIVE-1) : circ_by1_ex[11:0];

    wire [11:0] rect_dx0_raw = (rect_x0 > DRAW_M) ? (rect_x0 - DRAW_M) : 12'd0;
    wire [11:0] rect_dx0 = (rect_dx0_raw < i_draw_x0) ? i_draw_x0 : rect_dx0_raw;
    wire [12:0] rect_dx1_ex = {1'b0, rect_x1} + {1'b0, DRAW_M};
    wire [11:0] rect_dx1 = (rect_dx1_ex >= H_ACTIVE) ? (H_ACTIVE-1) : rect_dx1_ex[11:0];
    wire [11:0] rect_dy0 = (rect_y0 > DRAW_M) ? (rect_y0 - DRAW_M) : 12'd0;
    wire [12:0] rect_dy1_ex = {1'b0, rect_y1} + {1'b0, DRAW_M};
    wire [11:0] rect_dy1 = (rect_dy1_ex >= V_ACTIVE) ? (V_ACTIVE-1) : rect_dy1_ex[11:0];

    wire tri_box_valid = tri_valid | tri_rot_valid;
    wire [11:0] tri_box_x0 = tri_rot_valid ? tri_rot_x0 : tri_x0;
    wire [11:0] tri_box_y0 = tri_rot_valid ? tri_rot_y0 : tri_y0;
    wire [11:0] tri_box_x1 = tri_rot_valid ? tri_rot_x1 : tri_x1;
    wire [11:0] tri_box_y1 = tri_rot_valid ? tri_rot_y1 : tri_y1;
    wire [11:0] tri_dx0_raw = (tri_box_x0 > DRAW_M) ? (tri_box_x0 - DRAW_M) : 12'd0;
    wire [11:0] tri_dx0 = (tri_dx0_raw < i_draw_x0) ? i_draw_x0 : tri_dx0_raw;
    wire [12:0] tri_dx1_ex = {1'b0, tri_box_x1} + {1'b0, DRAW_M};
    wire [11:0] tri_dx1 = (tri_dx1_ex >= H_ACTIVE) ? (H_ACTIVE-1) : tri_dx1_ex[11:0];
    wire [11:0] tri_dy0 = (tri_box_y0 > DRAW_M) ? (tri_box_y0 - DRAW_M) : 12'd0;
    wire [12:0] tri_dy1_ex = {1'b0, tri_box_y1} + {1'b0, DRAW_M};
    wire [11:0] tri_dy1 = (tri_dy1_ex >= V_ACTIVE) ? (V_ACTIVE-1) : tri_dy1_ex[11:0];

    // 2-pixel colored boxes stay legible on the HDMI monitor and match the
    // reference capture without touching the one-pixel white edge map.
    wire rect_y_lo = (b_row == rect_dy0) || (b_row == (rect_dy0 + 12'd1));
    wire rect_y_hi = (b_row == rect_dy1) ||
                     ((rect_dy1 != 12'd0) && (b_row == (rect_dy1 - 12'd1)));
    wire rect_x_lo = (b_col == rect_dx0) || (b_col == (rect_dx0 + 12'd1));
    wire rect_x_hi = (b_col == rect_dx1) ||
                     ((rect_dx1 != 12'd0) && (b_col == (rect_dx1 - 12'd1)));
    wire circ_y_lo = (b_row == circ_by0) || (b_row == (circ_by0 + 12'd1));
    wire circ_y_hi = (b_row == circ_by1) ||
                     ((circ_by1 != 12'd0) && (b_row == (circ_by1 - 12'd1)));
    wire circ_x_lo = (b_col == circ_bx0) || (b_col == (circ_bx0 + 12'd1));
    wire circ_x_hi = (b_col == circ_bx1) ||
                     ((circ_bx1 != 12'd0) && (b_col == (circ_bx1 - 12'd1)));
    wire tri_y_lo = (b_row == tri_dy0) || (b_row == (tri_dy0 + 12'd1));
    wire tri_y_hi = (b_row == tri_dy1) ||
                    ((tri_dy1 != 12'd0) && (b_row == (tri_dy1 - 12'd1)));
    wire tri_x_lo = (b_col == tri_dx0) || (b_col == (tri_dx0 + 12'd1));
    wire tri_x_hi = (b_col == tri_dx1) ||
                    ((tri_dx1 != 12'd0) && (b_col == (tri_dx1 - 12'd1)));

    wire on_rect_h = i_en && rect_valid && (rect_y_lo || rect_y_hi) &&
                     (b_col >= i_draw_x0) &&
                     (b_col >= rect_dx0) && (b_col <= rect_dx1);
    wire on_rect_v = i_en && rect_valid && (rect_x_lo || rect_x_hi) &&
                     (b_col >= i_draw_x0) &&
                     (b_row >= rect_dy0) && (b_row <= rect_dy1);
    // 识别结果叠加：与参考图一致，圆/矩形/三角形都画细外框，
    // 白色边缘仍由 edge_proc 的二值边缘提供。圆不再画八边形环，
    // 这样框线与目标本身的白色双线轮廓不会互相干扰。
    wire on_circ_h = i_en && circ_valid && (circ_y_lo || circ_y_hi) &&
                     (b_col >= i_draw_x0) &&
                     (b_col >= circ_bx0) && (b_col <= circ_bx1);
    wire on_circ_v = i_en && circ_valid && (circ_x_lo || circ_x_hi) &&
                     (b_col >= i_draw_x0) &&
                     (b_row >= circ_by0) && (b_row <= circ_by1);
    wire on_circ   = on_circ_h | on_circ_v;

    // 三角形用外接矩形框（参考图 TRI 的显示方式），不再只画三条斜边。
    wire on_tri_h = i_en && tri_box_valid && (tri_y_lo || tri_y_hi) &&
                    (b_col >= i_draw_x0) &&
                    (b_col >= tri_dx0) && (b_col <= tri_dx1);
    wire on_tri_v = i_en && tri_box_valid && (tri_x_lo || tri_x_hi) &&
                    (b_col >= i_draw_x0) &&
                    (b_row >= tri_dy0) && (b_row <= tri_dy1);
    wire on_tri   = on_tri_h | on_tri_v;

    // 标签放在框上方; 16x16 中文字库. 圆=1 字, 矩形=2 字, 三角形=3 字.
    wire [11:0] circ_lx = circ_bx0 + 12'd2;
    wire [11:0] circ_ly = (circ_by0 >= 12'd22) ? (circ_by0 - 12'd20) : (circ_by0 + 12'd2);
    wire [11:0] circ_dx = b_col - circ_lx;
    wire [11:0] circ_dy = b_row - circ_ly;
    wire [2:0]  circ_ci = circ_dx[6:4];
    wire [3:0]  circ_gx = circ_dx[3:0];
    wire [2:0]  circ_ch = circ_char_cn(circ_ci);
    wire [15:0] circ_glyph = font16_cn(circ_ch, circ_dy[3:0]);
    wire        circ_lb = i_en && circ_valid && b_de &&
                          (b_col >= i_draw_x0) && (b_col >= circ_lx) &&
                          (b_row >= circ_ly) &&
                          (circ_dx < 12'd20) && (circ_dy < 12'd20);
    wire        circ_lp = i_en && circ_valid && b_de &&
                          (b_col >= i_draw_x0) && (b_col >= circ_lx) &&
                          (b_row >= circ_ly) &&
                          (circ_dx < 12'd16) && (circ_dy < 12'd16) &&
                          circ_glyph[4'd15 - circ_gx];

    wire [11:0] rect_lx = rect_dx0 + 12'd2;
    wire [11:0] rect_ly = (rect_dy0 >= 12'd22) ? (rect_dy0 - 12'd20) : (rect_dy0 + 12'd2);
    wire [11:0] rect_dx = b_col - rect_lx;
    wire [11:0] rect_dy = b_row - rect_ly;
    wire [2:0]  rect_ci = rect_dx[6:4];
    wire [3:0]  rect_gx = rect_dx[3:0];
    wire [2:0]  rect_ch = rect_char_cn(rect_ci);
    wire [15:0] rect_glyph = font16_cn(rect_ch, rect_dy[3:0]);
    wire        rect_lb = i_en && rect_valid && b_de &&
                          (b_col >= i_draw_x0) && (b_col >= rect_lx) &&
                          (b_row >= rect_ly) &&
                          (rect_dx < 12'd36) && (rect_dy < 12'd20);
    wire        rect_lp = i_en && rect_valid && b_de &&
                          (b_col >= i_draw_x0) && (b_col >= rect_lx) &&
                          (b_row >= rect_ly) &&
                          (rect_dx < 12'd32) && (rect_dy < 12'd16) &&
                          rect_glyph[4'd15 - rect_gx];

    // TRI 标签也放在框的左上角，与参考图的三个标签对齐方式一致。
    wire [11:0] tri_lbl_x_raw = tri_dx0 + 12'd2;
    wire [11:0] tri_lbl_x = (tri_lbl_x_raw < i_draw_x0) ? i_draw_x0 : tri_lbl_x_raw;
    wire [11:0] tri_ly = (tri_dy0 >= 12'd22) ? (tri_dy0 - 12'd20) : (tri_dy0 + 12'd2);
    wire [11:0] tri_dx = b_col - tri_lbl_x;
    wire [11:0] tri_dy = b_row - tri_ly;
    wire [2:0]  tri_ci = tri_dx[6:4];
    wire [3:0]  tri_gx = tri_dx[3:0];
    wire [2:0]  tri_ch = tri_char_cn(tri_ci);
    wire [15:0] tri_glyph = font16_cn(tri_ch, tri_dy[3:0]);
    wire        tri_lb = i_en && tri_box_valid && b_de &&
                         (b_col >= i_draw_x0) && (b_col >= tri_lbl_x) &&
                         (b_row >= tri_ly) &&
                         (tri_dx < 12'd52) && (tri_dy < 12'd20);
    wire        tri_lp = i_en && tri_box_valid && b_de &&
                         (b_col >= i_draw_x0) && (b_col >= tri_lbl_x) &&
                         (b_row >= tri_ly) &&
                         (tri_dx < 12'd48) && (tri_dy < 12'd16) &&
                          tri_glyph[4'd15 - tri_gx];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_rgb <= 24'd0; o_de <= 1'b0; o_hs <= 1'b0; o_vs <= 1'b0;
            o_shape_hit <= 1'b0;
        end else begin
        o_rgb <= circ_lp ? COL_TXT  :
                 rect_lp ? COL_TXT  :
                 tri_lp  ? COL_TXT  :
                 circ_lb ? COL_LBL  :
                 rect_lb ? COL_LBL  :
                 tri_lb  ? COL_LBL  :
                 on_circ ? COL_CIRC :
                 (on_rect_h | on_rect_v) ? COL_RECT :
                 on_tri  ? COL_TRI  : b_rgb;
            o_de <= b_de; o_hs <= b_hs; o_vs <= b_vs;
            o_shape_hit <= (rect_valid | circ_valid | tri_box_valid) & i_en;
        end
    end

    assign o_dbg = {(rect_valid | tri_box_valid), circ_valid, clr_done, fifo_drop};


endmodule
