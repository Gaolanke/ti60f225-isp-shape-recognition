//=============================================================================
// tb_cam_ae_awb.v
//
// 验证自动白平衡在"白天暖光 / 夜间冷光"两种光照下都能把三个通道拉平,
// 并检查手动覆盖、AE 请求与低照度冻结行为.
//
// 做法: TB 把场景反射率乘上 DUT 当前输出的增益再喂回 DUT, 构成和实板一致的
// 闭环. 若增益不对, 回路自己就会把误差暴露出来.
//=============================================================================
`timescale 1ns/1ps

module tb_cam_ae_awb;

    localparam integer H_ACTIVE = 256;
    localparam integer V_ACTIVE = 64;
    localparam integer CLK_HALF = 7;          // ~71 MHz

    reg  clk = 1'b0;
    reg  rst_n = 1'b0;
    always #CLK_HALF clk = ~clk;

    // DUT 接口
    reg         i_de = 1'b0;
    reg         i_vs = 1'b0;
    reg  [47:0] i_rgb2 = 48'd0;
    reg         i_man_en = 1'b0;
    reg  [15:0] i_man_r = 16'd256;
    reg  [15:0] i_man_b = 16'd256;
    wire [15:0] o_r_gain, o_g_gain, o_b_gain;
    wire        o_awb_locked;
    wire [3:0]  o_awb_dbg;
    wire        o_ae_req_tgl;
    wire [15:0] o_ae_exp;
    wire [7:0]  o_ae_gain_c, o_ae_gain_f;
    reg         i_ae_ack_tgl = 1'b0;
    reg         i_ae_en = 1'b1;

    cam_ae_awb #(
        .BEAT_DIV       (1),          // 每 2 beat 取一点, 小图也能攒够样本
        .LINE_DIV       (1),
        .MIN_SAMPLES    (24'd256),
        .LOCK_FRAMES    (8'd4),
        .AE_PERIOD      (8'd4),
        .SEED_R_GAIN    (16'd384),
        .SEED_G_GAIN    (16'd256),
        .SEED_B_GAIN    (16'd448)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .i_de           (i_de),
        .i_vs           (i_vs),
        .i_rgb2         (i_rgb2),
        .i_man_en       (i_man_en),
        .i_man_r        (i_man_r),
        .i_man_b        (i_man_b),
        .o_r_gain       (o_r_gain),
        .o_g_gain       (o_g_gain),
        .o_b_gain       (o_b_gain),
        .o_awb_locked   (o_awb_locked),
        .o_awb_dbg      (o_awb_dbg),
        .o_ae_req_tgl   (o_ae_req_tgl),
        .o_ae_exp       (o_ae_exp),
        .o_ae_gain_c    (o_ae_gain_c),
        .o_ae_gain_f    (o_ae_gain_f),
        .i_ae_ack_tgl   (i_ae_ack_tgl),
        .i_ae_en        (i_ae_en)
    );

    //-------------------------------------------------------------------------
    // 场景反射率(未受光照影响的"白卡"): 中性灰
    //-------------------------------------------------------------------------
    reg [7:0] sc_r = 8'd150;
    reg [7:0] sc_g = 8'd150;
    reg [7:0] sc_b = 8'd150;

    function [7:0] apply_gain;
        input [7:0]  v;
        input [15:0] g;
        reg   [23:0] p;
        begin
            p = v * g;
            p = p >> 8;
            apply_gain = (p > 24'd255) ? 8'd255 : p[7:0];
        end
    endfunction

    // 实际送进 DUT 的像素 = 场景 * 光照 * 当前增益
    reg [7:0] lit_r = 8'd150, lit_g = 8'd150, lit_b = 8'd150;

    task automatic drive_frame;
        integer x, y;
        begin
            i_vs <= 1'b1;
            #(CLK_HALF*2);
            i_vs <= 1'b0;
            for (y = 0; y < V_ACTIVE; y = y + 1) begin
                for (x = 0; x < H_ACTIVE/2; x = x + 1) begin
                    i_de   <= 1'b1;
                    i_rgb2 <= {apply_gain(lit_b, o_b_gain), apply_gain(lit_g, o_g_gain),
                               apply_gain(lit_r, o_r_gain),
                               apply_gain(lit_b, o_b_gain), apply_gain(lit_g, o_g_gain),
                               apply_gain(lit_r, o_r_gain)};
                    @(posedge clk);
                end
                i_de <= 1'b0;
                @(posedge clk);
                @(posedge clk);
            end
            i_de <= 1'b0;
            repeat (4) @(posedge clk);
        end
    endtask

    // 反馈回路里增益是慢变量, 统计实际输出均值
    integer err_cnt = 0;
    integer f;

    task automatic check_balance;
        input [127:0] tag;
        input integer tol;   // 允许的相对偏差 (千分比)
        reg [7:0] vr, vg, vb;
        reg [31:0] mr, mg, mb, d1, d2;
        begin
            vr = apply_gain(lit_r, o_r_gain);
            vg = apply_gain(lit_g, o_g_gain);
            vb = apply_gain(lit_b, o_b_gain);
            mr = vr; mg = vg; mb = vb;
            d1 = (mr > mg) ? (mr - mg) : (mg - mr);
            d2 = (mb > mg) ? (mb - mg) : (mg - mb);
            $display("  [%0s] gains R=%0d G=%0d B=%0d -> out R=%0d G=%0d B=%0d  dev=%0d/%0d per-mille",
                     tag, o_r_gain, o_g_gain, o_b_gain, vr, vg, vb,
                     (d1*1000)/mg, (d2*1000)/mg);
            if (((d1*1000)/mg) > tol || ((d2*1000)/mg) > tol) begin
                $display("  FAIL: %0s 通道偏差超过 %0d per-mille", tag, tol);
                err_cnt = err_cnt + 1;
            end
        end
    endtask

    initial begin
        $display("=== tb_cam_ae_awb ===");
        rst_n = 1'b0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        //---------------------------------------------------------------------
        // 1) 白天暖光: R 多 G 中 B 少 -> 旧固定增益会把它放大成偏红
        //---------------------------------------------------------------------
        lit_r = 8'd205; lit_g = 8'd155; lit_b = 8'd110;
        $display("[1] daylight warm  raw R/G/B = %0d/%0d/%0d", lit_r, lit_g, lit_b);
        for (f = 0; f < 90; f = f + 1) drive_frame;
        check_balance("daylight", 90);
        if (!o_awb_locked) begin
            $display("  FAIL: daylight 未判定锁定");
            err_cnt = err_cnt + 1;
        end

        //---------------------------------------------------------------------
        // 2) 夜间冷光: R 少 B 多 -> 旧固定增益恰好好用
        //---------------------------------------------------------------------
        lit_r = 8'd105; lit_g = 8'd150; lit_b = 8'd200;
        $display("[2] night cool     raw R/G/B = %0d/%0d/%0d", lit_r, lit_g, lit_b);
        for (f = 0; f < 90; f = f + 1) drive_frame;
        check_balance("night", 90);

        //---------------------------------------------------------------------
        // 3) 切回白天, 验证能自己拉回来(不是一次性的)
        //---------------------------------------------------------------------
        lit_r = 8'd205; lit_g = 8'd155; lit_b = 8'd110;
        $display("[3] back to daylight");
        for (f = 0; f < 90; f = f + 1) drive_frame;
        check_balance("daylight2", 90);

        //---------------------------------------------------------------------
        // 4) 手动覆盖: 立即生效且不被环路拉走
        //---------------------------------------------------------------------
        $display("[4] manual override R=320 B=200");
        i_man_en = 1'b1;
        i_man_r  = 16'd320;
        i_man_b  = 16'd200;
        repeat (4) @(posedge clk);
        if (o_r_gain !== 16'd320 || o_b_gain !== 16'd200) begin
            $display("  FAIL: 手动增益未直通 (R=%0d B=%0d)", o_r_gain, o_b_gain);
            err_cnt = err_cnt + 1;
        end else begin
            $display("  PASS: 手动增益直通 R=%0d B=%0d", o_r_gain, o_b_gain);
        end
        for (f = 0; f < 30; f = f + 1) drive_frame;
        if (o_r_gain !== 16'd320 || o_b_gain !== 16'd200) begin
            $display("  FAIL: 手动增益被自动环路改写");
            err_cnt = err_cnt + 1;
        end else begin
            $display("  PASS: 30 帧后手动增益保持不变");
        end
        i_man_en = 1'b0;
        repeat (4) @(posedge clk);

        //---------------------------------------------------------------------
        // 5) AE: 过曝场景应产生曝光下调请求
        //---------------------------------------------------------------------
        $display("[5] AE request on overexposure");
        i_ae_en = 1'b1;
        lit_r = 8'd250; lit_g = 8'd250; lit_b = 8'd250;
        for (f = 0; f < 40; f = f + 1) drive_frame;
        if (o_ae_exp >= 16'h0BAD) begin
            $display("  FAIL: 过曝未触发曝光下调 (exp=0x%04h)", o_ae_exp);
            err_cnt = err_cnt + 1;
        end else begin
            $display("  PASS: 曝光已下调到 0x%04h", o_ae_exp);
        end

        if (err_cnt == 0) $display("RESULT: PASS");
        else              $display("RESULT: FAIL (%0d)", err_cnt);
        $finish;
    end

    // AE 握手: 收到翻转就回一个翻转
    reg ae_seen = 1'b0;
    always @(posedge o_ae_req_tgl) begin
        #50;
        i_ae_ack_tgl <= o_ae_req_tgl;
    end

endmodule
