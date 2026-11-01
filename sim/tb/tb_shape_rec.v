//=============================================================================
// tb_shape_rec.v —— 新识别器 (连通域 + 几何特征 + 决策树) 的功能回归
//
// 直接驱动边缘图: 圆 / 矩形 / 三角形 / 加号(负样本), 检查
//   1) 正确的类别被点亮, 其他类不点亮;
//   2) 外接框与真实外接框偏差在容差内;
//   3) 负样本不产生任何识别结果.
//=============================================================================
`timescale 1ns/1ps

module tb_shape_rec;

    localparam integer H = 640;
    localparam integer V = 360;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;

    reg         i_de = 1'b0;
    reg         i_hs = 1'b0;
    reg         i_vs = 1'b0;
    reg  [23:0] i_rgb = 24'h202020;
    reg         i_edge = 1'b0;
    reg  [3:0]  i_dir16 = 4'd0;
    reg  [10:0] i_row = 11'd0;
    reg         i_en = 1'b1;

    wire        o_de, o_hs, o_vs;
    wire [23:0] o_rgb;
    wire        o_shape_hit;
    wire [3:0]  o_dbg;

    shape_rec #(
        .H_ACTIVE (H),
        .V_ACTIVE (V),
        .MIN_SPAN (24),
        .MAX_SPAN (12000)
    ) dut (
        .clk (clk), .rst_n (rst_n),
        .i_de (i_de), .i_hs (i_hs), .i_vs (i_vs), .i_rgb (i_rgb),
        .i_edge (i_edge), .i_dir16 (i_dir16), .i_row (i_row),
        .i_en (i_en), .i_draw_x0 (12'd0),
        .o_de (o_de), .o_hs (o_hs), .o_vs (o_vs), .o_rgb (o_rgb),
        .o_shape_hit (o_shape_hit), .o_dbg (o_dbg)
    );

    //-------------------------------------------------------------------------
    // 图形定义
    //-------------------------------------------------------------------------
    localparam integer KIND_CIRC = 0;
    localparam integer KIND_RECT = 1;
    localparam integer KIND_TRI  = 2;
    localparam integer KIND_PLUS = 3;
    localparam integer KIND_NONE = 4;

    integer kind = KIND_NONE;
    integer cx = 320, cy = 180, rr = 60;
    integer rw = 120, rh = 100;

    function integer seg_dist2;
        input integer px, py, ax, ay, bx, by;
        integer dx, dy, t, nx, ny, ddx, ddy;
        begin
            dx = bx - ax; dy = by - ay;
            t  = ((px - ax) * dx + (py - ay) * dy);
            if (t <= 0) begin
                ddx = px - ax; ddy = py - ay;
            end else if (t >= dx * dx + dy * dy) begin
                ddx = px - bx; ddy = py - by;
            end else begin
                nx = ax + (t * dx) / (dx * dx + dy * dy + 1);
                ny = ay + (t * dy) / (dx * dx + dy * dy + 1);
                ddx = px - nx; ddy = py - ny;
            end
            seg_dist2 = ddx * ddx + ddy * ddy;
        end
    endfunction

    function is_edge_px;
        input integer x, y;
        integer d2, ax, ay, bx, by, t;
        begin
            is_edge_px = 1'b0;
            case (kind)
            KIND_CIRC: begin
                d2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
                is_edge_px = (d2 >= (rr - 1) * (rr - 1)) && (d2 <= (rr + 1) * (rr + 1));
            end
            KIND_RECT: begin
                is_edge_px = (x >= cx - rw / 2) && (x <= cx + rw / 2) &&
                             (y >= cy - rh / 2) && (y <= cy + rh / 2) &&
                             (((x <= cx - rw / 2 + 1) || (x >= cx + rw / 2 - 1)) ||
                              ((y <= cy - rh / 2 + 1) || (y >= cy + rh / 2 - 1)));
            end
            KIND_TRI: begin
                // 顶点 (cx, cy-rh/2), 底边两端
                ax = cx;        ay = cy - rh / 2;
                bx = cx - rw / 2; by = cy + rh / 2;
                is_edge_px = (seg_dist2(x, y, ax, ay, bx, by) < 4) ||
                             (seg_dist2(x, y, bx, by, cx + rw / 2, by) < 4) ||
                             (seg_dist2(x, y, cx + rw / 2, by, ax, ay) < 4);
            end
            KIND_PLUS: begin
                is_edge_px = ((x >= cx - rr) && (x <= cx + rr) && (y >= cy - 6) && (y <= cy + 6)) ||
                             ((y >= cy - rr) && (y <= cy + rr) && (x >= cx - 6) && (x <= cx + 6));
            end
            default: is_edge_px = 1'b0;
            endcase
        end
    endfunction

    //-------------------------------------------------------------------------
    task automatic drive_frame;
        integer x, y;
        begin
            i_vs <= 1'b1;
            @(posedge clk);
            i_vs <= 1'b0;
            for (y = 0; y < V; y = y + 1) begin
                i_row <= y[10:0];
                for (x = 0; x < H; x = x + 1) begin
                    i_de   <= 1'b1;
                    i_edge <= is_edge_px(x, y);
                    @(posedge clk);
                end
                i_de <= 1'b0;
                @(posedge clk);
            end
            repeat (4000) @(posedge clk);   // 等帧末判决完成
        end
    endtask

    integer errs = 0;
    integer f;

    task automatic check_case;
        input integer k;
        input integer want;          // 1=圆 2=矩形 3=三角 0=无
        input integer ex0, ey0, ex1, ey1;
        input integer tol;
        reg got;
        begin
            kind = k;
            for (f = 0; f < 3; f = f + 1) drive_frame;
            got = 0;
            $display("  case kind=%0d  circ=%b rect=%b tri=%b", k,
                     dut.v_circ, dut.v_rect, dut.v_tri);
            if (want == 1) begin
                if (!dut.v_circ) begin
                    $display("  FAIL: 未识别出圆"); errs = errs + 1;
                end else begin
                    $display("    circle box (%0d,%0d)-(%0d,%0d)  expect (%0d,%0d)-(%0d,%0d)",
                             dut.bx0[0]*4, dut.by0[0]*4, dut.bx1[0]*4, dut.by1[0]*4,
                             ex0, ey0, ex1, ey1);
                    if ((dut.bx0[0]*4 > ex0 + tol) || (dut.bx0[0]*4 < ex0 - tol) ||
                        (dut.by0[0]*4 > ey0 + tol) || (dut.by0[0]*4 < ey0 - tol) ||
                        (dut.bx1[0]*4 > ex1 + tol) || (dut.bx1[0]*4 < ex1 - tol) ||
                        (dut.by1[0]*4 > ey1 + tol) || (dut.by1[0]*4 < ey1 - tol)) begin
                        $display("  FAIL: 圆外接框超差"); errs = errs + 1;
                    end
                end
                if (dut.v_rect || dut.v_tri) begin
                    $display("  FAIL: 圆同时点亮了其他类"); errs = errs + 1;
                end
            end else if (want == 2) begin
                if (!dut.v_rect) begin
                    $display("  FAIL: 未识别出矩形"); errs = errs + 1;
                end else begin
                    $display("    rect box (%0d,%0d)-(%0d,%0d)  expect (%0d,%0d)-(%0d,%0d)",
                             dut.bx0[1]*4, dut.by0[1]*4, dut.bx1[1]*4, dut.by1[1]*4,
                             ex0, ey0, ex1, ey1);
                    if ((dut.bx0[1]*4 > ex0 + tol) || (dut.bx0[1]*4 < ex0 - tol) ||
                        (dut.by0[1]*4 > ey0 + tol) || (dut.by0[1]*4 < ey0 - tol) ||
                        (dut.bx1[1]*4 > ex1 + tol) || (dut.bx1[1]*4 < ex1 - tol) ||
                        (dut.by1[1]*4 > ey1 + tol) || (dut.by1[1]*4 < ey1 - tol)) begin
                        $display("  FAIL: 矩形外接框超差"); errs = errs + 1;
                    end
                end
                if (dut.v_circ || dut.v_tri) begin
                    $display("  FAIL: 矩形同时点亮了其他类"); errs = errs + 1;
                end
            end else if (want == 3) begin
                if (!dut.v_tri) begin
                    $display("  FAIL: 未识别出三角形"); errs = errs + 1;
                end else begin
                    $display("    tri box (%0d,%0d)-(%0d,%0d)  expect (%0d,%0d)-(%0d,%0d)",
                             dut.bx0[2]*4, dut.by0[2]*4, dut.bx1[2]*4, dut.by1[2]*4,
                             ex0, ey0, ex1, ey1);
                    if ((dut.bx0[2]*4 > ex0 + tol) || (dut.bx0[2]*4 < ex0 - tol) ||
                        (dut.by0[2]*4 > ey0 + tol) || (dut.by0[2]*4 < ey0 - tol) ||
                        (dut.bx1[2]*4 > ex1 + tol) || (dut.bx1[2]*4 < ex1 - tol) ||
                        (dut.by1[2]*4 > ey1 + tol) || (dut.by1[2]*4 < ey1 - tol)) begin
                        $display("  FAIL: 三角形外接框超差"); errs = errs + 1;
                    end
                end
                if (dut.v_circ || dut.v_rect) begin
                    $display("  FAIL: 三角形同时点亮了其他类"); errs = errs + 1;
                end
            end else begin
                if (dut.v_circ || dut.v_rect || dut.v_tri) begin
                    $display("  FAIL: 负样本被误识别"); errs = errs + 1;
                end else begin
                    $display("    PASS: 负样本无识别结果");
                end
            end
        end
    endtask

    initial begin
        $display("=== tb_shape_rec ===");
        rst_n = 1'b0;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        check_case(KIND_CIRC, 1, cx - rr - 6, cy - rr - 6, cx + rr + 6, cy + rr + 6, 24);
        check_case(KIND_RECT, 2, cx - rw/2 - 6, cy - rh/2 - 6, cx + rw/2 + 6, cy + rh/2 + 6, 24);
        check_case(KIND_TRI,  3, cx - rw/2 - 6, cy - rh/2 - 6, cx + rw/2 + 6, cy + rh/2 + 6, 28);
        check_case(KIND_PLUS, 0, 0, 0, 0, 0, 0);

        if (errs == 0) $display("RESULT: PASS");
        else           $display("RESULT: FAIL (%0d)", errs);
        $finish;
    end

endmodule
