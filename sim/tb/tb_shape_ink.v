//==============================================================================
// tb_shape_ink.v -- 墨迹分割识别器回归
//   合成一张 640x360 画面, 依次画圆/矩形/三角形/空白, 检查分类结果
//==============================================================================
`timescale 1ns/1ps

module tb_shape_ink;

    localparam integer H = 640;
    localparam integer V = 360;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg        i_de = 0, i_hs = 0, i_vs = 0;
    reg [23:0] i_rgb = 24'hFFFFFF;
    reg [10:0] i_row = 0;
    reg        i_en = 1;
    reg [7:0]  i_thr = 8'd0;

    wire        o_de, o_hs, o_vs, o_shape_hit;
    wire [23:0] o_rgb;
    wire [3:0]  o_dbg;

    shape_ink #(
        .H_ACTIVE (H),
        .V_ACTIVE (V),
        .LAB      (32),
        .MIN_SPAN (200),
        .MIN_WH   (8),
        .THR_DEF  (18)
    ) dut (
        .clk (clk), .rst_n (rst_n),
        .i_de (i_de), .i_hs (i_hs), .i_vs (i_vs),
        .i_rgb (i_rgb), .i_edge (1'b0), .i_dir16 (4'd0), .i_row (i_row),
        .i_en (i_en), .i_draw_x0 (12'd0), .i_thr (i_thr),
        .o_de (o_de), .o_hs (o_hs), .o_vs (o_vs), .o_rgb (o_rgb),
        .o_shape_hit (o_shape_hit), .o_dbg (o_dbg)
    );

    // ---- 画面内容 ----
    localparam integer KIND_CIRCLE = 0, KIND_RECT = 1, KIND_TRI = 2, KIND_EMPTY = 3;
    integer kind = KIND_CIRCLE;

    localparam integer CX = 320, CY = 180, RR = 90;

    function integer in_shape;
        input integer x, y;
        input integer k;
        integer dx, dy, d2;
        begin
            in_shape = 0;
            case (k)
            KIND_CIRCLE: begin
                dx = x - CX; dy = y - CY; d2 = dx*dx + dy*dy;
                in_shape = (d2 >= (RR-6)*(RR-6)) && (d2 <= (RR+6)*(RR+6));
            end
            KIND_RECT: begin
                in_shape = (x >= CX-RR) && (x <= CX+RR) && (y >= CY-RR) && (y <= CY+RR) &&
                           ((x <= CX-RR+8) || (x >= CX+RR-8) ||
                            (y <= CY-RR+8) || (y >= CY+RR-8));
            end
            KIND_TRI: begin
                // 顶点 (CX, CY-RR), 底边 y=CY+RR, 半宽 RR
                dy = y - (CY - RR);
                if (dy < 0) in_shape = 0;
                else in_shape = (dy <= 2*RR) &&
                                (x >= CX - dy) && (x <= CX + dy) &&
                                ((dy <= 10) || (dy >= 2*RR - 10) ||
                                 (x <= CX - dy + 10) || (x >= CX + dy - 10));
            end
            default: in_shape = 0;
            endcase
        end
    endfunction

    integer pass_cnt = 0, fail_cnt = 0;
    reg [2:0] first_cls = 3'd0;
    integer got_circle = 0, got_rect = 0, got_tri = 0, got_extra = 0;

    task run_frame;
        input integer k;
        integer x, y;
        begin
            kind = k;
            @(posedge clk); i_vs <= 1; @(posedge clk); i_vs <= 0;
            for (y = 0; y < V; y = y + 1) begin
                i_row <= y[10:0];
                for (x = 0; x < H; x = x + 1) begin
                    i_de  <= 1;
                    i_rgb <= in_shape(x, y, k) ? 24'h000000 : 24'hF0F0F0;
                    @(posedge clk);
                end
                i_de <= 0;
                @(posedge clk);
            end
            repeat(4000) @(posedge clk);
        end
    endtask

    task report;
        input integer k;
        integer i, n;
        begin
            n = 0; first_cls = 3'd0;
            $write("  frame kind=%0d :", k);
            for (i = 0; i < 32; i = i + 1) begin
                if (dut.bcls[i] != 3'd0) begin
                    $write(" L%0d cls=%0d fill-ish span=%0d h=%0d ms=%0d box=(%0d,%0d)-(%0d,%0d)",
                           i, dut.bcls[i], dut.L_span[i],
                           dut.L_y1[i]-dut.L_y0[i]+1, dut.L_ms[i],
                           dut.L_x0[i], dut.L_y0[i], dut.L_x1[i], dut.L_y1[i]);
                    if (first_cls == 3'd0) first_cls = dut.bcls[i];
                    n = n + 1;
                end
            end
            $write("\n");
            // 判定
            case (k)
            KIND_CIRCLE: if (n == 1 && first_cls == 3'd1) begin
                             $display("  -> 圆 PASS"); pass_cnt = pass_cnt + 1;
                         end else begin
                             $display("  -> 圆 FAIL (n=%0d cls=%0d)", n, first_cls); fail_cnt = fail_cnt + 1;
                         end
            KIND_RECT:   if (n == 1 && first_cls == 3'd2) begin
                             $display("  -> 矩形 PASS"); pass_cnt = pass_cnt + 1;
                         end else begin
                             $display("  -> 矩形 FAIL (n=%0d cls=%0d)", n, first_cls); fail_cnt = fail_cnt + 1;
                         end
            KIND_TRI:    if (n == 1 && first_cls == 3'd3) begin
                             $display("  -> 三角形 PASS"); pass_cnt = pass_cnt + 1;
                         end else begin
                             $display("  -> 三角形 FAIL (n=%0d cls=%0d)", n, first_cls); fail_cnt = fail_cnt + 1;
                         end
            default:     if (n == 0) begin
                             $display("  -> 空白 PASS (无误检)"); pass_cnt = pass_cnt + 1;
                         end else begin
                             $display("  -> 空白 FAIL (误检 %0d 个)", n); fail_cnt = fail_cnt + 1;
                         end
            endcase
        end
    endtask

    initial begin
        repeat(20) @(posedge clk);
        rst_n = 1;
        repeat(10) @(posedge clk);

        run_frame(KIND_CIRCLE); report(KIND_CIRCLE);
        run_frame(KIND_RECT);   report(KIND_RECT);
        run_frame(KIND_TRI);    report(KIND_TRI);
        run_frame(KIND_EMPTY);  report(KIND_EMPTY);

        $display("\n==== shape_ink: %0d pass, %0d fail ====", pass_cnt, fail_cnt);
        if (fail_cnt == 0) $display("RESULT: ALL PASS");
        else               $display("RESULT: FAIL");
        $finish;
    end

    initial begin
        #400_000_000;
        $display("RESULT: TIMEOUT");
        $finish;
    end

endmodule
