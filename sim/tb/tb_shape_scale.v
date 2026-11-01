`timescale 1ns/1ps

// Production-scale shape_proc stress test.
//
// This bypasses edge_proc and feeds an ideal one-pixel ellipse outline with a
// quantized gradient direction.  It targets the part that the small 64x48
// regression cannot cover: an 8-pixel Hough accumulator bin and a 16-pixel
// radius step on a target with a production-sized radius.  The ellipse is
// deliberately between scan radii so the arc verifier must tolerate the
// expected coarse radius/centre error.

module tb_shape_scale;

    localparam integer H = 320;
    localparam integer V = 240;
    localparam integer N = H * V;
    localparam integer CX = 160;
    localparam integer CY = 120;
    localparam integer RX = 78;
    localparam integer RY = 102;
    localparam integer ACC_W = 256;
    localparam integer ACC_H = (V >> 3) + 1;
    localparam integer ACC_SIZE = ACC_W * ACC_H;
    localparam integer VBLANK = 9000;
    localparam integer TAIL = 200;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg i_de = 1'b0;
    reg i_hs = 1'b0;
    reg i_vs = 1'b0;
    reg [10:0] i_row = 11'd0;
    reg i_edge = 1'b0;
    reg [3:0] i_dir16 = 4'd0;
    reg [23:0] i_rgb = 24'd0;

    wire o_de, o_hs, o_vs, o_shape_hit;
    wire [23:0] o_rgb;
    wire [3:0] o_dbg;

    reg edge_map [0:N-1];
    reg [3:0] dir_map [0:N-1];

    shape_proc #(
        .H_ACTIVE          (H),
        .V_ACTIVE          (V),
        .BIN_SHIFT         (3),
        .ACC_W             (ACC_W),
        .RECT_LEN_MIN      (64),
        .RECT_LEN_MAX      (360),
        .RECT_GAP_MIN      (40),
        .RECT_SIDE_MIN     (40),
        .RUN_GAP           (12),
        .TOL_PX            (10),
        .R0                (40),
        .R_STEP            (16),
        .R_NUM             (10),
        .HOUGH_TH          (20),
        .CIRC_RATIO_SHIFT  (4),
        .CIRC_MIN_EDGE     (24),
        .CIRC_CORNER_MAX   (64),
        .CIRC_ARC_TOL      (16),
        .CIRC_ARC_MIN      (6),
        .CIRC_HOLD         (3),
        .RECT_HOLD         (3),
        .TRI_W_MIN         (72),
        .TRI_TOL_PX        (12)
    ) u_shape (
        .clk         (clk),
        .rst_n       (rst_n),
        .i_de        (i_de),
        .i_hs        (i_hs),
        .i_vs        (i_vs),
        .i_rgb       (i_rgb),
        .i_edge      (i_edge),
        .i_dir16     (i_dir16),
        .i_row       (i_row),
        .i_en        (1'b1),
        .i_draw_x0   (12'd0),
        .o_de        (o_de),
        .o_hs        (o_hs),
        .o_vs        (o_vs),
        .o_rgb       (o_rgb),
        .o_shape_hit (o_shape_hit),
        .o_dbg       (o_dbg)
    );

    always #5 clk = ~clk;

    integer errors;
    integer frames_run;
    integer circ_seen;
    integer circ_cx_seen;
    integer circ_cy_seen;
    integer circ_rr_seen;
    integer max_peak_seen;
    integer max_edges_seen;
    integer checker_circ_seen;
    integer checker_peak_seen;
    integer checker_edges_seen;
    integer checker_rect_seen;
    integer checker_tri_seen;
    integer lx, ly;

    always @(negedge clk) begin
        if (rst_n) begin
            if (u_shape.circ_valid) begin
                circ_seen    = 1;
                circ_cx_seen = u_shape.circ_cx;
                circ_cy_seen = u_shape.circ_cy;
                circ_rr_seen = u_shape.circ_rr;
            end
            if (u_shape.peak_val > max_peak_seen)
                max_peak_seen = u_shape.peak_val;
            if (u_shape.frame_edge_count > max_edges_seen)
                max_edges_seen = u_shape.frame_edge_count;
        end
    end

    always @(posedge clk) begin
        if (rst_n && u_shape.rst_frame)
            $display("DBG scale_frame edges=%0d peak=%0d scan=(%0d,%0d) r=%0d/%0d cand=(%0d,%0d) r=%0d/%0d meas=(%0d,%0d)-(%0d,%0d) active=%b corner=%0d arc=%0d/%b diag=%0d circ=%b bbox=(%0d,%0d) r=%0d/%0d",
                     u_shape.frame_edge_count, u_shape.peak_val,
                     u_shape.peak_cx, u_shape.peak_cy,
                     u_shape.cur_r, u_shape.cur_ry,
                     u_shape.circ_cand_cx, u_shape.circ_cand_cy,
                     u_shape.circ_cand_rx, u_shape.circ_cand_ry,
                     u_shape.circ_meas_x0, u_shape.circ_meas_y0,
                     u_shape.circ_meas_x1, u_shape.circ_meas_y1,
                     u_shape.circ_cand_active, u_shape.circ_corner_hits,
                     u_shape.circ_arc_hits, u_shape.circ_arc_mask,
                     u_shape.circ_diag_cnt, u_shape.circ_valid,
                     u_shape.circ_cx, u_shape.circ_cy,
                     u_shape.circ_rr, u_shape.circ_ry);
    end

    task do_reset;
        begin
            rst_n = 1'b0;
            i_de = 1'b0; i_hs = 1'b0; i_vs = 1'b0;
            i_row = 11'd0; i_edge = 1'b0; i_dir16 = 4'd0;
            repeat (16) @(negedge clk);
            rst_n = 1'b1;
            repeat (8) @(negedge clk);
        end
    endtask

    task clear_rams;
        integer k;
        begin
            for (k = 0; k < 1024; k = k + 1) begin
                u_shape.vf_a[k] = 16'd0;
                u_shape.vf_b[k] = 16'd0;
            end
            for (k = 0; k < ACC_SIZE; k = k + 1)
                u_shape.acc_mem[k] = 8'd0;
        end
    endtask

    task mark_edge;
        input integer px;
        input integer py;
        real dxf, dyf, gx, gy, ang;
        integer bnd;
        begin
            dxf = px - CX;
            dyf = py - CY;
            gx = dxf / (RX * RX);
            gy = dyf / (RY * RY);
            ang = $atan2(gy, gx);
            if (ang < 0.0) ang = -ang;
            if (ang > 1.5707963268) ang = 3.1415926536 - ang;
            bnd = $rtoi((ang * 16.0) / 3.1415926536 + 0.5);
            if (bnd < 0) bnd = 0;
            if (bnd > 7) bnd = 7;
            if ((px >= 0) && (px < H) && (py >= 0) && (py < V)) begin
                edge_map[py*H + px] = 1'b1;
                dir_map[py*H + px] = {bnd[2:0], 1'b0};
            end
        end
    endtask

    task build_ellipse;
        integer xx, yy, half, j;
        real dyf, halff;
        begin
            for (yy = 0; yy < V; yy = yy + 1)
                for (xx = 0; xx < H; xx = xx + 1) begin
                    edge_map[yy*H + xx] = 1'b0;
                    dir_map[yy*H + xx] = 4'd0;
                end

            for (yy = CY - RY; yy <= CY + RY; yy = yy + 1) begin
                dyf = (yy - CY);
                halff = RX * $sqrt(1.0 - (dyf * dyf) / (RY * RY));
                half = $rtoi(halff + 0.5);
                for (j = 0; j < 2; j = j + 1) begin
                    mark_edge(CX - half + j, yy);
                    mark_edge(CX + half - j, yy);
                end
            end
        end
    endtask

    // Dense 16-pixel-line texture.  This is intentionally harsher than the
    // low-contrast checker used by the small regression: every line is a
    // binary edge vote, so the circle verifier sees the worst-case clutter
    // produced by a tabletop grid or a checkerboard background.
    task build_checker_texture;
        integer xx, yy;
        begin
            for (yy = 0; yy < V; yy = yy + 1)
                for (xx = 0; xx < H; xx = xx + 1) begin
                    edge_map[yy*H + xx] = 1'b0;
                    dir_map[yy*H + xx] = 4'd0;
                end
            for (yy = 0; yy < V; yy = yy + 1)
                for (xx = 0; xx < H; xx = xx + 1) begin
                    if (((xx % 16) == 0) || ((yy % 16) == 0)) begin
                        edge_map[yy*H + xx] = 1'b1;
                        if ((xx % 16) == 0)
                            dir_map[yy*H + xx] = 4'd0;   // vertical line
                        else
                            dir_map[yy*H + xx] = 4'b1110; // horizontal line
                    end
                end
        end
    endtask

    task run_frame;
        integer r, c;
        begin
            @(negedge clk);
            i_vs = 1'b1; i_de = 1'b0; i_hs = 1'b0;
            @(negedge clk);
            i_vs = 1'b0;
            repeat (VBLANK) @(negedge clk);

            for (r = 0; r < V; r = r + 1) begin
                for (c = 0; c < H; c = c + 1) begin
                    @(negedge clk);
                    i_de   = 1'b1;
                    i_hs   = (c == 0);
                    i_row  = r[10:0];
                    i_edge = edge_map[r*H + c];
                    i_dir16 = dir_map[r*H + c];
                    i_rgb  = i_edge ? 24'hFFFFFF : 24'h000000;
                end
                @(negedge clk);
                i_de = 1'b0; i_hs = 1'b0; i_edge = 1'b0; i_dir16 = 4'd0;
            end
            repeat (TAIL) @(negedge clk);
        end
    endtask

    initial begin
        errors = 0;
        frames_run = 0;
        circ_seen = 0;
        circ_cx_seen = 0;
        circ_cy_seen = 0;
        circ_rr_seen = 0;
        max_peak_seen = 0;
        max_edges_seen = 0;

        // Negative test first: ten radius states over a dense grid texture.
        build_checker_texture;
        clear_rams;
        do_reset;
        circ_seen = 0;
        for (frames_run = 0; frames_run < 12; frames_run = frames_run + 1)
            run_frame;
        checker_circ_seen = circ_seen;
        checker_peak_seen = max_peak_seen;
        checker_edges_seen = max_edges_seen;
        checker_rect_seen = u_shape.rect_valid;
        checker_tri_seen  = u_shape.tri_valid;

        $display("INFO scale_checker: circ=%0d rect=%0d tri=%0d max_peak=%0d max_edges=%0d",
                 checker_circ_seen, checker_rect_seen, checker_tri_seen,
                 checker_peak_seen, checker_edges_seen);
        if (checker_circ_seen) begin
            $display("FAIL scale_checker: grid texture produced a circle lock");
            errors = errors + 1;
        end

        // Positive test: production-radius ellipse between scan radii.
        build_ellipse;
        clear_rams;
        do_reset;
        circ_seen = 0;
        max_peak_seen = 0;
        max_edges_seen = 0;
        frames_run = 0;

        // Ten radius states plus verification and hold frames.  Stop as soon
        // as a result is latched so the regression does not waste time.
        while ((frames_run < 18) && !circ_seen) begin
            run_frame;
            frames_run = frames_run + 1;
        end
        repeat (2) run_frame;

        $display("INFO scale_ellipse: center=(%0d,%0d) r=(%0d,%0d), frames=%0d",
                 CX, CY, RX, RY, frames_run);
        $display("INFO scale_result: seen=%0d center=(%0d,%0d) r=%0d max_peak=%0d max_edges=%0d",
                 circ_seen, circ_cx_seen, circ_cy_seen, circ_rr_seen,
                 max_peak_seen, max_edges_seen);

        if (!circ_seen) begin
            $display("FAIL scale_ellipse: production-radius ellipse was not recognized");
            errors = errors + 1;
        end else begin
            lx = (circ_cx_seen > CX) ? (circ_cx_seen - CX) : (CX - circ_cx_seen);
            ly = (circ_cy_seen > CY) ? (circ_cy_seen - CY) : (CY - circ_cy_seen);
            if ((lx > 12) || (ly > 12)) begin
                $display("FAIL scale_ellipse: centre error %0d,%0d exceeds 12 px", lx, ly);
                errors = errors + 1;
            end
            if ((circ_rr_seen < 64) || (circ_rr_seen > 96)) begin
                $display("FAIL scale_ellipse: radius %0d is not near the 78/102 target", circ_rr_seen);
                errors = errors + 1;
            end
        end

        if (u_shape.rect_valid || u_shape.tri_valid) begin
            $display("FAIL scale_ellipse: ellipse also triggered another shape classifier");
            errors = errors + 1;
        end

        if (errors == 0)
            $display("RESULT: PASS (production-scale ellipse)");
        else
            $display("RESULT: FAIL (production-scale ellipse)");
        $finish;
    end

endmodule
