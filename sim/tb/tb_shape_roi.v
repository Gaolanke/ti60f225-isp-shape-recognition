`timescale 1ns/1ps

// Production-width ROI regression for shape_proc.
//
// The older 320x240 test uses ROI=0.  Mode 4 on the board uses a 1920-wide
// video stream and enables recognition only for x>=960.  This test exercises
// that exact column split with absolute coordinates, so an accidental use of
// a display-relative x coordinate cannot pass unnoticed.

module tb_shape_roi;

    localparam integer H = 1920;
    localparam integer V = 240;
    localparam integer N = H * V;
    localparam [11:0] ROI_X = 12'd960;
    localparam integer ACC_W = 256;
    localparam integer ACC_H = (V >> 3) + 1;
    localparam integer ACC_SIZE = ACC_W * ACC_H;
    localparam integer VBLANK = 10000;
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
    reg [11:0] shape_roi_x = ROI_X;
    reg compose_no_clear = 1'b0;
    reg target_sheet_mode = 1'b0;

    // External-image stimulus (sim/tools/make_stimulus.py).  Edge/direction
    // maps cover only the recognition half (ROI_X..H-1) as packed hex words.
    localparam integer IMG_W = H - ROI_X;
    localparam integer IMG_WORDS = (IMG_W * V) / 8;
    reg  [31:0] img_edge_words [0:IMG_WORDS-1];
    reg  [31:0] img_dir_words  [0:IMG_WORDS-1];
    reg         img_mode = 1'b0;
    reg [8*240-1:0] img_prefix;
    reg [8*250-1:0] img_edge_file, img_dir_file;
    integer     img_asp = -1;
    integer     img_frames = 12;
    integer     img_expect = 0;      // bit0 circle, bit1 rect, bit2 triangle
    integer     img_idx;

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
        .i_draw_x0   (shape_roi_x),
        .o_de        (o_de),
        .o_hs        (o_hs),
        .o_vs        (o_vs),
        .o_rgb       (o_rgb),
        .o_shape_hit (o_shape_hit),
        .o_dbg       (o_dbg)
    );

    always #5 clk = ~clk;

    // Focused-scan helper: force the ellipse aspect hypothesis so one image
    // test only has to walk the ten radius steps instead of all fifty
    // (radius x aspect) combinations.  Assign after the DUT's posedge update
    // so the forced value is the one used for the coming frame.
    always @(negedge clk) begin
        if (img_mode && (img_asp >= 0))
            u_shape.frame_asp = img_asp[2:0];
    end

    integer errors;
    integer frame_no;
    integer circ_seen;
    integer rect_seen;
    integer tri_seen;
    integer tri_h_seen;
    integer tri_rot_seen;
    integer circ_cx_seen;
    integer circ_cy_seen;
    integer circ_rr_seen;
    integer circ_ry_seen;
    integer rect_x0_seen;
    integer rect_y0_seen;
    integer rect_x1_seen;
    integer rect_y1_seen;
    integer tri_x0_seen;
    integer tri_y0_seen;
    integer tri_x1_seen;
    integer tri_y1_seen;
    integer max_edges_seen;
    integer max_peak_seen;
    integer circ_far_seen;
    integer dbg_push_cnt;
    integer dbg_pop_cnt;
    integer dbg_drop_cnt;
    integer dbg_max_used_a;
    integer dbg_max_used_b;
    integer hacc_sum;
    integer hacc_best0, hacc_best1, hacc_best2, hacc_best3, hacc_best4;
    integer hacc_addr0, hacc_addr1, hacc_addr2, hacc_addr3, hacc_addr4;
    integer hacc_j;

    always @(posedge clk) begin
        if (rst_n) begin
            if (u_shape.push1) dbg_push_cnt = dbg_push_cnt + 1;
            if (u_shape.push2) dbg_push_cnt = dbg_push_cnt + 1;
            if (u_shape.pop_now) dbg_pop_cnt = dbg_pop_cnt + 1;
            if (u_shape.drop1 || u_shape.drop2) dbg_drop_cnt = dbg_drop_cnt + 1;
            if (u_shape.used_a > dbg_max_used_a) dbg_max_used_a = u_shape.used_a;
            if (u_shape.used_b > dbg_max_used_b) dbg_max_used_b = u_shape.used_b;
        end
    end

    always @(posedge clk) begin
        if (rst_n && u_shape.p_valid &&
            (((u_shape.p_col == 12'd295) && (u_shape.p_row == 12'd200)) ||
             ((u_shape.p_col == 12'd420) && (u_shape.p_row == 12'd70))))
            $display("HACC_PT col=%0d row=%0d d=%b band=%0d quad=%b uv=%0d/%0d dx=%0d dy=%0d a1=(%0d,%0d) a2=(%0d,%0d) ok=%b%b",
                     u_shape.p_col, u_shape.p_row, u_shape.d_dir,
                     u_shape.v_band, u_shape.v_quad,
                     u_shape.par_x, u_shape.par_y, u_shape.dxv, u_shape.dyv,
                     u_shape.a1x, u_shape.a1y, u_shape.a2x, u_shape.a2y,
                     u_shape.ok1, u_shape.ok2);
    end

    function integer abs_int;
        input integer value;
        begin
            abs_int = (value < 0) ? -value : value;
        end
    endfunction

    always @(negedge clk) begin
        if (rst_n) begin
            if (u_shape.circ_valid) begin
                circ_seen = 1;
                circ_cx_seen = u_shape.circ_cx;
                circ_cy_seen = u_shape.circ_cy;
                circ_rr_seen = u_shape.circ_rr;
                circ_ry_seen = u_shape.circ_ry;
                if (target_sheet_mode && !circ_far_seen &&
                    ((abs_int(u_shape.circ_cx - 420) > 36) ||
                     (abs_int(u_shape.circ_cy - 150) > 36))) begin
                    $display("FAIL target_sheet: far circle at frame=%0d center=(%0d,%0d) r=%0d/%0d", frame_no,
                             u_shape.circ_cx, u_shape.circ_cy,
                             u_shape.circ_rr, u_shape.circ_ry);
                    circ_far_seen = 1;
                end
            end
            if (u_shape.rect_valid) begin
                rect_seen = 1;
                rect_x0_seen = u_shape.rect_x0;
                rect_y0_seen = u_shape.rect_y0;
                rect_x1_seen = u_shape.rect_x1;
                rect_y1_seen = u_shape.rect_y1;
            end
            if (u_shape.tri_valid) begin
                tri_seen = 1;
                tri_h_seen = 1;
                tri_x0_seen = u_shape.tri_x0;
                tri_y0_seen = u_shape.tri_y0;
                tri_x1_seen = u_shape.tri_x1;
                tri_y1_seen = u_shape.tri_y1;
            end
            if (u_shape.tri_rot_valid) begin
                tri_seen = 1;
                tri_rot_seen = 1;
                tri_x0_seen = u_shape.tri_rot_x0;
                tri_y0_seen = u_shape.tri_rot_y0;
                tri_x1_seen = u_shape.tri_rot_x1;
                tri_y1_seen = u_shape.tri_rot_y1;
            end
            if (u_shape.frame_edge_count > max_edges_seen)
                max_edges_seen = u_shape.frame_edge_count;
            if (u_shape.peak_val > max_peak_seen)
                max_peak_seen = u_shape.peak_val;
        end
    end

    always @(posedge clk) begin
        if (rst_n && u_shape.rst_frame)
            $display("DBG roi_frame=%0d edges=%0d peak=%0d cand=(%0d,%0d) r=%0d/%0d active=%b corner=%0d arc=%0d/%b circ=%b rect=%b tri=%b",
                     frame_no, u_shape.frame_edge_count, u_shape.peak_val,
                     u_shape.peak_cx, u_shape.peak_cy,
                     u_shape.cur_r, u_shape.cur_ry,
                     u_shape.circ_cand_active, u_shape.circ_corner_hits,
                     u_shape.circ_arc_hits, u_shape.circ_arc_mask,
                     u_shape.circ_valid, u_shape.rect_valid, u_shape.tri_valid);
        if (rst_n && u_shape.rst_frame)
            $display("DBG2 h1=%b/%0d/(%0d,%0d)-(%0d,%0d) h2=%b/%0d/(%0d,%0d)-(%0d,%0d) h3=%b/%0d/(%0d,%0d)-(%0d,%0d) nextcand=%b/(%0d,%0d)-(%0d,%0d) v=%0d/%0d h=%0d/%0d vok=%b hok=%b",
                     u_shape.h1_v, u_shape.h1_len, u_shape.h1_x0, u_shape.h1_y, u_shape.h1_x1, u_shape.h1_y,
                     u_shape.h2_v, u_shape.h2_len, u_shape.h2_x0, u_shape.h2_y, u_shape.h2_x1, u_shape.h2_y,
                     u_shape.h3_v, u_shape.h3_len, u_shape.h3_x0, u_shape.h3_y, u_shape.h3_x1, u_shape.h3_y,
                     u_shape.new_cand, u_shape.ov_x0, u_shape.ov_y0, u_shape.ov_x1, u_shape.ov_y1,
                     u_shape.vl_cnt, u_shape.vr_cnt, u_shape.hh_cnt0, u_shape.hh_cnt1,
                     u_shape.vert_ok_r, u_shape.horz_ok_r);
        if (rst_n && u_shape.rst_frame)
            $display("DBG3 vhist c=%0d/%0d/%0d bin=%0d/%0d/%0d valid=%b%b%b meas_on=%b slot=%0d y0=%0d/%0d/%0d y1=%0d/%0d/%0d rcand=%b rneed=%0d hits=%0d/%0d rvalid=%b",
                     u_shape.vrank_c1, u_shape.vrank_c2, u_shape.vrank_c3,
                     u_shape.vrank_i1, u_shape.vrank_i2, u_shape.vrank_i3,
                     u_shape.vb_meas_v0, u_shape.vb_meas_v1, u_shape.vb_meas_v2,
                     u_shape.vb_meas_on, u_shape.vb_slot,
                     u_shape.vby0_0, u_shape.vby0_1, u_shape.vby0_2,
                     u_shape.vby1_0, u_shape.vby1_1, u_shape.vby1_2,
                     u_shape.tri_rot_cand_active, u_shape.tri_rot_need,
                     u_shape.tri_rot_r_hits, u_shape.tri_rot_l_hits,
                     u_shape.tri_rot_valid);
        if (rst_n && u_shape.rst_frame && $test$plusargs("hacc_debug"))
            $display("DBG4 hacc true=(420,150)=%0d (412,156)=%0d (428,148)=%0d false=(1204,124)=%0d drop=%b",
                     u_shape.acc_mem[(150 >> 3) * ACC_W + (420 >> 3)],
                     u_shape.acc_mem[(156 >> 3) * ACC_W + (412 >> 3)],
                     u_shape.acc_mem[(148 >> 3) * ACC_W + (428 >> 3)],
                     u_shape.acc_mem[(124 >> 3) * ACC_W + (1204 >> 3)],
                     u_shape.fifo_drop);
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

    task clear_map;
        integer k;
        begin
            for (k = 0; k < N; k = k + 1) begin
                edge_map[k] = 1'b0;
                dir_map[k] = 4'd0;
            end
        end
    endtask

    task mark_edge;
        input integer px;
        input integer py;
        input integer cx;
        input integer cy;
        input integer rx;
        input integer ry;
        real dxf, dyf, gx, gy, ang;
        integer bnd;
        reg quad;
        begin
            if ((px >= 0) && (px < H) && (py >= 0) && (py < V)) begin
                dxf = px - cx;
                dyf = py - cy;
                gx = dxf / (rx * rx);
                gy = dyf / (ry * ry);
                // Match the production direction code: band8 is the
                // unsigned 0..90-degree slope magnitude, while quad is the
                // XOR of the signed gradient components.  Folding the angle
                // without preserving that parity sends half of an ellipse to
                // the wrong Hough centre and splits one ellipse into several
                // off-centre peaks.
                ang = $atan2((gy < 0.0) ? -gy : gy,
                             (gx < 0.0) ? -gx : gx);
                quad = 1'b0;
                if (((gx < 0.0) && (gy > 0.0)) ||
                    ((gx > 0.0) && (gy < 0.0)))
                    quad = 1'b1;
                bnd = $rtoi((ang * 16.0) / 3.1415926536 + 0.5);
                if (bnd < 0) bnd = 0;
                if (bnd > 7) bnd = 7;
                edge_map[py*H + px] = 1'b1;
                dir_map[py*H + px] = {bnd[2:0], quad};
            end
        end
    endtask

    task build_ellipse;
        input integer cx;
        input integer cy;
        input integer rx;
        input integer ry;
        integer xx, yy, half, j;
        real dyf, halff;
        begin
            if (!compose_no_clear) clear_map;
            for (yy = cy - ry; yy <= cy + ry; yy = yy + 1) begin
                dyf = (yy - cy);
                halff = rx * $sqrt(1.0 - (dyf * dyf) / (ry * ry));
                half = $rtoi(halff + 0.5);
                for (j = 0; j < 2; j = j + 1) begin
                    mark_edge(cx - half + j, yy, cx, cy, rx, ry);
                    mark_edge(cx + half - j, yy, cx, cy, rx, ry);
                end
            end
        end
    endtask

    task build_rect;
        input integer x0;
        input integer y0;
        input integer x1;
        input integer y1;
        integer xx, yy, j;
        begin
            if (!compose_no_clear) clear_map;
            for (j = 0; j < 2; j = j + 1) begin
                for (xx = x0; xx <= x1; xx = xx + 1) begin
                    if ((xx >= 0) && (xx < H) && ((y0+j) >= 0) && ((y0+j) < V)) begin
                        edge_map[(y0+j)*H + xx] = 1'b1;
                        dir_map[(y0+j)*H + xx] = 4'b1110;
                    end
                    if ((xx >= 0) && (xx < H) && ((y1-j) >= 0) && ((y1-j) < V)) begin
                        edge_map[(y1-j)*H + xx] = 1'b1;
                        dir_map[(y1-j)*H + xx] = 4'b1110;
                    end
                end
                for (yy = y0; yy <= y1; yy = yy + 1) begin
                    if (((x0+j) >= 0) && ((x0+j) < H) && (yy >= 0) && (yy < V)) begin
                        edge_map[yy*H + x0+j] = 1'b1;
                        dir_map[yy*H + x0+j] = 4'd0;
                    end
                    if (((x1-j) >= 0) && ((x1-j) < H) && (yy >= 0) && (yy < V)) begin
                        edge_map[yy*H + x1-j] = 1'b1;
                        dir_map[yy*H + x1-j] = 4'd0;
                    end
                end
            end
        end
    endtask

    task build_triangle;
        input integer apex_x;
        input integer apex_y;
        input integer base_y;
        input integer half_w;
        integer height, yy, xl, xr, j;
        begin
            if (!compose_no_clear) clear_map;
            height = base_y - apex_y;
            for (yy = apex_y; yy <= base_y; yy = yy + 1) begin
                xl = apex_x - ((yy - apex_y) * half_w) / height;
                xr = apex_x + ((yy - apex_y) * half_w) / height;
                for (j = 0; j < 2; j = j + 1) begin
                    if ((yy >= 0) && (yy < V)) begin
                        if ((xl+j >= 0) && (xl+j < H)) begin
                            edge_map[yy*H + xl+j] = 1'b1;
                            dir_map[yy*H + xl+j] = 4'b0100;
                        end
                        if ((xr-j >= 0) && (xr-j < H)) begin
                            edge_map[yy*H + xr-j] = 1'b1;
                            dir_map[yy*H + xr-j] = 4'b1100;
                        end
                    end
                end
            end
            for (j = 0; j < 2; j = j + 1) begin
                for (xl = apex_x-half_w; xl <= apex_x+half_w; xl = xl + 1) begin
                    if ((xl >= 0) && (xl < H) && ((base_y-j) >= 0) && ((base_y-j) < V)) begin
                        edge_map[(base_y-j)*H + xl] = 1'b1;
                        dir_map[(base_y-j)*H + xl] = 4'b1110;
                    end
                end
            end
        end
    endtask

    // Triangle rotated 90 degrees: vertical base on the left, apex on the
    // right.  This is the orientation printed on the reference target used by
    // the board camera, and is deliberately not the same as build_triangle.
    task build_triangle_right;
        input integer base_x;
        input integer base_y0;
        input integer base_y1;
        input integer apex_x;
        integer yy, xx, yc, j;
        begin
            if (!compose_no_clear) clear_map;
            yc = (base_y0 + base_y1) / 2;

            for (yy = base_y0; yy <= base_y1; yy = yy + 1) begin
                for (j = 0; j < 2; j = j + 1) begin
                    if (((base_x+j) >= 0) && ((base_x+j) < H) &&
                        (yy >= 0) && (yy < V)) begin
                        edge_map[yy*H + base_x+j] = 1'b1;
                        dir_map[yy*H + base_x+j] = 4'b0000;
                    end
                end
            end

            for (yy = base_y0; yy <= yc; yy = yy + 1) begin
                xx = base_x + ((yy - base_y0) * (apex_x - base_x)) /
                              (yc - base_y0);
                for (j = 0; j < 2; j = j + 1) begin
                    if (((xx+j) >= 0) && ((xx+j) < H) &&
                        (yy >= 0) && (yy < V)) begin
                        edge_map[yy*H + xx+j] = 1'b1;
                        dir_map[yy*H + xx+j] = 4'b0010;
                    end
                end
            end

            for (yy = yc; yy <= base_y1; yy = yy + 1) begin
                xx = apex_x - ((yy - yc) * (apex_x - base_x)) /
                              (base_y1 - yc);
                for (j = 0; j < 2; j = j + 1) begin
                    if (((xx-j) >= 0) && ((xx-j) < H) &&
                        (yy >= 0) && (yy < V)) begin
                        edge_map[yy*H + xx-j] = 1'b1;
                        dir_map[yy*H + xx-j] = 4'b0110;
                    end
                end
            end
        end
    endtask

    // Plus sign from the target sheet.  It is a negative sample: none of the
    // circle/rectangle/triangle classifiers may fire on it.
    task build_plus;
        input integer cx;
        input integer cy;
        input integer arm_x;
        input integer arm_y;
        input integer half_t;
        integer k, j, x, y;
        begin
            if (!compose_no_clear) clear_map;

            for (j = 0; j < 2; j = j + 1) begin
                // Vertical outer edges of the centre bar.
                for (y = cy - arm_y; y <= cy - half_t; y = y + 1) begin
                    if (((cx-half_t+j) >= 0) && ((cx-half_t+j) < H) &&
                        (y >= 0) && (y < V)) begin
                        edge_map[y*H + cx-half_t+j] = 1'b1;
                        dir_map[y*H + cx-half_t+j] = 4'b0000;
                    end
                    if (((cx+half_t-j) >= 0) && ((cx+half_t-j) < H) &&
                        (y >= 0) && (y < V)) begin
                        edge_map[y*H + cx+half_t-j] = 1'b1;
                        dir_map[y*H + cx+half_t-j] = 4'b0000;
                    end
                end
                for (y = cy + half_t; y <= cy + arm_y; y = y + 1) begin
                    if (((cx-half_t+j) >= 0) && ((cx-half_t+j) < H) &&
                        (y >= 0) && (y < V)) begin
                        edge_map[y*H + cx-half_t+j] = 1'b1;
                        dir_map[y*H + cx-half_t+j] = 4'b0000;
                    end
                    if (((cx+half_t-j) >= 0) && ((cx+half_t-j) < H) &&
                        (y >= 0) && (y < V)) begin
                        edge_map[y*H + cx+half_t-j] = 1'b1;
                        dir_map[y*H + cx+half_t-j] = 4'b0000;
                    end
                end

                // Vertical ends of the two horizontal arms.
                for (y = cy - half_t; y <= cy + half_t; y = y + 1) begin
                    if (((cx-arm_x+j) >= 0) && ((cx-arm_x+j) < H) &&
                        (y >= 0) && (y < V)) begin
                        edge_map[y*H + cx-arm_x+j] = 1'b1;
                        dir_map[y*H + cx-arm_x+j] = 4'b0000;
                    end
                    if (((cx+arm_x-j) >= 0) && ((cx+arm_x-j) < H) &&
                        (y >= 0) && (y < V)) begin
                        edge_map[y*H + cx+arm_x-j] = 1'b1;
                        dir_map[y*H + cx+arm_x-j] = 4'b0000;
                    end
                end

                // Horizontal silhouette edges.
                for (x = cx-half_t; x <= cx+half_t; x = x + 1) begin
                    if ((x >= 0) && (x < H) &&
                        ((cy-arm_y+j) >= 0) && ((cy-arm_y+j) < V)) begin
                        edge_map[(cy-arm_y+j)*H + x] = 1'b1;
                        dir_map[(cy-arm_y+j)*H + x] = 4'b1110;
                    end
                    if ((x >= 0) && (x < H) &&
                        ((cy+arm_y-j) >= 0) && ((cy+arm_y-j) < V)) begin
                        edge_map[(cy+arm_y-j)*H + x] = 1'b1;
                        dir_map[(cy+arm_y-j)*H + x] = 4'b1110;
                    end
                end
                for (x = cx-arm_x; x <= cx-half_t; x = x + 1) begin
                    if ((x >= 0) && (x < H) &&
                        ((cy-half_t+j) >= 0) && ((cy-half_t+j) < V)) begin
                        edge_map[(cy-half_t+j)*H + x] = 1'b1;
                        dir_map[(cy-half_t+j)*H + x] = 4'b1110;
                    end
                    if ((x >= 0) && (x < H) &&
                        ((cy+half_t-j) >= 0) && ((cy+half_t-j) < V)) begin
                        edge_map[(cy+half_t-j)*H + x] = 1'b1;
                        dir_map[(cy+half_t-j)*H + x] = 4'b1110;
                    end
                end
                for (x = cx+half_t; x <= cx+arm_x; x = x + 1) begin
                    if ((x >= 0) && (x < H) &&
                        ((cy-half_t+j) >= 0) && ((cy-half_t+j) < V)) begin
                        edge_map[(cy-half_t+j)*H + x] = 1'b1;
                        dir_map[(cy-half_t+j)*H + x] = 4'b1110;
                    end
                    if ((x >= 0) && (x < H) &&
                        ((cy+half_t-j) >= 0) && ((cy+half_t-j) < V)) begin
                        edge_map[(cy+half_t-j)*H + x] = 1'b1;
                        dir_map[(cy+half_t-j)*H + x] = 4'b1110;
                    end
                end
            end
        end
    endtask

    task run_frame;
        integer r, c;
        begin
            @(negedge clk);
            i_vs = 1'b1; i_de = 1'b0; i_hs = 1'b0;
            i_row = 11'd0;
            @(negedge clk);
            i_vs = 1'b0;
            repeat (VBLANK) @(negedge clk);

            for (r = 0; r < V; r = r + 1) begin
                for (c = 0; c < H; c = c + 1) begin
                    @(negedge clk);
                    i_de = 1'b1;
                    i_hs = (c == 0);
                    i_row = r[10:0];
                    if (img_mode) begin
                        if (c >= ROI_X) begin
                            img_idx = r * IMG_W + (c - ROI_X);
                            i_edge  = (img_edge_words[img_idx >> 3] >> (img_idx & 7)) & 1'b1;
                            i_dir16 = (img_dir_words[img_idx >> 3] >> ((img_idx & 7) * 4)) & 4'hF;
                        end else begin
                            i_edge = 1'b0; i_dir16 = 4'd0;
                        end
                    end else begin
                        i_edge = edge_map[r*H + c];
                        i_dir16 = dir_map[r*H + c];
                    end
                    i_rgb = i_edge ? 24'hFFFFFF : 24'h000000;
                end
                @(negedge clk);
                i_de = 1'b0; i_hs = 1'b0; i_edge = 1'b0; i_dir16 = 4'd0;
                // edge_proc advances o_row on the first blank beat after the
                // last active pixel.  shape_proc deliberately subtracts one
                // at end_line, so reproduce that ordering here.
                i_row = r + 1;
            end
            repeat (TAIL) @(negedge clk);
            frame_no = frame_no + 1;
        end
    endtask

    task prepare_test;
        begin
            clear_rams;
            do_reset;
            frame_no = 0;
            circ_seen = 0; rect_seen = 0; tri_seen = 0;
            tri_h_seen = 0; tri_rot_seen = 0;
            circ_cx_seen = 0; circ_cy_seen = 0; circ_rr_seen = 0; circ_ry_seen = 0;
            rect_x0_seen = 0; rect_y0_seen = 0;
            rect_x1_seen = 0; rect_y1_seen = 0;
            tri_x0_seen = 0; tri_y0_seen = 0;
            tri_x1_seen = 0; tri_y1_seen = 0;
            max_edges_seen = 0; max_peak_seen = 0;
            circ_far_seen = 0;
        end
    endtask

    initial begin
        errors = 0;
        frame_no = 0;
        circ_seen = 0; rect_seen = 0; tri_seen = 0;
        tri_h_seen = 0; tri_rot_seen = 0;
        circ_cx_seen = 0; circ_cy_seen = 0; circ_rr_seen = 0; circ_ry_seen = 0;
        rect_x0_seen = 0; rect_y0_seen = 0;
        rect_x1_seen = 0; rect_y1_seen = 0;
        tri_x0_seen = 0; tri_y0_seen = 0;
        tri_x1_seen = 0; tri_y1_seen = 0;
        max_edges_seen = 0; max_peak_seen = 0;

        if ($test$plusargs("imgtest")) begin
            img_mode = 1'b1;
            shape_roi_x = ROI_X;
            if (!$value$plusargs("imgfile=%s", img_prefix)) begin
                $display("RESULT: FAIL (imgtest requires +imgfile=<prefix>)");
                $finish;
            end
            if (!$value$plusargs("img_asp=%d", img_asp)) img_asp = -1;
            if (!$value$plusargs("img_frames=%d", img_frames)) img_frames = 12;
            if (!$value$plusargs("expect=%d", img_expect)) img_expect = 0;
            img_edge_file = {img_prefix, "_edge.hex"};
            img_dir_file  = {img_prefix, "_dir.hex"};
            $readmemh(img_edge_file, img_edge_words);
            $readmemh(img_dir_file,  img_dir_words);
            prepare_test;
            repeat (img_frames) run_frame;
            repeat (2) run_frame;
            $display("INFO image_test file=%0s frames=%0d edges=%0d peak=%0d circ=%0d center=(%0d,%0d) r=%0d/%0d rect=%0d bbox=(%0d,%0d)-(%0d,%0d) tri=%0d rot=%0d bbox=(%0d,%0d)-(%0d,%0d)",
                     img_prefix, frame_no, max_edges_seen, max_peak_seen,
                     circ_seen, circ_cx_seen, circ_cy_seen, circ_rr_seen, circ_ry_seen,
                     rect_seen, rect_x0_seen, rect_y0_seen, rect_x1_seen, rect_y1_seen,
                     tri_seen, tri_rot_seen, tri_x0_seen, tri_y0_seen, tri_x1_seen, tri_y1_seen);
            errors = 0;
            if (((img_expect & 1) != 0) && (circ_seen == 0)) errors = errors + 1;
            if (((img_expect & 1) == 0) && (circ_seen != 0)) errors = errors + 1;
            if (((img_expect & 2) != 0) && (rect_seen == 0)) errors = errors + 1;
            if (((img_expect & 2) == 0) && (rect_seen != 0)) errors = errors + 1;
            if (((img_expect & 4) != 0) && (tri_seen  == 0)) errors = errors + 1;
            if (((img_expect & 4) == 0) && (tri_seen  != 0)) errors = errors + 1;
            if (errors == 0) $display("RESULT: PASS (image_test expect=%0d)", img_expect);
            else             $display("RESULT: FAIL (image_test expect=%0d errors=%0d)", img_expect, errors);
            $finish;
        end

        // Reproduce the uploaded target sheet.  The left-pointing triangle has
        // a vertical base on the right and a depth/base-height ratio of about
        // 1.76.  Run it once on the right side (outside the first 1024 columns)
        // and once near the centre to separate coverage from geometry errors.
        if ($test$plusargs("tri_full")) begin
            shape_roi_x = 12'd0;
            build_triangle_right(1500, 40, 200, 1218);
            prepare_test;
            while ((frame_no < 24) && !tri_seen)
                run_frame;
            repeat (2) run_frame;
            $display("INFO target_tri_full: tri=%0d rot=%0d circ=%0d bbox=(%0d,%0d)-(%0d,%0d) frames=%0d",
                     tri_seen, tri_rot_seen, circ_seen,
                     tri_x0_seen, tri_y0_seen, tri_x1_seen, tri_y1_seen,
                     frame_no);
            if (tri_rot_seen && !circ_seen)
                $display("RESULT: PASS (right-side left-pointing triangle)");
            else
                $display("RESULT: FAIL (right-side left-pointing triangle)");
            $finish;
        end

        if ($test$plusargs("tri_mid")) begin
            shape_roi_x = 12'd0;
            build_triangle_right(900, 40, 200, 618);
            prepare_test;
            while ((frame_no < 24) && !tri_seen)
                run_frame;
            repeat (2) run_frame;
            $display("INFO target_tri_mid: tri=%0d rot=%0d circ=%0d bbox=(%0d,%0d)-(%0d,%0d) frames=%0d",
                     tri_seen, tri_rot_seen, circ_seen,
                     tri_x0_seen, tri_y0_seen, tri_x1_seen, tri_y1_seen,
                     frame_no);
            if (tri_rot_seen && !circ_seen)
                $display("RESULT: PASS (centre left-pointing triangle)");
            else
                $display("RESULT: FAIL (centre left-pointing triangle)");
            $finish;
        end

        if ($test$plusargs("target_sheet")) begin
            shape_roi_x = 12'd0;
            target_sheet_mode = 1'b1;
            compose_no_clear = 1'b1;
            clear_map;
            build_ellipse(420, 150, 160, 80);
            build_rect(1050, 30, 1400, 170);
            build_triangle_right(1880, 30, 210, 1563);
            compose_no_clear = 1'b0;
            prepare_test;
            while ((frame_no < 44) && !(tri_seen && rect_seen && circ_seen))
                run_frame;
            repeat (2) run_frame;
            $display("INFO target_sheet: tri=%0d rot=%0d bbox=(%0d,%0d)-(%0d,%0d) rect=%0d bbox=(%0d,%0d)-(%0d,%0d) circ=%0d center=(%0d,%0d) r=%0d/%0d far=%0d frames=%0d",
                     tri_seen, tri_rot_seen,
                     tri_x0_seen, tri_y0_seen, tri_x1_seen, tri_y1_seen,
                     rect_seen, rect_x0_seen, rect_y0_seen, rect_x1_seen, rect_y1_seen,
                     circ_seen, circ_cx_seen, circ_cy_seen, circ_rr_seen, circ_ry_seen,
                     circ_far_seen, frame_no);
            if (tri_rot_seen &&
                (abs_int(tri_x0_seen - 1563) <= 36) &&
                (abs_int(tri_x1_seen - 1880) <= 36) &&
                (abs_int(tri_y0_seen - 30) <= 20) &&
                (abs_int(tri_y1_seen - 210) <= 20) &&
                rect_seen &&
                (abs_int(rect_x0_seen - 1050) <= 24) &&
                (abs_int(rect_x1_seen - 1400) <= 24) &&
                (abs_int(rect_y0_seen - 30) <= 24) &&
                (abs_int(rect_y1_seen - 170) <= 24) &&
                circ_seen && !circ_far_seen &&
                (abs_int(circ_cx_seen - 420) <= 36) &&
                (abs_int(circ_cy_seen - 150) <= 36))
                $display("RESULT: PASS (target sheet)");
            else
                $display("RESULT: FAIL (target sheet)");
            $finish;
        end

        if ($test$plusargs("plus_negative")) begin
            shape_roi_x = 12'd0;
            target_sheet_mode = 1'b0;
            build_plus(960, 120, 220, 100, 38);
            prepare_test;
            repeat (44) run_frame;
            $display("INFO plus_negative: circ=%0d center=(%0d,%0d) r=%0d/%0d rect=%0d bbox=(%0d,%0d)-(%0d,%0d) tri=%0d bbox=(%0d,%0d)-(%0d,%0d)",
                     circ_seen, circ_cx_seen, circ_cy_seen, circ_rr_seen, circ_ry_seen,
                     rect_seen, rect_x0_seen, rect_y0_seen, rect_x1_seen, rect_y1_seen,
                     tri_seen, tri_x0_seen, tri_y0_seen, tri_x1_seen, tri_y1_seen);
            if (!circ_seen && !rect_seen && !tri_seen)
                $display("RESULT: PASS (plus is a negative sample)");
            else
                $display("RESULT: FAIL (plus triggered a shape classifier)");
            $finish;
        end

        // Fast Hough diagnostic: run exactly one populated frame at the end of
        // the 0.5-aspect radius scan (the frame that should contain rx=168,
        // ry=84 for the synthetic target ellipse).
        if ($test$plusargs("hacc_one")) begin
            shape_roi_x = 12'd0;
            target_sheet_mode = 1'b1;
            compose_no_clear = 1'b1;
            clear_map;
            build_ellipse(420, 150, 160, 80);
            build_rect(1050, 30, 1400, 170);
            build_triangle_right(1880, 30, 210, 1563);
            compose_no_clear = 1'b0;
            prepare_test;
            $display("HACC_DIR left=%b right=%b top=%b bottom=%b ll=%b lr=%b ul=%b ur=%b",
                     dir_map[150*H+260], dir_map[150*H+580],
                     dir_map[70*H+420], dir_map[230*H+420],
                     dir_map[200*H+295], dir_map[200*H+545],
                     dir_map[100*H+295], dir_map[100*H+545]);
            // rst_frame at the start of run_frame advances k 7 -> 8.
            u_shape.frame_k = 4'd7;
            u_shape.frame_asp = 3'd3;
            dbg_push_cnt = 0; dbg_pop_cnt = 0; dbg_drop_cnt = 0;
            dbg_max_used_a = 0; dbg_max_used_b = 0;
            run_frame;
            hacc_sum = 0;
            hacc_best0 = 0; hacc_best1 = 0; hacc_best2 = 0;
            hacc_best3 = 0; hacc_best4 = 0;
            hacc_addr0 = 0; hacc_addr1 = 0; hacc_addr2 = 0;
            hacc_addr3 = 0; hacc_addr4 = 0;
            for (hacc_j = 0; hacc_j < ACC_SIZE; hacc_j = hacc_j + 1) begin
                hacc_sum = hacc_sum + u_shape.acc_mem[hacc_j];
                if (u_shape.acc_mem[hacc_j] > hacc_best0) begin
                    hacc_best4 = hacc_best3; hacc_addr4 = hacc_addr3;
                    hacc_best3 = hacc_best2; hacc_addr3 = hacc_addr2;
                    hacc_best2 = hacc_best1; hacc_addr2 = hacc_addr1;
                    hacc_best1 = hacc_best0; hacc_addr1 = hacc_addr0;
                    hacc_best0 = u_shape.acc_mem[hacc_j]; hacc_addr0 = hacc_j;
                end else if (u_shape.acc_mem[hacc_j] > hacc_best1) begin
                    hacc_best4 = hacc_best3; hacc_addr4 = hacc_addr3;
                    hacc_best3 = hacc_best2; hacc_addr3 = hacc_addr2;
                    hacc_best2 = hacc_best1; hacc_addr2 = hacc_addr1;
                    hacc_best1 = u_shape.acc_mem[hacc_j]; hacc_addr1 = hacc_j;
                end else if (u_shape.acc_mem[hacc_j] > hacc_best2) begin
                    hacc_best4 = hacc_best3; hacc_addr4 = hacc_addr3;
                    hacc_best3 = hacc_best2; hacc_addr3 = hacc_addr2;
                    hacc_best2 = u_shape.acc_mem[hacc_j]; hacc_addr2 = hacc_j;
                end else if (u_shape.acc_mem[hacc_j] > hacc_best3) begin
                    hacc_best4 = hacc_best3; hacc_addr4 = hacc_addr3;
                    hacc_best3 = u_shape.acc_mem[hacc_j]; hacc_addr3 = hacc_j;
                end else if (u_shape.acc_mem[hacc_j] > hacc_best4) begin
                    hacc_best4 = u_shape.acc_mem[hacc_j]; hacc_addr4 = hacc_j;
                end
            end
            $display("HACC_ONE k=%0d asp=%0d r=%0d/%0d peak=%0d@(%0d,%0d) true=%0d near412=%0d near428=%0d false=%0d drop=%b",
                     u_shape.frame_k, u_shape.frame_asp,
                     u_shape.cur_r, u_shape.cur_ry,
                     u_shape.peak_val, u_shape.peak_cx, u_shape.peak_cy,
                     u_shape.acc_mem[(150 >> 3) * ACC_W + (420 >> 3)],
                     u_shape.acc_mem[(156 >> 3) * ACC_W + (412 >> 3)],
                     u_shape.acc_mem[(148 >> 3) * ACC_W + (428 >> 3)],
                     u_shape.acc_mem[(124 >> 3) * ACC_W + (1204 >> 3)],
                     u_shape.fifo_drop);
            $display("HACC_FLOW push=%0d pop=%0d drop=%0d max_used=%0d/%0d",
                     dbg_push_cnt, dbg_pop_cnt, dbg_drop_cnt,
                     dbg_max_used_a, dbg_max_used_b);
            $display("HACC_SUM sum=%0d top=%0d@(%0d,%0d) %0d@(%0d,%0d) %0d@(%0d,%0d) %0d@(%0d,%0d) %0d@(%0d,%0d)",
                     hacc_sum,
                     hacc_best0, (hacc_addr0 % ACC_W) * 8 + 4, (hacc_addr0 / ACC_W) * 8 + 4,
                     hacc_best1, (hacc_addr1 % ACC_W) * 8 + 4, (hacc_addr1 / ACC_W) * 8 + 4,
                     hacc_best2, (hacc_addr2 % ACC_W) * 8 + 4, (hacc_addr2 / ACC_W) * 8 + 4,
                     hacc_best3, (hacc_addr3 % ACC_W) * 8 + 4, (hacc_addr3 / ACC_W) * 8 + 4,
                     hacc_best4, (hacc_addr4 % ACC_W) * 8 + 4, (hacc_addr4 / ACC_W) * 8 + 4);
            $finish;
        end

        if ($test$plusargs("rect_only")) begin
            build_rect(1370, 60, 1480, 210);
            prepare_test;
            while ((frame_no < 12) && !rect_seen)
                run_frame;
            repeat (2) run_frame;
            $display("INFO roi_rect_only: seen=%0d bbox=(%0d,%0d)-(%0d,%0d) frames=%0d",
                     rect_seen, rect_x0_seen, rect_y0_seen,
                     rect_x1_seen, rect_y1_seen, frame_no);
            if (rect_seen)
                $display("RESULT: PASS (rect-only debug)");
            else
                $display("RESULT: FAIL (rect-only debug)");
            $finish;
        end
        if ($test$plusargs("ellipse_only")) begin
            build_ellipse(1440, 120, 55, 86);
            prepare_test;
            while ((frame_no < 26) && !circ_seen)
                run_frame;
            repeat (2) run_frame;
            $display("INFO roi_ellipse_only: seen=%0d center=(%0d,%0d) r=%0d/%0d frames=%0d",
                     circ_seen, circ_cx_seen, circ_cy_seen, circ_rr_seen,
                     circ_ry_seen, frame_no);
            if (circ_seen)
                $display("RESULT: PASS (ellipse-only debug)");
            else
                $display("RESULT: FAIL (ellipse-only debug)");
            $finish;
        end
        if ($test$plusargs("wide_only")) begin
            build_ellipse(1440, 120, 86, 55);
            prepare_test;
            while ((frame_no < 44) && !circ_seen)
                run_frame;
            repeat (2) run_frame;
            $display("INFO roi_ellipse_wide_only: seen=%0d center=(%0d,%0d) r=%0d/%0d frames=%0d",
                     circ_seen, circ_cx_seen, circ_cy_seen, circ_rr_seen,
                     circ_ry_seen, frame_no);
            if (circ_seen &&
                (abs_int(circ_cx_seen - 1440) <= 12) &&
                (abs_int(circ_cy_seen - 120) <= 12) &&
                (circ_rr_seen >= 72) && (circ_rr_seen <= 102) &&
                (circ_ry_seen >= 42) && (circ_ry_seen <= 68))
                $display("RESULT: PASS (wide-ellipse debug)");
            else
                $display("RESULT: FAIL (wide-ellipse debug)");
            $finish;
        end
        if ($test$plusargs("rtri_only")) begin
            build_triangle_right(1340, 60, 170, 1505);
            prepare_test;
            while ((frame_no < 18) && !tri_seen)
                run_frame;
            repeat (2) run_frame;
            $display("INFO roi_triangle_right_only: seen=%0d bbox=(%0d,%0d)-(%0d,%0d) frames=%0d",
                     tri_seen, tri_x0_seen, tri_y0_seen,
                     tri_x1_seen, tri_y1_seen, frame_no);
            if (tri_seen)
                $display("RESULT: PASS (right-triangle debug)");
            else
                $display("RESULT: FAIL (right-triangle debug)");
            $finish;
        end

        // Right-half ellipse with the tall perspective ratio measured from the
        // reference capture (55/75 is 1.36; the real sheet is closer to 1.56).
        build_ellipse(1440, 120, 55, 86);
        prepare_test;
        while ((frame_no < 22) && !circ_seen)
            run_frame;
        repeat (2) run_frame;
        $display("INFO roi_ellipse: seen=%0d center=(%0d,%0d) r=%0d/%0d edges=%0d peak=%0d frames=%0d",
                 circ_seen, circ_cx_seen, circ_cy_seen, circ_rr_seen, circ_ry_seen,
                 max_edges_seen, max_peak_seen, frame_no);
        if (!circ_seen) begin
            $display("FAIL roi_ellipse: right-half ellipse was not recognized");
            errors = errors + 1;
        end else begin
            if ((abs_int(circ_cx_seen - 1440) > 12) ||
                (abs_int(circ_cy_seen - 120) > 12) ||
                (circ_rr_seen < 44) || (circ_rr_seen > 68) ||
                (circ_ry_seen < 74) || (circ_ry_seen > 100)) begin
                $display("FAIL roi_ellipse: bbox (%0d,%0d) r=%0d/%0d is not near (1440,120) r=55/86",
                         circ_cx_seen, circ_cy_seen, circ_rr_seen, circ_ry_seen);
                errors = errors + 1;
            end
        end

        // The demo sheet's circle is wider than it is tall.  Scan the
        // transposed aspect ratio as well; a tall-only Hough hypothesis misses
        // this target completely.
        build_ellipse(1440, 120, 86, 55);
        prepare_test;
        while ((frame_no < 44) && !circ_seen)
            run_frame;
        repeat (2) run_frame;
        $display("INFO roi_ellipse_wide: seen=%0d center=(%0d,%0d) r=%0d/%0d edges=%0d peak=%0d frames=%0d",
                 circ_seen, circ_cx_seen, circ_cy_seen, circ_rr_seen, circ_ry_seen,
                 max_edges_seen, max_peak_seen, frame_no);
        if (!circ_seen) begin
            $display("FAIL roi_ellipse_wide: horizontal ellipse was not recognized");
            errors = errors + 1;
        end else begin
            if ((abs_int(circ_cx_seen - 1440) > 12) ||
                (abs_int(circ_cy_seen - 120) > 12) ||
                (circ_rr_seen < 72) || (circ_rr_seen > 102) ||
                (circ_ry_seen < 42) || (circ_ry_seen > 68)) begin
                $display("FAIL roi_ellipse_wide: bbox (%0d,%0d) r=%0d/%0d is not near (1440,120) r=86/55",
                         circ_cx_seen, circ_cy_seen, circ_rr_seen, circ_ry_seen);
                errors = errors + 1;
            end
        end

        // A shape wholly in the left half must not leak through the ROI gate.
        build_ellipse(480, 120, 78, 102);
        prepare_test;
        for (frame_no = 0; frame_no < 12; frame_no = frame_no + 1)
            run_frame;
        $display("INFO roi_left_negative: circ=%0d rect=%0d tri=%0d edges=%0d",
                 circ_seen, rect_seen, tri_seen, max_edges_seen);
        if (circ_seen || rect_seen || tri_seen) begin
            $display("FAIL roi_left_negative: left-half target produced a shape lock");
            errors = errors + 1;
        end
        if (max_edges_seen != 0) begin
            $display("FAIL roi_left_negative: ROI edge counter saw left-half pixels");
            errors = errors + 1;
        end

        // Right-half rectangle.
        build_rect(1370, 60, 1480, 210);
        prepare_test;
        while ((frame_no < 10) && !rect_seen)
            run_frame;
        repeat (2) run_frame;
        $display("INFO roi_rect: seen=%0d bbox=(%0d,%0d)-(%0d,%0d) frames=%0d",
                 rect_seen, rect_x0_seen, rect_y0_seen,
                 rect_x1_seen, rect_y1_seen, frame_no);
        if (!rect_seen) begin
            $display("FAIL roi_rect: right-half rectangle was not recognized");
            errors = errors + 1;
        end else if ((abs_int(rect_x0_seen - 1370) > 12) ||
                     (abs_int(rect_x1_seen - 1480) > 12) ||
                     (abs_int(rect_y0_seen - 60) > 12) ||
                     (abs_int(rect_y1_seen - 210) > 12)) begin
            $display("FAIL roi_rect: bbox does not follow (1370,60)-(1480,210)");
            errors = errors + 1;
        end
        if (tri_rot_seen) begin
            $display("FAIL roi_rect: rectangle falsely produced a rotated-triangle lock");
            errors = errors + 1;
        end

        // Right-half upright triangle, height = 1.5 * base as expected by RTL.
        build_triangle(1440, 70, 220, 50);
        prepare_test;
        while ((frame_no < 10) && !tri_seen)
            run_frame;
        repeat (2) run_frame;
        $display("INFO roi_triangle: seen=%0d bbox=(%0d,%0d)-(%0d,%0d) frames=%0d",
                 tri_seen, tri_x0_seen, tri_y0_seen,
                 tri_x1_seen, tri_y1_seen, frame_no);
        if (!tri_h_seen) begin
            $display("FAIL roi_triangle: right-half triangle was not recognized");
            errors = errors + 1;
        end

        // Same triangle rotated 90 degrees, matching the actual demo sheet.
        build_triangle_right(1340, 60, 170, 1505);
        prepare_test;
        while ((frame_no < 18) && !tri_seen)
            run_frame;
        repeat (2) run_frame;
        $display("INFO roi_triangle_right: seen=%0d bbox=(%0d,%0d)-(%0d,%0d) frames=%0d",
                 tri_seen, tri_x0_seen, tri_y0_seen,
                 tri_x1_seen, tri_y1_seen, frame_no);
        if (!tri_rot_seen) begin
            $display("FAIL roi_triangle_right: right-pointing triangle was not recognized");
            errors = errors + 1;
        end

        if (errors == 0)
            $display("RESULT: PASS (1920-wide right-half ROI shapes)");
        else
            $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

endmodule
