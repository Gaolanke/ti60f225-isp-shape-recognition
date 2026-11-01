`timescale 1ns/1ps

// Integrated simulation of the board image chain:
//   synthetic {B,G,R} video -> edge_proc -> shape_proc -> {B,G,R} video
//
// It runs the real RTL for both modules.  The shape_proc memories are cleared
// explicitly because Icarus does not model the Efinity RAM10 power-up state.
// Test 1 checks exact passthrough with shape recognition disabled.  Test 2 and
// 3 enable shape recognition and check that the chain still produces one valid
// pixel per input pixel without X propagation.

module tb_edge_chain;

    localparam integer H = 64;
    localparam integer V = 48;
    localparam integer N = H * V;
    localparam integer TOP_BLANK = 8;
    localparam integer COL_BLANK = 2;
    localparam integer ACC_H = (V >> 3) + 1;
    localparam integer ACC_SIZE = 256 * ACC_H;

    reg         clk = 1'b0;
    reg         rst_n = 1'b0;
    reg         i_de = 1'b0;
    reg         i_hs = 1'b0;
    reg         i_vs = 1'b0;
    reg  [23:0] i_rgb = 24'd0;
    reg  [7:0]  i_th_low = 8'd2;
    reg  [7:0]  i_th_high = 8'd5;
    reg  [1:0]  i_prefilt = 2'd2;
    reg         i_overlay = 1'b1;
    reg  [10:0] i_split_col = 11'd0;
    reg  [1:0]  i_color_mode = 2'd0;
    reg         i_pip = 1'b0;

    wire [23:0] e_rgb;
    wire        e_de, e_hs, e_vs, e_edge_flag, e_edge;
    wire [3:0]  e_dir16;
    wire [10:0] e_row;

    edge_proc #(
        .H_ACTIVE   (H),
        .SPLIT_COL  (0),
        .TH_LOW     (8'd2),
        .TH_HIGH    (8'd5),
        .TOP_BLANK  (TOP_BLANK),
        .COL_BLANK  (COL_BLANK),
        .V_ACTIVE   (V),
        .PIP_W      (16),
        .PIP_H      (16)
    ) u_edge (
        .clk          (clk),
        .rst_n        (rst_n),
        .i_de         (i_de),
        .i_hs         (i_hs),
        .i_vs         (i_vs),
        .i_rgb        (i_rgb),
        .i_th_low     (i_th_low),
        .i_th_high    (i_th_high),
        .i_prefilt    (i_prefilt),
        .i_overlay    (i_overlay),
        .i_split_col  (i_split_col),
        .i_color_mode (i_color_mode),
        .i_pip        (i_pip),
        .i_shape_mode (1'b0),
        .o_de         (e_de),
        .o_hs         (e_hs),
        .o_vs         (e_vs),
        .o_rgb        (e_rgb),
        .o_edge_flag  (e_edge_flag),
        .o_edge       (e_edge),
        .o_dir16      (e_dir16),
        .o_row        (e_row)
    );

    reg         shape_en = 1'b0;
    wire [23:0] s_rgb;
    wire        s_de, s_hs, s_vs, s_hit;
    wire [3:0]  s_dbg;

    shape_proc #(
        .H_ACTIVE     (H),
        .V_ACTIVE     (V),
        .BIN_SHIFT    (3),
        .ACC_W        (256),
        .RECT_LEN_MIN (16),
        .RECT_LEN_MAX (48),
        .RECT_GAP_MIN (4),
        .RECT_SIDE_MIN(4),
        .RUN_GAP      (2),
        .TOL_PX       (2),
        .R0           (8),
        .R_STEP       (4),
        .R_NUM        (4),
        .HOUGH_TH     (8),
        .CIRC_RATIO_SHIFT (4),
        .CIRC_ARC_TOL (4),
        .CIRC_ARC_MIN (6),
        .CIRC_CORNER_MAX (32),
        .CIRC_HOLD    (3),
        .RECT_HOLD    (3),
        .TRI_W_MIN    (8),
        .TRI_TOL_PX   (3)
    ) u_shape (
        .clk         (clk),
        .rst_n       (rst_n),
        .i_de        (e_de),
        .i_hs        (e_hs),
        .i_vs        (e_vs),
        .i_rgb       (e_rgb),
        .i_edge      (e_edge),
        .i_dir16     (e_dir16),
        .i_row       (e_row),
        .i_en        (shape_en),
        .i_draw_x0   (12'd0),
        .o_de        (s_de),
        .o_hs        (s_hs),
        .o_vs        (s_vs),
        .o_rgb       (s_rgb),
        .o_shape_hit (s_hit),
        .o_dbg       (s_dbg)
    );

    always #5 clk = ~clk;

    reg [23:0] src [0:N-1];
    reg [23:0] shape_cap [0:N-1];
    reg [23:0] ref_cap [0:N-1];
    integer shape_count;
    integer ref_count;
    integer shape_hit_events;
    integer chain_edge_pixels;
    integer rect_seen;
    integer circ_seen;
    integer tri_seen;
    integer tri_rot_seen;
    integer circ_cand_seen;
    integer circ_corner_max_seen;
    integer circ_confirm_events;
    integer circ_cx_seen;
    integer circ_cy_seen;
    integer circ_rr_seen;
    integer errors;
    integer tests_run;
    integer err0;
    integer idx, row, col;
    integer x_count;
    integer mismatch;
    integer acc_i;
    reg     capture_on = 1'b0;
    reg     s_hit_d = 1'b0;
    reg     circ_valid_d = 1'b0;
    reg     shape_debug = 1'b0;

    // Reference delay: shape_proc registers i_rgb/i_de three times on the way
    // to o_rgb/o_de (a_rgb/a_de, b_rgb/b_de, o_rgb/o_de).
    reg [23:0] ref1, ref2, ref3;
    reg        ref_de1, ref_de2, ref_de3;

    always @(posedge clk) begin
        ref1 <= e_rgb;  ref2 <= ref1;  ref3 <= ref2;
        ref_de1 <= e_de; ref_de2 <= ref_de1; ref_de3 <= ref_de2;
    end

    always @(negedge clk) begin
        if (rst_n) begin
            if (capture_on && e_de && e_edge) chain_edge_pixels = chain_edge_pixels + 1;
            if (s_hit && !s_hit_d) shape_hit_events = shape_hit_events + 1;
            s_hit_d <= s_hit;
            if (u_shape.rect_valid) rect_seen = 1;
            if (u_shape.circ_valid) circ_seen = 1;
            if (u_shape.tri_valid)  tri_seen  = 1;
            if (u_shape.tri_rot_valid) tri_rot_seen = 1;
            if (u_shape.circ_cand_active) begin
                circ_cand_seen = 1;
                if (u_shape.circ_corner_hits > circ_corner_max_seen)
                    circ_corner_max_seen = u_shape.circ_corner_hits;
            end
            if (u_shape.circ_valid && !circ_valid_d)
                circ_confirm_events = circ_confirm_events + 1;
            circ_valid_d <= u_shape.circ_valid;
            if (u_shape.circ_valid) begin
                circ_cx_seen = u_shape.circ_cx;
                circ_cy_seen = u_shape.circ_cy;
                circ_rr_seen = u_shape.circ_rr;
            end
            if (s_de && capture_on) begin
                if (shape_count < N)
                    shape_cap[shape_count] = s_rgb;
                shape_count = shape_count + 1;
            end
            if (ref_de3 && capture_on) begin
                if (ref_count < N)
                    ref_cap[ref_count] = ref3;
                ref_count = ref_count + 1;
            end
        end
    end

    always @(posedge clk) begin
        if (rst_n && shape_debug && u_shape.rst_frame)
            $display("DBG shape_frame edges=%0d peak=%0d cand=(%0d,%0d) r=%0d/%0d active=%b corner=%0d arc=%0d/%b diag=%0d circ=%b bbox=(%0d,%0d) r=%0d | tri=(%0d,%0d)-(%0d,%0d) active=%b hits=%0d/%0d need=%0d inner=%0d valid=%b box=(%0d,%0d)-(%0d,%0d) | rect=%b box=(%0d,%0d)-(%0d,%0d)",
                     u_shape.frame_edge_count, u_shape.peak_val,
                     u_shape.peak_cx, u_shape.peak_cy,
                     u_shape.cur_r, u_shape.cur_ry, u_shape.circ_cand_active,
                     u_shape.circ_corner_hits, u_shape.circ_arc_hits,
                     u_shape.circ_arc_mask, u_shape.circ_diag_cnt,
                     u_shape.circ_valid,
                     u_shape.circ_cx, u_shape.circ_cy, u_shape.circ_rr,
                     u_shape.tri_cand_x0, u_shape.tri_cand_y0,
                     u_shape.tri_cand_x1, u_shape.tri_cand_y1,
                     u_shape.tri_cand_active, u_shape.tri_l_hits,
                     u_shape.tri_r_hits, u_shape.tri_need,
                     u_shape.tri_inner_hits, u_shape.tri_valid,
                     u_shape.tri_x0, u_shape.tri_y0,
                     u_shape.tri_x1, u_shape.tri_y1,
                     u_shape.rect_valid,
                     u_shape.rect_x0, u_shape.rect_y0,
                     u_shape.rect_x1, u_shape.rect_y1);
    end

    task do_reset;
        begin
            rst_n = 1'b0;
            i_de = 1'b0; i_hs = 1'b0; i_vs = 1'b0; i_rgb = 24'd0;
            repeat (12) @(negedge clk);
            rst_n = 1'b1;
            repeat (6) @(negedge clk);
        end
    endtask

    task clear_rams;
        integer i;
        begin
            for (i = 0; i < 2048; i = i + 1) begin
                u_edge.lb0_mem[i]  = 8'd0;
                u_edge.lb1_mem[i]  = 8'd0;
                u_edge.flb0_mem[i] = 8'd0;
                u_edge.flb1_mem[i] = 8'd0;
                u_edge.flb2_mem[i] = 8'd0;
                u_edge.flb3_mem[i] = 8'd0;
                u_edge.mb0_mem[i]  = 18'd0;
                u_edge.mb1_mem[i]  = 18'd0;
                u_edge.sb0_mem[i]  = 2'd0;
                u_edge.sb1_mem[i]  = 2'd0;
                u_edge.hb_mem[i]   = 1'b0;
                u_edge.out_line_mem[i] = 40'd0;
            end
            for (i = 0; i < 1024; i = i + 1) begin
                u_shape.vf_a[i] = 16'd0;
                u_shape.vf_b[i] = 16'd0;
            end
            for (i = 0; i < ACC_SIZE; i = i + 1)
                u_shape.acc_mem[i] = 8'd0;
        end
    endtask

    task clear_capture;
        integer i;
        begin
            for (i = 0; i < N; i = i + 1)
                shape_cap[i] = 24'd0;
            for (i = 0; i < N; i = i + 1)
                ref_cap[i] = 24'd0;
            shape_count = 0;
            ref_count = 0;
            shape_hit_events = 0;
            chain_edge_pixels = 0;
            rect_seen = 0;
            circ_seen = 0;
            tri_seen = 0;
            tri_rot_seen = 0;
            circ_cand_seen = 0;
            circ_corner_max_seen = 0;
            circ_confirm_events = 0;
            circ_cx_seen = 0;
            circ_cy_seen = 0;
            circ_rr_seen = 0;
            s_hit_d = 1'b0;
        end
    endtask

    task prepare_test;
        begin
            clear_rams;
            do_reset;
            clear_capture;
        end
    endtask

    task run_frame;
        input integer vblank;
        input integer tail;
        integer r, c;
        begin
            @(negedge clk);
            i_vs = 1'b1; i_de = 1'b0; i_hs = 1'b0; i_rgb = 24'd0;
            @(negedge clk);
            i_vs = 1'b0;
            repeat (vblank) @(negedge clk);

            for (r = 0; r < V; r = r + 1) begin
                for (c = 0; c < H; c = c + 1) begin
                    @(negedge clk);
                    i_de = 1'b1;
                    i_hs = (c == 0);
                    i_rgb = src[r*H + c];
                end
                @(negedge clk);
                i_de = 1'b0;
                i_hs = 1'b0;
                i_rgb = 24'd0;
            end
            repeat (tail) @(negedge clk);
        end
    endtask

    task fill_checker8;
        integer r, c;
        begin
            for (r = 0; r < V; r = r + 1)
                for (c = 0; c < H; c = c + 1)
                    src[r*H + c] = (((r/8) + (c/8)) & 1) ? 24'hFFFFFF : 24'h000000;
        end
    endtask

    task fill_color;
        input [7:0] r;
        input [7:0] g;
        input [7:0] b;
        integer k;
        begin
            for (k = 0; k < N; k = k + 1)
                src[k] = {b, g, r};
        end
    endtask

    task fill_filled_rect;
        input integer x0, y0, x1, y1;
        integer r, c;
        begin
            fill_color(8'd0, 8'd0, 8'd0);
            for (r = y0; r <= y1; r = r + 1)
                for (c = x0; c <= x1; c = c + 1)
                    if ((r >= 0) && (r < V) && (c >= 0) && (c < H))
                        src[r*H + c] = 24'hFFFFFF;
        end
    endtask

    task fill_filled_ellipse;
        input integer cx, cy, rx, ry;
        integer r, c, dx, dy, lhs, rhs;
        begin
            fill_color(8'd0, 8'd0, 8'd0);
            rhs = rx * rx * ry * ry;
            for (r = 0; r < V; r = r + 1) begin
                dy = r - cy;
                for (c = 0; c < H; c = c + 1) begin
                    dx = c - cx;
                    lhs = dx * dx * ry * ry + dy * dy * rx * rx;
                    if (lhs <= rhs)
                        src[r*H + c] = 24'hFFFFFF;
                end
            end
        end
    endtask

    // Low-contrast checker background with a bright filled ellipse.  This
    // keeps the ellipse boundary visible on both checker phases while adding
    // realistic clutter to the Hough accumulator and corner verifier.
    task fill_ellipse_on_checker8;
        input integer cx, cy, rx, ry;
        integer r, c, dx, dy, lhs, rhs;
        begin
            rhs = rx * rx * ry * ry;
            for (r = 0; r < V; r = r + 1) begin
                dy = r - cy;
                for (c = 0; c < H; c = c + 1) begin
                    dx = c - cx;
                    src[r*H + c] = (((r/8) + (c/8)) & 1) ? 24'h303030 : 24'h101010;
                    lhs = dx * dx * ry * ry + dy * dy * rx * rx;
                    if (lhs <= rhs)
                        src[r*H + c] = 24'hFFFFFF;
                end
            end
        end
    endtask

    task fill_filled_triangle;
        input integer apex_x, apex_y, base_y, half_w;
        integer r, c, h, half, xl, xr;
        begin
            fill_color(8'd0, 8'd0, 8'd0);
            if (base_y > apex_y) begin
                h = base_y - apex_y;
                for (r = apex_y; r <= base_y; r = r + 1) begin
                    half = ((r - apex_y) * half_w) / h;
                    xl = apex_x - half;
                    xr = apex_x + half;
                    for (c = xl; c <= xr; c = c + 1)
                        if ((r >= 0) && (r < V) && (c >= 0) && (c < H))
                            src[r*H + c] = 24'hFFFFFF;
                end
            end
        end
    endtask

    task run_n_frames;
        input integer count;
        input integer vblank;
        input integer tail;
        integer k;
        begin
            for (k = 0; k < count; k = k + 1)
                run_frame(vblank, tail);
        end
    endtask

    task write_ppm;
        input [1023:0] path;
        integer fd, k;
        begin
            fd = $fopen(path, "wb");
            if (fd == 0) begin
                $display("WARN: cannot open %0s", path);
            end else begin
                $fwrite(fd, "P6\n%0d %0d\n255\n", H, V);
                for (k = 0; k < N; k = k + 1)
                    $fwrite(fd, "%c%c%c", shape_cap[k][7:0], shape_cap[k][15:8], shape_cap[k][23:16]);
                $fclose(fd);
                $display("INFO saved %0s", path);
            end
        end
    endtask

    function integer count_x;
        integer i;
        begin
            count_x = 0;
            for (i = 0; i < N; i = i + 1)
                if ((^shape_cap[i]) === 1'bx)
                    count_x = count_x + 1;
        end
    endfunction

    function integer abs_int;
        input integer value;
        begin
            abs_int = (value < 0) ? -value : value;
        end
    endfunction

    initial begin
        errors = 0;
        tests_run = 0;
        shape_count = 0;
        ref_count = 0;
        shape_hit_events = 0;
        shape_debug = $test$plusargs("shape_debug");

        if ($test$plusargs("vcd")) begin
            $dumpfile("wave/tb_edge_chain.vcd");
            $dumpvars(0, tb_edge_chain);
        end

        // ------------------------------------------------------------------
        // Test 1: shape disabled, exact pixel/de passthrough.
        // ------------------------------------------------------------------
        prepare_test;
        fill_checker8;
        shape_en = 1'b0;
        capture_on = 1'b0;
        run_frame(2300, 200);
        clear_capture;
        capture_on = 1'b1;
        run_frame(2300, 200);
        capture_on = 1'b0;
        tests_run = tests_run + 1;
        err0 = errors;

        if (shape_count != N) begin
            $display("FAIL chain_passthrough: shape output beats=%0d, expected %0d", shape_count, N);
            errors = errors + 1;
        end
        x_count = count_x();
        if (x_count != 0) begin
            $display("FAIL chain_passthrough: %0d output pixels contain X", x_count);
            errors = errors + 1;
        end
        // Compare captured pixels against the three-stage delayed edge output.
        mismatch = 0;
        if (ref_count != shape_count) begin
            $display("FAIL chain_passthrough: reference beats=%0d, shape beats=%0d",
                     ref_count, shape_count);
            errors = errors + 1;
        end
        for (idx = 0; idx < N; idx = idx + 1) begin
            if (shape_cap[idx] !== ref_cap[idx]) begin
                if (mismatch < 10)
                    $display("FAIL chain_passthrough: pixel %0d shape=%06h ref=%06h",
                             idx, shape_cap[idx], ref_cap[idx]);
                mismatch = mismatch + 1;
            end
        end
        if (mismatch != 0) begin
            $display("FAIL chain_passthrough: %0d mismatching pixels", mismatch);
            errors = errors + 1;
        end
        write_ppm("out/chain_passthrough.ppm");
        if (errors == err0)
            $display("TEST chain_passthrough: PASS");
        else
            $display("TEST chain_passthrough: FAIL");

        // ------------------------------------------------------------------
        // Test 2: shape enabled on checkerboard, no X and exactly N beats.
        // ------------------------------------------------------------------
        prepare_test;
        fill_checker8;
        shape_en = 1'b1;
        capture_on = 1'b0;
        run_n_frames(7, 2300, 200);
        capture_on = 1'b1;
        run_frame(2300, 200);
        capture_on = 1'b0;
        tests_run = tests_run + 1;
        err0 = errors;

        if (shape_count != N) begin
            $display("FAIL chain_shape: shape output beats=%0d, expected %0d", shape_count, N);
            errors = errors + 1;
        end
        x_count = count_x();
        if (x_count != 0) begin
            $display("FAIL chain_shape: %0d output pixels contain X", x_count);
            errors = errors + 1;
        end
        $display("INFO chain_shape: shape_hit_events=%0d dbg=%b peak=%0d edges=%0d circ=%b rect=%b tri=%b trirot=%b",
                 shape_hit_events, s_dbg, u_shape.peak_val, chain_edge_pixels,
                 u_shape.circ_valid, u_shape.rect_valid, u_shape.tri_valid,
                 u_shape.tri_rot_valid);
        if ((shape_hit_events != 0) || tri_rot_seen) begin
            $display("FAIL chain_shape: checkerboard produced %0d false shape hits", shape_hit_events);
            errors = errors + 1;
        end
        write_ppm("out/chain_shape.ppm");
        if (errors == err0)
            $display("TEST chain_shape_enabled: PASS");
        else
            $display("TEST chain_shape_enabled: FAIL");

        // ------------------------------------------------------------------
        // Test 3: shape enabled on a constant frame.  This exercises the
        // accumulator/Hough clear path with no edge votes.
        // ------------------------------------------------------------------
        prepare_test;
        fill_color(8'd40, 8'd80, 8'd120);
        shape_en = 1'b1;
        capture_on = 1'b0;
        run_frame(2300, 200);
        clear_capture;
        capture_on = 1'b1;
        run_frame(2300, 200);
        capture_on = 1'b0;
        tests_run = tests_run + 1;
        err0 = errors;

        if (shape_count != N) begin
            $display("FAIL chain_constant: shape output beats=%0d, expected %0d", shape_count, N);
            errors = errors + 1;
        end
        x_count = count_x();
        if (x_count != 0) begin
            $display("FAIL chain_constant: %0d output pixels contain X", x_count);
            errors = errors + 1;
        end
        $display("INFO chain_constant: shape_hit_events=%0d peak=%0d edges=%0d circ=%b rect=%b tri=%b trirot=%b",
                 shape_hit_events, u_shape.peak_val, chain_edge_pixels,
                 u_shape.circ_valid, u_shape.rect_valid, u_shape.tri_valid,
                 u_shape.tri_rot_valid);
        if ((shape_hit_events != 0) || tri_rot_seen) begin
            $display("FAIL chain_constant: constant frame produced %0d false shape hits", shape_hit_events);
            errors = errors + 1;
        end
        write_ppm("out/chain_constant.ppm");
        if (errors == err0)
            $display("TEST chain_constant_frame: PASS");
        else
            $display("TEST chain_constant_frame: FAIL");

        // ------------------------------------------------------------------
        // Test 4: synthetic filled rectangle.  The top/bottom long runs are
        // found in one frame and the four-edge coverage is validated in the
        // next frame.
        // ------------------------------------------------------------------
        prepare_test;
        fill_filled_rect(10, 12, 54, 40);
        shape_en = 1'b1;
        capture_on = 1'b0;
        run_n_frames(5, 2300, 200);
        capture_on = 1'b1;
        run_frame(2300, 200);
        capture_on = 1'b0;
        tests_run = tests_run + 1;
        err0 = errors;

        $display("INFO chain_rect: hit_events=%0d valid=%b peak=%0d edges=%0d",
                 shape_hit_events, u_shape.rect_valid, u_shape.peak_val, chain_edge_pixels);
        $display("INFO chain_rect seen: rect=%0d circ=%0d tri=%0d trirot=%0d",
                 rect_seen, circ_seen, tri_seen, tri_rot_seen);
        if (!rect_seen || (shape_hit_events == 0)) begin
            $display("FAIL chain_rect: rectangle was not recognized");
            errors = errors + 1;
        end
        if (circ_seen || tri_seen || tri_rot_seen) begin
            $display("FAIL chain_rect: rectangle also triggered circle/triangle classifiers");
            errors = errors + 1;
        end
        write_ppm("out/chain_rect.ppm");
        if (errors == err0)
            $display("TEST chain_rect_positive: PASS");
        else
            $display("TEST chain_rect_positive: FAIL");

        // ------------------------------------------------------------------
        // Test 5: synthetic filled ellipse.  The Hough scan treats the demo
        // circle as a 1.3125x vertically stretched ellipse.
        // ------------------------------------------------------------------
        prepare_test;
        fill_filled_ellipse(32, 26, 12, 15);
        shape_en = 1'b1;
        capture_on = 1'b0;
        run_n_frames(5, 2300, 200);
        capture_on = 1'b1;
        run_frame(2300, 200);
        capture_on = 1'b0;
        tests_run = tests_run + 1;
        err0 = errors;

        $display("INFO chain_circle: hit_events=%0d valid=%b peak=%0d edges=%0d",
                 shape_hit_events, u_shape.circ_valid, u_shape.peak_val, chain_edge_pixels);
        $display("INFO chain_circle seen: rect=%0d circ=%0d tri=%0d trirot=%0d cand=%0d max_corner=%0d confirms=%0d bbox=(%0d,%0d) r=%0d",
                 rect_seen, circ_seen, tri_seen, tri_rot_seen, circ_cand_seen,
                 circ_corner_max_seen, circ_confirm_events,
                 circ_cx_seen, circ_cy_seen, circ_rr_seen);
        if (!circ_seen || (shape_hit_events == 0)) begin
            $display("FAIL chain_circle: ellipse was not recognized");
            errors = errors + 1;
        end
        if (rect_seen || tri_seen || tri_rot_seen) begin
            $display("FAIL chain_circle: ellipse also triggered rectangle/triangle classifiers");
            errors = errors + 1;
        end
        if (circ_seen && ((abs_int(circ_cx_seen - 32) > 10) ||
                          (abs_int(circ_cy_seen - 26) > 10) ||
                          (circ_rr_seen < 8) || (circ_rr_seen > 20))) begin
            $display("FAIL chain_circle: bbox does not follow the 12/15 ellipse");
            errors = errors + 1;
        end
        write_ppm("out/chain_circle.ppm");
        if (errors == err0)
            $display("TEST chain_circle_positive: PASS");
        else
            $display("TEST chain_circle_positive: FAIL");

        // ------------------------------------------------------------------
        // Test 5a: ellipse whose radii deliberately fall between the Hough
        // radius scan steps.  This checks that a target slightly larger or
        // smaller than a scanned radius is still accepted.
        // ------------------------------------------------------------------
        prepare_test;
        fill_filled_ellipse(32, 26, 18, 24);
        shape_en = 1'b1;
        capture_on = 1'b0;
        run_n_frames(11, 2300, 200);
        clear_capture;
        capture_on = 1'b1;
        run_frame(2300, 200);
        capture_on = 1'b0;
        tests_run = tests_run + 1;
        err0 = errors;

        $display("INFO chain_circle_offgrid: hit_events=%0d valid=%b peak=%0d edges=%0d",
                 shape_hit_events, u_shape.circ_valid, u_shape.peak_val, chain_edge_pixels);
        $display("INFO chain_circle_offgrid seen: rect=%0d circ=%0d tri=%0d trirot=%0d cand=%0d max_corner=%0d confirms=%0d bbox=(%0d,%0d) r=%0d",
                 rect_seen, circ_seen, tri_seen, tri_rot_seen, circ_cand_seen,
                 circ_corner_max_seen, circ_confirm_events,
                 circ_cx_seen, circ_cy_seen, circ_rr_seen);
        // circ_valid is a held level: the box may still be up from the
        // previous frame, so a fresh rising edge is not required.
        if (!circ_seen || ((shape_hit_events == 0) && !s_hit)) begin
            $display("FAIL chain_circle_offgrid: off-grid ellipse was not recognized");
            errors = errors + 1;
        end
        if (rect_seen || tri_seen || tri_rot_seen) begin
            $display("FAIL chain_circle_offgrid: ellipse also triggered rectangle/triangle classifiers");
            errors = errors + 1;
        end
        if (circ_seen && ((abs_int(circ_cx_seen - 32) > 10) ||
                          (abs_int(circ_cy_seen - 26) > 10) ||
                          (circ_rr_seen < 12) || (circ_rr_seen > 24))) begin
            $display("FAIL chain_circle_offgrid: bbox does not follow the 18/24 ellipse");
            errors = errors + 1;
        end
        write_ppm("out/chain_circle_offgrid.ppm");
        if (errors == err0)
            $display("TEST chain_circle_offgrid: PASS");
        else
            $display("TEST chain_circle_offgrid: FAIL");

        // ------------------------------------------------------------------
        // Test 5b: ellipse on a low-contrast checker background.  The target
        // must remain detectable without being classified as another shape.
        // ------------------------------------------------------------------
        prepare_test;
        fill_ellipse_on_checker8(32, 26, 12, 15);
        shape_en = 1'b1;
        capture_on = 1'b0;
        run_n_frames(7, 2300, 200);
        capture_on = 1'b1;
        run_frame(2300, 200);
        capture_on = 1'b0;
        tests_run = tests_run + 1;
        err0 = errors;

        $display("INFO chain_circle_clutter: hit_events=%0d valid=%b peak=%0d edges=%0d",
                 shape_hit_events, u_shape.circ_valid, u_shape.peak_val, chain_edge_pixels);
        $display("INFO chain_circle_clutter seen: rect=%0d circ=%0d tri=%0d trirot=%0d cand=%0d max_corner=%0d confirms=%0d bbox=(%0d,%0d) r=%0d",
                 rect_seen, circ_seen, tri_seen, tri_rot_seen, circ_cand_seen,
                 circ_corner_max_seen, circ_confirm_events,
                 circ_cx_seen, circ_cy_seen, circ_rr_seen);
        if (!circ_seen || (shape_hit_events == 0)) begin
            $display("FAIL chain_circle_clutter: ellipse on checker was not recognized");
            errors = errors + 1;
        end
        if (rect_seen || tri_seen || tri_rot_seen) begin
            $display("FAIL chain_circle_clutter: ellipse also triggered rectangle/triangle classifiers");
            errors = errors + 1;
        end
        if (circ_seen && ((abs_int(circ_cx_seen - 32) > 10) ||
                          (abs_int(circ_cy_seen - 26) > 10) ||
                          (circ_rr_seen < 8) || (circ_rr_seen > 20))) begin
            $display("FAIL chain_circle_clutter: bbox does not follow the 12/15 ellipse");
            errors = errors + 1;
        end
        write_ppm("out/chain_circle_clutter.ppm");
        if (errors == err0)
            $display("TEST chain_circle_clutter: PASS");
        else
            $display("TEST chain_circle_clutter: FAIL");

        // ------------------------------------------------------------------
        // Test 6: synthetic upright triangle.  Base y=36, width 16 and height
        // 32 fit both the line-run candidate and the slope validator.
        // ------------------------------------------------------------------
        prepare_test;
        fill_filled_triangle(32, 4, 36, 8);
        shape_en = 1'b1;
        capture_on = 1'b0;
        run_n_frames(5, 2300, 200);
        capture_on = 1'b1;
        run_frame(2300, 200);
        capture_on = 1'b0;
        tests_run = tests_run + 1;
        err0 = errors;

        $display("INFO chain_triangle: hit_events=%0d valid=%b peak=%0d edges=%0d cand=%b [%0d,%0d]-[%0d,%0d] hits=%0d/%0d need=%0d",
                 shape_hit_events, u_shape.tri_valid, u_shape.peak_val, chain_edge_pixels,
                 u_shape.tri_cand_active,
                 u_shape.tri_cand_x0, u_shape.tri_cand_y0,
                 u_shape.tri_cand_x1, u_shape.tri_cand_y1,
                 u_shape.tri_l_hits, u_shape.tri_r_hits, u_shape.tri_need);
        $display("INFO chain_triangle seen: rect=%0d circ=%0d tri=%0d trirot=%0d cand=%0d max_corner=%0d confirms=%0d",
                 rect_seen, circ_seen, tri_seen, tri_rot_seen, circ_cand_seen,
                 circ_corner_max_seen, circ_confirm_events);
        if (!tri_seen || (shape_hit_events == 0)) begin
            $display("FAIL chain_triangle: triangle was not recognized");
            errors = errors + 1;
        end
        if (rect_seen || circ_seen || tri_rot_seen) begin
            $display("FAIL chain_triangle: triangle also triggered rectangle/circle classifiers");
            errors = errors + 1;
        end
        write_ppm("out/chain_triangle.ppm");
        if (errors == err0)
            $display("TEST chain_triangle_positive: PASS");
        else
            $display("TEST chain_triangle_positive: FAIL");

        if (errors != 0) begin
            $display("RESULT: FAIL (%0d tests, %0d errors)", tests_run, errors);
            $fatal(1, "tb_edge_chain failed");
        end else begin
            $display("RESULT: PASS (%0d tests)", tests_run);
        end
        $finish;
    end

endmodule
