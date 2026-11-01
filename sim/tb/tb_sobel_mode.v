`timescale 1ns/1ps

// Mode-4 visual regression.
//
// Phase A: full-screen original-colour source + red raw-Sobel contour.
//   The contour must composite over the camera image, never replace it with a
//   grayscale copy, and must stay off the TOP_BLANK rows.
//
// Phase B: a flat field with one faint 24-level block.
//   Recognition uses TH_LOW (2), so the block outline must reach o_edge.
//   The red overlay uses max(TH_HIGH + SOBEL_DISP_EXTRA, 24).  With TH_HIGH=5
//   this is 24, so the same
//   outline must never be painted.  This is what keeps skin texture from
//   turning the whole face pink.
//
// The pixel pipeline delays the image by whole lines.  Phase A measures that
// latency from the captured frame instead of hard-coding it.
module tb_sobel_mode;
    localparam integer H = 64;
    localparam integer V = 48;
    localparam integer N = H * V;
    localparam integer TOP_BLANK = 8;
    localparam [23:0] BG_COLOR = 24'h2050C0;
    localparam [23:0] FG_COLOR = 24'h20C0F0;
    localparam [23:0] LC_COLOR = 24'h80C0F0;   // +24 grey: tracked, never painted
    localparam [23:0] RED      = 24'h0000FF;
    localparam integer LOW_R0 = 18;
    localparam integer LOW_R1 = 34;
    localparam integer LOW_C0 = 18;
    localparam integer LOW_C1 = 46;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg i_de = 1'b0;
    reg i_hs = 1'b0;
    reg i_vs = 1'b0;
    reg [23:0] i_rgb = 24'd0;
    reg [7:0] i_th_low = 8'd2;
    reg [7:0] i_th_high = 8'd5;
    reg [1:0] i_prefilt = 2'd2;
    reg i_overlay = 1'b1;
    reg [10:0] i_split_col = 11'd0;
    reg [1:0] i_color_mode = 2'd0;
    reg i_pip = 1'b0;

    wire o_de, o_hs, o_vs, o_edge_flag, o_edge;
    wire [23:0] o_rgb;
    wire [3:0] o_dir16;
    wire [10:0] o_row;

    edge_proc #(
        .H_ACTIVE   (H),
        .SPLIT_COL  (H/2),
        .TH_LOW     (8'd2),
        .TH_HIGH    (8'd5),
        .TOP_BLANK  (TOP_BLANK),
        .COL_BLANK  (2),
        .V_ACTIVE   (V),
        .PIP_W      (16),
        .PIP_H      (16)
    ) dut (
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
        .i_shape_mode (1'b1),
        .o_de         (o_de),
        .o_hs         (o_hs),
        .o_vs         (o_vs),
        .o_rgb        (o_rgb),
        .o_edge_flag  (o_edge_flag),
        .o_edge       (o_edge),
        .o_dir16      (o_dir16),
        .o_row        (o_row)
    );

    always #5 clk = ~clk;

    reg [23:0] src [0:N-1];
    reg [23:0] cap [0:N-1];
    reg        cap_edge [0:N-1];
    integer cap_count;
    integer errors;
    integer idx, expected_idx, r, c, cap_col, cap_row;
    integer red_count, color_count, color_mismatch, right_color, top_nonblack;
    integer edge_count, edge_without_red, red_without_edge;
    integer try_shift, try_mismatch, best_shift, best_mismatch;
    integer faint_red, faint_edge;
    integer fd;
    reg capture_on;
    reg o_vs_d;

    always @(negedge clk) begin
        if (rst_n) begin
            o_vs_d <= o_vs;
            if (o_de && capture_on) begin
                cap_col = (dut.o_col == 0) ? (H - 1) : (dut.o_col - 1);
                cap_row = o_row;
                if ((cap_row < V) && (cap_col < H)) begin
                    cap[cap_row * H + cap_col] = o_rgb;
                    cap_edge[cap_row * H + cap_col] = o_edge;
                    cap_count = cap_count + 1;
                end
            end
        end
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
        integer k;
        begin
            for (k = 0; k < 2048; k = k + 1) begin
                dut.lb0_mem[k]  = 8'd0;
                dut.lb1_mem[k]  = 8'd0;
                dut.flb0_mem[k] = 8'd0;
                dut.flb1_mem[k] = 8'd0;
                dut.flb2_mem[k] = 8'd0;
                dut.flb3_mem[k] = 8'd0;
                dut.mb0_mem[k]  = 18'd0;
                dut.mb1_mem[k]  = 18'd0;
                dut.sb0_mem[k]  = 2'd0;
                dut.sb1_mem[k]  = 2'd0;
                dut.hb_mem[k]   = 1'b0;
                dut.out_line_mem[k] = 41'd0;
            end
        end
    endtask

    task clear_capture;
        integer k;
        begin
            for (k = 0; k < N; k = k + 1) begin
                cap[k] = 24'd0;
                cap_edge[k] = 1'b0;
            end
            cap_count = 0;
        end
    endtask

    task run_frame;
        integer rr, cc;
        begin
            @(negedge clk);
            i_vs = 1'b1; i_de = 1'b0; i_hs = 1'b0; i_rgb = 24'd0;
            @(negedge clk);
            i_vs = 1'b0;
            repeat (80) @(negedge clk);

            for (rr = 0; rr < V; rr = rr + 1) begin
                for (cc = 0; cc < H; cc = cc + 1) begin
                    @(negedge clk);
                    i_de = 1'b1;
                    i_hs = (cc == 0);
                    i_rgb = src[rr * H + cc];
                end
                @(negedge clk);
                i_de = 1'b0; i_hs = 1'b0; i_rgb = 24'd0;
            end
            repeat (300) @(negedge clk);
        end
    endtask

    initial begin
        errors = 0;
        cap_count = 0;
        capture_on = 1'b0;
        o_vs_d = 1'b0;

        //---------------------------------------------------------------------
        // Phase A: colour background + high-contrast rectangle + faint block
        //---------------------------------------------------------------------
        for (idx = 0; idx < N; idx = idx + 1)
            src[idx] = BG_COLOR;
        for (r = 14; r <= 38; r = r + 1)
            for (c = 12; c <= 52; c = c + 1)
                src[r * H + c] = FG_COLOR;
        for (r = LOW_R0; r <= LOW_R1; r = r + 1)
            for (c = LOW_C0; c <= LOW_C1; c = c + 1)
                src[r * H + c] = LC_COLOR;
        // Three-line marker in the background: makes the measured line latency
        // unambiguous (a uniform background matches several shifts).
        for (r = 10; r <= 12; r = r + 1)
            for (c = 0; c < H; c = c + 1)
                src[r * H + c] = 24'h103050;

        clear_rams;
        do_reset;
        run_frame;
        clear_capture;
        capture_on = 1'b1;
        run_frame;
        capture_on = 1'b0;

        best_shift = 0;
        best_mismatch = 32'h7FFFFFFF;
        for (try_shift = 0; try_shift <= 8; try_shift = try_shift + 1) begin
            try_mismatch = 0;
            for (idx = 0; idx < N; idx = idx + 1) begin
                r = idx / H;
                c = idx % H;
                if ((r >= try_shift + TOP_BLANK) && (cap[idx] !== RED)) begin
                    expected_idx = (r - try_shift) * H + c;
                    if (cap[idx] !== src[expected_idx])
                        try_mismatch = try_mismatch + 1;
                end
            end
            if (try_mismatch < best_mismatch) begin
                best_mismatch = try_mismatch;
                best_shift = try_shift;
            end
        end

        red_count = 0; color_count = 0; color_mismatch = 0;
        right_color = 0; top_nonblack = 0;
        edge_count = 0; edge_without_red = 0; red_without_edge = 0;

        for (idx = 0; idx < N; idx = idx + 1) begin
            r = idx / H;
            c = idx % H;

            if (cap[idx] === RED) begin
                red_count = red_count + 1;
            end else if (r >= best_shift + TOP_BLANK) begin
                expected_idx = (r - best_shift) * H + c;
                if (cap[idx] === src[expected_idx])
                    color_count = color_count + 1;
                else
                    color_mismatch = color_mismatch + 1;
            end

            if ((r >= TOP_BLANK) && (c >= H/2) && (cap[idx] === BG_COLOR))
                right_color = right_color + 1;
            if ((r < TOP_BLANK) && (cap[idx] !== 24'h000000))
                top_nonblack = top_nonblack + 1;

            if (cap_edge[idx])
                edge_count = edge_count + 1;
            if (cap_edge[idx] && (cap[idx] !== RED))
                edge_without_red = edge_without_red + 1;
            if (!cap_edge[idx] && (cap[idx] === RED))
                red_without_edge = red_without_edge + 1;
        end

        fd = $fopen("E:/FPGA/FPGA_MAX/sim/out/sobel_mode4.ppm", "wb");
        $fwrite(fd, "P6\n%0d %0d\n255\n", H, V);
        for (idx = 0; idx < N; idx = idx + 1)
            $fwrite(fd, "%c%c%c", cap[idx][7:0], cap[idx][15:8], cap[idx][23:16]);
        $fclose(fd);

        if (cap_count != N) begin
            $display("FAIL sobel_mode: captured %0d pixels, expected %0d", cap_count, N);
            errors = errors + 1;
        end
        if ((best_mismatch != 0) || (best_shift < 1) || (best_shift > 8)) begin
            $display("FAIL sobel_mode: line latency not found (shift=%0d mismatch=%0d)",
                     best_shift, best_mismatch);
            errors = errors + 1;
        end
        if (color_mismatch != 0) begin
            $display("FAIL sobel_mode: %0d non-red pixels differ from the camera colour", color_mismatch);
            errors = errors + 1;
        end
        if (color_count < 1200) begin
            $display("FAIL sobel_mode: only %0d original-colour background pixels", color_count);
            errors = errors + 1;
        end
        if (red_count < 80) begin
            $display("FAIL sobel_mode: only %0d red Sobel pixels", red_count);
            errors = errors + 1;
        end
        if (right_color < 500) begin
            $display("FAIL sobel_mode: right half lost the original colour background (colour=%0d)", right_color);
            errors = errors + 1;
        end
        if (top_nonblack != 0) begin
            $display("FAIL sobel_mode: %0d non-black pixels in TOP_BLANK", top_nonblack);
            errors = errors + 1;
        end
        if (red_without_edge != 0) begin
            $display("FAIL sobel_mode: %0d red pixels are not recognised edges", red_without_edge);
            errors = errors + 1;
        end
        if (edge_without_red < 8) begin
            $display("FAIL sobel_mode: display threshold removed no faint edges (edge_no_red=%0d)",
                     edge_without_red);
            errors = errors + 1;
        end

        //---------------------------------------------------------------------
        // Phase B: flat field + faint block only.  Any red pixel here means the
        // display threshold followed the recognition threshold.
        //---------------------------------------------------------------------
        for (idx = 0; idx < N; idx = idx + 1)
            src[idx] = FG_COLOR;
        for (r = 20; r <= 30; r = r + 1)
            for (c = 22; c <= 42; c = c + 1)
                src[r * H + c] = LC_COLOR;

        clear_rams;
        do_reset;
        run_frame;
        clear_capture;
        capture_on = 1'b1;
        run_frame;
        capture_on = 1'b0;

        faint_red = 0;
        faint_edge = 0;
        for (idx = 0; idx < N; idx = idx + 1) begin
            r = idx / H;
            c = idx % H;
            if ((r >= 10) && (r <= 38) && (c >= 10) && (c <= 54)) begin
                if (cap[idx] === RED)
                    faint_red = faint_red + 1;
                if (cap_edge[idx])
                    faint_edge = faint_edge + 1;
            end
        end

        if (faint_red != 0) begin
            $display("FAIL sobel_mode: faint texture painted red on %0d pixels", faint_red);
            errors = errors + 1;
        end
        if (faint_edge < 8) begin
            $display("FAIL sobel_mode: faint texture not recognised (edge=%0d)", faint_edge);
            errors = errors + 1;
        end

        $display("INFO sobel_mode: shift=%0d red=%0d edge=%0d edge_no_red=%0d red_no_edge=%0d colour=%0d right_colour=%0d colour_mismatch=%0d top_nonblack=%0d faint_red=%0d faint_edge=%0d",
                 best_shift, red_count, edge_count, edge_without_red, red_without_edge,
                 color_count, right_color, color_mismatch, top_nonblack, faint_red, faint_edge);
        if (errors == 0)
            $display("RESULT: PASS (sobel_mode)");
        else
            $display("RESULT: FAIL (sobel_mode, %0d errors)", errors);
        $finish;
    end
endmodule
