`timescale 1ns/1ps

// BGGR debayer regression. Two Bayer samples are packed per clock: even x in
// the low byte and odd x in the high byte. Broad colour fields expose the
// half-row phase error that made the whole camera image look green.

module tb_debayer_bg;

    localparam integer W  = 64;
    localparam integer WB = W / 2;
    localparam integer H  = 16;
    localparam integer LINE_GAP  = 6;
    localparam integer FRAME_GAP = 24;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg raw_vs = 1'b0;
    reg raw_hs = 1'b0;
    reg raw_de = 1'b0;
    reg [15:0] raw_data = 16'd0;
    reg [15:0] gain_r = 16'd256;
    reg [15:0] gain_g = 16'd256;
    reg [15:0] gain_b = 16'd256;

    wire rgb_vs, rgb_hs, rgb_de, rgb_valid;
    wire [15:0] rgb_r, rgb_g, rgb_b;
    wire [47:0] rgb_datax2;

    integer errors;
    integer tests_run;
    integer sample_count;
    integer sum_r;
    integer sum_g;
    integer sum_b;
    integer out_line;
    reg out_de_d;

    debayer_top_2to1 dut (
        .in_pclk      (clk),
        .in_rstn      (rst_n),
        .raw_vs_i     (raw_vs),
        .raw_hs_i     (raw_hs),
        .raw_de_i     (raw_de),
        .raw_valid_i  (raw_de),
        .raw_datax4_i (raw_data),
        .r_gain       (gain_r),
        .g_gain       (gain_g),
        .b_gain       (gain_b),
        .rgb_vs_o     (rgb_vs),
        .rgb_hs_o     (rgb_hs),
        .rgb_de_o     (rgb_de),
        .rgb_valid_o  (rgb_valid),
        .rgb_bout     (rgb_b),
        .rgb_gout     (rgb_g),
        .rgb_rout     (rgb_r),
        .rgb_datax2_o (rgb_datax2)
    );

    always #5 clk = ~clk;

    always @(negedge clk) begin
        if (!rst_n) begin
            out_de_d <= 1'b0;
        end else begin
            out_de_d <= rgb_de;
            if (!out_de_d && rgb_de)
                out_line = out_line + 1;

            if (rgb_de && (out_line >= 3) && (out_line < (H - 3)) &&
                (dut.inst_raw_to_rgb.o_x_cnt >= 6) &&
                (dut.inst_raw_to_rgb.o_x_cnt < (WB - 6))) begin
                sum_r = sum_r + rgb_r[7:0] + rgb_r[15:8];
                sum_g = sum_g + rgb_g[7:0] + rgb_g[15:8];
                sum_b = sum_b + rgb_b[7:0] + rgb_b[15:8];
                sample_count = sample_count + 2;
            end
        end
    end

    task clear_stats;
        begin
            sample_count = 0;
            sum_r = 0;
            sum_g = 0;
            sum_b = 0;
            out_line = 0;
            out_de_d = 1'b0;
        end
    endtask

    task check_gain_mapping;
        integer avg_r;
        integer avg_g;
        integer avg_b;
        begin
            tests_run = tests_run + 1;
            gain_r = 16'd320;   // 1.25x
            gain_g = 16'd256;   // 1.00x
            gain_b = 16'd192;   // 0.75x
            drive_frame(3);

            if (sample_count < 128) begin
                $display("FAIL debayer gain: only %0d samples", sample_count);
                errors = errors + 1;
            end else begin
                avg_r = sum_r / sample_count;
                avg_g = sum_g / sample_count;
                avg_b = sum_b / sample_count;
                $display("DBG debayer gain white 1.25/1.00/0.75: R/G/B=%0d/%0d/%0d",
                         avg_r, avg_g, avg_b);
                if ((avg_r < (avg_g + 25)) || (avg_r > (avg_g + 65)) ||
                    (avg_b > (avg_g - 25)) || (avg_b < (avg_g - 65))) begin
                    $display("FAIL debayer gain: BGGR gain mapping is crossed");
                    errors = errors + 1;
                end
            end
            gain_r = 16'd256;
            gain_g = 16'd256;
            gain_b = 16'd256;
        end
    endtask

    task drive_frame;
        input integer mode;
        integer y;
        integer bx;
        integer x;
        integer rr;
        integer gg;
        integer bb;
        reg [7:0] sample_even;
        reg [7:0] sample_odd;
        begin
            @(negedge clk);
            raw_vs = 1'b0;
            raw_hs = 1'b0;
            raw_de = 1'b0;
            raw_data = 16'd0;
            repeat (FRAME_GAP) @(negedge clk);
            clear_stats;

            for (y = 0; y < H; y = y + 1) begin
                for (bx = 0; bx < WB; bx = bx + 1) begin
                    x = bx * 2;
                    case (mode)
                        0: begin rr = 200; gg = 80;  bb = 20;  end
                        1: begin rr = 20;  gg = 80;  bb = 200; end
                        2: begin rr = 20;  gg = 200; bb = 20;  end
                        default: begin rr = 180; gg = 180; bb = 180; end
                    endcase
                    if ((y & 1) == 0) begin
                        sample_even = bb;
                        sample_odd  = gg;
                    end else begin
                        sample_even = gg;
                        sample_odd  = rr;
                    end
                    @(negedge clk);
                    raw_vs   = 1'b1;
                    raw_hs   = 1'b1;
                    raw_de   = 1'b1;
                    raw_data = {sample_odd, sample_even};
                end
                @(negedge clk);
                raw_de = 1'b0;
                raw_hs = 1'b0;
                raw_data = 16'd0;
                repeat (LINE_GAP) @(negedge clk);
            end

            @(negedge clk);
            raw_vs = 1'b0;
            raw_de = 1'b0;
            repeat (FRAME_GAP) @(negedge clk);
        end
    endtask

    task check_field;
        input integer mode;
        input [8*6-1:0] name;
        integer avg_r;
        integer avg_g;
        integer avg_b;
        integer drg;
        integer dgb;
        begin
            tests_run = tests_run + 1;
            drive_frame(mode);

            if (sample_count < 128) begin
                $display("FAIL debayer %0s: only %0d samples", name, sample_count);
                errors = errors + 1;
            end else begin
                avg_r = sum_r / sample_count;
                avg_g = sum_g / sample_count;
                avg_b = sum_b / sample_count;
                drg = avg_r - avg_g;
                dgb = avg_g - avg_b;
                $display("DBG debayer %0s: samples=%0d avg R/G/B=%0d/%0d/%0d",
                         name, sample_count, avg_r, avg_g, avg_b);

                if (mode == 0) begin
                    if ((avg_r < (avg_g + 40)) || (avg_r < (avg_b + 80))) begin
                        $display("FAIL debayer red: expected R dominant");
                        errors = errors + 1;
                    end
                end else if (mode == 1) begin
                    if ((avg_b < (avg_g + 40)) || (avg_b < (avg_r + 80))) begin
                        $display("FAIL debayer blue: expected B dominant");
                        errors = errors + 1;
                    end
                end else if (mode == 2) begin
                    if ((avg_g < (avg_r + 80)) || (avg_g < (avg_b + 80))) begin
                        $display("FAIL debayer green: expected G dominant");
                        errors = errors + 1;
                    end
                end else if (mode == 3) begin
                    if (((drg > 16) || (drg < -16)) ||
                        ((dgb > 16) || (dgb < -16)) || (avg_g < 100)) begin
                        $display("FAIL debayer white: expected neutral gray");
                        errors = errors + 1;
                    end
                end
            end
        end
    endtask

    initial begin
        errors = 0;
        tests_run = 0;
        clear_stats();
        raw_vs = 1'b0;
        raw_hs = 1'b0;
        raw_de = 1'b0;
        raw_data = 16'd0;

        rst_n = 1'b0;
        repeat (20) @(negedge clk);
        rst_n = 1'b1;
        repeat (20) @(negedge clk);

        check_field(0, "red");
        check_field(1, "blue");
        check_field(2, "green");
        check_field(3, "white");
        check_gain_mapping();

        if (errors == 0) begin
            $display("RESULT: PASS (debayer BGGR, %0d colour/gain tests)", tests_run);
            $finish;
        end else begin
            $display("RESULT: FAIL (%0d tests, %0d errors)", tests_run, errors);
            $fatal(1, "tb_debayer_bg failed");
        end
    end

endmodule
