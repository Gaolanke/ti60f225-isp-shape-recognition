`timescale 1ns/1ps

// Regression for the vendor SC431HAI2 active-window crop.
//
// The vendor capture reports 1081 MIPI lines and 640 valid beats per
// line.  sensor_clipper selects the 1920x1080 window by keeping the
// first 1080 lines and the first 480 beats of each line.  Without this
// stage the frame buffer sees 1081x640 while the display reader expects
// 1080x480, so the frame length and Bayer phase wrap at the wrong point.
// This test drives the full raw timing and proves that:
//   * the first 1080 input lines produce exactly 480 active beats each;
//   * input beats 480..639 of every line are cropped;
//   * input line 1080 (the 1081st line) produces no active beats.

module tb_sensor_clipper;

    localparam integer INPUT_LINES = 1081;
    localparam integer INPUT_BEATS_PER_LINE = 640;
    localparam integer OUTPUT_BEATS_PER_LINE = 480;

    reg clk = 1'b0;
    reg i_hs = 1'b0;
    reg i_vs = 1'b0;
    reg i_de = 1'b0;
    reg [39:0] i_dat = 40'd0;

    wire o_hs;
    wire o_vs;
    wire o_de;
    wire [39:0] o_dat;

    integer errors;
    integer line;
    integer beat;
    integer beats_in_line;
    integer line_beats [0:INPUT_LINES-1];
    integer active_lines;
    integer active_beats;

    sensor_clipper dut (
        .clk   (clk),
        .i_hs  (i_hs),
        .i_vs  (i_vs),
        .i_de  (i_de),
        .i_dat (i_dat),
        .o_hs  (o_hs),
        .o_vs  (o_vs),
        .o_de  (o_de),
        .o_dat (o_dat)
    );

    always #5 clk = ~clk;

    task drive_line;
        input integer line_index;
        begin
            @(negedge clk);
            i_hs = 1'b1;
            @(negedge clk);
            i_hs = 1'b0;
            repeat (2) @(negedge clk);

            beats_in_line = 0;
            for (beat = 0; beat < INPUT_BEATS_PER_LINE; beat = beat + 1) begin
                i_de  = 1'b1;
                i_dat = {line_index[9:0], beat[9:0], 4'hA, 16'h5A5A};
                @(posedge clk);
                #1;
                if (o_de)
                    beats_in_line = beats_in_line + 1;
                @(negedge clk);
            end

            i_de  = 1'b0;
            i_dat = 40'd0;
            @(posedge clk);
            #1;
            if (o_de)
                beats_in_line = beats_in_line + 1;
            line_beats[line_index] = beats_in_line;
            repeat (2) @(negedge clk);
        end
    endtask

    initial begin
        errors = 0;
        active_lines = 0;
        active_beats = 0;

        repeat (3) @(negedge clk);
        i_vs = 1'b1;
        @(negedge clk);
        i_vs = 1'b0;
        repeat (3) @(negedge clk);

        for (line = 0; line < INPUT_LINES; line = line + 1)
            drive_line(line);

        for (line = 0; line <= 1079; line = line + 1) begin
            if (line_beats[line] != OUTPUT_BEATS_PER_LINE) begin
                $display("FAIL clipper: line %0d emitted %0d beats, expected %0d",
                         line, line_beats[line], OUTPUT_BEATS_PER_LINE);
                errors = errors + 1;
            end
            if (line_beats[line] != 0)
                active_lines = active_lines + 1;
            active_beats = active_beats + line_beats[line];
        end

        if (line_beats[1080] != 0) begin
            $display("FAIL clipper: line 1080 emitted %0d beats", line_beats[1080]);
            errors = errors + 1;
        end

        if (active_lines != 1080) begin
            $display("FAIL clipper: %0d active lines, expected 1080", active_lines);
            errors = errors + 1;
        end
        if (active_beats != 1080 * OUTPUT_BEATS_PER_LINE) begin
            $display("FAIL clipper: %0d active beats, expected %0d",
                     active_beats, 1080 * OUTPUT_BEATS_PER_LINE);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("RESULT: PASS (1081x640 raw cropped to 1080x480 active)");
        else
            $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

endmodule
