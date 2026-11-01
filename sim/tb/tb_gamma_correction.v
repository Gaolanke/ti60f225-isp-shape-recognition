`timescale 1ns/1ps

// Bit-exact regression for the 8-bit synchronous gamma ROM path.

module tb_gamma_correction;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg i_valid = 1'b0;
    reg i_hs = 1'b0;
    reg i_vs = 1'b0;
    reg [7:0] i_red = 8'd0;
    reg [7:0] i_green = 8'd0;
    reg [7:0] i_blue = 8'd0;

    wire [7:0] o_red, o_green, o_blue;
    wire o_valid, o_hs, o_vs;

    reg [7:0] exp_table [0:255];
    integer errors;
    integer x;
    integer expected_r;
    integer expected_g;
    integer expected_b;

    gamma_correction #(
        .DATA_WIDTH  (8),
        .GAMMA_CURVE ("../rtl/gamma_conrrection/gamma_2p2_8bit.mem")
    ) dut (
        .i_pclk  (clk),
        .i_rstn  (rst_n),
        .i_valid (i_valid),
        .i_hs    (i_hs),
        .i_vs    (i_vs),
        .i_red   (i_red),
        .i_green (i_green),
        .i_blue  (i_blue),
        .o_red   (o_red),
        .o_green (o_green),
        .o_blue  (o_blue),
        .o_valid (o_valid),
        .o_hs    (o_hs),
        .o_vs    (o_vs)
    );

    always #5 clk = ~clk;

    initial begin
        $readmemh("../rtl/gamma_conrrection/gamma_2p2_8bit.mem", exp_table);
    end

    task check_cycle;
        input exp_valid;
        input exp_hs;
        input exp_vs;
        input integer er;
        input integer eg;
        input integer eb;
        begin
            @(posedge clk);
            #1;
            if (o_valid !== exp_valid) begin
                $display("FAIL gamma: valid=%b expected %b", o_valid, exp_valid);
                errors = errors + 1;
            end
            if (o_hs !== exp_hs) begin
                $display("FAIL gamma: hs=%b expected %b", o_hs, exp_hs);
                errors = errors + 1;
            end
            if (o_vs !== exp_vs) begin
                $display("FAIL gamma: vs=%b expected %b", o_vs, exp_vs);
                errors = errors + 1;
            end
            if (o_red !== er[7:0]) begin
                $display("FAIL gamma: R=%02h expected %02h", o_red, er[7:0]);
                errors = errors + 1;
            end
            if (o_green !== eg[7:0]) begin
                $display("FAIL gamma: G=%02h expected %02h", o_green, eg[7:0]);
                errors = errors + 1;
            end
            if (o_blue !== eb[7:0]) begin
                $display("FAIL gamma: B=%02h expected %02h", o_blue, eb[7:0]);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        errors = 0;

        repeat (3) @(negedge clk);
        rst_n = 1'b1;

        // One invalid beat.  The registered outputs must go to zero while
        // valid/hs/vs follow the input with one cycle of latency.
        @(negedge clk);
        i_valid = 1'b0;
        i_hs = 1'b1;
        i_vs = 1'b1;
        i_red = 8'hA5; i_green = 8'h5A; i_blue = 8'h3C;
        check_cycle(1'b0, 1'b1, 1'b1, 0, 0, 0);

        // Sweep every possible 8-bit value on all three channels.  The three
        // addresses differ, proving the three ROM read ports are independent.
        for (x = 0; x < 256; x = x + 1) begin
            @(negedge clk);
            i_valid = 1'b1;
            i_hs = ((x & 1) != 0);
            i_vs = ((x & 2) != 0);
            i_red   = x[7:0];
            i_green = (255 - x);
            i_blue  = (x ^ 8'h5A);
            expected_r = exp_table[x];
            expected_g = exp_table[255 - x];
            expected_b = exp_table[x ^ 8'h5A];
            check_cycle(1'b1, ((x & 1) != 0), ((x & 2) != 0),
                        expected_r, expected_g, expected_b);
        end

        // Back-to-back invalid beat clears the data again without disturbing
        // the already-registered timing controls.
        @(negedge clk);
        i_valid = 1'b0;
        i_hs = 1'b0;
        i_vs = 1'b1;
        i_red = 8'hFF; i_green = 8'h00; i_blue = 8'hFF;
        check_cycle(1'b0, 1'b0, 1'b1, 0, 0, 0);

        if (errors == 0)
            $display("RESULT: PASS (gamma 8-bit ROM, all 256 values x 3 channels)");
        else
            $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

endmodule
