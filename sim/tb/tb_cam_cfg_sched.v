`timescale 1ns/1ps

module tb_cam_cfg_sched;

    reg         clk = 1'b0;
    reg         rst_n = 1'b0;
    reg         init_done = 1'b0;
    reg         cfg_req_tgl = 1'b0;
    reg  [1:0]  cfg_sel = 2'd0;
    reg  [15:0] cfg_exp = 16'h0BAD;
    reg  [7:0]  cfg_gain_c = 8'h81;
    reg  [7:0]  cfg_gain_f = 8'h32;

    wire        cfg_ack_tgl;
    wire        wr_en;
    wire [15:0] addr;
    wire [7:0]  din;
    wire        busy;

    reg         wr_done = 1'b0;
    reg  [2:0]  ack_delay = 3'd0;

    reg [15:0] exp_addr [0:7];
    reg [7:0]  exp_data [0:7];
    integer    exp_len = 0;
    integer    cap_count = 0;
    integer    errors = 0;
    reg        ack_prev = 1'b0;

    cam_cfg_sched dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .init_done   (init_done),
        .wr_done     (wr_done),
        .cfg_req_tgl (cfg_req_tgl),
        .cfg_sel     (cfg_sel),
        .cfg_exp     (cfg_exp),
        .cfg_gain_c  (cfg_gain_c),
        .cfg_gain_f  (cfg_gain_f),
        .cfg_ack_tgl (cfg_ack_tgl),
        .wr_en       (wr_en),
        .addr        (addr),
        .din         (din),
        .busy        (busy)
    );

    always #5 clk = ~clk;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_done   <= 1'b0;
            ack_delay <= 3'd0;
        end else begin
            wr_done <= 1'b0;
            if (wr_en) begin
                ack_delay <= 3'd1;
            end else if (ack_delay != 3'd0) begin
                if (ack_delay == 3'd3) begin
                    ack_delay <= 3'd0;
                    wr_done   <= 1'b1;
                end else begin
                    ack_delay <= ack_delay + 1'b1;
                end
            end
        end
    end

    // capture marker
    always @(posedge clk) begin
        if (wr_en) begin
            if (cap_count < exp_len) begin
                if (addr !== exp_addr[cap_count] || din !== exp_data[cap_count])
                    errors = errors + 1;
            end else begin
                errors = errors + 1;
            end
            cap_count = cap_count + 1;
        end
    end

    task prepare_expect;
        input [1:0]  sel;
        input [15:0] exp;
        input [7:0]  gc;
        input [7:0]  gf;
        begin
            exp_addr[0] = 16'h3812; exp_data[0] = 8'h00;
            if (sel == 2'd0) begin
                exp_len = 5;
                exp_addr[1] = 16'h3e00; exp_data[1] = {4'b0000, exp[15:12]};
                exp_addr[2] = 16'h3e01; exp_data[2] = exp[11:4];
                exp_addr[3] = 16'h3e02; exp_data[3] = {exp[3:0], 4'b0000};
                exp_addr[4] = 16'h3812; exp_data[4] = 8'h30;
            end else if (sel == 2'd1) begin
                exp_len = 4;
                exp_addr[1] = 16'h3e08; exp_data[1] = gc;
                exp_addr[2] = 16'h3e09; exp_data[2] = gf;
                exp_addr[3] = 16'h3812; exp_data[3] = 8'h30;
            end else begin
                exp_len = 7;
                exp_addr[1] = 16'h3e00; exp_data[1] = {4'b0000, exp[15:12]};
                exp_addr[2] = 16'h3e01; exp_data[2] = exp[11:4];
                exp_addr[3] = 16'h3e02; exp_data[3] = {exp[3:0], 4'b0000};
                exp_addr[4] = 16'h3e08; exp_data[4] = gc;
                exp_addr[5] = 16'h3e09; exp_data[5] = gf;
                exp_addr[6] = 16'h3812; exp_data[6] = 8'h30;
            end
        end
    endtask
    // @(negedge clk)

    task wait_ack;
        begin
            wait (cfg_ack_tgl != ack_prev);
            ack_prev = cfg_ack_tgl;
            @(posedge clk);
        end
    endtask

    task issue_job;
        input [1:0]  sel;
        input [15:0] exp;
        input [7:0]  gc;
        input [7:0]  gf;
        begin
            prepare_expect(sel, exp, gc, gf);
            cap_count = 0;
            cfg_sel    = sel;
            cfg_exp    = exp;
            cfg_gain_c = gc;
            cfg_gain_f = gf;
            @(negedge clk);
            cfg_req_tgl = ~cfg_req_tgl;
            wait (cap_count == exp_len);
            wait_ack;
        end
    endtask

    initial begin
        #50;
        rst_n = 1'b1;
        #30;
        prepare_expect(2'd0, 16'h1234, 8'h81, 8'h32);
        cfg_sel    = 2'd0;
        cfg_exp    = 16'h1234;
        cfg_gain_c = 8'h81;
        cfg_gain_f = 8'h32;
        @(negedge clk);
        cfg_req_tgl = ~cfg_req_tgl;
        #200;
        if (cap_count != 0 || wr_en) errors = errors + 1;
        init_done = 1'b1;
        cap_count = 0;
        wait (cap_count == 5);
        wait_ack;
        issue_job(2'd0, 16'h2345, 8'h81, 8'h32);
        issue_job(2'd1, 16'h0BAD, 8'h83, 8'h30);
        issue_job(2'd2, 16'h0BAD, 8'h81, 8'h32);
        issue_job(2'd1, 16'h0BAD, 8'h81, 8'h32);
        issue_job(2'd0, 16'h0BAD, 8'h81, 8'h32);
        if (busy) errors = errors + 1;
        if (errors == 0) $display("RESULT: PASS (SC431HAI runtime exposure/gain group-hold)");
        else $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin
        #50000;
        $display("RESULT: FAIL (timeout)");
        $finish;
    end

endmodule
