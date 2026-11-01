`timescale 1ns/1ps

module tb_sensor_gain;

    reg         clock = 1'b0;
    reg  [7:0]  addr_ptr = 8'd0;
    wire [24:0] rdata_out;
    integer     errors;

    i2c_master_reg_rom #(
        .ROM_SIZE        (25),
        .TOTAL_ROM_DEPTH (256),
        .ADDR_WIDTH      (8)
    ) dut (
        .clock    (clock),
        .addr_ptr (addr_ptr),
        .rdata_out(rdata_out)
    );

    always #5 clock = ~clock;

    task check_reg;
        input [7:0]  index;
        input [15:0] reg_addr;
        input [7:0]  value;
        begin
            @(negedge clock);
            addr_ptr = index;
            @(posedge clock);
            #1;
            if (rdata_out !== {reg_addr, value, 1'b0}) begin
                $display("FAIL index=%02h expected={%04h,%02h,0} got=%05h",
                         index, reg_addr, value, rdata_out);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        errors = 0;

        check_reg(8'h74, 16'h3e00, 8'h00);
        check_reg(8'h75, 16'h3e01, 8'hba);
        check_reg(8'h76, 16'h3e02, 8'hd0);
        check_reg(8'h91, 16'h0100, 8'h01);
        check_reg(8'h92, 16'h3e08, 8'h81);
        check_reg(8'h93, 16'h3e09, 8'h32);

        if (errors == 0)
            $display("RESULT: PASS (max exposure + 4.813x analog gain sequence)");
        else
            $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

endmodule
