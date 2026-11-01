`timescale 1ns/1ps

// SC431HAI 运行时曝光/增益写调度器
//
// 上电时仍由 i2c_master_reg_set 按 ROM 顺序写入完整初始化序列；本模块只在
// boot_done 之后接管写通道。每次用户调节使用 Group Hold 把多字节配置打包成
// 一组，等最后一条写完成后再释放，避免曝光/增益只更新一半造成画面跳变。
module cam_cfg_sched #(
    parameter [15:0] EXP_DEFAULT    = 16'h0BAD,
    parameter [7:0]  GAIN_C_DEFAULT = 8'h81,
    parameter [7:0]  GAIN_F_DEFAULT = 8'h32
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        init_done,       // 初始化 ROM 全部写完
    input  wire        wr_done,         // i2c_16addr_8data 单条写完成脉冲

    // 来自 HDMI 时钟域的 4 相握手请求；req 翻转后 payload 保持不变，直到 ack。
    input  wire        cfg_req_tgl,
    input  wire [1:0]  cfg_sel,         // 0=曝光, 1=增益, 2=曝光+增益/恢复默认
    input  wire [15:0] cfg_exp,
    input  wire [7:0]  cfg_gain_c,
    input  wire [7:0]  cfg_gain_f,
    output reg         cfg_ack_tgl,

    output reg         wr_en,
    output reg  [15:0] addr,
    output reg  [7:0]  din,
    output wire        busy
);

    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_SEND = 2'd1;
    localparam [1:0] S_WAIT = 2'd2;
    localparam [1:0] S_END  = 2'd3;

    reg        req_meta;
    reg        req_sync;
    reg        req_seen;
    reg [1:0]  state;
    reg [1:0]  job_sel;
    reg [15:0] job_exp;
    reg [7:0]  job_gain_c;
    reg [7:0]  job_gain_f;
    reg [3:0]  seq_idx;
    reg [3:0]  seq_len;

    reg [15:0] seq_addr;
    reg [7:0]  seq_din;

    assign busy = (state != S_IDLE);

    // 请求翻转信号跨到 CLK_5M 域。数据在翻转前沿已经稳定，等到这里看到
    // 翻转后再采样，属于典型的数据稳定型四相握手。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            req_meta <= 1'b0;
            req_sync <= 1'b0;
        end else begin
            req_meta <= cfg_req_tgl;
            req_sync <= req_meta;
        end
    end

    always @(*) begin
        seq_addr = 16'h3812;
        seq_din  = 8'h00;

        case (job_sel)
            2'd0: begin // 曝光
                case (seq_idx)
                    4'd0: begin seq_addr = 16'h3812; seq_din = 8'h00; end
                    4'd1: begin seq_addr = 16'h3e00; seq_din = {4'b0000, job_exp[15:12]}; end
                    4'd2: begin seq_addr = 16'h3e01; seq_din = job_exp[11:4]; end
                    4'd3: begin seq_addr = 16'h3e02; seq_din = {job_exp[3:0], 4'b0000}; end
                    4'd4: begin seq_addr = 16'h3812; seq_din = 8'h30; end
                    default: begin seq_addr = 16'h3812; seq_din = 8'h30; end
                endcase
            end
            2'd1: begin // 模拟增益
                case (seq_idx)
                    4'd0: begin seq_addr = 16'h3812; seq_din = 8'h00; end
                    4'd1: begin seq_addr = 16'h3e08; seq_din = job_gain_c; end
                    4'd2: begin seq_addr = 16'h3e09; seq_din = job_gain_f; end
                    4'd3: begin seq_addr = 16'h3812; seq_din = 8'h30; end
                    default: begin seq_addr = 16'h3812; seq_din = 8'h30; end
                endcase
            end
            default: begin // 曝光 + 增益，同时用于恢复默认值
                case (seq_idx)
                    4'd0: begin seq_addr = 16'h3812; seq_din = 8'h00; end
                    4'd1: begin seq_addr = 16'h3e00; seq_din = {4'b0000, job_exp[15:12]}; end
                    4'd2: begin seq_addr = 16'h3e01; seq_din = job_exp[11:4]; end
                    4'd3: begin seq_addr = 16'h3e02; seq_din = {job_exp[3:0], 4'b0000}; end
                    4'd4: begin seq_addr = 16'h3e08; seq_din = job_gain_c; end
                    4'd5: begin seq_addr = 16'h3e09; seq_din = job_gain_f; end
                    4'd6: begin seq_addr = 16'h3812; seq_din = 8'h30; end
                    default: begin seq_addr = 16'h3812; seq_din = 8'h30; end
                endcase
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            cfg_ack_tgl <= 1'b0;
            req_seen    <= 1'b0;
            wr_en       <= 1'b0;
            addr        <= 16'h0000;
            din         <= 8'h00;
            job_sel     <= 2'd0;
            job_exp     <= EXP_DEFAULT;
            job_gain_c  <= GAIN_C_DEFAULT;
            job_gain_f  <= GAIN_F_DEFAULT;
            seq_idx     <= 4'd0;
            seq_len     <= 4'd0;
        end else begin
            wr_en <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (init_done && (req_sync != req_seen)) begin
                        job_sel    <= cfg_sel;
                        job_exp    <= cfg_exp;
                        job_gain_c <= cfg_gain_c;
                        job_gain_f <= cfg_gain_f;
                        seq_idx    <= 4'd0;
                        if (cfg_sel == 2'd0)
                            seq_len <= 4'd5;
                        else if (cfg_sel == 2'd1)
                            seq_len <= 4'd4;
                        else
                            seq_len <= 4'd7;
                        state <= S_SEND;
                    end
                end

                S_SEND: begin
                    wr_en <= 1'b1;
                    addr  <= seq_addr;
                    din   <= seq_din;
                    state <= S_WAIT;
                end

                S_WAIT: begin
                    wr_en <= 1'b0;
                    if (wr_done) begin
                        if ((seq_idx + 1'b1) >= seq_len)
                            state <= S_END;
                        else begin
                            seq_idx <= seq_idx + 1'b1;
                            state   <= S_SEND;
                        end
                    end
                end

                S_END: begin
                    cfg_ack_tgl <= req_sync;
                    req_seen    <= req_sync;
                    state       <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
