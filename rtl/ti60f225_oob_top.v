
/////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2013-2021 Efinix Inc. All rights reserved.
//
// Description:
// Example top file for ti60f225 dev kit OOB design
//
// Language:  Verilog 2001
//Ti60f225_sc431hai2hdmi_v6：
//使用了最新版本的framebuffer_v3;
// ------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////
//`define SOFT_TAP 1
`include "ddr3_controller/ddr3_parameter.vh"
// `include "define.v"

module ti60f225_oob_top #(

	parameter                       RANK_RATIO         = 1,       // # of unique CS outputs per rank
	parameter                       ASYN_AXI_CLK       = `ASYN_AXI_CLK, 
	parameter                       RANKS              = `RANKS,
	parameter                       CK_WIDTH           = `CK_WIDTH,       // # of CK/CK# outputs to memory   
	parameter                       CKE_WIDTH          = `CKE_WIDTH,       // # of cke outputs
	parameter                       CS_WIDTH           = `CS_WIDTH,       // # of unique CS outputs
	parameter                       BANK_WIDTH         = `BANK_WIDTH,       // # of bank bits
	parameter                       ROW_WIDTH          = `ROW_WIDTH,       // DRAM address bus width
	parameter                       COL_WIDTH          = `COL_WIDTH,      // column address width
	parameter                       DM_WIDTH           = `DM_WIDTH,       // # of DM (data mask)
	parameter                       DQS_WIDTH          = `DQS_WIDTH,       // # of DQS (strobe)
	parameter                       DQ_WIDTH           = `DQ_WIDTH,      // # of DQ (data)
	parameter                       ODT_WIDTH          = `ODT_WIDTH,
	parameter                       DQ_CNT_WIDTH       = `DQ_CNT_WIDTH,       // = ceil(log2(DQ_WIDTH))
	parameter                       DQS_CNT_WIDTH      = `DQS_CNT_WIDTH,       // = ceil(log2(DQS_WIDTH))  
	parameter                       DRAM_WIDTH         = `DRAM_WIDTH,       // # of DQ per DQS   
	parameter                       DATA_WIDTH         = `DATA_WIDTH,
	parameter                       ADDR_WIDTH         = `ADDR_WIDTH,    
	parameter                       AXI_ID_WIDTH       = `AXI_ID_WIDTH,
	parameter                       AXI_ADDR_WIDTH     = `AXI_ADDR_WIDTH,
	parameter                       AXI_DATA_WIDTH     = `AXI_DATA_WIDTH,
	parameter [7:0]                 SOBEL_TH_LOW       = 8'd2,
	parameter [7:0]                 SOBEL_TH_HIGH      = 8'd5,
	parameter                       SOBEL_GRAY_SHIFTADD = 1,
	parameter                       SOBEL_PREFILT      = 2'd0    // 0=中值 1=高斯 2=直通(运行时可按键切换)
)
(

  (* syn_peri_port = 0 *) input jtag_inst1_CAPTURE,
  (* syn_peri_port = 0 *) input jtag_inst1_DRCK,
  (* syn_peri_port = 0 *) input jtag_inst1_RESET,
  (* syn_peri_port = 0 *) input jtag_inst1_RUNTEST,
  (* syn_peri_port = 0 *) input jtag_inst1_SEL,
  (* syn_peri_port = 0 *) input jtag_inst1_SHIFT,
  (* syn_peri_port = 0 *) input jtag_inst1_TCK,
  (* syn_peri_port = 0 *) input jtag_inst1_TDI,
  (* syn_peri_port = 0 *) input jtag_inst1_TMS,
  (* syn_peri_port = 0 *) input jtag_inst1_UPDATE,
  (* syn_peri_port = 0 *) output jtag_inst1_TDO,

   //Clocks 
	input	wire	i_arstn,
    input	wire	i_mipi_rx_pclk,
    input wire  vid_clk_dvi2,
    input wire hdmi_tx_slow_clk,
    input wire CLK_5M,
    input  wire CLK_25M,
	output  wire    pll_inst1_RSTN,
	input	wire	i_pll_locked,
	input 	wire  	pll_locked,
	output  wire    pll_inst4_RSTN,
	input   wire    pll_inst4_LOCKED,
	output  wire  	USER_PLL_RSTN,
    output 	wire 	DDR3_PLL_RSTN,
	input 	wire  	user_pll_locked,
    //CSI Interface
    input	 wire		io_cam_sda_IN,
    output wire   io_cam_sda_OUT,
    output wire		io_cam_sda_OE,

    output io_cam_scl_OUT,
    input io_cam_scl_IN,
    output io_cam_scl_OE,

    output	wire		o_cam_rst_p,
    
    input	wire		i_cam_ck_LP_P_IN,
    input	wire		i_cam_ck_LP_N_IN,
    output	wire		o_cam_ck_HS_TERM,
    output	wire		o_cam_ck_HS_ENA,
    input	wire		i_cam_ck_CLKOUT,
    
    input	wire	[7:0]			cam_d0_HS_IN,
    input	  wire		      cam_d0_LP_P_IN,
    input	  wire		      cam_d0_LP_N_IN,
    output	wire		      cam_d0_HS_TERM,
    output	wire		      cam_d0_HS_ENA,
    output	wire		      cam_d0_RST,
    output	wire		      cam_d0_FIFO_RD,
    input	  wire		      cam_d0_FIFO_EMPTY,
    
    input 	wire	[7:0]			cam_d1_HS_IN,
    input	  wire		        cam_d1_LP_P_IN,
    input	  wire		        cam_d1_LP_N_IN,
    output	wire		        cam_d1_HS_TERM,
    output	wire		        cam_d1_HS_ENA,
    output	wire		        cam_d1_RST,
    output	wire		        cam_d1_FIFO_RD,
    input	  wire		        cam_d1_FIFO_EMPTY,
    
    input 	wire	[7:0]			cam_d2_HS_IN,
    input	  wire		        cam_d2_LP_P_IN,
    input	  wire		        cam_d2_LP_N_IN,
    output	wire		        cam_d2_HS_TERM,
    output	wire		        cam_d2_HS_ENA,
    output	wire		        cam_d2_RST,
    output	wire		        cam_d2_FIFO_RD,
    input	  wire		        cam_d2_FIFO_EMPTY,  
   
    input 	wire	[7:0]			cam_d3_HS_IN,
    input	  wire		        cam_d3_LP_P_IN,
    input	  wire		        cam_d3_LP_N_IN,
    output	wire		        cam_d3_HS_TERM,
    output	wire		        cam_d3_HS_ENA,
    output	wire		        cam_d3_RST,
    output	wire		        cam_d3_FIFO_RD,
    input	  wire		        cam_d3_FIFO_EMPTY,

	
	input 					tx_cal_clk_90edge,
  	input 					rx_cal_clk,
  	input 					tx_cal_clk,
	input                              core_clk,     // CORE CLK @ 100MHz
	input                              sdram_clk,    // SDRAM CK @ 400MHz
	// PLL status flags  
	output [2:0]                       pll_shift,  
	output [4:0]                       pll_shift_sel,
	output                             pll_shift_ena,  
	// memory interface ports
	output                             ddr_ck_hi,
	output                             ddr_ck_lo,
	output                             ddr_reset_n,
	output [CKE_WIDTH-1:0]             ddr_cke,     
	output [ROW_WIDTH-1:0]             ddr_addr,
	output [BANK_WIDTH-1:0]            ddr_ba,
	output                             ddr_cas_n,
 
	output [CS_WIDTH*RANK_RATIO-1:0]   ddr_cs_n,
	output                             ddr_ras_n,
	output                             ddr_we_n,
	
	input  [DQS_WIDTH-1:0]             ddr_dqs_in_hi,
	input  [DQS_WIDTH-1:0]             ddr_dqs_in_lo,
	input  [DQ_WIDTH-1:0]              ddr_dq_in_hi,
	input  [DQ_WIDTH-1:0]              ddr_dq_in_lo,
	
	output [DQS_WIDTH-1:0]             ddr_dqs_oe,
	output [DQS_WIDTH-1:0]             ddr_dqs_oe_n,
	output [DQ_WIDTH-1:0]              ddr_dq_oe,  
	output [DQS_WIDTH-1:0]             ddr_dqs_out_hi,
	output [DQS_WIDTH-1:0]             ddr_dqs_out_lo,
	output [DQ_WIDTH-1:0]              ddr_dq_out_hi,
	output [DQ_WIDTH-1:0]              ddr_dq_out_lo,
	output [DM_WIDTH-1:0]              ddr_dm_hi,
	output [DM_WIDTH-1:0]              ddr_dm_lo,
	output [ODT_WIDTH-1:0]             ddr_odt,

       //LED
       output [7:0] led,

       // 用户按键(低有效, 按下 = 0)
       input  wire [2:0] key_i,

       // MIPI DSI
       input	wire	                     i_mipi_tx_pclk		,
       output	wire	                     mipi_dp_clk_LP_P_OUT		,
       output	wire	                     mipi_dp_clk_LP_N_OUT		,
       output	wire	[7:0] 	              mipi_dp_clk_HS_OUT		,
       output	wire	                     mipi_dp_clk_HS_OE		,
       output	wire	                     mipi_dp_data3_LP_P_OUT	,
       output	wire	                     mipi_dp_data2_LP_P_OUT	,
       output	wire	                     mipi_dp_data1_LP_P_OUT	,
       output	wire	                     mipi_dp_data0_LP_P_OUT	,
       output	wire	                     mipi_dp_data3_LP_N_OUT	,
       output	wire	                     mipi_dp_data2_LP_N_OUT	,
       output	wire	                     mipi_dp_data1_LP_N_OUT	,
       output	wire	                     mipi_dp_data0_LP_N_OUT	,
       output	wire	[7:0] 	              mipi_dp_data0_HS_OUT	       ,
       output	wire	[7:0] 	              mipi_dp_data1_HS_OUT	       ,
       output	wire	[7:0] 	              mipi_dp_data2_HS_OUT	       ,
       output	wire	[7:0] 	              mipi_dp_data3_HS_OUT	       ,
       output	wire	                     mipi_dp_data3_HS_OE		,
       output	wire	                     mipi_dp_data2_HS_OE		,
       output	wire	                     mipi_dp_data1_HS_OE		,
       output	wire	                     mipi_dp_data0_HS_OE		,

       output	wire	                     mipi_dp_clk_RST		,
       output	wire	                     mipi_dp_data0_RST		,
       output	wire	                     mipi_dp_data1_RST		,
       output	wire	                     mipi_dp_data2_RST		,
       output	wire	                     mipi_dp_data3_RST		,
       output	wire	                     mipi_dp_clk_LP_P_OE		,
       output	wire	                     mipi_dp_clk_LP_N_OE		,
       output	wire	                     mipi_dp_data3_LP_P_OE	,
       output	wire	                     mipi_dp_data3_LP_N_OE	,
       output	wire	                     mipi_dp_data2_LP_P_OE	,
       output	wire	                     mipi_dp_data2_LP_N_OE	,
       output	wire	                     mipi_dp_data1_LP_P_OE	,
       output	wire	                     mipi_dp_data1_LP_N_OE	,
       output	wire	                     mipi_dp_data0_LP_P_OE	,
       output	wire	                     mipi_dp_data0_LP_N_OE	,

       input  wire	                     mipi_dp_data0_LP_P_IN	,
       input  wire	                     mipi_dp_data0_LP_N_IN	,
       output	wire	                     LCD_RST_P			,
       output wire                        LCD_POWER			,

       // hdmi interface

    output tmds_tx_clk_TX_OE,
    output [9:0] tmds_tx_clk_TX_DATA,
    output tmds_tx_clk_TX_RST,
    output tmds_tx_data0_TX_OE,
    output [9:0] tmds_tx_data0_TX_DATA,
    output tmds_tx_data0_TX_RST,
    output tmds_tx_data1_TX_OE,
    output [9:0] tmds_tx_data1_TX_DATA,
    output tmds_tx_data1_TX_RST,
    output tmds_tx_data2_TX_OE,
    output [9:0] tmds_tx_data2_TX_DATA,
    output tmds_tx_data2_TX_RST


);

function integer log2;
	input	integer	val;
	integer	i;
	begin
		log2 = 0;
		for (i=0; 2**i<val; i=i+1)
			log2 = i+1;
	end
endfunction


//===============================================================================
//localparam
//===============================================================================
localparam WR_FIFO_DEPTH  = 1024;
localparam RD_FIFO_DEPTH  = 1024;

// localparam	MAX_HRES		= 12'd1920;
// localparam	MAX_VRES		= 12'd1080;
// localparam	HSP			= 8'd2;
// localparam	HBP			= 8'd88;
// localparam	HFP			= 8'd120;
// localparam	VSP			= 8'd2;
// localparam	VBP			= 8'd20;
// localparam	VFP			= 8'd20;

localparam HACT     = 1920;
localparam HFP      = 88;
localparam HSP      = 44;
localparam HBP      = 148;

localparam VACT     = 1080;
localparam VFP      = 4;
localparam VSP      = 5;
localparam VBP      = 36;
//------------------------------------------------------------------------------
// 颜色相关默认值
//   DEF_*_GAIN: 白平衡默认增益(Q8.8, 256=1.00x). 与调试 VIO 的 r/g/b_gain 并存:
//               VIO 给了非 0 值就用 VIO, 否则用这里的默认值
//               整幅偏紫(洋红) = 红/蓝相对绿偏高 -> 增大 G 或减小 R/B; 偏绿则相反
//               SC431HAI2 的 RAW 绿色响应明显高于红/蓝, 中性 256/256/256
//               在实板上呈一层绿色滤镜; 默认用 1.5625x/1.00x/1.8125x 做固定白平衡.
//   RB_SWAP_DEF: 上电默认是否交换红蓝通道. 若画面颜色明显反了(脸发青绿/绿叶发紫),
//               说明 Bayer 相位相反, 把这里改成 1'b1 即可 (也可按键长按在线切换)
//------------------------------------------------------------------------------
localparam [15:0] DEF_R_GAIN  = 16'd400;   // 1.5625x, trimmed against mild red cast
localparam [15:0] DEF_G_GAIN  = 16'd256;
localparam [15:0] DEF_B_GAIN  = 16'd464;   // 1.8125x, trimmed with red channel
localparam        RB_SWAP_DEF = 1'b0;
localparam        USE_VIO_WB  = 1'b0;   // 0=白平衡只用 DEF_*_GAIN(推荐, 不受调试VIO上电初值影响)
//                                           1=用调试VIO的 r/g/b_gain(可在 Efinity Debugger 里在线调)
//===============================================================================
//signal
//===============================================================================
   
wire                              app_sr_active;
wire                              app_ref_ack;
wire                              app_zq_ack;
wire                              cal_done;

// Slave Interface Write Address Ports
wire [AXI_ID_WIDTH-1:0]           s_axi_awid;
wire [AXI_ADDR_WIDTH-1:0]         s_axi_awaddr;
wire [7:0]                        s_axi_awlen;
wire [2:0]                        s_axi_awsize;
wire [1:0]                        s_axi_awburst;
wire [0:0]                        s_axi_awlock;
wire [3:0]                        s_axi_awcache;
wire [2:0]                        s_axi_awprot;
wire                              s_axi_awvalid;
wire                              s_axi_awready;
// Slave Interface Write Data Ports
wire [AXI_DATA_WIDTH-1:0]         s_axi_wdata;
wire [(AXI_DATA_WIDTH/8)-1:0]     s_axi_wstrb;
wire                              s_axi_wlast;
wire                              s_axi_wvalid;
wire                              s_axi_wready;
// Slave Interface Write Response Ports
wire                              s_axi_bready;
wire [AXI_ID_WIDTH-1:0]           s_axi_bid;
wire [1:0]                        s_axi_bresp;
wire                              s_axi_bvalid;
// Slave Interface Read Address Ports
wire [AXI_ID_WIDTH-1:0]           s_axi_arid;
wire [AXI_ADDR_WIDTH-1:0]         s_axi_araddr;
wire [7:0]                        s_axi_arlen;
wire [2:0]                        s_axi_arsize;
wire [1:0]                        s_axi_arburst;
wire [0:0]                        s_axi_arlock;
wire [3:0]                        s_axi_arcache;
wire [2:0]                        s_axi_arprot;
wire                              s_axi_arvalid;
wire                              s_axi_arready;
// Slave Interface Read Data Ports
wire                              s_axi_rready;
wire [AXI_ID_WIDTH-1:0]           s_axi_rid;
wire [AXI_DATA_WIDTH-1:0]         s_axi_rdata;
wire [1:0]                        s_axi_rresp;
wire                              s_axi_rlast;
wire                              s_axi_rvalid;




////////////////////////////////////////////////////////////////
// System & Debugger
wire    w_arstn;
// wire	w_sysclk_mcu_arstn;
// wire	w_sysclk_mcu_arst;
wire	w_mipi_rx_pclk_arstn;
wire	w_mipi_rx_pclk_arst;

wire		       w_mipi_rx_vs;
wire		       w_mipi_rx_hs;
wire	       w_mipi_rx_de;
wire	[63:0]			w_mipi_rx_data	;

wire					io_systemReset;



//===================================================================================================
//reset module
//===================================================================================================

assign w_arstn  		       = i_pll_locked & pll_locked & user_pll_locked;
assign pll_inst1_RSTN  		= i_arstn;
assign USER_PLL_RSTN      		= i_arstn;
assign mipi_dsi_tx_pll_RSTN 	= i_arstn;
assign DDR3_PLL_RSTN 		= i_arstn;
reset_ctrl
#(
	.NUM_RST		(2),
	.CYCLE			(1),
	.IN_RST_ACTIVE	(4'b00),
	.OUT_RST_ACTIVE	(4'b10)
)
inst_reset_ctrl
(
	.i_arst		({2{w_arstn}}),//({{4{i_pll_locked}},                  {2{i_arstn}}}),
	.i_clk			({2{i_mipi_rx_pclk}}),
	.o_srst		({w_mipi_rx_pclk_arst, w_mipi_rx_pclk_arstn})
);



reg [25:0] cnt = 'd0;
always @( posedge core_clk)
begin 
       cnt <= cnt + 1'b1;
end
// D0/D1 的最终驱动统一放在按键模式声明之后，便于调节模式覆盖显示。

//---------------------------------------------------------------
// 新增诊断 LED
//   D1 = 视频活动指示（闪烁 = HDMI 正在输出帧）
//   D2 = 边缘检测活动（亮/闪 = 右半屏检测到边缘）
//---------------------------------------------------------------
reg [23:0] edge_beat = 24'd0;
always @(posedge hdmi_tx_slow_clk or negedge vid_rst_n) begin
    if (!vid_rst_n)
        edge_beat <= 24'd0;
    else
        edge_beat <= edge_beat + 1'b1;
end
// D1/D2 的最终驱动见下面按键模式声明之后。


//========================================================================================================
//MIPI RX
//========================================================================================================
wire i_mipi_clk ;
assign i_mipi_clk = core_clk;

		assign	cam_d0_RST		= 1'b0;
		assign	cam_d1_RST		= 1'b0;
		assign  cam_d2_RST = 1'b0;
    assign  cam_d3_RST = 1'b0;
	
wire [15:0] mipi_debug_out;
		

		csi_rx_controller inst_efx_csi2_rx
		(
              .reset_n			(i_arstn),
              .clk				(i_mipi_clk),
              .reset_byte_HS_n	(i_arstn),
              .clk_byte_HS		  (i_cam_ck_CLKOUT),
              .reset_pixel_n		(w_mipi_rx_pclk_arstn),
              .clk_pixel			  (i_mipi_rx_pclk),
              
              .Rx_LP_CLK_P		  (i_cam_ck_LP_P_IN),
              .Rx_LP_CLK_N		  (i_cam_ck_LP_N_IN),
              .Rx_HS_enable_C		(o_cam_ck_HS_ENA),
              .LVDS_termen_C		(o_cam_ck_HS_TERM),
              
              .Rx_LP_D_P			({cam_d3_LP_P_IN, cam_d2_LP_P_IN, cam_d1_LP_P_IN, cam_d0_LP_P_IN}),//(r_mipi_rx_data_LP_P_IN_2P),
              .Rx_LP_D_N			({cam_d3_LP_N_IN, cam_d2_LP_N_IN, cam_d1_LP_N_IN, cam_d0_LP_N_IN}),//(r_mipi_rx_data_LP_N_IN_2P),
              .Rx_HS_D_0			(cam_d0_HS_IN),//(r_mipi_rx_data_HS_IN_2P[0*8+:8]),
              .Rx_HS_D_1			(cam_d1_HS_IN),//(r_mipi_rx_data_HS_IN_2P[1*8+:8]),
              .Rx_HS_D_2			(cam_d2_HS_IN),
              .Rx_HS_D_3			(cam_d3_HS_IN),
              .Rx_HS_D_4			(),
              .Rx_HS_D_5			(),
              .Rx_HS_D_6			(),
              .Rx_HS_D_7			(),
              .Rx_HS_enable_D		({cam_d3_HS_ENA    ,cam_d2_HS_ENA    ,cam_d1_HS_ENA    ,cam_d0_HS_ENA    }),//( w_cam_d_HS_ENA),
              .LVDS_termen_D		({cam_d3_HS_TERM   ,cam_d2_HS_TERM   ,cam_d1_HS_TERM   ,cam_d0_HS_TERM   }),
              .fifo_rd_enable		({cam_d3_FIFO_RD   ,cam_d2_FIFO_RD   ,cam_d1_FIFO_RD   ,cam_d0_FIFO_RD   }),
              .fifo_rd_empty		({cam_d3_FIFO_EMPTY,cam_d2_FIFO_EMPTY,cam_d1_FIFO_EMPTY,cam_d0_FIFO_EMPTY}),
              .DLY_enable_D		       (),
              .DLY_inc_D			(),
              .u_dly_enable_D		(),
              .u_dly_inc_D		       (),
              
              .axi_clk			(1'b0),
              .axi_reset_n		       (1'b0),
              .axi_awaddr			(6'b0),
              .axi_awvalid		       (1'b0),
              .axi_awready		       (),
              .axi_wdata			(32'b0),
              .axi_wvalid			(1'b0),
              .axi_wready			(),
              
              .axi_bvalid			(),
              .axi_bready			(1'b0),
              .axi_araddr			(6'b0),
              .axi_arvalid		       (1'b0),
              .axi_arready		       (),
              .axi_rdata			(),
              .axi_rvalid			(),
              .axi_rready			(1'b1),
              
              .hsync_vc0			(w_mipi_rx_hs),
              .hsync_vc1			(),
              .hsync_vc2			(),
              .hsync_vc3			(),
              .vsync_vc0			(w_mipi_rx_vs),
              .vsync_vc1			(),
              .vsync_vc2			(),
              .vsync_vc3			(),
              .vc				(),
              .word_count			(),
              .shortpkt_data_field        (),
              .datatype			(),
              .pixel_per_clk		(),
              .pixel_data			(w_mipi_rx_data),
              .pixel_data_valid	       (w_mipi_rx_de),
              .irq				(),
              .mipi_debug_out(mipi_debug_out)
		);

// Vendor v9 crops the 2560x1440-class sensor stream to 1920x1080 here:
// 1081 MIPI lines x 640 beats arrive, but the frame buffer/display path
// expects 1080 lines x 480 beats.  Without this stage the extra 160 beats
// per line and one extra line change the frame length and Bayer phase,
// producing magenta/purple surfaces, colour noise and frame flicker.
wire		       w1_mipi_rx_vs;
wire		       w1_mipi_rx_hs;
wire	       w1_mipi_rx_de;
wire	[39:0]			w1_mipi_rx_data	;
sensor_clipper  sensor_clipper_inst (
    .clk(i_mipi_rx_pclk),
    .i_vs(w_mipi_rx_vs),
    .i_hs(w_mipi_rx_hs),
    .i_de(w_mipi_rx_de),
    .i_dat({w_mipi_rx_data[9:2],w_mipi_rx_data[19:12],
            w_mipi_rx_data[29:22],w_mipi_rx_data[39:32]}),

    .o_hs(w1_mipi_rx_hs),
    .o_vs(w1_mipi_rx_vs),
    .o_de(w1_mipi_rx_de),
    .o_dat(w1_mipi_rx_data)
   );


wire mdebug_pixel_fifo_full     = mipi_debug_out[0] ;
wire mdebug_pixel_fifo_empty    = mipi_debug_out[1] ;
wire mdebug_crc_error           = mipi_debug_out[2] ;
wire mdebug_ecc_1bit_error      = mipi_debug_out[3] ;
wire mdebug_ecc_2bit_error      = mipi_debug_out[4] ;
wire mdebug_undersize_pkt_error = mipi_debug_out[5] ;
wire mdebug_line_vc0_error      = mipi_debug_out[6] ;
wire mdebug_frame_vc0_error    = mipi_debug_out[10];   
wire mdebug_receive_error      = mipi_debug_out[14];


//========================================================================================================
//
//========================================================================================================
		

//*************** imx219 config********************/
// mipi_config_imx219_top u_config_imx219(
// 										.i_sys_clk(CLK_5M)    	,    /////48M 
// 										.sys_rstn(vid_rst_n )    ,
// 										.o_sen_rst_n(o_cam_rstn)  	,
										
										// .o_iic_sda_OE(io_cam_sda_OE) ,
										// .i_iic_sda(io_cam_sda_IN)   	,
										// .o_iic_sda(io_cam_sda_OUT)   	,
										// .o_iic_scl(io_cam_scl)    	
										// ); 	

wire scl_padoen_o;
wire sda_padoen_o;

//==============================================================================
// 摄像头曝光/增益运行时调节
//   cfg_req_tgl -> CLK_5M 域的 cam_cfg_sched -> I2C Group Hold 写寄存器
//   默认值保持当前已调好的启动基线: EXP=0x0BAD, GAIN=0x81/0x32
//==============================================================================
reg         cfg_req_tgl;
reg  [1:0]  cfg_sel_job;
reg  [15:0] cfg_exp_job;
reg  [7:0]  cfg_gain_c_job;
reg  [7:0]  cfg_gain_f_job;
wire        cfg_ack_tgl;
reg         cfg_ack_meta;
reg         cfg_ack_sync;
wire        cfg_busy = (cfg_req_tgl != cfg_ack_sync);

// 按键通道的请求(可能与 AE 通道共用同一组 cfg_* 寄存器)
reg         key_req_tgl;
reg  [1:0]  key_sel;
reg  [15:0] key_exp;
reg  [7:0]  key_gain_c;
reg  [7:0]  key_gain_f;

// 调节模式与档位(在 HDMI 时钟域维护, 同时用于 LED 反馈)
reg         cam_adj_mode;
reg  [3:0]  exp_level;
reg  [3:0]  gain_level;

always @(posedge hdmi_tx_slow_clk or negedge vid_rst_n) begin
    if (!vid_rst_n) begin
        cfg_ack_meta <= 1'b0;
        cfg_ack_sync <= 1'b0;
    end else begin
        cfg_ack_meta <= cfg_ack_tgl;
        cfg_ack_sync <= cfg_ack_meta;
    end
end


reg [12:0] i2c_rst_cnt = '0;

always @( posedge CLK_5M or negedge w_arstn )
begin
    if( !w_arstn )
       i2c_rst_cnt <= 'd0;
    else 
       i2c_rst_cnt <= i2c_rst_cnt[12] ? i2c_rst_cnt : i2c_rst_cnt + 1'b1;
end
wire i2c_rst_n = i2c_rst_cnt[12];

i2c_master_ctrl_top #(
  .DATA_LENGTH (148)
) u2_i2c_master_ctrl_top(
  /*i*/.clk			(CLK_5M		),
  /*i*/.rst_n			(i2c_rst_n		),
  /*i*/.cfg_req_tgl		(cfg_req_tgl	),
  /*i*/.cfg_sel			(cfg_sel_job	),
  /*i*/.cfg_exp			(cfg_exp_job	),
  /*i*/.cfg_gain_c		(cfg_gain_c_job	),
  /*i*/.cfg_gain_f		(cfg_gain_f_job	),
  /*o*/.cfg_ack_tgl		(cfg_ack_tgl	),
  /*i*/.scl_pad_i     (io_cam_scl_IN),//(1'b1),
  /*o*/.scl_pad_o     (io_cam_scl_OUT),
  /*o*/.scl_padoen_o  (scl_padoen_o),
  /*i*/.sda_pad_i     (io_cam_sda_IN),
  /*o*/.sda_pad_o     (io_cam_sda_OUT),
  /*o*/.sda_padoen_o  (sda_padoen_o)
  
  );
assign o_cam_rst_p = ~w_arstn;
assign io_cam_scl_OE = ~scl_padoen_o;
assign io_cam_sda_OE = ~sda_padoen_o;

//=====================================================================================
// 高阶挑战: 帧缓存(DDR3) + 冻结/回放 (采集/存储/读取/处理 分离)
//   free_on=1 (KEY2 长按) 时, 在摄像头场同步边沿切断写侧(i_vs/i_de): DDR3 不再写入新帧;
//   写侧最后一帧排空后, ddr_buffer 再在显示帧边界锁住读侧 bank, 让 cvo_axi
//   按显示帧节拍反复读取同一个完整 bank (避免长按瞬间锁到未写完的 bank).
//   注意: 不能只切断写侧而不冻结读 bank, 否则三 bank 握手会在读侧等不到写侧切换,
//         读地址会落到正在清空/未写完的 bank, 表现就是冻结时整屏闪.
//   => 画面静止 = 冻结/回放. 冻结期间改阈值/滤波/显示模式, 同一帧会被重新处理,
//      可以直接对比同一帧在不同算法参数下的效果
//   在 vsync 边沿切换是为了保证写侧永远停在完整帧边界(不会写坏 bank)
//=====================================================================================
reg         free_on    = 1'b0;   // hdmi_tx_slow_clk 域: 冻结状态(电平, 0=正常采集)
reg         free_on_p0 = 1'b0;   // 同步到 i_mipi_rx_pclk 域
reg         free_on_p1 = 1'b0;
reg         rx_vs_d    = 1'b0;
reg         fr_gate    = 1'b0;   // i_mipi_rx_pclk 域: 写侧门控, 只在场同步边沿更新
always @(posedge i_mipi_rx_pclk or negedge w_mipi_rx_pclk_arstn) begin
    if (!w_mipi_rx_pclk_arstn) begin
        free_on_p0 <= 1'b0;
        free_on_p1 <= 1'b0;
        rx_vs_d    <= 1'b0;
        fr_gate    <= 1'b0;
    end else begin
        free_on_p0 <= free_on;
        free_on_p1 <= free_on_p0;
        rx_vs_d    <= w1_mipi_rx_vs;
        if (w1_mipi_rx_vs != rx_vs_d)   // 场同步边沿 = 帧边界
            fr_gate <= free_on_p1;
    end
end

//=====================================================================================
//frame buffer
//=====================================================================================
wire [15:0]   ch0_raw;
wire [7:0] 	ch0_r;
wire [7:0]    ch0_g = ch0_raw[7:0];
wire [7:0]    ch0_b = ch0_raw[15:8];
wire ch0_vs;
wire ch0_hs;
wire ch0_de;

wire  [16-1:0] m_axis_tdata;
wire           m_axis_tvalid;
wire           m_axis_tready;
wire           m_axis_tlast;
wire           m_axis_tuser;
wire  fifo_rd_period;
// IMX219
  frame_buffer #(
.I_VID_WIDTH         (32),
.O_VID_WIDTH         (16),
.START_ADDR          (32'h00000        ),
.AXI_DATA_WIDTH      ( AXI_DATA_WIDTH	),
.AXI_ADDR_WIDTH      ( AXI_ADDR_WIDTH	),
.WR_FIFO_DEPTH	      ( 1024		),    
.RD_FIFO_DEPTH 	      ( 1024 	),
.BURST_LEN  	        (127),
.FB_NUM	              (3),
.MAX_VID_WIDTH	      (960) ,
.MAX_VID_HIGHT	      (1080),
.O_FRAME_WIDTH      (960),
.O_FRAME_HEIGHT     (1080)

)checker0(
       .axi_clk		(core_clk 	        ),
       .rst_n			(vid_rst_n   ),

/*i*/.i_clk			(i_mipi_rx_pclk      ),
/*i*/.i_vs			(w1_mipi_rx_vs & ~fr_gate			),//冻结时切断写侧
/*i*/.i_de			(w1_mipi_rx_de & w1_mipi_rx_hs & ~fr_gate	),
/*i*/.freeze			(free_on                          ),//冻结请求; ddr_buffer 写侧排空后锁 bank
/*i*/.vin 			(w1_mipi_rx_data[31:0]	),
                     
/*i*/.o_clk			(vid_clk_dvi2),//(i_mipi_rx_pclk	),                  
/*i*/.fifo_rd_period(fifo_rd_period),//(fifo_rd_period  ),

    .m_axis_tdata  (m_axis_tdata),
    .m_axis_tvalid (m_axis_tvalid),
    .m_axis_tready (m_axis_tready),
    .m_axis_tuser  (m_axis_tuser),
    .m_axis_tlast  (m_axis_tlast),
	

       .awid			(s_axi_awid	),      
       .awaddr		(s_axi_awaddr	),
       .awlen			(s_axi_awlen	),
       .awsize		(s_axi_awsize	),
       .awburst		(s_axi_awburst),
       .awprot    (s_axi_awprot ),
       .awcache   (s_axi_awcache),
       .awlock		(s_axi_awlock	),
       .awvalid		(s_axi_awvalid),
       .awready		(s_axi_awready),

       .arid			(s_axi_arid	  ),
       .araddr		(s_axi_araddr	),
       .arlen			(s_axi_arlen	),
       .arsize		(s_axi_arsize	),
       .arburst 	(s_axi_arburst),

       .arprot    (s_axi_arprot ),
       .arcache   (s_axi_arcache),
       .arlock 		(s_axi_arlock	),
       .arvalid		(s_axi_arvalid),
       .arready		(s_axi_arready),

       .wdata			(s_axi_wdata	),
       .wstrb			(s_axi_wstrb	),
       .wlast			(s_axi_wlast	),
       .wvalid		(s_axi_wvalid	),
       .wready		(s_axi_wready	),


       .rid			  (s_axi_rid	),
       .rdata			(s_axi_rdata	),
       .rlast			(s_axi_rlast	),
       .rvalid 		(s_axi_rvalid	),
       .rready		(s_axi_rready	),
       .rresp			(s_axi_rresp	),

       .bid			  (s_axi_bid	),
       .bvalid		(s_axi_bvalid	),
       .bready	  (s_axi_bready	)
);

		    
		    
cvo_axi # (
    .SYMBOL_WIDTH(8),
    .SYMBOL_NUM(1),
    .PIXEL_NUM(2),
    .HSYNC_POL(1),
    .VSYNC_POL(1),
    .FIFO_DEPTH(1024),
    .FIFO_ALMOST_FULL(1000),
    .MAX_H_VALID(1920),
    .MAX_V_VALID(1080)
  )
  cvo_axi_inst (
    .o_clk          (vid_clk_dvi2   ),
    .rst_n          (vid_rst_n      ),
    .fifo_rd_period (fifo_rd_period ),
    .s_axi_clk      (vid_clk_dvi2   ),
    .s_axis_tdata   (m_axis_tdata   ),
    .s_axis_tvalid  (m_axis_tvalid  ),
    .s_axis_tready  (m_axis_tready  ),
    .s_axis_tlast   (m_axis_tlast   ),
    .s_axis_tuser   (m_axis_tuser   ),
    .H_FRONT_PORCH  (HFP            ),
    .H_SYNC         (HSP            ),
    .H_VALID        (HACT           ),
    .H_BACK_PORCH   (HBP            ),
    .V_FRONT_PORCH  (VFP            ),
    .V_SYNC         (VSP            ),
    .V_VALID        (VACT           ),
    .V_BACK_PORCH   (VBP            ),
     .vout          (ch0_raw        ),
    .o_hs           (ch0_hs         ),
    .o_vs           (ch0_vs         ),
    .o_de           (ch0_de         )
  );		       


ddr3_top                 u_ddr3_top
(

.axi_clk                (core_clk            ),
.core_clk               (core_clk            ),
.sdram_clk              (sdram_clk           ),  
.rx_cal_clk             (rx_cal_clk          ),
.tx_cal_clk             (tx_cal_clk          ),
.tx_cal_clk_90edge      (tx_cal_clk_90edge   ),
.rstn                   (w_arstn             ),      
.pll_shift              (pll_shift           ),
.pll_shift_sel          (pll_shift_sel       ),
.pll_shift_ena          (pll_shift_ena       ),       
///////////////DDR BUS
.ddr_ck_hi              (ddr_ck_hi           ),
.ddr_ck_lo              (ddr_ck_lo           ),
.ddr_cke                (ddr_cke             ),    
.ddr_reset_n            (ddr_reset_n         ),
.ddr_cs_n               (ddr_cs_n            ),
.ddr_ras_n              (ddr_ras_n           ),
.ddr_cas_n              (ddr_cas_n           ),
.ddr_we_n               (ddr_we_n            ),     
.ddr_addr               (ddr_addr            ),
.ddr_ba                 (ddr_ba              ),

.ddr_dqs_oe             (ddr_dqs_oe          ),
.ddr_dqs_oe_n           (ddr_dqs_oe_n        ),
.ddr_dq_oe              (ddr_dq_oe           ),
.ddr_dqs_in_hi          (ddr_dqs_in_hi       ),
.ddr_dqs_in_lo          (ddr_dqs_in_lo       ),
.ddr_dq_in_hi           (ddr_dq_in_hi        ),
.ddr_dq_in_lo           (ddr_dq_in_lo        ),

.ddr_dqs_out_hi         (ddr_dqs_out_hi      ),
.ddr_dqs_out_lo         (ddr_dqs_out_lo      ),
.ddr_dq_out_hi          (ddr_dq_out_hi       ),
.ddr_dq_out_lo          (ddr_dq_out_lo       ),

.ddr_dm_hi              (ddr_dm_hi           ),
.ddr_dm_lo              (ddr_dm_lo           ),
.ddr_odt                (ddr_odt             ),

// Application interface ports
.app_sr_req                     (1'b0),
.app_ref_req                    (1'b0),
.app_zq_req                     (1'b0),
.app_sr_active                  (app_sr_active),
.app_ref_ack                    (app_ref_ack),
.app_zq_ack                     (app_zq_ack),

// Slave Interface Write Address Ports
.s_axi_awid                     (s_axi_awid        ),
.s_axi_awaddr                   (s_axi_awaddr      ),
.s_axi_awlen                    (s_axi_awlen       ),
.s_axi_awsize                   (s_axi_awsize      ),
.s_axi_awburst                  (s_axi_awburst     ),
.s_axi_awlock                   (s_axi_awlock      ),
.s_axi_awcache                  (s_axi_awcache     ),
.s_axi_awprot                   (s_axi_awprot      ),
.s_axi_awqos                    (4'h0              ),
.s_axi_awvalid                  (s_axi_awvalid     ),
.s_axi_awready                  (s_axi_awready     ),
// Slave Interface Write Data Ports
.s_axi_wdata                    (s_axi_wdata       ),
.s_axi_wstrb                    (s_axi_wstrb       ),
.s_axi_wlast                    (s_axi_wlast       ),
.s_axi_wvalid                   (s_axi_wvalid      ),
.s_axi_wready                   (s_axi_wready      ),
// Slave Interface Write Response Ports
.s_axi_bid                      (s_axi_bid         ),
.s_axi_bresp                    (s_axi_bresp       ),
.s_axi_bvalid                   (s_axi_bvalid      ),
.s_axi_bready                   (s_axi_bready      ),
// Slave Interface Read Address Ports
.s_axi_arid                     (s_axi_arid        ),
.s_axi_araddr                   (s_axi_araddr      ),
.s_axi_arlen                    (s_axi_arlen       ),
.s_axi_arsize                   (s_axi_arsize      ),
.s_axi_arburst                  (s_axi_arburst     ),
.s_axi_arlock                   (s_axi_arlock      ),
.s_axi_arcache                  (s_axi_arcache     ),
.s_axi_arprot                   (s_axi_arprot      ),
.s_axi_arqos                    (4'h0              ),
.s_axi_arvalid                  (s_axi_arvalid     ),
.s_axi_arready                  (s_axi_arready     ),
// Slave Interface Read Data Ports
.s_axi_rid                      (s_axi_rid         ),
.s_axi_rdata                    (s_axi_rdata       ),
.s_axi_rresp                    (s_axi_rresp       ),
.s_axi_rlast                    (s_axi_rlast       ),
.s_axi_rvalid                   (s_axi_rvalid      ),
.s_axi_rready                   (s_axi_rready      ),
//DEBUG       
.wrlvl_dq_check                 (wrlvl_dq_check    ) ,
.rd_level_dqs_check             (rd_level_dqs_check) ,
.init_cur_state                 (init_cur_state    ) ,
.idelay_ld                      (idelay_ld         ) ,
.mpr_rdlvl_dly                  (mpr_rdlvl_dly     ) ,
.cal_done                       (cal_done          ) 
);

//***************************************************************************
// debayer
//***************************************************************************


wire        rgb_vs;
wire        rgb_hs;
wire        rgb_de;
wire        rgb_valid;
wire [47:0] rgb_datax2;
wire [15:0] r_gain;
wire [15:0] g_gain;
wire [15:0] b_gain;
// Runtime white-balance trim (HDMI/key domain), centred on DEF_R/DEF_B.
// Level 8/16 is the compiled default; each step is 8/256 = 3.125%.
reg  [3:0] wb_r_level;
reg  [3:0] wb_b_level;
reg        wb_page;
wire [3:0] wb_r_level_gray = wb_r_level ^ (wb_r_level >> 1);
wire [3:0] wb_b_level_gray = wb_b_level ^ (wb_b_level >> 1);

// Gray-coded slow controls cross into the camera pixel clock safely.
reg  [3:0] wb_r_level_gm, wb_r_level_gs;
reg  [3:0] wb_b_level_gm, wb_b_level_gs;
always @(posedge vid_clk_dvi2 or negedge vid_rst_n) begin
    if (!vid_rst_n) begin
        wb_r_level_gm <= 4'd0;
        wb_r_level_gs <= 4'd0;
        wb_b_level_gm <= 4'd0;
        wb_b_level_gs <= 4'd0;
    end else begin
        wb_r_level_gm <= wb_r_level_gray;
        wb_r_level_gs <= wb_r_level_gm;
        wb_b_level_gm <= wb_b_level_gray;
        wb_b_level_gs <= wb_b_level_gm;
    end
end

function [3:0] gray4_to_bin;
    input [3:0] gray;
    begin
        gray4_to_bin[3] = gray[3];
        gray4_to_bin[2] = gray[3] ^ gray[2];
        gray4_to_bin[1] = gray[3] ^ gray[2] ^ gray[1];
        gray4_to_bin[0] = gray[3] ^ gray[2] ^ gray[1] ^ gray[0];
    end
endfunction

wire [3:0]  wb_r_level_px = gray4_to_bin(wb_r_level_gs);
wire [3:0]  wb_b_level_px = gray4_to_bin(wb_b_level_gs);

//-----------------------------------------------------------------------------
// 白平衡增益: 自动白平衡(cam_ae_awb) + 手动覆盖
//
// 旧实现是编译期常量 DEF_R/ DEF_B, 与光照无关: 夜间冷光下凑出来的 1.5625x/1.8125x
// 在白天阳光下会把本来就偏高的 R 再放大一次, 而且是 8bit 乘法后饱和到 255,
// 红通道直接削顶 -> 画面偏红且丢层次. 手动窗口只有 +-4 档, 下限仍是 1.3125x,
// 压不回 1.0, 所以"手动白平衡无效".
//
// 现在由 cam_ae_awb 逐帧估计增益; 进入调节模式的"白平衡页"后 i_man_en=1,
// 环路冻结, 手动档位直接给出 96..576 (0.375x..2.25x, 每档 0.125x) 的增益,
// 范围和方向都覆盖实际需要, 退出该页即恢复自动.
//-----------------------------------------------------------------------------
wire [15:0] awb_r_gain, awb_g_gain, awb_b_gain;
wire        awb_locked;
wire [3:0]  awb_dbg;
wire        ae_req_vid, ae_ack_vid;
wire [15:0] ae_exp_vid;
wire [7:0]  ae_gc_vid, ae_gf_vid;

// 慢速控制位跨到像素时钟域 (2FF 同步)
reg cam_adj_v1, cam_adj_v2, wb_page_v1, wb_page_v2;
always @(posedge vid_clk_dvi2 or negedge vid_rst_n) begin
    if (!vid_rst_n) begin
        cam_adj_v1 <= 1'b0; cam_adj_v2 <= 1'b0;
        wb_page_v1 <= 1'b0; wb_page_v2 <= 1'b0;
    end else begin
        cam_adj_v1 <= cam_adj_mode; cam_adj_v2 <= cam_adj_v1;
        wb_page_v1 <= wb_page;      wb_page_v2 <= wb_page_v1;
    end
end
wire        wb_man_en = cam_adj_v2 & wb_page_v2;
wire [15:0] wb_r_man  = 16'd96 + {9'd0, wb_r_level_px, 5'b0};
wire [15:0] wb_b_man  = 16'd96 + {9'd0, wb_b_level_px, 5'b0};

wire [15:0] wb_r = wb_man_en ? wb_r_man : awb_r_gain;
wire [15:0] wb_g = awb_g_gain;
wire [15:0] wb_b = wb_man_en ? wb_b_man : awb_b_gain;

debayer_top_2to1 debayer_top
(
	.in_pclk		  (vid_clk_dvi2),//(i_mipi_rx_pclk ),
	.in_rstn		  (vid_rst_n	),
	
	.raw_vs_i		  (ch0_vs		      ),
	.raw_hs_i		  (ch0_hs		      ),
	.raw_de_i		  (ch0_de		      ),
	.raw_valid_i	(ch0_de	        ),
	.raw_datax4_i	({ch0_raw[7:0],ch0_raw[15:8]}),
	.r_gain       ( wb_r ),
  .g_gain       ( wb_g ),
  .b_gain       ( wb_b ),
	.rgb_vs_o		  (rgb_vs         ),
	.rgb_hs_o		  (rgb_hs         ),
	.rgb_de_o		  (rgb_de         ),
	.rgb_valid_o	(rgb_valid      ),
	.rgb_datax2_o (rgb_datax2     )//b,g,r,b,g,r
);

//-----------------------------------------------------------------------------
// 自动白平衡 / 自动曝光
//   统计点在去马赛克之后 (即用户实际看到的颜色), 增益回送到 debayer 的
//   rgb_gain_v1, 构成一个逐帧闭环. 全在 vid_clk_dvi2 域内, 不跨时钟.
//-----------------------------------------------------------------------------
cam_ae_awb #(
    .SEED_R_GAIN (DEF_R_GAIN),
    .SEED_G_GAIN (DEF_G_GAIN),
    .SEED_B_GAIN (DEF_B_GAIN)
) u_cam_ae_awb (
    .clk          (vid_clk_dvi2),
    .rst_n        (vid_rst_n),
    .i_de         (rgb_de),
    .i_vs         (rgb_vs),
    .i_rgb2       (rgb_datax2),
    .i_man_en     (wb_man_en),
    .i_man_r      (wb_r_man),
    .i_man_b      (wb_b_man),
    .o_r_gain     (awb_r_gain),
    .o_g_gain     (awb_g_gain),
    .o_b_gain     (awb_b_gain),
    .o_awb_locked (awb_locked),
    .o_awb_dbg    (awb_dbg),
    .o_ae_req_tgl (ae_req_vid),
    .o_ae_exp     (ae_exp_vid),
    .o_ae_gain_c  (ae_gc_vid),
    .o_ae_gain_f  (ae_gf_vid),
    .i_ae_ack_tgl (ae_ack_vid),
    .i_ae_en      (~cam_adj_v2)
);

//-----------------------------------------------------------------------------
// cfg_* 总线仲裁 (hdmi_tx_slow_clk 域)
//   按键通道只在"摄像头调节模式"内产生请求, AE 只在非调节模式下发请求,
//   两者互斥, 共用一套四相握手, 不会互相踩掉.
//-----------------------------------------------------------------------------
reg         ae_req_m, ae_req_s, ae_req_seen;
reg         ae_pending;
reg  [15:0] ae_exp_lat;
reg  [7:0]  ae_gc_lat, ae_gf_lat;

wire ae_fire = ae_pending && !cfg_busy && !cam_adj_mode &&
               (key_req_tgl == cfg_req_tgl);

always @(posedge hdmi_tx_slow_clk or negedge vid_rst_n) begin
    if (!vid_rst_n) begin
        ae_req_m   <= 1'b0;
        ae_req_s   <= 1'b0;
        ae_req_seen<= 1'b0;
        ae_pending <= 1'b0;
        ae_exp_lat <= 16'h0BAD;
        ae_gc_lat  <= 8'h81;
        ae_gf_lat  <= 8'h32;
    end else begin
        ae_req_m <= ae_req_vid;
        ae_req_s <= ae_req_m;
        if (ae_req_s != ae_req_seen) begin
            ae_req_seen <= ae_req_s;
            ae_exp_lat  <= ae_exp_vid;
            ae_gc_lat   <= ae_gc_vid;
            ae_gf_lat   <= ae_gf_vid;
            ae_pending  <= 1'b1;
        end else if (ae_fire) begin
            ae_pending  <= 1'b0;
        end
    end
end

always @(posedge hdmi_tx_slow_clk or negedge vid_rst_n) begin
    if (!vid_rst_n) begin
        cfg_req_tgl    <= 1'b0;
        cfg_sel_job    <= 2'd0;
        cfg_exp_job    <= 16'h0BAD;
        cfg_gain_c_job <= 8'h81;
        cfg_gain_f_job <= 8'h32;
    end else if ((key_req_tgl != cfg_req_tgl) && !cfg_busy) begin
        cfg_sel_job    <= key_sel;
        cfg_exp_job    <= key_exp;
        cfg_gain_c_job <= key_gain_c;
        cfg_gain_f_job <= key_gain_f;
        cfg_req_tgl    <= key_req_tgl;
    end else if (ae_fire) begin
        cfg_sel_job    <= 2'd2;
        cfg_exp_job    <= ae_exp_lat;
        cfg_gain_c_job <= ae_gc_lat;
        cfg_gain_f_job <= ae_gf_lat;
        cfg_req_tgl    <= ~cfg_req_tgl;
    end
end

// I2C 完成信号回到像素时钟域, 给 AE 做握手确认
reg ae_ack_1, ae_ack_2;
always @(posedge vid_clk_dvi2 or negedge vid_rst_n) begin
    if (!vid_rst_n) begin
        ae_ack_1 <= 1'b0;
        ae_ack_2 <= 1'b0;
    end else begin
        ae_ack_1 <= cfg_ack_sync;
        ae_ack_2 <= ae_ack_1;
    end
end
assign ae_ack_vid = ae_ack_2;
reg rgb_vs_r = 'd0;
always @( posedge hdmi_tx_slow_clk )
begin
  rgb_vs_r <= rgb_vs;
end
assign pos_vs = {rgb_vs_r,rgb_vs} == 2'b01;
reg vid_cnt = 'd0;
reg [7:0] hdmi_tx_rdata ;
reg [7:0] hdmi_tx_gdata ;
reg [7:0] hdmi_tx_bdata ;
reg hdmi_tx_vs;
reg hdmi_tx_hs;
reg hdmi_tx_de;
always @( posedge hdmi_tx_slow_clk or negedge vid_rst_n )
begin
    if( !vid_rst_n ) begin
        vid_cnt       <= 1'b0;
        hdmi_tx_rdata <= 8'd0;
        hdmi_tx_gdata <= 8'd0;
        hdmi_tx_bdata <= 8'd0;
        hdmi_tx_vs    <= 1'b0;
        hdmi_tx_hs    <= 1'b0;
        hdmi_tx_de    <= 1'b0;
    end else begin
        hdmi_tx_vs <= rgb_vs;
        hdmi_tx_hs <= rgb_hs;
        hdmi_tx_de <= rgb_de;

        // Re-align the 2-to-1 pixel serializer at every line start.
        // The first valid beat must always be rgb_datax2[47:24].
        if( !rgb_de ) begin
            vid_cnt       <= 1'b0;
            hdmi_tx_rdata <= 8'd0;
            hdmi_tx_gdata <= 8'd0;
            hdmi_tx_bdata <= 8'd0;
        end else begin
            if( !vid_cnt )
              {hdmi_tx_bdata,hdmi_tx_gdata,hdmi_tx_rdata} <= rb_swap ? {rgb_datax2[31:24],rgb_datax2[39:32],rgb_datax2[47:40]} : rgb_datax2[47:24];
            else
              {hdmi_tx_bdata,hdmi_tx_gdata,hdmi_tx_rdata} <= rb_swap ? {rgb_datax2[7:0],rgb_datax2[15:8],rgb_datax2[23:16]} : rgb_datax2[23:0];
            vid_cnt <= ~vid_cnt;
        end
    end
end

//==============================================================================
// 按键(低有效, 按下=0; 短按=<600ms, 长按=按住>=600ms):
//   KEY1(GPIOR_22) 短按 = 阈值档位 +1(边缘更少) / 长按 = 边缘颜色循环
//                  边缘颜色: 0=按方向彩(竖红/横绿/斜黄青) 1=统一红 2=统一白 3=统一绿
//   KEY2(GPIOR_21) 短按 = 阈值档位 -1(边缘更多) / 长按 = 冻结/回放(DDR帧缓存)
//   KEY3(GPIOL_03) 短按 = 前置滤波循环(中值->高斯->直通->中值+高斯) / 长按 = 显示模式循环
//
// 摄像头曝光/增益/白平衡调节模式(不影响上面的正常模式):
//   KEY1+KEY2 同时按住再松开(组合键) = 进入/退出调节模式
//   KEY1+KEY3 同时按住再松开(调节模式内) = 曝光/增益页 <-> 白平衡页
//   调节模式中:
//     曝光/增益页: KEY1 短按=曝光- / 长按=曝光+; KEY2 短按=增益- / 长按=增益+
//     白平衡页:   KEY1 短按=红-   / 长按=红+;   KEY2 短按=蓝-   / 长按=蓝+
//     KEY3 短按 = 恢复当前页默认值; KEY3 长按 = 退出调节模式
//   调节模式 LED:
//     曝光/增益页: D7~D4=曝光档位(15=最大), D3~D0=增益档位(6=默认4.813x)
//     白平衡页:   D7~D4=红增益档位(8=默认), D3~D0=蓝增益档位(8=默认)
//
// 正常模式 LED: D7~D4=阈值档位(二进制) | D3=冻结中(常亮)
//      D2: 模式0=灭 模式1=常亮 模式2=快闪 模式3=慢闪 模式4=识别到形状时常亮
//      D1=视频心跳 | D0=DDR3校准心跳
//==============================================================================
wire k_sh0, k_lg0, k_dn0, k_sh1, k_lg1, k_dn1, k_sh2, k_lg2, k_dn2;

key_ctrl u_key0 (.clk(hdmi_tx_slow_clk), .rst_n(vid_rst_n), .key_n(key_i[0]),
                 .o_short(k_sh0), .o_long(k_lg0), .o_down(k_dn0));
key_ctrl u_key1 (.clk(hdmi_tx_slow_clk), .rst_n(vid_rst_n), .key_n(key_i[1]),
                 .o_short(k_sh1), .o_long(k_lg1), .o_down(k_dn1));
key_ctrl u_key2 (.clk(hdmi_tx_slow_clk), .rst_n(vid_rst_n), .key_n(key_i[2]),
                 .o_short(k_sh2), .o_long(k_lg2), .o_down(k_dn2));

// KEY1+KEY2 组合键检测。两键都按住后锁定，等两键全部松开并静默 80ms 才产生
// 一次模式切换；静默期同时屏蔽这两键的普通短按/长按动作。
localparam integer CAM_COMBO_QUIET_CNT = 148500000 / 1000 * 80;
reg [23:0] combo_quiet_cnt;
reg        combo12_active;
reg        combo12_fire;
wire       both_key12 = k_dn0 & k_dn1;
wire       cam_combo_block = combo12_active | combo12_fire;

always @(posedge hdmi_tx_slow_clk or negedge vid_rst_n) begin
    if (!vid_rst_n) begin
        combo_quiet_cnt <= 24'd0;
        combo12_active  <= 1'b0;
        combo12_fire    <= 1'b0;
    end else begin
        combo12_fire <= 1'b0;
        if (both_key12) begin
            combo12_active  <= 1'b1;
            combo_quiet_cnt <= 24'd0;
        end else if (combo12_active) begin
            if (!k_dn0 && !k_dn1) begin
                if (combo_quiet_cnt >= CAM_COMBO_QUIET_CNT) begin
                    combo_quiet_cnt <= 24'd0;
                    combo12_active  <= 1'b0;
                    combo12_fire    <= 1'b1;
                end else begin
                    combo_quiet_cnt <= combo_quiet_cnt + 1'b1;
                end
            end else begin
                combo_quiet_cnt <= 24'd0;
            end
        end
    end
end

// KEY1+KEY3 is active only inside the camera adjustment mode.  It toggles
// between exposure/gain and white-balance pages after both keys are released.
reg [23:0] combo13_quiet_cnt;
reg        combo13_active;
reg        combo13_fire;
wire       both_key13 = k_dn0 & k_dn2;
wire       cam_combo13_block = combo13_active | combo13_fire;

always @(posedge hdmi_tx_slow_clk or negedge vid_rst_n) begin
    if (!vid_rst_n) begin
        combo13_quiet_cnt <= 24'd0;
        combo13_active    <= 1'b0;
        combo13_fire      <= 1'b0;
    end else begin
        combo13_fire <= 1'b0;
        if (cam_adj_mode && both_key13) begin
            combo13_active    <= 1'b1;
            combo13_quiet_cnt <= 24'd0;
        end else if (combo13_active) begin
            if (!k_dn0 && !k_dn2) begin
                if (combo13_quiet_cnt >= CAM_COMBO_QUIET_CNT) begin
                    combo13_quiet_cnt <= 24'd0;
                    combo13_active    <= 1'b0;
                    combo13_fire      <= 1'b1;
                end else begin
                    combo13_quiet_cnt <= combo13_quiet_cnt + 1'b1;
                end
            end else begin
                combo13_quiet_cnt <= 24'd0;
            end
        end
    end
end

// 曝光档位 0..15：16 个等步长档位，15 = 当前 ROM 基线 0x0BAD(2*1500-11 半行)。
// 保持默认档位 15，不改变已经确认的启动亮度；需要更亮时优先加增益。
function [15:0] exp_code_from_level;
    input [3:0] level;
    begin
        case (level)
            4'd0 : exp_code_from_level = 16'd187;
            4'd1 : exp_code_from_level = 16'd374;
            4'd2 : exp_code_from_level = 16'd561;
            4'd3 : exp_code_from_level = 16'd748;
            4'd4 : exp_code_from_level = 16'd935;
            4'd5 : exp_code_from_level = 16'd1122;
            4'd6 : exp_code_from_level = 16'd1309;
            4'd7 : exp_code_from_level = 16'd1496;
            4'd8 : exp_code_from_level = 16'd1683;
            4'd9 : exp_code_from_level = 16'd1870;
            4'd10: exp_code_from_level = 16'd2057;
            4'd11: exp_code_from_level = 16'd2244;
            4'd12: exp_code_from_level = 16'd2431;
            4'd13: exp_code_from_level = 16'd2618;
            4'd14: exp_code_from_level = 16'd2805;
            default: exp_code_from_level = 16'h0BAD;
        endcase
    end
endfunction

// 增益档位来自 SC431HAI 手册表 2-7 的实际编码。默认档位 6 = 0x81/0x32
// (4.813x)，与 ROM 启动值完全一致；不改变固定白平衡 R/G/B_GAIN。
function [7:0] gain_coarse_from_level;
    input [3:0] level;
    begin
        case (level)
            4'd0: gain_coarse_from_level = 8'h00; // 1.000x
            4'd1: gain_coarse_from_level = 8'h80; // 1.540x
            4'd2: gain_coarse_from_level = 8'h80; // 2.118x
            4'd3: gain_coarse_from_level = 8'h80; // 2.503x
            4'd4: gain_coarse_from_level = 8'h81; // 3.080x
            4'd5: gain_coarse_from_level = 8'h81; // 3.850x
            4'd6: gain_coarse_from_level = 8'h81; // 4.813x (默认)
            4'd7: gain_coarse_from_level = 8'h81; // 6.064x
            4'd8: gain_coarse_from_level = 8'h83; // 7.700x
            4'd9: gain_coarse_from_level = 8'h83; // 9.240x
            default: gain_coarse_from_level = 8'h81;
        endcase
    end
endfunction

function [7:0] gain_fine_from_level;
    input [3:0] level;
    begin
        case (level)
            4'd0: gain_fine_from_level = 8'h20;
            4'd1: gain_fine_from_level = 8'h20;
            4'd2: gain_fine_from_level = 8'h2C;
            4'd3: gain_fine_from_level = 8'h34;
            4'd4: gain_fine_from_level = 8'h20;
            4'd5: gain_fine_from_level = 8'h28;
            4'd6: gain_fine_from_level = 8'h32;
            4'd7: gain_fine_from_level = 8'h3F;
            4'd8: gain_fine_from_level = 8'h28;
            4'd9: gain_fine_from_level = 8'h30;
            default: gain_fine_from_level = 8'h32;
        endcase
    end
endfunction

reg [3:0] th_level;     // 阈值档位 0~15
reg [1:0] prefilt;      // 0=中值 1=高斯 2=直通 3=中值+高斯(默认, 最干净)
reg [2:0] dmode;        // 0=分屏(灰度|彩边) 1=全屏彩色+彩边 2=分屏(灰度|二值边缘)
                        // 3=画中画(彩色+右下角二值边缘小窗) 4=形状识别(彩色原图+红色Sobel边+识别框)
reg [1:0] color_mode;   // 边缘颜色: 0=按梯度方向彩 1=统一红 2=统一白 3=统一绿
reg       rb_swap;      // 1 = 交换红蓝通道(修正 Bayer 相位导致的偏色); 现固定为 RB_SWAP_DEF
// Mode 4 is a full-screen original-colour + red-Sobel view.  Only the two
// split-screen modes use the 960-pixel boundary.
wire [10:0] split_col = ((dmode == 3'd0) || (dmode == 3'd2)) ? 11'd960 : 11'd0;
wire        pip_on    = (dmode == 3'd3);   // 画中画
wire        shape_on  = (dmode == 3'd4);   // 形状识别

always @(posedge hdmi_tx_slow_clk or negedge vid_rst_n) begin
    if (!vid_rst_n) begin
        th_level   <= 4'd5;      // 默认档位 5 (中值+高斯级联后, 该档位在真实图像上最干净)
        prefilt    <= 2'd3;
        dmode      <= 3'd0;
        color_mode <= 2'd1;      // 默认统一红色，避免只有竖直边呈红色
        rb_swap    <= RB_SWAP_DEF;
        free_on    <= 1'b0;
        cam_adj_mode   <= 1'b0;
        exp_level      <= 4'd15;
        gain_level     <= 4'd6;
        wb_page        <= 1'b0;
        wb_r_level     <= 4'd8;
        wb_b_level     <= 4'd8;
        key_req_tgl    <= 1'b0;
        key_sel    <= 2'd0;
        key_exp    <= 16'h0BAD;
        key_gain_c <= 8'h81;
        key_gain_f <= 8'h32;
    end else begin
        if (combo12_fire) begin
            cam_adj_mode <= ~cam_adj_mode;       // 组合键切换调节模式
            wb_page      <= 1'b0;
        end else if (combo13_fire && cam_adj_mode) begin
            wb_page <= ~wb_page;                 // KEY1+KEY3: 曝光/增益 <-> 白平衡
        end else if (!cam_combo_block && !cam_combo13_block) begin
            if (cam_adj_mode) begin
                if (k_lg2) begin
                    cam_adj_mode <= 1'b0;         // KEY3 长按退出调节模式
                    wb_page      <= 1'b0;
                end else if (!cfg_busy) begin
                    if (k_sh2) begin
                        if (wb_page) begin       // 白平衡页: 恢复 R/B 默认
                            wb_r_level <= 4'd8;
                            wb_b_level <= 4'd8;
                        end else begin           // 曝光/增益页: 恢复传感器默认
                            exp_level      <= 4'd15;
                            gain_level     <= 4'd6;
                            key_sel    <= 2'd2;
                            key_exp    <= 16'h0BAD;
                            key_gain_c <= 8'h81;
                            key_gain_f <= 8'h32;
                            key_req_tgl    <= ~key_req_tgl;
                        end
                    end else if (wb_page) begin
                        if (k_lg0) begin          // 红通道 +
                            if (wb_r_level != 4'd15) wb_r_level <= wb_r_level + 1'b1;
                        end else if (k_sh0) begin // 红通道 -
                            if (wb_r_level != 4'd0) wb_r_level <= wb_r_level - 1'b1;
                        end else if (k_lg1) begin // 蓝通道 +
                            if (wb_b_level != 4'd15) wb_b_level <= wb_b_level + 1'b1;
                        end else if (k_sh1) begin // 蓝通道 -
                            if (wb_b_level != 4'd0) wb_b_level <= wb_b_level - 1'b1;
                        end
                    end else if (k_lg0) begin     // 曝光 +
                        if (exp_level != 4'd15) begin
                            exp_level   <= exp_level + 1'b1;
                            key_sel <= 2'd0;
                            key_exp <= exp_code_from_level(exp_level + 1'b1);
                            key_req_tgl <= ~key_req_tgl;
                        end
                    end else if (k_sh0) begin     // 曝光 -
                        if (exp_level != 4'd0) begin
                            exp_level   <= exp_level - 1'b1;
                            key_sel <= 2'd0;
                            key_exp <= exp_code_from_level(exp_level - 1'b1);
                            key_req_tgl <= ~key_req_tgl;
                        end
                    end else if (k_lg1) begin     // 增益 +
                        if (gain_level != 4'd9) begin
                            gain_level     <= gain_level + 1'b1;
                            key_sel    <= 2'd1;
                            key_gain_c <= gain_coarse_from_level(gain_level + 1'b1);
                            key_gain_f <= gain_fine_from_level(gain_level + 1'b1);
                            key_req_tgl    <= ~key_req_tgl;
                        end
                    end else if (k_sh1) begin     // 增益 -
                        if (gain_level != 4'd0) begin
                            gain_level     <= gain_level - 1'b1;
                            key_sel    <= 2'd1;
                            key_gain_c <= gain_coarse_from_level(gain_level - 1'b1);
                            key_gain_f <= gain_fine_from_level(gain_level - 1'b1);
                            key_req_tgl    <= ~key_req_tgl;
                        end
                    end
                end
            end else begin
                if (k_sh0 && (th_level != 4'hF)) th_level <= th_level + 1'b1;   // 阈值↑ 边缘更少
                if (k_sh1 && (th_level != 4'h0)) th_level <= th_level - 1'b1;   // 阈值↓ 边缘更多
                if (k_sh2) prefilt <= (prefilt == 2'd3) ? 2'd0 : prefilt + 1'b1;
                if (k_lg2) begin
                    dmode <= (dmode == 3'd4) ? 3'd0 : dmode + 1'b1;   // 显示模式循环
                end
                if (k_lg0) color_mode <= (color_mode == 2'd3) ? 2'd0 : color_mode + 1'b1; // 边缘颜色循环
                if (k_lg1) free_on <= ~free_on;                                  // 冻结/回放
            end
        end
    end
end

// 档位 -> 阈值表 (阈值作用在 幅值>>3 的 0~255 尺度)
reg [7:0] th_low_v, th_high_v;
always @(*) begin
    case (th_level)
        4'd0 : begin th_low_v = 8'd1;  th_high_v = 8'd2;   end
        4'd1 : begin th_low_v = 8'd1;  th_high_v = 8'd3;   end
        4'd2 : begin th_low_v = 8'd2;  th_high_v = 8'd5;   end
        4'd3 : begin th_low_v = 8'd3;  th_high_v = 8'd7;   end
        4'd4 : begin th_low_v = 8'd4;  th_high_v = 8'd9;   end
        4'd5 : begin th_low_v = 8'd5;  th_high_v = 8'd12;  end
        4'd6 : begin th_low_v = 8'd7;  th_high_v = 8'd15;  end
        4'd7 : begin th_low_v = 8'd9;  th_high_v = 8'd20;  end
        4'd8 : begin th_low_v = 8'd12; th_high_v = 8'd25;  end
        4'd9 : begin th_low_v = 8'd16; th_high_v = 8'd32;  end
        4'd10: begin th_low_v = 8'd20; th_high_v = 8'd40;  end
        4'd11: begin th_low_v = 8'd25; th_high_v = 8'd50;  end
        4'd12: begin th_low_v = 8'd32; th_high_v = 8'd64;  end
        4'd13: begin th_low_v = 8'd40; th_high_v = 8'd80;  end
        4'd14: begin th_low_v = 8'd50; th_high_v = 8'd100; end
        default: begin th_low_v = 8'd64; th_high_v = 8'd128; end
    endcase
end

// 状态/档位指示灯 (放在声明之后, 避免前向引用)
// 调节模式 曝光/增益页: D7~D4 = 曝光档位, D3~D0 = 增益档位。
// 调节模式 白平衡页:   D7~D4 = 红增益档位, D3~D0 = 蓝增益档位。
// 正常模式: D7~D4 = 阈值档位; D3 = 冻结; D2 = 显示/识别状态; D1/D0 = 心跳。
wire [3:0] cam_low_led = wb_page ? wb_b_level : gain_level;
assign led[7:4] = cam_adj_mode ? (wb_page ? wb_r_level : exp_level) : th_level;
assign led[3]   = cam_adj_mode ? cam_low_led[3] : free_on;
assign led[2]   = cam_adj_mode ? cam_low_led[2] :
                  (dmode == 3'd1) ? 1'b1 :
                  (dmode == 3'd2) ? edge_beat[22] :
                  (dmode == 3'd3) ? edge_beat[23] :
                  (dmode == 3'd4) ? shape_hit : 1'b0;
assign led[1]   = cam_adj_mode ? cam_low_led[1] : edge_beat[23];
assign led[0]   = cam_adj_mode ? cam_low_led[0] : (cal_done ? cnt[24] : 1'b0);
//==============================================================================
// 中值/高斯级联 + 灰度化 + Canny(简化) 边缘检测 + 分屏/彩色叠加/画中画 (edge_proc.v)
//   左半屏 = 摄像头原灰度图; 右半屏 = 二值边缘图 或 彩色原图+方向彩色边(i_overlay)
//   3x3 中值/高斯级联(可切) -> Sobel -> 8 分带方向 -> NMS -> 双阈值滞回
//   阈值由按键档位表给出(见下面用户按键部分)
//   输出附带 o_edge/o_dir16/o_row 给 shape_proc 做圆/矩形识别
//==============================================================================
wire [23:0] e0_rgb;
wire        e0_de, e0_hs, e0_vs;
wire        e0_edge_flag;
wire        e0_edge;
wire [3:0]  e0_dir16;
wire [10:0] e0_row;

edge_proc #(
    .H_ACTIVE       (HACT),
    .SPLIT_COL      (HACT/2),
    .TH_LOW         (SOBEL_TH_LOW),
    .TH_HIGH        (SOBEL_TH_HIGH),
    .GRAY_SHIFT_ADD (SOBEL_GRAY_SHIFTADD),
    .ISOLATE_MIN    (4),
    .MAG_ISO        (1),
    .NMS_INTERP     (1),
    .HYST_CONN      (1),
    .EDGE_COLOR_DIR (1),
    .TOP_BLANK      (8),      // 帧首 8 行不出边缘(消除行缓存跨帧预热白线)
    .COL_BLANK      (2),      // 行首行尾 2 列不出边缘(消除窗口填充白线)
    .ISO_EN         (0),      // 0 = 只用连通性传播(实测毛刺更少)
    .V_ACTIVE       (VACT),
    .PIP_W          (640),
    .PIP_H          (480)
) u_edge_proc (
    .clk          (hdmi_tx_slow_clk),
    .rst_n        (vid_rst_n),
    .i_de         (hdmi_tx_de),
    .i_hs         (hdmi_tx_hs),
    .i_vs         (hdmi_tx_vs),
    .i_rgb        ({hdmi_tx_bdata, hdmi_tx_gdata, hdmi_tx_rdata}),
    .i_th_low     (th_low_v),
    .i_th_high    (th_high_v),
    .i_prefilt    (prefilt),
    .i_overlay    ((dmode != 3'd2) && (dmode != 3'd4)),
    .i_split_col  (split_col),       // 0=全屏叠加, 960=分屏
    .i_color_mode (color_mode),
    .i_pip        (pip_on),
    .i_shape_mode (shape_on),

    .o_de         (e0_de),
    .o_hs         (e0_hs),
    .o_vs         (e0_vs),
    .o_rgb        (e0_rgb),
    .o_edge_flag  (e0_edge_flag),
    .o_edge       (e0_edge),
    .o_dir16      (e0_dir16),
    .o_row        (e0_row)
);

//==============================================================================
// 简易目标识别(高阶挑战): 圆/矩形/三角形细外接框 + 左上角中文标签
//   模式 4: 左半屏灰度原图, 右半屏白边黑底; 检测与画框都只在右半屏(ROI 起点 960)
//   矩形: 行程法找上下两条水平长边 -> 候选框 -> 下一帧四边覆盖率验证
//   圆形: 半径扫描 Hough, 边缘像素沿梯度方向 +/-R 投票, 峰值超阈值即锁定圆心
//   三角形: 最长水平底边 + 两侧斜线覆盖率验证
//   仅显示模式 4 使能; 其它模式纯直通(画面不受影响)
//==============================================================================
wire [23:0] edge_rgb;
wire        edge_de, edge_hs, edge_vs;
wire        shape_hit;
wire [3:0]  shape_dbg;

//==============================================================================
// 形状识别器选择
//   SHAPE_LEGACY = 0 : 墨迹分割 + 填充率分类 (默认)
//       找"比周围更暗的笔迹", 不依赖 Sobel 方向, 细线/浅线/光照不均都能用
//   SHAPE_LEGACY = 1 : 旧的参数化检测器 (固定半径 Hough + 固定行程)
//       需要目标线条很粗、对比度很高时才稳
//==============================================================================
localparam integer SHAPE_LEGACY = 0;

wire [7:0] ink_thr = th_low_v + 8'd8;

generate
if (SHAPE_LEGACY) begin : g_legacy_shape
    shape_proc #(
        .H_ACTIVE       (HACT),
        .V_ACTIVE       (VACT),
        .BIN_SHIFT      (3),
        .ACC_W          (256),
        .RECT_LEN_MIN   (64),
        .RECT_LEN_MAX   (360),
        .RECT_GAP_MIN   (40),
        .RECT_SIDE_MIN  (40),
        .RUN_GAP        (12),
        .TOL_PX         (10),
        .R0             (40),
        .R_STEP         (16),
        .R_NUM          (10),
        .HOUGH_TH       (20),
        .CIRC_RATIO_SHIFT (4),
        .CIRC_ARC_TOL   (16),
        .CIRC_ARC_MIN   (6),
        .CIRC_CORNER_MAX(64),
        .CIRC_HOLD      (90),
        .RECT_HOLD      (90),
        .TRI_W_MIN      (72)
    ) u_shape_proc (
        .clk         (hdmi_tx_slow_clk),
        .rst_n       (vid_rst_n),
        .i_de        (e0_de),
        .i_hs        (e0_hs),
        .i_vs        (e0_vs),
        .i_rgb       (e0_rgb),
        .i_edge      (e0_edge),
        .i_dir16     (e0_dir16),
        .i_row       (e0_row),
        .i_en        (shape_on),
        .i_draw_x0   (11'd0),
        .o_de        (edge_de),
        .o_hs        (edge_hs),
        .o_vs        (edge_vs),
        .o_rgb       (edge_rgb),
        .o_shape_hit (shape_hit),
        .o_dbg       (shape_dbg)
    );
end else begin : g_ink_shape
    shape_ink #(
        .H_ACTIVE     (HACT),
        .V_ACTIVE     (VACT),
        .LAB          (32),
        .MIN_SPAN     (400),
        .MIN_WH       (14),
        .THR_DEF      (18),
        .Y_OFS        (2)
    ) u_shape_proc (
        .clk         (hdmi_tx_slow_clk),
        .rst_n       (vid_rst_n),
        .i_de        (hdmi_tx_de),
        .i_hs        (hdmi_tx_hs),
        .i_vs        (hdmi_tx_vs),
        .i_rgb       ({hdmi_tx_bdata, hdmi_tx_gdata, hdmi_tx_rdata}),
        .i_edge      (e0_edge),
        .i_dir16     (e0_dir16),
        .i_row       (e0_row),
        .i_en        (shape_on),
        .i_draw_x0   (12'd0),
        .i_thr       (ink_thr),
        .o_de        (edge_de),
        .o_hs        (edge_hs),
        .o_vs        (edge_vs),
        .o_rgb       (edge_rgb),
        .o_shape_hit (shape_hit),
        .o_dbg       (shape_dbg)
    );
end
endgenerate

// 诊断信号汇合(避免未使用告警)
wire dbg_sink = e0_edge_flag ^ shape_dbg[3] ^ shape_dbg[2] ^ shape_dbg[1] ^ shape_dbg[0];
//==============================================================================
// MIPI DSI
//==============================================================================
// signal 

reg		[25:0]	r_rst_cnt;

wire	[31:0]	w_axi_rdata;
wire			w_axi_awready;
wire			w_axi_wready;
wire			w_axi_arready;
wire			w_axi_rvalid;
wire			w_axi_bvalid;

wire	[6:0]	w_axi_awaddr;
wire			w_axi_awvalid;
wire	[31:0]	w_axi_wdata;
wire			w_axi_wvalid;
wire			w_axi_bready;
wire	[6:0]	w_axi_araddr;
wire			w_axi_arvalid;
wire			w_axi_rready;

wire			w_confdone;
assign w_confdone = 1'b1;

assign  LCD_POWER 			= i_arstn;
assign	LCD_RST_P	      = ~w_arstn;//~r_rst_cnt[23]; LCD复位先释放，然后再释放MIPI 的复位
assign	mipi_dp_clk_RST		= ~i_arstn;
assign	mipi_dp_data0_RST	= ~i_arstn;
assign	mipi_dp_data1_RST	= ~i_arstn;
assign	mipi_dp_data2_RST	= ~i_arstn;
assign	mipi_dp_data3_RST	= ~i_arstn;
////////////////////////////////////////////////////////////////
always@(posedge i_mipi_clk or  negedge w_arstn )
begin
  if ( !w_arstn ) begin
    r_rst_cnt	<= 'd0;
  end else 		
    r_rst_cnt	<= r_rst_cnt[25] ? r_rst_cnt :r_rst_cnt + 1'b1;
end

wire axi_rst_n = r_rst_cnt[25];

reg [26:0] dly_cnt = 'd0;

always @( posedge vid_clk_dvi2 or negedge w_arstn )
begin
  if( !w_arstn )
    dly_cnt <= 'd0;
  else if( w_confdone )
    dly_cnt <= dly_cnt[26] ? dly_cnt :(dly_cnt + 1'b1);
  else 
    dly_cnt <= 'd0;
end 

assign vid_rst_n = dly_cnt[26];




wire hs;
wire vs;
wire de;
wire [7:0] rgb_r;
wire [7:0] rgb_g;
wire [7:0] rgb_b;
  
  // color_bar_rgb #(
  // .HS_POLORY 		(1'b1		),
  // .VS_POLORY 		(1'b1		),
  // .H_FRONT_PORCH 	(HFP		),///2
  // .H_SYNC 		(HSP		),//
  // .H_VALID 		(MAX_HRES	),//
  // .H_BACK_PORCH 	(HBP		),//
  // .V_FRONT_PORCH 	(VFP		),
  // .V_SYNC 		(VSP		),
  // .V_VALID 		(MAX_VRES	),
  // .V_BACK_PORCH 	(VBP		),
  // .TEST_MODE 		(2'd2)
  // )u_color_bar_rgb1(
  // /*i*/.clk	(vid_clk_dvi2),//(vid_clk),
  // /*i*/.rst_n	(vid_rst_n),
  // /*o*/.hs	(hs),
  // /*o*/.vs	(vs),
  // /*o*/.de	(de),
  // // /*O*/.h_cnt (h_cnt),
  // // /*O*/.v_cnt (v_cnt),
  // /*o*/.rgb_r	(rgb_r),    //像素数据、红色分量
  // /*o*/.rgb_g	(rgb_g),    //像素数据、绿色分量
  // /*o*/.rgb_b (rgb_b)    //像素数据、蓝色分量
  
  // );

//=================================================================================
//hdmi tx ctrl 
//=================================================================================
wire                            video_hs;
wire                            video_vs;
wire                            video_de;
wire[7:0]                       video_r;
wire[7:0]                       video_g;
wire[7:0]                       video_b;

assign tmds_tx_data0_TX_OE = 1'b1;
assign tmds_tx_data1_TX_OE = 1'b1;
assign tmds_tx_data2_TX_OE = 1'b1;
assign tmds_tx_clk_TX_OE   = 1'b1;

assign tmds_tx_data0_TX_RST = 1'b0;
assign tmds_tx_data1_TX_RST = 1'b0;
assign tmds_tx_data2_TX_RST = 1'b0;
assign tmds_tx_clk_TX_RST   = 1'b0;

// 	  color_bar color_bar_m0(
// 	.clk(hdmi_tx_slow_clk),
// 	.rst(~vid_rst_n),
// 	.hs(video_hs),
// 	.vs(video_vs),
// 	.de(video_de),
// 	.rgb_r(video_r),
// 	.rgb_g(video_g),
// 	.rgb_b(video_b)
// );

wire [9:0] tmds_data0;
wire [9:0] tmds_data1;
wire [9:0] tmds_data2;
wire [9:0] tmds_clk ;
wire [7:0] gamma_rdata ;
wire [7:0] gamma_gdata ;
wire [7:0] gamma_bdata ;
wire gamma_valid;
wire gamma_hs;
wire gamma_vs;
wire gamma_sel;

wire [7:0] con_rdata ;
wire [7:0] con_gdata ;
wire [7:0] con_bdata ;
wire con_de;
wire con_hs;
wire con_vs;
wire con_sel;

wire [7:0] BRIGHT_SIG    ;
wire CONTRAST_SIG_G;
wire CONTRAST_SIG_R;
wire [7:0] CONTRAST_G    ;
wire CONTRAST_SIG_B;
wire [7:0] BRIGHT        ;
wire [7:0] CONTRAST_R    ;


wire CONTRAST_B    ;


gamma_correction
#(
	.DATA_WIDTH		(8),
	.GAMMA_CURVE	("./rtl/gamma_conrrection/gamma_2p2_8bit.mem")
)
inst_gamma_correction
(
	.i_pclk		(hdmi_tx_slow_clk),
	.i_rstn		(vid_rst_n),
	
	.i_red		(edge_rgb[7:0]  ),//(con_rdata  ),//
	.i_green	(edge_rgb[15:8] ),//(con_gdata  ),//
	.i_blue		(edge_rgb[23:16]),//(con_bdata  ),//
  .i_valid	(edge_de   ),//(con_de     ),//
  .i_hs		(edge_hs   ),//(con_hs     ),//
  .i_vs		(edge_vs   ),//(con_vs     ),//
	.o_blue		(gamma_bdata		),
  .o_green	(gamma_gdata	  ),
	.o_red		(gamma_rdata		),
	.o_valid	(gamma_valid	  ),
  .o_hs     (gamma_hs       ),
  .o_vs     (gamma_vs       )
);

dvi_encoder dvi_encoder_m0
(
	.pixelclk      		(hdmi_tx_slow_clk          ),// system clock
	.rst_p         		(~vid_rst_n      ),// reset
	// edge_rgb is {B,G,R}, matching edge_proc and gamma_correction.  The old
	// fallback connected R to the blue lane and B to the red lane.
	.i_bdata      		(gamma_sel ? gamma_bdata	: edge_rgb[23:16] 	),
	.i_gdata     		  (gamma_sel ? gamma_gdata	: edge_rgb[15:8]    ),
	.i_rdata       		(gamma_sel ? gamma_rdata	: edge_rgb[7:0]   	),
  .i_de            	(gamma_sel ? gamma_valid	: edge_de      ),//(hdmi_tx_de    ),//(video_de           ),//(rx_de		  ),   //
	.i_hs         		(gamma_sel ? gamma_hs	: edge_hs    ),//(video_hs           ),//(rx_hsync	),   //   
	.i_vs         		(gamma_sel ? gamma_vs	: edge_vs    ),//(video_vs           ),//(rx_vsync	),   //  
	
  .video_format     (video_format), //// 00 = RGB, 01 = YCbCr 4:2:2, 10 = YCbCr 4:4:4
  .video_VIC        (0),
	.audio_L			    (audio_L),
  .audio_R			    (audio_R),
  .audio_valid		  (audio_valid),
  .audio_N                (6144),     //(20'h01880),//     
  .audio_CTS              (148500),      //(20'h28488),//      
  .audio_sample_frequency (audio_samp_freq),    //(3'b000),// 
  .audio_word_length      ({audio_samp_word_len,audio_max_word_length}),//(4'b1011),//

	.tmds_data0    		(tmds_data0),
  .tmds_data1    		(tmds_data1),
  .tmds_data2    		(tmds_data2),
  .tmds_clk      	  (tmds_clk  )
);
assign tmds_tx_clk_TX_DATA   = ~tmds_clk;
assign tmds_tx_data0_TX_DATA = ~tmds_data0;
assign tmds_tx_data1_TX_DATA = ~tmds_data1;
assign tmds_tx_data2_TX_DATA = ~tmds_data2;

debug_edb_top debug_edb_top_inst (
    .bscan_CAPTURE      ( jtag_inst1_CAPTURE  ),
    .bscan_DRCK         ( jtag_inst1_DRCK     ),
    .bscan_RESET        ( jtag_inst1_RESET    ),
    .bscan_RUNTEST      ( jtag_inst1_RUNTEST  ),
    .bscan_SEL          ( jtag_inst1_SEL      ),
    .bscan_SHIFT        ( jtag_inst1_SHIFT    ),
    .bscan_TCK          ( jtag_inst1_TCK      ),
    .bscan_TDI          ( jtag_inst1_TDI      ),
    .bscan_TMS          ( jtag_inst1_TMS      ),
    .bscan_UPDATE       ( jtag_inst1_UPDATE   ),
    .bscan_TDO          ( jtag_inst1_TDO      ),
    .vio0_clk           ( vid_clk_dvi2        ),
    .vio0_r_gain        ( r_gain              ),
    .vio0_g_gain        ( g_gain              ),
    .vio0_b_gain        ( b_gain              ),

    .vio0_gamma_sel     (gamma_sel),
    .la0_clk            ( vid_clk_dvi2        ),
    .la0_ch0_g          ( ch0_g               ),
    .la0_ch0_b          ( ch0_b               ),
    .la0_ch0_de         ( ch0_de              ),
    .la0_cal_done       ( cal_done            )
);

endmodule

//==============================================================================
// 按键去抖 + 短按/长按识别 (15ms 去抖, 600ms 长按)
//   o_short: 短按脉冲(松开时产生, 若期间未触发长按)
//   o_long : 长按脉冲(按住达 600ms 时产生)
//==============================================================================
module key_ctrl #(
    parameter integer CLK_HZ  = 148500000,
    parameter integer DB_MS   = 15,
    parameter integer LONG_MS = 600
)(
    input  wire clk,
    input  wire rst_n,
    input  wire key_n,      // 低有效: 按下 = 0
    output reg  o_short,
    output reg  o_long,
    output wire o_down      // 去抖后的按下电平(1=按下), 用于组合键检测
);
    localparam integer DB_CNT   = (CLK_HZ / 1000) * DB_MS;
    localparam integer LONG_CNT = (CLK_HZ / 1000) * LONG_MS;

    reg [2:0] sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sync <= 3'b111;
        else        sync <= {sync[1:0], key_n};
    end

    wire pin_pressed = ~sync[2];
    reg [27:0] db_cnt;
    reg [27:0] hold_cnt;
    reg        stable;          // 去抖后电平: 1 = 按下
    reg        long_fired;
    assign o_down = stable;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            db_cnt <= 28'd0; hold_cnt <= 28'd0;
            stable <= 1'b0; long_fired <= 1'b0;
            o_short <= 1'b0; o_long <= 1'b0;
        end else begin
            o_short <= 1'b0;
            o_long  <= 1'b0;

            // 去抖: 输入与稳定值不同则计数, 计满才更新稳定值
            if (pin_pressed == stable)
                db_cnt <= 28'd0;
            else if (db_cnt >= DB_CNT) begin
                db_cnt   <= 28'd0;
                stable   <= pin_pressed;
                hold_cnt <= 28'd0;
                if (pin_pressed)
                    long_fired <= 1'b0;             // 按下瞬间
                else if (!long_fired)
                    o_short <= 1'b1;                // 松开且未长按 -> 短按
            end else
                db_cnt <= db_cnt + 1'b1;

            // 长按计时
            if (stable) begin
                if (hold_cnt >= LONG_CNT) begin
                    if (!long_fired) begin
                        long_fired <= 1'b1;
                        o_long <= 1'b1;
                    end
                end else
                    hold_cnt <= hold_cnt + 1'b1;
            end
        end
    end
endmodule
