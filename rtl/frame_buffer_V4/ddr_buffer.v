

`timescale 1ps/1ps
module ddr_buffer #(
parameter AXI_DATA_WIDTH = 512, //!AXI接口位宽
parameter AXI_ADDR_WIDTH = 33,  //!AXI地址位宽
parameter WR_FIFO_DEPTH	= 1024, //!Write fifo depth   
parameter RD_FIFO_DEPTH = 1024, //!read fifo depth
parameter FB_NUM			= 	3,//2 buffer ,3 buffer   
parameter MAX_VID_WIDTH		=	1920 ,//video width 
parameter MAX_VID_HIGHT		=	1080 ,//wideo height
parameter START_ADDR				=	33'h000201900,
parameter BURST_LEN       = 8'd15,
parameter I_VID_WIDTH = 16,
parameter AXI_STRB_WIDTH = AXI_DATA_WIDTH/8//!总共的字节数量，= AXI_DATA_WIDTH/8

) (

input 											axi_clk,
input                       wr_clk_rst_n,
input                       axi_clk_rst_n,
input                       rd_clk_rst_n,
input 											rd_start,
input											wr_start, // pulse signal 
input                       freeze,   // 1 = hold the read bank, keep the write handshake alive

// input		[AXI_ADDR_WIDTH-1:0]				start_wr_addr,
// input		[AXI_ADDR_WIDTH-1:0]				start_rd_addr,
input 		[31:0] 								wr_busrt_len,
input 		[31:0] 								rd_burst_len,
output		[7:0] test_cnt ,

input                     wr_fifo_rst_p,
input											wr_fifo_wrclk,
input											wr_fifo_wren,
output											wr_fifo_wrfull,
input	 [AXI_DATA_WIDTH-1:0]					wr_fifo_wrdata, 
input											rd_fifo_rdclk,
input                       rd_fifo_rst_p,
output											rd_fifo_rdvalid,
output	 [AXI_DATA_WIDTH-1:0] 					rd_fifo_rddata,
output wire										rd_fifo_rdempty,
input	 wire									rd_fifo_rden,

output 	[5:0] 									awid,
output  [AXI_ADDR_WIDTH-1:0] 					awaddr,
output  [8-1:0] 					awlen,
output  [2:0] 									awsize,
output  [1:0] 									awburst,
output  [3:0] 									awcache,
output [2:0]                    awprot,
output  										awlock,
output  										awvalid,
output  										awcobuf,
output  										awapcmd,
output  										awallstrb,
output  										awqos,
input 											awready,

output [5:0] 									arid,
output  [AXI_ADDR_WIDTH-1:0] 					araddr,
output  [8-1:0] 					arlen,
output  [2:0] 									arsize,
output  [1:0] 									arburst,
output  										arlock,
output  										arvalid,
output  										arapcmd,
output 											arqos,
input 											arready,
output  [3:0] 									arcache,
output [2:0]                    arprot,
output  [AXI_DATA_WIDTH-1:0] 					wdata,  
output	[31:0]									test_wdata,
output	[31:0]									test_rdata,
output  [AXI_STRB_WIDTH-1:0] 					wstrb,
output  										wlast,
output  										wvalid,
input 											wready,

input [5:0] 									rid,
input [AXI_DATA_WIDTH-1:0] 						rdata,
input 											rlast,
input 											rvalid,
output  										rready,
input [1:0] 									rresp,
output[31:0]									wr_fifo_rd_data_test,
input [5:0] 									bid,
input 											bvalid,
output  										bready,

output [7:0] test_BURST_LEN
);
assign test_BURST_LEN = BURST_LEN;

//=============================================================
//reg define 
//=============================================================   


//=============================================================
//wire define 
//============================================================= 



wire		                        wr_sw_ack           ;
wire		                        wr_sw 		          ;
wire		                        rd_sw			          ;
wire		                        rd_sw_ack           ;
wire		                        wr_idle             ;
wire		[AXI_ADDR_WIDTH-1:0]		wr_start_addr       ;
wire		[AXI_ADDR_WIDTH-1:0]    rd_start_addr_bank  ;
wire		[AXI_ADDR_WIDTH-1:0]    rd_start_addr       ;
reg                                     freeze_rd_lock      ;
reg		[AXI_ADDR_WIDTH-1:0]		freeze_rd_addr      ;
reg		[2:0]							freeze_rel_wait     ;
reg                                     rd_start_axi_d0     ;
reg                                     rd_start_axi_d1     ;
wire                                    pos_rd_start_axi    ;

//=============================================================  
//RTL                                                     
//=============================================================  



ddr_wr_buffer1 # (
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
    .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
    .WR_FIFO_DEPTH(WR_FIFO_DEPTH),
    .BURST_LEN (BURST_LEN        )
  )

  u_ddr_wr_buffer (//_inst
    .axi_clk(axi_clk),
    .rst_n(axi_clk_rst_n),//(rst_n),
    .start(wr_start),
    .start_addr(wr_start_addr),//(0),//
    .total_burst_len(wr_busrt_len),
/*i*/.bank_sw_ack	(wr_sw_ack   	),
/*o*/.bank_sw 		  (wr_sw  		 	),
    .wr_idle(wr_idle),
    .test_cnt(test_cnt),
    .wr_fifo_rst_p(wr_fifo_rst_p),
    .wr_fifo_wrclk(wr_fifo_wrclk),
    .wr_fifo_wren(wr_fifo_wren),
    .wr_fifo_wrfull(wr_fifo_wrfull),
    .wr_fifo_wrdata(wr_fifo_wrdata),
    .awid(awid),
    .awaddr(awaddr),
    .awlen(awlen),
    .awsize(awsize),
    .awburst(awburst),
    .awcache(awcache),
    .awlock(awlock),
    .awvalid(awvalid),
    .awcobuf(awcobuf),
    .awapcmd(awapcmd),
    .awallstrb(awallstrb),
    .awprot (awprot ),
    .awqos(awqos),
    .awready(awready),
    .wdata(wdata),
    .wstrb(wstrb),
    .wlast(wlast),
    .wvalid(wvalid),
    .wready(wready),
	  .bid	( bid			),
    .bvalid	( bvalid		),
    .bready	( bready		)
  );


assign test_wdata = wdata[32:0];
assign test_rdata = rdata[32:0];
//=============================================================================================
// FIFO RD_PROCESSING
//=============================================================================================
ddr_rd_buffer # (
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
    .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
    .RD_FIFO_DEPTH(RD_FIFO_DEPTH),
    .BURST_LEN (BURST_LEN        )
  )
  ddr_rd_buffer_inst (
    .axi_clk(axi_clk),
    .axi_clk_rst_n(axi_clk_rst_n),
    .start(rd_start),
    .start_addr(rd_start_addr),//(0),//
    .burst_len(rd_burst_len),
    .rd_fifo_rdclk(rd_fifo_rdclk),
    .rd_fifo_rst_p (rd_fifo_rst_p),
    .rd_fifo_rdvalid(rd_fifo_rdvalid),
    .rd_fifo_rddata(rd_fifo_rddata),
    .rd_fifo_rdempty(rd_fifo_rdempty),
    .rd_fifo_rden(rd_fifo_rden),
/*i*/.bank_sw_ack	(rd_sw_ack   	),
/*o*/.bank_sw 		  (rd_sw  		 	),
    .arid(arid),
    .araddr(araddr),
    .arlen(arlen),
    .arsize(arsize),
    .arburst(arburst),
    .arlock(arlock),
    .arvalid(arvalid),
    .arapcmd(arapcmd),
    .arqos(arqos),
    .arready(arready),
    .arprot (arprot ),
    .arcache(arcache),
    .rid(rid),
    .rdata(rdata),
    .rlast(rlast),
    .rvalid(rvalid),
    .rready(rready),
    .rresp(rresp),
    .wr_fifo_rd_data_test(wr_fifo_rd_data_test)
  );
//===================================================================================== 
//
//===================================================================================== 

// A freeze request locks the current, already complete read bank immediately.
// The lock is released only at a display-frame start.  Normally release waits
// for wr_idle, but a bounded frame-start timeout prevents a stuck write-side
// status from holding the replay image forever.  Both paths stay on a frame
// boundary, so the read address cannot jump banks inside a visible frame.
assign pos_rd_start_axi = rd_start_axi_d0 & ~rd_start_axi_d1;

always @( posedge axi_clk or negedge axi_clk_rst_n )
begin
    if( !axi_clk_rst_n ) begin
        freeze_rd_lock  <= 1'b0;
        freeze_rd_addr  <= {AXI_ADDR_WIDTH{1'b0}};
        freeze_rel_wait <= 3'd0;
        rd_start_axi_d0 <= 1'b0;
        rd_start_axi_d1 <= 1'b0;
    end else begin
        rd_start_axi_d0 <= rd_start;
        rd_start_axi_d1 <= rd_start_axi_d0;

        if( freeze ) begin
            if( !freeze_rd_lock ) begin
                freeze_rd_addr <= rd_start_addr_bank;
            end
            freeze_rel_wait <= 3'd0;
            freeze_rd_lock  <= 1'b1;
        end else if( freeze_rd_lock && pos_rd_start_axi ) begin
            // Deassert exactly at a display frame boundary.  ddr_rd_buffer has
            // already sampled the held address for this frame, so the read data
            // cannot jump banks partway through a visible frame.  If wr_idle
            // never asserts, release after seven frame starts as a fail-safe.
            if( wr_idle || (freeze_rel_wait == 3'd6) ) begin
                freeze_rd_lock  <= 1'b0;
                freeze_rel_wait <= 3'd0;
            end else if( freeze_rel_wait != 3'd7 ) begin
                freeze_rel_wait <= freeze_rel_wait + 1'b1;
            end
        end
    end
end

assign rd_start_addr = freeze_rd_lock ? freeze_rd_addr : rd_start_addr_bank;
  


bank_switch #(
		.FB_NUM					( FB_NUM),
		.MAX_VID_WIDTH 	( MAX_VID_WIDTH),
		.MAX_VID_HIGHT 	( MAX_VID_HIGHT),
		.START_ADDR			(	START_ADDR 	),
		.VID_DATA_WIDTH	(I_VID_WIDTH	),
		.AXI_DATA_WIDTH (AXI_DATA_WIDTH )
)
  u_bank_sw
(
/*i*/.ddr_clk		(axi_clk	),
/*i*/.rst_n			(axi_clk_rst_n),
              	
/*i*/.wr_sw			(wr_sw),
/*i*/.rd_sw			(rd_sw),
// Keep the bank selector frozen for the same frame-aligned window as the
// address shadow above.  The writer may still drain its independent bank.
/*i*/.freeze		(freeze_rd_lock),
                
/*o*/.wr_bank		(wr_bank	),
/*o*/.rd_bank		(rd_bank	),
/*o*/.rd_sw_ack	(rd_sw_ack),
/*o*/.wr_sw_ack (wr_sw_ack),
/*o*/.rd_start_addr(rd_start_addr_bank),
/*o*/.wr_start_addr(wr_start_addr)
);



endmodule
