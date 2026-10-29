`timescale 1ns/1ps

//`include "ddr3_controller.vh"


module example_top 
(
	////////////////////////////////////////////////////////////////
	//	External Clock & Reset
	//input 			nrst, 			//	Button K2
	input 			clk_24m,			//	24MHz Crystal
	input 			clk_25m,			//	25MHz Crystal 
	
	
	////////////////////////////////////////////////////////////////
	//	System Clock
	output 			sys_pll_rstn_o, 		
	
	input 			clk_sys,			//	Sys PLL 96MHz 
	input 			clk_pixel,			//	Sys PLL 74.25MHz
	input 			clk_pixel_2x,		//	Sys PLL 148.5MHz
	input 			clk_pixel_10x,		//	Sys PLL 742.5MHz
	
	input 			sys_pll_lock,		//	Sys PLL Lock
	
	////////////////////////////////////////////////////////////////
	//	MIPI-DSI Clock & Reset
	output 			dsi_pll_rstn_o,
	
	input 			dsi_refclk_i,		//	48MHz Reference Clock (for DSI PLL)
	input 			dsi_byteclk_i,		//	DSI Byte Clock (1X)
	input 			dsi_serclk_i,		//	DSI Serial Clock (4X 45)
	input 			dsi_txcclk_i,		//	DSI Serial Clock (4X 135)
	
	input 			dsi_pll_lock,
	
	////////////////////////////////////////////////////////////////
	//	DDR Clock
	output 			ddr_pll_rstn_o, 
	
	input 			tdqss_clk,			
	input 			core_clk,			//	DDR PLL 200MHz
	input 			tac_clk,			
	input 			twd_clk,			
	
	input 			ddr_pll_lock,		//	DDR PLL Lock
	
	////////////////////////////////////////////////////////////////
	//	DDR PLL Phase Shift Interface
	output 	[2:0] 	shift,
	output 	[4:0] 	shift_sel,
	output 			shift_ena,
	
	
	
	////////////////////////////////////////////////////////////////
	//	LVDS Clock
	output 			lvds_pll_rstn_o, 
	
	input 			clk_lvds_1x, 
	input 			clk_lvds_7x, 
	input 			clk_27m, 			//	RGB 1X Clock (16MHz)
	input 			clk_54m, 			//	RGB 2X Clock (32MHz, for export control)
	
	input 			lvds_pll_lock, 
	
	
	
	////////////////////////////////////////////////////////////////
	//	DDR Interface Ports
	output 	[15:0] 	addr,
	output 	[2:0] 	ba,
	output 			we,
	output 			reset,
	output 			ras,
	output 			cas,
	output 			odt,
	output 			cke,
	output 			cs,
	
	//	DQ I/O
	input 	[15:0] 	i_dq_hi,
	input 	[15:0] 	i_dq_lo,
	
	output 	[15:0] 	o_dq_hi,
	output 	[15:0] 	o_dq_lo,
	output 	[15:0] 	o_dq_oe,
	
	//	DM O
	output 	[1:0] 	o_dm_hi,
	output 	[1:0] 	o_dm_lo,
	
	//	DQS I/O
	input 	[1:0] 	i_dqs_hi,
	input 	[1:0] 	i_dqs_lo,
	
	input 	[1:0] 	i_dqs_n_hi,
	input 	[1:0] 	i_dqs_n_lo,
	
	output 	[1:0] 	o_dqs_hi,
	output 	[1:0] 	o_dqs_lo,
	
	output 	[1:0] 	o_dqs_n_hi,
	output 	[1:0] 	o_dqs_n_lo,
	
	output 	[1:0] 	o_dqs_oe,
	output 	[1:0] 	o_dqs_n_oe,
	
	//	CK
	output 			clk_p_hi, 
	output 			clk_p_lo, 
	output 			clk_n_hi, 
	output 			clk_n_lo, 
	
	
	
	////////////////////////////////////////////////////////////////
	//	MIPI-CSI Ctl / I2C
	output 			csi_ctl0_o,
	output 			csi_ctl0_oe,
	input 			csi_ctl0_i,
	
	output 			csi_ctl1_o,
	output 			csi_ctl1_oe,
	input 			csi_ctl1_i,
	
	output 			csi_scl_o,
	output 			csi_scl_oe,
	input 			csi_scl_i,
	
	output 			csi_sda_o,
	output 			csi_sda_oe,
	input 			csi_sda_i,
	
	//	MIPI-CSI RXC 
	input 			csi_rxc_lp_p_i,
	input 			csi_rxc_lp_n_i,
	output 			csi_rxc_hs_en_o,
	output 			csi_rxc_hs_term_en_o,
	input 			csi_rxc_i,
	
	//	MIPI-CSI RXD0
	output 			csi_rxd0_rst_o,
	output 			csi_rxd0_hs_en_o,
	output 			csi_rxd0_hs_term_en_o,
	
	input 			csi_rxd0_lp_p_i,
	input 			csi_rxd0_lp_n_i,
	input 	[7:0] 	csi_rxd0_hs_i,
	
	//	MIPI-CSI RXD1
	output 			csi_rxd1_rst_o,
	output 			csi_rxd1_hs_en_o,
	output 			csi_rxd1_hs_term_en_o,
	
	input 			csi_rxd1_lp_n_i,
	input 			csi_rxd1_lp_p_i,
	input 	[7:0] 	csi_rxd1_hs_i,
	
	//	MIPI-CSI RXD2
	output 			csi_rxd2_rst_o,
	output 			csi_rxd2_hs_en_o,
	output 			csi_rxd2_hs_term_en_o,
	
	input 			csi_rxd2_lp_p_i,
	input 			csi_rxd2_lp_n_i,
	input 	[7:0] 	csi_rxd2_hs_i,
	
	//	MIPI-CSI RXD3
	output 			csi_rxd3_rst_o,
	output 			csi_rxd3_hs_en_o,
	output 			csi_rxd3_hs_term_en_o,
	
	input 			csi_rxd3_lp_p_i,
	input 			csi_rxd3_lp_n_i,
	input 	[7:0] 	csi_rxd3_hs_i,
	
	//output 			csi_rxd0_fifo_rd_o, 
	//input 			csi_rxd0_fifo_empty_i, 
	//output 			csi_rxd1_fifo_rd_o, 
	//input 			csi_rxd1_fifo_empty_i, 
	//output 			csi_rxd2_fifo_rd_o, 
	//input 			csi_rxd2_fifo_empty_i, 
	//output 			csi_rxd3_fifo_rd_o, 
	//input 			csi_rxd3_fifo_empty_i, 
	
	
	
	////////////////////////////////////////////////////////////////
	//	DSI PWM & Reset Control 
	output 			dsi_pwm_o,			//	MIPI-DSI LCD PWM
	output 			dsi_resetn_o,		//	MIPI-DSI LCD Reset
	
	//	MIPI-DSI TXC / TXD
	output 			dsi_txc_rst_o,
	output 			dsi_txc_lp_p_oe,
	output 			dsi_txc_lp_p_o,
	output 			dsi_txc_lp_n_oe,
	output 			dsi_txc_lp_n_o,
	output 			dsi_txc_hs_oe,
	output 	[7:0] 	dsi_txc_hs_o,
	
	output 			dsi_txd0_rst_o,
	output 			dsi_txd0_hs_oe,
	output 	[7:0] 	dsi_txd0_hs_o,
	output 			dsi_txd0_lp_p_oe,
	output 			dsi_txd0_lp_p_o,
	output 			dsi_txd0_lp_n_oe,
	output 			dsi_txd0_lp_n_o,
	
	output 			dsi_txd1_rst_o,
	output 			dsi_txd1_lp_p_oe,
	output 			dsi_txd1_lp_p_o,
	output 			dsi_txd1_lp_n_oe,
	output 			dsi_txd1_lp_n_o,
	output 			dsi_txd1_hs_oe,
	output 	[7:0] 	dsi_txd1_hs_o,
	
	output 			dsi_txd2_rst_o,
	output 			dsi_txd2_lp_p_oe,
	output 			dsi_txd2_lp_p_o,
	output 			dsi_txd2_lp_n_oe,
	output 			dsi_txd2_lp_n_o,
	output 			dsi_txd2_hs_oe,
	output 	[7:0] 	dsi_txd2_hs_o,
	
	output 			dsi_txd3_rst_o,
	output 			dsi_txd3_lp_p_oe,
	output 			dsi_txd3_lp_p_o,
	output 			dsi_txd3_lp_n_oe,
	output 			dsi_txd3_lp_n_o,
	output 			dsi_txd3_hs_oe,
	output 	[7:0] 	dsi_txd3_hs_o,
	
	input 			dsi_txd0_lp_p_i,
	input 			dsi_txd0_lp_n_i,
	input 			dsi_txd1_lp_p_i,
	input 			dsi_txd1_lp_n_i,
	input 			dsi_txd2_lp_p_i,
	input 			dsi_txd2_lp_n_i,
	input 			dsi_txd3_lp_p_i,
	input 			dsi_txd3_lp_n_i,
	
	
	////////////////////////////////////////////////////////////////
	//	User Buttons —— Canny 阈值现场调节
	//	1.8 V LVCMOS，外部 10k 上拉、按下为 0；K3 = CRESET_N 不可用。
	//	★ 2026-10-16：GPIOL_P_02 / GPIOL_N_02 原分给 lcd_de_o / lcd_pclk_o，但这两个
	//	  LCD 端口在本设计里从未被驱动（lcd_pclk_o 的端口声明本身就是注释掉的），
	//	  而显示走 HDMI，所以把这两个脚改分给按键（Ti60_Demo.peri.xml 同步改）。
	input 		 	key1_i,				//	K1 = CSI = GPIOL_P_02 -> Canny 平滑模式循环
	input 		 	key2_i,				//	K2 = CSO = GPIOL_N_02 -> Canny 阈值循环（步长 1）


	////////////////////////////////////////////////////////////////
	//	UART Interface
	input 		 	uart_rx_i,			//	Support 460800-8-N-1. 
	output 		 	uart_tx_o, 
	
	
	output 	[5:0] 	led_o,			//	
	
	////////////////////////////////////////////////////////////////
	//	GM6020 CAN（必须外接 3.3 V CAN 收发器，如 SN65HVD230）
	//	  can_tx_o -> 球 A9 = GPIOT_N_11 = User_IO34 = 底板 P4 第 39 脚
	//	  can_rx_i <- 球 B9 = GPIOT_P_11 = User_IO35 = 底板 P4 第 40 脚
	//	  bank 2B 为 1.8 V LVCMOS，经底板 TXS0108E 转 3.3 V 出排针
	output 			can_tx_o,
	input  			can_rx_i,
	
	////////////////////////////////////////////////////////////////
	//	CMOS Sensor
	output 			cmos_sclk,
	input 			cmos_sdat_IN,
	output 			cmos_sdat_OUT,
	output 			cmos_sdat_OE,
	
	//	CMOS Interface
	input 			cmos_pclk,
	input 			cmos_vsync,
	input 			cmos_href,
	input 	[7:0] 	cmos_data,
	// input 			cmos_ctl1,
	// output 			cmos_ctl2,
	// output 			cmos_ctl3,
	
	
	////////////////////////////////////////////////////////////////
	//	HDMI Interface
	output 			hdmi_txc_oe,
	output 			hdmi_txd0_oe,
	output 			hdmi_txd1_oe,
	output 			hdmi_txd2_oe,
	
	output 			hdmi_txc_rst_o,
	output 			hdmi_txd0_rst_o,
	output 			hdmi_txd1_rst_o,
	output 			hdmi_txd2_rst_o,
	
	output 	[9:0] 	hdmi_txc_o,
	output 	[9:0] 	hdmi_txd0_o,
	output 	[9:0] 	hdmi_txd1_o,
	output 	[9:0] 	hdmi_txd2_o,
	
	
	////////////////////////////////////////////////////////////////
	//	LVDS Interface
	output 			lvds_txc_oe,
	output 	[6:0] 	lvds_txc_o,
	output 			lvds_txc_rst_o,
	
	output 			lvds_txd0_oe,
	output 	[6:0] 	lvds_txd0_o,
	output 			lvds_txd0_rst_o,
	
	output 			lvds_txd1_oe,
	output 	[6:0] 	lvds_txd1_o,
	output 			lvds_txd1_rst_o,
	
	output 			lvds_txd2_oe,
	output 	[6:0] 	lvds_txd2_o,
	output 			lvds_txd2_rst_o,
	
	output 			lvds_txd3_oe,
	output 	[6:0] 	lvds_txd3_o,
	output 			lvds_txd3_rst_o,
	
	
	////////////////////////////////////////////////////////////////
	//	RGB LCD 5Inch 800x480
	output 			lcd_tp_sda_o,		//	TP SDA
	output 			lcd_tp_sda_oe,
	input 			lcd_tp_sda_i,
	
	output 			lcd_tp_scl_o,		//	TP SCL
	output 			lcd_tp_scl_oe,
	input 			lcd_tp_scl_i,
	
	output 			lcd_tp_int_o,		//	TP INT
	output 			lcd_tp_int_oe,
	input 			lcd_tp_int_i,
	
	output 			lcd_tp_rst_o,		//	TP RST
	
	output 			lcd_pwm_o,			//	Backlight
	output 			lcd_blen_o,
	
	//output 			lcd_pclk_o,			//	PCLK & SCK Mux
	output 			lcd_vs_o,			//	VS & SSN Mux. Fixed to 1. Use DE-Only mode. 
	output 			lcd_hs_o,			//	HS. Fixed to 1. Use DE-Only mode. 
	output 			lcd_de_o,			//	DE. 

	output 	[7:0] 	lcd_b7_0_o,			//	B7:B0. 
	output 	[7:0] 	lcd_g7_0_o,			//	G7:G0. Must output 8'hFF when access SPI. 
	output 	[7:0] 	lcd_r7_0_o,			//	R7:R0. 
	
	output 	[7:0] 	lcd_b7_0_oe,		//	B7:B0. 
	output 	[7:0] 	lcd_g7_0_oe,		//	G7:G0. Must output 8'hFF when access SPI. 
	output 	[7:0] 	lcd_r7_0_oe,		//	R7:R0. 

	input 	[7:0] 	lcd_b7_0_i,			//	B7:B0. 
	input 	[7:0] 	lcd_g7_0_i,			//	G7:G0. Must output 8'hFF when access SPI. 
	input 	[7:0] 	lcd_r7_0_i,			//	R7:R0. 
	
	//	SPI Pins
	output 			spi_sck_o, 
	output 			spi_ssn_o 			
);
	
	wire 			csi_rxd0_fifo_rd_o; 
	wire 			csi_rxd0_fifo_empty_i = 0;  
	wire 			csi_rxd1_fifo_rd_o;  
	wire 			csi_rxd1_fifo_empty_i = 0;  
	wire 			csi_rxd2_fifo_rd_o;  
	wire 			csi_rxd2_fifo_empty_i = 0;  
	wire 			csi_rxd3_fifo_rd_o;  
	wire 			csi_rxd3_fifo_empty_i = 0;  
	
	
	parameter 	SIM_DATA 	= 0; 
	
	//	Hardware Configuration
	assign clk_p_hi = 1'b0;	//	DDR3 Clock requires 180 degree shifted. 
	assign clk_p_lo = 1'b1;
	assign clk_n_hi = 1'b1;
	assign clk_n_lo = 1'b0; 
	
	//	System Clock Tree Control
	assign sys_pll_rstn_o = 1'b1; 	//	nrst; 	//	Reset whole system when nrst (K2) is pressed. 
	
	assign dsi_pll_rstn_o = sys_pll_lock; 
	assign ddr_pll_rstn_o = sys_pll_lock; 
	assign lvds_pll_rstn_o = sys_pll_lock; 
	
	wire 			w_pll_lock = sys_pll_lock && dsi_pll_lock && ddr_pll_lock && lvds_pll_lock; 
	
	//	Synchronize System Resets. 
	reg 			rstn_sys = 0, rstn_pixel = 0; 
	wire 			rst_sys = ~rstn_sys, rst_pixel = ~rstn_pixel; 
	
	reg 			rstn_dsi_refclk = 0, rstn_dsi_byteclk = 0; 
	wire 			rst_dsi_refclk = ~rstn_dsi_refclk, rst_dsi_byteclk = ~rstn_dsi_byteclk; 
	
	reg 			rstn_lvds_1x = 0; 
	wire 			rst_lvds_1x = ~rstn_lvds_1x; 
	
	reg 			rstn_27m = 0, rstn_54m = 0; 
	wire 			rst_27m = ~rstn_27m, rst_54m = ~rstn_54m; 
	
	//	Clock Gen
	always @(posedge clk_27m or negedge w_pll_lock) begin if(~w_pll_lock) rstn_27m <= 0; else rstn_27m <= 1; end
	always @(posedge clk_54m or negedge w_pll_lock) begin if(~w_pll_lock) rstn_54m <= 0; else rstn_54m <= 1; end
	always @(posedge clk_sys or negedge w_pll_lock) begin if(~w_pll_lock) rstn_sys <= 0; else rstn_sys <= 1; end
	always @(posedge clk_pixel or negedge w_pll_lock) begin if(~w_pll_lock) rstn_pixel <= 0; else rstn_pixel <= 1; end
	always @(posedge dsi_refclk_i or negedge w_pll_lock) begin if(~w_pll_lock) rstn_dsi_refclk <= 0; else rstn_dsi_refclk <= 1; end
	always @(posedge dsi_byteclk_i or negedge w_pll_lock) begin if(~w_pll_lock) rstn_dsi_byteclk <= 0; else rstn_dsi_byteclk <= 1; end
	always @(posedge clk_lvds_1x or negedge w_pll_lock) begin if(~w_pll_lock) rstn_lvds_1x <= 0; else rstn_lvds_1x <= 1; end
	
	
	localparam 	CLOCK_MAIN 	= 96000000; 	//	System clock using 96MHz. 
	
	
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	Flash Burner Control
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	wire 			w_ustick, w_mstick; 
	
	wire  [7:0] 	w_dev_index_o;  
	wire  [7:0] 	w_dev_cmd_o;  
	wire  [31:0] 	w_dev_wdata_o;  
	wire  		w_dev_wvalid_o;  
	wire  		w_dev_rvalid_o;  
	wire 	[31:0] 	w_dev_rdata_i;  
	
	wire 			w_spi_ssn_o, w_spi_sck_o; 
	wire 	[3:0] 	w_spi_data_o, w_spi_data_oe; 
	wire 	[3:0] 	w_spi_data_i; 
	
	//	Flash Control
	reg 			r_flash_en = 0; 		//	0x00:0x00 Enable Flash
	
	always @(posedge clk_sys) begin
		r_flash_en <= (w_dev_wvalid_o && (w_dev_index_o == 8'h00) && (w_dev_cmd_o == 8'h00)) ? w_dev_wdata_o : r_flash_en; 
	end
	
	
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	LCD Data Mux
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	wire 	[7:0] 	w_lcd_b_o, w_lcd_g_o, w_lcd_r_o; 
	
	assign lcd_b7_0_o = r_flash_en ? {4'b0, w_spi_data_o[3:2], 2'b0} : w_lcd_b_o; 
	assign lcd_g7_0_o = r_flash_en ? {6'h0, w_spi_data_o[1:0]} : w_lcd_g_o; 
	assign lcd_r7_0_o = r_flash_en ? {8'h00} : w_lcd_r_o; 
	
	assign lcd_b7_0_oe = r_flash_en ? {4'b0, w_spi_data_oe[3:2], 2'b0} : 8'hFF; 
	assign lcd_g7_0_oe = r_flash_en ? {6'h0, w_spi_data_oe[1:0]} : 8'hFF; 
	assign lcd_r7_0_oe = r_flash_en ? {8'h00} : 8'hFF; 
	
	assign spi_sck_o = w_spi_sck_o; 
	assign spi_ssn_o = w_spi_ssn_o; 
	assign w_spi_data_i = {lcd_b7_0_i[3:2], lcd_g7_0_i[1:0]}; 
	
	
	
	
	
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	DDR3 Controller
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	wire			w_ddr3_ui_clk = clk_sys;
	wire			w_ddr3_ui_rst = rst_sys;
	wire			w_ddr3_ui_areset = rst_sys;
	wire			w_ddr3_ui_aresetn = rstn_sys;
	

	//	General AXI Interface 
	wire	[3:0] 	w_ddr3_awid;
	wire	[31:0]	w_ddr3_awaddr;
	wire	[7:0]		w_ddr3_awlen;
	wire			w_ddr3_awvalid;
	wire			w_ddr3_awready;
	
	wire 	[3:0]  	w_ddr3_wid;
	wire 	[127:0] 	w_ddr3_wdata;
	wire 	[15:0]	w_ddr3_wstrb;
	wire			w_ddr3_wlast;
	wire			w_ddr3_wvalid;
	wire			w_ddr3_wready;
	
	wire 	[3:0] 	w_ddr3_bid;
	wire 	[1:0] 	w_ddr3_bresp;
	wire			w_ddr3_bvalid;
	wire			w_ddr3_bready;
	
	wire	[3:0] 	w_ddr3_arid;
	wire	[31:0]	w_ddr3_araddr;
	wire	[7:0]		w_ddr3_arlen;
	wire			w_ddr3_arvalid;
	wire			w_ddr3_arready;
	
	wire 	[3:0] 	w_ddr3_rid;
	wire 	[127:0] 	w_ddr3_rdata;
	wire			w_ddr3_rlast;
	wire			w_ddr3_rvalid;
	wire			w_ddr3_rready;
	wire 	[1:0] 	w_ddr3_rresp;
	
	
	//	AXI Interface Request
	wire 	[3:0] 	w_ddr3_aid;
	wire 	[31:0] 	w_ddr3_aaddr;
	wire 	[7:0]  	w_ddr3_alen;
	wire 	[2:0]  	w_ddr3_asize;
	wire 	[1:0]  	w_ddr3_aburst;
	wire 	[1:0]  	w_ddr3_alock;
	wire			w_ddr3_avalid;
	wire			w_ddr3_aready;
	wire			w_ddr3_atype;
	
	wire 			w_ddr3_cal_done, w_ddr3_cal_pass; 
	
	//	Do not issue DDR read / write when ~cal_done. 
	reg 			r_ddr_unlock = 0; 
	always @(posedge w_ddr3_ui_clk or negedge w_ddr3_ui_aresetn) begin
		if(~w_ddr3_ui_aresetn)
			r_ddr_unlock <= 0; 
		else
			r_ddr_unlock <= w_ddr3_cal_done; 
	end
	
	DdrCtrl ddr3_ctl_axi (	
		.core_clk		(core_clk),
		.tac_clk		(tac_clk),
		.twd_clk		(twd_clk),	
		.tdqss_clk		(tdqss_clk),
		
		.reset		(reset),
		.cs			(cs),
		.ras			(ras),
		.cas			(cas),
		.we			(we),
		.cke			(cke),    
		.addr			(addr),
		.ba			(ba),
		.odt			(odt),
		
		.o_dm_hi		(o_dm_hi),
		.o_dm_lo		(o_dm_lo),
		
		.i_dq_hi		(i_dq_hi),
		.i_dq_lo		(i_dq_lo),
		.o_dq_hi		(o_dq_hi),
		.o_dq_lo		(o_dq_lo),
		.o_dq_oe		(o_dq_oe),
		
		.i_dqs_hi		(i_dqs_hi),
		.i_dqs_lo		(i_dqs_lo),
		.i_dqs_n_hi		(i_dqs_n_hi),
		.i_dqs_n_lo		(i_dqs_n_lo),
		.o_dqs_hi		(o_dqs_hi),
		.o_dqs_lo		(o_dqs_lo),
		.o_dqs_n_hi		(o_dqs_n_hi),
		.o_dqs_n_lo		(o_dqs_n_lo),
		.o_dqs_oe		(o_dqs_oe),
		.o_dqs_n_oe		(o_dqs_n_oe),
		
		.clk			(w_ddr3_ui_clk),
		.reset_n		(w_ddr3_ui_aresetn),
		
		.axi_avalid		(w_ddr3_avalid && r_ddr_unlock),	//	Enable command only when unlocked. 
		.axi_aready		(w_ddr3_aready),
		.axi_aaddr		(w_ddr3_aaddr),
		.axi_aid		(w_ddr3_aid),
		.axi_alen		(w_ddr3_alen),
		.axi_asize		(w_ddr3_asize),
		.axi_aburst		(w_ddr3_aburst),
		.axi_alock		(w_ddr3_alock),
		.axi_atype		(w_ddr3_atype),
		
		.axi_wid		(w_ddr3_wid),
		.axi_wvalid		(w_ddr3_wvalid),
		.axi_wready		(w_ddr3_wready),
		.axi_wdata		(w_ddr3_wdata),
		.axi_wstrb		(w_ddr3_wstrb),
		.axi_wlast		(w_ddr3_wlast),
		
		.axi_bvalid		(w_ddr3_bvalid),
		.axi_bready		(w_ddr3_bready),
		.axi_bid		(w_ddr3_bid),
		.axi_bresp		(w_ddr3_bresp),
		
		.axi_rvalid		(w_ddr3_rvalid),
		.axi_rready		(w_ddr3_rready),
		.axi_rdata		(w_ddr3_rdata),
		.axi_rid		(w_ddr3_rid),
		.axi_rresp		(w_ddr3_rresp),
		.axi_rlast		(w_ddr3_rlast),
		
		.shift		(shift),
		.shift_sel		(),
		.shift_ena		(shift_ena),
		
		.cal_ena		(1'b1),
		.cal_done		(w_ddr3_cal_done),
		.cal_pass		(w_ddr3_cal_pass)
	);
	
	assign w_ddr3_bready = 1'b1; 
	assign shift_sel = 5'b00100; 		//	ddr_tac_clk always use PLLOUT[2]. 
	
	
	AXI4_AWARMux #(.AID_LEN(4), .AADDR_LEN(32)) axi4_awar_mux (
		.aclk_i			(w_ddr3_ui_clk), 
		.arst_i			(w_ddr3_ui_rst), 
		
		.awid_i			(w_ddr3_awid),
		.awaddr_i			(w_ddr3_awaddr),
		.awlen_i			(w_ddr3_awlen),
		.awvalid_i			(w_ddr3_awvalid),
		.awready_o			(w_ddr3_awready),
		
		.arid_i			(w_ddr3_arid),
		.araddr_i			(w_ddr3_araddr),
		.arlen_i			(w_ddr3_arlen),
		.arvalid_i			(w_ddr3_arvalid),
		.arready_o			(w_ddr3_arready),
		
		.aid_o			(w_ddr3_aid),
		.aaddr_o			(w_ddr3_aaddr),
		.alen_o			(w_ddr3_alen),
		.atype_o			(w_ddr3_atype),
		.avalid_o			(w_ddr3_avalid),
		.aready_i			(w_ddr3_aready && r_ddr_unlock)
	);
	
	assign w_ddr3_asize = 4; 		//	Fixed 128 bits (16 bytes, size = 4)
	assign w_ddr3_aburst = 1; 
	assign w_ddr3_alock = 0; 
	
	//assign led_o[1:0] = {w_ddr3_cal_pass, w_ddr3_cal_done}; 
	
	
	
	
	
	
	
	
	
	// ////////////////////////////////////////////////////////////////
	// //	I2C Config (SC130GS)
	
	// //  i2c timing controller module of 16Bit
	// wire            [ 7:0]          sc130_i2c_config_index;
	// wire            [23:0]          sc130_i2c_config_data;
	// wire            [ 7:0]          sc130_i2c_config_size;
	// wire                            sc130_i2c_config_done;
	
	// i2c_timing_ctrl_16bit
	// #(
	//     .CLK_FREQ           (CLOCK_MAIN),                              //  100 MHz
	//     .I2C_FREQ           (50_000    )                               //  10 KHz(<= 400KHz)
	// ) u_i2c_timing_ctrl_16bit (
	//     //  global clock
	//     .clk                (clk_sys                 ),                          //  96MHz
	//     .rst_n              (rstn_sys                ),                          //  system reset

	//     //  i2c interface
	//     .i2c_sclk           (csi_scl_o               ),                          //  i2c clock
	//     .i2c_sdat_IN        (csi_sda_i               ),
	//     .i2c_sdat_OUT       (csi_sda_o               ),
	//     .i2c_sdat_OE        (csi_sda_oe              ),

	//     //  i2c config data
	//     .i2c_config_index   (sc130_i2c_config_index        ),                          //  i2c config reg index, read 2 reg and write xx reg
	//     .i2c_config_data    ({8'h60, sc130_i2c_config_data}),                     //  i2c config data
	//     .i2c_config_size    (sc130_i2c_config_size         ),                          //  i2c config data counte
	//     .i2c_config_done    (sc130_i2c_config_done         )                          //  i2c config timing complete
	// );
	// assign csi_scl_oe = 1; 

	// //  I2C Configure Data of SC130GS
	// I2C_SC130GS_12801024_4Lanes_Config u_I2C_SC130GS_12801024_4Lanes_Config 
	// (
	//     .LUT_INDEX  (sc130_i2c_config_index   ),
	//     .LUT_DATA   (sc130_i2c_config_data    ),
	//     .LUT_SIZE   (sc130_i2c_config_size    )
	// );
	
//wire  i2c_config_done = 1'b1;
//----------------------------------------------
//i2c timing controller module of 16Bit
	wire	[ 7:0]          i2c_config_index;
	wire	[23:0]          i2c_config_data;
	wire	[ 7:0]          i2c_config_size;
	wire	                i2c_config_done;
	wire	[23:0]          sensor_config_data;                     //  {addr,data}
	wire	                sensor_config_data_valid;
	i2c_timing_ctrl_reg16_dat8_wronly
	#(
		.CLK_FREQ       (CLOCK_MAIN ),                                      //  120 MHz
		.I2C_FREQ       (100_000    )                                       //  100 KHz(<= 400KHz)
	)
	u_i2c_timing_ctrl_reg16_dat8_wronly
	(
		//  global clock
		.clk                        (clk_sys                    ),          //  120MHz
		//.rst_n                      (0                 ),          //  system reset
		//.rst_n                      (1                 ),          //  system reset
		.rst_n                      (rstn_sys                 ),          //  system reset
		//.rst_n                      (rst_sys                 ),          //  system reset
	
		//  i2c interface
		.i2c_sclk                   (cmos_sclk                  ),          //  i2c clock
		.i2c_sdat_i                 (cmos_sdat_IN              ),
		.i2c_sdat_o                 (cmos_sdat_OUT              ),
		.i2c_sdat_oe                (cmos_sdat_OE               ),
	
		//  i2c config data
		.i2c_config_index           (i2c_config_index           ),          //  i2c config reg index, read 2 reg and write xx reg
		.i2c_config_data            ({8'h60, i2c_config_data}   ),          //  i2c config data
		.i2c_config_size            (i2c_config_size            ),          //  i2c config data counte
		.i2c_config_done            (i2c_config_done            )           //  i2c config timing complete
	);
	
	
	//----------------------------------
	//I2C Configure Data of SC1336
	I2C_SC1336_1280720_Config    u_I2C_SC1336_1280720_Config   //Auto/Manual Exposure
	(
		.LUT_INDEX  (i2c_config_index   ),
		.LUT_DATA   (i2c_config_data    ),
		.LUT_SIZE   (i2c_config_size    )
	);
	// ////////////////////////////////////////////////////////////////
	// //	I2C Config (AR0135)
	
	// //  i2c timing controller module of 16Bit
	// wire            [ 7:0]          ar0135_i2c_config_index;
	// wire            [31:0]          ar0135_i2c_config_data;
	// wire            [ 7:0]          ar0135_i2c_config_size;
	// wire                            ar0135_i2c_config_done;
	// wire            [15:0]          ar0135_i2c_rdata;                              //  i2c register data

	// i2c_timing_ctrl_reg16_dat16
	// #(
	//     .SIMU_EN            (0              ),
	//     .I2C_READ_EN        (1              ),                              //  Enable I2C Register read
	//     .CLK_FREQ           (CLOCK_MAIN     ),                              //  120 MHz
	//     .I2C_FREQ           (50_000         )                               //  10 KHz(<= 400KHz)
	// )
	// u_i2c_timing_ctrl_16reg_16bit
	// (
	//     //global clock
	//     .clk                (clk_sys                 ),                          //  96MHz
	//     .rst_n              (rstn_sys                ),                          //  system reset

	//     //i2c interface
	//     .i2c_sclk           (cmos_sclk                  ),                  //  i2c clock
	// 	.i2c_sdat_o 	(cmos_sdat_OUT), 
	// 	.i2c_sdat_oe	(cmos_sdat_OE), 
	// 	.i2c_sdat_i 	(cmos_sdat_IN), 
		
	//     //i2c config data
	//     .i2c_config_index   (ar0135_i2c_config_index           ),                  //  i2c config reg index, read 2 reg and write xx reg
	//     .i2c_config_data    ({8'h20, ar0135_i2c_config_data}   ),                  //  i2c config data
	//     .i2c_config_size    (ar0135_i2c_config_size            ),                  //  i2c config data counte
	//     .i2c_config_done    (ar0135_i2c_config_done            ),                  //  i2c config timing complete
	//     .i2c_rdata          (ar0135_i2c_rdata                  )                   //  i2c register data while read i2c slave
	// );
	
	// //----------------------------------------------------------------------
	// //  I2C Configure Data of AR0135
	// I2C_AR0135_1280720_Config u_I2C_AR0135_1280720_Config
	// (
	//     .LUT_INDEX  (ar0135_i2c_config_index   ),
	//     .LUT_DATA   (ar0135_i2c_config_data    ),
	//     .LUT_SIZE   (ar0135_i2c_config_size    )
	// );
	
	
		
	//	CMOS Interface
	//input 			cmos_pclk,
	//input 			cmos_vsync,
	//input 			cmos_href,
	//input 	[7:0] 	cmos_data,
	//input 			cmos_ctl1,
	//output 			cmos_ctl2,
	//output 			cmos_ctl3,

	
	//	Output LED
	reg 	[3:0]		r_cmos_fv_o = 0; 
	reg 	[1:0] 	r_cmos_rx_vsync0_in = 0; 
	always @(posedge cmos_pclk) begin
		r_cmos_rx_vsync0_in <= {r_cmos_rx_vsync0_in, cmos_vsync}; 
		r_cmos_fv_o <= r_cmos_fv_o + ((r_cmos_rx_vsync0_in == 2'b01) ? 1 : 0); 
	end
	//-----------------------------------------------------------------
	//------------------------------------------------------------- PWM 跟随相关线网（前置声明，供上方 LED 使用）
	wire              pwm_motor;
	wire [3:0]        pwm_status;
	wire signed [9:0] pwm_ex_dbg;
	wire              pwm_found_dbg;
	wire [20:0]       pwm_duty_dbg;

	// GM6020 PWM 三角形跟随状态 + PLL lock + camera frame activity.
	//   led[0] 视觉域（clk_pixel）：本帧抓到三角形 —— 证明上游识别通了
	//   led[1] 电机域（clk_sys ）：跨时钟域后收到 found —— 与 led[0] 对比即可判断 CDC 是否断链
	//   led[2] 误差进入死区（画面中心附近，稳定）
	//   led[3] 脉宽顶到行程端点
	// 板载 LED 低电平点亮，所以取反。
    assign led_o[0]=~pwm_found_dbg;     // 视觉域 found
    assign led_o[1]=~pwm_status[0];     // 电机域 found（跨域后）
    assign led_o[2]=~pwm_status[1];     // 进死区
    assign led_o[3]=~pwm_status[3];     // 顶到端点
    assign led_o[4]=~w_pll_lock;
    assign led_o[5]=r_cmos_fv_o[3];

	
	
	
	
	
	
	
	
	////////////////////////////////////////////////////////////////
	//	System Control. Can be removed for public. 
	
	localparam 	CLK_FREQ 	= 96_000_000; 	//	clk_sys is 96MHz. 
	localparam 	BAUD_RATE 	= 460_800; 		//	Use 460800-8-N-1. 
	
	
	//	SFR I/O Interface
	wire 	[7:0] 	w_sfr_addr_o; 	//	SFR Address (0xFF00 ~ 0xFFFF). 00:Power; 40~5F:Stream0; 60~7F:Stream1. 
	wire 	[7:0] 	w_sfr_wdata_o; 	//	SFR Write Data. 
	wire 			w_sfr_we_o; 		//	SFR WE. 
	reg 	[7:0] 	w_sfr_rdata_i; 	//	Must be valid after sfr_rd_o. 
	wire 			w_sfr_rd_o; 		//	SFR RD. 
	
	
	//	System Control Registers
	reg 			r_dsi_tx_rstn = 0; 	//	DSI TX Reset
	reg 	[7:0] 	r_dsi_pwm = 64; 		//	[6:0]PWM, [7]Pol
	reg 			r_dsi_resetn_o = 0; 	//	DSI Panel Reset
	reg 			r_dsi_data_rstn = 0; 	//	DSI TX Reset
	
	reg 	[3:0] 	r_dsi_lp_p_ovr = 0; 	
	reg 	[3:0] 	r_dsi_lp_n_ovr = 0; 	
	
	
	
	//	AXI-Lite Interface Bridge
	localparam 	CSI_AXILITE_ID 	= 0; 				//	Select DSI_TX when r_axi_sel = DSI_AXILITE_ID. 
	localparam 	DSI_AXILITE_ID 	= 1; 				//	Select DSI_TX when r_axi_sel = DSI_AXILITE_ID. 
	
	reg 	[7:0] 	r_axi_addr = 8'h18; 		//	0xE0 (RW)
	reg 	[31:0] 	r_axi_wdata = 32'h0000000A; 	//	0xE1~0xE4 (RW)
	wire 	[31:0] 	w_axi_rdata; 			//	0xE5~0xE8 (RO)
	reg 	[0:0] 	r_axi_sel = 1; 			//	0xE9[7:2] (RW)
	reg 			r_axi_r1w0 = 0; 			//	0xE9[1] (RW)
	reg 			r_axi_req = 0; 			//	0xE9[0] (WO, Single Cycle)
	reg 			r_axi_req_o = 0; 			//	Delayed of r_axi_req. 
	
	
	//	Buffered AXI Read Data
	reg 	[31:0] 	r_axi_rdata = 0; 		//	Use state machine
	reg 			r_axi_idle = 0; 		//	AXI Idle 
	
	reg 	[3:0] 	rs_axilite = 0; 		//	AXI Access
	wire 	[3:0] 	ws_axilite_idle = 0; 		
	wire 	[3:0] 	ws_axilite_write = 1; 
	wire 	[3:0] 	ws_axilite_read = 2; 
	wire 	[3:0] 	ws_axilite_endread = 3; 
	
	reg 			r_axi_awvalid = 0, r_axi_wvalid = 0, r_axi_arvalid = 0; 
	wire 			w_axi_awready, w_axi_wready, w_axi_arready, w_axi_rvalid; 
	
	
	reg 	[3:0] 	rc_axi_init = 0; 

	always @(posedge clk_sys or posedge rst_sys) begin
		if(rst_sys) begin
			r_dsi_tx_rstn <= 0; 
			r_dsi_pwm <= 64; 
			r_axi_req <= 0; 
			r_dsi_resetn_o <= 0; 
			r_dsi_data_rstn <= 0; 
			
			rs_axilite <= 0; 
			r_axi_awvalid <= 0; 
			r_axi_wvalid <= 0;
			r_axi_arvalid <= 0; 
			
			r_dsi_lp_p_ovr <= 0; 
			r_dsi_lp_n_ovr <= 0; 
			
			rc_axi_init <= 0; 
			r_axi_idle <= 0; 
			
		end else begin
			r_dsi_tx_rstn <= 1; 
			r_dsi_resetn_o <= 1; 
			r_dsi_data_rstn <= 1; 
			
		end
	end
	
	assign dsi_resetn_o = r_dsi_resetn_o; 
	
	assign csi_ctl0_oe = 0; 
	assign csi_ctl1_oe = 0; 
	
	
	PWMLite dsi_pwm (		//	#(.ENABLE_TICK(0), .PWM_BITS(7)) 
		.clk_i			(clk_sys),
		.rst_i			(rst_sys),
		.pwm_i			(r_dsi_pwm[6:0]),
		.pol_i			(r_dsi_pwm[7]), 
		.pwm_o			(dsi_pwm_o)
	);






	////////////////////////////////////////////////////////////////
	//	MIPI CSI RX
	
	//	The CSI RXC shall not be inverted. Data can be inverted with swapped LP data and flipped HS data. 
	localparam 	CSI_RXD_INV 	= 4'b1111; 
	localparam 	CSI_DATA_WIDTH 	= 8; 			
	localparam 	CSI_STRB_WIDTH 	= CSI_DATA_WIDTH / 8; 

	
	////////////////////////////////////////////////////////////////
	//	MIPI-CSI Crop
	
	wire			XYCrop_frame_vsync; 
	wire			XYCrop_frame_href;
	wire			XYCrop_frame_de;
	wire	[63:0]	XYCrop_frame_Gray;

	Sensor_Image_XYCrop
	#(
		.IMAGE_HSIZE_SOURCE (1280 / CSI_STRB_WIDTH),
		.IMAGE_VSIZE_SOURCE (720	 ),
		.IMAGE_HSIZE_TARGET (1280 / CSI_STRB_WIDTH),
		.IMAGE_YSIZE_TARGET (720 	 ),
		.PIXEL_DATA_WIDTH	(CSI_DATA_WIDTH) 		//	32		 )
	)
	u_Sensor_Image_XYCrop
	(
		//	globel clock
		.clk			(cmos_pclk),			//	image pixel clock
		.rst_n		(rstn_sys),			//	system reset
		
		//CMOS Sensor interface
		.image_in_vsync (cmos_vsync		),			//H : Data Valid; L : Frame Sync(Set it by register)
		.image_in_href	(cmos_href		),			//H : Data vaild, L : Line Sync
		.image_in_de	(cmos_href		), 			//H : Data Enable, L : Line Sync
		.image_in_data	(cmos_data),			//8 bits cmos data input
		
		.image_out_vsync(XYCrop_frame_vsync ),			//H : Data Valid; L : Frame Sync(Set it by register)
		.image_out_href (XYCrop_frame_href	),			//H : Data vaild, L : Line Sync
		.image_out_de	(XYCrop_frame_de	), 			//H : Data Enable, L : Line Sync
		.image_out_data (XYCrop_frame_Gray	)			//8 bits cmos data input	
	);

	reg			r_XYCrop_frame_vsync = 0; 
	reg			r_XYCrop_frame_href = 0;
	reg			r_XYCrop_frame_de = 0;
	reg	[63:0]	r_XYCrop_frame_Gray = 0;
	
	always @(posedge cmos_pclk) begin
		r_XYCrop_frame_vsync <= XYCrop_frame_vsync; 
		r_XYCrop_frame_href <= XYCrop_frame_href;
		r_XYCrop_frame_de <= XYCrop_frame_de;
		r_XYCrop_frame_Gray <= XYCrop_frame_Gray;
	end
	
	//	Data Write Assignment
	wire			cmos_frame_vsync = r_XYCrop_frame_vsync;                     //  cmos frame data vsync valid signal
	wire			cmos_frame_href = r_XYCrop_frame_href && r_XYCrop_frame_de;	 //  cmos frame data href vaild  signal
	wire	[63:0]	cmos_frame_Gray = r_XYCrop_frame_Gray; 
	wire 			cmos_vsync_end;

	












	////////////////////////////////////////////////////////////////
	//	DDR R/W Control
	

	wire                            lcd_de;
	wire                            lcd_hs;      
	wire                            lcd_vs;
	wire 					  lcd_request; 
	wire            [7:0]           lcd_red, lcd_red2;
	wire            [7:0]           lcd_green, lcd_green2;
	wire            [7:0]           lcd_blue, lcd_blue2;
	wire            [15:0]          lcd_data;


	assign w_ddr3_awid = 0; 
	assign w_ddr3_wid = 0; 
	
	wire 			w_wframe_vsync; 
	wire 	[7:0] 	w_axi_tp; 
	
    wire [2:0] app_play_mode;
    wire [63:0] iir_tuning;
    wire app_tnr,app_status_req,app_status_valid;
    wire [3:0] app_age_a,app_age_b;
    wire [7:0] app_replay_div;
    wire [31:0] app_status_sys,app_status_pixel;
    frame_status_mailbox status_mailbox(
        .src_clk(clk_sys),.src_rst(rst_sys),.src_status(app_status_sys),
        .dst_clk(clk_pixel),.dst_rst(~rstn_pixel),.request_i(app_status_req),
        .valid_o(app_status_valid),.status_o(app_status_pixel));
    // Gate both sides of the DDR command handshake until calibration completes.
    // Otherwise the address bridge can acknowledge a command that never reached
    // the gated DDR input, leaving a master waiting forever for its response.
    wire ddr_app_reset=w_ddr3_ui_rst || !r_ddr_unlock;
    (* async_reg="true" *) reg [1:0] ddr_ready_pixel;
    always @(posedge clk_pixel or negedge rstn_pixel)begin
      if(!rstn_pixel)ddr_ready_pixel<=0;
      else ddr_ready_pixel<={ddr_ready_pixel[0],r_ddr_unlock};
    end
    wire temporal_pixel_reset=~rstn_pixel || !ddr_ready_pixel[1];
    wire [31:0] raw_awaddr,raw_araddr,hist_awaddr,hist_araddr,raw_status,hist_status;
    wire [7:0] raw_awlen,raw_arlen,hist_awlen,hist_arlen;
    wire [127:0] raw_wdata,hist_wdata;
    wire raw_awvalid,raw_awready,raw_wlast,raw_wvalid,raw_wready,raw_bvalid;
    wire raw_arvalid,raw_arready,raw_rvalid;
    wire hist_awvalid,hist_awready,hist_wlast,hist_wvalid,hist_wready,hist_bvalid;
    wire hist_arvalid,hist_arready,hist_rvalid;
    wire [15:0] raw_source_id;
    wire [31:0] history_pixel_status,strong_pixels,weak_pixels,bypass_pixels;
    wire history_req,history_valid;
    frame_status_mailbox history_mailbox(
      .src_clk(w_ddr3_ui_clk),.src_rst(w_ddr3_ui_rst),.src_status(hist_status),
      .dst_clk(clk_pixel),.dst_rst(~rstn_pixel),.request_i(history_req),
      .valid_o(history_valid),.status_o(history_pixel_status));
    (* async_reg="true" *) reg [15:0] source_meta,source_pixel;
    always @(posedge clk_pixel) begin source_meta<=raw_source_id;source_pixel<=source_meta;end
    wire [7:0] median_y,temporal_y;
    wire median_vs,median_de,temporal_vs,temporal_de;
    assign app_status_sys={raw_status[31:24]|hist_status[31:24],raw_status[23:0]};
    assign w_ddr3_wstrb=16'hffff;
    assign w_ddr3_arid=0;
    assign w_ddr3_rready=1'b1;
    temporal_ddr_history u_temporal_history(
      .sc(w_ddr3_ui_clk),.pc(clk_pixel),.rst(ddr_app_reset),.prst(temporal_pixel_reset),
      .frame_vs(median_vs),.pixel_de(median_de),.pixel_y(median_y),
      .source_id(source_pixel),.enable_i(app_tnr),.mode_i(app_play_mode),
      .config_i({r_awb_gain,b_awb_gain,iir_tuning}),
      .filtered_vs(temporal_vs),.filtered_de(temporal_de),.filtered_y(temporal_y),.status_o(hist_status),
      .strong_pixels_o(strong_pixels),.weak_pixels_o(weak_pixels),.bypass_pixels_o(bypass_pixels),
      .awaddr(hist_awaddr),.awlen(hist_awlen),.awvalid(hist_awvalid),.awready(hist_awready),
      .wdata(hist_wdata),.wlast(hist_wlast),.wvalid(hist_wvalid),.wready(hist_wready),
      .bvalid(hist_bvalid),.bresp(w_ddr3_bresp),
      .araddr(hist_araddr),.arlen(hist_arlen),.arvalid(hist_arvalid),.arready(hist_arready),
      .rdata(w_ddr3_rdata),.rlast(w_ddr3_rlast),.rvalid(hist_rvalid),.rresp(w_ddr3_rresp));
    temporal_axi_mux u_temporal_mux(
      .clk(w_ddr3_ui_clk),.rst(ddr_app_reset),
      .a_awaddr(raw_awaddr),.b_awaddr(hist_awaddr),
      .a_awlen(raw_awlen),.b_awlen(hist_awlen),
      .a_awvalid(raw_awvalid),.b_awvalid(hist_awvalid),
      .a_awready(raw_awready),.b_awready(hist_awready),
      .a_wdata(raw_wdata),.b_wdata(hist_wdata),
      .a_wlast(raw_wlast),.b_wlast(hist_wlast),
      .a_wvalid(raw_wvalid),.b_wvalid(hist_wvalid),
      .a_wready(raw_wready),.b_wready(hist_wready),
      .a_bvalid(raw_bvalid),.b_bvalid(hist_bvalid),
      .a_araddr(raw_araddr),.b_araddr(hist_araddr),
      .a_arlen(raw_arlen),.b_arlen(hist_arlen),
      .a_arvalid(raw_arvalid),.b_arvalid(hist_arvalid),
      .a_arready(raw_arready),.b_arready(hist_arready),
      .a_rvalid(raw_rvalid),.b_rvalid(hist_rvalid),
      .awaddr(w_ddr3_awaddr),
      .araddr(w_ddr3_araddr),
      .awlen(w_ddr3_awlen),
      .arlen(w_ddr3_arlen),
      .awvalid(w_ddr3_awvalid),
      .arvalid(w_ddr3_arvalid),
      .awready(w_ddr3_awready),
      .arready(w_ddr3_arready),
      .wdata(w_ddr3_wdata),
      .wlast(w_ddr3_wlast),
      .wvalid(w_ddr3_wvalid),
      .wready(w_ddr3_wready),
      .bvalid(w_ddr3_bvalid),
      .rlast(w_ddr3_rlast),
      .rvalid(w_ddr3_rvalid));
	ddr_video_store #(.C_RD_END_ADDR(1280 * 720), .C_W_WIDTH(CSI_DATA_WIDTH), .C_R_WIDTH(8), .C_ID_LEN(4)) u_axi4_ctrl (

		.axi_clk        (w_ddr3_ui_clk            ),
		.axi_reset      (ddr_app_reset            ),

		.axi_awaddr     (raw_awaddr       ),
		.axi_awlen      (raw_awlen        ),
		.axi_awvalid    (raw_awvalid      ),
		.axi_awready    (raw_awready      ),

		.axi_wdata      (raw_wdata        ),
		.axi_wstrb      (),
		.axi_wlast      (raw_wlast        ),
		.axi_wvalid     (raw_wvalid       ),
		.axi_wready     (raw_wready       ),

		.axi_bid        (0          ),
		.axi_bresp      (w_ddr3_bresp),
		.axi_bvalid     (raw_bvalid),

		.axi_arid       (),
		.axi_araddr     (raw_araddr       ),
		.axi_arlen      (raw_arlen        ),
		.axi_arvalid    (raw_arvalid      ),
		.axi_arready    (raw_arready      ),

		.axi_rid        (w_ddr3_rid          ),
		.axi_rdata      (w_ddr3_rdata        ),
		.axi_rresp      (w_ddr3_rresp),
		.axi_rlast      (w_ddr3_rlast        ),
		.axi_rvalid     (raw_rvalid       ),
		.axi_rready     (),

		.wframe_pclk    (cmos_pclk          ),
		.wframe_vsync   (cmos_frame_vsync), //w_wframe_vsync   ),		//	Writter VSync. Flush on rising edge. Connect to EOF. 
		.wframe_data_en (cmos_frame_href   ),
		.wframe_data    (cmos_frame_Gray),
		
		.rframe_pclk    (clk_pixel            ),
		.rframe_vsync   (~lcd_vs             ),		//	Reader VSync. Flush on rising edge. Connect to ~EOF. 
		.rframe_data_en (lcd_request             ),
		.rframe_data    (lcd_data           ),
		
		.play_mode_i(app_play_mode),.tnr_enable_i(1'b0),
        .age_a_i(app_age_a),.age_b_i(app_age_b),.replay_div_i(app_replay_div),
        .status_o(raw_status),.display_source_o(raw_source_id),
        .tp_o 		(w_axi_tp)
	);
	//assign led_o[3:0] = w_axi_tp; 	// See application LED assignments above
	
	
	
	
	////////////////////////////////////////////////////////////////
	//  LCD Timing Driver
	
	lcd_driver u_lcd_driver
	(
	    //  global clock
	    .clk        (clk_pixel   ),
	    .rst_n      (rstn_pixel), 
	    
	    //  lcd interface
	    .lcd_dclk   (               ),
	    .lcd_blank  (               ),
	    .lcd_sync   (               ),
	    .lcd_request(lcd_request    ), 	//	Request data 1 cycle ahead. 
	    .lcd_hs     (lcd_hs         ),
	    .lcd_vs     (lcd_vs         ),
	    .lcd_en     (lcd_de         ),
	    .lcd_rgb    ({lcd_red2,lcd_green2,lcd_blue2, lcd_red,lcd_green,lcd_blue}),
	    
	    //  user interface
	    .lcd_data   ({{3{lcd_data[15:8]}}, {3{lcd_data[7:0]}}}  )
	);
	
	// =========================================================================================================================================
// bayer 2 rgb
// ========================================================================================================================================= 
	
	wire 			w_rgb_vsync, w_rgb_href; 
	wire 	[7:0] 	w_rgb_pre_r, w_rgb_pre_g, w_rgb_pre_b; 
	
    VIP_RAW8_RGB888 #(.IMG_HDISP(1280), .IMG_VDISP(720)) bayer2rgb (
    .clk                               (clk_pixel                ),//cmos video pixel clock
    .rst_n                             (rstn_pixel             ),//global reset
	    
    .mirror                            (2'b00                     ),
		
		    //CMOS YCbCr444 data output
    .per_frame_vsync                   (lcd_vs                    ),//	Prepared Image data vsync valid signal. Reset on falling edge. 
    .per_frame_href                    (lcd_de                    ),//	Prepared Image data href vaild  signal
    .per_frame_hsync                   (lcd_hs                    ),
    .per_img_RAW                       (lcd_blue[7:0]             ),//	Input data from AXI reader directly. Latency is 1T. Matches with VS / HS / DE signals. 
		
    .post_frame_vsync                  (w_rgb_vsync               ),//Processed Image data vsync valid signal
    .post_frame_href                   (w_rgb_href                ),//Processed Image data href vaild  signal
    .post_frame_hsync                  (w_rgb_hsync               ),//Processed Image data href vaild  signal
    .post_img_red                      (w_rgb_pre_r               ),//Prepared Image green data to be processed 
    .post_img_green                    (w_rgb_pre_g               ),//Prepared Image green data to be processed
    .post_img_blue                     (w_rgb_pre_b               ) //Prepared Image blue data to be processed
    );
// =========================================================================================================================================
// manual awb
// ========================================================================================================================================= 
//手动白平衡算法
wire [7:0] w_rgb_r;
wire [7:0] w_rgb_g;
wire [7:0] w_rgb_b;
// ★ 2026-10-15：两个增益从写死的常数（409/384）改成**运行时可调寄存器**（由 uart_cmd 给出；
//   复位值 102/96 与 409/384 只差 0.25%/0%，不插串口时画面与改动前逐像素一致）。
//   动机：上板实测"白墙在正常室内白灯下发青" = 相机链路 R 通道偏低，写死的 ×1.6 补不回来；
//   而调这个系数要试好几轮，所以做成旋钮（'A'/'B' 命令）而不是改常数反复综合。
//   定点：增益 = byte/64；R' = (R*gain)>>6，饱和判据 (R*gain) >= 256*64 = 16384。
wire     [   7:0]   r_awb_gain;                       // 来自 uart_cmd（复位 102 ≈ ×1.59）
wire     [   7:0]   b_awb_gain;                       // 来自 uart_cmd（复位  96  = ×1.50）
wire     [15:0]     w_rgb_r_mult = w_rgb_pre_r * r_awb_gain;
wire     [15:0]     w_rgb_b_mult = w_rgb_pre_b * b_awb_gain;
wire     [ 9:0]     w_rgb_r_sc   = w_rgb_r_mult[15:6];
wire     [ 9:0]     w_rgb_b_sc   = w_rgb_b_mult[15:6];
assign w_rgb_r = (w_rgb_r_mult >= 16'd16384) ? 8'hFF : w_rgb_r_sc[7:0];
assign w_rgb_g = w_rgb_pre_g;
assign w_rgb_b = (w_rgb_b_mult >= 16'd16384) ? 8'hFF : w_rgb_b_sc[7:0];

// =========================================================================================================================================
// Image pipeline : rgb_to_gray -> pix_window_3x3 -> canny_edge -> image_mix
// 插入点：AWB 之后、FrameBoundCrop 之前（docs/ARCH.md §1），全部在 clk_pixel 单域
// =========================================================================================================================================
localparam IMG_MODE_DEF = 2'd3;  // 0=原图 1=边缘图 2=分屏(左灰度+右边缘) 3=左灰度+右原图叠加亮黄色边缘（默认）
localparam IMG_VIEW_DEF = 2'd0;  // 0=灰度抗锯齿边缘图 1=mag8 图 2=六状态假彩图 3=Y 图
                                 // （四种视图的口径见 image_pipeline_top 的【view_i】）
localparam IMG_THR_DEF  = 8'd12; // Canny 强边阈值（内部 mag8 = mag>>3 域，弱边 = 该值/2）
                                 // ★★ 从 40 → 16 → 12：原值 40 实际要求满幅 **42%** 的对比度
                                 //    （这个口不是 0..255 量程），是"只有高对比度才出边"的根因；
                                 //    2026-10-15 上板扫出来的最佳工作点是 12（除最暗的紫/灰外全部
                                 //    检出、且没有多余细线），所以复位值落 12。完整的反查标尺见
                                 //    canny_edge.v 头注释。
                                 //    ★ 2026-10-16：这个值现在是**按键控制器的复位值**
                                 //      （2026-10-17 起：K2 循环阈值 4..48、步长 1；K1 循环
                                 //      平滑模式），上板后不插串口、不重综合就能改；
                                 //      见下面的 canny_threshold_key_ctrl 例化。
                                 //    ★ 若以后把采集端修好（AWB/曝光/换打印纸卡），可以再抬回 16。
localparam IMG_LINK_DEF = 1'b0;  // 方向约束流式边缘连接复位值：1 开 / 0 关
                                 //   ★ 2026-10-19 方案A硬件A/B：先置 0（功能关，逻辑仍在），
                                 //     与 HEAD 逐像素行为一致；用来判定黑屏是否由 link 功能引起。
                                 //   'L' 命令可运行时切换（view=2 下绿色 = 仅因传播保留的弱边）；
                                 //   关掉即退回"单遍滞后 + 1 像素膨胀"的旧行为，用来上板对比。
                                 //   反馈用 2 块 IMG_W×3bit 行缓存（ping-pong），不增加流水线延迟。
localparam IMG_LAT      = 27;    // = image_pipeline_top 的 LATENCY，两处必须一致
                                 //   （2026-10-17 接入 3x3 中值：+1 中值 +2 后级窗口）

wire                                    pipe_vs                    ;
wire                                    pipe_de                    ;
wire                   [  23:0]         pipe_rgb                   ;
wire                   [   7:0]         w_img_thr                  ;
wire                   [   7:0]         w_uart_thr                 ;   // uart_cmd 'T' 写入值
wire                                    w_uart_thr_valid           ;   // 同拍单脉冲
// 自适应阈值偏移：手动值相对默认值的有符号差（K2/'T' 在 Otsu 模式下
// 不再是绝对阈值，而是 ±微调；IMG_THR_DEF=12 → 偏移 0）
wire signed [8:0] w_thr_off = $signed({1'b0, w_img_thr})
                            - $signed({1'b0, IMG_THR_DEF[7:0]});
wire                   [   1:0]         r_img_mode                 ;
wire                   [   1:0]         r_img_view                 ;
wire                   [   1:0]         r_smooth_mode              ;   // uart_cmd 'S' 寄存器
wire                                    r_smooth_valid             ;   // 同拍单脉冲
wire                                    ocr_overlay_enable         ;   // UART 'E': annotation A/B
wire                   [   1:0]         w_smooth_mode              ;   // 实际生效（按键也会改）
wire                                    r_img_link                 ;   // uart_cmd 'L' 寄存器

//---------------------------------------------------------------------
// Application UART: controls and atomic frame-store status only.
wire cmd_tx_busy,cmd_tx;
wire uart_cmd_rx=uart_rx_i;

//---------------------------------------------------------------------
// 串口命令接口：把 view / mode 从编译期常量变成运行时寄存器，现场调参不用
// 重新综合。uart_rx_i / uart_tx_o 早已声明并在 Ti60_Demo.peri.xml 里分配了
// 引脚（GPIOT_N_16 / GPIOT_P_16），此前全工程从未引用。跑在 clk_pixel 域，
// 与流水线同域，不需要 CDC。
// 命令格式（2 字节帧：命令 + 参数）：'T'<thr> 'S'<smooth> 'V'<view> 'M'<mode>，
// 'A'<AWB R 增益>，'B'<AWB B 增益>，'L'<link_en>，'R' 回读 8 字节
// {thr, view, mode, gain_r, gain_b, smooth, link, '\n'}。详见 src/common/uart_cmd.v。
//
// ★ 2026-10-17：**阈值/平滑串口控制**（与按键并存，串口优先）。
//   thr_o/thr_valid_o、smooth_o/smooth_valid_o 接到下面
//   canny_threshold_key_ctrl 的对应 uart 输入：
//   · 'T'<byte> 写入即生效，量程 0..255；K2 循环阈值，步长 1、范围 4..48；
//   · 'S'<byte> 写平滑模式 0/1/2（>2 截断）；K1 循环 0→1→2→0；
//   · 'R' 回读的第 1/6 字节分别是实际生效阈值/平滑模式（含按键修改）。
//---------------------------------------------------------------------
uart_cmd #(
    .CLK_FREQ (74250000),        // clk_pixel
    .BAUD     (460800),
    .THR_DEF  (IMG_THR_DEF),     // 'T' 写入前的回读值；与按键控制器默认值相同
    .VIEW_DEF (IMG_VIEW_DEF),
    .MODE_DEF (IMG_MODE_DEF),
    .LINK_DEF (IMG_LINK_DEF)     // 'L' 复位值：方向约束流式反馈默认开
) u_uart_cmd (
    .clk    (clk_pixel                 ),
    .rst_n  (rstn_pixel                ),
    .rx_i   (uart_cmd_rx               ),   // 直方图忙时被拉高（见上）
    .tx_o   (cmd_tx                    ),   // Application UART TX

    .thr_read_i   (w_img_thr            ),   // 'R' 回读实际生效阈值（含按键）
    .smooth_read_i(w_smooth_mode        ),   // 'R' 回读实际生效平滑模式
    .thr_o       (w_uart_thr           ),   // 'T' 写入 → 按键控制器（串口优先）
    .thr_valid_o (w_uart_thr_valid     ),   // 同拍单脉冲
    .view_o (r_img_view                ),
    .mode_o (r_img_mode                ),
    .smooth_o (r_smooth_mode           ),   // 'S' 写入 → 按键控制器
    .smooth_valid_o (r_smooth_valid    ),   // 同拍单脉冲
    .link_o   (r_img_link              ),   // 'L' 写入 → image_pipeline_top.link_en_i

    .gain_r_o (r_awb_gain              ),   // AWB R 增益（1/64 定点，复位 112 = ×1.75）
    .gain_b_o (b_awb_gain              ),   // AWB B 增益（复位 96 = ×1.50）
    .ocr_enable_o(ocr_overlay_enable   ),

    .play_mode_o(app_play_mode),.tnr_enable_o(app_tnr),
    .age_a_o(app_age_a),.age_b_o(app_age_b),.replay_div_o(app_replay_div),
    .status_req_o(app_status_req),.status_valid_i(app_status_valid),.app_status_i(app_status_pixel),
    .iir_tuning_o(iir_tuning),.history_req_o(history_req),.history_valid_i(history_valid),.history_status_i(history_pixel_status),
    .strong_pixels_i(strong_pixels),.weak_pixels_i(weak_pixels),.bypass_pixels_i(bypass_pixels),
    .hist_req_o (),   // Historical diagnostic request, unused
    .tx_busy_o  (cmd_tx_busy           )    // Application command reply busy
);

//---------------------------------------------------------------------
// Canny 阈值/平滑控制（K1 / K2 + 串口 'T'/'S'）—— 两个参数的唯一来源
//   模块自带 2FF 同步 + 20 ms 消抖 + 按下沿事件，全在 clk_pixel 域，与流水线同域。
//   K2 循环阈值：4,5,...,48,4,...（步长 1，IMG_THR_DEF 是复位值）；
//   K1 循环平滑模式：0 标准高斯 -> 1 弱平滑 -> 2 旁路 -> 0（复位值 1）。
//   串口 'T'/'S' 通过 uart_*_valid_i 直接写入，优先于按键；'T' 量程 0..255。
//   按住不放不会连续步进；不按则参数保持不变。
//---------------------------------------------------------------------
canny_threshold_key_ctrl #(
    .CLK_FREQ_HZ (74250000),     // clk_pixel
    .DEBOUNCE_MS (20),
    .THR_DEFAULT (IMG_THR_DEF),
    .THR_MIN     (8'd4),
    .THR_MAX     (8'd48),
    .THR_STEP    (8'd1)
) u_canny_thr_key (
    .clk   (clk_pixel                  ),
    .rst_n (rstn_pixel                 ),

    .key_data          ({key2_i, key1_i}),   // [0] = K1 平滑 / [1] = K2 阈值
    .uart_thr_valid_i  (w_uart_thr_valid),   // 'T' 单拍脉冲（串口优先）
    .uart_thr_i        (w_uart_thr     ),    // 'T' 写入值，量程 0..255
    .uart_smooth_valid_i (r_smooth_valid),   // 'S' 单拍脉冲（串口优先）
    .uart_smooth_i     (r_smooth_mode  ),    // 'S' 写入值，0..2

    .threshold_o       (w_img_thr      ),    // -> image_pipeline_top.thr_i
    .threshold_low_o   (               ),    // 流水线内部自己算 thr_low = thr_i >> 1
    .smooth_mode_o     (w_smooth_mode  ),    // -> image_pipeline_top.smooth_mode_i
    .key_press_o       (               )     // 单拍按下脉冲：只在板上用 ILA 才看得到
);

wire shape_edge,shape_de,shape_vs;
image_pipeline_top #(.IMG_W(1280),.IMG_H(720),.LATENCY(IMG_LAT),
                     .TEMPORAL_EXT(1),.DBG_LINK_COLOR(0),
                     .LINK_TURN90(0),.LINK_GAP1(1),.WEAK_STRUCT(1),.EDGE_AA(1)) u_image_pipeline(   // Application edge/gray views; weak structure retained
    .clk_i                             (clk_pixel                 ),
    .rst_ni                            (rstn_pixel                ),

    .vsync_i                           (w_rgb_vsync               ),
    .de_i                              (w_rgb_href                ),
    .rgb_i                             ({w_rgb_r, w_rgb_g, w_rgb_b}),
    .thr_i                             (w_img_thr                 ),
    .auto_en_i                         (1'b1                      ),   // Otsu 自适应阈值开；K2/'T' 变微调偏移
    .thr_off_i                         (w_thr_off                 ),   // = 手动阈值 − IMG_THR_DEF（有符号）
    .mode_i                            (r_img_mode                ),
    .view_i                            ((r_img_mode==3) ? 2'd0 : (r_img_view==3 ? 2'd3 : 2'd0)),
    .smooth_mode_i                     (w_smooth_mode             ),
    .link_en_i                         (r_img_link                ),   // 'L' 运行时开关

    .vsync_o                           (pipe_vs                   ),
    .de_o                              (pipe_de                   ),
    .rgb_o                             (pipe_rgb                  ),

    // M6 调试抽头：中值后、Canny 实际看到的 Y/de/vs（不增加逻辑）
    .dbg_y_o                           (median_y),
    .dbg_de_o                          (median_de),
    .dbg_vs_o                          (median_vs),

    .dbg_cond_o                        (),
    .dbg_link_en_o                     (),
    .dbg_cond_seen_o                   (),
    .dbg_link_seen_o                   (),
    .temporal_y_i(temporal_y),.temporal_de_i(temporal_de),.temporal_vs_i(temporal_vs),
    .dbg_edge_o                        (shape_edge),
    .dbg_edge_de_o                     (shape_de),
    .dbg_edge_vs_o                     (shape_vs)
);

//---------------------------------------------------------------------
// Shape-only recognition profile. Character RTL is retained in src/recognition
// for future integration; the active project excludes its detection chain.
// Coordinates follow the final pipeline stream. Annotations are combinational
// and do not change video timing or the DDR/IIR path.
//---------------------------------------------------------------------
wire [10:0] overlay_x;
wire [9:0] overlay_y;
wire [23:0] pipe_rgb_detect;
wire [287:0] geometry_objects;
wire [7:0] geometry_valid;
shape_geometry_top u_geometry (
    .clk(clk_pixel),.rst_n(rstn_pixel),.vsync_i(shape_vs),.de_i(shape_de),.edge_i(shape_edge),
    .objects_o(geometry_objects),.valid_o(geometry_valid),.busy_o(),.rejected_o(),.cycles_o()
);
shape_geometry_overlay u_geometry_overlay (
    .x_i(overlay_x),.y_i(overlay_y),.de_i(pipe_de),.enable_i(ocr_overlay_enable),
    .rgb_i(pipe_rgb),.objects_i(geometry_objects),.valid_i(geometry_valid),.rgb_o(pipe_rgb_detect)
);
video_xy_counter #(.IMG_W(1280)) u_overlay_xy (
    .clk(clk_pixel),.rst_n(rstn_pixel),.vsync_i(pipe_vs),.de_i(pipe_de),
    .x_o(overlay_x),.y_o(overlay_y)
);

//---------------------------------------------------------------------
// Application command UART, with no histogram/image diagnostic TX mux.
//---------------------------------------------------------------------
assign uart_tx_o = cmd_tx;

// hsync 要比 de 早 IMG_LAT 个像素才与流水线后的数据同拍（= image_pipeline_top 的
// LATENCY），否则 rgb2dvi 送出的 HS 与 DE 错位，HDMI 画面横移。
// 深度用 IMG_LAT 参数化：改流水线延迟时只改上面那个 localparam，别只改一边。
reg [IMG_LAT-1:0] r_hs_dly = {IMG_LAT{1'b0}};
always @(posedge clk_pixel or negedge rstn_pixel) begin
    if (!rstn_pixel) r_hs_dly <= {IMG_LAT{1'b0}};
    else             r_hs_dly <= {r_hs_dly[IMG_LAT-2:0], w_rgb_hsync};
end
wire                                    pipe_hs  = r_hs_dly[IMG_LAT-1];

wire                                    boundcrop_vs               ;
wire                                    boundcrop_hs               ;
wire                                    boundcrop_de               ;
wire                   [  23:0]         boundcrop_data             ;
FrameBoundCrop #(.SKIP_ROWS(3),.SKIP_COLS(2),.TOTAL_ROWS(720),.TOTAL_COLS(1280)) inst2_FrameCrop(
    .clk_i                             (clk_pixel                 ),
    .rst_i                             (~rstn_pixel               ),

    .vs_i                              (pipe_vs                   ),
    .hs_i                              (pipe_hs                   ),
    .de_i                              (pipe_de                   ),
    .data_i                            (pipe_rgb_detect       ),

    .vs_o                              (boundcrop_vs              ),
    .hs_o                              (boundcrop_hs              ),
    .de_o                              (boundcrop_de              ),
    .data_o                            (boundcrop_data            )
);



	assign hdmi_txd0_rst_o = rst_pixel; 
	assign hdmi_txd1_rst_o = rst_pixel; 
	assign hdmi_txd2_rst_o = rst_pixel; 
	assign hdmi_txc_rst_o = rst_pixel; 
	
	assign hdmi_txd0_oe = 1'b1; 
	assign hdmi_txd1_oe = 1'b1; 
	assign hdmi_txd2_oe = 1'b1; 
	assign hdmi_txc_oe = 1'b1; 
	

    rgb2dvi #(.ENABLE_OSERDES(0)) u_rgb2dvi
    (
    .oe_i                              (1                         ),//	Always enable output
    .bitflip_i                         (4'b0000                   ),//	Reverse clock & data lanes. 
		
    .aRst                              (1'b0                      ),
    .aRst_n                            (1'b1                      ),
		
    .PixelClk                          (clk_pixel                 ),//pixel clk = 74.25M
    .SerialClk                         (                          ),//pixel clk *5 = 371.25M
		
    .vid_pVSync                        (boundcrop_vs              ),
    .vid_pHSync                        (boundcrop_hs              ),
    .vid_pVDE                          (boundcrop_de              ),
    .vid_pData                         (boundcrop_data            ),
		
    .txc_o                             (hdmi_txc_o                ),
    .txd0_o                            (hdmi_txd0_o               ),
    .txd1_o                            (hdmi_txd1_o               ),
    .txd2_o                            (hdmi_txd2_o               ) 
    );
		
	






	
	

	////////////////////////////////////////////////////////////////
	//	GM6020 PWM 输出（位置模式，慢速爬行档）
	//
	//	clk_sys 96 MHz : PWM 载波 50 Hz，1 us = 96 clk，脉宽零误差
	//	clk_pixel 74.25 MHz : 视觉域（当前 CREEP 模式不依赖视觉）
	//
	//	接线（三根线，P4 排针）：
	//	  电机 PWM 线 A(白 信号) -> P4 第 39 脚（can_tx_o 脚，本版输出 PWM）
	//	  电机 PWM 线 C(黑 电源-) -> P4 第 30 脚（GND）
	//	  电机 PWM 线 B(灰)      -> 悬空（Assistant 调参用）
	//	  电机 24V 电源正常接；不要再插 CAN 收发器
	//
	//	注意：本脚与 CAN 版共用 P4-39，插拔线时别搞混
	////////////////////////////////////////////////////////////////

	gm6020_pwm_top #(
		.MAX_OBJECTS (8         ),
		.GRID_W      (160       ),
		.GRID_H      (90        ),
		.TRI_CLASS   (4'd1      ),		// 跟踪目标：1=三角形（当前）2=正方形 3=长方形 4=圆 …

		//--------------------------------------------- 目标门限（阶段0 抗噪，识别不到时可调小）
		.MIN_SIZE    (2         ),		// 最小边（格）
		.MAX_SIZE    (48        ),		// 最大边（格）
		.MIN_AREA    (9         ),		// 最小面积（格^2）：真目标>=3x3 格，噪块通常更小

		.PULSE_MIN   (96_000    ),		// 1000 us -> 0 deg
		.PULSE_MAX   (184_320   ),		// 1920 us -> theta_max
		.PULSE_MID   (140_160   ),		// 1460 us 上电中位

		.GAIN_NUM    (44        ),
		.GAIN_SHIFT  (2         ),
		.DEADZONE    (3         ),
		.MAX_STEP    (3_000     ),
		.DIR_INVERT  (0         ),
		.LOST_HOLD   (30        ),
		.RETURN_STEP (200       ),
		.SEEK_STEP   (400       ),
		.SEEK_DELAY  (30        ),

		.SELF_TEST   (0         ),		// =1：纯 PWM 自检（不看视觉）；=0：识别三角形并跟转（当前）
		.CAL_MODE    (0         ),		// 首次上电做行程标定时置 1

		//---------------------------------------------------------------- 往返速度（仅 SELF_TEST=1 时有效）
		.SCAN_STEP   (192       ),

		//---------------------------------------------------------------- 慢速爬行档（当前生效）
		//	脉宽在 1400~1600 us 之间以 25 us/s 缓慢往返，约 8 s 单程
		.CREEP_MODE  (0         ),
		.CREEP_MIN   (134_400   ),		// 1400 us
		.CREEP_MAX   (153_600   ),		// 1600 us
		.CREEP_STEP  (48        ),		// counts / 20 ms = 25 us/s

		//---------------------------------------------------------------- 恒速档
		.HOLD_PULSE  (0         )
	) u_pwm_tracker (
		.clk_pixel   (clk_pixel         ),
		.rst_pixel_n (rstn_pixel        ),
		.vsync_i     (shape_vs          ),
		.objects_i   (geometry_objects  ),
		.valid_i     (geometry_valid    ),
		.clk_sys     (clk_sys           ),
		.rst_sys_n   (rstn_sys          ),
		.pwm_o       (pwm_motor         ),
		.status_o    (pwm_status        ),
		.ex_o        (pwm_ex_dbg        ),
		.found_o     (pwm_found_dbg     ),
		.pulse_o     (pwm_duty_dbg      )
	);

	assign can_tx_o = pwm_motor;	//	PWM 从 P4-39（与 CAN 版共用的球 A9）输出

endmodule


