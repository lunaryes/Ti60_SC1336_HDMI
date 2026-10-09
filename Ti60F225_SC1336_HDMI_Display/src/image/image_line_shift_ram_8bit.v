//==============================================================
// 【参考副本，未参与编译】
// 本文件是 src/isp/Line_Shift_RAM_8Bit.v 的复制件（模块名已改，避免与
// 原件重名冲突），仅为满足 README 铁律 1「需要行缓存就复制到 src/image/
// 再改造」而保留，供对照阅读。
//
// pix_window_3x3 最终没有采用它，原因：
//   (1) 该原语实现的是"延迟 1 行 + 1 像素"（内部地址模 DATA_DEPTH+1 循环，
//       INIT_ADDR = DATA_DEPTH），相邻两行之间有固定 1 像素错位，必须像
//       VIP_Matrix_Generate_3X3_8Bit 那样用 pixel_cnt 特判去补；
//   (2) 存储深度写成 2**ADDR_WIDTH，1280 点的行要占 2048 深，浪费约 40%。
// 改用「3 个 bank 按行轮转 + 预测式读地址」结构，见 pix_window_3x3.v。
//
// 本文件未登记进 Ti60_Demo.xml，不参与综合。
//==============================================================
//  by CrazyBird
module image_line_shift_ram_8bit
#(
    parameter DATA_WIDTH = 8    ,
    parameter ADDR_WIDTH = 11   ,
    parameter DATA_DEPTH = 1280 ,
    parameter DELAY_NUM  = 0
)(
    input  wire                     clk     ,
    input  wire                     rst_n   ,
    input  wire                     clken   ,
    input  wire [DATA_WIDTH-1:0]    din     ,   
    output wire [DATA_WIDTH-1:0]    dout    
);
//----------------------------------------------------------------------
localparam BRAM_DEPTH = DATA_DEPTH + 1;
localparam INIT_ADDR  = DATA_DEPTH - DELAY_NUM;

//----------------------------------------------------------------------
reg             [ADDR_WIDTH-1:0]    bram_waddr;
reg             [ADDR_WIDTH-1:0]    bram_raddr;

always @(posedge clk or negedge rst_n)
begin
    if(rst_n == 1'b0)
    begin
        bram_waddr <= INIT_ADDR;
        bram_raddr <= 0;
    end
    else
    begin
        if(clken == 1'b1)
        begin
            if(bram_waddr < DATA_DEPTH)
                bram_waddr <= bram_waddr + 1'b1;
            else
                bram_waddr <= 0;
            if(bram_raddr < DATA_DEPTH)
                bram_raddr <= bram_raddr + 1'b1;
            else
                bram_raddr <= 0;
        end
        else
        begin
            bram_waddr <= bram_waddr;
            bram_raddr <= bram_raddr;
        end
    end
end




//----------------------------------------------------------------------
wire            [DATA_WIDTH-1:0]    bram_wdata;
wire                                bram_wenb;
reg            [DATA_WIDTH-1:0]    bram_rdata;

assign bram_wdata = din;
assign bram_wenb  = clken;


localparam ADDR_MSB = 2 ** ADDR_WIDTH - 1; 
reg 	[DATA_WIDTH-1:0] 	r_ram[ADDR_MSB:0]; 

always @(posedge clk) begin
	if(bram_wenb) begin
		r_ram[bram_waddr] <= bram_wdata; 
	end else begin
	end
end
always @(posedge clk) begin
	bram_rdata <= r_ram[bram_raddr]; 
end

//shift_reg_bram
//#(
//    .DATA_WIDTH_A   (DATA_WIDTH ),
//    .ADDR_WIDTH_A   (ADDR_WIDTH ),
//    .DATA_DEPTH_A   (BRAM_DEPTH ),
//    .DATA_WIDTH_B   (DATA_WIDTH ),
//    .ADDR_WIDTH_B   (ADDR_WIDTH ),
//    .DATA_DEPTH_B   (BRAM_DEPTH )
//)
//u_shift_reg_bram
//( 
//    .clka   (clk        ),
//    .addra  (bram_waddr ),
//    .dia    (bram_wdata ),
//    .cea    (bram_wenb  ),
//    
//    .clkb   (clk        ),
//    .addrb  (bram_raddr ),
//    .ceb    (1'b1       ),
//    .dob    (bram_rdata )
//);

assign dout = bram_rdata;

endmodule
