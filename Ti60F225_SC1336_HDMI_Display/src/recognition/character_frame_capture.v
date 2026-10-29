`timescale 1ns/1ps
// 独立灰度支路：4x4 占比池化，分别保存深色/浅色前景，不要求卡片。
// 忙时跳过分析帧，不向摄像头或 HDMI 施加反压。
module character_frame_capture #(
 parameter integer IMG_W=1280,IMG_H=720,W=IMG_W/4,H=IMG_H/4,AW=$clog2(W*H)
)(
 input wire clk,rst_n,vsync_i,de_i,input wire [7:0] gray_i,input wire busy_i,
 output reg ready_o,output reg [7:0] threshold_o,output reg contrast_o,
 input wire [AW-1:0] addr_i,output wire [3:0] data_o
);
 localparam integer TAIL_PIXELS=(W*H/2048<2)?2:W*H/2048;
 localparam integer TILE_COLS=(IMG_W+255)/256,TILE_ROWS=(IMG_H+255)/256,TILES=TILE_COLS*TILE_ROWS;
 reg [7:0] tile_low[0:TILES-1],tile_high[0:TILES-1],tile_threshold[0:TILES-1];
 reg [TILES-1:0] tile_valid;reg tiles_complete;
 reg [10:0] x;reg [9:0] y;reg vs_d,de_d,capturing;
 reg [2:0] horizontal;reg [4:0] vertical[0:W-1];
 reg [7:0] frame_threshold;
 wire [7:0] tile_index=(y>>8)*TILE_COLS+(x>>8);
 wire [7:0] selected_threshold=tile_valid[tile_index]?tile_threshold[tile_index]:frame_threshold;
 wire dark=gray_i<selected_threshold;
 wire [3:0] count_h={1'b0,horizontal}+dark;
 wire [4:0] count_v=(y[1:0]==0?5'd0:vertical[x>>2])+count_h;
 reg we;reg [AW-1:0] wa;reg [3:0] wd;
 reg [15:0] hist[0:15];reg [1:0] cal;reg [3:0] hi;
 reg [16:0] cumulative;reg low_found,high_found;reg [7:0] low_level,high_level;
 wire [16:0] next_sum=cumulative+hist[hi];
 wire [8:0] mid_level={1'b0,low_level}+{1'b0,high_level};
 reg [8:0] tile_mid;
 shape_sdp_ram #(.WIDTH(4),.DEPTH(W*H),.AW(AW)) mem(clk,we,wa,addr_i,wd,data_o);
 integer i;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   x<=0;y<=0;vs_d<=0;de_d<=0;capturing<=0;horizontal<=0;frame_threshold<=128;
   ready_o<=0;threshold_o<=128;contrast_o<=0;we<=0;wa<=0;wd<=0;
   cal<=0;hi<=0;cumulative<=0;low_found<=0;high_found<=0;low_level<=0;high_level<=0;
   tiles_complete<=0;tile_valid<=0;
   for(i=0;i<TILES;i=i+1) begin tile_low[i]<=255;tile_high[i]<=0;tile_threshold[i]<=128;end
   for(i=0;i<16;i=i+1) hist[i]<=0;
  end else begin
   vs_d<=vsync_i;de_d<=de_i;we<=0;ready_o<=0;
   if(de_i) x<=x==IMG_W-1?0:x+1'b1;else begin x<=0;horizontal<=0;end
   if(de_d&&!de_i) y<=y+1'b1;
   if(capturing&&de_i) begin
    horizontal<=count_h;
    if(x[1:0]==3) begin
     horizontal<=0;vertical[x>>2]<=count_v;
     if(y[1:0]==3) begin
      // 四分之一占比保留斜向细笔画，避免多数表决把描边字切成碎段。
      we<=1;wa<=(y>>2)*W+(x>>2);wd<={count_v<=8,count_v>=8,count_v<=12,count_v>=4};
      hist[gray_i[7:4]]<=hist[gray_i[7:4]]+1'b1;
      if(gray_i<tile_low[tile_index]) tile_low[tile_index]<=gray_i;
      if(gray_i>tile_high[tile_index]) tile_high[tile_index]<=gray_i;
     end
    end
    if(x==IMG_W-1&&y==IMG_H-1) begin capturing<=0;tiles_complete<=1;end
   end
   if(we&&wa==W*H-1) begin
    ready_o<=contrast_o;cal<=1;hi<=0;cumulative<=0;low_found<=0;high_found<=0;low_level<=0;high_level<=0;
   end
   if(cal==1) begin
    cumulative<=next_sum;hist[hi]<=0;
    if(!low_found&&next_sum>=TAIL_PIXELS) begin low_found<=1;low_level<={hi,4'h8};end
    if(hi==15) cal<=2;else hi<=hi+1'b1;
    if(!high_found&&next_sum>=W*H-TAIL_PIXELS) begin high_found<=1;high_level<={hi,4'h8};end
   end else if(cal==2) begin
    threshold_o<=mid_level[8:1];contrast_o<=low_found&&high_found&&high_level>=low_level+32;cal<=0;
   end
   if(vsync_i&&!vs_d) begin
    x<=0;y<=0;horizontal<=0;capturing<=!busy_i;frame_threshold<=threshold_o;
    // 上一完整分析帧的分区阈值；整帧内不变，避免扫描过程中参数跳变。
    if(tiles_complete) begin
     tiles_complete<=0;
     for(i=0;i<TILES;i=i+1) begin
      tile_mid=({1'b0,tile_low[i]}+{1'b0,tile_high[i]})>>1;
      tile_valid[i]<={1'b0,tile_high[i]}>={1'b0,tile_low[i]}+48;
      // 局部校正限幅，纯色区/低对比度区回退全局阈值。
      if(tile_mid>{1'b0,threshold_o}+48) tile_threshold[i]<=threshold_o>207?255:threshold_o+48;
      else if(tile_mid+48<{1'b0,threshold_o}) tile_threshold[i]<=threshold_o<48?0:threshold_o-48;
      else tile_threshold[i]<=tile_mid[7:0];
     end
    end
    if(!busy_i) for(i=0;i<TILES;i=i+1) begin tile_low[i]<=255;tile_high[i]<=0;end
   end
  end
 end
endmodule
