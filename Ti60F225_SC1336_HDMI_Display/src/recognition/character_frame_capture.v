`timescale 1ns/1ps
// 独立灰度支路：4x4 占比池化，分别保存深色/浅色前景，不要求卡片。
// 忙时跳过分析帧，不向摄像头或 HDMI 施加反压。
module character_frame_capture #(
 parameter integer IMG_W=1280,IMG_H=720,W=IMG_W/4,H=IMG_H/4,AW=$clog2(W*H)
)(
 input wire clk,rst_n,vsync_i,de_i,input wire [7:0] gray_i,input wire busy_i,
 output reg ready_o,output reg [7:0] threshold_o,output reg contrast_o,
 input wire [AW-1:0] addr_i,output wire [1:0] data_o
);
 localparam integer TAIL_PIXELS=(W*H/2048<2)?2:W*H/2048;
 reg [10:0] x;reg [9:0] y;reg vs_d,de_d,capturing;
 reg [2:0] horizontal;reg [4:0] vertical[0:W-1];
 reg [7:0] frame_threshold;wire dark=gray_i<frame_threshold;
 wire [3:0] count_h={1'b0,horizontal}+dark;
 wire [4:0] count_v=(y[1:0]==0?5'd0:vertical[x>>2])+count_h;
 reg we;reg [AW-1:0] wa;reg [1:0] wd;
 reg [15:0] hist[0:15];reg [1:0] cal;reg [3:0] hi;
 reg [16:0] cumulative;reg low_found,high_found;reg [7:0] low_level,high_level;
 wire [16:0] next_sum=cumulative+hist[hi];
 wire [8:0] mid_level={1'b0,low_level}+{1'b0,high_level};
 shape_sdp_ram #(.WIDTH(2),.DEPTH(W*H),.AW(AW)) mem(clk,we,wa,addr_i,wd,data_o);
 integer i;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   x<=0;y<=0;vs_d<=0;de_d<=0;capturing<=0;horizontal<=0;frame_threshold<=128;
   ready_o<=0;threshold_o<=128;contrast_o<=0;we<=0;wa<=0;wd<=0;
   cal<=0;hi<=0;cumulative<=0;low_found<=0;high_found<=0;low_level<=0;high_level<=0;
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
      we<=1;wa<=(y>>2)*W+(x>>2);wd<={count_v<=8,count_v>=8};
      hist[gray_i[7:4]]<=hist[gray_i[7:4]]+1'b1;
     end
    end
    if(x==IMG_W-1&&y==IMG_H-1) capturing<=0;
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
   end
  end
 end
endmodule
