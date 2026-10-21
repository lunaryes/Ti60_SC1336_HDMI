`timescale 1ns/1ps
// 中文标注；非标注像素逐位直通，不改变视频时序。
module character_overlay_cn(
 input wire [10:0] x_i,input wire [9:0] y_i,input wire de_i,enable_i,
 input wire [23:0] rgb_i,input wire [167:0] objects_i,input wire [3:0] valid_i,
 input wire legacy_recognized_i,input wire [5:0] legacy_class_i,output reg [23:0] rgb_o
);
 integer i;reg [41:0] box;reg [10:0] x0,x1,sx;reg [9:0] y0,y1,sy,label_y;
 reg border,in_text,unknown;reg [7:0] ascii;
 wire ink;wire [6:0] dx=x_i-sx;wire [4:0] dy=y_i-sy;
 vision_label_cn label(1'b1,unknown,ascii,dx,dy,ink);
 always @(*) begin
  box=0;x0=0;x1=0;y0=0;y1=0;sx=0;sy=0;label_y=0;border=0;in_text=0;ascii=0;unknown=0;
  for(i=0;i<4;i=i+1) begin
   box=objects_i[i*42+:42];x0={box[33:25],2'b0};x1={box[24:16],2'b11};
   y0={box[15:8],2'b0};y1={box[7:0],2'b11};label_y=y0>=34?y0-34:y0+2;
   if(valid_i[i]) begin
    if(((x_i==x0||x_i==x1)&&y_i>=y0&&y_i<=y1)||((y_i==y0||y_i==y1)&&x_i>=x0&&x_i<=x1)) border=1;
    if(x_i>=x0&&x_i<x0+96&&y_i>=label_y&&y_i<label_y+32) begin in_text=1;sx=x0;sy=label_y;ascii=box[41:34];end
   end
  end
  if(x_i>=24&&x_i<120&&y_i>=128&&y_i<160) begin
   in_text=1;sx=24;sy=128;unknown=!legacy_recognized_i&&valid_i==0;
   ascii=legacy_class_i<=10?legacy_class_i+8'd47:legacy_class_i+8'd54;
   if(!legacy_recognized_i) begin
    if(valid_i[0]) ascii=objects_i[34+:8];
    else if(valid_i[1]) ascii=objects_i[76+:8];
    else if(valid_i[2]) ascii=objects_i[118+:8];
    else if(valid_i[3]) ascii=objects_i[160+:8];
   end
  end
 end
 always @(*) begin
  rgb_o=rgb_i;
  if(enable_i&&de_i) begin
   if(border) rgb_o=24'hffff00;
   if(in_text) rgb_o=ink?24'h00ff00:24'h101010;
  end
 end
endmodule
