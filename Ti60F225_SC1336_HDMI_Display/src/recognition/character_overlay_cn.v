`timescale 1ns/1ps
// 中文标注；非标注像素逐位直通，不改变视频时序。
module character_overlay_cn #(parameter integer MAX_OBJECTS=8)(
 input wire [10:0] x_i,input wire [9:0] y_i,input wire de_i,enable_i,
 input wire [23:0] rgb_i,input wire [MAX_OBJECTS*42-1:0] objects_i,input wire [MAX_OBJECTS-1:0] valid_i,
 input wire legacy_recognized_i,input wire [5:0] legacy_class_i,output reg [23:0] rgb_o,
 input wire clk,rst_n
);
 integer i,k;reg [41:0] box;reg [10:0] x0,x1,sx;reg [9:0] y0,y1,sy,label_y;
 reg [10:0] caption_x[0:MAX_OBJECTS-1],work_x[0:MAX_OBJECTS-1];reg [9:0] caption_y[0:MAX_OBJECTS-1],work_y[0:MAX_OBJECTS-1];
 reg [MAX_OBJECTS*42-1:0] snapshot;reg [MAX_OBJECTS-1:0] snapshot_valid;reg [2:0] state;
 localparam IW=(MAX_OBJECTS<2)?1:$clog2(MAX_OBJECTS);
 reg [IW-1:0] oi,cj,attempt;reg [10:0] candidate_x;reg [9:0] candidate_y;
 wire [41:0] object=snapshot[oi*42+:42];
 wire [10:0] ox={object[33:25],2'b0};wire [9:0] oy={object[15:8],2'b0},bottom={object[7:0],2'b11};
 // 文字排版按对象/冲突逐步完成；不串在每个视频像素的数据组合路径上。
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   snapshot<=0;snapshot_valid<=0;state<=0;oi<=0;cj<=0;attempt<=0;candidate_x<=0;candidate_y<=0;
   for(k=0;k<MAX_OBJECTS;k=k+1) begin caption_x[k]<=0;caption_y[k]<=0;work_x[k]<=0;work_y[k]<=0;end
  end else case(state)
   0:if(snapshot!=objects_i||snapshot_valid!=valid_i) begin snapshot<=objects_i;snapshot_valid<=valid_i;oi<=0;state<=1;end
   1:begin
    candidate_x<=ox>1184?1184:ox;candidate_y<=oy>=34?oy-34:(bottom<=686?bottom+2:688);
    cj<=0;attempt<=0;state<=2;
   end
   2:begin
    if(cj<oi&&snapshot_valid[cj]&&snapshot_valid[oi]&&
       candidate_x<work_x[cj]+96&&work_x[cj]<candidate_x+96&&
       candidate_y<work_y[cj]+32&&work_y[cj]<candidate_y+32) state<=3;
    else if(cj==MAX_OBJECTS-1) state<=4;else cj<=cj+1'b1;
   end
   3:begin
    if(attempt==MAX_OBJECTS-1) state<=4;
    else begin
     candidate_y<=candidate_y>=34?candidate_y-34:
       (bottom+2+attempt*34<=688?bottom+2+attempt*34:688-attempt*34);
     attempt<=attempt+1'b1;cj<=0;state<=2;
    end
   end
   4:begin work_x[oi]<=candidate_x;work_y[oi]<=candidate_y;
    if(oi==MAX_OBJECTS-1) state<=5;else begin oi<=oi+1'b1;state<=1;end
   end
   5:begin for(k=0;k<MAX_OBJECTS;k=k+1) begin caption_x[k]<=work_x[k];caption_y[k]<=work_y[k];end state<=0;end
   default:state<=0;
  endcase
 end
 reg border,in_text,unknown;reg [7:0] ascii;
 wire ink;wire [6:0] dx=x_i-sx;wire [4:0] dy=y_i-sy;
 vision_label_cn label(1'b1,unknown,ascii,dx,dy,ink);
 always @(*) begin
  box=0;x0=0;x1=0;y0=0;y1=0;sx=0;sy=0;label_y=0;border=0;in_text=0;ascii=0;unknown=0;
  for(i=0;i<MAX_OBJECTS;i=i+1) begin
   box=objects_i[i*42+:42];x0={box[33:25],2'b0};x1={box[24:16],2'b11};
   y0={box[15:8],2'b0};y1={box[7:0],2'b11};label_y=caption_y[i];
   if(valid_i[i]) begin
    if(((x_i==x0||x_i==x1)&&y_i>=y0&&y_i<=y1)||((y_i==y0||y_i==y1)&&x_i>=x0&&x_i<=x1)) border=1;
    if(x_i>=caption_x[i]&&x_i<caption_x[i]+96&&y_i>=label_y&&y_i<label_y+32) begin in_text=1;sx=caption_x[i];sy=label_y;ascii=box[41:34];end
   end
  end
  // 不再用左上角的单个字符概括多目标场景；仅标注确认后的各目标。
 end
 always @(*) begin
  rgb_o=rgb_i;
  if(enable_i&&de_i) begin
   if(border) rgb_o=24'hffff00;
   if(in_text) rgb_o=ink?24'h00ff00:24'h101010;
  end
 end
endmodule
