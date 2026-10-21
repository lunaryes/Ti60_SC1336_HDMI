`timescale 1ns/1ps
// 只抑制已确认字符包围范围内的从属几何轮廓，不更改 CCL/Moore 结果本身。
// 0/O/1/I 本身与几何形状存在歧义，不用它们压制独立图形。
module recognition_object_fusion(
 input wire [143:0] geometry_objects_i,input wire [3:0] geometry_valid_i,
 input wire [167:0] characters_i,input wire [3:0] character_valid_i,
 output reg [3:0] geometry_valid_o
);
 integer i,j;reg [35:0] g;reg [41:0] c;reg [10:0] gx0,gx1,cx0,cx1;
 reg [9:0] gy0,gy1,cy0,cy1;reg [7:0] ascii;
 always @(*) begin
  geometry_valid_o=geometry_valid_i;g=0;c=0;gx0=0;gx1=0;gy0=0;gy1=0;
  cx0=0;cx1=0;cy0=0;cy1=0;ascii=0;
  for(i=0;i<4;i=i+1) begin
   g=geometry_objects_i[i*36+:36];gx0={g[31:24],3'b000};gx1={g[23:16],3'b111};
   gy0={g[15:8],3'b000};gy1={g[7:0],3'b111};
   for(j=0;j<4;j=j+1) begin
    c=characters_i[j*42+:42];ascii=c[41:34];
    cx0={c[33:25],2'b00};cx1={c[24:16],2'b11};cy0={c[15:8],2'b00};cy1={c[7:0],2'b11};
    if(character_valid_i[j]&&ascii!="0"&&ascii!="O"&&ascii!="1"&&ascii!="I"&&
       {1'b0,gx0}+8>={1'b0,cx0}&&{1'b0,gx1}<={1'b0,cx1}+8&&
       {1'b0,gy0}+8>={1'b0,cy0}&&{1'b0,gy1}<={1'b0,cy1}+8)
     geometry_valid_o[i]=0;
   end
  end
 end
endmodule
