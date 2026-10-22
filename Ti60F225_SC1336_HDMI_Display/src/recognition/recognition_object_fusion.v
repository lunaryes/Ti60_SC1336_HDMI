`timescale 1ns/1ps
// 只抑制已确认字符包围范围内的从属几何轮廓，不更改 CCL/Moore 结果本身。
// 0/O/1/I 本身与几何形状存在歧义，不用它们压制独立图形。
// GC/CC 分开：图形与字符的槽位数可以不同（字符常常需要更多槽位）。
module recognition_object_fusion #(parameter integer GC=8,CC=12)(
 input wire [GC*36-1:0] geometry_objects_i,input wire [GC-1:0] geometry_valid_i,
 input wire [CC*42-1:0] characters_i,input wire [CC-1:0] character_valid_i,
 output wire [GC-1:0] geometry_valid_o,input wire clk,rst_n
);
 localparam integer MAXC=(GC>CC)?GC:CC;
 localparam IW=$clog2(MAXC);
 reg [GC*36-1:0] gs;reg [CC*42-1:0] cs;
 reg [GC-1:0] gv;reg [CC-1:0] cv;
 reg [GC-1:0] work_mask,mask;
 reg [IW-1:0] i,j;reg busy,suppress;
 wire same=gs==geometry_objects_i&&cs==characters_i&&gv==geometry_valid_i&&cv==character_valid_i;
 reg [35:0] g;reg [41:0] c;reg [10:0] gx0,gx1,cx0,cx1;
 reg [9:0] gy0,gy1,cy0,cy1;reg [7:0] ascii;
 // 每拍比较一组 (i,j)，GC*CC 拍完成一轮。
 always @(*) begin
  suppress=0;g=0;c=0;gx0=0;gx1=0;gy0=0;gy1=0;
  cx0=0;cx1=0;cy0=0;cy1=0;ascii=0;
   g=gs[i*36+:36];gx0={g[31:24],3'b000};gx1={g[23:16],3'b111};
   gy0={g[15:8],3'b000};gy1={g[7:0],3'b111};
    c=cs[j*42+:42];ascii=c[41:34];
    cx0={c[33:25],2'b00};cx1={c[24:16],2'b11};cy0={c[15:8],2'b00};cy1={c[7:0],2'b11};
    if(cv[j]&&ascii!="0"&&ascii!="O"&&ascii!="1"&&ascii!="I"&&
       {1'b0,gx0}+8>={1'b0,cx0}&&{1'b0,gx1}<={1'b0,cx1}+8&&
       {1'b0,gy0}+8>={1'b0,cy0}&&{1'b0,gy1}<={1'b0,cy1}+8)
     suppress=1;
 end
 // Changed snapshots bypass stale suppression until the new mask is complete.
 assign geometry_valid_o=geometry_valid_i&((same&&!busy)?mask:{GC{1'b1}});
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin gs<=0;cs<=0;gv<=0;cv<=0;work_mask<=0;mask<=0;i<=0;j<=0;busy<=0;end
  else if(!same) begin
   gs<=geometry_objects_i;cs<=characters_i;gv<=geometry_valid_i;cv<=character_valid_i;
   work_mask<=geometry_valid_i;i<=0;j<=0;busy<=1;
  end else if(busy) begin
   if(suppress) work_mask[i]<=0;
   if(j==CC-1) begin
    j<=0;
    if(i==GC-1) begin
     mask<=work_mask&~(suppress?({{(GC-1){1'b0}},1'b1}<<i):{GC{1'b0}});busy<=0;
    end else i<=i+1'b1;
   end else j<=j+1'b1;
  end
 end
endmodule
