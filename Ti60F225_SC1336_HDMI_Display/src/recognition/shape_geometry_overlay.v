`timescale 1ns/1ps
// Combinational annotations only: no change to HDMI HS/VS/DE or latency.
module shape_geometry_overlay (
 input wire [10:0] x_i,input wire [9:0] y_i,
 input wire de_i,enable_i,input wire [23:0] rgb_i,
 input wire [143:0] objects_i,input wire [3:0] valid_i,
 output reg [23:0] rgb_o
);
 integer i;reg [35:0] box;reg [10:0] x0,x1;reg [9:0] y0,y1;
 reg border,in_text;reg [7:0] selected_class;
 reg [10:0] selected_x;reg [9:0] selected_y;
 wire ink;wire [6:0] dx=x_i-selected_x;wire [4:0] dy=y_i-selected_y;
 vision_label_cn u_label(1'b0,1'b0,selected_class,dx,dy,ink);
 always @(*) begin
   border=0;in_text=0;selected_class=0;box=0;x0=0;x1=0;y0=0;y1=0;selected_x=0;selected_y=0;
   for(i=0;i<4;i=i+1) begin
     box=objects_i[i*36+:36];x0={box[31:24],3'b0};x1={box[23:16],3'b111};
     y0={box[15:8],3'b0};y1={box[7:0],3'b111};
     if(valid_i[i]) begin
       if(((x_i==x0||x_i==x1)&&y_i>=y0&&y_i<=y1)||
          ((y_i==y0||y_i==y1)&&x_i>=x0&&x_i<=x1)) border=1;
       if(x_i>=x0&&x_i<x0+96&&y_i>=y0+2&&y_i<y0+34) begin
         in_text=1;selected_class={4'b0,box[35:32]};selected_x=x0;selected_y=y0+2;
       end
     end
   end
 end
 always @(*) begin
   rgb_o=rgb_i;
   if(enable_i&&de_i) begin
     if(border) rgb_o=24'h00ffff;
     if(in_text) rgb_o=ink?24'h00ff00:24'h101010;
   end
 end
endmodule
