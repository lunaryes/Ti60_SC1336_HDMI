`timescale 1ns/1ps
// Combinational annotations only: no change to HDMI HS/VS/DE or latency.
module shape_geometry_overlay (
 input wire [10:0] x_i,input wire [9:0] y_i,
 input wire de_i,enable_i,input wire [23:0] rgb_i,
 input wire [143:0] objects_i,input wire [3:0] valid_i,
 output reg [23:0] rgb_o
);
 integer i;reg [35:0] box;reg [10:0] x0,x1;reg [9:0] y0,y1;
 reg border,in_text;reg [71:0] label_text;reg [7:0] letter;
 reg [5:0] font_addr;reg [2:0] fx,fy;reg [3:0] col;
 reg [10:0] selected_x;reg [9:0] selected_y;
 wire [63:0] bitmap;wire [7:0] unused_ascii;
 symbol_template_rom u_font(.addr_i(font_addr),.bitmap_o(bitmap),.ascii_o(unused_ascii));
 function [71:0] name9;
 input [3:0] id;
 begin case(id)
 1:name9="TRIANGLE ";2:name9="SQUARE   ";3:name9="RECTANGLE";
 4:name9="CIRCLE   ";5:name9="ELLIPSE  ";6:name9="PENTAGON ";
 7:name9="HEXAGON  ";8:name9="PLUS     ";9:name9="QUAD     ";
 default:name9="         ";endcase end
 endfunction
 always @(*) begin
   border=0;in_text=0;label_text=0;box=0;x0=0;x1=0;y0=0;y1=0;selected_x=0;selected_y=0;
   for(i=0;i<4;i=i+1) begin
     box=objects_i[i*36+:36];x0={box[31:24],3'b0};x1={box[23:16],3'b111};
     y0={box[15:8],3'b0};y1={box[7:0],3'b111};
     if(valid_i[i]) begin
       if(((x_i==x0||x_i==x1)&&y_i>=y0&&y_i<=y1)||
          ((y_i==y0||y_i==y1)&&x_i>=x0&&x_i<=x1)) border=1;
       if(x_i>=x0&&x_i<x0+144&&y_i>=y0+2&&y_i<y0+18) begin
         in_text=1;label_text=name9(box[35:32]);selected_x=x0;selected_y=y0+2;
       end
     end
   end
   col=(x_i-selected_x)>>4;letter=label_text>>((8-col)*8);
   font_addr=letter>="A"&&letter<="Z"?letter-8'd55:6'd63;
   fx=(x_i-selected_x)>>1;fy=(y_i-selected_y)>>1;
 end
 always @(*) begin
   rgb_o=rgb_i;
   if(enable_i&&de_i) begin
     if(border) rgb_o=24'h00ffff;
     if(in_text) rgb_o=bitmap[6'd63-{fy,fx}]?24'h00ff00:24'h101010;
   end
 end
endmodule
