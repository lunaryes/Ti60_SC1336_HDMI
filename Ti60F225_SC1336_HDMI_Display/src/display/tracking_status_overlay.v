// Frame-latched HUD; combinational pixels preserve the video latency.
module tracking_status_overlay(input wire clk,rst_n,vsync_i,de_i,
 input wire [10:0] x_i,input wire [9:0] y_i,input wire [23:0] rgb_i,
 input wire [7:0] threshold_i,input wire [4:0] tracking_i,
 output reg [23:0] rgb_o);
 reg vs_prev;reg [7:0] threshold;reg [4:0] tracking;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin vs_prev<=0;threshold<=12;tracking<=0;end
 else begin vs_prev<=vsync_i;if(vsync_i && !vs_prev) begin
 threshold<=threshold_i;tracking<=tracking_i;end end
 function [34:0] font;input [7:0] ch;begin case(ch)
 8'd48:font=35'b01110100011001110101110011000101110;
 8'd49:font=35'b00100011000010000100001000010001110;
 8'd50:font=35'b01110100010000100010001000100011111;
 8'd51:font=35'b11110000010000101110000010000111110;
 8'd52:font=35'b00010001100101010010111110001000010;
 8'd53:font=35'b11111100001000011110000010000111110;
 8'd54:font=35'b01110100001000011110100011000101110;
 8'd55:font=35'b11111000010001000100010000100001000;
 8'd56:font=35'b01110100011000101110100011000101110;
 8'd57:font=35'b01110100011000101111000010000101110;
 8'd84:font=35'b11111001000010000100001000010000100;
 8'd72:font=35'b10001100011000111111100011000110001;
 8'd82:font=35'b11110100011000111110101001001010001;
 8'd58:font=35'b00000001000010000000001000010000000;
 8'd80:font=35'b11110100011000111110100001000010000;
 8'd65:font=35'b01110100011000111111100011000110001;
 8'd85:font=35'b10001100011000110001100011000101110;
 8'd83:font=35'b01111100001000001110000010000111110;
 8'd69:font=35'b11111100001000011110100001000011111;
 8'd68:font=35'b11110100011000110001100011000111110;
 8'd87:font=35'b10001100011000110101101011010101010;
 8'd73:font=35'b01110001000010000100001000010001110;
 8'd67:font=35'b01110100011000010000100001000101110;
 8'd78:font=35'b10001110011010110011100011000110001;
 8'd76:font=35'b10000100001000010000100001000011111;
 8'd77:font=35'b10001110111010110101100011000110001;
 8'd75:font=35'b10001100101010011000101001001010001;
 default:font=0;endcase end endfunction
 reg [47:0] label;reg [7:0] ch;reg [34:0] bits;
 integer col,row,gx,gy;reg ink;reg [23:0] color;
 always @* begin
  if(tracking[4]) begin label="PAUSED";color=24'hffb040;end
  else if(!tracking[0]) begin label="WAIT  ";color=24'hffff00;end
  else if(tracking[3]) begin label="LIMIT ";color=24'hff6060;end
  else if(tracking[1]) begin label="CENTER";color=24'h40ff80;end
  else begin label="TRACK ";color=24'h40ffff;end
  ch=32;col=0;row=0;gx=0;gy=0;bits=0;ink=0;rgb_o=rgb_i;
  if(de_i && x_i>=16 && x_i<240 && y_i>=16 && y_i<80) begin
   rgb_o=24'h101820;
   col=(x_i-16)>>5;row=(y_i-16)>>5;
   gx=((x_i-16)&31)>>2;gy=((y_i-16)&31)>>2;
   if(row==0) case(col)
    0:ch="T";1:ch="H";2:ch="R";3:ch=":";
    4:ch=48+threshold/10;5:ch=48+threshold%10;
    default:ch=32;endcase
   else case(col)
    0:ch=label[47:40];1:ch=label[39:32];2:ch=label[31:24];
    3:ch=label[23:16];4:ch=label[15:8];5:ch=label[7:0];default:ch=32;endcase
   bits=font(ch);
   if(gx<5 && gy<7) ink=bits[34-gy*5-gx];
   if(ink) rgb_o=row==0?24'hffffff:color;
  end
 end
endmodule
