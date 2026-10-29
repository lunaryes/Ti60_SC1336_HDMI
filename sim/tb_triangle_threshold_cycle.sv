`timescale 1ns/1ps
module tb_triangle_threshold_cycle;
reg clk=0;always #5 clk=~clk;reg rst=0,k1=1,k2=1,tv=0;reg [7:0] t=12;
wire [7:0] high,low;wire [1:0] smooth,press;
canny_threshold_key_ctrl #(.CNT_MAX(3)) dut(clk,rst,{1'b1,k1},tv,t,1'b0,2'd0,high,low,smooth,press);
task check(input integer expected);begin if(high!=expected||low!=(expected>>1)||smooth!=0)$fatal(1,"threshold=%0d expected=%0d",high,expected);end endtask
task push;begin k1=0;repeat(16)@(negedge clk);k1=1;repeat(16)@(negedge clk);end endtask
initial begin
repeat(3)@(negedge clk);rst=1;@(negedge clk);check(12);
push;check(16);push;check(20);push;check(24);push;check(28);push;check(32);push;check(4);push;check(8);
k1=0;repeat(60)@(negedge clk);check(12);k1=1;repeat(16)@(negedge clk);
k2=0;repeat(40)@(negedge clk);check(12);k2=1;
tv=1;t=29;@(negedge clk);tv=0;@(negedge clk);check(29);push;check(32);push;check(4);
$display("PASS K1 cycle, hold, UART off-grid boundary, K2 excluded");$finish;
end
endmodule
