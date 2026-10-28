`timescale 1ns/1ps
module tb_pwm;
 reg clk=0;always #5 clk=~clk;
 reg rst=0;
 wire pwm;wire [3:0] status;wire signed [9:0] ex;wire [20:0] pulse;
 triangle_tracking_top dut(clk,rst,1'b0,288'd0,8'd0,clk,rst,1'b1,pwm,status,ex,pulse,);
 integer n=0,highs=0,periods=0;
 initial begin #21;rst=1;
 // Change requested width inside first pulse; current pulse must stay 140160.
 repeat(10000) @(negedge clk);
 force dut.pulse_o=21'd153600;
 end
 always @(negedge clk) if(rst) begin
  if(pwm) highs=highs+1;
  n=n+1;
  if(n==1920000) begin
   if(highs!=(periods==0?140160:153600)) begin
    $display("FAIL PWM period=%0d high=%0d",periods,highs);$fatal;
   end
   periods=periods+1;n=0;highs=0;
   if(periods==3) begin $display("PASS 50Hz period=1920000 cycles, atomic widths 140160/153600 cycles");$finish;end
  end
 end
 initial begin #60000000;$fatal(1,"wave watchdog");end
endmodule
