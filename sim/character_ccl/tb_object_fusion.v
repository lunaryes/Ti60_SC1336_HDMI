`timescale 1ns/1ps
module tb_object_fusion;
 reg [143:0] geo=0;reg [3:0] gv=3;reg [167:0] chars=0;reg [3:0] cv=1;wire [3:0] out;
 recognition_object_fusion dut(geo,gv,chars,cv,out);
 initial begin
  // A at 400..603 x 200..403. Internal triangle suppressed, separate shape retained.
  chars[41:0]={8'h41,9'd100,9'd150,8'd50,8'd100};
  geo[35:0]={4'd1,8'd55,8'd65,8'd30,8'd40};
  geo[71:36]={4'd3,8'd90,8'd110,8'd30,8'd50};
  #1;if(out!=2) $fatal(1,"internal shape not suppressed / independent shape lost");
  cv=0;#1;if(out!=3) $fatal(1,"no character must preserve all shapes");
  cv=1;chars[41:34]="O";#1;if(out!=3) $fatal(1,"ambiguous O must not suppress shape");
  $display("PASS hierarchical suppression and independent/ambiguous shape preservation");$finish;
 end
endmodule
