`timescale 1ns/1ps
module tb_object_fusion;
 reg clk=0,rst=0;always #5 clk=~clk;
 reg [287:0] geo=0;reg [7:0] gv=3;reg [335:0] chars=0;reg [7:0] cv=1;wire [7:0] out;
 recognition_object_fusion dut(geo,gv,chars,cv,out,clk,rst);
 initial begin
  // A at 400..603 x 200..403. Internal triangle suppressed, separate shape retained.
  chars[41:0]={8'h41,9'd100,9'd150,8'd50,8'd100};
  geo[35:0]={4'd1,8'd55,8'd65,8'd30,8'd40};
  geo[71:36]={4'd3,8'd90,8'd110,8'd30,8'd50};
  repeat(3) @(negedge clk);rst=1;repeat(70) @(negedge clk);
  if(out!=2) $fatal(1,"internal shape not suppressed / independent shape lost");
  // New descriptors must not inherit the old suppressed slot's mask.
  geo[35:0]={4'd2,8'd100,8'd120,8'd55,8'd75};
  #1;if(out!=3) $fatal(1,"stale mask hid changed shape");
  repeat(70) @(negedge clk);if(out!=3) $fatal(1,"changed independent shape lost");
  cv=0;repeat(70) @(negedge clk);if(out!=3) $fatal(1,"no character must preserve all shapes");
  cv=1;chars[41:34]="O";repeat(70) @(negedge clk);if(out!=3) $fatal(1,"ambiguous O must not suppress shape");
  gv=8'hff;cv=8'h80;chars[7*42+:42]={8'h41,9'd100,9'd150,8'd50,8'd100};
  geo[7*36+:36]={4'd1,8'd55,8'd65,8'd30,8'd40};
  repeat(70) @(negedge clk);if(out[7]) $fatal(1,"eighth slot was not fused");
  $display("PASS hierarchical suppression and independent/ambiguous shape preservation");$finish;
 end
endmodule
