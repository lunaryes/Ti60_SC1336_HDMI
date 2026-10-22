`timescale 1ns/1ps
module tb_character_stability;
 reg clk=0,rst=0,vs=0;always #5 clk=~clk;
 wire [335:0] objects;wire [7:0] valid;wire busy,overflow;wire [7:0] threshold;
 reg done=0,over=0;reg [335:0] raw_objects=0;reg [7:0] raw_valid=0;
 character_detection_top dut(clk,rst,vs,1'b0,8'd255,objects,valid,busy,overflow,threshold,);
 task result;begin @(negedge clk);done=1;@(negedge clk);done=0;repeat(250) @(negedge clk);end endtask
 task boundary;begin @(negedge clk);vs=1;@(negedge clk);vs=0;end endtask
 integer i;reg [8:0] test_x;
 initial begin
  force dut.done=done;force dut.raw_objects=raw_objects;force dut.raw_valid=raw_valid;
  force dut.contrast=1'b1;force dut.overflow_o=over;
  repeat(3) @(negedge clk);rst=1;
  raw_objects[41:0]={8'h41,9'd80,9'd120,8'd30,8'd80};raw_valid=1;
  result();boundary();if(valid!=0) $fatal(1,"one observation must not publish");
  result();boundary();if(valid!=0) $fatal(1,"two observations must not publish");
  result();if(valid!=0) $fatal(1,"non-atomic update");
  boundary();if(valid!=1) $fatal(1,"three confirmations absent");
  // CCL result moves from slot 0 to slot 2: track must remain the same.
  raw_objects[125:84]=raw_objects[41:0];raw_valid=4;
  result();boundary();if(valid!=1||objects[41:34]!="A") $fatal(1,"slot permutation broke track");
  raw_valid=0;result();boundary();if(valid!=1) $fatal(1,"single miss should hold");
  result();boundary();if(valid!=0) $fatal(1,"second miss must clear");
  raw_valid=4;result();result();result();boundary();if(valid!=1) $fatal(1,"reacquisition");
  raw_objects[125:118]="B";result();boundary();if(valid!=0) $fatal(1,"changed class stale label");
  result();result();boundary();if(valid!=1||objects[41:34]!="B") $fatal(1,"new class confirmation");
  over=1;result();boundary();if(valid!=0) $fatal(1,"overflow must reject");
  over=0;result();result();result();boundary();if(valid!=1) $fatal(1,"recovery failed");
  for(i=0;i<26;i=i+1) boundary();if(valid!=0) $fatal(1,"expired label");
  raw_objects=0;raw_valid=8'hff;
  for(i=0;i<8;i=i+1) begin test_x=20+i*35;raw_objects[i*42+:42]={8'h34,test_x,test_x+9'd15,8'd20,8'd50};end
  result();result();result();boundary();if(valid!=8'hff) $fatal(1,"eight tracks did not confirm");
  for(i=0;i<8;i=i+1) begin test_x=20+(7-i)*35;raw_objects[i*42+:42]={8'h34,test_x,test_x+9'd15,8'd20,8'd50};end
  result();boundary();if(valid!=8'hff) $fatal(1,"eight-track permutation lost targets");
  $display("PASS position association, slot permutation, three confirmations, one-miss hold, expiry and overflow");$finish;
 end
endmodule
