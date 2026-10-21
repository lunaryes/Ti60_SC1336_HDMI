`timescale 1ns/1ps
module tb_character_stability;
 reg clk=0,rst=0,vs=0;always #5 clk=~clk;
 wire [167:0] objects;wire [3:0] valid;wire busy,overflow;wire [7:0] threshold;
 reg done=0,over=0;reg [167:0] raw_objects=0;reg [3:0] raw_valid=0;
 character_detection_top dut(clk,rst,vs,1'b0,8'd255,objects,valid,busy,overflow,threshold);
 task result;
 begin @(negedge clk);done=1;@(negedge clk);done=0;end
 endtask
 task boundary;
 begin @(negedge clk);vs=1;@(negedge clk);vs=0;end
 endtask
 integer i;
 initial begin
  force dut.done=done;force dut.raw_objects=raw_objects;force dut.raw_valid=raw_valid;
  force dut.contrast=1'b1;force dut.overflow_o=over;
  repeat(3) @(negedge clk);rst=1;
  raw_objects[41:0]={8'h41,9'd80,9'd120,8'd30,8'd80};raw_valid=1;
  result();boundary();if(valid!=0) $fatal(1,"single confirmation");
  result();if(valid!=0) $fatal(1,"non-atomic update");
  boundary();if(valid!=1) $fatal(1,"stable result absent");
  raw_objects[41:34]="B";result();boundary();if(valid!=0) $fatal(1,"old label not cleared after change");
  result();boundary();if(valid!=1) $fatal(1,"new stable label absent");
  over=1;result();boundary();if(valid!=0) $fatal(1,"overflow must reject");
  over=0;result();result();boundary();if(valid!=1) $fatal(1,"recovery failed");
  for(i=0;i<14;i=i+1) boundary();if(valid!=0) $fatal(1,"expired label");
  $display("PASS two confirmations, frame-atomic update, changed-class clear, overflow rejection and stale timeout");$finish;
 end
endmodule
