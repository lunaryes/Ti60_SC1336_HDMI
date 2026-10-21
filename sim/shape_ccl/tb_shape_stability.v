`timescale 1ns/1ps
module tb_shape_stability;
 reg clk=0,rst=0,vs=0;always #5 clk=~clk;
 wire [143:0] objects;wire [3:0] valid;wire busy;
 wire [15:0] rejected;wire [23:0] cycles;
 reg done=0;reg [143:0] raw_objects=0;reg [3:0] raw_valid=0;
 shape_geometry_top dut(clk,rst,vs,1'b0,1'b0,objects,valid,busy,rejected,cycles);
 task result;
 begin @(negedge clk);done=1;@(negedge clk);done=0;end
 endtask
 task boundary;
 begin @(negedge clk);vs=1;@(negedge clk);vs=0;end
 endtask
 initial begin
  force dut.done=done;force dut.raw_objects=raw_objects;force dut.raw_valid=raw_valid;
  repeat(3) @(negedge clk);rst=1;
  raw_objects[35:0]={4'd1,8'd20,8'd80,8'd15,8'd70};raw_valid=1;
  result();boundary();if(valid!=0) $fatal(1,"single-frame label was not rejected");
  result();if(valid!=0) $fatal(1,"label changed during active frame");
  boundary();if(valid!=1||objects!=raw_objects) $fatal(1,"stable result missing");
  raw_valid=0;result();if(valid!=1) $fatal(1,"non-atomic clear");
  boundary();if(valid!=0) $fatal(1,"stale result retained");
  $display("PASS two-result confirmation, atomic publication, empty-result clear");$finish;
 end
endmodule
