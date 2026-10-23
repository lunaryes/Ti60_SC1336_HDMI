`timescale 1ns/1ps
module tb_tracker;
reg clk=0;always #5 clk=~clk;
reg rst=0,vs=0,done=0;reg [71:0] objects=0;reg [1:0] valid=0;wire [71:0] result;wire [1:0] accepted;
recognition_object_tracker #(.MAX_OBJECTS(2),.TYPE_BITS(4),.X_BITS(8),.Y_BITS(8),.CONFIRM(3)) dut(clk,rst,vs,done,1'b0,1'b1,objects,valid,result,accepted);
task analysis_frame(input integer cls);begin
 @(negedge clk);valid=(cls!=0)?1:0;objects={36'd0,cls[3:0],8'd20,8'd40,8'd20,8'd40};done=1;
 @(negedge clk);done=0;repeat(40) @(negedge clk);vs=1;@(negedge clk);vs=0;@(negedge clk);
end endtask
initial begin
 repeat(3) @(negedge clk);rst=1;
 analysis_frame(2);if(accepted) $fatal(1,"one hit");
 analysis_frame(2);if(accepted) $fatal(1,"two hits");
 analysis_frame(0);if(accepted) $fatal(1,"gap");
 analysis_frame(2);analysis_frame(2);if(accepted) $fatal(1,"gap retained hits");
 analysis_frame(2);if(!accepted) $fatal(1,"third hit not confirmed");
 analysis_frame(4);if(accepted) $fatal(1,"class change not cleared");
 analysis_frame(4);analysis_frame(4);if(!accepted) $fatal(1,"new class not confirmed");
 analysis_frame(0);analysis_frame(0);if(accepted) $fatal(1,"stale track");
 $display("PASS tracker confirmation, gaps, class changes, expiry");$finish;
end
initial begin #100000;$fatal(1,"timeout");end
endmodule
