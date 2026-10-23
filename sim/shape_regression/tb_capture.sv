`timescale 1ns/1ps
module tb_capture;
reg clk=0;always #5 clk=~clk;
reg rst=0,vs=0,de=0,edge_bit=0,busy=0;wire ready,data;reg [1:0] addr=0;integer ready_count=0,x,y,k;
shape_frame_capture #(.IMG_W(16),.IMG_H(16)) dut(clk,rst,vs,de,edge_bit,busy,ready,addr,data);
always @(posedge clk) if(ready) ready_count=ready_count+1;
task frame(input integer pattern);begin
 @(negedge clk);vs=1;@(negedge clk);vs=0;
 for(y=0;y<16;y=y+1) begin
  for(x=0;x<16;x=x+1) begin
   @(negedge clk);de=1;edge_bit=pattern&&((y==0&&x<4)||(y==0&&x>=8&&x<11)||(x==0&&y>=8));
  end
  @(negedge clk);de=0;edge_bit=0;
 end
 repeat(5) @(negedge clk);
end endtask
task check(input [3:0] expected);begin
 for(k=0;k<4;k=k+1) begin
  addr=k;repeat(2) @(negedge clk);
  if(data!==expected[k]) $fatal(1,"capture cell %0d got %b",k,data);
 end
end endtask
initial begin
 repeat(3) @(negedge clk);rst=1;
 frame(1);check(4'b0101);if(ready_count!=1) $fatal(1,"ready pulse");
 frame(0);check(0);if(ready_count!=2) $fatal(1,"second ready");
 busy=1;frame(1);check(0);if(ready_count!=2) $fatal(1,"busy overwrote frame");
 $display("PASS capture threshold, frame replacement, busy exclusion");$finish;
end
initial begin #100000;$fatal(1,"timeout");end
endmodule
