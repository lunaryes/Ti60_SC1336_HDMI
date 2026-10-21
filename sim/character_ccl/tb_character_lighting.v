`timescale 1ns/1ps
module tb_character_lighting;
 reg clk=0,rst=0,vs=0,de=0;reg [7:0] gray=0;always #5 clk=~clk;
 reg [10:0] addr=0;wire [3:0] data;wire ready,contrast;wire [7:0] threshold;
 character_frame_capture #(.IMG_W(512),.IMG_H(64)) dut(clk,rst,vs,de,gray,1'b0,ready,threshold,contrast,addr,data);
 integer x,y;
 task frame;
 begin
  @(negedge clk);vs=1;repeat(3) @(negedge clk);vs=0;repeat(3) @(negedge clk);
  for(y=0;y<64;y=y+1) begin
   for(x=0;x<512;x=x+1) begin
    de=1;gray=x<256?220:100;
    if(y>=8&&y<48&&((x>=8&&x<40)||(x>=264&&x<296))) gray=x<256?120:20;
    @(negedge clk);
   end
   de=0;repeat(4) @(negedge clk);
  end
  repeat(30) @(negedge clk);
 end endtask
 initial begin
  repeat(3) @(negedge clk);rst=1;frame();frame();
  addr=2*128+2;repeat(3) @(negedge clk);if(data!=5) $fatal(1,"bright-side gray ink missing");
  addr=2*128+66;repeat(3) @(negedge clk);if(data!=5) $fatal(1,"shadow-side ink missing");
  addr=2*128+80;repeat(3) @(negedge clk);if(data!=10) $fatal(1,"shadow background became ink");
  $display("PASS local thresholds distinguish both lit and shadowed foreground/background");$finish;
 end
endmodule
