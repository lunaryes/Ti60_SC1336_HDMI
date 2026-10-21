`timescale 1ns/1ps
module tb_character_end_to_end;
 reg clk=0,rst=0,vs=0,de=0;reg [7:0] gray=255;always #5 clk=~clk;
 wire [167:0] objects;wire [3:0] valid;wire busy,overflow;wire [7:0] threshold;
 character_detection_top dut(clk,rst,vs,de,gray,objects,valid,busy,overflow,threshold);
 reg [7:0] image[0:1280*720-1];reg [2047:0] filename;integer x,y,t;
 task frame;
 begin
  @(negedge clk);vs=1;repeat(4) @(negedge clk);vs=0;
  repeat(4) @(negedge clk);
  for(y=0;y<720;y=y+1) begin
   for(x=0;x<1280;x=x+1) begin de=1;gray=image[y*1280+x];@(negedge clk);end
   de=0;gray=255;repeat(4) @(negedge clk);
  end
  repeat(50) @(negedge clk);
  t=0;while(busy&&t<6100000) begin @(negedge clk);t=t+1;end
  if(busy||overflow) $fatal(1,"engine timeout/overflow");
 end endtask
 initial begin
  if(!$value$plusargs("FILE=%s",filename)) $fatal(1,"FILE required");$readmemh(filename,image);
  repeat(3) @(negedge clk);rst=1;
  frame();if(!dut.contrast) $fatal(1,"small glyph omitted by contrast calibration");
  frame();frame();
  @(negedge clk);vs=1;repeat(3) @(negedge clk);
  if(valid!=1||objects[41:34]!="4") $fatal(1,"end-to-end missing digit 4: valid=%b ascii=%h",valid,objects[41:34]);
  $display("PASS full 1280x720 grayscale -> adaptive threshold -> CCL -> normalization -> matcher -> stable result; threshold=%0d",threshold);$finish;
 end
endmodule
