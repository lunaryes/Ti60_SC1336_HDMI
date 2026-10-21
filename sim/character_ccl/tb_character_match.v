`timescale 1ns/1ps
module tb_character_match;
 reg clk=0,rst=0,start=0;always #5 clk=~clk;
 reg [255:0] glyph;integer w,h,t;wire done,ok;wire [7:0] ascii;
 wire [5:0] id;wire [8:0] best,second;wire [255:0] bitmap;
 character_matcher16 dut(clk,rst,start,1'b1,glyph,w[10:0],h[9:0],done,ok,id,ascii,best,second,bitmap);
 initial begin
  if(!$value$plusargs("GLYPH=%h",glyph)) $fatal(1,"GLYPH required");
  if(!$value$plusargs("W=%d",w)) w=16;
  if(!$value$plusargs("H=%d",h)) h=16;
  repeat(3) @(negedge clk);rst=1;start=1;@(negedge clk);start=0;t=0;
  while(!done&&t<30000) begin @(negedge clk);t=t+1;end
  if(!done) $fatal(1,"matcher timeout");
  $display("MATCH winner=%c actual=%c ok=%b best=%0d second=%0d unmatched=%0d raw=%0d",
    dut.best_ascii,ascii,ok,best,second,dut.best_unmatched,dut.best_error);$finish;
 end
endmodule
