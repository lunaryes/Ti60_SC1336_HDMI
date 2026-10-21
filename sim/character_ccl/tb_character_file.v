`timescale 1ns/1ps
module tb_character_file;
 localparam W=320,H=180,AW=$clog2(W*H);
 reg clk=0,rst=0,start=0;always #5 clk=~clk;
 wire busy,done;wire [AW-1:0] addr;reg [3:0] data;
 reg [3:0] mask[0:W*H-1];wire [167:0] objects;wire [3:0] valid;wire overflow;wire [23:0] cycles;
 character_ccl_engine dut(clk,rst,start,busy,done,addr,data,objects,valid,overflow,cycles);
 always @(posedge clk) data<=mask[addr];
`ifdef CHARACTER_DEBUG
 always @(posedge clk) if(dut.match_done) $display("MATCH winner=%c ascii=%c ok=%b best=%0d second=%0d unmatched=%0d raw=%0d box=%0dx%0d glyph=%h",dut.u_matcher.best_ascii,dut.matched_ascii,dut.recognized,dut.best,dut.second,dut.u_matcher.best_unmatched,dut.u_matcher.best_error,dut.width,dut.height,dut.glyph);
`endif
 reg [2047:0] filename;integer t,k;
 initial begin
  if(!$value$plusargs("FILE=%s",filename)) $fatal(1,"FILE required");$readmemh(filename,mask);
  repeat(3) @(negedge clk);rst=1;start=1;@(negedge clk);start=0;t=0;
  while(!done&&t<12100000) begin @(negedge clk);t=t+1;end
  if(!done) $fatal(1,"timeout");
  $write("RESULT valid=%h overflow=%b cycles=%0d ascii=",valid,overflow,cycles);
  for(k=0;k<4;k=k+1) if(valid[k]) $write("%c",objects[k*42+34+:8]);
  $display("");$finish;
 end
endmodule
