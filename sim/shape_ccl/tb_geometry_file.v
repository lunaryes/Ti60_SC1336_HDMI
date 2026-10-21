`timescale 1ns/1ps
module tb_geometry_file;
 localparam W=160,H=90,AW=$clog2(W*H);
 reg clk=0,rst_n=0,start=0;always #5 clk=~clk;
 wire busy,done;wire [AW-1:0] addr;reg data;
 reg mask[0:W*H-1];wire [143:0] objects;wire [3:0] valid;
 wire [15:0] rejected;wire [23:0] cycles;
 shape_geometry_engine dut(clk,rst_n,start,busy,done,addr,data,objects,valid,rejected,cycles);
 always @(posedge clk) data<=mask[addr];
 reg [2047:0] filename;integer t,k;
 initial begin
  if(!$value$plusargs("FILE=%s",filename)) $fatal(1,"need FILE");
  $readmemh(filename,mask);
  repeat(3) @(negedge clk);rst_n=1;start=1;@(negedge clk);start=0;t=0;
  while(!done && t<2100000) begin @(negedge clk);t=t+1;end
  if(!done) $fatal(1,"timeout");
  $display("RESULT valid=%h objects=%h rejected=%0d cycles=%0d",valid,objects,rejected,cycles);
  $finish;
 end
endmodule
