`timescale 1ns/1ps
module tb_geometry;
 localparam W=96,H=72,AW=$clog2(W*H);
 reg clk=0,rst_n=0,start=0; always #5 clk=~clk;
 wire busy,done;wire [AW-1:0] addr;reg data;
 reg mask[0:W*H-1];wire [287:0] objects;wire [7:0] valid;
 wire [15:0] rejected;wire [23:0] cycles;
 shape_geometry_engine #(.W(W),.H(H)) dut(clk,rst_n,start,busy,done,addr,data,objects,valid,rejected,cycles);
 always @(posedge clk) data<=mask[addr];
 integer x,y,k,t,case_id,expected;
 task run_case;
 input integer id,want;
 begin
  @(negedge clk);rst_n=0;repeat(3) @(negedge clk);rst_n=1;start=1;
  @(negedge clk);start=0;t=0;
  while(!done && t<4100000) begin @(negedge clk);t=t+1;end
  $display("CASE %0d valid=%b class=%0d reject=%0d cycles=%0d vertices=%0d area2=%0d perimeter10=%0d",id,valid,objects[35:32],rejected,cycles,dut.vertices,dut.area2,dut.perimeter10);
  if(!done || (want==0 ? valid!=0 : (!valid[0] || objects[35:32]!=want))) $fatal(1,"classification mismatch case %0d",id);
 end endtask
 initial begin
  for(k=0;k<W*H;k=k+1) mask[k]=0;
  run_case(0,0);
  for(y=15;y<=55;y=y+1) for(x=20;x<=60;x=x+1)
    if(x==20||x==60||y==15||y==55) mask[y*W+x]=1;
  run_case(1,2);
  for(k=0;k<W*H;k=k+1) mask[k]=0;
  for(y=15;y<=55;y=y+1) for(x=20;x<=70;x=x+1) mask[y*W+x]=1;
  run_case(2,3);
  for(k=0;k<W*H;k=k+1) mask[k]=0;
  for(y=15;y<=55;y=y+1) for(x=20;x<=60;x=x+1)
    if(y==55 || x==40-(y-15)/2 || x==40+(y-15)/2) mask[y*W+x]=1;
  run_case(3,1);
  $display("PASS: empty, hollow square, solid rectangle, hollow triangle");$finish;
 end
endmodule
