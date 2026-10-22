`timescale 1ns/1ps
module tb_shape_io;
 reg clk=0,rst=0,vs=0,de=0,edge_bit=0,busy=0;always #5 clk=~clk;
 wire ready,rd;reg [3:0] ra=0;integer x,y,k,ready_count=0;
 shape_frame_capture #(.IMG_W(32),.IMG_H(32)) cap(clk,rst,vs,de,edge_bit,busy,ready,ra,rd);
 always @(posedge clk) if(ready) ready_count<=ready_count+1;
 reg [10:0] ox=0;reg [9:0] oy=0;reg ode=1,en=0;
 reg [287:0] boxes=0;reg [7:0] valid=0;
 wire [23:0] rgb;
 shape_geometry_overlay ov(ox,oy,ode,en,24'h123456,boxes,valid,rgb);
 task frame;
 begin
  @(negedge clk);vs=1;repeat(3) @(negedge clk);vs=0;
  repeat(3) @(negedge clk);
  for(y=0;y<32;y=y+1) begin
   for(x=0;x<32;x=x+1) begin
    de=1;edge_bit=(x==9&&y==10)||(x==31&&y==31);@(negedge clk);
   end
   de=0;edge_bit=0;repeat(4) @(negedge clk);
  end
  repeat(5) @(negedge clk);
 end endtask
 initial begin
  repeat(3) @(negedge clk);rst=1;frame();
  if(ready_count!=1) $fatal(1,"frame completion pulse");
  for(k=0;k<16;k=k+1) begin
   ra=k;repeat(3) @(negedge clk);
   if(rd!==(k==5||k==15)) $fatal(1,"pool address %0d got %b",k,rd);
  end
  busy=1;frame();if(ready_count!=1) $fatal(1,"captured while engine busy");
  boxes[35:0]={4'd1,8'd20,8'd70,8'd15,8'd65};valid=1;
  for(y=0;y<720;y=y+13) for(x=0;x<1280;x=x+11) begin
   ox=x;oy=y;en=0;#1;if(rgb!==24'h123456) $fatal(1,"disabled passthrough");
   en=1;ode=0;#1;if(rgb!==24'h123456) $fatal(1,"blanking passthrough");ode=1;
   if((x<160||x>567||y<120||y>527)&&!(x>=160&&x<256&&y>=86&&y<118)) begin
    #1;if(rgb!==24'h123456) $fatal(1,"unannotated passthrough");
   end
  end
  $display("PASS frame pool, last-pixel write, busy skip, overlay passthrough");$finish;
 end
endmodule
