`timescale 1ns/1ps
module tb_character_io;
 reg clk=0,rst=0,vs=0,de=0,busy=0;reg [7:0] gray=220;always #5 clk=~clk;
 wire ready,contrast;wire [7:0] threshold;reg [7:0] addr=0;wire [1:0] data;
 integer x,y,k,ready_count=0;
 character_frame_capture #(.IMG_W(64),.IMG_H(64)) cap(clk,rst,vs,de,gray,busy,ready,threshold,contrast,addr,data);
 always @(posedge clk) if(ready) ready_count<=ready_count+1;
 task frame;
 begin
  @(negedge clk);vs=1;repeat(3) @(negedge clk);vs=0;
  repeat(3) @(negedge clk);
  for(y=0;y<64;y=y+1) begin
   for(x=0;x<64;x=x+1) begin
    de=1;gray=((x>=8&&x<24&&y>=8&&y<40)||(x>=40&&x<42&&y>=12&&y<16))?20:220;@(negedge clk);
   end
   de=0;repeat(4) @(negedge clk);
  end
  repeat(30) @(negedge clk);
 end endtask
 reg [10:0] ox=0;reg [9:0] oy=0;reg en=0,ode=1;wire [23:0] rgb;
 reg [167:0] objects=0;reg [3:0] valid=0;
 character_overlay_cn overlay(ox,oy,ode,en,24'h123456,objects,valid,1'b0,6'd0,rgb);
 initial begin
  repeat(3) @(negedge clk);rst=1;frame();frame();
  if(!contrast||threshold!=120||ready_count!=1) $fatal(1,"adaptive threshold / warmup mismatch %d %d",threshold,ready_count);
  addr=34;repeat(3) @(negedge clk);if(data!=1) $fatal(1,"dark pooling");
  addr=0;repeat(3) @(negedge clk);if(data!=2) $fatal(1,"light pooling");
  addr=58;repeat(3) @(negedge clk);if(data!=3) $fatal(1,"half-coverage pooling %d",data);
  busy=1;frame();if(ready_count!=1) $fatal(1,"busy frame overwritten");
  objects[41:0]={8'h41,9'd100,9'd130,8'd50,8'd100};valid=1;
  for(y=0;y<720;y=y+11) for(x=0;x<1280;x=x+13) begin
   ox=x;oy=y;en=0;#1;if(rgb!==24'h123456) $fatal(1,"disabled passthrough");
   en=1;ode=0;#1;if(rgb!==24'h123456) $fatal(1,"blanking passthrough");ode=1;
   if((x<400||x>523||y<166||y>403)&&(x<24||x>=120||y<128||y>=160)) begin
    #1;if(rgb!==24'h123456) $fatal(1,"unannotated pixel changed");
   end
  end
  $display("PASS adaptive threshold, polarity pooling, busy skip, Chinese-overlay passthrough");$finish;
 end
endmodule
