`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,en=1,tick=0,sample=0,found=0;
 reg signed [9:0] ex=0;
 wire [20:0] pos,speed,inv;wire [3:0] ps,ss;
 triangle_follow_ctrl #(.CLK_HZ(1000)) a(clk,rst,en,tick,sample,found,ex,pos,ps);
 triangle_follow_ctrl #(.CLK_HZ(1000),.SPEED_MODE(1)) b(clk,rst,en,tick,sample,found,ex,speed,ss);
 triangle_follow_ctrl #(.CLK_HZ(1000),.DIR_INVERT(1)) c(clk,rst,en,tick,sample,found,ex,inv,);
 task frame;input f;input signed [9:0] e;begin
 @(negedge clk);found=f;ex=e;sample=1;
 @(negedge clk);sample=0;end endtask
 task update;begin @(negedge clk);tick=1;@(negedge clk);tick=0;end endtask
 task check;input ok;input [511:0] msg;begin
 if(!ok) begin $display("FAIL %0s pos=%0d speed=%0d inv=%0d",msg,pos,speed,inv);$fatal;end end endtask
 reg [20:0] saved;
 reg vs=0;reg [287:0] objs=0;reg [7:0] valid=0;
 wire signed [9:0] dx,dy;wire selected,frameout;
 triangle_target_select sel(clk,rst,vs,objs,valid,dx,dy,selected,,,frameout);
 reg dst_clk=0;always #7 dst_clk=~dst_clk;
 wire [10:0] cdata;wire cv;
 axis_cdc #(.W(11)) cdc(clk,rst,{selected,dx},frameout,dst_clk,rst,cdata,cv);
 integer transfers=0;
 always @(posedge dst_clk) if(cv) transfers=transfers+1;
 task select_frame;begin
 @(negedge clk);vs=1;@(negedge clk);vs=0;
 repeat(25) @(negedge clk);end endtask
 initial begin
 #21;rst=1;repeat(5) @(negedge clk);
 check(pos==140160 && speed==144000,"reset neutral");
 frame(1,80);update;check(pos==140160,"require confirmation");
 frame(1,80);update;check(pos==140544 && inv==139776 && speed==144960,"right and invert");
 saved=pos;update;check(pos==saved,"no duplicate integration");
 frame(1,0);update;check(pos==saved && speed==144000 && ps[1],"center hold");
 frame(1,4);update;check(pos==saved,"hysteresis");
 frame(1,-80);update;check(pos==saved-384 && speed==143040,"left");
 saved=pos;frame(0,0);update;check(pos==saved && speed==144000,"lost hold");
 frame(1,80);frame(1,80);update;
 repeat(305) @(negedge clk);saved=pos;update;
 check(pos==saved && speed==144000 && !ps[0],"camera timeout");
 frame(1,80);frame(1,80);update;
 en=0;repeat(5) @(negedge clk);saved=pos;update;
 check(pos==saved && speed==144000,"disable");
 en=1;repeat(5) @(negedge clk);
 repeat(90) begin frame(1,150);update;end
 check(pos==165120 && ps[3],"upper limit");
 repeat(150) begin frame(1,-150);update;end
 check(pos==115200 && ps[3],"lower limit");
 // Centered bbox [70..89] => pixel center 639.5, exact error zero.
 objs[35:0]={4'd1,8'd70,8'd89,8'd35,8'd54};valid=1;
 select_frame;check(selected && dx==0 && dy==0,"bbox center reference");
 check(cdata=={1'b1,10'd0} && transfers==1,"coherent CDC exactly once");
 // Wrong class closer than triangle must not be selected.
 objs[35:0]={4'd2,8'd70,8'd89,8'd35,8'd54};
 objs[71:36]={4'd1,8'd90,8'd109,8'd35,8'd54};valid=3;
 select_frame;check(selected && dx==40,"triangle class filter");
 objs[71:36]={4'd1,8'd150,8'd170,8'd35,8'd54};
 select_frame;check(!selected,"reject outside image");
 valid=0;select_frame;check(!selected && transfers==4,"loss publishes once");
 $display("PASS triangle control, sign, hysteresis, bounds, timeout, class, center, CDC");$finish;
 end
 initial begin #2000000;$fatal(1,"watchdog");end
endmodule
