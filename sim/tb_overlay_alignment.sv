`timescale 1ns/1ps
// End-to-end regression: horizontal alignment plus unchanged vertical edges.
// Run once without arguments and once with +VERTICAL.
module tb_overlay_alignment;
reg clk=0;always #5 clk=~clk;reg rst=0,vs=0,de=0;reg [23:0] rgb=0;
wire ovs,ode;wire [23:0] out;
image_pipeline_top #(.IMG_W(64),.IMG_H(48),.EDGE_AA(1)) dut(.clk_i(clk),.rst_ni(rst),.vsync_i(vs),.de_i(de),.rgb_i(rgb),.thr_i(8'd12),.mode_i(2'd3),.view_i(2'd0),.smooth_mode_i(2'd1),.link_en_i(1'b0),.temporal_y_i(0),.temporal_de_i(0),.temporal_vs_i(0),.vsync_o(ovs),.de_o(ode),.rgb_o(out));
reg [23:0] captured[0:3071];
integer x,y,f,j,fd;integer vertical;initial begin
vertical=$test$plusargs("VERTICAL");
fd=$fopen("alignment.csv","w");repeat(3) @(negedge clk);rst=1;
for(f=0;f<2;f=f+1) begin
vs=1;repeat(4) @(negedge clk);vs=0;
for(y=0;y<48;y=y+1) begin
for(x=0;x<64;x=x+1) begin de=1;rgb=(vertical?(x>=44):(y>=20))?24'hffffff:0;@(negedge clk);end
de=0;rgb=0;repeat(16) @(negedge clk);
end
repeat(30) @(negedge clk);
end
$fclose(fd);
if(vertical) begin
 if(captured[30*64+42]!==0 || captured[30*64+43]!==24'hffff00 ||
    captured[30*64+44]!==24'hffff00 || captured[30*64+45]!==24'hffffff)
  $fatal(1,"vertical edge alignment failed");
end else begin
 if(captured[24*64+16]!==0 || captured[25*64+16]!==24'hffffff ||
    captured[23*64+48]!==0 || captured[24*64+48]!==24'hffff00 ||
    captured[25*64+48]!==24'hffff00 || captured[26*64+48]!==24'hffffff)
  $fatal(1,"horizontal edge alignment failed");
end
$display("ALIGNMENT PASS vertical=%0d",vertical);$finish;end
integer row=0,col=0;reg od=0,ov=0;
always @(posedge clk) begin #1;
if(ovs&&!ov)begin row=0;col=0;end
if(ode)begin if(f==1 && row<48 && col<64) captured[row*64+col]=out; $fwrite(fd,"%0d,%0d,%0d,%h\n",f,row,col,out);col=col+1;end
if(od&&!ode)begin row=row+1;col=0;end
od=ode;ov=ovs;
end
endmodule
