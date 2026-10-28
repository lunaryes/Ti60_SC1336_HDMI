`timescale 1ns/1ps
module tb_follow_link;
reg clk=0,pix=0;always #5 clk=~clk;always #7 pix=~pix;
reg rst=0,en=1,vs=0;reg [287:0] objects=0;reg [7:0] valid=1;
wire pwm;wire [3:0] status;wire signed [9:0] error;wire [20:0] pulse;
wire [5:0] debug;
triangle_tracking_top #(.DIR_INVERT(1)) dut(
.clk_pixel(pix),.rst_pixel_n(rst),.vsync_i(vs),.objects_i(objects),
.valid_i(valid),.clk_sys(clk),.rst_sys_n(rst),.enable_i(en),
.pwm_o(pwm),.status_o(status),.error_o(error),.pulse_o(pulse),.debug_o(debug));
task frame;input [7:0] x0,x1;begin
 @(negedge pix);objects[35:0]={4'd1,x0,x1,8'd35,8'd54};vs=1;
 @(negedge pix);vs=0;repeat(24) @(negedge pix);end endtask
task period;begin repeat(1920100) @(negedge clk);end endtask
initial begin
 #21;rst=1;
 frame(20,39);frame(20,39);period;
 if(error!=-100 || pulse!=140544 || !debug[0] || !debug[1] || !debug[4])
 $fatal(1,"left target did not propagate to PWM: ex=%0d pulse=%0d dbg=%b",error,pulse,debug);
 period;if(pulse!=140544) $fatal(1,"stale frame reused");
 frame(120,139);frame(120,139);period;
 if(error!=100 || pulse!=140160) $fatal(1,"right target failed");
 frame(70,89);frame(70,89);period;
 if(error!=0 || pulse!=140160 || !status[1]) $fatal(1,"center failed");
 en=0;frame(20,39);frame(20,39);period;
 if(pulse!=140160 || !debug[2] || debug[1]) $fatal(1,"pause gate failed");
 $display("PASS real 96MHz-count chain: triangle descriptors -> asynchronous CDC -> inverted tracking -> PWM command; center/pause/stale tested");
 $finish;end
initial begin #150000000;$fatal(1,"link watchdog");end
endmodule
