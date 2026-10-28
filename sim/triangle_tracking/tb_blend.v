`timescale 1ns/1ps
module tb_blend;
reg clk=0;always #5 clk=~clk;
wire [23:0] rgb;
image_mix #(.IMG_W(2)) dut(clk,1'b1,1'b0,1'b1,24'd0,24'd0,24'd0,2'd3,,,rgb);
integer v,alpha,expected,got;
initial begin
for(v=0;v<256;v=v+1) for(alpha=0;alpha<256;alpha=alpha+1) begin
 expected=(v*alpha+127)/255;
 got=dut.scale_coverage(v,alpha);
 if(got!=expected) $fatal(1,"blend arithmetic mismatch");
end
$display("PASS coverage rounding: all 65536 value/alpha combinations");$finish;end
endmodule
