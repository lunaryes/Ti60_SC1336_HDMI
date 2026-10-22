`timescale 1ns/1ps
module tb_ddr_startup;
 reg clk=0,rst=1,unlocked=0,awvalid=1,arvalid=0;
 always #5 clk=~clk;
 wire awready,arready,valid,kind;wire [31:0] addr;wire [7:0] len;wire [3:0] id;
 integer accepted=0,acknowledged=0;
 AXI4_AWARMux bridge(.aclk_i(clk),.arst_i(rst),
  .awid_i(4'd0),.awaddr_i(32'h12340040),.awlen_i(8'd3),.awvalid_i(awvalid),.awready_o(awready),
  .arid_i(4'd0),.araddr_i(32'h45670080),.arlen_i(8'd7),.arvalid_i(arvalid),.arready_o(arready),
  .aid_o(id),.aaddr_o(addr),.alen_o(len),.atype_o(kind),.avalid_o(valid),
  .aready_i(1'b1 && unlocked));
 always @(posedge clk)if(!rst)begin
  if(!unlocked && (awready||arready))$fatal(1,"command acknowledged before DDR calibration");
  if(valid&&unlocked)begin
   accepted<=accepted+1;
   if(kind && (addr!=32'h12340040||len!=3))$fatal(1,"bad write command");
   if(!kind && (addr!=32'h45670080||len!=7))$fatal(1,"bad read command");
  end
  if(awready||arready)begin
   if(acknowledged>=accepted)$fatal(1,"response without DDR acceptance");
   acknowledged<=acknowledged+1;
  end
 end
 initial begin
  repeat(4)@(negedge clk);rst=0;
  repeat(100)@(negedge clk);
  if(accepted!=0||acknowledged!=0)$fatal(1,"calibration gate failed");
  unlocked=1;wait(awready);@(negedge clk);awvalid=0;arvalid=1;
  wait(arready);@(negedge clk);arvalid=0;
  repeat(10)@(negedge clk);
  if(accepted!=2||acknowledged!=2)$fatal(1,"command count mismatch");
  $display("DDR_STARTUP_PASS: no false READY during calibration, read/write restart");$finish;
 end
 initial begin #10000;$fatal(1,"timeout");end
endmodule
