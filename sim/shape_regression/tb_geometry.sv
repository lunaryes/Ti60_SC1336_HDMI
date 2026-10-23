`timescale 1ns/1ps
module tb_geometry;
reg clk=0; always #5 clk=~clk;
reg rst=0,start=0; wire busy,done;wire [13:0] addr;reg data;
reg mem[0:14399];wire [287:0] objects;wire [7:0] valid;wire [15:0] rejects;wire [23:0] cycles;
reg [1023:0] file;integer expected,expected_count,i,n,cls;
shape_geometry_engine dut(clk,rst,start,busy,done,addr,data,objects,valid,rejects,cycles);
always @(posedge clk) data<=mem[addr];
`ifdef DEBUG_FEATURES
always @(posedge clk) if(dut.state==23) $display("features area=%0d bbox=%0d,%0d q=%0d,%0d,%0d,%0d widths=%0d,%0d,%0d,%0d,%0d endpoints=%0d",dut.area,dut.fw,dut.fh,dut.q0,dut.q1,dut.q2,dut.q3,dut.wq1,dut.wq2,dut.wq3,dut.wq4,dut.wq5,dut.endpoint_count);
`endif
initial begin
 if(!$value$plusargs("MEM=%s",file)) $fatal(1,"MEM missing");
 if(!$value$plusargs("EXPECT=%d",expected)) $fatal(1,"EXPECT missing");
 expected_count=(expected==0)?0:1;
 if($value$plusargs("COUNT=%d",expected_count)) begin end
 $readmemh(file,mem);repeat(3) @(negedge clk);rst=1;@(negedge clk);start=1;@(negedge clk);start=0;
 wait(done);#1;n=0;cls=0;
 for(i=0;i<8;i=i+1) if(valid[i]) begin n=n+1;cls=objects[i*36+32+:4];if(cls!=expected) $fatal(1,"wrong class in slot %0d",i);end
 $display("count=%0d class=%0d expected=%0d cycles=%0d",n,cls,expected,cycles);
 if(n!=expected_count) $fatal(1,"classification mismatch");
 $finish;
end
initial begin #41000000;$fatal(1,"watchdog");end
endmodule
