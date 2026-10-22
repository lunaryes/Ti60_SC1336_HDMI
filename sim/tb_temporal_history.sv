`timescale 1ns/1ps
module tb_temporal_history;
 reg sc=0,pc=0,rst=1,prst=1;
 always #5 sc=~sc;always #7 pc=~pc;
 reg [63:0] tuning=64'h0818080405020201;
 reg vs=0,de=0,en=1;reg [7:0] y=100;reg [15:0] id=1;
 wire ovs,ode;wire [7:0] oy;wire [31:0] status;
 wire [31:0] strong_pixels,weak_pixels,bypass_pixels;
 wire [31:0] aa,ra;wire [7:0] al,rl;wire av,ar,wv,wl,bv,rv,last;
 wire [127:0] wd;reg [127:0] rd;
 integer tick=0,wa=0,rx=0,wbeat=0,rbeat=0,bdelay=0,rleft=0;
 reg write_busy=0,read_busy=0;
 wire aready=!write_busy && bdelay==0 && tick%3!=0;
 wire wready=write_busy && tick%4!=0;
 wire rready=!read_busy && tick%5!=0;
 assign bv=bdelay==1;assign rv=read_busy && tick%3!=0;
 assign last=rleft==1;
 reg [127:0] memory[0:95];
 wire [31:0] ha,hr;wire [7:0] hal,hrl;wire hav,har,hwv,hwl,hbv,hrv,hawready,hwready,harready;
 wire [127:0] hwd;
 wire aawready,awready,abvalid,aarready,arvalid;
 reg [2:0] astate=0;reg [2:0] abeat=0;integer acompleted=0;
 wire aav=astate==0 && acompleted<64 && !rst;
 wire awv=astate==1;wire aarl=astate==3;
 temporal_axi_mux arb(.clk(sc),.rst(rst),
  .a_awaddr(32'd0),.a_awlen(8'd3),.a_awvalid(aav),.a_awready(aawready),
  .a_wdata(128'hcafe),.a_wlast(abeat==3),.a_wvalid(awv),.a_wready(awready),.a_bvalid(abvalid),
  .a_araddr(32'd0),.a_arlen(8'd3),.a_arvalid(aarl),.a_arready(aarready),.a_rvalid(arvalid),
  .b_awaddr(ha),.b_awlen(hal),.b_awvalid(hav),.b_awready(hawready),
  .b_wdata(hwd),.b_wlast(hwl),.b_wvalid(hwv),.b_wready(hwready),.b_bvalid(hbv),
  .b_araddr(hr),.b_arlen(hrl),.b_arvalid(har),.b_arready(harready),.b_rvalid(hrv),
  .awaddr(aa),.awlen(al),.awvalid(av),.awready(aready),.wdata(wd),.wlast(wl),.wvalid(wv),.wready(wready),
  .bvalid(bv),.araddr(ra),.arlen(rl),.arvalid(ar),.arready(rready),.rvalid(rv),.rlast(last));
 always @(posedge sc)if(!rst)begin
  case(astate)
   0:if(aav&&aawready)begin astate<=1;abeat<=0;end
   1:if(awready)begin if(abeat==3)astate<=2;else abeat<=abeat+1'b1;end
   2:if(abvalid)astate<=3;
   3:if(aarready)astate<=4;
   4:if(arvalid)begin
    if(rd!==128'hcafe)$fatal(1,"AXI masters mixed data");
    if(last)begin astate<=0;acompleted<=acompleted+1;end
   end
  endcase
 end
 temporal_ddr_history #(.W(32),.H(8),.BURST(4)) dut(
  .sc(sc),.pc(pc),.rst(rst),.prst(prst),.frame_vs(vs),.pixel_de(de),.pixel_y(y),
  .source_id(id),.enable_i(en),.mode_i(3'd0),.config_i({16'h7060,tuning}),
  .filtered_vs(ovs),.filtered_de(ode),.filtered_y(oy),.status_o(status),
  .strong_pixels_o(strong_pixels),.weak_pixels_o(weak_pixels),.bypass_pixels_o(bypass_pixels),
  .awaddr(ha),.awlen(hal),.awvalid(hav),.awready(hawready),.wdata(hwd),.wlast(hwl),.wvalid(hwv),.wready(hwready),
  .bvalid(hbv),.bresp(2'd0),.araddr(hr),.arlen(hrl),.arvalid(har),.arready(harready),
  .rdata(rd),.rlast(last),.rvalid(hrv),.rresp(2'd0));
 always @* rd=memory[rx+rbeat];
 always @(posedge sc)begin
  if(!rst)begin
   tick<=tick+1;
   if(bdelay>0)bdelay<=bdelay-1;
   if(av&&aready)begin wa<=(aa[24]?(aa[21]?32:0):64)+int'(aa[20:4]);wbeat<=0;write_busy<=1;end
   if(wv&&wready)begin
    memory[wa+wbeat]<=wd;
    if(wl!=(wbeat==3))$fatal(1,"bad WLAST");
    if(wl)begin write_busy<=0;bdelay<=6;end else wbeat<=wbeat+1;
   end
   if(ar&&rready)begin rx<=(ra[24]?(ra[21]?32:0):64)+int'(ra[20:4]);rbeat<=0;rleft<=rl+1;read_busy<=1;end
   if(rv)begin rbeat<=rbeat+1;rleft<=rleft-1;if(last)read_busy<=0;end
  end
 end
 integer n=0,frame=0,observed=0,new_ghost=0,old_ghost=0,ghost;reg [7:0] cached[0:255];
 integer reference_q[0:255],expected_y[0:255],next_q[0:255],current_y[0:255],prior_row[0:31];
 integer last_id=-1,model_valid=0,j,delta,dc,compensated_dc,block_total,prev_delta,res,dx,dy,step,v,choice,weight;
 reg [63:0] last_tuning=0;
 task model(input integer sid);
 begin
  dc=0;block_total=0;prev_delta=0;
  for(j=0;j<256;j=j+1)begin
   delta=current_y[j]-(reference_q[j]>>4);
   compensated_dc=dc;
   if(compensated_dc>int'(tuning[63:56]))compensated_dc=int'(tuning[63:56]);
   if(compensated_dc< -int'(tuning[63:56]))compensated_dc= -int'(tuning[63:56]);
   res=(delta>compensated_dc)?delta-compensated_dc:compensated_dc-delta;
   dx=(j%32==0)?0:((delta>prev_delta)?delta-prev_delta:prev_delta-delta);
   dy=(j<32)?0:((delta>prior_row[j%32])?delta-prior_row[j%32]:prior_row[j%32]-delta);
   choice=0;
   if(delta<=int'(tuning[55:48]) && delta>=-int'(tuning[55:48]) && res<=int'(tuning[23:16]) && dx<=int'(tuning[39:32]) && dy<=int'(tuning[39:32]))choice=2;
   else if(delta<=int'(tuning[55:48]) && delta>=-int'(tuning[55:48]) && res<=int'(tuning[31:24]) && dx<=int'(tuning[47:40]) && dy<=int'(tuning[47:40]))choice=1;
   v=current_y[j]*16;
   if(en && model_valid && tuning==last_tuning && sid==last_id)v=reference_q[j];
   else if(en && model_valid && tuning==last_tuning && sid==last_id+1)begin
    step=v-reference_q[j];
    weight=(choice==2)?int'(tuning[1:0]):((choice==1)?int'(tuning[9:8]):3);
    case(weight)
     0:step=(step<0)?-((-step+2)/4):(step+2)/4;
     1:step=(step<0)?-((-step+1)/2):(step+1)/2;
     2:step=(step<0)?-((-step*3+2)/4):(step*3+2)/4;
     default:;
    endcase
    v=reference_q[j]+step;
   end
   next_q[j]=v;expected_y[j]=(v+8)/16;if(expected_y[j]>255)expected_y[j]=255;
   prev_delta=delta;prior_row[j%32]=delta;block_total=block_total+delta;
   if(j%16==15)begin dc=block_total/16;block_total=0;end
  end
  for(j=0;j<256;j=j+1)reference_q[j]=next_q[j];
  model_valid=en;last_id=sid;last_tuning=tuning;
 end endtask
 always @(negedge pc)if(ode)begin
  if(oy!=expected_y[observed])$fatal(1,"reference mismatch frame=%d pixel=%d got=%d expected=%d",frame,observed,oy,expected_y[observed]);
  if(frame==1 && oy!=100)$fatal(1,"initialization changed input %d",oy);
  if(frame==2 && oy!=100)$fatal(1,"duplicate source recursively updated %d",oy);
  if(frame==3)begin
   if(oy<100 || oy>108)$fatal(1,"IIR did not blend %d",oy);
   cached[observed]=oy;
  end
  if(frame==4 && oy!=cached[observed])$fatal(1,"cached output differs");
  if(frame==5 && oy!=200)$fatal(1,"seek failed to reset history");
  if(frame==6 && oy!=90)$fatal(1,"disabled filter not bypassed");
  if((frame==9 || frame==19) && observed%32==16)begin
   ghost=(oy>current_y[observed])?oy-current_y[observed]:current_y[observed]-oy;
   if(frame==9)new_ghost=new_ghost+ghost;else old_ghost=old_ghost+ghost;
  end
  observed=observed+1;
 end
 task send_frame(input integer f,input [15:0] sid,input [7:0] value);
 begin
  frame=f;observed=0;y=value;
  for(j=0;j<256;j=j+1)begin
   current_y[j]=value;
   if(f>=8)current_y[j]=((j%32<((f==8 || f==18)?16:17))?100:108)+((j*7+((f>=18)?f-10:f)*3)%5)-2;
  end
  model(sid);
  // Real HDMI polarity: low sync pulse, then rising edge. The RAW source ID
  // changes ~100 pixel clocks AFTER that edge, not before the low pulse.
  @(negedge pc);vs=0;repeat(150)@(negedge pc);vs=1;
  repeat(100)@(negedge pc);id=sid;
  repeat(160)@(negedge pc);
  for(n=0;n<256;n=n+1)begin de=1;y=current_y[n];@(negedge pc);if(n%32==31)begin de=0;repeat(8)@(negedge pc);end end
  de=0;repeat(400)@(negedge pc);
  if(observed!=256)$fatal(1,"output count %d",observed);
  if(status[31:24]!=0)$fatal(1,"history errors %h",status);
  if(f==4 && (strong_pixels!=240 || weak_pixels!=0 || bypass_pixels!=16))
   $fatal(1,"fusion counters not aligned with computed frame: %d %d %d",strong_pixels,weak_pixels,bypass_pixels);
 end endtask
 initial begin
  repeat(20)@(negedge pc);rst=0;prst=0;
  send_frame(1,1,100);
  send_frame(2,1,180); // repeat source must use cached result, regardless of input
  send_frame(3,2,108);
  send_frame(4,2,108);
  send_frame(5,20,200);
  en=0;send_frame(6,21,90);
  en=1;send_frame(7,22,90);
  send_frame(8,40,0);send_frame(9,41,0);send_frame(10,41,0);
  tuning[55:48]=12;send_frame(11,41,0); // same source, changed tuning must invalidate cache
  send_frame(12,42,0);
  tuning=64'hff3010080a040100;send_frame(13,42,0); // legacy 1/4,1/2,1 is still selectable
  send_frame(14,43,0);
  tuning[15:0]=16'h0303;send_frame(15,43,0);send_frame(16,44,0); // all-current coefficients
  tuning=64'hff3010080a040100;send_frame(18,60,0);send_frame(19,61,0);
  if(new_ghost>=old_ghost)$fatal(1,"synthetic moving edge did not improve new=%d old=%d",new_ghost,old_ghost);
  $display("SYNTHETIC_GHOST: new=%0d legacy=%0d (8 disoccluded edge pixels, absolute gray error)",new_ghost,old_ghost);
  if(acompleted!=64)$fatal(1,"AXI arbitration progress incomplete");
  $display("TEMPORAL_HISTORY_PASS: Q8.4 IIR, duplicate identity, seek reset, bypass, DDR backpressure");$finish;
 end
 initial begin #2000000;$fatal(1,"timeout");end
endmodule
