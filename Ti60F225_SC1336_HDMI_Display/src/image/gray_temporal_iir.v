`timescale 1ns/1ps
// Three pixel clocks. Q8.4 history is retained in a 16-bit DDR word.
// Motion features: local DC-corrected difference and horizontal/vertical
// differences of the temporal residual (changes in spatial gradients).
module gray_temporal_iir #(parameter W=1280)(
 input wire clk,rst,frame_start,vs_i,de_i,
 input wire [7:0] y_i,input wire [15:0] history_i,
 input wire [1:0] operation_i, // 0 bypass, 1 initialize, 2 recursive, 3 cached
 input wire [63:0] tuning_i,
 output reg vs_o,de_o,output reg [7:0] y_o,
 output reg [15:0] history_o,output reg [1:0] alpha_o
);
 reg [2:0] vp,dp;
 reg [15:0] col,row;
 reg signed [8:0] previous_delta,local_dc;
 reg signed [13:0] block_sum;
 reg [3:0] block_count;
 reg signed [8:0] line_delta[0:W-1];
 reg signed [8:0] vertical_q;
 wire signed [8:0] delta=$signed({1'b0,y_i})-$signed({1'b0,history_i[11:4]});
 reg signed [8:0] d1,left1,dc1;
 reg [7:0] y1,y2;
 reg [15:0] h1,h2;
 reg [1:0] op1,op2;
 reg first_col1,first_row1;
 reg [1:0] choice2;
 integer residual,dx,dy,absolute,step,value,weight;
 function integer ab;
 input integer x;begin ab=(x<0)?-x:x;end
 endfunction
 always @(posedge clk) begin
   if(de_i) begin vertical_q<=line_delta[col];line_delta[col]<=delta;end
 end
 always @(posedge clk) begin
  if(rst) begin
   vp<=0;dp<=0;vs_o<=0;de_o<=0;y_o<=0;history_o<=0;alpha_o<=0;
   col<=0;row<=0;previous_delta<=0;local_dc<=0;block_sum<=0;block_count<=0;
   d1<=0;left1<=0;dc1<=0;y1<=0;y2<=0;h1<=0;h2<=0;op1<=0;op2<=0;
   first_col1<=1;first_row1<=1;choice2<=0;
  end else begin
   vp<={vp[1:0],vs_i};dp<={dp[1:0],de_i};
   vs_o<=vp[1];de_o<=dp[1];
   if(frame_start) begin col<=0;row<=0;block_count<=0;block_sum<=0;local_dc<=0;previous_delta<=0;end
   if(de_i) begin
    d1<=delta;left1<=previous_delta;
    if($signed(local_dc)>$signed({1'b0,tuning_i[63:56]}))dc1<=$signed({1'b0,tuning_i[63:56]});
    else if($signed(local_dc)<-$signed({1'b0,tuning_i[63:56]}))dc1<=-$signed({1'b0,tuning_i[63:56]});
    else dc1<=local_dc;
    first_col1<=col==0;first_row1<=row==0;
    y1<=y_i;h1<=history_i;op1<=operation_i;previous_delta<=delta;
    if(block_count==15) begin local_dc<=($signed(block_sum)+$signed(delta))/16;block_sum<=0;block_count<=0;end
    else begin block_sum<=block_sum+delta;block_count<=block_count+1'b1;end
    if(col==W-1) begin col<=0;row<=row+1'b1;block_count<=0;block_sum<=0;end
    else col<=col+1'b1;
   end
   y2<=y1;h2<=h1;op2<=op1;
   residual=ab($signed(d1)-$signed(dc1));absolute=ab($signed(d1));
   dx=first_col1?0:ab($signed(d1)-$signed(left1));
   dy=first_row1?0:ab($signed(d1)-$signed(vertical_q));
   choice2<=0;
   if(absolute<=tuning_i[55:48] && residual<=tuning_i[23:16] && dx<=tuning_i[39:32] && dy<=tuning_i[39:32]) choice2<=2;
   else if(absolute<=tuning_i[55:48] && residual<=tuning_i[31:24] && dx<=tuning_i[47:40] && dy<=tuning_i[47:40]) choice2<=1;
   value={y2,4'b0};
   weight=(choice2==2)?tuning_i[1:0]:((choice2==1)?tuning_i[9:8]:3);
   if(op2==3) value=h2;
   else if(op2==2) begin
    step=($signed({1'b0,y2,4'b0})-$signed({1'b0,h2}));
    // Symmetric rounding; no unsigned negative-difference wraparound.
    case(weight)
     0:step=(step<0)?-((-step+2)/4):(step+2)/4;
     1:step=(step<0)?-((-step+1)/2):(step+1)/2;
     2:step=(step<0)?-((-step*3+2)/4):(step*3+2)/4;
     default:;
    endcase
    value=h2+step;
   end
   if(value<0)value=0;if(value>4080)value=4080;
   history_o<=value[15:0];
   y_o<=(value>=4072)?8'd255:(value+8)>>4;
   alpha_o<=(op2==2 && weight!=3)?choice2:0;
  end
 end
endmodule

// Dual-clock block-RAM FIFO, registered FWFT read. Reset is held long enough
// in both domains at each video epoch; pointers use standard Gray CDC.
module temporal_async_fifo #(parameter A=9)(
 input wire wc,rc,rst,input wire push,pop,input wire [127:0] data_i,
 output reg [127:0] data_o,output wire full,empty,
 output wire [A:0] write_used,read_used
);
 reg [127:0] mem[0:(1<<A)-1];
 reg [A:0] wp,rp,wg,rg;
 (* async_reg="true" *) reg [A:0] r1,r2,w1,w2;
 function [A:0] binary;input [A:0] g;integer n;
 begin binary[A]=g[A];for(n=A-1;n>=0;n=n-1)binary[n]=binary[n+1]^g[n];end endfunction
 assign write_used=wp-binary(r2);assign read_used=binary(w2)-rp;
 assign full=write_used==(1<<A);assign empty=rg==w2;
 wire [A:0] next_r=rp+((pop&&!empty)?1'b1:1'b0);
 always @(posedge wc) begin
  if(push&&!full&&!rst)mem[wp[A-1:0]]<=data_i;
 end
 always @(posedge rc) data_o<=mem[next_r[A-1:0]];
 always @(posedge wc or posedge rst) begin
  if(rst)begin wp<=0;wg<=0;r1<=0;r2<=0;end
  else begin r1<=rg;r2<=r1;if(push&&!full)begin wp<=wp+1'b1;wg<=((wp+1'b1)>>1)^(wp+1'b1);end end
 end
 always @(posedge rc or posedge rst) begin
  if(rst)begin rp<=0;rg<=0;w1<=0;w2<=0;end
  else begin w1<=wg;w2<=w1;rp<=next_r;rg<=(next_r>>1)^next_r;end
 end
endmodule
