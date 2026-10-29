// Position-mode incremental visual servo; speed mode uses neutral on loss.
// Error units: 4 pixels. Pulse units: 96 MHz clock cycles.
module triangle_follow_ctrl #(
 parameter integer CLK_HZ=96000000,SPEED_MODE=0,DIR_INVERT=0,
 parameter integer MID=140160,NEUTRAL=144000,LO=115200,HI=165120,
 parameter integer GAIN=8,MAX_STEP=384,SPEED_GAIN=12,SPEED_LIMIT=1920,
 parameter integer DEAD_IN=3,DEAD_OUT=6,TIMEOUT_MS=300
)(
 input wire clk,rst_n,enable_i,tick_i,sample_i,found_i,
 input wire signed [9:0] ex_i,
 output reg [20:0] pulse_o,output wire [3:0] status_o
);
 localparam integer TIMEOUT=CLK_HZ/1000*TIMEOUT_MS;
 reg [31:0] age;
 reg signed [9:0] error;
 reg found,fresh,centered;
 reg [1:0] hits,en_sync;
 reg enable_prev;
 reg signed [31:0] step,next_p;
 wire usable=en_sync[1] && found && hits>=2 && age<TIMEOUT;
 assign status_o={pulse_o==LO||pulse_o==HI,!usable,centered&&usable,usable};
 always @* begin
  step=$signed(error)*(SPEED_MODE ? SPEED_GAIN : GAIN);
  if(DIR_INVERT) step=-step;
  if(SPEED_MODE) begin
   if(step>SPEED_LIMIT) step=SPEED_LIMIT;
   if(step< -SPEED_LIMIT) step=-SPEED_LIMIT;
   next_p=NEUTRAL+step;
  end else begin
   if(step>MAX_STEP) step=MAX_STEP;
   if(step< -MAX_STEP) step=-MAX_STEP;
   next_p=$signed({1'b0,pulse_o})+step;
  end
  if(next_p<LO) next_p=LO;
  if(next_p>HI) next_p=HI;
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   pulse_o<=SPEED_MODE?NEUTRAL:MID;age<=TIMEOUT;
   error<=0;found<=0;fresh<=0;centered<=1;hits<=0;en_sync<=0;enable_prev<=0;
  end else begin
   en_sync<={en_sync[0],enable_i};
   enable_prev<=en_sync[1];
   if(age<TIMEOUT) age<=age+1'b1;
   if(tick_i) begin
    if(!usable) begin
     if(SPEED_MODE) pulse_o<=NEUTRAL;
    end else if(fresh) begin
     if(centered) begin
      if(error>DEAD_OUT || error< -DEAD_OUT) begin
       centered<=0;pulse_o<=next_p;
      end else if(SPEED_MODE) pulse_o<=NEUTRAL;
     end else if(error<=DEAD_IN && error>= -DEAD_IN) begin
      centered<=1;
      if(SPEED_MODE) pulse_o<=NEUTRAL;
     end else pulse_o<=next_p;
    end
    fresh<=0;
   end
   if(sample_i && en_sync[1] && enable_prev) begin
    age<=0;error<=ex_i;found<=found_i;fresh<=1;
    if(!found_i) hits<=0;
    else if(hits<2) hits<=hits+1'b1;
   end
   if(!en_sync[1] || !enable_prev || (age>=TIMEOUT && !sample_i)) begin
    hits<=0;centered<=1;fresh<=0;found<=0;age<=TIMEOUT;error<=0;
    if(SPEED_MODE) pulse_o<=NEUTRAL;
   end
  end
 end
endmodule
