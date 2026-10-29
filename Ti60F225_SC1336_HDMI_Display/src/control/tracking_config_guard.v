// Configuration is already transferred as a coherent CDC snapshot.
// Non-live sources latch the K2 state paused. Threshold writes only rearm.
module tracking_config_guard(input wire clk,rst_n,
 input wire [18:0] config_i,input wire valid_i,
 output wire auto_pause_o,output wire rearming_o);
 reg [18:0] previous;reg seen;reg [5:0] rearm;
 wire source_changed=valid_i && seen && config_i[18:8]!=previous[18:8];
 assign auto_pause_o=(seen && previous[18:16]!=0) ||
  (valid_i && config_i[18:16]!=0) || source_changed;
 assign rearming_o=rearm!=0;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin previous<=19'd12;seen<=0;rearm<=0;end
 else begin
  if(rearm!=0) rearm<=rearm-1'b1;
  if(valid_i) begin
   previous<=config_i;seen<=1;
   if(seen && config_i[7:0]!=previous[7:0]) rearm<=63;
  end
 end
endmodule
