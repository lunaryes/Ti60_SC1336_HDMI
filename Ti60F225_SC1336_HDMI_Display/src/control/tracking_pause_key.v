// Active-low K2: one toggle per stable press, including release debounce.
module tracking_pause_key #(parameter integer CLK_HZ=96000000, DEBOUNCE_MS=20,
 parameter integer COUNT_MAX=CLK_HZ/1000*DEBOUNCE_MS-1)(
 input wire clk,rst_n,key_n,input wire auto_pause_i,output reg enabled_o);
 localparam integer CW=COUNT_MAX<1?1:$clog2(COUNT_MAX+1);
 reg [1:0] sync; reg stable; reg [CW-1:0] count;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin sync<=2'b11;stable<=1;count<=0;enabled_o<=1;end
 else begin
  sync<={sync[0],key_n};
  if(sync[1]==stable) count<=0;
  else if(count==COUNT_MAX) begin
   stable<=sync[1];count<=0;
   if(!sync[1]) enabled_o<=!enabled_o;
  end else count<=count+1'b1;
  if(auto_pause_i) enabled_o<=0;
 end
endmodule
