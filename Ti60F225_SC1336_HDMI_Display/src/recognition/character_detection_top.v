`timescale 1ns/1ps
// 使用既有像素时钟；连续两次确认，消隐期原子更新。
module character_detection_top(
 input wire clk,rst_n,vsync_i,de_i,input wire [7:0] gray_i,
 output reg [167:0] objects_o,output reg [3:0] valid_o,
 output wire busy_o,overflow_o,output wire [7:0] threshold_o
);
 wire ready,done,contrast;wire [15:0] addr;wire [1:0] data;
 wire [167:0] raw_objects;wire [3:0] raw_valid;wire [23:0] cycles;
 character_frame_capture capture(clk,rst_n,vsync_i,de_i,gray_i,busy_o,ready,threshold_o,contrast,addr,data);
 character_ccl_engine engine(clk,rst_n,ready,busy_o,done,addr,data,raw_objects,raw_valid,overflow_o,cycles);
 reg [167:0] previous,pending;reg [3:0] previous_valid,pending_valid;reg vs_d;
 reg [3:0] idle_frames;integer i;
 function near9;input [8:0] a,b;begin near9=({1'b0,a}+3>={1'b0,b})&&({1'b0,b}+3>={1'b0,a});end endfunction
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin objects_o<=0;valid_o<=0;previous<=0;pending<=0;previous_valid<=0;pending_valid<=0;vs_d<=0;idle_frames<=0;end
  else begin
   vs_d<=vsync_i;
   if(vsync_i&&!vs_d) begin
    objects_o<=pending;valid_o<=pending_valid;
    if(idle_frames<15) idle_frames<=idle_frames+1'b1;
    if(idle_frames>=12||!contrast) begin valid_o<=0;pending_valid<=0;previous_valid<=0;end
   end
   if(done) begin
    previous<=raw_objects;previous_valid<=raw_valid;pending<=raw_objects;idle_frames<=0;
    for(i=0;i<4;i=i+1) pending_valid[i]<=raw_valid[i]&&previous_valid[i]&&!overflow_o&&
      raw_objects[i*42+34+:8]==previous[i*42+34+:8]&&
      near9(raw_objects[i*42+25+:9],previous[i*42+25+:9])&&
      near9(raw_objects[i*42+16+:9],previous[i*42+16+:9])&&
      near9({1'b0,raw_objects[i*42+8+:8]},{1'b0,previous[i*42+8+:8]})&&
      near9({1'b0,raw_objects[i*42+:8]},{1'b0,previous[i*42+:8]});
   end
  end
 end
endmodule
