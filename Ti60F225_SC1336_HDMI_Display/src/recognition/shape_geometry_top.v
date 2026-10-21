`timescale 1ns/1ps
// All logic runs in clk_pixel. Full-frame shape detection is independent of
// the existing fixed-ROI digit/letter recognizer and never stalls video.
module shape_geometry_top (
 input wire clk,rst_n,vsync_i,de_i,edge_i,
 output reg [143:0] objects_o,output reg [3:0] valid_o,
 output wire busy_o,output wire [15:0] rejected_o,output wire [23:0] cycles_o
);
 wire ready;wire [13:0] addr;wire bit_data,done;
 wire [143:0] raw_objects;wire [3:0] raw_valid;
 shape_frame_capture u_capture(.clk(clk),.rst_n(rst_n),.vsync_i(vsync_i),.de_i(de_i),
   .edge_i(edge_i),.busy_i(busy_o),.frame_ready_o(ready),.read_addr_i(addr),.read_data_o(bit_data));
 shape_geometry_engine u_ccl_moore(.clk(clk),.rst_n(rst_n),.start_i(ready),.busy_o(busy_o),
   .result_valid_o(done),.raw_addr_o(addr),.raw_data_i(bit_data),.objects_o(raw_objects),
   .valid_o(raw_valid),.rejected_o(rejected_o),.cycles_o(cycles_o));
 reg [143:0] previous,pending;reg [3:0] previous_valid,pending_valid;
 reg vs_d;integer i;
 function near8;
 input [7:0] a,b;
 begin near8=({1'b0,a}+3>={1'b0,b})&&({1'b0,b}+3>={1'b0,a});end
 endfunction
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     previous<=0;pending<=0;previous_valid<=0;pending_valid<=0;objects_o<=0;valid_o<=0;vs_d<=0;
   end else begin
     vs_d<=vsync_i;
     if(done) begin
       previous<=raw_objects;previous_valid<=raw_valid;pending<=raw_objects;
       for(i=0;i<4;i=i+1) begin
         pending_valid[i]<=raw_valid[i]&&previous_valid[i]&&
           raw_objects[i*36+32+:4]==previous[i*36+32+:4]&&
           near8(raw_objects[i*36+24+:8],previous[i*36+24+:8])&&
           near8(raw_objects[i*36+16+:8],previous[i*36+16+:8])&&
           near8(raw_objects[i*36+8+:8],previous[i*36+8+:8])&&
           near8(raw_objects[i*36+:8],previous[i*36+:8]);
       end
     end
     // Atomically publish during blanking, not halfway down the display.
     if(vsync_i&&!vs_d) begin objects_o<=pending;valid_o<=pending_valid;end
   end
 end
endmodule
