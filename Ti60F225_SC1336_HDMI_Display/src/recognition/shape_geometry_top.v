`timescale 1ns/1ps
// All logic runs in clk_pixel. Full-frame shape detection is independent of
// the existing fixed-ROI digit/letter recognizer and never stalls video.
module shape_geometry_top #(parameter integer MAX_OBJECTS=8) (
 input wire clk,rst_n,vsync_i,de_i,edge_i,
 output wire [MAX_OBJECTS*36-1:0] objects_o,output wire [MAX_OBJECTS-1:0] valid_o,
 output wire busy_o,output wire [15:0] rejected_o,output wire [23:0] cycles_o
);
 wire ready;wire [13:0] addr;wire bit_data,done;
 wire [MAX_OBJECTS*36-1:0] raw_objects;wire [MAX_OBJECTS-1:0] raw_valid;
 shape_frame_capture u_capture(.clk(clk),.rst_n(rst_n),.vsync_i(vsync_i),.de_i(de_i),
   .edge_i(edge_i),.busy_i(busy_o),.frame_ready_o(ready),.read_addr_i(addr),.read_data_o(bit_data));
 shape_geometry_engine #(.MAX_OBJECTS(MAX_OBJECTS)) u_ccl_moore(.clk(clk),.rst_n(rst_n),.start_i(ready),.busy_o(busy_o),
   .result_valid_o(done),.raw_addr_o(addr),.raw_data_i(bit_data),.objects_o(raw_objects),
   .valid_o(raw_valid),.rejected_o(rejected_o),.cycles_o(cycles_o));
 recognition_object_tracker #(.MAX_OBJECTS(MAX_OBJECTS),.TYPE_BITS(4),.X_BITS(8),.Y_BITS(8),.CONFIRM(2),.TOLERANCE(3)) tracker(
 .clk(clk),.rst_n(rst_n),.vsync_i(vsync_i),.done(done),.overflow_o(1'b0),.contrast(1'b1),
 .raw_objects(raw_objects),.raw_valid(raw_valid),.objects_o(objects_o),.valid_o(valid_o));
endmodule
