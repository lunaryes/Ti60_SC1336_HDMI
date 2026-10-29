`timescale 1ns/1ps
// 逐候选空间关联 FSM：避免四目标贪心匹配形成串接组合长路径。
// 三次同类确认，允许一次分析轮次漏检；结果只在场消隐边界发布。
module character_detection_top #(parameter integer MAX_OBJECTS=8)(
 input wire clk,rst_n,vsync_i,de_i,input wire [7:0] gray_i,
 output wire [MAX_OBJECTS*42-1:0] objects_o,output wire [MAX_OBJECTS-1:0] valid_o,
 output wire busy_o,overflow_o,output wire [7:0] threshold_o,output wire [3:0] diagnostic_o
);
 wire ready,done,contrast;wire [15:0] addr;wire [3:0] data;
 wire [MAX_OBJECTS*42-1:0] raw_objects;wire [MAX_OBJECTS-1:0] raw_valid;wire [23:0] cycles;
 character_frame_capture capture(clk,rst_n,vsync_i,de_i,gray_i,busy_o,ready,threshold_o,contrast,addr,data);
 character_ccl_engine #(.MAX_OBJECTS(MAX_OBJECTS)) engine(clk,rst_n,ready,busy_o,done,addr,data,raw_objects,raw_valid,overflow_o,cycles,diagnostic_o);
 recognition_object_tracker #(.MAX_OBJECTS(MAX_OBJECTS)) tracker(
 .clk(clk),.rst_n(rst_n),.vsync_i(vsync_i),.done(done),.overflow_o(overflow_o),.contrast(contrast),
 .raw_objects(raw_objects),.raw_valid(raw_valid),.objects_o(objects_o),.valid_o(valid_o));
endmodule
