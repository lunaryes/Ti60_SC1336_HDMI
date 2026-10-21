`timescale 1ns/1ps
// 逐候选空间关联 FSM：避免四目标贪心匹配形成串接组合长路径。
// 三次同类确认，允许一次分析轮次漏检；结果只在场消隐边界发布。
module character_detection_top(
 input wire clk,rst_n,vsync_i,de_i,input wire [7:0] gray_i,
 output reg [167:0] objects_o,output reg [3:0] valid_o,
 output wire busy_o,overflow_o,output wire [7:0] threshold_o
);
 wire ready,done,contrast;wire [15:0] addr;wire [3:0] data;
 wire [167:0] raw_objects;wire [3:0] raw_valid;wire [23:0] cycles;
 character_frame_capture capture(clk,rst_n,vsync_i,de_i,gray_i,busy_o,ready,threshold_o,contrast,addr,data);
 character_ccl_engine engine(clk,rst_n,ready,busy_o,done,addr,data,raw_objects,raw_valid,overflow_o,cycles);
 localparam IDLE=0,SCAN_INIT=1,COMPARE=2,APPLY=3,ALLOC_INIT=4,FIND_FREE=5,PUBLISH=6;
 reg [2:0] state;reg [1:0] ti,ri,fi,chosen;reg chosen_valid;
 reg [167:0] tracks,work_tracks,snapshot;
 reg [3:0] live,confirmed,work_live,work_confirmed,snapshot_valid,used;
 reg [7:0] hits,miss,work_hits,work_miss;
 reg vs_d;reg [5:0] idle_frames;reg [10:0] best_cost;
 wire [41:0] a=tracks[ti*42+:42],b=snapshot[ri*42+:42],selected=snapshot[chosen*42+:42];
 function [9:0] delta;input [8:0] a,b;
 begin delta=a>b?{1'b0,a}-{1'b0,b}:{1'b0,b}-{1'b0,a};end endfunction
 wire [9:0] d0=delta(a[33:25],b[33:25]),d1=delta(a[24:16],b[24:16]);
 wire [9:0] d2=delta({1'b0,a[15:8]},{1'b0,b[15:8]}),d3=delta({1'b0,a[7:0]},{1'b0,b[7:0]});
 wire nearby=d0<=6&&d1<=6&&d2<=6&&d3<=6;
 wire [10:0] cost={1'b0,d0}+{1'b0,d1}+{1'b0,d2}+{1'b0,d3};
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   objects_o<=0;valid_o<=0;tracks<=0;work_tracks<=0;snapshot<=0;
   live<=0;confirmed<=0;work_live<=0;work_confirmed<=0;snapshot_valid<=0;used<=0;
   hits<=0;miss<=0;work_hits<=0;work_miss<=0;vs_d<=0;idle_frames<=0;
   state<=IDLE;ti<=0;ri<=0;fi<=0;chosen<=0;chosen_valid<=0;best_cost<=2047;
  end else begin
   vs_d<=vsync_i;
   if(vsync_i&&!vs_d) begin
    objects_o<=tracks;valid_o<=confirmed;
    if(idle_frames<63) idle_frames<=idle_frames+1'b1;
   end
   case(state)
    IDLE:if(done) begin
     idle_frames<=0;
     if(overflow_o) begin live<=0;confirmed<=0;hits<=0;miss<=0;end
     else begin
      snapshot<=raw_objects;snapshot_valid<=raw_valid;used<=0;
      work_tracks<=tracks;work_live<=live;work_confirmed<=confirmed;work_hits<=hits;work_miss<=miss;
      ti<=0;state<=SCAN_INIT;
     end
    end
    SCAN_INIT:begin
     chosen_valid<=0;best_cost<=2047;ri<=0;
     if(live[ti]) state<=COMPARE;
     else if(ti==3) state<=ALLOC_INIT;else ti<=ti+1'b1;
    end
    COMPARE:begin
     if(snapshot_valid[ri]&&!used[ri]&&nearby&&cost<best_cost) begin
      chosen<=ri;chosen_valid<=1;best_cost<=cost;
     end
     if(ri==3) state<=APPLY;else ri<=ri+1'b1;
    end
    APPLY:begin
     if(chosen_valid) begin
      used[chosen]<=1;work_tracks[ti*42+:42]<=selected;work_miss[ti*2+:2]<=0;
      if(a[41:34]==selected[41:34]) begin
       if(hits[ti*2+:2]<3) work_hits[ti*2+:2]<=hits[ti*2+:2]+1'b1;
       work_confirmed[ti]<=hits[ti*2+:2]>=2;
      end else begin work_hits[ti*2+:2]<=1;work_confirmed[ti]<=0;end
     end else if(miss[ti*2+:2]==0&&confirmed[ti]) work_miss[ti*2+:2]<=1;
     else begin work_live[ti]<=0;work_confirmed[ti]<=0;work_hits[ti*2+:2]<=0;end
     if(ti==3) state<=ALLOC_INIT;else begin ti<=ti+1'b1;state<=SCAN_INIT;end
    end
    ALLOC_INIT:begin ri<=0;fi<=0;state<=FIND_FREE;end
    FIND_FREE:begin
     if(snapshot_valid[ri]&&!used[ri]&&!work_live[fi]) begin
      work_tracks[fi*42+:42]<=b;work_live[fi]<=1;work_confirmed[fi]<=0;
      work_hits[fi*2+:2]<=1;work_miss[fi*2+:2]<=0;
      fi<=0;if(ri==3) state<=PUBLISH;else ri<=ri+1'b1;
     end else if(!snapshot_valid[ri]||used[ri]||fi==3) begin
      fi<=0;if(ri==3) state<=PUBLISH;else ri<=ri+1'b1;
     end else fi<=fi+1'b1;
    end
    PUBLISH:begin
     tracks<=work_tracks;live<=work_live;confirmed<=work_confirmed;hits<=work_hits;miss<=work_miss;state<=IDLE;
    end
    default:state<=IDLE;
   endcase
   if(vsync_i&&!vs_d&&(idle_frames>=24||!contrast)) begin
    valid_o<=0;live<=0;confirmed<=0;hits<=0;miss<=0;state<=IDLE;
   end
  end
 end
endmodule
