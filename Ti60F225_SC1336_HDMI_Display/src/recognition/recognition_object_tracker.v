`timescale 1ns/1ps
// Serial nearest-box tracking, independent of raw CCL output slot order.
module recognition_object_tracker #(
 parameter integer MAX_OBJECTS=8,TYPE_BITS=8,X_BITS=9,Y_BITS=8,
 parameter integer CONFIRM=3,TOLERANCE=6,TTL=24,
 parameter integer DW=TYPE_BITS+2*X_BITS+2*Y_BITS
)(
 input wire clk,rst_n,vsync_i,done,overflow_o,contrast,
 input wire [MAX_OBJECTS*DW-1:0] raw_objects,input wire [MAX_OBJECTS-1:0] raw_valid,
 output reg [MAX_OBJECTS*DW-1:0] objects_o,output reg [MAX_OBJECTS-1:0] valid_o
);
 localparam IDLE=0,SCAN_INIT=1,COMPARE=2,APPLY=3,ALLOC_INIT=4,FIND_FREE=5,PUBLISH=6;
 reg [2:0] state;localparam IW=(MAX_OBJECTS<2)?1:$clog2(MAX_OBJECTS);
 reg [IW-1:0] ti,ri,fi,chosen;reg chosen_valid;
 reg [MAX_OBJECTS*DW-1:0] tracks,work_tracks,snapshot;
 reg [MAX_OBJECTS-1:0] live,confirmed,work_live,work_confirmed,snapshot_valid,used;
 reg [MAX_OBJECTS*2-1:0] hits,miss,work_hits,work_miss;
 reg vs_d;reg [5:0] idle_frames;reg [10:0] best_cost;
 // Explicit fixed slices avoid non-power-of-two variable part-select barrel
 // shifters (42-bit descriptors otherwise expanded badly in Efinity).
 reg [DW-1:0] a,b,selected;integer mux_index,store_index;
 always @(*) begin
  a=0;b=0;selected=0;
  for(mux_index=0;mux_index<MAX_OBJECTS;mux_index=mux_index+1) begin
   if(ti==mux_index) a=tracks[mux_index*DW+:DW];
   if(ri==mux_index) b=snapshot[mux_index*DW+:DW];
   if(chosen==mux_index) selected=snapshot[mux_index*DW+:DW];
  end
 end
 function [9:0] delta;input [8:0] a,b;
 begin delta=a>b?{1'b0,a}-{1'b0,b}:{1'b0,b}-{1'b0,a};end endfunction
 wire [9:0] d0=delta(a[2*Y_BITS+X_BITS+:X_BITS],b[2*Y_BITS+X_BITS+:X_BITS]),d1=delta(a[2*Y_BITS+:X_BITS],b[2*Y_BITS+:X_BITS]);
 wire [9:0] d2=delta(a[Y_BITS+:Y_BITS],b[Y_BITS+:Y_BITS]),d3=delta(a[0+:Y_BITS],b[0+:Y_BITS]);
 wire nearby=d0<=TOLERANCE&&d1<=TOLERANCE&&d2<=TOLERANCE&&d3<=TOLERANCE;
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
     else if(ti==MAX_OBJECTS-1) state<=ALLOC_INIT;else ti<=ti+1'b1;
    end
    COMPARE:begin
     if(snapshot_valid[ri]&&!used[ri]&&nearby&&cost<best_cost) begin
      chosen<=ri;chosen_valid<=1;best_cost<=cost;
     end
     if(ri==MAX_OBJECTS-1) state<=APPLY;else ri<=ri+1'b1;
    end
    APPLY:begin
     if(chosen_valid) begin
      used[chosen]<=1;work_miss[ti*2+:2]<=0;
      for(store_index=0;store_index<MAX_OBJECTS;store_index=store_index+1)
       if(ti==store_index) work_tracks[store_index*DW+:DW]<=selected;
      if(a[DW-1-:TYPE_BITS]==selected[DW-1-:TYPE_BITS]) begin
       if(hits[ti*2+:2]<3) work_hits[ti*2+:2]<=hits[ti*2+:2]+1'b1;
       work_confirmed[ti]<=hits[ti*2+:2]>=CONFIRM-1;
      end else begin work_hits[ti*2+:2]<=1;work_confirmed[ti]<=0;end
     end else if(miss[ti*2+:2]==0&&confirmed[ti]) work_miss[ti*2+:2]<=1;
     else begin work_live[ti]<=0;work_confirmed[ti]<=0;work_hits[ti*2+:2]<=0;end
     if(ti==MAX_OBJECTS-1) state<=ALLOC_INIT;else begin ti<=ti+1'b1;state<=SCAN_INIT;end
    end
    ALLOC_INIT:begin ri<=0;fi<=0;state<=FIND_FREE;end
    FIND_FREE:begin
     if(snapshot_valid[ri]&&!used[ri]&&!work_live[fi]) begin
      for(store_index=0;store_index<MAX_OBJECTS;store_index=store_index+1)
       if(fi==store_index) work_tracks[store_index*DW+:DW]<=b;
      work_live[fi]<=1;work_confirmed[fi]<=0;
      work_hits[fi*2+:2]<=1;work_miss[fi*2+:2]<=0;
      fi<=0;if(ri==MAX_OBJECTS-1) state<=PUBLISH;else ri<=ri+1'b1;
     end else if(!snapshot_valid[ri]||used[ri]||fi==MAX_OBJECTS-1) begin
      fi<=0;if(ri==MAX_OBJECTS-1) state<=PUBLISH;else ri<=ri+1'b1;
     end else fi<=fi+1'b1;
    end
    PUBLISH:begin
     tracks<=work_tracks;live<=work_live;confirmed<=work_confirmed;hits<=work_hits;miss<=work_miss;state<=IDLE;
    end
    default:state<=IDLE;
   endcase
   if(vsync_i&&!vs_d&&(idle_frames>=TTL||!contrast)) begin
    valid_o<=0;live<=0;confirmed<=0;hits<=0;miss<=0;state<=IDLE;
   end
  end
 end
endmodule
