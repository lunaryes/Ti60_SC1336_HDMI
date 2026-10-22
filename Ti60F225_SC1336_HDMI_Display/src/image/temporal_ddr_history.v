`timescale 1ns/1ps
// Two 2-MiB history pages at 16/18 MiB, disjoint from RAW slots 0..15.
// A page becomes visible only after exact pixel count and final AXI B response.
module temporal_ddr_history #(parameter W=1280,H=720,BURST=128)(
 input wire sc,pc,rst,prst,
 input wire frame_vs,pixel_de,input wire [7:0] pixel_y,
 input wire [15:0] source_id,input wire enable_i,input wire [2:0] mode_i,
 input wire [79:0] config_i,
 output wire filtered_vs,filtered_de,output wire [7:0] filtered_y,
 output wire [31:0] status_o,
 output reg [31:0] strong_pixels_o,weak_pixels_o,bypass_pixels_o,
 output wire [31:0] awaddr,output wire [7:0] awlen,output wire awvalid,input wire awready,
 output wire [127:0] wdata,output wire wlast,wvalid,input wire wready,
 input wire bvalid,input wire [1:0] bresp,
 output wire [31:0] araddr,output wire [7:0] arlen,output wire arvalid,input wire arready,
 input wire [127:0] rdata,input wire rlast,rvalid,input wire [1:0] rresp
);
 localparam BYTES=W*H*2;
 reg req_toggle,done_toggle,vs_d,seen_ack;
 reg [7:0] vs_wait;
 reg [15:0] requested_id;reg [79:0] requested_config;
 reg [63:0] active_tuning;
 reg requested_enable;reg [2:0] requested_mode;
 (* async_reg="true" *) reg [1:0] req_sync,done_sync,ack_sync;
 reg ack_toggle,seen_req,seen_done;
 reg [1:0] operation;
 reg [1:0] op_meta,op_pixel;
 reg start_pixel;
 reg [3:0] reset_count;
 reg fifo_reset;
 reg valid_history,valid_page,read_page,write_page;
 reg [15:0] history_id,epoch_id;reg [79:0] history_config,epoch_config;
 reg do_read,do_write,epoch_done,epoch_bad;
 reg [31:0] roffset,woffset;
 reg [7:0] rb,wb;
 reg [1:0] rs,ws;
 reg [7:0] errors;
 wire rfull,rempty,wfull,wempty;
 wire [9:0] rused_w,rused_r,wused_w,wused_r;
 wire [127:0] read_word,write_word;
 reg [127:0] pack;
 reg [2:0] pixel_index;
 reg [31:0] pixel_count;
 reg pixel_bad,done_bad;
 reg [31:0] done_count;
 reg write_push;
 reg [127:0] write_push_data;
 wire [15:0] hpixel=history_pixel;
 wire history_pop=pixel_de && (op_pixel==2 || op_pixel==3) && pixel_index==7 && !rempty;
 wire [15:0] result_q;
 wire [1:0] alpha;
 wire result_de,result_vs;
 assign filtered_de=result_de;assign filtered_vs=result_vs;
 gray_temporal_iir #(.W(W)) core(.clk(pc),.rst(prst),.frame_start(start_pixel),
  .vs_i(frame_vs),.de_i(pixel_de),.y_i(pixel_y),.history_i(hpixel),.tuning_i(active_tuning),
  .operation_i((rempty && (op_pixel==2 || op_pixel==3))?2'd1:op_pixel),
  .vs_o(result_vs),.de_o(result_de),.y_o(filtered_y),.history_o(result_q),.alpha_o(alpha));
 reg [31:0] strong_count,weak_count,bypass_count;
 reg result_vs_d;
 always @(posedge pc or posedge prst) begin
  if(prst)begin
   req_toggle<=0;done_toggle<=0;vs_d<=0;seen_ack<=0;vs_wait<=0;requested_id<=0;requested_config<=0;
   active_tuning<=64'h0818080405020201;requested_enable<=0;requested_mode<=0;ack_sync<=0;op_meta<=0;op_pixel<=0;
   start_pixel<=0;pixel_index<=0;pixel_count<=0;pixel_bad<=0;done_bad<=0;done_count<=0;
   pack<=0;write_push<=0;write_push_data<=0;
   strong_count<=0;weak_count<=0;bypass_count<=0;result_vs_d<=0;
   strong_pixels_o<=0;weak_pixels_o<=0;bypass_pixels_o<=0;
  end else begin
   vs_d<=frame_vs;ack_sync<={ack_sync[0],ack_toggle};op_meta<=operation;
   start_pixel<=0;write_push<=0;result_vs_d<=result_vs;
   // HDMI VS is active-low. The RAW reader resets at its rising edge and
   // takes ~105 pixel clocks to select/latch a source. Wait 128 clocks in
   // vertical back porch before capturing its ID, well before active DE.
   if(!vs_d && frame_vs)vs_wait<=128;
   else if(vs_wait!=0)vs_wait<=vs_wait-1'b1;
   if(vs_wait==1)begin
    requested_id<=source_id;requested_enable<=enable_i;requested_mode<=mode_i;
    requested_config<=config_i;req_toggle<=~req_toggle;
   end
   if(ack_sync[1]!=seen_ack)begin
    seen_ack<=ack_sync[1];active_tuning<=requested_config[63:0];
    op_pixel<=op_meta;start_pixel<=1;pixel_index<=0;pixel_count<=0;pixel_bad<=0;
   end
   if(pixel_de && (op_pixel==2 || op_pixel==3) && rempty) pixel_bad<=1;
   // Output has the same pixel ordering but a three-clock pipeline delay.
   if(result_vs && !result_vs_d)begin
    if(op_pixel!=3)begin
     strong_pixels_o<=strong_count;weak_pixels_o<=weak_count;bypass_pixels_o<=bypass_count;
    end
    strong_count<=0;weak_count<=0;bypass_count<=0;
   end
   if(result_de)begin
    if(alpha==2)strong_count<=strong_count+1'b1;
    else if(alpha==1)weak_count<=weak_count+1'b1;
    else bypass_count<=bypass_count+1'b1;
    if(op_pixel==1 || op_pixel==2)begin
     pack[pixel_index*16+:16]<=result_q;
     if(pixel_index==7)begin
      write_push_data<={result_q,pack[111:0]};write_push<=!wfull;
      if(wfull)pixel_bad<=1;
     end
    end
    pixel_index<=pixel_index+1'b1;pixel_count<=pixel_count+1'b1;
    if(pixel_count==W*H-1)begin done_count<=pixel_count+1'b1;done_bad<=pixel_bad;done_toggle<=~done_toggle;end
   end
  end
 end
 // Read consumption must use input coordinates, independently of output pack.
 reg [2:0] history_index;
 always @(posedge pc or posedge prst)begin
  if(prst)history_index<=0;
  else if(start_pixel)history_index<=0;
  else if(pixel_de)history_index<=history_index+1'b1;
 end
 // Dedicated asynchronous FIFOs hold short bursts only.
 wire hist_pop=pixel_de && (op_pixel==2 || op_pixel==3) && history_index==7 && !rempty;
 wire [15:0] history_pixel=read_word[history_index*16+:16];
 temporal_async_fifo read_fifo(.wc(sc),.rc(pc),.rst(rst||fifo_reset),
  .push(rs==2 && rvalid),.pop(hist_pop),.data_i(rdata),.data_o(read_word),
  .full(rfull),.empty(rempty),.write_used(rused_w),.read_used(rused_r));
 temporal_async_fifo write_fifo(.wc(pc),.rc(sc),.rst(rst||fifo_reset),
  .push(write_push),.pop(wvalid&&wready),.data_i(write_push_data),.data_o(write_word),
  .full(wfull),.empty(wempty),.write_used(wused_w),.read_used(wused_r));
 assign awaddr=32'h01000000+(write_page?32'h00200000:0)+woffset;
 assign araddr=32'h01000000+(read_page?32'h00200000:0)+roffset;
 assign awlen=BURST-1;assign arlen=BURST-1;
 assign awvalid=ws==1;assign wvalid=ws==2;assign wlast=wb==BURST-1;assign wdata=write_word;
 assign arvalid=rs==1;
 reg pending;
 reg [1:0] bad_meta;
 reg [31:0] count_meta,count_sync;
 always @(posedge sc)begin
  if(rst)begin
   req_sync<=0;done_sync<=0;seen_req<=0;seen_done<=0;ack_toggle<=0;
   operation<=0;reset_count<=15;fifo_reset<=1;pending<=0;
   valid_history<=0;valid_page<=0;history_id<=0;history_config<=0;
   epoch_id<=0;epoch_config<=0;read_page<=0;write_page<=1;do_read<=0;do_write<=0;
   epoch_done<=0;epoch_bad<=0;roffset<=0;woffset<=0;rb<=0;wb<=0;rs<=0;ws<=0;
   errors<=0;bad_meta<=0;count_meta<=0;count_sync<=0;
  end else begin
   req_sync<={req_sync[0],req_toggle};done_sync<={done_sync[0],done_toggle};
   bad_meta<={bad_meta[0],done_bad};count_meta<=done_count;count_sync<=count_meta;
   if(req_sync[1]!=seen_req)begin pending<=1;seen_req<=req_sync[1];end
   if(done_sync[1]!=seen_done)begin
    seen_done<=done_sync[1];epoch_done<=1;
    if(bad_meta[1] || count_sync!=W*H)begin epoch_bad<=1;errors[0]<=1;end
   end
   if(rs==0 && !pending && !fifo_reset && do_read && roffset<BYTES && rused_w<=256)begin rs<=1;rb<=0;end
   if(rs==1 && arready)rs<=2;
   if(rs==2 && rvalid)begin
    rb<=rb+1'b1;
    if(rresp!=0 || (rlast!=(rb==BURST-1)))begin errors[1]<=1;epoch_bad<=1;end
    if(rlast)begin rs<=0;roffset<=roffset+BURST*16;end
   end
   if(ws==0 && !pending && !fifo_reset && do_write && woffset<BYTES && wused_r>=BURST)begin ws<=1;wb<=0;end
   if(ws==1 && awready)ws<=2;
   if(ws==2 && wready)begin if(wlast)ws<=3;else wb<=wb+1'b1;end
   if(ws==3 && bvalid)begin
    if(bresp!=0)begin errors[2]<=1;epoch_bad<=1;end
    ws<=0;woffset<=woffset+BURST*16;
   end
   if(epoch_done && do_write && ws==0 && woffset==BYTES && wempty)begin
    if(!epoch_bad)begin valid_history<=1;valid_page<=write_page;history_id<=epoch_id;history_config<=epoch_config;end
    else valid_history<=0;
    do_write<=0;
   end
   if(pending && rs==0 && ws==0)begin
    pending<=0;fifo_reset<=1;reset_count<=15;
    roffset<=0;woffset<=0;epoch_done<=0;epoch_bad<=0;
    epoch_id<=requested_id;epoch_config<=requested_config;
    read_page<=valid_page;write_page<=~valid_page;
    if(!requested_enable || requested_mode>=3)begin
     operation<=0;do_read<=0;do_write<=0;valid_history<=0;
    end else if(valid_history && requested_config==history_config && requested_id==history_id)begin
     operation<=3;do_read<=1;do_write<=0;
    end else begin
     operation<=(valid_history && requested_config==history_config && requested_id==history_id+16'd1)?2:1;
     do_read<=valid_history && requested_config==history_config && requested_id==history_id+16'd1;
     do_write<=1;
    end
   end else if(fifo_reset)begin
    if(reset_count!=0)reset_count<=reset_count-1'b1;
    else begin fifo_reset<=0;ack_toggle<=seen_req;end
   end
  end
 end
 assign status_o={errors,history_id,4'd0,valid_history,operation,epoch_bad};
endmodule
