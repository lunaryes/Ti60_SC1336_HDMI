`timescale 1ns/1ps
// Application frame store. Camera -> complete DDR frames -> selectable reader
// -> optional motion-adaptive TWO-FRAME FIR -> existing Bayer/ISP/Canny.
// No UART image capture RAM, histogram, or diagnostic frame recorder.
module ddr_video_store #(
    parameter C_ID_LEN=4,C_W_WIDTH=8,C_R_WIDTH=8,
    parameter C_RD_END_ADDR=1280*720,
    parameter FRAME_W=1280,SLOTS=16,PAGE_BITS=20,BURST=128
)(
    input wire axi_clk,axi_reset,
    output wire [31:0] axi_awaddr,
    output wire [7:0] axi_awlen,
    output wire axi_awvalid,input wire axi_awready,
    output wire [127:0] axi_wdata,output wire [15:0] axi_wstrb,
    output wire axi_wlast,axi_wvalid,input wire axi_wready,
    input wire [C_ID_LEN-1:0] axi_bid,input wire [1:0] axi_bresp,input wire axi_bvalid,
    output wire [C_ID_LEN-1:0] axi_arid,
    output wire [31:0] axi_araddr,output wire [7:0] axi_arlen,
    output wire axi_arvalid,input wire axi_arready,
    input wire [C_ID_LEN-1:0] axi_rid,input wire [127:0] axi_rdata,
    input wire [1:0] axi_rresp,input wire axi_rlast,axi_rvalid,
    output wire axi_rready,
    input wire wframe_pclk,wframe_vsync,wframe_data_en,
    input wire [C_W_WIDTH-1:0] wframe_data,
    input wire rframe_pclk,rframe_vsync,rframe_data_en,
    output wire [C_R_WIDTH-1:0] rframe_data,
    input wire [2:0] play_mode_i,
    input wire tnr_enable_i,
    input wire [3:0] age_a_i,age_b_i,
    input wire [7:0] replay_div_i,
    output wire [31:0] tp_o,
    output wire [31:0] status_o,
    output wire [15:0] display_source_o
);
    localparam TOTAL=C_RD_END_ADDR;
    localparam STRIDE=BURST*16;
    (* async_reg="true" *) reg [19:0] cfg_meta,cfg;
    wire hold_record=cfg[19:17]!=0;
    always @(posedge axi_clk) begin
        if(axi_reset) begin cfg_meta<=0;cfg<=0;end
        else begin
            cfg_meta<={play_mode_i,tnr_enable_i,age_a_i,age_b_i,replay_div_i};
            cfg<=cfg_meta;
        end
    end

    // Stop/start recording only at a camera EOF. In-progress raw writes use
    // an invalid slot, so stopping cannot damage frames retained for playback.
    (* async_reg="true" *) reg [1:0] hold_cam;
    reg cam_vs_d,capture_on,overflow;
    reg [31:0] camera_count,camera_length;
    reg camera_bad;
    wire wf_full,wf_empty,wf_pempty;
    reg wf_reset;
    wire wf_pop;
    wire [127:0] wf_data;
    always @(posedge wframe_pclk or posedge axi_reset) begin
        if(axi_reset) begin
            hold_cam<=0;cam_vs_d<=0;capture_on<=0;overflow<=0;
            camera_count<=0;camera_length<=0;camera_bad<=0;
        end else begin
            hold_cam<={hold_cam[0],hold_record};cam_vs_d<=wframe_vsync;
            if(capture_on && wframe_data_en) begin
                camera_count<=camera_count+1'b1;
                if(wf_full) overflow<=1;
            end
            if(cam_vs_d && !wframe_vsync) begin
                camera_length<=camera_count;camera_bad<=overflow;
                camera_count<=0;overflow<=0;capture_on<=!hold_cam[1];
            end
        end
    end
    W0_FIFO_8 u_write_fifo(
        .wr_clk_i(wframe_pclk),.wr_en_i(capture_on && wframe_data_en && !wf_reset),
        .wdata(wframe_data[7:0]),.a_rst_i(axi_reset || wf_reset),
        .rd_clk_i(axi_clk),.rd_en_i(wf_pop),.rdata(wf_data),
        .prog_empty_o(wf_pempty),.empty_o(wf_empty),.full_o(wf_full));

    reg [SLOTS-1:0] valid_slots;
    reg [15:0] frame_sequence [0:SLOTS-1];
    reg [15:0] latest_sequence;
    reg [3:0] writer_slot,slot_a,slot_b;
    reg [2:0] mode;
    reg tnr;
    reg [7:0] replay_count;
    reg [15:0] replay_age;
    reg [31:0] source_id;
    reg ready;
    reg [7:0] errors;
    integer i;
    reg [3:0] next_writer;
    reg found_writer;
    integer k;
    always @* begin
        next_writer=writer_slot;found_writer=0;
        for(k=1;k<SLOTS;k=k+1) begin
            if(!found_writer && ((writer_slot+k)%SLOTS)!=slot_a && ((writer_slot+k)%SLOTS)!=slot_b &&
               ((writer_slot+k)%SLOTS)!=read_a && ((writer_slot+k)%SLOTS)!=read_b &&
               (!select_busy || (((writer_slot+k)%SLOTS)!=best_a && ((writer_slot+k)%SLOTS)!=best_b))) begin
                next_writer=(writer_slot+k)%SLOTS;found_writer=1;
            end
        end
    end

    reg [15:0] max_age;
    reg [4:0] retained;
    // Three cycles per slot keeps frame-age subtraction, distance and the
    // minimum comparison in separate clock periods. No 16-way serial min chain.
    reg select_busy,select_done;
    reg [1:0] select_phase;
    reg [3:0] select_index,best_a,best_b;
    reg [15:0] request_a,request_b,scan_sequence,scan_age,distance_a,distance_b;
    reg [15:0] best_distance_a,best_distance_b,scan_max;
    reg [4:0] scan_count;
    reg scan_valid;

    // Writer accepts only full frames. Each DDR burst is committed by B, not
    // merely by its last W beat. Publication happens after the final B response.
    localparam W_IDLE=0,W_ADDR=1,W_DATA=2,W_RESP=3,W_RESET=4;
    reg [2:0] ws;
    reg [31:0] write_offset;
    reg [7:0] write_beat;
    reg [3:0] reset_count,eof_wait;
    (* async_reg="true" *) reg [2:0] camera_vs;
    reg eof_pending,write_bad;
    reg [31:0] length_meta,length_sync;
    reg bad_meta,bad_sync;
    wire publish=ws==W_IDLE && eof_pending && eof_wait==0 && wf_empty &&
                 write_offset==TOTAL && length_sync==TOTAL && !write_bad && !bad_sync && !hold_record;
    assign axi_awaddr={writer_slot,{PAGE_BITS{1'b0}}}+write_offset;
    assign axi_awlen=BURST-1;
    assign axi_awvalid=ws==W_ADDR;
    assign axi_wvalid=ws==W_DATA;
    assign axi_wdata=wf_data;
    assign axi_wstrb=16'hffff;
    assign axi_wlast=write_beat==BURST-1;
    assign wf_pop=axi_wvalid && axi_wready;
    always @(posedge axi_clk) begin
        if(axi_reset) begin
            ws<=W_RESET;wf_reset<=1;reset_count<=15;write_offset<=0;write_beat<=0;
            camera_vs<=0;eof_pending<=0;eof_wait<=0;write_bad<=0;
            length_meta<=0;length_sync<=0;bad_meta<=0;bad_sync<=0;
        end else begin
            camera_vs<={camera_vs[1:0],wframe_vsync};
            length_meta<=camera_length;length_sync<=length_meta;
            bad_meta<=camera_bad;bad_sync<=bad_meta;
            if(camera_vs[2:1]==2'b10) begin eof_pending<=1;eof_wait<=8;end
            else if(eof_wait!=0) eof_wait<=eof_wait-1'b1;
            case(ws)
                W_IDLE:begin
                    wf_reset<=0;write_beat<=0;
                    if(!wf_pempty && write_offset<TOTAL) ws<=W_ADDR;
                    else if(eof_pending && eof_wait==0) begin
                        if(!wf_empty && length_sync==TOTAL && write_offset<TOTAL) ws<=W_ADDR;
                        else begin ws<=W_RESET;wf_reset<=1;reset_count<=15;end
                    end
                end
                W_ADDR:if(axi_awready) ws<=W_DATA;
                W_DATA:if(axi_wready) begin
                    if(axi_wlast) ws<=W_RESP;
                    else write_beat<=write_beat+1'b1;
                end
                W_RESP:if(axi_bvalid) begin
                    if(axi_bresp!=0) write_bad<=1;
                    write_offset<=write_offset+STRIDE;ws<=W_IDLE;
                end
                W_RESET:begin
                    if(reset_count!=0) reset_count<=reset_count-1'b1;
                    else begin
                        wf_reset<=0;eof_pending<=0;write_bad<=0;write_offset<=0;ws<=W_IDLE;
                    end
                end
                default:ws<=W_RESET;
            endcase
        end
    end

    // Display FIFO reset follows the original EOF convention. No AXI read is
    // cancelled mid-transaction: old responses are drained before a new epoch.
    reg display_vs_d;
    reg [5:0] display_reset_count;
    wire display_reset=axi_reset || display_reset_count!=0;
    always @(posedge rframe_pclk or posedge axi_reset) begin
        if(axi_reset) begin display_vs_d<=0;display_reset_count<=63;end
        else begin
            display_vs_d<=rframe_vsync;
            if(display_vs_d && !rframe_vsync) display_reset_count<=63;
            else if(display_reset_count!=0) display_reset_count<=display_reset_count-1'b1;
        end
    end
    (* async_reg="true" *) reg [2:0] display_reset_sync;
    wire frame_tick=display_reset_sync[2:1]==2'b10;
    always @(posedge axi_clk) begin
        if(axi_reset) display_reset_sync<=7;
        else display_reset_sync<={display_reset_sync[1:0],display_reset};
    end
    wire rf_full,rf_empty;
    wire [7:0] rf_data;
    reg rf_push;
    reg [127:0] rf_word;
    R0_FIFO_8 u_display_fifo(
        .wr_clk_i(axi_clk),.wr_en_i(rf_push && !display_reset_sync[2]),.wdata(rf_word),
        .a_rst_i(display_reset),.prog_full_o(rf_full),
        .rd_clk_i(rframe_pclk),.rd_en_i(rframe_data_en && !rf_empty),
        .rdata(rf_data),.empty_o(rf_empty));
    assign rframe_data=rf_empty?8'd0:rf_data;

    reg [15:0] chosen_age;
    always @* begin
        chosen_age=(cfg[19:17]==0)?0:{12'd0,cfg[15:12]};
        if(cfg[19:17]==2) chosen_age=(mode==2)?replay_age:max_age;
    end
    always @(posedge axi_clk) begin
        if(axi_reset) begin
            valid_slots<=0;latest_sequence<=0;writer_slot<=0;slot_a<=14;slot_b<=15;
            mode<=0;tnr<=0;replay_age<=0;replay_count<=0;source_id<=0;ready<=0;errors<=0;
            select_busy<=0;select_done<=0;select_phase<=0;select_index<=0;
            best_a<=14;best_b<=15;request_a<=0;request_b<=1;scan_sequence<=0;
            scan_age<=0;distance_a<=0;distance_b<=0;best_distance_a<=16'hffff;best_distance_b<=16'hffff;
            max_age<=0;retained<=0;scan_max<=0;scan_count<=0;scan_valid<=0;
            for(i=0;i<SLOTS;i=i+1) frame_sequence[i]<=0;
        end else begin
            if(publish) begin
                frame_sequence[writer_slot]<=latest_sequence+1'b1;
                latest_sequence<=latest_sequence+1'b1;
                valid_slots[writer_slot]<=1;
                writer_slot<=next_writer;valid_slots[next_writer]<=0;
            end
            if(ws==W_IDLE && eof_pending && eof_wait==0 && wf_empty && write_offset!=0 && !publish && !hold_record)
                errors[0]<=1;
            if(ws==W_RESP && axi_bvalid && axi_bresp!=0) errors[1]<=1;
            if(axi_rvalid && axi_rresp!=0) errors[2]<=1;
            if(frame_tick) begin
                mode<=cfg[19:17];tnr<=cfg[16];
                select_busy<=1;select_done<=0;select_phase<=0;select_index<=0;
                best_a<=slot_a;best_b<=slot_b;scan_max<=0;scan_count<=0;
                best_distance_a<=16'hffff;best_distance_b<=16'hffff;
                request_a<=chosen_age;
                request_b<=(cfg[19:17]>=3)?{12'd0,cfg[11:8]}:chosen_age+1'b1;
                scan_sequence<=latest_sequence;
                if(cfg[19:17]==2) begin
                    if(mode!=2) begin replay_age<=max_age;replay_count<=0;end
                    else if(replay_count+1 >= ((cfg[7:0]==0)?1:cfg[7:0])) begin
                        replay_count<=0;replay_age<=(replay_age==0)?max_age:replay_age-1'b1;
                    end else replay_count<=replay_count+1'b1;
                end else begin replay_count<=0;replay_age<=max_age;end
            end else if(select_busy) begin
                case(select_phase)
                    0:begin
                        scan_age<=scan_sequence-frame_sequence[select_index];
                        scan_valid<=valid_slots[select_index];select_phase<=1;
                    end
                    1:begin
                        distance_a<=(scan_age>request_a)?scan_age-request_a:request_a-scan_age;
                        distance_b<=(scan_age>request_b)?scan_age-request_b:request_b-scan_age;
                        select_phase<=2;
                    end
                    2:begin
                        if(scan_valid && valid_slots[select_index]) begin
                            scan_count<=scan_count+1'b1;
                            if(scan_age>scan_max) scan_max<=scan_age;
                            if(distance_a<best_distance_a) begin best_distance_a<=distance_a;best_a<=select_index;end
                            if(distance_b<best_distance_b) begin best_distance_b<=distance_b;best_b<=select_index;end
                        end
                        if(select_index==SLOTS-1) select_phase<=3;
                        else begin select_index<=select_index+1'b1;select_phase<=0;end
                    end
                    3:begin
                        slot_a<=best_a;slot_b<=best_b;source_id<={frame_sequence[best_b],frame_sequence[best_a]};
                        ready<=scan_count!=0;retained<=scan_count;max_age<=scan_max;
                        select_busy<=0;select_done<=1;
                    end
                endcase
            end
        end
    end

    // Two short burst buffers, not complete on-chip frames. Their synchronous
    // read ports feed the mixer and display FIFO. The full frame lives in DDR.
    reg [127:0] current_burst[0:BURST-1],reference_burst[0:BURST-1];
    reg [127:0] current_q,reference_q;
    reg [7:0] receive_beat,emit_beat;
    localparam R_WAIT=0,R_CUR_ADDR=1,R_CUR_DATA=2,R_REF_ADDR=3,R_REF_DATA=4,R_LOAD=5,R_EMIT=6,
               R_MIX1=7,R_MIX2=8,R_MIX3=9;
    reg [3:0] rs;
    reg [31:0] read_offset;
    reg [3:0] read_a,read_b;
    reg [2:0] read_mode;
    reg read_tnr;
    reg [15:0] column;
    reg frame_pending,abort_frame;
    wire want_reference=read_mode>=3 || (read_tnr && read_a!=read_b);
    assign axi_arid=0;
    assign axi_arlen=BURST-1;
    assign axi_araddr=((rs==R_REF_ADDR)?{read_b,{PAGE_BITS{1'b0}}}:{read_a,{PAGE_BITS{1'b0}}})+read_offset;
    assign axi_arvalid=rs==R_CUR_ADDR || rs==R_REF_ADDR;
    assign axi_rready=1;
    always @(posedge axi_clk) begin
        if(rs==R_CUR_DATA && axi_rvalid) current_burst[receive_beat]<=axi_rdata;
        if(rs==R_REF_DATA && axi_rvalid) reference_burst[receive_beat]<=axi_rdata;
        current_q<=current_burst[emit_beat];reference_q<=reference_burst[emit_beat];
    end
    wire [127:0] mixed;
    frame_pair_mix #(.FRAME_W(FRAME_W)) mix(
        .clk_i(axi_clk),
        .current_i(current_q),.reference_i(reference_q),.column_i(column),
        .mode_i(read_mode),.tnr_i(read_tnr && read_a!=read_b),.data_o(mixed));
    always @(posedge axi_clk) begin
        if(axi_reset) begin
            rs<=R_WAIT;read_offset<=0;receive_beat<=0;emit_beat<=0;column<=0;
            frame_pending<=1;abort_frame<=1;rf_push<=0;rf_word<=0;
            read_a<=0;read_b<=0;read_mode<=0;read_tnr<=0;
        end else begin
            rf_push<=0;
            if(display_reset_sync[2]) abort_frame<=1;
            if(frame_tick) frame_pending<=1;
            case(rs)
                R_WAIT:begin
                    if(frame_pending && !display_reset_sync[2] && !select_busy && select_done) begin
                        read_offset<=0;column<=0;frame_pending<=0;abort_frame<=0;
                        read_a<=slot_a;read_b<=slot_b;read_mode<=mode;read_tnr<=tnr;
                    end else if(!abort_frame && ready && !rf_full && read_offset<TOTAL) begin
                        receive_beat<=0;rs<=R_CUR_ADDR;
                    end
                end
                R_CUR_ADDR:if(axi_arready) begin receive_beat<=0;rs<=R_CUR_DATA;end
                R_CUR_DATA:if(axi_rvalid) begin
                    if(axi_rlast) begin
                        if(abort_frame || display_reset_sync[2]) rs<=R_WAIT;
                        else if(want_reference) begin receive_beat<=0;rs<=R_REF_ADDR;end
                        else begin emit_beat<=0;rs<=R_LOAD;end
                    end else receive_beat<=receive_beat+1'b1;
                end
                R_REF_ADDR:if(axi_arready) begin receive_beat<=0;rs<=R_REF_DATA;end
                R_REF_DATA:if(axi_rvalid) begin
                    if(axi_rlast) begin
                        if(abort_frame || display_reset_sync[2]) rs<=R_WAIT;
                        else begin emit_beat<=0;rs<=R_LOAD;end
                    end else receive_beat<=receive_beat+1'b1;
                end
                R_LOAD:begin
                    if(abort_frame || display_reset_sync[2]) rs<=R_WAIT;
                    else rs<=R_MIX1;
                end
                R_MIX1:rs<=R_MIX2;
                R_MIX2:rs<=R_MIX3;
                R_MIX3:rs<=R_EMIT;
                R_EMIT:begin
                    if(abort_frame || display_reset_sync[2]) rs<=R_WAIT;
                    else if(!rf_full) begin
                        rf_word<=mixed;rf_push<=1;
                        column<=(column>=FRAME_W-16)?0:column+16;
                        if(emit_beat==BURST-1) begin read_offset<=read_offset+STRIDE;rs<=R_WAIT;end
                        else begin emit_beat<=emit_beat+1'b1;rs<=R_LOAD;end
                    end
                end
                default:rs<=R_WAIT;
            endcase
        end
    end
    assign tp_o={20'd0,ready,tnr,mode,ws,rs};
    assign status_o={errors,retained,mode,source_id[15:0]};
    assign display_source_o=frame_sequence[read_a];
endmodule

// Bundled-data request/ack mailbox. Status is latched atomically in the DDR
// domain, then held until the pixel-domain receiver acknowledges a new query.
module frame_status_mailbox(
    input wire src_clk,src_rst,input wire [31:0] src_status,
    input wire dst_clk,dst_rst,request_i,
    output reg valid_o,output reg [31:0] status_o
);
    reg request_toggle,ack_toggle;
    (* async_reg="true" *) reg [1:0] request_sync,ack_sync;
    reg [31:0] held_status;
    reg received_toggle;
    always @(posedge src_clk or posedge src_rst) begin
        if(src_rst) begin request_sync<=0;ack_toggle<=0;held_status<=0;end
        else begin
            request_sync<={request_sync[0],request_toggle};
            if(request_sync[1]!=ack_toggle) begin
                held_status<=src_status;ack_toggle<=request_sync[1];
            end
        end
    end
    always @(posedge dst_clk or posedge dst_rst) begin
        if(dst_rst) begin request_toggle<=0;ack_sync<=0;received_toggle<=0;valid_o<=0;status_o<=0;end
        else begin
            ack_sync<={ack_sync[0],ack_toggle};valid_o<=0;
            if(request_i && request_toggle==ack_sync[1]) request_toggle<=~request_toggle;
            if(ack_sync[1]!=received_toggle) begin
                status_o<=held_status;valid_o<=1;received_toggle<=ack_sync[1];
            end
        end
    end
endmodule

// Bayer RAW8-domain motion-adaptive two-frame FIR. A 16-pixel DC difference
// estimate prevents a small uniform brightness change from disabling blending.
// Large absolute changes always bypass history; first/same frame is bypassed.
module frame_pair_mix #(parameter FRAME_W=1280)(
    input wire clk_i,
    input wire [127:0] current_i,reference_i,
    input wire [15:0] column_i,input wire [2:0] mode_i,input wire tnr_i,
    output reg [127:0] data_o
);
    reg signed [8:0] delta1[0:15],delta2[0:15];
    reg signed [10:0] partial[0:3];
    reg signed [8:0] dc;
    reg [127:0] c1,r1,c2,r2;
    wire signed [11:0] half0=$signed(partial[0])+$signed(partial[1]);
    wire signed [11:0] half1=$signed(partial[2])+$signed(partial[3]);
    wire signed [12:0] total=$signed(half0)+$signed(half1);
    // Signed division truncates towards zero, as does the independent model.
    wire signed [12:0] mean=total/13'sd16;
    integer j,g,group_sum,diff,residual,absolute,c,r,v;
    always @(posedge clk_i) begin
        c1<=current_i;r1<=reference_i;c2<=c1;r2<=r1;
        for(j=0;j<16;j=j+1) begin
            delta1[j]<=$signed({1'b0,current_i[j*8+:8]})-$signed({1'b0,reference_i[j*8+:8]});
            delta2[j]<=delta1[j];
        end
        for(g=0;g<4;g=g+1) begin
            group_sum=0;
            for(j=0;j<4;j=j+1)
                group_sum=group_sum+$signed({1'b0,current_i[(g*4+j)*8+:8]})-$signed({1'b0,reference_i[(g*4+j)*8+:8]});
            partial[g]<=group_sum;
        end
        dc<=mean[8:0];
        for(j=0;j<16;j=j+1) begin
            c=c2[j*8+:8];r=r2[j*8+:8];diff=$signed(delta2[j]);
            residual=(diff>dc)?diff-dc:dc-diff;absolute=(diff<0)?-diff:diff;
            v=c;
            if(mode_i==3) v=(column_i+j>=FRAME_W/2)?r:c;
            else if(mode_i==4) begin v=absolute*4;if(v>255) v=255;end
            else if(tnr_i && residual<=4 && absolute<=24) v=(c+r+1)/2;
            else if(tnr_i && residual<=8 && absolute<=24) v=(3*c+r+2)/4;
            data_o[j*8+:8]<=v[7:0];
        end
    end
endmodule
