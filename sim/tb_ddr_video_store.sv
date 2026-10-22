`timescale 1ns/1ps
module tb_ddr_video_store;
    reg clk=0,cam=0,pix=0;
    always #5 clk=~clk;
    always #9 cam=~cam;
    always #13 pix=~pix;
    reg rst=1,vs=0,de=0,pvs=1,pde=0;
    reg [7:0] data=0;
    reg [2:0] mode=0;
    reg tnr=0;
    reg [3:0] age_a=0,age_b=1;
    reg [7:0] divider=2;
    reg mailbox_request=0;
    wire mailbox_valid;
    wire [31:0] mailbox_result;
    reg [15:0] mailbox_counter=0;
    wire [31:0] mailbox_source={~mailbox_counter,mailbox_counter};
    integer mailbox_checks=0;
    frame_status_mailbox u_mailbox(.src_clk(clk),.src_rst(rst),.src_status(mailbox_source),
        .dst_clk(pix),.dst_rst(rst),.request_i(mailbox_request),
        .valid_o(mailbox_valid),.status_o(mailbox_result));
    always @(posedge clk) if(!rst) mailbox_counter<=mailbox_counter+1'b1;
    initial begin
        wait(!rst);
        repeat(100) begin
            repeat(3) @(negedge pix);
            mailbox_request=1;@(negedge pix);mailbox_request=0;
            wait(mailbox_valid);@(negedge pix);
            if((mailbox_result[31:16]^mailbox_result[15:0])!=16'hffff)
                $fatal(1,"non-atomic status mailbox");
            mailbox_checks=mailbox_checks+1;
        end
    end
    wire [31:0] awaddr,araddr,status;
    wire [7:0] awlen,arlen;
    wire awvalid,wvalid,wlast,arvalid,rready;
    reg awready=0,wready=0,arready=0,bvalid=0,rvalid=0,rlast=0;
    wire [127:0] wdata;
    reg [127:0] rdata=0;
    wire [7:0] pixel;
    wire [15:0] mask;
    wire [3:0] rid;
    ddr_video_store #(.C_RD_END_ADDR(512),.FRAME_W(32),.BURST(4),.PAGE_BITS(10)) dut(
        .axi_clk(clk),.axi_reset(rst),.axi_awaddr(awaddr),.axi_awlen(awlen),
        .axi_awvalid(awvalid),.axi_awready(awready),.axi_wdata(wdata),.axi_wstrb(mask),
        .axi_wlast(wlast),.axi_wvalid(wvalid),.axi_wready(wready),
        .axi_bid(4'd0),.axi_bresp(2'd0),.axi_bvalid(bvalid),
        .axi_arid(rid),.axi_araddr(araddr),.axi_arlen(arlen),.axi_arvalid(arvalid),.axi_arready(arready),
        .axi_rid(4'd0),.axi_rdata(rdata),.axi_rresp(2'd0),.axi_rlast(rlast),.axi_rvalid(rvalid),.axi_rready(rready),
        .wframe_pclk(cam),.wframe_vsync(vs),.wframe_data_en(de),.wframe_data(data),
        .rframe_pclk(pix),.rframe_vsync(pvs),.rframe_data_en(pde),.rframe_data(pixel),
        .play_mode_i(mode),.tnr_enable_i(tnr),.age_a_i(age_a),.age_b_i(age_b),.replay_div_i(divider),
        .tp_o(),.status_o(status));

    byte memory[0:16383];
    integer wa=0,ra=0,wb=0,rb=0,bdelay=0,cycle=0,j;
    reg writing=0,reading=0;
    reg [31:0] stalled_ar;
    reg ar_stalled=0;
    always @(negedge clk) begin
        cycle=cycle+1;
        awready=!writing && bdelay==0 && !bvalid && cycle%3!=0;
        wready=writing && cycle%4!=0;
        arready=!reading && cycle%5!=0;
    end
    always @(posedge clk) begin
        if(rst) begin writing=0;reading=0;bvalid<=0;rvalid<=0;rlast<=0;bdelay=0;ar_stalled=0;end
        else begin
            if(ar_stalled && (!arvalid || araddr!=stalled_ar)) $fatal(1,"AR changed under backpressure");
            ar_stalled=arvalid && !arready;stalled_ar=araddr;
            if(awvalid && awready) begin wa=awaddr;wb=0;writing=1;end
            if(wvalid && wready) begin
                for(j=0;j<16;j=j+1) memory[wa+wb*16+j]=wdata[j*8+:8];
                if(wlast!=(wb==3)) $fatal(1,"bad WLAST");
                if(wlast) begin writing=0;bdelay=7;end
                else wb=wb+1;
            end
            if(bdelay>0) begin bdelay=bdelay-1;if(bdelay==0) bvalid<=1;end
            else if(bvalid) bvalid<=0;
            if(arvalid && arready) begin ra=araddr;rb=0;reading=1;end
            if(rvalid && rready) begin
                rvalid<=0;
                if(rlast) begin reading=0;rlast<=0;end else rb=rb+1;
            end
            if(reading && !rvalid && cycle%3!=0) begin
                for(j=0;j<16;j=j+1) rdata[j*8+:8]<=memory[ra+rb*16+j];
                rvalid<=1;rlast<=rb==3;
            end
        end
    end
    integer f,x;
    initial begin
        repeat(8) @(negedge cam);rst=0;
        forever begin
            for(f=1;f<1000;f=f+1) begin
                @(negedge cam);vs=1;
                for(x=0;x<512;x=x+1) begin de=1;data=(f*8+x)%256;@(negedge cam);end
                de=0;vs=0;repeat(200) @(negedge cam);
            end
        end
    end
    integer p;
    initial begin
        wait(!rst);
        forever begin
            pvs=0;repeat(100) @(negedge pix);pvs=1;
            repeat(20) @(negedge pix);
            for(p=0;p<512;p=p+1) begin pde=1;@(negedge pix);end
            pde=0;repeat(70) @(negedge pix);
        end
    end
    reg [15:0] frozen_id;
    reg [3:0] frozen_slot;
    reg [7:0] first_byte;
    integer checked=0,steps=0;
    initial begin
        wait(!rst);wait(dut.latest_sequence>=20);
        @(negedge pix);mode=1;
        wait(dut.mode==1);repeat(3000) @(posedge clk);
        frozen_id=status[15:0];frozen_slot=dut.slot_a;first_byte=memory[frozen_slot*1024];
        repeat(10000) begin
            @(posedge clk);
            if(status[15:0]!=frozen_id || dut.slot_a!=frozen_slot) $fatal(1,"freeze source changed");
            if(memory[frozen_slot*1024]!=first_byte) $fatal(1,"frozen frame overwritten");
        end
        if(dut.retained<10) $fatal(1,"insufficient replay frames");
        @(negedge pix);age_a=3;
        wait(status[15:0]!=frozen_id);frozen_id=status[15:0];
        @(negedge pix);mode=2;
        repeat(12000) begin @(posedge clk);if(status[15:0]!=frozen_id) begin frozen_id=status[15:0];steps=steps+1;end end
        if(steps<2) $fatal(1,"playback did not advance");
        @(negedge pix);mode=3;age_a=0;age_b=1;
        wait(dut.mode==3);repeat(3000) @(posedge clk);
        @(negedge pix);mode=4;wait(dut.mode==4);repeat(3000) @(posedge clk);
        @(negedge pix);mode=0;tnr=1;
        wait(dut.mode==0);wait(dut.latest_sequence>25);repeat(3000) @(posedge clk);
        if(status[31:24]!=0) $fatal(1,"store errors %h",status);
        if(mailbox_checks!=100) $fatal(1,"status mailbox test incomplete");
        $display("DDR_VIDEO_STORE_PASS: freeze, frame selection, playback, comparison, live restore, AXI backpressure");$finish;
    end
    // Every FIFO output burst must contain the selected committed frame data.
    integer offset,expected,pa,pb,dc,total,c,r,delta,residual,v;
    reg [127:0] expected_word;
    reg check_pending=0;
    always @(posedge clk) if(!rst) begin
        if(dut.rf_push && !dut.display_reset_sync[2]) begin
            if(!check_pending || dut.rf_word!==expected_word) $fatal(1,"DDR pixel mismatch, mode %0d",dut.read_mode);
            checked=checked+1;
        end
        check_pending<=0;
        if(dut.rs==6 && !dut.abort_frame && !dut.display_reset_sync[2] && !dut.rf_full) begin
            offset=dut.read_offset+dut.emit_beat*16;
            total=0;
            for(pa=0;pa<16;pa=pa+1) total=total+int'(unsigned'(memory[dut.read_a*1024+offset+pa]))-int'(unsigned'(memory[dut.read_b*1024+offset+pa]));
            dc=total/16;
            for(pa=0;pa<16;pa=pa+1) begin
                c=unsigned'(memory[dut.read_a*1024+offset+pa]);r=unsigned'(memory[dut.read_b*1024+offset+pa]);
                delta=c-r;residual=(delta>=dc)?delta-dc:dc-delta;
                v=c;
                if(dut.read_mode==3) v=((offset+pa)%32>=16)?r:c;
                else if(dut.read_mode==4) begin v=((delta<0)?-delta:delta)*4;if(v>255)v=255;end
                else if(dut.read_tnr && dut.read_a!=dut.read_b) begin
                    if(residual<=4 && delta<=24 && delta>=-24) v=(c+r+1)/2;
                    else if(residual<=8 && delta<=24 && delta>=-24) v=(3*c+r+2)/4;
                end
                expected_word[pa*8+:8]=v[7:0];
            end
            check_pending<=1;
        end
    end
    initial begin #20000000;$fatal(1,"test timeout");end
endmodule

// Behavioral FIFO models only for simulation. Production uses the existing
// generated dual-clock width-converting FIFO IP, without editing vendor files.
module W0_FIFO_8(input wr_clk_i,wr_en_i,input [7:0] wdata,input a_rst_i,
    input rd_clk_i,rd_en_i,output reg [127:0] rdata,
    output wire prog_empty_o,empty_o,full_o);
    byte q[$];byte discarded;
    integer n;
    assign empty_o=q.size()<16;
    assign prog_empty_o=q.size()<64;
    assign full_o=q.size()>8000;
    always @* begin rdata=0;for(integer z=0;z<16;z=z+1) if(z<q.size()) rdata[z*8+:8]=q[z];end
    always @(posedge wr_clk_i or posedge a_rst_i) begin
        if(a_rst_i) q.delete();else if(wr_en_i) q.push_back(wdata);
    end
    always @(posedge rd_clk_i) if(!a_rst_i && rd_en_i) begin
        if(q.size()<16) $fatal(1,"write FIFO underflow");
        for(n=0;n<16;n=n+1) discarded=q.pop_front();
    end
endmodule

module R0_FIFO_8(input wr_clk_i,wr_en_i,input [127:0] wdata,input a_rst_i,
    output wire prog_full_o,input rd_clk_i,rd_en_i,output wire [7:0] rdata,output wire empty_o);
    byte q[$];byte discarded;
    assign prog_full_o=q.size()>512;
    assign empty_o=q.size()==0;
    assign rdata=q.size()?q[0]:0;
    always @(posedge wr_clk_i or posedge a_rst_i) begin
        if(a_rst_i) q.delete();else if(wr_en_i) for(integer n=0;n<16;n=n+1) q.push_back(wdata[n*8+:8]);
    end
    always @(posedge rd_clk_i) if(!a_rst_i && rd_en_i) begin
        if(q.size()==0) $fatal(1,"read FIFO underflow");
        discarded=q.pop_front();
    end
endmodule
