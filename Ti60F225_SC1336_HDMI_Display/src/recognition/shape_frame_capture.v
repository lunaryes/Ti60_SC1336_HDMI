`timescale 1ns/1ps
// 8x8 OR pooling preserves thin Canny contours. A complete frame is captured
// only while the geometry engine is idle. No backpressure enters the video
// pipeline: frames may be skipped for analysis, but never for HDMI display.
module shape_frame_capture #(
    parameter integer IMG_W=1280,IMG_H=720,SCALE_LOG2=3,
    parameter integer W=IMG_W>>SCALE_LOG2,H=IMG_H>>SCALE_LOG2,
    parameter integer AW=$clog2(W*H)
)(
    input wire clk,rst_n,vsync_i,de_i,edge_i,busy_i,
    output reg frame_ready_o,
    input wire [AW-1:0] read_addr_i,output wire read_data_o
);
    localparam integer SCALE=1<<SCALE_LOG2;
    reg [10:0] x;reg [9:0] y;
    reg vs_d,de_d,capturing,horizontal_or;
    reg vertical_or[0:W-1];
    reg write_en;reg [AW-1:0] write_addr;reg write_bit;
    wire fs=vsync_i&&!vs_d;
    wire [10:0] bx=x>>SCALE_LOG2;
    wire [9:0] by=y>>SCALE_LOG2;
    wire block_end=(x&(SCALE-1))==SCALE-1;
    wire stripe_first=(y&(SCALE-1))==0;
    wire stripe_last=(y&(SCALE-1))==SCALE-1;
    wire pooled=horizontal_or|edge_i;
    shape_sdp_ram #(.WIDTH(1),.DEPTH(W*H),.AW(AW)) u_frame(
        .clk(clk),.we(write_en),.wa(write_addr),.wd(write_bit),.ra(read_addr_i),.rd(read_data_o));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            x<=0;y<=0;vs_d<=0;de_d<=0;capturing<=0;horizontal_or<=0;
            frame_ready_o<=0;write_en<=0;write_addr<=0;write_bit<=0;
        end else begin
            vs_d<=vsync_i;de_d<=de_i;write_en<=0;frame_ready_o<=0;
            if(de_i) x<=x==IMG_W-1?0:x+1'b1;else x<=0;
            if(de_d&&!de_i) y<=y+1'b1;
            if(!de_i) horizontal_or<=0;
            if(capturing&&de_i) begin
                horizontal_or<=pooled;
                if(block_end) begin
                    horizontal_or<=0;
                    vertical_or[bx]<=stripe_first?pooled:(vertical_or[bx]|pooled);
                    if(stripe_last) begin
                        write_en<=1;write_addr<=by*W+bx;
                        write_bit<=vertical_or[bx]|pooled;
                    end
                end
                if(x==IMG_W-1&&y==IMG_H-1) capturing<=0;
            end
            // write_en was registered on the final pixel; publish only after
            // that last RAM write has taken place.
            if(write_en&&write_addr==W*H-1) frame_ready_o<=1;
            if(fs) begin
                x<=0;y<=0;capturing<=!busy_i;horizontal_or<=0;
            end
        end
    end
endmodule
