`timescale 1ns / 1ps

// Active-video coordinate generator for the HDMI pipeline.
// vsync_i is treated as a frame-start pulse; de_i marks valid pixels.
module video_xy_counter #(
    parameter integer IMG_W = 1280
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        vsync_i,
    input  wire        de_i,
    output wire [10:0] x_o,
    output wire [9:0]  y_o
);
    reg [10:0] x_cnt;
    reg [9:0]  y_cnt;
    reg        vsync_d;
    reg        de_d;

    wire frame_start = vsync_i && !vsync_d;
    wire line_end    = de_d && !de_i;

    assign x_o = x_cnt;
    assign y_o = y_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_cnt   <= 11'd0;
            y_cnt   <= 10'd0;
            vsync_d <= 1'b0;
            de_d    <= 1'b0;
        end else begin
            vsync_d <= vsync_i;
            de_d    <= de_i;

            if (frame_start) begin
                x_cnt <= 11'd0;
                y_cnt <= 10'd0;
            end else begin
                if (de_i) begin
                    if (x_cnt == IMG_W-1)
                        x_cnt <= 11'd0;
                    else
                        x_cnt <= x_cnt + 11'd1;
                end else begin
                    x_cnt <= 11'd0;
                end

                if (line_end)
                    y_cnt <= y_cnt + 10'd1;
            end
        end
    end
endmodule
