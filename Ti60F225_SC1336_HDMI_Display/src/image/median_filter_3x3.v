`timescale 1ns / 1ps

// 3x3 median filter.
// The nine inputs come from pix_window_3x3.  An odd-even sorting network
// orders them combinationally; stage9[4] is therefore the median value.
module median_filter_3x3 (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       vsync_i,
    input  wire       de_i,
    input  wire [7:0] p11,
    input  wire [7:0] p12,
    input  wire [7:0] p13,
    input  wire [7:0] p21,
    input  wire [7:0] p22,
    input  wire [7:0] p23,
    input  wire [7:0] p31,
    input  wire [7:0] p32,
    input  wire [7:0] p33,

    output reg        vsync_o,
    output reg        de_o,
    output reg  [7:0] gray_o
);

    function [7:0] min8;
        input [7:0] a;
        input [7:0] b;
        begin
            min8 = (a < b) ? a : b;
        end
    endfunction

    function [7:0] max8;
        input [7:0] a;
        input [7:0] b;
        begin
            max8 = (a < b) ? b : a;
        end
    endfunction

    wire [7:0] stage0 [0:8];
    wire [7:0] stage1 [0:8];
    wire [7:0] stage2 [0:8];
    wire [7:0] stage3 [0:8];
    wire [7:0] stage4 [0:8];
    wire [7:0] stage5 [0:8];
    wire [7:0] stage6 [0:8];
    wire [7:0] stage7 [0:8];
    wire [7:0] stage8 [0:8];
    wire [7:0] stage9 [0:8];

    assign stage0[0] = p11;
    assign stage0[1] = p12;
    assign stage0[2] = p13;
    assign stage0[3] = p21;
    assign stage0[4] = p22;
    assign stage0[5] = p23;
    assign stage0[6] = p31;
    assign stage0[7] = p32;
    assign stage0[8] = p33;

    // Nine odd-even transposition phases sort all nine pixels.
    assign stage1[0] = min8(stage0[0], stage0[1]);
    assign stage1[1] = max8(stage0[0], stage0[1]);
    assign stage1[2] = min8(stage0[2], stage0[3]);
    assign stage1[3] = max8(stage0[2], stage0[3]);
    assign stage1[4] = min8(stage0[4], stage0[5]);
    assign stage1[5] = max8(stage0[4], stage0[5]);
    assign stage1[6] = min8(stage0[6], stage0[7]);
    assign stage1[7] = max8(stage0[6], stage0[7]);
    assign stage1[8] = stage0[8];

    assign stage2[0] = stage1[0];
    assign stage2[1] = min8(stage1[1], stage1[2]);
    assign stage2[2] = max8(stage1[1], stage1[2]);
    assign stage2[3] = min8(stage1[3], stage1[4]);
    assign stage2[4] = max8(stage1[3], stage1[4]);
    assign stage2[5] = min8(stage1[5], stage1[6]);
    assign stage2[6] = max8(stage1[5], stage1[6]);
    assign stage2[7] = min8(stage1[7], stage1[8]);
    assign stage2[8] = max8(stage1[7], stage1[8]);

    assign stage3[0] = min8(stage2[0], stage2[1]);
    assign stage3[1] = max8(stage2[0], stage2[1]);
    assign stage3[2] = min8(stage2[2], stage2[3]);
    assign stage3[3] = max8(stage2[2], stage2[3]);
    assign stage3[4] = min8(stage2[4], stage2[5]);
    assign stage3[5] = max8(stage2[4], stage2[5]);
    assign stage3[6] = min8(stage2[6], stage2[7]);
    assign stage3[7] = max8(stage2[6], stage2[7]);
    assign stage3[8] = stage2[8];

    assign stage4[0] = stage3[0];
    assign stage4[1] = min8(stage3[1], stage3[2]);
    assign stage4[2] = max8(stage3[1], stage3[2]);
    assign stage4[3] = min8(stage3[3], stage3[4]);
    assign stage4[4] = max8(stage3[3], stage3[4]);
    assign stage4[5] = min8(stage3[5], stage3[6]);
    assign stage4[6] = max8(stage3[5], stage3[6]);
    assign stage4[7] = min8(stage3[7], stage3[8]);
    assign stage4[8] = max8(stage3[7], stage3[8]);

    assign stage5[0] = min8(stage4[0], stage4[1]);
    assign stage5[1] = max8(stage4[0], stage4[1]);
    assign stage5[2] = min8(stage4[2], stage4[3]);
    assign stage5[3] = max8(stage4[2], stage4[3]);
    assign stage5[4] = min8(stage4[4], stage4[5]);
    assign stage5[5] = max8(stage4[4], stage4[5]);
    assign stage5[6] = min8(stage4[6], stage4[7]);
    assign stage5[7] = max8(stage4[6], stage4[7]);
    assign stage5[8] = stage4[8];

    assign stage6[0] = stage5[0];
    assign stage6[1] = min8(stage5[1], stage5[2]);
    assign stage6[2] = max8(stage5[1], stage5[2]);
    assign stage6[3] = min8(stage5[3], stage5[4]);
    assign stage6[4] = max8(stage5[3], stage5[4]);
    assign stage6[5] = min8(stage5[5], stage5[6]);
    assign stage6[6] = max8(stage5[5], stage5[6]);
    assign stage6[7] = min8(stage5[7], stage5[8]);
    assign stage6[8] = max8(stage5[7], stage5[8]);

    assign stage7[0] = min8(stage6[0], stage6[1]);
    assign stage7[1] = max8(stage6[0], stage6[1]);
    assign stage7[2] = min8(stage6[2], stage6[3]);
    assign stage7[3] = max8(stage6[2], stage6[3]);
    assign stage7[4] = min8(stage6[4], stage6[5]);
    assign stage7[5] = max8(stage6[4], stage6[5]);
    assign stage7[6] = min8(stage6[6], stage6[7]);
    assign stage7[7] = max8(stage6[6], stage6[7]);
    assign stage7[8] = stage6[8];

    assign stage8[0] = stage7[0];
    assign stage8[1] = min8(stage7[1], stage7[2]);
    assign stage8[2] = max8(stage7[1], stage7[2]);
    assign stage8[3] = min8(stage7[3], stage7[4]);
    assign stage8[4] = max8(stage7[3], stage7[4]);
    assign stage8[5] = min8(stage7[5], stage7[6]);
    assign stage8[6] = max8(stage7[5], stage7[6]);
    assign stage8[7] = min8(stage7[7], stage7[8]);
    assign stage8[8] = max8(stage7[7], stage7[8]);

    assign stage9[0] = min8(stage8[0], stage8[1]);
    assign stage9[1] = max8(stage8[0], stage8[1]);
    assign stage9[2] = min8(stage8[2], stage8[3]);
    assign stage9[3] = max8(stage8[2], stage8[3]);
    assign stage9[4] = min8(stage8[4], stage8[5]);
    assign stage9[5] = max8(stage8[4], stage8[5]);
    assign stage9[6] = min8(stage8[6], stage8[7]);
    assign stage9[7] = max8(stage8[6], stage8[7]);
    assign stage9[8] = stage8[8];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_o <= 1'b0;
            de_o    <= 1'b0;
            gray_o  <= 8'd0;
        end else begin
            vsync_o <= vsync_i;
            de_o    <= de_i;
            gray_o  <= de_i ? stage9[4] : 8'd0;
        end
    end

endmodule
