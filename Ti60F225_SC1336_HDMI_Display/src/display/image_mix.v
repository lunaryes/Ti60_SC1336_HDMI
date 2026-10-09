`timescale 1ns / 1ps
//=====================================================================
// image_mix  —— 接口 I-C
//---------------------------------------------------------------------
// 把原图与边缘图合成为一路 RGB888。
//
//   mode_i = 0  原图
//   mode_i = 1  边缘图
//   mode_i = 2  左半原图 + 右半边缘图
//   mode_i = 3  画中画（边缘图缩至右下 1/4）—— 本轮未实现，暂回落为边缘图
//
// 【输入对齐前置条件】
//   orig_rgb_i 与 edge_rgb_i 必须**同拍**。对齐由 image_pipeline_top 用
//   sync_delay_align 保证，本模块不做任何延迟补偿。
//
// 【时序】输出寄存 1 拍，延迟固定为 1。vsync_o / de_o 同步寄存，与 rgb_o 同拍。
//
// 【参数化】mode 2 用输入列计数器比较 IMG_W/2，IMG_W / IMG_H 必须参数化。
//=====================================================================
module image_mix #(
    parameter IMG_W = 1280,
    parameter IMG_H = 720
)(
    input  wire        clk_i,
    input  wire        rst_ni,
    input  wire        vsync_i,
    input  wire        de_i,
    input  wire [23:0] orig_rgb_i,   // 原图，与边缘图同拍
    input  wire [23:0] edge_rgb_i,   // 边缘图
    input  wire [1:0]  mode_i,

    output reg         vsync_o,
    output reg         de_o,
    output reg  [23:0] rgb_o
);

    localparam AW        = 11;                       // IMG_W <= 2**AW
    localparam [AW-1:0] COL_ZERO = {AW{1'b0}};
    localparam [AW-1:0] COL_HALF = IMG_W >> 1;       // 分屏界线

    //-----------------------------------------------------------------
    // 列计数：DE 有效期内 0 .. IMG_W-1
    //-----------------------------------------------------------------
    reg [AW-1:0] col;
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni)
            col <= COL_ZERO;
        else if (!de_i)
            col <= COL_ZERO;
        else if (col == (IMG_W - 1))
            col <= COL_ZERO;
        else
            col <= col + 1'b1;
    end

    //-----------------------------------------------------------------
    // 合成 + 输出寄存 1 拍
    //-----------------------------------------------------------------
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            vsync_o <= 1'b0;
            de_o    <= 1'b0;
            rgb_o   <= 24'h000000;
        end else begin
            vsync_o <= vsync_i;
            de_o    <= de_i;

            if (!de_i)
                rgb_o <= 24'h000000;
            else
                case (mode_i)
                    2'd0    : rgb_o <= orig_rgb_i;                            // 原图
                    2'd1    : rgb_o <= edge_rgb_i;                            // 边缘图
                    2'd2    : rgb_o <= (col < COL_HALF) ? orig_rgb_i          // 左半原图
                                                        : edge_rgb_i;         // 右半边缘图
                    default : rgb_o <= edge_rgb_i;                            // mode 3 待实现
                endcase
        end
    end

endmodule
