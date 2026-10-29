`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name : image_mix
//
// 功能:
//   把原图、灰度图与边缘图合成为一路 RGB888，四种模式由 mode_i 选择：
//     mode_i = 0 : 原图
//     mode_i = 1 : 边缘图
//     mode_i = 2 : 分屏，左半灰度图 + 右半边缘图（分界列 IMG_W/2）
//     mode_i = 3 : 左半灰度图 + 右半原图叠加亮黄色边缘（默认模式）
//   三路输入必须逐像素同拍（orig/gray/edge 对应同一个像素）。
//   gray_rgb_i 是 {Y,Y,Y}（中值后的算法灰度），与 edge_rgb_i 同拍。
//
// LATENCY = 1（四种模式一致）:
//   rgb_o 是本模块唯一的输出寄存器，在时钟沿采样"当拍"的组合值，而该组合值直接来自
//   当拍输入的像素，因此输出比输入晚 1 拍。vsync_o / de_o 与 rgb_o 在同一个 always 块
//   中寄存，严格同拍。四种模式的组合路径长度相同，延迟不随 mode_i 变化。
//
// 边界策略:
//   · 只处理 de_i = 1 的像素；de_i = 0 时不清零 rgb_o（保持 mode 0 与输入逐像素一致，
//     供回归对比用），下游 FrameBoundCrop / rgb2dvi 本来就只看 de_o = 1 的像素。
//   · col_cnt 在 de_i 低电平清零，故 col_cnt 即当前像素的 0 基列号；分界取
//     col_cnt < IMG_W/2 为灰度图、否则为边缘图（IMG_W 为奇数时向下取整，左半少 1 列）。
//   · 本模块没有跨行 / 跨帧状态，复位后第一帧即为有效数据，无预热期。
//
// 参数:
//   IMG_W : 图像宽度（列数），需为偶数，2 <= IMG_W <= 2048（决定 COL_W = 11）
//   IMG_H : 图像高度（行数），本版未使用，为对齐冻结接口保留
//   约束  : 每行 de 周期数 = IMG_W（与 src/isp/FrameBoundCrop.v 的假设一致，
//           对应 lcd_para.v 的 H_DISP = 1280 / V_DISP = 720）
//
// Display-only gamma 0.70 brightens gray/RGB before yellow blending.
// mode 3 使用已对齐的边缘亮度作掩码，不增加帧缓存或流水线延迟。
//////////////////////////////////////////////////////////////////////////////////
module image_mix #(
    parameter IMG_W = 1280,
    parameter IMG_H = 720,
    parameter DISPLAY_GAMMA_ENABLE = 1
)(
    input  wire        clk_i,
    input  wire        rst_ni,
    input  wire        vsync_i,
    input  wire        de_i,
    input  wire [23:0] orig_rgb_i,   // 原图，与边缘图同拍
    input  wire [23:0] gray_rgb_i,   // 灰度图 {Y,Y,Y}，与边缘图同拍
    input  wire [23:0] edge_rgb_i,   // 边缘图
    input  wire [1:0]  mode_i,
    output reg         vsync_o,
    output reg         de_o,
    output reg  [23:0] rgb_o
);

    //------------------------------------------------------------------
    // 常量（端口已冻结，宽度用 localparam 给出，不新增 parameter）
    //------------------------------------------------------------------
    localparam HALF_W = IMG_W >> 1;   // 左/右分界列
    localparam COL_W  = 11;           // 列计数位宽：IMG_W <= 2048

    //------------------------------------------------------------------
    // 列计数器：de 低电平清零 → col_cnt 即当前像素的 0 基列号
    //------------------------------------------------------------------
    reg [COL_W-1:0] col_cnt;

    always @(posedge clk_i or negedge rst_ni) begin
        if(!rst_ni)
            col_cnt <= {COL_W{1'b0}};
        else if(~de_i)
            col_cnt <= {COL_W{1'b0}};
        else
            col_cnt <= col_cnt + 1'b1;
    end

    //------------------------------------------------------------------
    // 输出选择
    //------------------------------------------------------------------
    wire        in_right  = (col_cnt >= HALF_W);                // 右半列
    // Only the displayed RGB/gray branches are corrected. The upstream
    // Canny/geometry inputs and AA coverage remain unchanged.
    wire [23:0] display_orig;
    wire [7:0] display_y;
    wire [23:0] display_gray={display_y,display_y,display_y};
    generate if(DISPLAY_GAMMA_ENABLE) begin: gamma_display
        display_gamma_lut r(orig_rgb_i[23:16],display_orig[23:16]);
        display_gamma_lut g(orig_rgb_i[15:8],display_orig[15:8]);
        display_gamma_lut b(orig_rgb_i[7:0],display_orig[7:0]);
        display_gamma_lut y(gray_rgb_i[7:0],display_y);
    end else begin: gamma_bypass
        assign display_orig=orig_rgb_i;
        assign display_y=gray_rgb_i[7:0];
    end endgenerate
    wire [23:0] split_rgb = in_right ? edge_rgb_i : display_gray;
    // Mode 3 receives grayscale AA coverage: 0=original, 255=yellow.
    // Rounded division by 255 without a divider; endpoints are exact.
    function [7:0] scale_coverage;
        input [7:0] value,coverage;
        reg [15:0] product;
        reg [16:0] rounded;
        begin
            product=value*coverage;
            rounded={1'b0,product}+17'd128;
            scale_coverage=(rounded+(rounded>>8))>>8;
        end
    endfunction
    wire [7:0] alpha=edge_rgb_i[7:0];
    wire [7:0] tint_r=display_orig[23:16]+scale_coverage(8'd255-display_orig[23:16],alpha);
    wire [7:0] tint_g=display_orig[15:8]+scale_coverage(8'd255-display_orig[15:8],alpha);
    wire [7:0] tint_b=display_orig[7:0]-scale_coverage(display_orig[7:0],alpha);
    wire [23:0] overlay_rgb={tint_r,tint_g,tint_b};
    wire [23:0] split_overlay_rgb = in_right ? overlay_rgb : display_gray;

    //------------------------------------------------------------------
    // 唯一的输出寄存器：数据与 vsync/de 同块寄存，延迟恒为 1 拍
    //------------------------------------------------------------------
    always @(posedge clk_i or negedge rst_ni) begin
        if(!rst_ni) begin
            vsync_o <= 1'b0;
            de_o    <= 1'b0;
            rgb_o   <= 24'h000000;
        end else begin
            vsync_o <= vsync_i;
            de_o    <= de_i;
            case(mode_i)
                2'd0    : rgb_o <= display_orig;   // 原图
                2'd1    : rgb_o <= edge_rgb_i;   // 边缘图
                2'd2    : rgb_o <= split_rgb;    // 分屏：左半灰度 + 右半边缘图
                2'd3    : rgb_o <= split_overlay_rgb; // 左灰度 + 右原图叠加亮黄色边缘
                default : rgb_o <= display_orig;
            endcase
        end
    end

endmodule
