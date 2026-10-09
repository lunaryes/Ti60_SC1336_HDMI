`timescale 1ns / 1ps
//=====================================================================
// image_pipeline_top  —— 接口 I-A
//---------------------------------------------------------------------
// 算法流水线顶层，插在 AWB 之后、FrameBoundCrop 之前（唯一插入点）。
//
//    rgb_i ──► pix_window_3x3 ──┬──► sobel_edge ──► mag / edge
//                               │                        │
//                               └── sync_delay_align ────┤（旁路对齐）
//                                                        ▼
//                                                   image_mix ──► rgb_o
//
// 【旁路对齐（关键）】
//   旁路路径不是简单地把 rgb_i 直接接到 MUX：
//     - 原图取 pix_window_3x3 输出的中心像素 p22（第 n-1 行、第 x-2 列），
//       与 Sobel 的窗口中心天然同源同拍，省掉一整块行缓存；
//     - 再用 sync_delay_align 补上 sobel_edge 的 4 级流水延迟。
//   这样进 image_mix 的 orig_rgb_i 与 edge_rgb_i 严格同拍，mode=0 不会错位。
//
// 【对外延迟】
//   数据与握手信号同延迟：2 拍（pix_window_3x3）+ 4 拍（sobel_edge）+ 1 拍（image_mix）
//   = 7 拍，外加垂直 1 行。
//   即 vsync_o / de_o 相对 vsync_i / de_i 延迟 LATENCY = 7 拍。
//   example_top 必须把 hsync 也延迟 7 拍，否则 HS 与 DE 不再同步、图像会横移
//   7 个像素。
//   垂直方向整幅下移 1 行，落在 FrameBoundCrop 的最外 2 行空白区内，不可见。
//
// 【thr_i】复位默认 40，由 example_top 保持缺省值。
//=====================================================================
module image_pipeline_top #(
    parameter IMG_W       = 1280,
    parameter IMG_H       = 720,
    parameter EDGE_BINARY = 1,       // 1 = 二值边缘图（thr 直观可见）；0 = 梯度幅值灰度图
    parameter LATENCY     = 7        // 对外延迟（拍），需与上面推导一致
)(
    input  wire        clk_i,        // clk_pixel
    input  wire        rst_ni,
    input  wire        vsync_i,
    input  wire        de_i,
    input  wire [23:0] rgb_i,
    input  wire [7:0]  thr_i,        // 默认 40
    input  wire [1:0]  mode_i,       // 含义见 image_mix
    output wire        vsync_o,
    output wire        de_o,
    output wire [23:0] rgb_o
);

    localparam SOBEL_LAT = 4;

    //-----------------------------------------------------------------
    // 3x3 灰度窗口（只例化一次，Sobel 与中值共享同一组 p11..p33）
    //-----------------------------------------------------------------
    wire        win_vs, win_de;
    wire [7:0]  p11, p12, p13;
    wire [7:0]  p21, p22, p23;
    wire [7:0]  p31, p32, p33;

    pix_window_3x3 #(
        .IMG_W (IMG_W),
        .IMG_H (IMG_H)
    ) u_pix_window_3x3 (
        .clk      (clk_i),
        .rst_n    (rst_ni),
        .vsync_i  (vsync_i),
        .de_i     (de_i),
        .rgb_i    (rgb_i),
        .vsync_o  (win_vs),
        .de_o     (win_de),
        .p11      (p11), .p12(p12), .p13(p13),
        .p21      (p21), .p22(p22), .p23(p23),
        .p31      (p31), .p32(p32), .p33(p33)
    );

    //-----------------------------------------------------------------
    // Sobel 梯度 + 阈值
    //-----------------------------------------------------------------
    wire [7:0] sobel_mag;
    wire       sobel_edge_flag;
    wire       sobel_vs, sobel_de;

    sobel_edge #(
        .LATENCY (SOBEL_LAT)
    ) u_sobel_edge (
        .clk      (clk_i),
        .rst_n    (rst_ni),
        .vsync_i  (win_vs),
        .de_i     (win_de),
        .p11      (p11), .p12(p12), .p13(p13),
        .p21      (p21), .p22(p22), .p23(p23),
        .p31      (p31), .p32(p32), .p33(p33),
        .thr_i    (thr_i),
        .vsync_o  (sobel_vs),
        .de_o     (sobel_de),
        .mag_o    (sobel_mag),
        .edge_o   (sobel_edge_flag)
    );

    //-----------------------------------------------------------------
    // 旁路对齐：原图取窗口中心像素 p22，再补 Sobel 的流水延迟
    //-----------------------------------------------------------------
    wire        orig_vs, orig_de;
    wire [23:0] orig_rgb;

    sync_delay_align #(
        .DEPTH (SOBEL_LAT),
        .DW    (24)
    ) u_sync_delay_align (
        .clk      (clk_i),
        .rst_n    (rst_ni),
        .vsync_i  (win_vs),
        .de_i     (win_de),
        .data_i   ({p22, p22, p22}),     // {Y,Y,Y}
        .vsync_o  (orig_vs),
        .de_o     (orig_de),
        .data_o   (orig_rgb)
    );

    //-----------------------------------------------------------------
    // 边缘图
    //-----------------------------------------------------------------
    wire [23:0] edge_rgb = EDGE_BINARY ? (sobel_edge_flag ? 24'hFFFFFF : 24'h000000)
                                       : {sobel_mag, sobel_mag, sobel_mag};

    //-----------------------------------------------------------------
    // 合成输出
    //-----------------------------------------------------------------
    image_mix #(
        .IMG_W (IMG_W),
        .IMG_H (IMG_H)
    ) u_image_mix (
        .clk_i      (clk_i),
        .rst_ni     (rst_ni),
        .vsync_i    (orig_vs),
        .de_i       (orig_de),
        .orig_rgb_i (orig_rgb),
        .edge_rgb_i (edge_rgb),
        .mode_i     (mode_i),
        .vsync_o    (vsync_o),
        .de_o       (de_o),
        .rgb_o      (rgb_o)
    );

endmodule
