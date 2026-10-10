`timescale 1ns / 1ps
//=====================================================================
// image_pipeline_top  —— 接口 I-A
//---------------------------------------------------------------------
// 算法流水线顶层，插在 AWB 之后、FrameBoundCrop 之前（唯一插入点，
// 见 docs/ARCH.md §1）。内部串联：
//
//   rgb_i ──► rgb_to_gray ──► pix_window_3x3 ──► sobel_edge ──┐
//       │            (Y8)          (3x3 灰度窗)   (mag / edge) │
//       │                                                      ▼
//       └───────────────► orig_dly(6 拍) ───────────────► image_mix ──► rgb_o
//                                                        (mode 选通)
//
// 【对外延迟】LATENCY = 8 拍，数据与握手信号严格同拍：
//   rgb_to_gray  1 拍（灰度寄存）
//   pix_window   2 拍（bank 同步读 1 拍 + p11..p33 寄存 1 拍）
//   sobel_edge   4 拍（S1~S4 流水）
//   image_mix    1 拍（唯一输出寄存器）
//   -------------------------------------------------
//   合计         8 拍
//   de_o = de_i 延迟 8；vsync_o = vsync_i 延迟 8，宽度不变（送到 rgb2dvi 的
//   VS 仍是完整帧同步电平，不能被压成单拍，否则显示器锁不住帧）。
//
// 【原图路径为什么是 7 拍】rgb_i 经 orig_dly 延迟 7 拍后进 image_mix.orig_rgb_i，
//   与被 image_mix 寄存的 edge_rgb_i 同拍，两者一起再延迟 1 拍输出。这条路径是
//   纯寄存器延迟、不含行缓存，所以在 mode_i = 0 时 rgb_o 与 rgb_i 逐像素完全一致
//   （只差固定 8 拍），满足接口文档 §6 的旁路回归要求。
//   代价是 7×24 = 168 个寄存器；相比之下，若把原图改成取窗口中心像素 p22 来省这
//   个延迟，mode 0 就不再是逐像素旁路，故不采用。
//
// 【边缘路径的空间偏移】p11..p33 是"当前像素左上角一格"的邻域（窗口中心 = 上一行、
//   左一列），因此边缘图相对原图整体右下偏移 1 行 1 列。这两行两列落在外层，
//   会被 FrameBoundCrop 的 SKIP_ROWS/SKIP_COLS = 2 涂黑，实际不可见；分屏模式下
//   接缝处最多差 1 个像素，无影响。
//
// 【vsync 约定】vsync_i 按接口文档 §2 为"帧有效电平"。pix_window_3x3 内部自己做
//   帧首上升沿检测，故这里直接透传电平即可。
//
// 【thr_i】复位默认值 40 由 example_top 给出；edge_o = (mag_o > thr_i)。
//=====================================================================
module image_pipeline_top #(
    parameter IMG_W       = 1280,
    parameter IMG_H       = 720,
    parameter EDGE_BINARY = 1,       // 1 = 二值边缘图（白线黑底，thr 直观可见）
                                     // 0 = 梯度幅值灰度图（{mag,mag,mag}）
    parameter LATENCY     = 8        // 对外延迟（拍），与上面的推导一致
)(
    input  wire        clk_i,        // clk_pixel
    input  wire        rst_ni,
    input  wire        vsync_i,      // 帧有效电平
    input  wire        de_i,         // 像素有效
    input  wire [23:0] rgb_i,        // {R[7:0], G[7:0], B[7:0]}
    input  wire [7:0]  thr_i,        // 默认 40
    input  wire [1:0]  mode_i,       // 含义见 image_mix（0 原图 / 1 边缘 / 2、3 分屏）
    output wire        vsync_o,
    output wire        de_o,
    output wire [23:0] rgb_o
);

    localparam SOBEL_LAT = 4;
    localparam WIN_LAT   = 2;        // pix_window_3x3：bank 同步读 1 拍 + 窗口寄存 1 拍
    localparam ORIG_LAT  = 1 + WIN_LAT + SOBEL_LAT;   // = 7，进 image_mix 时的拍数

    //-----------------------------------------------------------------
    // 1) 灰度化：RGB888 → Y8
    //    pix_window_3x3 的端口是 gray_i，灰度在本级完成；
    //    借用 rgb_to_gray，取它的 rgb_o[7:0] 即 Y（输出是 {Y,Y,Y}）。
    //-----------------------------------------------------------------
    wire        gray_vs, gray_de;
    wire [23:0] gray_rgb;

    rgb_to_gray u_rgb_to_gray (
        .clk   (clk_i),
        .rst_n (rst_ni),

        .vs_i  (vsync_i),
        .hs_i  (1'b0),
        .de_i  (de_i),
        .rgb_i (rgb_i),

        .vs_o  (gray_vs),
        .hs_o  (),
        .de_o  (gray_de),
        .rgb_o (gray_rgb)
    );

    wire [7:0] gray_y = gray_rgb[7:0];

    //-----------------------------------------------------------------
    // 2) 3x3 灰度窗口（只例化一次；Sobel 与后续中值共享同一组 9 像素）
    //-----------------------------------------------------------------
    wire        win_vs, win_de;
    wire [7:0]  p11, p12, p13;
    wire [7:0]  p21, p22, p23;
    wire [7:0]  p31, p32, p33;

    pix_window_3x3 #(
        .IMG_W (IMG_W)
    ) u_pix_window_3x3 (
        .clk     (clk_i),
        .rst_n   (rst_ni),

        .vsync_i (gray_vs),
        .de_i    (gray_de),
        .gray_i  (gray_y),

        .vsync_o (win_vs),
        .de_o    (win_de),

        .p11     (p11), .p12 (p12), .p13 (p13),
        .p21     (p21), .p22 (p22), .p23 (p23),
        .p31     (p31), .p32 (p32), .p33 (p33)
    );

    //-----------------------------------------------------------------
    // 3) Sobel：梯度幅值 + 阈值边缘标志
    //-----------------------------------------------------------------
    wire        sob_vs, sob_de;
    wire [7:0]  sob_mag;
    wire        sob_edge;

    sobel_edge #(
        .LATENCY (SOBEL_LAT)
    ) u_sobel_edge (
        .clk     (clk_i),
        .rst_n   (rst_ni),

        .vsync_i (win_vs),
        .de_i    (win_de),
        .p11     (p11), .p12 (p12), .p13 (p13),
        .p21     (p21), .p22 (p22), .p23 (p23),
        .p31     (p31), .p32 (p32), .p33 (p33),
        .thr_i   (thr_i),

        .vsync_o (sob_vs),
        .de_o    (sob_de),
        .mag_o   (sob_mag),
        .edge_o  (sob_edge)
    );

    //-----------------------------------------------------------------
    // 4) 边缘图（与 sob_de / sob_mag / sob_edge 同拍）
    //-----------------------------------------------------------------
    wire [23:0] edge_rgb = EDGE_BINARY ? (sob_edge ? 24'hFFFFFF : 24'h000000)
                                       : {sob_mag, sob_mag, sob_mag};

    //-----------------------------------------------------------------
    // 5) 原图延迟 ORIG_LAT 拍（纯寄存器，无行缓存，保证 mode 0 逐像素一致）
    //-----------------------------------------------------------------
    reg [23:0] orig_dly [0:ORIG_LAT-1];

    integer k;
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            for (k = 0; k < ORIG_LAT; k = k + 1)
                orig_dly[k] <= 24'h000000;
        end else begin
            orig_dly[0] <= rgb_i;
            for (k = 1; k < ORIG_LAT; k = k + 1)
                orig_dly[k] <= orig_dly[k-1];
        end
    end

    wire [23:0] orig_rgb = orig_dly[ORIG_LAT-1];

    //-----------------------------------------------------------------
    // 6) 合成：vsync / de 由 Sobel 链给出，与两路图像严格同拍
    //-----------------------------------------------------------------
    image_mix #(
        .IMG_W (IMG_W),
        .IMG_H (IMG_H)
    ) u_image_mix (
        .clk_i      (clk_i),
        .rst_ni     (rst_ni),

        .vsync_i    (sob_vs),
        .de_i       (sob_de),
        .orig_rgb_i (orig_rgb),
        .edge_rgb_i (edge_rgb),
        .mode_i     (mode_i),

        .vsync_o    (vsync_o),
        .de_o       (de_o),
        .rgb_o      (rgb_o)
    );

endmodule
