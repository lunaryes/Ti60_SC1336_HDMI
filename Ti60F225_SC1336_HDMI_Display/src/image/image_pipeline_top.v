`timescale 1ns / 1ps
//=====================================================================
// image_pipeline_top  —— 接口 I-A
//---------------------------------------------------------------------
// 算法流水线顶层，插在 AWB 之后、FrameBoundCrop 之前（唯一插入点，
// 见 docs/ARCH.md §1）。内部串联：
//
//   rgb_i ──► rgb_to_gray ──► pix_window_3x3 ──► canny_edge ──┐
//       │            (Y8)          (3x3 灰度窗)   (mag / edge) │
//       │                                                      ▼
//       └──────────────► orig_dly(15 拍) ──────────────► image_mix ──► rgb_o
//                                                        (mode 选通)
//
// 【对外延迟】LATENCY = 16 拍，数据与握手信号严格同拍：
//   rgb_to_gray  1 拍（灰度寄存）
//   pix_window   2 拍（bank 同步读 1 拍 + p11..p33 寄存 1 拍）
//   canny_edge  12 拍（S1~S12 流水，逐级见 canny_edge.v 头注释）
//   image_mix    1 拍（唯一输出寄存器）
//   -------------------------------------------------
//   合计        16 拍
//   de_o = de_i 延迟 16；vsync_o = vsync_i 延迟 16，宽度不变（送到 rgb2dvi 的
//   VS 仍是完整帧同步电平，不能被压成单拍，否则显示器锁不住帧）。
//   ★ 改这个拍数必须同步改 example_top 里的 hsync 延迟（同一个数，见那里的
//     IMG_LAT 与 r_hs_dly），否则 rgb2dvi 的 HS 与 DE 错位、HDMI 画面横移。
//
// 【原图路径为什么是 15 拍】rgb_i 经 orig_dly 延迟 15 拍后进 image_mix.orig_rgb_i，
//   与被 image_mix 寄存的 edge_rgb_i 同拍，两者一起再延迟 1 拍输出。这条路径是
//   纯寄存器延迟、不含行缓存，所以在 mode_i = 0 时 rgb_o 与 rgb_i 逐像素完全一致
//   （只差固定 16 拍），满足接口文档 §6 的旁路回归要求。
//   代价是 15×24 = 360 个寄存器；相比之下，若把原图改成取窗口中心像素 p22 来省这
//   个延迟，mode 0 就不再是逐像素旁路，故不采用。
//   （接 Canny 前是 7 拍：sobel_edge 的 4 拍换成 canny_edge 的 12 拍，原图路径
//     跟着加长 8 拍。）
//
// 【边缘路径的空间偏移】p11..p33 是"当前像素左上角一格"的邻域（窗口中心 = 上一行、
//   左一列）。Canny 内部还有两级 3x3 窗口（高斯场、梯度场），每多一级就多下移
//   1 行，所以边缘图相对原图是**下移 3 行、0 列**（实测值，见 canny_edge.v 头
//   注释的"空间对齐"）。下级 FrameBoundCrop 的 SKIP_ROWS/SKIP_COLS = 2 只盖住
//   2 行，帧首还剩几行是行缓存没填满的暂态（实测输出第 0..5 行，可见区里剩
//   第 2..5 行，实测数据见 canny_edge.v 头注释的"边界口径"）——上板看效果后
//   再定：要么把 SKIP_ROWS 加到 6，要么接受这几行（留到 M5）。分屏模式下两侧
//   接缝相差 3 行。
//
// 【vsync 约定】vsync_i 按接口文档 §2 为"帧有效电平"。pix_window_3x3 内部自己做
//   帧首上升沿检测，故这里直接透传电平即可。
//
// 【thr_i】顶层只保留一个 8bit 阈值口（接口 I-A 不变、example_top 不改这个口），
//   内部映射成 Canny 的双阈值：thr_high = thr_i、thr_low = thr_i >> 1（经典
//   Canny 的 2:1；thr_i 默认 16 时即 16/8，与仿真台/参考模型一致）。
//   判据域是 canny_edge 内部的 mag8 = mag >> 3（**不是** sobel 那种"先饱和到
//   255 再比"），所以这个口的刻度与 sobel_edge 的 thr_i 不可直接换算。
//   ★★ 这个口的有效量程只有 0..96（横竖边）/ 0..140（对角边），不是 0..255 ——
//      把 thr 当成 "n/255 的对比度" 会高估 2.7 倍。原来的默认值 40 实际要求
//      **满幅 42% 的对比度**，这就是"只有高对比度才出边"的根因（M5 事故）。
//      完整的标尺表见 canny_edge.v 头注释；经验值 16（≈17% 对比度）。
//   要单独调阈值/视图，改 example_top 里的寄存器（默认走 UART 可运行时调）。
//
// 【view_i】边缘支路显示什么（M5 新增，默认 0 = 原来的二值边缘图）：
//   0 二值边缘图 / 1 mag8 灰度图 / 2 四类假彩图 / 3 Y 灰度图。
//   详细口径见下方第 4 节的注释。view_i 只影响边缘支路的像素内容，**不影响
//   延迟**（四种视图的组合路径都在 image_mix 之前，由 image_mix 统一寄存）。
//=====================================================================
module image_pipeline_top #(
    parameter IMG_W       = 1280,
    parameter IMG_H       = 720,
    parameter LATENCY     = 16       // 对外延迟（拍），与上面的推导一致
)(
    input  wire        clk_i,        // clk_pixel
    input  wire        rst_ni,
    input  wire        vsync_i,      // 帧有效电平
    input  wire        de_i,         // 像素有效
    input  wire [23:0] rgb_i,        // {R[7:0], G[7:0], B[7:0]}
    input  wire [7:0]  thr_i,        // 默认 16（原 40 太高，见 canny_edge.v 头注释）
    input  wire [1:0]  mode_i,       // 含义见 image_mix（0 原图 / 1 边缘 / 2、3 分屏）
    input  wire [1:0]  view_i,       // 边缘支路显示什么，见下方【view_i】
    input  wire [1:0]  smooth_mode_i,// 0 标准高斯 / 1 弱平滑 / 2 完全旁路
    output wire        vsync_o,
    output wire        de_o,
    output wire [23:0] rgb_o
);

    localparam CANNY_LAT = 12;
    localparam WIN_LAT   = 2;        // pix_window_3x3：bank 同步读 1 拍 + 窗口寄存 1 拍
    localparam ORIG_LAT  = 1 + WIN_LAT + CANNY_LAT;   // = 15，进 image_mix 时的拍数

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
    // 3) Canny：梯度幅值 + 阈值边缘标志（高斯 + Sobel + NMS + 双阈值滞后）
    //    sobel_edge 就地换成 canny_edge：p11..p33 / vsync / de / mag_o / edge_o
    //    一一对应，只有阈值口从 1 个变 2 个（方案 §11 决议 1：不做 generate
    //    二选一，省下的资源留给后续模块）。sobel.v 仍留在仓库里，对照走
    //    分支 / 图样。
    //-----------------------------------------------------------------
    wire        can_vs, can_de;
    wire [7:0]  can_mag8;             // 真实阈值域 mag8（未饱和），调试用
    wire [1:0]  can_cls;              // 判决分类：00/01/10/11，调试用
    wire        can_edge;

    // 单一阈值口 → 双阈值（2:1），见文件头的【thr_i】
    wire [7:0]  thr_high = thr_i;
    wire [7:0]  thr_low  = {1'b0, thr_i[7:1]};

    canny_edge #(
        .IMG_W   (IMG_W),
        .LATENCY (CANNY_LAT)
    ) u_canny_edge (
        .clk     (clk_i),
        .rst_n   (rst_ni),

        .vsync_i (win_vs),
        .de_i    (win_de),
        .p11     (p11), .p12 (p12), .p13 (p13),
        .p21     (p21), .p22 (p22), .p23 (p23),
        .p31     (p31), .p32 (p32), .p33 (p33),
        .thr_high_i (thr_high),
        .thr_low_i  (thr_low),
        .smooth_mode_i (smooth_mode_i),

        .vsync_o (can_vs),
        .de_o    (can_de),
        // mag_o 的 min(mag,255) 口径在显示上已被 mag8_o 取代 —— 它在 mag8>32 就
        // 饱和，没法用来调阈值（见 canny_edge.v 的 mag8_o 注释）。端口保留是为了
        // 不破坏已冻结的 I-B 语义与 sobel 的视觉对照，这里不再往外引。
        .mag_o   (),
        .edge_o  (can_edge),
        .mag8_o  (can_mag8),
        .cls_o   (can_cls)
    );

    //-----------------------------------------------------------------
    // 4) 边缘支路显示内容，由 view_i 选（与 can_de / can_mag / can_edge 同拍）
    //    0 = 二值边缘图（白线黑底）——工程默认
    //    1 = mag8 灰度图，显示值 = min(mag8<<1, 255)
    //        阈值面直接可读：thr=16 落在灰度 32、thr=40 落在 80，满对比度黑边
    //        （mag8=96）落在 192。**不能拿 mag_o 调阈值**——它在 mag8>32 就饱和
    //        （见 canny_edge.v 的 mag8_o 注释）。
    //    2 = 四类假彩图（"判死器"）：
    //          黑  = 无响应（mag8 < thr_low，该处真的没梯度）
    //          深红 = 有梯度但被 NMS 抑制
    //          蓝  = 弱边（过 NMS，未过强阈值）
    //          白  = 强边
    //    3 = Y 灰度图（算法眼里的世界）。用来量 ΔY：屏幕上读出两侧灰度差，
    //        查 canny_edge.v 头注释的标尺表就知道该不该出边。
    //        ★ y_dly 只做时间对齐（14 拍），不做空间对齐，所以本视图比边缘图
    //          高 3 行（正是已知的整体下移量），量对比度不受影响。
    //-----------------------------------------------------------------
    wire [8:0] mag8x2      = {1'b0, can_mag8} << 1;
    wire [7:0] mag8_view   = mag8x2[8] ? 8'hFF : mag8x2[7:0];

    // Five-state diagnosis:
    // black=no gradient, red=NMS rejected, yellow=weak/unlinked,
    // blue=weak/linked by hysteresis, white=strong edge.
    wire [23:0] cls_rgb = (can_cls == 2'b11) ? 24'hFFFFFF :
                          (can_cls == 2'b10) ?
                              (can_edge ? 24'h0000FF : 24'hFFFF00) :
                          (can_cls == 2'b01) ? 24'h800000 : 24'h000000;

    reg [7:0] y_dly [0:ORIG_LAT-2];    // gray_y 再延 ORIG_LAT-1 = 14 拍，
                                       // 与 orig_dly[ORIG_LAT-1] 同拍
    integer j;
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            for (j = 0; j < ORIG_LAT-1; j = j + 1)
                y_dly[j] <= 8'd0;
        end else begin
            y_dly[0] <= gray_y;
            for (j = 1; j < ORIG_LAT-1; j = j + 1)
                y_dly[j] <= y_dly[j-1];
        end
    end

    wire [23:0] edge_rgb = (view_i == 2'd1) ? {mag8_view, mag8_view, mag8_view} :
                           (view_i == 2'd2) ? cls_rgb :
                           (view_i == 2'd3) ? {y_dly[ORIG_LAT-2], y_dly[ORIG_LAT-2],
                                               y_dly[ORIG_LAT-2]} :
                                              (can_edge ? 24'hFFFFFF : 24'h000000);

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
    // 6) 合成：vsync / de 由 Canny 链给出，与两路图像严格同拍
    //-----------------------------------------------------------------
    image_mix #(
        .IMG_W (IMG_W),
        .IMG_H (IMG_H)
    ) u_image_mix (
        .clk_i      (clk_i),
        .rst_ni     (rst_ni),

        .vsync_i    (can_vs),
        .de_i       (can_de),
        .orig_rgb_i (orig_rgb),
        .edge_rgb_i (edge_rgb),
        .mode_i     (mode_i),

        .vsync_o    (vsync_o),
        .de_o       (de_o),
        .rgb_o      (rgb_o)
    );

endmodule
