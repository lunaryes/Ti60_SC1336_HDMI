`timescale 1ns / 1ps
//=====================================================================
// image_pipeline_top  —— 接口 I-A
//---------------------------------------------------------------------
// 算法流水线顶层，插在 AWB 之后、FrameBoundCrop 之前（唯一插入点，
// 见 docs/ARCH.md §1）。内部串联：
//
//   rgb_i ──► rgb_to_gray ──► pix_window_3x3 ──► median_filter_3x3 ──► pix_window_3x3 ──► canny_edge ──┐
//       │            (Y8)        (3x3 原始窗)        (3x3 中值)          (3x3 中值窗)     (mag / edge) │
//       │                                                                                            ▼
//       └──────────────────────► orig_dly(18 拍) ──────────────────────────────────────────► image_mix ──► rgb_o
//                                                                                            (mode 选通)
//
// 【对外延迟】LATENCY = 19 拍，数据与握手信号严格同拍：
//   rgb_to_gray        1 拍（灰度寄存）
//   pix_window (前)    2 拍（bank 同步读 1 拍 + p11..p33 寄存 1 拍）
//   median_filter_3x3  1 拍（组合中值 + 输出寄存）
//   pix_window (后)    2 拍（中值像素流重新构 3x3 邻域）
//   canny_edge        12 拍（S1~S12 流水，逐级见 canny_edge.v 头注释）
//   image_mix          1 拍（唯一输出寄存器）
//   -------------------------------------------------
//   合计              19 拍
//   de_o = de_i 延迟 19；vsync_o = vsync_i 延迟 19，宽度不变（送到 rgb2dvi 的
//   VS 仍是完整帧同步电平，不能被压成单拍，否则显示器锁不住帧）。
//   ★ 改这个拍数必须同步改 example_top 里的 hsync 延迟（同一个数，见那里的
//     IMG_LAT 与 r_hs_dly），否则 rgb2dvi 的 HS 与 DE 错位、HDMI 画面横移。
//
// 【原图路径为什么是 18 拍】rgb_i 经 orig_dly 延迟 18 拍后进 image_mix.orig_rgb_i，
//   与被 image_mix 寄存的 edge_rgb_i 同拍，两者一起再延迟 1 拍输出。这条路径是
//   纯寄存器延迟、不含行缓存，所以在 mode_i = 0 时 rgb_o 与 rgb_i 逐像素完全一致
//   （只差固定 19 拍），满足接口文档 §6 的旁路回归要求。
//   代价是 18×24 = 432 个寄存器；相比之下，若把原图改成取窗口中心像素 p22 来省这
//   个延迟，mode 0 就不再是逐像素旁路，故不采用。
//
// 【边缘路径的空间偏移】前级 p11..p33 是"当前像素左上角一格"的邻域（窗口中心 =
//   上一行、左一列）；中值滤波按 p22 输出；后级窗口再引入一级行缓存；Canny 内部
//   还有两级 3x3 窗口（高斯场、梯度场），每多一级就多下移 1 行。因此边缘图相对
//   原图约**下移 4 行、0 列**（比加中值前多 1 行；精确值以 canny_edge.v 头注释
//   的实测口径为准）。下级 FrameBoundCrop 的 SKIP_ROWS/SKIP_COLS = 2 只盖住 2 行，
//   帧首还会剩几行行缓存没填满的暂态——比加中值前多 1 行。上板看效果后再定：
//   把 SKIP_ROWS 加到 6，或接受这几行。分屏左侧灰度只下移 1 行，所以灰度/
//   边缘接缝处两侧相差约 3 行（不是 0，也不是 4）。
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
//   0 二值边缘图 / 1 mag8 灰度图 / 2 五状态假彩图 / 3 Y 灰度图。
//   详细口径见下方第 5 节的注释。view_i 只影响边缘支路的像素内容，**不影响
//   延迟**（四种视图的组合路径都在 image_mix 之前，由 image_mix 统一寄存）。
//
// 【smooth_mode_i】Canny 高斯前级的平滑程度（诊断分支合并）：
//   0 标准 1:2:1 高斯 / 1 弱平滑（高斯与原像素 1:1）/ 2 完全旁路。
//   模式在帧边界锁存，帧中改值不会在画面中间产生横向接缝；延迟不变。
//=====================================================================
module image_pipeline_top #(
    parameter IMG_W       = 1280,
    parameter IMG_H       = 720,
    parameter LATENCY     = 19,      // 对外延迟（拍），与上面的推导一致
    // 中值滤波参数：默认只替换"局部极值且与中值差 ≥24"的脉冲点，干净台阶边
    // 会被原样保留；要更强的去噪可把 IMPULSE_ONLY 设 0（全 3x3 中值）。
    parameter integer MEDIAN_IMPULSE_ONLY  = 1,
    parameter [7:0]   MEDIAN_IMPULSE_DELTA = 8'd24
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
    output wire [23:0] rgb_o,

    // M6/M7 增量调试端口（只增不改）：中值滤波之后的 Y 像素流，即 Canny
    // **实际看到的**灰度（与 med_y 是同一份寄存器，不额外消耗逻辑）。
    // 给 src/common/gray_hist_uart.v 做片内直方图用（无采集卡时量灰度分布）。
    // 接中值前想要原始 rgb_to_gray Y 的话，改接 gray_y/gray_de/gray_vs。
    // 不接直方图时可以悬空；不影响延迟和原有输出。
    output wire [7:0]  dbg_y_o,
    output wire        dbg_de_o,
    output wire        dbg_vs_o,
    output wire        dbg_edge_o,
    output wire        dbg_edge_de_o,
    output wire        dbg_edge_vs_o
);

    localparam CANNY_LAT = 12;
    localparam WIN_LAT   = 2;        // pix_window_3x3：bank 同步读 1 拍 + 窗口寄存 1 拍
    localparam MED_LAT   = 1;        // median_filter_3x3：组合中值 + 输出寄存
    // 1 灰度 + 2 前窗 + 1 中值 + 2 后窗 + 12 Canny；进 image_mix 时再加 1 拍
    // 输出寄存器，所以 LATENCY = ORIG_LAT + 1。
    localparam ORIG_LAT  = 1 + WIN_LAT + MED_LAT + WIN_LAT + CANNY_LAT;  // = 18
    // view 3 的 Y 延迟：med_y 已经比 gray_y 晚 (WIN_LAT+MED_LAT)=3 拍，再延
    // Y_DLY 拍后与 can_de 同拍（ORIG_LAT-1 = 17 拍），最后经 image_mix 1 拍输出。
    localparam Y_DLY     = ORIG_LAT - 1 - (WIN_LAT + MED_LAT);           // = 14

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

    // 直方图调试抽头在下面中值滤波之后：Canny 现在看到的是中值后的 Y，
    // 不是这里的 gray_y。

    //-----------------------------------------------------------------
    // 2) 前级 3x3 灰度窗口（给中值滤波提供 p11..p33）
    //    Canny 不能直接复用这组原始窗口：中值输出的是"一个像素"，要得到
    //    中值后的 3x3 邻域，必须在后面再例化一级窗口（第 3 节）。
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
    // 3) 3x3 中值滤波 + 后级 3x3 窗口
    //    median_filter_3x3（feature_median_filter 分支）对前级窗口做一次
    //    3x3 中值，输出仍是单像素流；要送进 Canny 的 3x3 高斯核，必须
    //    再构一级窗口。IMPULSE_ONLY=1 时只有"局部极值且与中值差 ≥DELTA"
    //    的中心点才替换，干净台阶边是恒等（不会把边糊掉）；IMPULSE_ONLY=0
    //    则是完整 3x3 中值，去噪更强但会牺牲一点边缘锐度。
    //-----------------------------------------------------------------
    wire        med_vs, med_de;
    wire [7:0]  med_y;

    median_filter_3x3 #(
        .IMPULSE_ONLY  (MEDIAN_IMPULSE_ONLY),
        .IMPULSE_DELTA (MEDIAN_IMPULSE_DELTA)
    ) u_median_filter_3x3 (
        .clk   (clk_i),
        .rst_n (rst_ni),

        .vsync_i (win_vs),
        .de_i    (win_de),
        .p11     (p11), .p12 (p12), .p13 (p13),
        .p21     (p21), .p22 (p22), .p23 (p23),
        .p31     (p31), .p32 (p32), .p33 (p33),

        .vsync_o (med_vs),
        .de_o    (med_de),
        .gray_o  (med_y)
    );

    // 直方图/调试视图抽头：Canny 实际看到的中值后 Y/de/vs
    assign dbg_y_o  = med_y;
    assign dbg_de_o = med_de;
    assign dbg_vs_o = med_vs;

    wire        win2_vs, win2_de;
    wire [7:0]  q11, q12, q13;
    wire [7:0]  q21, q22, q23;
    wire [7:0]  q31, q32, q33;

    pix_window_3x3 #(
        .IMG_W (IMG_W)
    ) u_pix_window_post_median (
        .clk     (clk_i),
        .rst_n   (rst_ni),

        .vsync_i (med_vs),
        .de_i    (med_de),
        .gray_i  (med_y),

        .vsync_o (win2_vs),
        .de_o    (win2_de),

        .p11     (q11), .p12 (q12), .p13 (q13),
        .p21     (q21), .p22 (q22), .p23 (q23),
        .p31     (q31), .p32 (q32), .p33 (q33)
    );

    //-----------------------------------------------------------------
    // 4) Canny：梯度幅值 + 阈值边缘标志（高斯 + Sobel + NMS + 双阈值滞后）
    //    输入改为中值后的 3x3 窗口（q11..q33）；其余口径不变。
    //    sobel.v 仍留在仓库里，对照走分支 / 图样。
    //-----------------------------------------------------------------
    wire        can_vs, can_de;
    wire [7:0]  can_mag8;             // 真实阈值域 mag8（未饱和），调试用
    wire [1:0]  can_cls;              // 判决分类：00/01/10/11，调试用
    wire        can_edge;
    assign dbg_edge_o=can_edge;
    assign dbg_edge_de_o=can_de;
    assign dbg_edge_vs_o=can_vs;

    // 单一阈值口 → 双阈值（2:1），见文件头的【thr_i】
    wire [7:0]  thr_high = thr_i;
    wire [7:0]  thr_low  = {1'b0, thr_i[7:1]};

    canny_edge #(
        .IMG_W   (IMG_W),
        .LATENCY (CANNY_LAT)
    ) u_canny_edge (
        .clk     (clk_i),
        .rst_n   (rst_ni),

        .vsync_i (win2_vs),
        .de_i    (win2_de),
        .p11     (q11), .p12 (q12), .p13 (q13),
        .p21     (q21), .p22 (q22), .p23 (q23),
        .p31     (q31), .p32 (q32), .p33 (q33),
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
    // 5) 边缘支路显示内容，由 view_i 选（与 can_de / can_mag / can_edge 同拍）
    //    0 = 二值边缘图（白线黑底）——工程默认
    //    1 = mag8 灰度图，显示值 = min(mag8<<1, 255)
    //        阈值面直接可读：thr=16 落在灰度 32、thr=40 落在 80，满对比度黑边
    //        （mag8=96）落在 192。**不能拿 mag_o 调阈值**——它在 mag8>32 就饱和
    //        （见 canny_edge.v 的 mag8_o 注释）。
    //    2 = 五状态假彩图（"判死器"，诊断分支合并）：
    //          黑  = 无响应（mag8 < thr_low，该处真的没梯度）
    //          深红 = 有梯度但被 NMS 抑制
    //          黄  = 弱边（过 NMS，但未被滞后连接保留）
    //          蓝  = 弱边，且被滞后连接保留
    //          白  = 强边
    //    3 = Y 灰度图（算法眼里的世界，中值后）。用来量 ΔY：屏幕上读出两侧
    //        灰度差，查 canny_edge.v 头注释的标尺表就知道该不该出边。
    //        ★ y_dly 只做时间对齐（14 拍），不做空间对齐：中值后的 Y 比原图
    //          下移 1 行，边缘图下移 4 行，所以本视图比边缘图高 3 行，量对比度
    //          不受影响。
    //-----------------------------------------------------------------
    wire [8:0] mag8x2      = {1'b0, can_mag8} << 1;
    wire [7:0] mag8_view   = mag8x2[8] ? 8'hFF : mag8x2[7:0];

    // 弱边再按 can_edge（滞后连接结果）分成黄/蓝：诊断时能直接看出
    // "弱梯度存在但没连上"和"弱梯度已保留"的区别。
    wire [23:0] cls_rgb = (can_cls == 2'b11) ? 24'hFFFFFF :   // 强边     白
                          (can_cls == 2'b10) ?
                              (can_edge ? 24'h0000FF :        // 弱边已连 蓝
                                          24'hFFFF00) :       // 弱边未连 黄
                          (can_cls == 2'b01) ? 24'h800000 :   // NMS 抑制 深红
                                               24'h000000;    // 无响应   黑

    // med_y 在整链的第 4 拍（rgb_to_gray 1 + 窗口 2 + 中值 1）；Canny 输出
    // 在 ORIG_LAT = 18 拍，故再延 Y_DLY = 14 拍与之同拍。
    reg [7:0] y_dly [0:Y_DLY-1];
    integer j;
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            for (j = 0; j < Y_DLY; j = j + 1)
                y_dly[j] <= 8'd0;
        end else begin
            y_dly[0] <= med_y;
            for (j = 1; j < Y_DLY; j = j + 1)
                y_dly[j] <= y_dly[j-1];
        end
    end

    // 分屏左半用的灰度：与 view 3 同一个抽头，已和 can_de / edge_rgb 同拍。
    wire [23:0] gray_rgb_view = {3{y_dly[Y_DLY-1]}};

    wire [23:0] edge_rgb = (view_i == 2'd1) ? {mag8_view, mag8_view, mag8_view} :
                           (view_i == 2'd2) ? cls_rgb :
                           (view_i == 2'd3) ? gray_rgb_view :
                                              (can_edge ? 24'hFFFFFF : 24'h000000);

    //-----------------------------------------------------------------
    // 6) 原图延迟 ORIG_LAT 拍（纯寄存器，无行缓存，保证 mode 0 逐像素一致）
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
    // 7) 合成：vsync / de 由 Canny 链给出，与两路图像严格同拍
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
        .gray_rgb_i (gray_rgb_view),
        .edge_rgb_i (edge_rgb),
        .mode_i     (mode_i),

        .vsync_o    (vsync_o),
        .de_o       (de_o),
        .rgb_o      (rgb_o)
    );

endmodule
