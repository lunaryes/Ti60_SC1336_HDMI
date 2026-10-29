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
//                     2026-10-19 增加方向约束流式反馈（link_en_i），不增加拍数
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
//   0 二值边缘图 / 1 mag8 灰度图 / 2 六状态假彩图 / 3 Y 灰度图。
//   详细口径见下方第 5 节的注释。view_i 只影响边缘支路的像素内容，**不影响
//   延迟**（四种视图的组合路径都在 image_mix 之前，由 image_mix 统一寄存）。
//
// 【smooth_mode_i】Canny 高斯前级的平滑程度（诊断分支合并）：
//   0 标准 1:2:1 高斯 / 1 弱平滑（高斯与原像素 1:1）/ 2 完全旁路。
//   模式在帧边界锁存，帧中改值不会在画面中间产生横向接缝；延迟不变。
//
// 【link_en_i】方向约束流式反馈开关（2026-10-19）：
//   1 = 弱边可以沿自己的切向传播、接到已保留的边缘上（长弱链不再只剩贴
//   强边的两三个像素）；0 = 退回原来的单遍滞后，edge_o 与加此功能前逐位
//   一致。在 canny_edge 的 vsync 上升沿锁存，帧内切换无接缝；**不影响延迟**。
//   view=2 把"仅因传播才保留"的弱边画成绿色，见第 5 节。
//=====================================================================
module image_pipeline_top #(
    parameter IMG_W       = 1280,
    parameter IMG_H       = 720,
    parameter LATENCY     = 20,      // 对外延迟（拍），与上面的推导一致
    // 中值滤波参数：默认只替换"局部极值且与中值差 ≥24"的脉冲点，干净台阶边
    // 会被原样保留；要更强的去噪可把 IMPULSE_ONLY 设 0（全 3x3 中值）。
    parameter integer MEDIAN_IMPULSE_ONLY  = 1,
    parameter [7:0]   MEDIAN_IMPULSE_DELTA = 8'd24,
    // ★ 2026-10-20 临时诊断：1 = view2 把"传播条件成立但不含门控"的弱边
    //   画成洋红，用来区分"条件没成立"和"被 enable/行列门控挡住"。
    //   默认 0，仿真参考模型（只认识绿/蓝/黄）不受影响；诊断完删掉。
    parameter integer DBG_LINK_COLOR = 0,
    // ★ 2026-10-20 黄色断边增强，见 canny_edge.v 同名参数注释。默认 0 =
    //   保持原有 ≤45° 约束和连续链语义（仿真参考模型不变）。
    parameter integer LINK_TURN90 = 0,
    parameter integer LINK_GAP1   = 0,
    // ★ 弱结构保留（不依赖强种子的第三条保留路径），见 canny_edge.v。
    parameter integer WEAK_STRUCT = 0,
    parameter integer EDGE_AA = 0,
    parameter integer TEMPORAL_EXT = 0
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
    input  wire        link_en_i,    // 1 = 切向流式反馈开；0 = 退回单遍滞后
    // 自适应阈值（Otsu，见 otsu_thr_gen.v）：auto_en_i=1 时强边阈值由
    // 每帧直方图自动算出，thr_i 退化为微调偏移（手动值−默认值，有符号）；
    // =0 时行为与旧版完全一致（thr_i 为绝对阈值），可随时回退。
    input  wire        auto_en_i,
    input  wire signed [8:0] thr_off_i,
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

    // ★ 2026-10-20 临时诊断口（定位板上没有绿色；见 canny_edge.v 同名注释）
    output wire        dbg_cond_o,
    output wire        dbg_link_en_o,
    output wire        dbg_cond_seen_o,
    output wire        dbg_link_seen_o,
    input wire [7:0] temporal_y_i,
    input wire temporal_de_i,temporal_vs_i,
    output wire        dbg_edge_o,
    output wire        dbg_edge_de_o,
    output wire        dbg_edge_vs_o
);

    localparam CANNY_LAT = 12;
    localparam WIN_LAT   = 2;        // pix_window_3x3：bank 同步读 1 拍 + 窗口寄存 1 拍
    localparam MED_LAT   = 1;        // median_filter_3x3：组合中值 + 输出寄存
    // 1 灰度 + 2 前窗 + 1 中值 + 2 后窗 + 12 Canny；进 image_mix 时再加 1 拍
    // 输出寄存器，所以 LATENCY = ORIG_LAT + 1。
    localparam ORIG_LAT  = 1 + WIN_LAT + MED_LAT + WIN_LAT + CANNY_LAT + (TEMPORAL_EXT?3:0);
    // view 3 的 Y 延迟：med_y 已经比 gray_y 晚 (WIN_LAT+MED_LAT)=3 拍，再延
    // Y_DLY 拍后与 can_de 同拍（ORIG_LAT-1 = 17 拍），最后经 image_mix 1 拍输出。
    localparam Y_DLY     = ORIG_LAT - 1 - (WIN_LAT + MED_LAT) - (TEMPORAL_EXT?3:0);

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

    wire [7:0] algorithm_y=TEMPORAL_EXT?temporal_y_i:med_y;
    wire algorithm_de=TEMPORAL_EXT?temporal_de_i:med_de;
    wire algorithm_vs=TEMPORAL_EXT?temporal_vs_i:med_vs;

    wire        win2_vs, win2_de;
    wire [7:0]  q11, q12, q13;
    wire [7:0]  q21, q22, q23;
    wire [7:0]  q31, q32, q33;

    pix_window_3x3 #(
        .IMG_W (IMG_W)
    ) u_pix_window_post_median (
        .clk     (clk_i),
        .rst_n   (rst_ni),

        .vsync_i (algorithm_vs),
        .de_i    (algorithm_de),
        .gray_i  (algorithm_y),

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
    wire        can_link;             // 仅因切向传播才保留的弱边，调试用
    wire        can_cond;             // ★诊断：传播条件（不含 enable/行列门控）
    wire        can_link_en;          // ★诊断：link_en_active 实际值
    wire        can_cond_seen;        // ★诊断：本帧出现过 cond
    wire        can_link_seen;        // ★诊断：本帧出现过 link
    assign dbg_edge_o=can_edge;
    assign dbg_edge_de_o=can_de;
    assign dbg_edge_vs_o=can_vs;

    //-----------------------------------------------------------------
    // 自适应阈值（Otsu）：统计 canny 输出域的 mag8 直方图（NMS 未过记 0，
    // 与论文口径一致），消隐期算出阈值、下帧生效。详见 otsu_thr_gen.v。
    // 手动路径保留：auto_en_i=0 时 thr_high/thr_low 与旧版逐位一致。
    //-----------------------------------------------------------------
    wire [7:0]  otsu_high;    // 自适应强边阈值（已钳位+滞回）
    wire [7:0]  otsu_raw;     // 钳位前原始 bin（观察用，暂不外引）

    otsu_thr_gen #(
        .IMG_W        (IMG_W),
        .THR_MIN      (8'd4),    // 与 canny_threshold_key_ctrl 的 THR_MIN 一致
        .THR_MAX      (8'd48),   // canny_edge.v 标定：>48≈关掉边缘检测
        .THR_DFLT     (8'd12),
        .THR_DEADBAND (2)
    ) u_otsu_thr (
        .clk        (clk_i),
        .rst_n      (rst_ni),
        .vsync_i    (can_vs),
        .de_i       (can_de),
        .mag8_i     (can_mag8),
        .cls_i      (can_cls),
        .thr_high_o (otsu_high),
        .thr_raw_o  (otsu_raw)
    );

    // auto 模式：Otsu 值 + 手动偏移，再钳到 [2,64]（手动偏移的护栏）
    wire signed [8:0] thr_auto_comb = $signed({1'b0, otsu_high}) + thr_off_i;
    wire [7:0] thr_auto = ($signed(thr_auto_comb) < 9'sd2)  ? 8'd2  :
                          ($signed(thr_auto_comb) > 9'sd64) ? 8'd64 :
                          thr_auto_comb[7:0];

    // 单一阈值口 → 双阈值（2:1），见文件头的【thr_i】
    wire [7:0]  thr_high = auto_en_i ? thr_auto : thr_i;
    wire [7:0]  thr_low  = {1'b0, thr_high[7:1]};

    canny_edge #(
        .IMG_W       (IMG_W),
        .LATENCY     (CANNY_LAT),
        .LINK_TURN90 (LINK_TURN90),
        .LINK_GAP1   (LINK_GAP1),
        .WEAK_STRUCT (WEAK_STRUCT),.EDGE_AA(EDGE_AA)
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
        .link_en_i  (link_en_i),.aa_o(can_aa),

        .vsync_o (can_vs),
        .de_o    (can_de),
        // mag_o 的 min(mag,255) 口径在显示上已被 mag8_o 取代 —— 它在 mag8>32 就
        // 饱和，没法用来调阈值（见 canny_edge.v 的 mag8_o 注释）。端口保留是为了
        // 不破坏已冻结的 I-B 语义与 sobel 的视觉对照，这里不再往外引。
        .mag_o   (),
        .edge_o  (can_edge),
        .link_o  (can_link),
        .mag8_o  (can_mag8),
        .cls_o   (can_cls),
        .cond_o       (can_cond),
        .link_en_dbg_o(can_link_en),
        .cond_seen_o  (can_cond_seen),
        .link_seen_o  (can_link_seen)
    );

    assign dbg_cond_o      = can_cond;
    assign dbg_link_en_o   = can_link_en;
    assign dbg_cond_seen_o = can_cond_seen;
    assign dbg_link_seen_o = can_link_seen;

    //-----------------------------------------------------------------
    // 5) 边缘支路显示内容，由 view_i 选（与 can_de / can_mag / can_edge 同拍）
    //    0 = 二值边缘图（白线黑底）——工程默认
    //    1 = mag8 灰度图，显示值 = min(mag8<<1, 255)
    //        阈值面直接可读：thr=16 落在灰度 32、thr=40 落在 80，满对比度黑边
    //        （mag8=96）落在 192。**不能拿 mag_o 调阈值**——它在 mag8>32 就饱和
    //        （见 canny_edge.v 的 mag8_o 注释）。
    //    2 = 六状态假彩图（"判死器"，诊断分支合并；2026-10-19 加绿色）：
    //          黑  = 无响应（mag8 < thr_low，该处真的没梯度）
    //          深红 = 有梯度但被 NMS 抑制
    //          黄  = 弱边（过 NMS，但未被滞后连接保留）
    //          蓝  = 弱边，被原来的 8 邻域单遍滞后保留
    //          绿  = 弱边，仅因切向流式传播才保留（link_en_i = 1 的新增部分）
    //          白  = 强边
    //        绿色直接回答"新逻辑到底连了什么"：关掉 link_en_i 后绿色应全部消失，
    //        原来蓝/黄/白的分布逐像素不变。
    //    3 = Y 灰度图（算法眼里的世界，中值后）。用来量 ΔY：屏幕上读出两侧
    //        灰度差，查 canny_edge.v 头注释的标尺表就知道该不该出边。
    //        ★ y_dly 只做时间对齐（14 拍），不做空间对齐：中值后的 Y 比原图
    //          下移 1 行，边缘图下移 4 行，所以本视图比边缘图高 3 行，量对比度
    //          不受影响。
    //-----------------------------------------------------------------
    wire [11:0] can_aa;
    wire [8:0] mag8x2      = {1'b0, can_mag8} << 1;
    wire [7:0] mag8_view   = mag8x2[8] ? 8'hFF : mag8x2[7:0];

    // 弱边再按 can_edge（滞后/传播结果）分成黄/蓝/绿：诊断时能直接看出
    // "弱梯度存在但没连上"、"被原来的单遍滞后保留"和"被切向传播连上"。
    wire [23:0] cls_rgb = (can_cls == 2'b11) ? 24'hFFFFFF :   // 强边     白
                          (can_cls == 2'b10) ?
                              (can_link ? 24'h00FF00 :        // 传播保留 绿
                               (DBG_LINK_COLOR && can_cond) ? 24'hFF00FF :
                                                              // ★诊断：条件成立但被门控
                               can_edge ? 24'h0000FF :        // 滞回连通 蓝
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
            y_dly[0] <= algorithm_y;
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
    // 7) Spatial alignment for RGB edge overlay (mode 3).
    // Median + post-median + Gaussian + gradient windows displace edges by
    // four rows relative to RGB, three relative to median grayscale.
    // Synchronous line reads add one common clock to every display mode.
    wire [23:0] overlay_orig;
    wire [7:0] overlay_gray;
    display_row_delay #(.WIDTH(24),.IMG_W(IMG_W),.ROWS(4)) u_overlay_orig_rows(
        .clk(clk_i),.rst_n(rst_ni),.vs_i(can_vs),.de_i(can_de),
        .data_i(orig_rgb),.data_o(overlay_orig));
    display_row_delay #(.WIDTH(8),.IMG_W(IMG_W),.ROWS(3)) u_overlay_gray_rows(
        .clk(clk_i),.rst_n(rst_ni),.vs_i(can_vs),.de_i(can_de),
        .data_i(gray_rgb_view[7:0]),.data_o(overlay_gray));
    reg aligned_vs,aligned_de;
    reg [23:0] orig_clock,edge_clock;
    reg [7:0] gray_clock;
    reg [12:0] point_clock;
    always @(posedge clk_i or negedge rst_ni) begin
        if(!rst_ni) begin
            aligned_vs<=0;aligned_de<=0;orig_clock<=0;edge_clock<=0;gray_clock<=0;point_clock<=0;
        end else begin
            aligned_vs<=can_vs;aligned_de<=can_de;
            orig_clock<=orig_rgb;gray_clock<=gray_rgb_view[7:0];
            edge_clock<=edge_rgb;point_clock<={can_edge,can_aa};
        end
    end
    wire [23:0] aligned_orig=(mode_i==3)?overlay_orig:orig_clock;
    wire [7:0] aligned_gray=(mode_i==3)?overlay_gray:gray_clock;

    // Common display alignment: optional 4-cycle renderer for all modes.
    //-----------------------------------------------------------------
    wire display_vs,display_de;
    wire [23:0] display_orig,display_gray,display_edge;
    generate if(EDGE_AA) begin: GEN_AA_DISPLAY
        edge_aa_renderer #(.IMG_W(IMG_W),.IMG_H(IMG_H)) u_aa(
            .clk(clk_i),.rst_n(rst_ni),.vs_i(aligned_vs),.de_i(aligned_de),
            .orig_i(aligned_orig),.gray_i(aligned_gray),.rgb_i(edge_clock),
            .point_i(point_clock),.aa_i(view_i==0),
            .vs_o(display_vs),.de_o(display_de),.orig_o(display_orig),
            .gray_o(display_gray),.rgb_o(display_edge));
    end else begin: GEN_NO_AA_DISPLAY
        assign display_vs=aligned_vs;assign display_de=aligned_de;
        assign display_orig=aligned_orig;assign display_gray={3{aligned_gray}};
        assign display_edge=edge_clock;
    end endgenerate
    image_mix #(
        .IMG_W (IMG_W),
        .IMG_H (IMG_H)
    ) u_image_mix (
        .clk_i      (clk_i),
        .rst_ni     (rst_ni),

        .vsync_i    (display_vs),
        .de_i       (display_de),
        .orig_rgb_i (display_orig),
        .gray_rgb_i (display_gray),
        .edge_rgb_i (display_edge),
        .mode_i     (mode_i),

        .vsync_o    (vsync_o),
        .de_o       (de_o),
        .rgb_o      (rgb_o)
    );

endmodule

// Local one-pixel-wide line coverage. DX/DY are pixel minus candidate.
// point={edge,axis_y,same_sign,positive_offset,offset_Q4[3:0],ratio_Q4[4:0]}.
// Approximate triangular pixel footprint, tangent support limited to 0.75px.
module edge_aa_coverage #(parameter integer DX=0,DY=0)(
    input wire [12:0] point_i,
    output reg [7:0] coverage_o
);
    reg [4:0] r;
    reg [3:0] f;
    reg [8:0] inv_norm,r2;
    reg [5:0] factor;
    reg [9:0] tangent_limit;
    reg signed [12:0] dot_value,tangent_value,signed_slope,slope_dx,slope_dy,offset_correction;
    reg [9:0] offset_product;
    localparam signed [12:0] BASE_X=DX*256,BASE_Y=DY*256;
    reg [12:0] abs_dot,abs_tangent;
    reg [21:0] distance_value;
    always @(*) begin
        r=(point_i[4:0]>16)?5'd16:point_i[4:0];
        f=(point_i[8:5]>8)?4'd8:point_i[8:5];
        case(r)
            5'd0:inv_norm=9'd256;
            5'd1:inv_norm=9'd256;
            5'd2:inv_norm=9'd254;
            5'd3:inv_norm=9'd252;
            5'd4:inv_norm=9'd248;
            5'd5:inv_norm=9'd244;
            5'd6:inv_norm=9'd240;
            5'd7:inv_norm=9'd235;
            5'd8:inv_norm=9'd229;
            5'd9:inv_norm=9'd223;
            5'd10:inv_norm=9'd217;
            5'd11:inv_norm=9'd211;
            5'd12:inv_norm=9'd205;
            5'd13:inv_norm=9'd199;
            5'd14:inv_norm=9'd193;
            5'd15:inv_norm=9'd187;
            5'd16:inv_norm=9'd181;
            default:inv_norm=9'd181;
        endcase
        case(r)
            5'd0:tangent_limit=10'd192;
            5'd1:tangent_limit=10'd192;
            5'd2:tangent_limit=10'd193;
            5'd3:tangent_limit=10'd195;
            5'd4:tangent_limit=10'd197;
            5'd5:tangent_limit=10'd201;
            5'd6:tangent_limit=10'd205;
            5'd7:tangent_limit=10'd209;
            5'd8:tangent_limit=10'd214;
            5'd9:tangent_limit=10'd220;
            5'd10:tangent_limit=10'd226;
            5'd11:tangent_limit=10'd232;
            5'd12:tangent_limit=10'd240;
            5'd13:tangent_limit=10'd247;
            5'd14:tangent_limit=10'd255;
            5'd15:tangent_limit=10'd263;
            5'd16:tangent_limit=10'd271;
            default:tangent_limit=10'd271;
        endcase
        r2=r*r;factor=6'd16+{1'b0,r2[8:4]};
        // Select signs explicitly: integer +/-1 products would infer wide DSPs.
        signed_slope=point_i[10]?$signed({4'd0,r,4'd0}):-$signed({4'd0,r,4'd0});
        slope_dx=(DX==1)?signed_slope:((DX==-1)?-signed_slope:13'sd0);
        slope_dy=(DY==1)?signed_slope:((DY==-1)?-signed_slope:13'sd0);
        offset_product=f*factor;
        offset_correction=point_i[9]?$signed({3'd0,offset_product}):-$signed({3'd0,offset_product});
        if(!point_i[11]) begin
            dot_value=BASE_X+slope_dy;
            tangent_value=BASE_Y-slope_dx;
        end else begin
            dot_value=BASE_Y+slope_dx;
            tangent_value=BASE_X-slope_dy;
        end
        dot_value=dot_value-offset_correction;
        abs_dot=dot_value[12]?-dot_value:dot_value;
        abs_tangent=tangent_value[12]?-tangent_value:tangent_value;
        distance_value=abs_dot*inv_norm;
        coverage_o=0;
        if(point_i[12] && abs_tangent<=tangent_limit && distance_value<22'd65280)
            coverage_o=8'd255-distance_value[15:8];
    end
endmodule

// Two window cycles + two arithmetic/output cycles. Every image path traverses
// the same center selection: one additional row of spatial window displacement.
module edge_aa_renderer #(parameter integer IMG_W=1280,IMG_H=720)(
    input wire clk,rst_n,vs_i,de_i,
    input wire [23:0] orig_i,rgb_i,
    input wire [7:0] gray_i,
    input wire [12:0] point_i,
    input wire aa_i,
    output reg vs_o,de_o,
    output reg [23:0] orig_o,gray_o,rgb_o
);
    wire [69:0] w11,w12,w13,w21,w22,w23,w31,w32,w33;
    wire wvs,wde;
    canny_win3x3 #(.W(70),.IMG_W(IMG_W)) u_window(
        .clk(clk),.rst_n(rst_n),.vsync_i(vs_i),.de_i(de_i),
        .din({orig_i,gray_i,rgb_i,aa_i,point_i}),.vsync_o(wvs),.de_o(wde),
        .w11(w11),.w12(w12),.w13(w13),.w21(w21),.w22(w22),.w23(w23),
        .w31(w31),.w32(w32),.w33(w33));
    wire [7:0] c11,c12,c13,c21,c22,c23,c31,c32,c33;
    edge_aa_coverage #(.DX(1),.DY(1)) a11(w11[12:0],c11);
    edge_aa_coverage #(.DX(0),.DY(1)) a12(w12[12:0],c12);
    edge_aa_coverage #(.DX(-1),.DY(1)) a13(w13[12:0],c13);
    edge_aa_coverage #(.DX(1),.DY(0)) a21(w21[12:0],c21);
    edge_aa_coverage #(.DX(0),.DY(0)) a22(w22[12:0],c22);
    edge_aa_coverage #(.DX(-1),.DY(0)) a23(w23[12:0],c23);
    edge_aa_coverage #(.DX(1),.DY(-1)) a31(w31[12:0],c31);
    edge_aa_coverage #(.DX(0),.DY(-1)) a32(w32[12:0],c32);
    edge_aa_coverage #(.DX(-1),.DY(-1)) a33(w33[12:0],c33);
    reg [7:0] q11,q12,q13,q21,q22,q23,q31,q32,q33;
    reg v1,d1,aa1;
    reg [23:0] orig1,rgb1;
    reg [7:0] gray1;
    reg wvs_d;
    reg [15:0] row,col;
    function [7:0] max3;
        input [7:0] a,b,c;
        begin max3=(a>=b && a>=c)?a:((b>=c)?b:c);end
    endfunction
    wire [7:0] maximum=max3(max3(q11,q12,q13),max3(q21,q22,q23),max3(q31,q32,q33));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            wvs_d<=0;row<=0;col<=0;
            q11<=0;q12<=0;q13<=0;q21<=0;q22<=0;q23<=0;q31<=0;q32<=0;q33<=0;
            v1<=0;d1<=0;aa1<=0;orig1<=0;gray1<=0;rgb1<=0;
            vs_o<=0;de_o<=0;orig_o<=0;gray_o<=0;rgb_o<=0;
        end else begin
            wvs_d<=wvs;
            if(wvs && !wvs_d) begin row<=0;col<=0;end
            else if(wde) begin
                if(col==IMG_W-1) begin col<=0;row<=row+1;end else col<=col+1;
            end
            q11<=c11;q12<=c12;q13<=c13;q21<=c21;q22<=c22;q23<=c23;q31<=c31;q32<=c32;q33<=c33;
            v1<=wvs;d1<=wde;
            aa1<=w22[13] && row>=2 && row<IMG_H-1 && col>=2 && col<IMG_W-2;
            orig1<=w22[69:46];gray1<=w22[45:38];rgb1<=w22[37:14];
            vs_o<=v1;de_o<=d1;orig_o<=orig1;gray_o<={3{gray1}};
            rgb_o<=aa1?{3{maximum}}:rgb1;
        end
    end
endmodule

// Active-pixel circular line store. Synchronous read-before-write infers BRAM.
// Reset only addresses/validity; RAM is not reset. Early-frame output is black
// until enough current-frame rows exist, preventing stale previous-frame data.
module display_row_delay #(parameter WIDTH=24,IMG_W=1280,ROWS=4)(
 input wire clk,rst_n,vs_i,de_i,input wire [WIDTH-1:0] data_i,
 output wire [WIDTH-1:0] data_o
);
 localparam DEPTH=IMG_W*ROWS,AW=$clog2(DEPTH);
 reg [WIDTH-1:0] memory[0:DEPTH-1];
 reg [AW-1:0] addr;reg [AW:0] filled;
 reg vs_d,valid;reg [WIDTH-1:0] q;
 wire frame_start=vs_i&&!vs_d;
 always @(posedge clk) begin
  if(de_i) begin q<=memory[addr];memory[addr]<=data_i;end
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin addr<=0;filled<=0;vs_d<=0;valid<=0;end
  else begin
   vs_d<=vs_i;valid<=de_i&&(filled>=DEPTH)&&!frame_start;
   if(frame_start) begin addr<=0;filled<=0;end
   else if(de_i) begin
    addr<=addr==DEPTH-1?0:addr+1'b1;
    if(filled<DEPTH) filled<=filled+1'b1;
   end
  end
 end
 assign data_o=valid?q:{WIDTH{1'b0}};
endmodule
