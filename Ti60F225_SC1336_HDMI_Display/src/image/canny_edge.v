`timescale 1ns / 1ps
//=====================================================================
// canny_edge  —— 接口 I-B
//---------------------------------------------------------------------
// 硬件近似版 Canny 边缘检测（方案见 docs/CANNY_PLAN.md）。
//
// 【当前版本：M3 + 方向约束流式反馈（2026-10-19，link_en_i / link_o）】
//   M3 = 全套：高斯 + 梯度 + 幅值 + 方向量化 + NMS + 双阈值滞后。
//   2026-10-19 在 M3 的 S11 里加了一条"沿边缘切向的流式反馈"：弱边如果沿着
//   自己的切向连到已经保留的边缘（强边或已传播的弱边），也保留。这条路径把
//   §3.7 的"单遍滞后"从"强边集外扩 1 像素"升级成**能沿弱链真正传播**，但仍是
//   单方向递推（只看 raster 扫描已经走过的三个切向邻居，链沿扫描方向生长）。
//   ★ 该功能**不增加流水线拍数**（LATENCY 仍是 12），可用 link_en_i 运行时关掉；
//     关掉后 edge_o 与 M3 逐位一致。link_o 标出"仅因传播才保留"的弱边像素，
//     view=2 用它把这些像素画成绿色（见 image_pipeline_top）。
//   2026-10-20 加三个**编译期**增强（默认关，仿真参考模型不变）：
//     LINK_TURN90=1 允许 90° 拐角转向（原判据只允许 ≤45°，网格线从强边拐进
//       垂直支路时会被挡住）；LINK_GAP1=1 允许跨过 1 个像素的断口（行缓存条目
//       多存一位 prev_edge），用来接上 thr 偏高时被阈值切断的弱链。
//       两者都只影响 link 的种子判据，不增加拍数、不改变 M3/link=0 的行为。
//     WEAK_STRUCT=1 是完全独立的第三条保留路径：弱像素的 8 邻域里有 ≥2 个
//       弱邻居就保留。不依赖强种子，专门救"整条都弱、又不挨强边"的细笔画
//       （实测典型：测试图里的数字轮廓）。
//   edge_o 的宽度：斜 45°/135° 实测最大 1 个像素，横/竖因方案 §11 决议 2 的
//   "平局保留"最多相邻 2 个像素，均与参考模型逐点一致。
//   端口与 sobel_edge 的差别只有阈值口：sobel 是单个 thr_i，这里是 thr_high_i /
//   thr_low_i 两个（§3.8 的 mag8 域，**与 sobel 的 thr_i 不可混用**）；p11..p33 /
//   mag_o / edge_o 语义不变，可就地替换实例。
//   mag_o 仍是未做 NMS、未过阈值的梯度幅值（饱和 8bit，与 sobel 的视觉亮度可直接
//   对比），细化与阈值都只体现在 edge_o 上。
//   接入顶层（image_pipeline_top，含 ORIG_LAT/LATENCY 改造）→ M4。
//
// 【与 sobel_edge 的差异】
//   1) 多一级 3x3 高斯 [1 2 1; 2 4 2; 1 2 1]/16（S1），移位加法实现、不占 DSP；
//   2) 高斯场要取 3x3 邻域，所以多了一组行 bank（S2/S3）—— sobel_edge 直接吃
//      灰度窗，没有这一级。方案 §3.1 原以为"全流程只需 2 个行缓存"，那是漏算
//      了高斯场自身的行历史：高斯把 3 行压成 1 行后，Sobel 还要它上下各一行，
//      必须自己存。实测 2 个 1280x8 bank（4 块 EFX_RAM10），见 §6 的偏差说明。
//   3) edge 语义：sobel 是 (饱和幅值 > thr)；这里是 NMS + 双阈值滞后，判据在
//      mag8 = mag>>3 的未饱和域。mag_o 的口径两者一致（都是饱和 8bit）。
//
// 【公式】
//   g   = (p11 + 2p12 + p13 + 2p21 + 4p22 + 2p23 + p31 + 2p32 + p33) >> 4  // 0..255
//   Gx  = (g13 + 2g23 + g33) - (g11 + 2g21 + g31)   右列 - 左列，-1020..+1020
//   Gy  = (g31 + 2g32 + g33) - (g11 + 2g12 + g13)   下行 - 上行，-1020..+1020
//   mag = |Gx| + |Gy|                               0..2040，L1 精确值
//   mag8 = mag >> 3                                 0..255，内部判据域（§3.8）
//   strong = NMS 过 && mag8 >= thr_high_i
//   weak   = NMS 过 && mag8 >= thr_low_i
//   edge   = strong || (weak && 8 邻域内含 strong) || link
//   link   = link_en && weak && 切向的"已走过"邻居是 edge 且方向相邻
//   系数 2 全部用左移一位（位拼接）实现，无乘法器、不占 DSP。
//
// ★ 方向约束流式反馈（2026-10-19）
//   单遍滞后的缺陷（§3.7）：弱链只有紧贴强边的一两个像素能活下来，再往里
//   全被截断。这里利用 S8 已经算好的方向 dir 和 S11 已经算好的 edge 结果，
//   沿**边缘切向**把"已经保留"的状态传给下一个像素。raster 扫描下每个中心
//   像素只可能从三个"过去"的切向邻居收到反馈（切向与扫描方向的交点）：
//     dir=00（梯度 0°，边竖直）  → 上      (R-1, C)
//     dir=01（梯度 90°，边水平） → 左      (R, C-1)
//     dir=10（梯度 45°，边 135°）→ 右上    (R-1, C+1)
//     dir=11（梯度 135°，边 45°）→ 左上    (R-1, C-1)
//   另外要求邻居的梯度扇区与本像素**相同或相邻**（扇区按 00→10→01→11 的
//   角度顺序循环，135° 与 0° 相邻），这样拐角处切向突变 90°（差两个扇区）
//   时不会硬连过去。反馈存 3 bit/像素 {dir, edge}，两个 1280×3 bank 按行
//   ping-pong（与 canny_win3x3 同一套 bank 轮转），只用 4 块 EFX_RAM10。
//   ★ 传播是**单向**的：每条弱链只能从 raster 顺序上更早的一端长出来；链的
//     起点必须已经是一个 strong 或单遍滞后保留的像素。没有种子的孤立弱链
//     仍然整条丢弃（这是与"连通域递归"的关键区别，也是流式架构的代价）。
//   ★ 前 7 行、左 3 列、右 2 列禁止传播：行缓存/边界复制在帧首和列边界有
//     已知暂态，反馈只在干净区启动；仿真参考模型用同样的门限（见
//     sim/canny_ref.py 的 LINK_GATE_*），否则启动差异会沿链一路传下去。
//
// ★★ 阈值口的量程（M5 标定事故的结论，调阈值前必读）★★
//   mag8 = mag>>3 是按理论满量程 2040 折算 0..255 的，但 3x3 高斯把 Sobel 的 4A
//   阶跃响应压成 **3A**，所以横竖边实际能达到的最大 mag 只有 768 ——
//   **mag8 的有效量程是 0..96（横竖边）/ 0..140（对角边），不是 0..255**。
//   把 thr_high 当成 "n/255 的对比度" 会系统性高估 2.7 倍。反查标尺：
//
//     thr        8    16    20    24    32    40    48
//     需要ΔY≥   21    42    53    63    85   106   127   （横竖边，ΔY = 边界两侧灰度差）
//     占满幅     8%   16%   21%   25%   33%   42%   50%
//
//   ★ 这几个数是 sim/color_probe.py 用本文件的定点算子**实算**出来的（含 >>4 截断），
//     不是解析式 3·ΔY/8 的估算 —— 后者会整体偏大 1（调阈值时 1 个 LSB 就要命）。
//     自动生成的完整表：python3 sim/color_probe.py
//
//   原来的默认值 40 实际要求 42% 的对比度 —— 这就是"只有高对比度才出边"的根因：
//   饱和红(Y≈99)/紫(≈90)/品红(≈105) 上的黑字边只有 mag8 37/33/39，全部落在
//   [thr_low, thr_high) 的弱带里，而单遍滞后只把强边集膨胀 1 像素，于是整条丢光。
//   经验约束：thr_high 不要超过 48（≈ 有效量程的一半），超过等于关掉边缘检测。
//
// 【流水线】LATENCY = 12 拍，vsync/de 用同深度移位寄存器对齐，
//   与 mag_o / edge_o 严格同拍：
//     S1  高斯             1
//     S2  高斯场同步读     1   ← canny_win3x3 #1（8bit）
//     S3  高斯场窗口       1
//     S4  加权和           1
//     S5  Gx / Gy          1
//     S6  |Gx|, |Gy|       1
//     S7  mag + 定向乘法   1   （|Gx|*106、|Gy|*256，为 S8 准备）
//     S8  方向量化         1   → 14bit 梯度字 {mag[11:0], dir[1:0]}
//     S9  梯度场同步读     1   ← canny_win3x3 #2（14bit）
//     S10 梯度场窗口       1
//     S11 NMS + 双阈值滞后 1   （复用同一个 3x3 梯度窗，不额外加行缓存）
//     S12 输出寄存         1
//   （sobel_edge 只有 4 拍：替换实例后 image_pipeline_top 的 ORIG_LAT 要跟着
//     改，见方案 §5，M4 一并处理。）
//
// 【空间对齐】窗口中心 = 与输出同拍进入的那一像素的"上一行、左一列"（与
//   pix_window_3x3 同一约定）；每多一级 3x3 窗口就多下移 1 行，M2 一共三级
//   （灰度的 pix_window_3x3 + 高斯场窗 + 梯度场窗），所以相对原图整体下移 3 行。
//   列方向上流水线自身的滞后与窗口的"左一列"恰好抵消，净偏移 0 列——M2 已实测
//   确认（在"按 de 行段抓取"的显示坐标系里，sim/canny_ref.py 十个图样 mag/edge
//   逐点 0 失配）：
//     pix_window_3x3.p22 相对原图              (-1, 0)
//     pix_window_3x3 + sobel_edge.mag 相对理想 (-1, 0)
//     pix_window_3x3 + canny_edge(M1).mag     (-2, 0)
//     pix_window_3x3 + canny_edge(M2).mag     (-3, 0)
//   ★ 抓取口径：必须"从本帧第一个输出行的行首"起抓，不能直接用帧计数器圈一段
//     抓——上一帧最后一行的输出还有 1 拍落在本帧开头（流水线尾巴），会让整个
//     数组错 1 个像素（看着像每行右移 1 列）。tb_canny_edge.v 已按行首起抓。
//   ★ 边界口径（比对时留边，与 pix_window_3x3 一致）：
//     · 左侧 0..3 列受"行首整窗复制晚 1 拍"影响；帧首第 3、4 行是行缓存还没填满
//       的暂态——mag 从第 5 行起干净，edge 因为 NMS 还要看一圈邻居，要第 6 行起
//       才干净。比对时 mag 留边 5、edge 留边 6（见 sim/canny_ref.py 的 margin）。
//     · 右侧末两列：实测（均匀场 128）末列的抽头 p33 读到的是**消隐期的 0**，
//       不是本行最后一个像素——3x3 窗的最后一次更新发生在输入已经转消隐的那一
//       拍（喂数据的一级在 de=0 时发 0），于是均匀场在末两列留下 mag = 32 / 16 的
//       小台阶（内部 mag8 = 2..4，远低于 thr_low，所以不产生任何 edge_o）。
//       这不是 M3 引入的：同一个图样单独喂 pix_window_3x3，末列 p33 同样是 0，
//       现有的 sobel 链路同样吃这一拍。要让末列彻底干净，应由喂数据的那一级在
//       消隐期保持最后一个像素而不是发 0（流水线级改动），留到 M4 一并决定。
//   ★ docs/INTERFACE.md 里"整体右下偏移 1 行 1 列"的列方向与实测不符（实为
//     1 行 0 列），M4 更新文档时一并订正。
//   ★ 3 行下移已经超出 FrameBoundCrop 的 SKIP_ROWS = 2：帧首/帧尾因行历史被
//     清空而产生的假边缘会漏 1 行到可见区。M4 接顶层时按需把黑边加到 3 行，
//     或接受这 1 行（方案 §11 未覆盖此处，届时说明）。
//
// 【vsync 约定】vsync_i 为"帧有效电平"（整个有效画面期间恒高），与
//   pix_window_3x3 一致：内部自己做帧首上升沿检测，直接透传电平即可。
//=====================================================================
module canny_edge #(
    parameter integer IMG_W   = 1280,
    parameter         LATENCY = 12,      // 对外延迟（拍），与上面的推导一致
    // ★ 2026-10-20 黄色断边增强（默认关：仿真参考模型只认识现有语义）
    //   LINK_TURN90 = 1：允许传播在 90° 拐角处转向。默认只允许 ≤45°，
    //                    网格线从强边拐进垂直支路时会被挡住，整条支路留黄。
    //   LINK_GAP1   = 1：允许跨过 1 个像素的断口继续传播（断线桥接）。
    //                    利用行缓存里新增的 prev_edge 位，不加额外行缓存。
    parameter integer LINK_TURN90 = 0,
    parameter integer LINK_GAP1   = 0,
    //   WEAK_STRUCT = 1：弱结构保留（不需要强种子）。弱像素的 8 邻域里
    //     至少有 2 个邻居的原始 mag8 ≥ thr_low 时，直接保留为边。
    //     细笔画（数字、网格线）经过 3×3 高斯后峰值被压低，常落在
    //     thr_low..thr_high 之间且不挨强边 —— link 永远够不到；而细线上
    //     每个像素天然有 ≥2 个同向弱邻居，孤立噪点没有，用连通性区分两者。
    //     默认 0：仿真参考模型不认识这条路径。
    parameter integer WEAK_STRUCT = 0,
    parameter integer EDGE_AA = 0
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        vsync_i,
    input  wire        de_i,
    input  wire [7:0]  p11, p12, p13,
    input  wire [7:0]  p21, p22, p23,
    input  wire [7:0]  p31, p32, p33,
    input  wire [7:0]  thr_high_i,
    input  wire [7:0]  thr_low_i,
    input  wire [1:0]  smooth_mode_i, // 0=Gaussian, 1=weak, 2=bypass
    input  wire        link_en_i,     // 方向约束流式反馈开关（vsync 上升沿锁存）

    output reg         vsync_o,
    output reg         de_o,
    output reg  [7:0]  mag_o,
    output reg         edge_o,
    output reg         link_o,        // 仅因切向传播才保留的弱边（调试，view=2 绿）

    //-----------------------------------------------------------------
    // 调试观测口（M5 新增，排查"只认高对比度"问题时加的）
    //   mag8_o：真实的阈值域 mag8 = mag>>3（未饱和，0..255），与 mag_o 同拍同像素。
    //     ★ 为什么要它：mag_o 是 min(mag,255)，而 mag8 = mag>>3 ⇒ mag_o 在 mag8 > 32
    //       就饱和了。thr_high 的默认值域在 16~48，对应的 mag = 128~384，**阈值平面
    //       落在 mag_o 的显示动态范围之外**——用 mag_o 图调阈值从根上不成立
    //       （33 和 96 在屏幕上是同一个纯白）。mag8_o 把这一级信息补回来。
    //   cls_o：判决分类，一次判死"故障出在阈值还是在逻辑"：
    //     00 = mag8 <  thr_low                      —— 无响应（该处真的没有梯度）
    //     01 = mag8 >= thr_low 但 NMS 没过          —— 有梯度、被非极大值抑制掉了
    //     10 = 过 NMS 且 thr_low <= mag8 < thr_high —— 弱边（滞回要连的那一类）
    //     11 = 过 NMS 且 mag8 >= thr_high           —— 强边
    //     cls_o = 10/11 而 edge_o = 0 ⇔ 逻辑/接线 bug；cls_o = 00 ⇔ 阈值再低也没用。
    //-----------------------------------------------------------------
    output reg  [7:0]  mag8_o,
    output reg  [1:0]  cls_o,

    //-----------------------------------------------------------------
    // ★ 2026-10-20 临时诊断口（板上"永远没有绿色"定位用，诊断完删除）：
    //   cond_o       = 传播条件本身（weak && !strong && !nb_strong &&
    //                  切向种子是 edge && 方向兼容），**不含** link_gate。
    //                  view=2 里用洋红显示：有洋红 = 种子/方向都对，只被
    //                  enable/行列门控挡住；没有洋红 = 条件根本没成立。
    //   link_en_dbg_o = link_en_active（帧首锁存的开关实际值），接 LED。
    //   cond_seen_o   = 本帧内 cond_c 是否出现过（帧首清零），接 LED。
    //   link_seen_o   = 本帧内 link_c 是否出现过（帧首清零），接 LED。
    //-----------------------------------------------------------------
    output reg         cond_o,
    output wire        link_en_dbg_o,
    output reg         cond_seen_o,
    output reg         link_seen_o,
    output reg [11:0] aa_o
);

    //-----------------------------------------------------------------
    // S1：3x3 高斯平滑，核 [1 2 1; 2 4 2; 1 2 1]/16
    //     行内 3 抽头（系数 2 用移位）→ 三行按 1:2:1 合并 → >>4
    //     和的上界 16*255 = 4080，12bit 承载；>>4 后恰好 0..255，不丢范围。
    //-----------------------------------------------------------------
    wire [9:0] gr1 = {1'b0, p11} + {p12, 1'b0} + {1'b0, p13};   // 上行
    wire [9:0] gr2 = {1'b0, p21} + {p22, 1'b0} + {1'b0, p23};   // 中行
    wire [9:0] gr3 = {1'b0, p31} + {p32, 1'b0} + {1'b0, p33};   // 下行

    wire [11:0] gsum = {1'b0, gr1} + {gr2, 1'b0} + {1'b0, gr3}; // 0..4080
    wire [7:0]  g_gaussian = gsum[11:4];                         // >>4

    // Apply mode changes only at the start of a frame.  This prevents a
    // UART command received mid-frame from creating a horizontal seam.
    reg         vsync_mode_d;
    reg  [1:0]  smooth_mode_active;
    reg         link_en_active;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_mode_d      <= 1'b0;
            smooth_mode_active <= 2'd0;
            link_en_active     <= 1'b0;
        end else begin
            vsync_mode_d <= vsync_i;
            if (vsync_i && !vsync_mode_d) begin
                smooth_mode_active <= smooth_mode_i;
                link_en_active     <= link_en_i;
            end
        end
    end

    // Weak mode blends the centre pixel and Gaussian output 1:1.  The +1
    // gives round-to-nearest before dividing by two.  All three modes keep
    // the same S1 register and therefore the same external latency.
    wire [8:0] g_weak_sum = {1'b0, p22} + {1'b0, g_gaussian} + 9'd1;
    wire [7:0] g_weak     = g_weak_sum[8:1];
    wire [7:0] g_selected = (smooth_mode_active == 2'd2) ? p22       :
                            (smooth_mode_active == 2'd1) ? g_weak    :
                                                                  g_gaussian;

    reg [7:0] g_s1;
    reg       vsync_s1, de_s1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            g_s1     <= 8'd0;
            vsync_s1 <= 1'b0;
            de_s1    <= 1'b0;
        end else begin
            g_s1     <= g_selected;
            vsync_s1 <= vsync_i;
            de_s1    <= de_i;
        end
    end

    //-----------------------------------------------------------------
    // S2/S3：高斯场的 3x3 邻域窗口（2 个行 bank 按行轮转 + 同步读）
    //   Sobel 需要"高斯场"的上下左右，而 pix_window_3x3 给的是灰度场，
    //   所以这里必须自己做一窗。见 canny_win3x3 头注释。
    //-----------------------------------------------------------------
    wire        gwin_vs, gwin_de;
    wire [7:0]  g11, g12, g13;
    wire [7:0]  g21, g22, g23;
    wire [7:0]  g31, g32, g33;

    canny_win3x3 #(
        .W     (8),
        .IMG_W (IMG_W)
    ) u_gwin (
        .clk     (clk),
        .rst_n   (rst_n),

        .vsync_i (vsync_s1),
        .de_i    (de_s1),
        .din     (g_s1),

        .vsync_o (gwin_vs),
        .de_o    (gwin_de),

        .w11 (g11), .w12 (g12), .w13 (g13),
        .w21 (g21), .w22 (g22), .w23 (g23),
        .w31 (g31), .w32 (g32), .w33 (g33)
    );

    //-----------------------------------------------------------------
    // S4：四个方向的加权和（与 sobel_edge 的 S1 同构，系数 2 用移位）
    //-----------------------------------------------------------------
    wire [9:0] c1 = {1'b0, g11} + {g21, 1'b0} + {1'b0, g31};   // 左列
    wire [9:0] c3 = {1'b0, g13} + {g23, 1'b0} + {1'b0, g33};   // 右列
    wire [9:0] r1 = {1'b0, g11} + {g12, 1'b0} + {1'b0, g13};   // 上行
    wire [9:0] r3 = {1'b0, g31} + {g32, 1'b0} + {1'b0, g33};   // 下行

    reg [9:0] c1_s4, c3_s4, r1_s4, r3_s4;
    reg       vsync_s4, de_s4;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c1_s4 <= 10'd0; c3_s4 <= 10'd0;
            r1_s4 <= 10'd0; r3_s4 <= 10'd0;
            vsync_s4 <= 1'b0; de_s4 <= 1'b0;
        end else begin
            c1_s4 <= c1; c3_s4 <= c3;
            r1_s4 <= r1; r3_s4 <= r3;
            vsync_s4 <= gwin_vs; de_s4 <= gwin_de;
        end
    end

    //-----------------------------------------------------------------
    // S5：Gx、Gy（有符号 11bit，-1020 .. +1020）
    //-----------------------------------------------------------------
    reg signed [10:0] gx_s5, gy_s5;
    reg               vsync_s5, de_s5;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gx_s5 <= 11'sd0; gy_s5 <= 11'sd0;
            vsync_s5 <= 1'b0; de_s5 <= 1'b0;
        end else begin
            gx_s5 <= $signed({1'b0, c3_s4}) - $signed({1'b0, c1_s4});
            gy_s5 <= $signed({1'b0, r3_s4}) - $signed({1'b0, r1_s4});
            vsync_s5 <= vsync_s4; de_s5 <= de_s4;
        end
    end

    //-----------------------------------------------------------------
    // S6：取绝对值
    //-----------------------------------------------------------------
    function [10:0] abs11;
        input signed [10:0] v;
        begin
            abs11 = v[10] ? (~v + 11'd1) : v;
        end
    endfunction

    reg [10:0] ax_s6, ay_s6;
    reg [1:0]  sign_s6;                  // {Gx 符号, Gy 符号}，与 ax_s6/ay_s6 同像素
    reg        vsync_s6, de_s6;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ax_s6 <= 11'd0; ay_s6 <= 11'd0; sign_s6 <= 2'b00;
            vsync_s6 <= 1'b0; de_s6 <= 1'b0;
        end else begin
            ax_s6 <= abs11(gx_s5);
            ay_s6 <= abs11(gy_s5);
            sign_s6 <= {gx_s5[10], gy_s5[10]};   // ★ 必须在这里取符号：下一拍 gx_s5/gy_s5
                                                 //   已经是下一个像素了
            vsync_s6 <= vsync_s5; de_s6 <= de_s5;
        end
    end

    //-----------------------------------------------------------------
    // S7：幅值 mag = |Gx| + |Gy|（0..2040，12bit）+ 方向量化要用的两个积
    //     ★ 方案 §3.4 已定：用精确 L1，不用 max + min/2 近似 —— NMS 靠幅值
    //       比较定极大值，近似会引入比较误差；L1 在硬件上反而更省。
    //     方向量化的定标常数见 S8；乘法同样用移位加法，不占 DSP。
    //-----------------------------------------------------------------
    wire [11:0] mag = {1'b0, ax_s6} + {1'b0, ay_s6};

    // |Gx| * 106 = |Gx|<<6 + <<5 + <<3 + <<1，20bit 承载（106*1020 = 108120）
    wire [19:0] ax106 = {3'b0, ax_s6, 6'b0} + {4'b0, ax_s6, 5'b0}
                      + {6'b0, ax_s6, 3'b0} + {8'b0, ax_s6, 1'b0};
    // |Gy| * 256 = |Gy|<<8，纯位拼接
    wire [19:0] ay256 = {1'b0, ay_s6, 8'b0};

    wire [6:0] geometry;
    reg [6:0] geometry_s7;
    edge_gradient_meta u_geometry(.ax_i(ax_s6),.ay_i(ay_s6),
        .same_i(~(sign_s6[1]^sign_s6[0])),.meta_o(geometry));
    reg [11:0] mag_s7;
    reg [19:0] ax106_s7, ay256_s7;
    reg [1:0]  sign_s7;                  // {Gx 符号, Gy 符号}，与 mag_s7 同像素
    reg        vsync_s7, de_s7;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mag_s7   <= 12'd0;
            geometry_s7<=0;
            ax106_s7 <= 20'd0;
            ay256_s7 <= 20'd0;
            sign_s7  <= 2'b00;
            vsync_s7 <= 1'b0;
            de_s7    <= 1'b0;
        end else begin
            mag_s7   <= mag;
            geometry_s7<=geometry;
            ax106_s7 <= ax106;
            ay256_s7 <= ay256;
            sign_s7  <= sign_s6;
            vsync_s7 <= vsync_s6;
            de_s7    <= de_s6;
        end
    end

    //-----------------------------------------------------------------
    // S8：方向量化（方案 §3.5）
    //   106/256 = 0.41406 ≈ tan(22.5°)，于是：
    //     |Gy| <  0.414|Gx|   ⟺ |Gy|*256 <  |Gx|*106  → 00 = 0°   （梯度近水平）
    //     |Gx| <  0.414|Gy|   ⟺ |Gx|*106 <  |Gy|*256  → 01 = 90°  （梯度近竖直）
    //     其余                → 对角，用 Gx*Gy 的符号分：
    //       同号（Gx、Gy 指向同一象限）→ 10 = 45°；异号 → 11 = 135°
    //   ★ 只用 106 这一个常数乘法：tan(67.5°) 的比较被"换一下操作数"等价实现了，
    //     省掉 618/256 那一组加法器。
    //   ★ 45°/135° 按"行号向下为 +y"的图像坐标定义，与 NMS 取邻域的方向一致。
    //   dir 的含义 = 该像素的梯度方向所在扇区，也就是 NMS 要比的两个邻居所在方向：
    //     00 → 同列左右（w21/w23）；01 → 同行上下（w12/w32）
    //     10 → 对角线（w11/w33）； 11 → 反对角线（w13/w31）
    //-----------------------------------------------------------------
    wire        dir_h = (ay256_s7 <  ax106_s7);            // 近水平
    wire        dir_v = (ax106_s7 <  ay256_s7);            // 近竖直
    wire        sq45  = ~(sign_s7[1] ^ sign_s7[0]);        // Gx、Gy 同号

    wire [1:0]  dir = dir_h ? 2'b00 :
                      dir_v ? 2'b01 :
                      sq45  ? 2'b10 : 2'b11;

    // 梯度字：14bit = {mag[11:0], dir[1:0]}
    //   ★ 方案 §11 决议 3：行缓存存 14bit 精确版，不存饱和后的 8bit ——
    //     NMS 是幅值比较，饱和到 255 后平局会大量出现，取消"严格极大值"的意义。
    reg [20:0] grad_s8;
    reg        vsync_s8, de_s8;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            grad_s8  <= 21'd0;
            vsync_s8 <= 1'b0;
            de_s8    <= 1'b0;
        end else begin
            grad_s8  <= {geometry_s7,mag_s7,dir};
            vsync_s8 <= vsync_s7;
            de_s8    <= de_s7;
        end
    end

    //-----------------------------------------------------------------
    // S9/S10：梯度场的 3x3 邻域窗口（canny_win3x3 #2）
    //   NMS 要拿中心像素的幅值和它 8 邻域的幅值比，所以梯度场自己也要一窗。
    //-----------------------------------------------------------------
    wire        mwin_vs, mwin_de;
    wire [20:0] m11, m12, m13;
    wire [20:0] m21, m22, m23;
    wire [20:0] m31, m32, m33;

    canny_win3x3 #(
        .W     (21),
        .IMG_W (IMG_W)
    ) u_mwin (
        .clk     (clk),
        .rst_n   (rst_n),

        .vsync_i (vsync_s8),
        .de_i    (de_s8),
        .din     (grad_s8),

        .vsync_o (mwin_vs),
        .de_o    (mwin_de),

        .w11 (m11), .w12 (m12), .w13 (m13),
        .w21 (m21), .w22 (m22), .w23 (m23),
        .w31 (m31), .w32 (m32), .w33 (m33)
    );

    //-----------------------------------------------------------------
    // S11：非极大值抑制（方案 §3.7）+ 双阈值滞后（方案 §3.7/§3.8）
    //   NMS：中心像素按自己的方向扇区取两个邻居比幅值；用 >= 保留平局
    //   （方案 §11 决议 2：不丢数据，宁可留粗一点也不把单像素线打断）。
    //   比较用**未饱和的 12bit 幅值**，不用 8bit 饱和值。
    //   滞后：strong = NMS 过且 mag8 >= thr_high；weak = NMS 过且 mag8 >= thr_low；
    //        edge = strong || (weak && 8 邻域内有 strong) || link。
    //   ★ 阈值域：mag8 = mag >> 3（0..2040 → 0..255，单调、无饱和平台），
    //     与 sobel_edge 的 thr_i（先饱和到 255 再比）**不可混用**。
    //   ★ 简化点（方案 §3.7 明示）：8 邻域的 strong 只看**未过 NMS 的原始幅值**，
    //     不是"邻居也过了 NMS"——单遍邻域判定，不是连通域递归；用同一个 3x3
    //     梯度窗，不额外加行缓存。link 是 2026-10-19 补的**切向单方向传播**，
    //     见下面的 S11b；它不改变这段单遍判定的语义，只是额外多一条保留路径。
    //   ★ thr_low 配成大于 thr_high 时 weak ⊆ strong，退化成单阈值，不会出错。
    //-----------------------------------------------------------------
    wire [11:0] mag_c = m22[13:2];       // 中心幅值
    wire [1:0]  dir_c = m22[1:0];        // 中心方向

    reg  [11:0] nb1, nb2;                // 组合：两个邻居的幅值

    always @(*) begin
        case (dir_c)
            2'b00:   begin nb1 = m21[13:2]; nb2 = m23[13:2]; end  // 左右
            2'b01:   begin nb1 = m12[13:2]; nb2 = m32[13:2]; end  // 上下
            2'b10:   begin nb1 = m11[13:2]; nb2 = m33[13:2]; end  // 对角线
            default: begin nb1 = m13[13:2]; nb2 = m31[13:2]; end  // 反对角线
        endcase
    end

    wire nms_legacy = (mag_c >= nb1) && (mag_c >= nb2);
    wire nms_interpolated;
    wire [11:0] aa_geometry;
    edge_aa_interpolate u_interpolated(
        .mag_i(mag_c),.ratio_i(m22[20:16]),.axis_y_i(m22[15]),.same_i(m22[14]),
        .m11(m11[13:2]),.m12(m12[13:2]),.m13(m13[13:2]),
        .m21(m21[13:2]),.m23(m23[13:2]),
        .m31(m31[13:2]),.m32(m32[13:2]),.m33(m33[13:2]),
        .keep_o(nms_interpolated),.geometry_o(aa_geometry));
    wire nms_keep = (EDGE_AA != 0) ? nms_interpolated : nms_legacy;

    // mag8：内部判据域（未饱和）。14bit 梯度字是 {mag[11:0], dir[1:0]}，
    // 即 w[13:2] = mag，所以 mag8 = mag >> 3 = w[13:5] 的低 8 位 = w[12:5]。
    // 直接写 w[12:5] 而不是 w[13:5]：mag = |Gx|+|Gy| 上界 2040 < 2048，mag[11]
    // 恒为 0（mag8 上界 255，也印证最高位恒 0），写全 9 bit 会被截掉最高位、
    // 综合工具也会报 WIDTHTRUNC。写成函数，免得两处取错比特位。
    function [7:0] mag8_of;
        input [20:0] w;
        begin
            mag8_of = w[12:5];
        end
    endfunction

    wire [7:0] mag8_c = mag8_of(m22);

    wire strong_c = nms_keep && (mag8_c >= thr_high_i);
    wire weak_c   = nms_keep && (mag8_c >= thr_low_i);

    // 8 邻域"有强边"：邻居也各自 >>3，只看原始幅值（不看它有没有过 NMS）
    wire nb_strong = (mag8_of(m11) >= thr_high_i) || (mag8_of(m12) >= thr_high_i) ||
                     (mag8_of(m13) >= thr_high_i) || (mag8_of(m21) >= thr_high_i) ||
                     (mag8_of(m23) >= thr_high_i) || (mag8_of(m31) >= thr_high_i) ||
                     (mag8_of(m32) >= thr_high_i) || (mag8_of(m33) >= thr_high_i);

    // 8 邻域"弱邻居"计数（WEAK_STRUCT）：同样只看原始 mag8，不看邻居的 NMS。
    wire [7:0] weak_nb_vec = {(mag8_of(m11) >= thr_low_i), (mag8_of(m12) >= thr_low_i),
                              (mag8_of(m13) >= thr_low_i), (mag8_of(m21) >= thr_low_i),
                              (mag8_of(m23) >= thr_low_i), (mag8_of(m31) >= thr_low_i),
                              (mag8_of(m32) >= thr_low_i), (mag8_of(m33) >= thr_low_i)};
    wire [3:0] weak_nb_cnt = {3'd0, weak_nb_vec[0]} + {3'd0, weak_nb_vec[1]} +
                             {3'd0, weak_nb_vec[2]} + {3'd0, weak_nb_vec[3]} +
                             {3'd0, weak_nb_vec[4]} + {3'd0, weak_nb_vec[5]} +
                             {3'd0, weak_nb_vec[6]} + {3'd0, weak_nb_vec[7]};
    // 细线上每个像素至少有 2 个同向弱邻居；孤立点/孤立两点不满足。
    wire weak_struct_c = weak_c && (weak_nb_cnt >= 4'd2);

    //-------------------------------------------------------------
    // S11b：方向约束流式反馈（link_en_i）
    //   反馈存每像素 4 bit = {prev_edge, dir[1:0], edge}（prev_edge 供 LINK_GAP1
    //   桥接 1 像素断口；LINK_GAP1=0 时该位不参与判决，行为与 3 bit 版一致）。
    //   两个 IMG_W×4 bank 按行轮转：
    //   当前行写 bank(sel)，上一行从 bank(sel^1) 读。语义与 canny_win3x3 的
    //   两 bank 完全一致，只是读地址要提前 2 列（见下面的预测式读地址）。
    //
    //   读地址为什么是 col+2：
    //     需要的是"上一行、当前中心列的右邻" up-right = prev[C+1]。S11 的
    //     中心列 C 在当拍，RAM 同步读要下一拍才出数，所以必须在 C-1 那拍就
    //     把地址 C+1 发出去；即每拍发 fb_col+2。读出的 q 经两级移位得到
    //     q=prev[C+1]（右上）、q_d1=prev[C]（上）、q_d2=prev[C-1]（左上）。
    //   左邻不需要 RAM：C-1 的中心结果就是上一拍的 fb_entry_c，一个 4bit
    //     寄存器即可（fb_left）。
    //
    //   边界门限（必须与 sim/canny_ref.py 的 LINK_GATE_* 完全一致）：
    //     · 行 0..6 不传播：帧首行缓存没填满，edge 第 6 行起才干净；第 7 行
    //       的切向种子只来自第 6 行（干净）。
    //     · 列 0..2 不传播：读地址预测在行首前 3 拍还落在消隐期的历史值上。
    //     · 列 W-2..W-1 不传播：行尾读地址进入 FB_LAST 钳位区，up-right
    //       不再对应当前行的有效像素（留 2 列，与参考模型的 W-3 上界一致）。
    //-------------------------------------------------------------
    localparam integer FB_AW = (IMG_W <= 2) ? 1 : $clog2(IMG_W);
    localparam [FB_AW-1:0] FB_LAST     = IMG_W - 1;
    localparam [FB_AW-1:0] FB_COL_LAST2 = IMG_W - 2;
    localparam [FB_AW-1:0] FB_COL_MAX  = IMG_W - 3;

    reg [FB_AW-1:0] fb_col;          // 中心列号，与 mwin_de 同拍 0..IMG_W-1
    reg             fb_bank_sel;     // 1 = bank1 是当前行（写），bank0 是上一行（读）
    reg             fb_de_d;
    reg             fb_vs_d;
    reg [3:0]       fb_row_cnt;      // 饱和行计数，只用来屏蔽前 7 行

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fb_col      <= {FB_AW{1'b0}};
            fb_bank_sel <= 1'b0;
            fb_de_d     <= 1'b0;
            fb_vs_d     <= 1'b0;
            fb_row_cnt  <= 4'd0;
        end else begin
            fb_de_d <= mwin_de;
            fb_vs_d <= mwin_vs;

            if (mwin_vs && !fb_vs_d) begin
                fb_bank_sel <= 1'b0;
                fb_row_cnt  <= 4'd0;
            end else if (fb_de_d && !mwin_de) begin
                // 行尾翻页；行计数在行尾递增，保证整行看到同一个行号
                fb_bank_sel <= ~fb_bank_sel;
                if (fb_row_cnt != 4'd15)
                    fb_row_cnt <= fb_row_cnt + 1'b1;
            end

            if (!mwin_de)
                fb_col <= {FB_AW{1'b0}};
            else if (fb_col == FB_LAST)
                fb_col <= {FB_AW{1'b0}};
            else
                fb_col <= fb_col + 1'b1;
        end
    end

    wire [FB_AW-1:0] fb_rd_addr = (!mwin_de) ? {FB_AW{1'b0}} :
                                  (fb_col >= FB_COL_LAST2) ? FB_LAST :
                                  (fb_col + 2'd2);

    // 行缓存条目 4 bit = {prev_edge, dir[1:0], edge}：
    //   [0]   本像素是边
    //   [2:1] 本像素的 NMS 方向
    //   [3]   本像素的切向过去邻居是边且方向相容（LINK_GAP1 桥接用）
    // [3] 只由真边（[0]）产生，不会把"桥接"再桥接下去，所以最多跨 1 个断口。
    reg [3:0] fb_bank0 [0:IMG_W-1];
    reg [3:0] fb_bank1 [0:IMG_W-1];
    reg [3:0] fb_q0, fb_q1;

    // bank_sel=0: 写 bank0(当前行)，读 bank1(上一行)；=1 时互换。
    wire [3:0] fb_prev = fb_bank_sel ? fb_q0 : fb_q1;

    reg [3:0] fb_prev_d1, fb_prev_d2;
    reg [3:0] fb_left;

    // 按中心像素自己的方向选"切向的过去邻居"
    reg [3:0] fb_seed;
    always @(*) begin
        case (dir_c)
            2'b00:   fb_seed = fb_prev_d1;   // 上
            2'b01:   fb_seed = fb_left;      // 左
            2'b10:   fb_seed = fb_prev;      // 右上
            default: fb_seed = fb_prev_d2;   // 左上
        endcase
    end

    // 方向相邻判据：扇区角度顺序 00(0°)→10(45°)→01(90°)→11(135°)，
    // 135° 与 0° 在 mod 180° 下相邻。差 2 个扇区（90°）= 拐角，不连。
    function [1:0] dir_rank;
        input [1:0] d;
        begin
            case (d)
                2'b00:   dir_rank = 2'd0;
                2'b10:   dir_rank = 2'd1;
                2'b01:   dir_rank = 2'd2;
                default: dir_rank = 2'd3;
            endcase
        end
    endfunction

    wire [1:0] dir_rank_c = dir_rank(dir_c);
    wire [1:0] dir_rank_n = dir_rank(fb_seed[2:1]);
    wire [1:0] dir_delta  = (dir_rank_c >= dir_rank_n) ?
                            (dir_rank_c - dir_rank_n) :
                            (dir_rank_n - dir_rank_c);
    // 默认只允许 ≤45°（差 0 或 3 个扇区）；LINK_TURN90=1 时也允许 90°（差 2）。
    wire dir_compat = (dir_delta == 2'd0) || (dir_delta == 2'd3) ||
                      ((LINK_TURN90 != 0) && (dir_delta == 2'd2));

    // 直接种子：切向过去邻居本身是边且方向相容。
    wire seed_direct_c = fb_seed[0] && dir_compat;
    // 断口桥接：过去邻居不是边，但它再往前一个切向像素是边（且与它相容）。
    // fb_seed[3] 来自行缓存的 prev_edge 位，只由真边产生 → 最多跨 1 个断口。
    wire seed_gap_c = (LINK_GAP1 != 0) && !fb_seed[0] && fb_seed[3] && dir_compat;
    wire seed_ok_c  = seed_direct_c || seed_gap_c;

    wire link_col_ok = (fb_col >= 2'd3) && (fb_col <= FB_COL_MAX);
    wire link_row_ok = (fb_row_cnt >= 4'd7);
    wire link_gate   = link_en_active && link_col_ok && link_row_ok;

    // 诊断：与 link_c 完全相同的条件，但去掉 link_gate（enable + 行/列门控）。
    wire cond_c = weak_c && !strong_c && !nb_strong && seed_ok_c;

    // 写回行缓存的条目（[3] 必须是"直接种子"，不能把桥接位再存回去）。
    wire [3:0] fb_entry_c = {seed_direct_c, dir_c, edge_c};

    // link 只标"本来会被丢掉、因为传播才留下"的弱边：
    // 已由 strong / 单遍滞后保留的像素不置 link（颜色仍按原语义）。
    wire link_c = link_gate && cond_c;

    wire edge_c = strong_c || (weak_c && nb_strong) ||
                  ((WEAK_STRUCT != 0) && weak_struct_c) || link_c;

    // 行历史右移链 + 同列左邻寄存器。q/q_d1/q_d2 的列对齐见上面的读地址说明。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fb_prev_d1 <= 4'd0;
            fb_prev_d2 <= 4'd0;
            fb_left    <= 4'd0;
        end else begin
            fb_prev_d2 <= fb_prev_d1;
            fb_prev_d1 <= fb_prev;
            if (mwin_de)
                fb_left <= fb_entry_c;
        end
    end

    // 反馈 RAM：单时钟、1 读 1 写，不做复位。当前行写、上一行读。
    always @(posedge clk) begin
        if (mwin_de && fb_bank_sel == 1'b0)
            fb_bank0[fb_col] <= fb_entry_c;
        fb_q0 <= fb_bank0[fb_rd_addr];
    end

    always @(posedge clk) begin
        if (mwin_de && fb_bank_sel == 1'b1)
            fb_bank1[fb_col] <= fb_entry_c;
        fb_q1 <= fb_bank1[fb_rd_addr];
    end

    // 调试判决分类（口径见文件头 cls_o）。按优先级串写而不是 {strong, weak}：
    // 方案 §3.7 允许 thr_low > thr_high（退化成单阈值），那时 strong ⊆ weak，
    // 直接拼位会把强边错报成弱边。
    wire [1:0] cls_c = strong_c              ? 2'b11 :
                       weak_c                ? 2'b10 :
                       (mag8_c >= thr_low_i) ? 2'b01 : 2'b00;

    // ★ 临时诊断：帧首锁存的 enable 实际值直接送 LED；再统计本帧内是否
    //   出现过 cond / link（LED 用；即使 vsync 一直不上升，粘滞位也不会丢）。
    assign link_en_dbg_o = link_en_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cond_seen_o <= 1'b0;
            link_seen_o <= 1'b0;
        end else if (vsync_i && !vsync_mode_d) begin
            cond_seen_o <= 1'b0;
            link_seen_o <= 1'b0;
        end else begin
            if (mwin_de && cond_c) cond_seen_o <= 1'b1;
            if (mwin_de && link_c) link_seen_o <= 1'b1;
        end
    end

    reg [11:0] aa_s11;
    reg [11:0] mag_s11;
    reg [7:0]  mag8_s11;
    reg [1:0]  cls_s11;
    reg        edge_s11;
    reg        link_s11;
    reg        cond_s11;
    reg        vsync_s11, de_s11;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mag_s11   <= 12'd0;
            aa_s11<=0;
            mag8_s11  <= 8'd0;
            cls_s11   <= 2'b00;
            edge_s11  <= 1'b0;
            link_s11  <= 1'b0;
            cond_s11  <= 1'b0;
            vsync_s11 <= 1'b0;
            de_s11    <= 1'b0;
        end else begin
            mag_s11   <= mag_c;
            aa_s11<=aa_geometry;
            mag8_s11  <= mag8_c;
            cls_s11   <= cls_c;
            edge_s11  <= edge_c;
            link_s11  <= link_c;
            cond_s11  <= cond_c;
            vsync_s11 <= mwin_vs;
            de_s11    <= mwin_de;
        end
    end

    //-----------------------------------------------------------------
    // S12：输出寄存
    //   mag_o 饱和到 8bit（与 sobel 的视觉亮度一致，取窗口中心那一像素）；
    //   edge_o = S11 算好的滞后结果，按 de 掩掉消隐期。
    //   link_o = "仅因切向传播才保留"的弱边标志（调试/假彩图用），与 edge_o
    //     同拍同像素；link_en_i = 0 时恒为 0。
    //   de 为 0 的区段所有输出都必须为 0（方案 §8 的仿真用例）。
    //   ★ mag_o 取窗口中心的幅值，与 edge_o 严格指向同一像素；这样板上 mag 图
    //     和 edge 图不会错位（代价是相对 M1 又多下移 1 行，见文件头空间对齐）。
    //-----------------------------------------------------------------
    wire [7:0] mag_sat = (mag_s11 > 12'd255) ? 8'hFF : mag_s11[7:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_o <= 1'b0;
            de_o    <= 1'b0;
            mag_o   <= 8'd0;
            edge_o  <= 1'b0;
            aa_o<=0;
            link_o  <= 1'b0;
            mag8_o  <= 8'd0;
            cls_o   <= 2'b00;
            cond_o  <= 1'b0;
        end else begin
            vsync_o <= vsync_s11;
            de_o    <= de_s11;
            mag_o   <= de_s11 ? mag_sat : 8'd0;
            edge_o  <= de_s11 & edge_s11;
            aa_o<=de_s11 ? aa_s11:12'd0;
            link_o  <= de_s11 & link_s11;
            cond_o  <= de_s11 & cond_s11;
            // 调试口与 mag_o/edge_o 同一口径：de = 0 的区段一律出 0
            mag8_o  <= de_s11 ? mag8_s11 : 8'd0;
            cls_o   <= de_s11 ? cls_s11  : 2'b00;
        end
    end

endmodule


//=====================================================================
// canny_win3x3 —— canny_edge 内部辅助模块（不作为 I-B 对外接口）
//---------------------------------------------------------------------
// 把流式像素流（每拍 1 个，raster 顺序、de 有效）变成 3x3 邻域窗口 w11..w33。
//
// 【2 个行 bank 按行轮转】
//   bank(sel)    = 上上行（正在被当前行就地改写）
//   bank(sel^1)  = 上一行（只读）
//   写地址 = col_cnt，读地址 = col_cnt+1（预测式）——读写恒不同址：
//     · 写 bank 里的"上上行"在地址 x 被改写时，该地址的数据早在本拍之前就读走了
//       （读地址始终比写地址大 1），不会读到被覆盖的值；
//     · 行尾/消隐读地址回绕到 0，而写地址是 COL_LAST，仍然不同址。
//   两拍后读出的数据正好落在"当前列"，于是行/列都天然对齐，不需要额外搬移。
//   （pix_window_3x3 用 3 个 bank 是为了另有"当前行"要存；这里当前行直接来自
//     输入流，2 个 bank 就够，省 2 块 EFX_RAM10。）
//
// 【延迟 2 拍】bank 同步读 1 拍 + 窗口寄存 1 拍，vsync_o/de_o 同深度对齐。
//
// 【窗口语义】与 pix_window_3x3 完全一致：窗口中心 = "与输出同拍进入的那一拍
//   数据的上一行、左一列"。两级窗口串联时偏移量可以直接相加。
//
// 【边界】行首（读侧第一拍）整窗复制"行首那两行 + 当前行的同一列"，即左侧边缘
//   复制、不补零；行尾那一拍读地址回绕，末列右邻取到的是第 0 列。这两种情况都
//   落在最外 2 行 2 列，与 pix_window_3x3 的处理口径一致（见 INTERFACE.md §3）。
//
// 【前提】与 pix_window_3x3 相同：每行 de 拉低后至少 2 拍消隐（本设计 H 消隐
//   370 拍，富余）。
//=====================================================================
module canny_win3x3 #(
    parameter integer W     = 8,
    parameter integer IMG_W = 1280
)(
    input  wire         clk,
    input  wire         rst_n,

    input  wire         vsync_i,
    input  wire         de_i,
    input  wire [W-1:0] din,

    output wire         vsync_o,
    output wire         de_o,

    output reg  [W-1:0] w11, w12, w13,
    output reg  [W-1:0] w21, w22, w23,
    output reg  [W-1:0] w31, w32, w33
);

    localparam integer  AW       = (IMG_W <= 2) ? 1 : $clog2(IMG_W);
    localparam [AW-1:0] COL_ZERO = {AW{1'b0}};
    localparam [AW-1:0] COL_LAST = IMG_W - 1;

    //-----------------------------------------------------------------
    // 1) 帧首检测 + 握手延迟（2 拍，与数据同延迟）
    //-----------------------------------------------------------------
    reg       vsync_d, de_d;
    reg [1:0] vsync_sr, de_sr;

    wire vsync_start  = vsync_i && !vsync_d;    // 帧首单拍
    wire de_r         = de_sr[0];               // 读侧使能（比 de_i 晚 1 拍）
    wire line_start_r = de_sr[0] && !de_sr[1];  // 读侧行首

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_d  <= 1'b0;
            de_d     <= 1'b0;
            vsync_sr <= 2'b00;
            de_sr    <= 2'b00;
        end else begin
            vsync_d  <= vsync_i;
            de_d     <= de_i;
            vsync_sr <= {vsync_sr[0], vsync_i};
            de_sr    <= {de_sr[0],    de_i};
        end
    end

    assign vsync_o = vsync_sr[1];
    assign de_o    = de_sr[1];

    //-----------------------------------------------------------------
    // 2) 列计数：de 期间 0..IMG_W-1，消隐期归零（写地址用它）
    //-----------------------------------------------------------------
    reg [AW-1:0] col_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            col_cnt <= COL_ZERO;
        else if (!de_i)
            col_cnt <= COL_ZERO;                // 消隐期归零 -> 行首读地址为 0
        else if (col_cnt == COL_LAST)
            col_cnt <= COL_ZERO;
        else
            col_cnt <= col_cnt + 1'b1;
    end

    //-----------------------------------------------------------------
    // 3) bank 轮转：行尾翻页，帧首归零
    //-----------------------------------------------------------------
    reg bank_sel;                               // 1 = bank1 是"上上行"（写 bank）

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bank_sel <= 1'b0;
        else if (vsync_start)
            bank_sel <= 1'b0;
        else if (de_d && !de_i)                 // 行尾翻页
            bank_sel <= ~bank_sel;
    end

    //-----------------------------------------------------------------
    // 4) 预测式读地址：提前一拍发 col+1
    //    当拍发 col+1，下一拍读出的正是"下一拍那一列"的历史行数据，于是
    //    rd_prev1/rd_prev2 与当拍 din 落在同一列，三行对齐、零错位。
    //    行尾与消隐期发 0：既避开 col+1 越界，又让每行第一拍读到第 0 列。
    //-----------------------------------------------------------------
    wire [AW-1:0] rd_addr = (!de_i || col_cnt == COL_LAST) ? COL_ZERO
                                                          : (col_cnt + 1'b1);

    //-----------------------------------------------------------------
    // 5) 2 个行 bank（单时钟、1 读 1 写，标准 SDP 推断写法；不做复位）
    //-----------------------------------------------------------------
    reg [W-1:0] bank0 [0:IMG_W-1];
    reg [W-1:0] bank1 [0:IMG_W-1];

    reg [W-1:0] q0, q1;

    always @(posedge clk) begin
        if (de_i && bank_sel == 1'b0) bank0[col_cnt] <= din;
        q0 <= bank0[rd_addr];
    end

    always @(posedge clk) begin
        if (de_i && bank_sel == 1'b1) bank1[col_cnt] <= din;
        q1 <= bank1[rd_addr];
    end

    // bank_sel = 0 时：bank0 = 上上行，bank1 = 上一行；bank_sel = 1 时互换。
    wire [W-1:0] rd_prev1 = bank_sel ? q0 : q1;   // 上一行，当前列
    wire [W-1:0] rd_prev2 = bank_sel ? q1 : q0;   // 上上行，当前列

    //-----------------------------------------------------------------
    // 6) 窗口寄存器（读侧）
    //    rowN_d1 / rowN_d2 是各行"当前列"的 1/2 拍历史，正好是左邻、左二邻。
    //    窗口中心 = 上一行、左一列：w2x 取上一行，w1x 取上上行，w3x 取当前行。
    //-----------------------------------------------------------------
    reg [W-1:0] row1_d1, row1_d2;     // 上上行的 1/2 拍历史
    reg [W-1:0] row2_d1, row2_d2;     // 上一行的 1/2 拍历史
    reg [W-1:0] row3_d1, row3_d2;     // 当前行的 1/2 拍历史

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row1_d1 <= {W{1'b0}}; row1_d2 <= {W{1'b0}};
            row2_d1 <= {W{1'b0}}; row2_d2 <= {W{1'b0}};
            row3_d1 <= {W{1'b0}}; row3_d2 <= {W{1'b0}};

            w11 <= {W{1'b0}}; w12 <= {W{1'b0}}; w13 <= {W{1'b0}};
            w21 <= {W{1'b0}}; w22 <= {W{1'b0}}; w23 <= {W{1'b0}};
            w31 <= {W{1'b0}}; w32 <= {W{1'b0}}; w33 <= {W{1'b0}};
        end else if (vsync_start) begin
            // 帧首清历史，避免首帧前几拍读到未定义的残值
            row1_d1 <= {W{1'b0}}; row1_d2 <= {W{1'b0}};
            row2_d1 <= {W{1'b0}}; row2_d2 <= {W{1'b0}};
            row3_d1 <= {W{1'b0}}; row3_d2 <= {W{1'b0}};
        end else if (de_r) begin
            if (line_start_r) begin
                // 行首：左邻缺失，整窗复制"当前列"（左边缘复制、不补零）
                w11 <= rd_prev2; w12 <= rd_prev2; w13 <= rd_prev2;
                w21 <= rd_prev1; w22 <= rd_prev1; w23 <= rd_prev1;
                w31 <= din;      w32 <= din;      w33 <= din;

                row1_d1 <= rd_prev2; row1_d2 <= rd_prev2;
                row2_d1 <= rd_prev1; row2_d2 <= rd_prev1;
                row3_d1 <= din;      row3_d2 <= din;
            end else begin
                w11 <= row1_d2; w12 <= row1_d1; w13 <= rd_prev2;
                w21 <= row2_d2; w22 <= row2_d1; w23 <= rd_prev1;
                w31 <= row3_d2; w32 <= row3_d1; w33 <= din;

                row1_d2 <= row1_d1; row1_d1 <= rd_prev2;
                row2_d2 <= row2_d1; row2_d1 <= rd_prev1;
                row3_d2 <= row3_d1; row3_d1 <= din;
            end
        end
    end

endmodule

// Q4 minor/major ratio, four restoring steps, no variable divider.
module edge_gradient_meta(
    input wire [10:0] ax_i,ay_i,
    input wire same_i,
    output wire [6:0] meta_o
);
    reg [11:0] remainder_value,denominator;
    reg [4:0] ratio;
    integer k;
    always @(*) begin
        denominator={1'b0,(ax_i>=ay_i?ax_i:ay_i)};
        remainder_value={1'b0,(ax_i>=ay_i?ay_i:ax_i)};
        ratio=0;
        if(denominator!=0) begin
            if(remainder_value>=denominator) ratio=16;
            else for(k=0;k<4;k=k+1) begin
                remainder_value=remainder_value<<1;ratio=ratio<<1;
                if(remainder_value>=denominator) begin
                    remainder_value=remainder_value-denominator;ratio=ratio|5'd1;
                end
            end
        end
    end
    assign meta_o={ratio,(ay_i>ax_i),same_i};
endmodule

// Canonical normal: right for X-major, down for Y-major.
// geometry={axis_y,same_sign,positive_offset,abs_offset_Q4[3:0],ratio_Q4[4:0]}.
module edge_aa_interpolate(
    input wire [11:0] mag_i,
    input wire [4:0] ratio_i,
    input wire axis_y_i,same_i,
    input wire [11:0] m11,m12,m13,m21,m23,m31,m32,m33,
    output wire keep_o,
    output wire [11:0] geometry_o
);
    reg [11:0] before_axis,after_axis,before_diag,after_diag;
    always @(*) begin
        if(!axis_y_i) begin
            before_axis=m21;after_axis=m23;
            before_diag=same_i?m11:m31;after_diag=same_i?m33:m13;
        end else begin
            before_axis=m12;after_axis=m32;
            before_diag=same_i?m11:m13;after_diag=same_i?m33:m31;
        end
    end
    wire [4:0] r=(ratio_i>16)?5'd16:ratio_i;
    wire [4:0] inv=5'd16-r;
    wire [16:0] before_value=before_axis*inv+before_diag*r;
    wire [16:0] after_value=after_axis*inv+after_diag*r;
    wire [16:0] center={1'b0,mag_i,4'b0};
    // One-sided strict comparison gives a deterministic plateau representative.
    assign keep_o=(center>before_value) && (center>=after_value);
    wire [17:0] curvature=({1'b0,center}<<1)-{1'b0,before_value}-{1'b0,after_value};
    wire [17:0] difference=(after_value>=before_value)?
        {1'b0,after_value}-{1'b0,before_value}:{1'b0,before_value}-{1'b0,after_value};
    reg [18:0] rem_value;
    reg [3:0] offset;
    integer i;
    always @(*) begin
        rem_value={1'b0,difference};offset=0;
        if(keep_o && curvature!=0) begin
            if(difference>=curvature) offset=8;
            else for(i=0;i<3;i=i+1) begin
                rem_value=rem_value<<1;offset=offset<<1;
                if(rem_value>={1'b0,curvature}) begin
                    rem_value=rem_value-{1'b0,curvature};offset=offset|4'd1;
                end
            end
        end
    end
    assign geometry_o={axis_y_i,same_i,(after_value>before_value),offset,r};
endmodule
