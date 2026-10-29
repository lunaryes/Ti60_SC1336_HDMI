`timescale 1ns / 1ps
//=====================================================================
// otsu_thr_gen —— Canny 自适应阈值（Otsu 类间方差最大法，纯硬件）
// 参考：Guo & Wu, "FPGA implementation of a real-time edge detection
//       system based on an improved Canny algorithm"（自适应阈值部分）
//---------------------------------------------------------------------
// 【方法论：论文方法与本工程的差异分析——为什么不是照抄】
//
// 论文流水线：NMS 输出 → 外部 SDRAM 整帧缓存 + 256 桶直方图 → Otsu
// （对数域消除法）→ 阈值 → 从缓存读回做二值化 + 滞后。
// 本工程改造点（每一条都是分析后的取舍，不是实现细节差异）：
//
// 1) 【不要整帧缓存】论文必须缓存整帧，因为它的二值化在同一帧内等阈值。
//    本工程 canny_edge 的 thr_high_i/thr_low_i 是电平判据，允许"本帧统计、
//    下帧生效"——阈值只在垂直消隐期更新，帧内无接缝。省掉整帧 SDRAM，
//    本模块只多占 1 块 256×20 直方图 RAM（约 1 块 EFX_RAM10）。
//
// 2) 【统计源复用调试口】论文直方图统计 NMS 输出（非极大值置 0）。
//    canny_edge 已有 mag8_o（阈值域 mag8=mag>>3，未饱和）与 cls_o 判决
//    分类口，等价重建论文口径：cls==01 = NMS 未过 → 记 0；其余记 mag8。
//    cls==00（mag8<thr_low）论文里应再分过/未过 NMS，但该段幅值极小、
//    对峰谷位置影响可忽略，统一按原值记入（工程近似）。
//
// 3) 【对数域消除法，实现降级】量纲分析：N≤921600(20b)、S=Σbin≤2.35e8
//    (28b)、d(k)=S·c(k)−N·s(k) 达 48 位——直接比较类间方差需要
//    d²·D ≈ 136 位乘法，与论文结论一致必须走对数域。但论文的通用对数
//    运算单元在此过重：本模块只做 argmax 比较、不做方差数值运算，用
//    "MSB 优先编码 + 8bit 尾数查表 + 8bit 线性插值"的专用 log2（Q6.8）。
//    ★ 调试记录：最初直接取尾数高位当小数（线性近似 log2(1+x)≈x），
//      最大误差 11%，argmax 偏移达 16 bin——查表插值版经
//      sim/check_otsu_math.py 300 组真实感分布验证，选阈的类间方差
//      最差保留最优值 99.36%（95 分位≈100%）。另用增量式
//        d(k) = d(k−1) + cnt(k)·(S − N·k)
//      把每 bin 两次大乘法（S·c 与 N·s）合并为一次 21×29 有符号乘法。
//
// 4) 【论文没有的闭环保护】阈值直连电机跟转闭环，必须防抖：
//    a. ±THR_DEADBAND 滞回：变化≤2 不更新，防逐帧跳变传导成电机抖动；
//    b. 钳位 [THR_MIN,THR_MAX]：canny_edge.v 头注释标定 mag8 有效量程
//       0..96（3×3 高斯把 4A 阶跃压成 3A），thr_high>48≈关掉边缘检测，
//       Otsu 在极端分布（全黑/全白）下可能输出 0 或爆表，必须钳住；
//    c. 帧尾先沉降 4 拍（等统计流水尾部 3 拍写完）再开扫描；
//    d. 扫描中 de 提前到来（消隐异常窄）→ 中止扫描、快速清零剩余 bin、
//       采用当前最优——统计完整性优先于阈值计算。
//
// 【时序预算】720p 垂直消隐 ≈ 45 行 × 1650 拍 ≈ 74k 拍 @74.25MHz；
// 扫描 256 bin × 6 拍 ≈ 1536 拍，余量 50 倍以上。
//
// 【阈值生效相位】本帧统计 → 消隐期算出 → 下帧整帧使用。对 160×90
// 识别网格 + CONFIRM=3 的下游完全无感。
//=====================================================================
module otsu_thr_gen #(
    parameter integer IMG_W        = 1280,
    parameter [7:0]   THR_MIN      = 8'd4,    // 钳位下限（低于此必是噪声）
    parameter [7:0]   THR_MAX      = 8'd48,   // 钳位上限（canny 标定：>48≈关检测）
    parameter [7:0]   THR_DFLT     = 8'd12,   // 上电/无有效解时的保持值
    parameter integer THR_DEADBAND = 2        // 滞回带宽（±，防逐帧跳变）
)(
    input  wire       clk,
    input  wire       rst_n,

    // 统计源：canny_edge 输出域（can_vs/can_de/can_mag8/can_cls 同拍）
    input  wire       vsync_i,    // 帧有效电平：高=有效帧期间
    input  wire       de_i,       // 像素有效
    input  wire [7:0] mag8_i,     // 阈值域幅值（未饱和）
    input  wire [1:0] cls_i,      // 判决分类（01=NMS 未过）

    output reg  [7:0] thr_high_o, // 自适应强边阈值（已钳位+滞回）
    output reg  [7:0] thr_raw_o   // 钳位前的 Otsu 原始 bin（观察/标定用）
);

    //-----------------------------------------------------------------
    // 1) 帧统计：256 桶直方图 + 总数 N + 总和 S
    //    直方图读改写冲突（连续同 bin 像素）用"同步读 + 回放寄存器"：
    //    写延迟 3 拍，与像素 i 的 RAM 读同沿发生的是像素 i-2 的写——
    //    读侧拿到旧值时用回放值（上一次写的终值）顶上。回放寄存器
    //    一组即可：更早的写都已落在读沿之前（与 gray_hist_uart 同一套
    //    已验证手法）。
    //-----------------------------------------------------------------
    reg [19:0] hist [0:255];
    reg [7:0]  s0_bin;   reg        s0_v;   // 级0：锁存 bin
    reg [7:0]  s1_bin;   reg        s1_v;   // 级1：RAM 读地址期
    reg [7:0]  s2_bin;   reg        s2_v;   // 级2：写回期
    reg [19:0] s2_q;
    reg [19:0] ram_q;
    reg [7:0]  rp_addr;  reg [19:0] rp_val; reg rp_v;

    // bin 生成：NMS 未过（cls==01）按论文口径记 0
    wire [7:0] bin_w = (cls_i == 2'b01) ? 8'd0 : mag8_i;

    reg [19:0] n_cnt;    // 本帧 de 像素总数（≤921600，20b 不饱和）
    reg [27:0] s_cnt;    // 本帧 bin 值总和（≤255×921600<2^28）

    //-----------------------------------------------------------------
    // 2) 消隐期 Otsu 扫描（argmax 类间方差，对数域比较）
    //    标准定义 σ²B = w0·w1·(μ0−μ1)²，代入 w0=c/N、μ0=s/c 展开得
    //    σ²(k) = (S·c − N·s)² / (N²·c·(N−c))   （N² 为公共常数）
    //    log 得分 = 2·log2|d| − log2(c) − log2(N−c)，Q6.8 定点。
    //    （推导勘误记录：分母 c 的幂次曾误写为 3 次，已按标准定义
    //    修正为 1 次，并由 sim/check_otsu_math.py 对 200+ 随机分布
    //    逐案例验证与直接浮点 Otsu 一致。）
    //-----------------------------------------------------------------
    localparam [3:0] S_IDLE  = 4'd0, S_SETTLE = 4'd1, S_READ = 4'd2,
                     S_WAIT  = 4'd3, S_CALC   = 4'd4, S_UPD  = 4'd5,
                     S_LOGB  = 4'd6, S_CMP    = 4'd7, S_CLRFAST = 4'd8;
    // "应用阈值"用一拍脉冲 apply_pulse 完成，不占状态编码。

    reg [3:0]  st;
    reg        apply_pulse;   // 一拍：应用 best_bin 到输出寄存器
    reg [7:0]  sc_k;
    reg [7:0]  ra;            // 扫描读地址
    reg [1:0]  settle_cnt;
    reg [19:0] cnt_r;
    reg [19:0] c_acc;
    reg signed [49:0] d_acc;
    reg [13:0] lg_d, lg_c, lg_nc;      // Q6.8
    reg signed [17:0] best_score;
    reg [7:0]  best_bin;
    reg [19:0] N_lat;
    reg [27:0] S_lat;

    reg vs_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) vs_d <= 1'b0;
        else        vs_d <= vsync_i;
    end

    // 统计/扫描统一的 RAM 读地址：统计流水期用 s1_bin，扫描期用 ra
    wire [7:0] rd_addr = (st == S_SETTLE || st == S_READ || st == S_WAIT ||
                          st == S_CALC || st == S_UPD  || st == S_LOGB ||
                          st == S_CMP  || st == S_CLRFAST) ? ra : s1_bin;

    // 回放值终值（组合）：同地址连击时用回放值顶上旧读数
    wire [19:0] s2_base = (rp_v && (rp_addr == s2_bin)) ? rp_val : s2_q;
    wire [19:0] s2_new  = s2_base + 1'b1;

    // RAM：单一 always 块，写口两路（统计写回 / 扫描读后即清），消隐期
    // 二者天然互斥；万一撞车（帧尾最后像素 vs bin0 清零）后写者胜，
    // 误差 ≤3 个计数，对 Otsu 峰谷位置无影响。
    always @(posedge clk) begin
        ram_q <= hist[rd_addr];
        if (s2_v)
            hist[s2_bin] <= s2_new;
        if (st == S_CALC)
            hist[sc_k] <= 20'd0;      // 读后即清，省独立清零阶段
        if (st == S_CLRFAST)
            hist[sc_k] <= 20'd0;      // 中止后的快速清零路径
    end

    //-----------------------------------------------------------------
    // 专用 log2：MSB 优先编码定整数部分（6b），尾数 8b 查表 + 8b 线性
    // 插值（Q6.8）。★ 为什么必须查表而非直接取尾数位当小数：log2(1+x)
    // 对 x 的线性近似最大误差 11%（x=0.5 处），曾导致 argmax 偏移 16 bin；
    // 查表+插值后经 sim/check_otsu_math.py 300 组真实感分布验证，
    // 选取阈值的类间方差最差保留最优值的 99.36%（95 分位≈100%）。
    //-----------------------------------------------------------------
    function [7:0] lut8;                 // round(log2(1+j/256)*256)
        input [7:0] j;
        begin
            case (j)
                8'd0: lut8 = 8'd0;  8'd1: lut8 = 8'd1;  8'd2: lut8 = 8'd3;  8'd3: lut8 = 8'd4;
                8'd4: lut8 = 8'd6;  8'd5: lut8 = 8'd7;  8'd6: lut8 = 8'd9;  8'd7: lut8 = 8'd10;
                8'd8: lut8 = 8'd11;  8'd9: lut8 = 8'd13;  8'd10: lut8 = 8'd14;  8'd11: lut8 = 8'd16;
                8'd12: lut8 = 8'd17;  8'd13: lut8 = 8'd18;  8'd14: lut8 = 8'd20;  8'd15: lut8 = 8'd21;
                8'd16: lut8 = 8'd22;  8'd17: lut8 = 8'd24;  8'd18: lut8 = 8'd25;  8'd19: lut8 = 8'd26;
                8'd20: lut8 = 8'd28;  8'd21: lut8 = 8'd29;  8'd22: lut8 = 8'd30;  8'd23: lut8 = 8'd32;
                8'd24: lut8 = 8'd33;  8'd25: lut8 = 8'd34;  8'd26: lut8 = 8'd36;  8'd27: lut8 = 8'd37;
                8'd28: lut8 = 8'd38;  8'd29: lut8 = 8'd40;  8'd30: lut8 = 8'd41;  8'd31: lut8 = 8'd42;
                8'd32: lut8 = 8'd44;  8'd33: lut8 = 8'd45;  8'd34: lut8 = 8'd46;  8'd35: lut8 = 8'd47;
                8'd36: lut8 = 8'd49;  8'd37: lut8 = 8'd50;  8'd38: lut8 = 8'd51;  8'd39: lut8 = 8'd52;
                8'd40: lut8 = 8'd54;  8'd41: lut8 = 8'd55;  8'd42: lut8 = 8'd56;  8'd43: lut8 = 8'd57;
                8'd44: lut8 = 8'd59;  8'd45: lut8 = 8'd60;  8'd46: lut8 = 8'd61;  8'd47: lut8 = 8'd62;
                8'd48: lut8 = 8'd63;  8'd49: lut8 = 8'd65;  8'd50: lut8 = 8'd66;  8'd51: lut8 = 8'd67;
                8'd52: lut8 = 8'd68;  8'd53: lut8 = 8'd69;  8'd54: lut8 = 8'd71;  8'd55: lut8 = 8'd72;
                8'd56: lut8 = 8'd73;  8'd57: lut8 = 8'd74;  8'd58: lut8 = 8'd75;  8'd59: lut8 = 8'd77;
                8'd60: lut8 = 8'd78;  8'd61: lut8 = 8'd79;  8'd62: lut8 = 8'd80;  8'd63: lut8 = 8'd81;
                8'd64: lut8 = 8'd82;  8'd65: lut8 = 8'd84;  8'd66: lut8 = 8'd85;  8'd67: lut8 = 8'd86;
                8'd68: lut8 = 8'd87;  8'd69: lut8 = 8'd88;  8'd70: lut8 = 8'd89;  8'd71: lut8 = 8'd90;
                8'd72: lut8 = 8'd92;  8'd73: lut8 = 8'd93;  8'd74: lut8 = 8'd94;  8'd75: lut8 = 8'd95;
                8'd76: lut8 = 8'd96;  8'd77: lut8 = 8'd97;  8'd78: lut8 = 8'd98;  8'd79: lut8 = 8'd99;
                8'd80: lut8 = 8'd100;  8'd81: lut8 = 8'd102;  8'd82: lut8 = 8'd103;  8'd83: lut8 = 8'd104;
                8'd84: lut8 = 8'd105;  8'd85: lut8 = 8'd106;  8'd86: lut8 = 8'd107;  8'd87: lut8 = 8'd108;
                8'd88: lut8 = 8'd109;  8'd89: lut8 = 8'd110;  8'd90: lut8 = 8'd111;  8'd91: lut8 = 8'd112;
                8'd92: lut8 = 8'd113;  8'd93: lut8 = 8'd114;  8'd94: lut8 = 8'd116;  8'd95: lut8 = 8'd117;
                8'd96: lut8 = 8'd118;  8'd97: lut8 = 8'd119;  8'd98: lut8 = 8'd120;  8'd99: lut8 = 8'd121;
                8'd100: lut8 = 8'd122;  8'd101: lut8 = 8'd123;  8'd102: lut8 = 8'd124;  8'd103: lut8 = 8'd125;
                8'd104: lut8 = 8'd126;  8'd105: lut8 = 8'd127;  8'd106: lut8 = 8'd128;  8'd107: lut8 = 8'd129;
                8'd108: lut8 = 8'd130;  8'd109: lut8 = 8'd131;  8'd110: lut8 = 8'd132;  8'd111: lut8 = 8'd133;
                8'd112: lut8 = 8'd134;  8'd113: lut8 = 8'd135;  8'd114: lut8 = 8'd136;  8'd115: lut8 = 8'd137;
                8'd116: lut8 = 8'd138;  8'd117: lut8 = 8'd139;  8'd118: lut8 = 8'd140;  8'd119: lut8 = 8'd141;
                8'd120: lut8 = 8'd142;  8'd121: lut8 = 8'd143;  8'd122: lut8 = 8'd144;  8'd123: lut8 = 8'd145;
                8'd124: lut8 = 8'd146;  8'd125: lut8 = 8'd147;  8'd126: lut8 = 8'd148;  8'd127: lut8 = 8'd149;
                8'd128: lut8 = 8'd150;  8'd129: lut8 = 8'd151;  8'd130: lut8 = 8'd152;  8'd131: lut8 = 8'd153;
                8'd132: lut8 = 8'd154;  8'd133: lut8 = 8'd155;  8'd134: lut8 = 8'd155;  8'd135: lut8 = 8'd156;
                8'd136: lut8 = 8'd157;  8'd137: lut8 = 8'd158;  8'd138: lut8 = 8'd159;  8'd139: lut8 = 8'd160;
                8'd140: lut8 = 8'd161;  8'd141: lut8 = 8'd162;  8'd142: lut8 = 8'd163;  8'd143: lut8 = 8'd164;
                8'd144: lut8 = 8'd165;  8'd145: lut8 = 8'd166;  8'd146: lut8 = 8'd167;  8'd147: lut8 = 8'd168;
                8'd148: lut8 = 8'd169;  8'd149: lut8 = 8'd169;  8'd150: lut8 = 8'd170;  8'd151: lut8 = 8'd171;
                8'd152: lut8 = 8'd172;  8'd153: lut8 = 8'd173;  8'd154: lut8 = 8'd174;  8'd155: lut8 = 8'd175;
                8'd156: lut8 = 8'd176;  8'd157: lut8 = 8'd177;  8'd158: lut8 = 8'd178;  8'd159: lut8 = 8'd178;
                8'd160: lut8 = 8'd179;  8'd161: lut8 = 8'd180;  8'd162: lut8 = 8'd181;  8'd163: lut8 = 8'd182;
                8'd164: lut8 = 8'd183;  8'd165: lut8 = 8'd184;  8'd166: lut8 = 8'd185;  8'd167: lut8 = 8'd185;
                8'd168: lut8 = 8'd186;  8'd169: lut8 = 8'd187;  8'd170: lut8 = 8'd188;  8'd171: lut8 = 8'd189;
                8'd172: lut8 = 8'd190;  8'd173: lut8 = 8'd191;  8'd174: lut8 = 8'd192;  8'd175: lut8 = 8'd192;
                8'd176: lut8 = 8'd193;  8'd177: lut8 = 8'd194;  8'd178: lut8 = 8'd195;  8'd179: lut8 = 8'd196;
                8'd180: lut8 = 8'd197;  8'd181: lut8 = 8'd198;  8'd182: lut8 = 8'd198;  8'd183: lut8 = 8'd199;
                8'd184: lut8 = 8'd200;  8'd185: lut8 = 8'd201;  8'd186: lut8 = 8'd202;  8'd187: lut8 = 8'd203;
                8'd188: lut8 = 8'd203;  8'd189: lut8 = 8'd204;  8'd190: lut8 = 8'd205;  8'd191: lut8 = 8'd206;
                8'd192: lut8 = 8'd207;  8'd193: lut8 = 8'd208;  8'd194: lut8 = 8'd208;  8'd195: lut8 = 8'd209;
                8'd196: lut8 = 8'd210;  8'd197: lut8 = 8'd211;  8'd198: lut8 = 8'd212;  8'd199: lut8 = 8'd212;
                8'd200: lut8 = 8'd213;  8'd201: lut8 = 8'd214;  8'd202: lut8 = 8'd215;  8'd203: lut8 = 8'd216;
                8'd204: lut8 = 8'd216;  8'd205: lut8 = 8'd217;  8'd206: lut8 = 8'd218;  8'd207: lut8 = 8'd219;
                8'd208: lut8 = 8'd220;  8'd209: lut8 = 8'd220;  8'd210: lut8 = 8'd221;  8'd211: lut8 = 8'd222;
                8'd212: lut8 = 8'd223;  8'd213: lut8 = 8'd224;  8'd214: lut8 = 8'd224;  8'd215: lut8 = 8'd225;
                8'd216: lut8 = 8'd226;  8'd217: lut8 = 8'd227;  8'd218: lut8 = 8'd228;  8'd219: lut8 = 8'd228;
                8'd220: lut8 = 8'd229;  8'd221: lut8 = 8'd230;  8'd222: lut8 = 8'd231;  8'd223: lut8 = 8'd231;
                8'd224: lut8 = 8'd232;  8'd225: lut8 = 8'd233;  8'd226: lut8 = 8'd234;  8'd227: lut8 = 8'd234;
                8'd228: lut8 = 8'd235;  8'd229: lut8 = 8'd236;  8'd230: lut8 = 8'd237;  8'd231: lut8 = 8'd238;
                8'd232: lut8 = 8'd238;  8'd233: lut8 = 8'd239;  8'd234: lut8 = 8'd240;  8'd235: lut8 = 8'd241;
                8'd236: lut8 = 8'd241;  8'd237: lut8 = 8'd242;  8'd238: lut8 = 8'd243;  8'd239: lut8 = 8'd244;
                8'd240: lut8 = 8'd244;  8'd241: lut8 = 8'd245;  8'd242: lut8 = 8'd246;  8'd243: lut8 = 8'd247;
                8'd244: lut8 = 8'd247;  8'd245: lut8 = 8'd248;  8'd246: lut8 = 8'd249;  8'd247: lut8 = 8'd249;
                8'd248: lut8 = 8'd250;  8'd249: lut8 = 8'd251;  8'd250: lut8 = 8'd252;  8'd251: lut8 = 8'd252;
                8'd252: lut8 = 8'd253;  8'd253: lut8 = 8'd254;  8'd254: lut8 = 8'd255;  default: lut8 = 8'd255;
            endcase
        end
    endfunction

    function [13:0] log2_q68;            // {e[5:0], frac[7:0]}，Q6.8
        input [47:0] v;
        reg   [47:0] sh;
        reg   [7:0]  m, r;
        reg   [5:0]  e;
        reg   [7:0]  f0, f1;
        reg   [8:0]  fd;
        reg   [15:0] fsum;
        reg          found;
        integer      i;
        begin
            if (v == 48'd0) begin
                log2_q68 = 14'd0;
            end else begin
                e = 6'd0; found = 1'b0;
                for (i = 47; i >= 0; i = i - 1) begin
                    if (v[i] && !found) begin
                        e = i[5:0];
                        found = 1'b1;
                    end
                end
                sh = v << (6'd47 - e);      // MSB 归一到 bit47
                m  = sh[46:39];             // 尾数高 8 位：查表索引
                r  = sh[38:31];             // 尾数次 8 位：插值余数
                f0 = lut8(m);
                f1 = (m == 8'd255) ? 8'd255 : lut8(m + 8'd1);
                fd = {1'b0, f1} - {1'b0, f0};           // 表间距（0..2，非负）
                fsum = {1'b0, f0} + ((fd * {1'b0, r}) >> 8);
                // fsum ≤ 255（m=255 处钳位 underestimate <1/256 octave）
                log2_q68 = {e, fsum[7:0]};
            end
        end
    endfunction

    // d 增量：cnt(k) × (S − N·k)，一次 21×29 有符号乘法
    // ★ 时序拆分：乘加在 S_UPD 拍完成并寄存，log2（优先编码+桶形移位+
    //   双查表+插值乘）在 S_LOGB 拍对已寄存的 d_acc/c_acc 做——两段各自
    //   独立满足 74.25MHz（首版串在一拍出现 clk_pixel setup −0.101ns）。
    wire signed [28:0] dk_w = $signed({1'b0, S_lat})
                            - $signed({1'b0, N_lat}) * $signed({1'b0, sc_k});
    wire signed [49:0] prod_w = $signed({1'b0, cnt_r}) * dk_w;
    wire signed [49:0] d_new  = d_acc + prod_w;
    wire [47:0] d_abs  = (d_acc < 0) ? (~d_acc[47:0] + 1'b1) : d_acc[47:0];
    wire [19:0] nc_new = N_lat - c_acc;

    // log 结果（组合，S_LOGB 拍寄存）
    wire [13:0] lg_d_w  = log2_q68(d_abs);
    wire [13:0] lg_c_w  = log2_q68({28'd0, c_acc});
    wire [13:0] lg_nc_w = log2_q68({28'd0, nc_new});

    wire [14:0] lg_d_x2 = {lg_d, 1'b0};                 // 2·lg_d ≤ 24574
    wire signed [17:0] score_w = $signed({3'b0, lg_d_x2})
                               - $signed({4'b0, lg_c})
                               - $signed({4'b0, lg_nc});
    wire cand_ok = (N_lat > 20'd0) && (c_acc > 20'd0) && (c_acc < N_lat)
                   && (d_acc != 50'd0);

    // 钳位 + 滞回
    wire [7:0] t_cl = (best_bin < THR_MIN) ? THR_MIN :
                      (best_bin > THR_MAX) ? THR_MAX : best_bin;
    wire [7:0] t_diff = (t_cl > thr_high_o) ? (t_cl - thr_high_o)
                                            : (thr_high_o - t_cl);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st         <= S_IDLE;
            apply_pulse<= 1'b0;
            sc_k       <= 8'd0;
            ra         <= 8'd0;
            settle_cnt <= 2'd0;
            cnt_r      <= 20'd0;
            c_acc      <= 20'd0;
            d_acc      <= 50'd0;
            best_score <= 18'sh20000;
            best_bin   <= THR_DFLT;
            N_lat      <= 20'd0;
            S_lat      <= 28'd0;
            thr_high_o <= THR_DFLT;
            thr_raw_o  <= THR_DFLT;
            lg_d <= 14'd0; lg_c <= 14'd0; lg_nc <= 14'd0;
            s0_bin <= 8'd0; s0_v <= 1'b0;
            s1_bin <= 8'd0; s1_v <= 1'b0;
            s2_bin <= 8'd0; s2_v <= 1'b0;
            s2_q   <= 20'd0;
            rp_addr<= 8'd0; rp_val <= 20'd0; rp_v <= 1'b0;
            n_cnt  <= 20'd0;
            s_cnt  <= 28'd0;
        end else begin
            apply_pulse <= 1'b0;

            //------------------------------------------------统计流水
            // 帧首清累计（帧有效电平上升沿 = 新帧开始）
            if (vsync_i && !vs_d) begin
                n_cnt <= 20'd0;
                s_cnt <= 28'd0;
            end else if (de_i) begin
                n_cnt <= n_cnt + 1'b1;
                s_cnt <= s_cnt + {20'd0, bin_w};
            end

            s0_bin <= bin_w;   s0_v <= de_i;
            s1_bin <= s0_bin;  s1_v <= s0_v;
            s2_bin <= s1_bin;  s2_v <= s1_v;
            s2_q   <= ram_q;
            rp_addr<= s2_bin;
            rp_val <= s2_new;
            rp_v   <= s2_v;

            //------------------------------------------------帧尾触发
            // 帧有效电平下降沿 = 本帧结束：锁存 N/S，沉降后开扫
            if (vs_d && !vsync_i && (st == S_IDLE)) begin
                N_lat      <= n_cnt;
                S_lat      <= s_cnt;
                st         <= S_SETTLE;
                settle_cnt <= 2'd0;
                sc_k       <= 8'd0;
                c_acc      <= 20'd0;
                d_acc      <= 50'd0;
                best_score <= 18'sh20000;
                best_bin   <= THR_DFLT;
            end

            //------------------------------------------------扫描 FSM
            case (st)
                S_SETTLE: begin          // 等统计流水尾部 3 拍写完
                    settle_cnt <= settle_cnt + 1'b1;
                    if (&settle_cnt) begin
                        st <= S_READ;
                        ra <= sc_k;
                    end
                end
                S_READ:  st <= S_WAIT;   // ra 已指向 sc_k
                S_WAIT:  st <= S_CALC;   // 本拍 RAM 采样 hist[ra]
                S_CALC: begin            // ram_q = hist[sc_k] 可用
                    cnt_r <= ram_q;
                    st    <= S_UPD;      // 清零写由 RAM 块按 st==S_CALC 完成
                end
                S_UPD: begin             // 增量乘加 → 寄存（短路径）
                    d_acc <= d_new;
                    c_acc <= c_acc + cnt_r;
                    st    <= S_LOGB;
                end
                S_LOGB: begin            // log2 查表插值（长路径，独立一拍）
                    lg_d  <= lg_d_w;
                    lg_c  <= lg_c_w;
                    lg_nc <= lg_nc_w;
                    st    <= S_CMP;
                end
                S_CMP: begin
                    if (cand_ok && (score_w > best_score)) begin
                        best_score <= score_w;
                        best_bin   <= sc_k;
                    end
                    if (sc_k == 8'd255) begin
                        apply_pulse <= 1'b1;
                        st          <= S_IDLE;
                    end else begin
                        sc_k <= sc_k + 1'b1;
                        ra   <= sc_k + 1'b1;
                        st   <= S_READ;
                    end
                end
                default: st <= S_IDLE;
            endcase

            // 保护：扫描中 de 提前到来（消隐异常窄）→ 快清剩余 bin 再收工
            if ((st == S_READ || st == S_WAIT || st == S_CALC ||
                 st == S_UPD  || st == S_LOGB || st == S_CMP) && de_i) begin
                st <= S_CLRFAST;
            end
            if (st == S_CLRFAST) begin
                if (sc_k == 8'd255) begin
                    apply_pulse <= 1'b1;
                    st          <= S_IDLE;
                end else begin
                    sc_k <= sc_k + 1'b1;
                end
            end

            //------------------------------------------------应用阈值
            if (apply_pulse) begin
                thr_raw_o <= best_bin;
                if (N_lat != 20'd0) begin
                    // 滞回：变化 ≤ DEADBAND 时保持旧值（防闭环抖动）
                    if (t_diff > THR_DEADBAND[7:0]) begin
                        thr_high_o <= t_cl;
                    end
                end
            end
        end
    end

    // thr_low 由使用方按 thr_high>>1 生成（与手动路径同一口径）

endmodule
