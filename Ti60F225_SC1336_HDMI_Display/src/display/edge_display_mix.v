`timescale 1ns / 1ps
//=====================================================================
// edge_display_mix  —— 接口 I-C
//---------------------------------------------------------------------
// 把原图与边缘图合成为一路 RGB888。
//
//   mode_i = 0  原图
//   mode_i = 1  边缘图
//   mode_i = 2  左半原图 + 右半边缘图
//   mode_i = 3  画中画：背景原图，右下 1/4 内嵌「边缘图缩放至 1/4」
//
// 【为什么叫这个名（2026-10-10 改名）】
//   本模块原名 `image_mix`，文件在 `src/display/image_mix.v`。
//   仓库所有者 lunaryes 在分支 `feature-image_mix` 上**同路径同名**另建了一份
//   `image_mix.v`（101 行，只做显示模块 + 占位接线）。两边如果同名同路径并存，
//   一是 git 合并必然 add/add 冲突，二是综合/仿真会看到**两个同名 module** 直接报错。
//   为把 `src/display/image_mix.v` 这个路径与 `image_mix` 这个名字**让给所有者**，
//   本次把我们的实现改名为 `edge_display_mix`（文件同名）。
//   端口、语义、时序与改名前的 `image_mix` **完全一致**（mode 3 除外，此前未实现）。
//
// 【mode 3 的资源现实与降级（2026-10-10）】
//   `docs/INTERFACE.md` §5 冻结的目标是「边缘图缩放至右下 1/4」，即 640x360 内嵌图。
//   但 640x360 **全彩** 需要 640*360*24 = 5.53 Mbit，超过 Ti60F225 全部 EBR
//   （2.56 Mbit / 256 块），**放不下**。故降级为：
//     · 内嵌图存 **1 bit/像素的二值边缘图**：640x360 bit = 230 Kbit（约 23 个 EBR 块）
//     · 抽取方式：水平 2:1、垂直 2:1（取每个 2x2 块的左上样点），得 640x360
//     · 内容取自 `edge_rgb_i`：`edge_rgb_i == 24'hFFFFFF` 记为白（边缘）、否则黑。
//       在 `EDGE_BINARY=1`（默认）下它**就是**二值边缘标志；
//       在 `EDGE_BINARY=0`（梯度幅值灰度图）下退化为「幅值饱和到 255」的像素。
//     · 内嵌图恒为二值，与 `EDGE_BINARY` 无关（缓冲只有 1 bit）。
//
// 【内嵌图的取样偏移（重要，避坑）】
//   `pix_window_3x3` 在**帧首行 / 行首两列**存在固有偏差（不做边界处理的后果，
//   见 docs/ARCH.md §11.4）。若把抽取结果 1:1 铺进内嵌图，内嵌图的**第一行 / 第一列**
//   会把这个已知偏差搬上屏幕——而这两行/列在整幅图上**不会**被 FrameBoundCrop 裁掉。
//   因此：
//     · 写侧：取样点 = 偶数输入行 & 偶数输入列（row[0]==0 && col[0]==0）；
//       抽取行号 dy = row>>1，抽取列号 dx = col>>1，**不加偏移**。这样每抽取行恰好
//       640 个样点 = 80 整字节，字节打包天然对齐，且不存在读写同拍冲突。
//     · 读侧：显示内嵌位置 (iy, ix) 时，读抽取样点 (min(iy+1,PH-1), min(ix+1,PW-1))。
//       即**跳过抽取后的第 0 行 / 第 0 列**（正是含偏差的那一行/列），末尾钳位补齐。
//       钳位点落在输出最外 1-2 行/列，会被 FrameBoundCrop 裁掉。
//   于是内嵌图**可见部分完全干净**，已由 L2 逐像素验证覆盖
//   （docs/L2_VERIFY.md，MODE=3 可见区 0 不符）。
//   注：垂直方向若不做 +1 偏移，会在 iy = PH-2 处出现同拍读写
//   （写在第 2(iy+1) 行、读在第 PH+iy 行，PH=360 时两者相等）；偏移后该点落到
//   输出第 718 行，已被裁剪，故安全。
//
// 【输入对齐前置条件】
//   orig_rgb_i 与 edge_rgb_i 必须**同拍**。对齐由 image_pipeline_top 用
//   sync_delay_align 保证，本模块不做任何延迟补偿。
//
// 【时序】输出寄存 1 拍，延迟固定为 1（四种模式一致）。
//   vsync_o / de_o 同步寄存，与 rgb_o 同拍。
//   内嵌缓冲的读地址**提前一拍**发出（预测下一拍像素位置），使 BRAM 的 1 拍读延迟
//   正好与输出寄存器对齐，不改变对外延迟。
//
// 【参数化】IMG_W / IMG_H 必须参数化；要求 IMG_W/2 是 8 的倍数（每抽取行整字节）。
//=====================================================================
module edge_display_mix #(
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

    //-----------------------------------------------------------------
    // 常量
    //-----------------------------------------------------------------
    localparam AW        = 11;                       // IMG_W <= 2**AW
    localparam AH        = 11;                       // IMG_H <= 2**AH
    localparam [AW-1:0] COL_ZERO = {AW{1'b0}};
    localparam [AH-1:0] ROW_ZERO = {AH{1'b0}};
    localparam [AW-1:0] COL_HALF = IMG_W >> 1;       // 分屏 / 内嵌界线

    localparam PW  = IMG_W >> 1;                     // 抽取后列数（640）
    localparam PH  = IMG_H >> 1;                     // 抽取后行数（360）
    localparam PS  = PW >> 3;                        // 每抽取行字节数（80）
    localparam PN  = PS * PH;                        // 缓冲总字节数（28800）
    localparam PAW = (PN > 1) ? $clog2(PN) : 1;      // 缓冲地址位宽

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
    // 行计数：每行结束（col 回绕）时 +1；帧首（vsync 上升沿）归零
    //-----------------------------------------------------------------
    reg vsync_i_d;
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) vsync_i_d <= 1'b0;
        else         vsync_i_d <= vsync_i;
    end
    wire vsync_rise = vsync_i & ~vsync_i_d;

    reg [AH-1:0] row;
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni)
            row <= ROW_ZERO;
        else if (vsync_rise)
            row <= ROW_ZERO;
        else if (de_i && (col == (IMG_W - 1)))
            row <= (row == (IMG_H - 1)) ? ROW_ZERO : (row + 1'b1);
    end

    //-----------------------------------------------------------------
    // 画中画缓冲：2:1 抽取的二值边缘图，1 bit/像素，按字节打包
    //-----------------------------------------------------------------
    reg [7:0] pip_mem [0:PN-1];

    // 输入像素判为「边缘」（= 二值边缘图的白点）
    wire edge_bin = (edge_rgb_i == 24'hFFFFFF);

    // 取样点：偶数行 & 偶数列
    wire pip_sample = de_i && (row[0] == 1'b0) && (col[0] == 1'b0);

    reg [7:0]     pip_acc;                           // 8 个样点的移位累加
    reg [2:0]     pip_cnt;                           // 样点在字节内的位号 0..7
    reg [PAW-1:0] pip_wa;
    reg [7:0]     pip_wd;
    reg           pip_we;

    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            pip_acc <= 8'd0;
            pip_cnt <= 3'd0;
            pip_wa  <= {PAW{1'b0}};
            pip_wd  <= 8'd0;
            pip_we  <= 1'b0;
        end else begin
            pip_we <= 1'b0;
            if (pip_sample) begin
                if (pip_cnt == 3'd7) begin
                    // 满 8 个样点 -> 整字节落盘（天然对齐，无需读改写）
                    // 位序：**第 1 个样点在 bit0**，第 8 个在 bit7，与读侧的
                    // `pip_q[dx & 7]` 一致。右移拼接才能得到这个位序，
                    // 左移拼接会得到 MSB 优先、整体反位（实测踩过）。
                    pip_we <= 1'b1;
                    pip_wd <= {edge_bin, pip_acc[7:1]};
                    pip_wa <= (row[AH-1:1] * PS) + col[AW-1:4];
                    pip_cnt <= 3'd0;
                    pip_acc <= 8'd0;
                end else begin
                    pip_acc <= {edge_bin, pip_acc[7:1]};
                    pip_cnt <= pip_cnt + 3'd1;
                end
            end
        end
    end

    always @(posedge clk_i) begin
        if (pip_we) pip_mem[pip_wa] <= pip_wd;
    end

    //-----------------------------------------------------------------
    // 内嵌图读取：地址提前一拍发出，与 BRAM 的 1 拍读延迟对齐
    //   预测"下一拍输入像素"的位置
    //-----------------------------------------------------------------
    wire [AW-1:0] nxt_col = de_i ? ((col == (IMG_W - 1)) ? COL_ZERO : (col + 1'b1))
                                 : COL_ZERO;
    wire [AH-1:0] nxt_row = de_i ? ((col == (IMG_W - 1))
                                        ? ((row == (IMG_H - 1)) ? ROW_ZERO : (row + 1'b1))
                                        : row)
                                 : row;

    wire          nxt_inset = (nxt_row >= (IMG_H >> 1)) && (nxt_col >= (IMG_W >> 1));
    wire [AH-1:0] nxt_iy    = nxt_row - (IMG_H >> 1);      // 0 .. PH-1
    wire [AW-1:0] nxt_ix    = nxt_col - (IMG_W >> 1);      // 0 .. PW-1
    // 跳过抽取后的第 0 行/第 0 列（含首行/首列固有偏差），末尾钳位
    wire [AH-1:0] rd_dy = (nxt_iy >= (PH - 1)) ? (PH - 1) : (nxt_iy + 1'b1);
    wire [AW-1:0] rd_dx = (nxt_ix >= (PW - 1)) ? (PW - 1) : (nxt_ix + 1'b1);

    wire [PAW-1:0] rd_addr_n = (rd_dy * PS) + rd_dx[AW-1:3];

    reg [2:0]     rd_bit_r;
    reg           rd_val_r;

    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            rd_bit_r  <= 3'd0;
            rd_val_r  <= 1'b0;
        end else begin
            rd_bit_r  <= rd_dx[2:0];
            rd_val_r  <= nxt_inset;
        end
    end

    // BRAM 自身有 1 拍读延迟，地址**直接用组合的预测地址**（比当前像素超前 2 拍），
    // 于是 pip_q 在下一拍拿到的正好是"当前像素"那个取样点。
    // 注意：这里不能再插一级地址寄存器，否则整体错位一个取样点（实测踩过）。
    reg [7:0] pip_q;
    always @(posedge clk_i) pip_q <= pip_mem[rd_addr_n];

    wire        pip_edge = pip_q[rd_bit_r];
    wire [23:0] pip_rgb  = pip_edge ? 24'hFFFFFF : 24'h000000;

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
                    2'd3    : rgb_o <= rd_val_r ? pip_rgb : orig_rgb_i;       // 画中画
                    default : rgb_o <= edge_rgb_i;
                endcase
        end
    end

endmodule
