`timescale 1ns / 1ps
//=====================================================================
// uart_cmd —— 极简串口命令接口（M5，排查 Canny 阈值标定问题时加的）
//---------------------------------------------------------------------
// 用途：把 image_pipeline_top 的 thr_i / mode_i / view_i 从"编译期 localparam"
//   变成"运行时可调"。调阈值、切调试视图不用重新综合（一次综合约 10 分钟），
//   现场就能用二分法扫出每个色块/每条边的实际 mag8。
//
//   ★ 2026-10-16：**阈值这一路已改由按键控制**。example_top 不再把 thr_o 接进
//     流水线，改接 src/control/canny_threshold_key_ctrl.v（板上 K1/K2 现场调，
//     不用插串口）。本模块的 'T' 命令与 'R' 回读里那个 thr 字节仍然按原协议
//     工作，但**不再是生效阈值**。view / mode / AWB 三路不受影响。
//     要让串口重新管阈值：把 thr_o 接到按键模块的 uart_thr_i，再给它一个
//     "'T' 写入" 的单拍脉冲（该模块已带这条通路，且串口写优先于按键）。
//
// 硬件前提：example_top 的 uart_rx_i / uart_tx_o 已经声明，且在 Ti60_Demo.peri.xml
//   里分配了引脚（GPIOT_N_16 / GPIOT_P_16），但全工程此前从未使用过。本模块接上
//   它们。板上需要 USB-UART 桥（TTL 侧）。
//
// 时钟：跑在 clk_pixel（74.25 MHz），与 image_pipeline_top **同域，不需要 CDC**。
//   BAUD / CLK_FREQ 都可参数化；460800 @ 74.25MHz 分频 = 161（误差 -0.08%）。
//   若将来 clk_pixel 被门控不可用，改用 clk_24m 常开晶振 + 2FF 同步，那时
//   thr_o / mode_o / view_o 要各自过一遍同步器。
//
//---------------------------------------------------------------------
// 命令（2 字节帧：命令字符 + 1 字节参数）
//   'T' <byte>   thr_high = byte（thr_low 自动 = byte>>1，与原来的 2:1 映射一致）
//                ★ 见上面的 ★：这条路现在不接流水线了，'T' 改不动画面上的阈值
//   'V' <byte>   view = byte[1:0]  0 边缘图 / 1 mag8 图 / 2 假彩图 / 3 Y 图
//   'M' <byte>   mode = byte[1:0]  0 原图 / 1 边缘图 / 2、3 分屏
//   'A' <byte>   AWB **R 通道增益** = byte/64（量程 0..3.98）。上板实测"白墙在室内
//                白灯下发青" = R 偏低（2026-10-15），这个旋钮用来现场把它调回中性。
//                复位值 **112 = ×1.75**：上板对着白墙二分出来的中性点（×1.59 偏青、
//                ×2.0 偏暖）；改动前写死的是 409/256 ≈ ×1.59
//   'B' <byte>   AWB **B 通道增益** = byte/64。复位值 96 = ×1.5 = 原来的 384/256
//   'R'          回读：回发 6 字节 {thr, view, mode, gain_r, gain_b, '\n'}
//                （第 1 个字节是上面那个寄存器，按键调过之后 ≠ 生效阈值）
//   其他字节一律忽略 —— 回车、换行、空格可以随便发，不会误触发。
//
//   例：把阈值调到 24 并切到假彩图 —— 发 'T' 0x18 'V' 0x02
//       回读当前状态 —— 发 'R'，应收到 18 02 03 70 60 0A（70/60 = 112/96 = 复位默认）
//       R 增益调到 ×2.5 —— 发 'A' 0xA0（160/64 = 2.5），再用 'R' 确认第 4 个字节是 A0
//
//---------------------------------------------------------------------
// 两处刻意的省略（都写在注释里，免得后人以为是漏了）：
//   ★ 没有 'L'（单独设 thr_low）：thr_low 当前只影响"强边集外扩 1 像素"那点
//     边界（见 canny_edge.v 的滞后说明），效果微乎其微。等滞回按 §3.7 重做成
//     真传播之后再加，免得为一个端口动已经冻结的 I-A。
//   ★ 参数帧没有超时保护：连发 'T' 之后隔很久再发的字节会被当成参数。调试口，
//     可接受；要换命令直接发新命令字符即可。
//
// 复位值 = THR_DEF / VIEW_DEF / MODE_DEF（由 example_top 传入），上电即工作，
//   不插串口也能正常显示。
//=====================================================================
module uart_cmd #(
    parameter integer CLK_FREQ = 74250000,   // clk_pixel
    parameter integer BAUD     = 460800,
    parameter [7:0]   THR_DEF  = 8'd16,      // = example_top 的 IMG_THR 默认值
    parameter [1:0]   VIEW_DEF = 2'd0,
    parameter [1:0]   MODE_DEF = 2'd3,
    parameter [7:0]   GAIN_R_DEF = 8'd112,   // 1/64 定点：112/64 = ×1.75（**上板实测定的**：
                                             // 2026-10-15 对白墙二分，×1.59 偏青、×2.0 偏暖、
                                             // ×1.75 中性。改前写死的是 409/256 ≈ ×1.59）
    parameter [7:0]   GAIN_B_DEF = 8'd96,
    parameter [1:0]   SMOOTH_DEF = 2'd1
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx_i,      // 空闲为高
    output wire       tx_o,

    // Threshold actually feeding Canny; used by the R status reply.
    input  wire [7:0] thr_read_i,
    // Smoothing mode actually feeding Canny; may be changed by board K1.
    input  wire [1:0] smooth_read_i,
    output reg  [7:0] thr_o,
    // One-clock pulse when a complete T<byte> command is accepted.
    output reg        thr_valid_o,
    output reg  [1:0] view_o,
    output reg  [1:0] mode_o,
    output reg  [1:0] smooth_o,
    // One-clock pulse when a complete S<byte> command is accepted.
    output reg        smooth_valid_o,
    // AWB 增益（1/64 定点）。默认 = 复位值 = 原 example_top 里写死的常数，
    // 所以"不插串口"和"插了不用"两种情况下画面与改动前逐像素一致。
    output reg  [7:0] gain_r_o,
    output reg  [7:0] gain_b_o
);

    //-----------------------------------------------------------------
    // 分频常数
    //-----------------------------------------------------------------
    function integer clog2;
        input integer value;
        integer i;
        begin
            clog2 = 0;
            for (i = value - 1; i > 0; i = i >> 1)
                clog2 = clog2 + 1;
        end
    endfunction

    localparam integer DIV = (CLK_FREQ + (BAUD / 2)) / BAUD;   // 四舍五入
    localparam integer CW  = clog2(DIV + 1);

    // 计数器预装值。RHS 特意先取 CW 位的部分选择再运算：DIV 是 integer，直接写
    // `DIV - 1` 会让表达式是 32 位，综合/仿真都报位宽截断（WIDTHTRUNC）。
    // CW = clog2(DIV+1) 保证 DIV < 2^CW，所以这个部分选择不丢信息。
    localparam [CW-1:0] CNT_BIT  = DIV[CW-1:0] - 1'b1;        // 一个 bit 周期
    localparam [CW-1:0] CNT_HALF = DIV[CW-1:0] / 2 - 1'b1;    // 半个 bit 周期

    //-----------------------------------------------------------------
    // 1) 接收：2FF 同步 + 起点检测 + 中点采样（8N1，LSB 先到）
    //-----------------------------------------------------------------
    reg [2:0] rx_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rx_sync <= 3'b111;                  // 空闲 = 高
        else        rx_sync <= {rx_sync[1:0], rx_i};
    end
    wire rx_bit = rx_sync[2];

    localparam [1:0] S_IDLE = 2'd0, S_START = 2'd1, S_DATA = 2'd2, S_STOP = 2'd3;

    reg [1:0]    rx_state;
    reg [CW-1:0] rx_cnt;
    reg [2:0]    rx_bidx;
    reg [7:0]    rx_sh;
    reg [7:0]    rx_byte;
    reg          rx_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state <= S_IDLE;
            rx_cnt   <= {CW{1'b0}};
            rx_bidx  <= 3'd0;
            rx_sh    <= 8'd0;
            rx_byte  <= 8'd0;
            rx_valid <= 1'b0;
        end else begin
            rx_valid <= 1'b0;                            // 单拍脉冲
            case (rx_state)
                S_IDLE: if (!rx_bit) begin               // 检测到起始位下降沿
                            rx_cnt   <= CNT_HALF;        // 半个 bit → 起始位中点
                            rx_state <= S_START;
                        end
                S_START: if (rx_cnt == 0) begin
                            rx_cnt   <= CNT_BIT;         // 再一个整 bit → bit0 中点
                            rx_bidx  <= 3'd0;
                            rx_state <= S_DATA;
                         end else rx_cnt <= rx_cnt - 1'b1;
                S_DATA:  if (rx_cnt == 0) begin
                            rx_sh    <= {rx_bit, rx_sh[7:1]};   // LSB 先到
                            rx_cnt   <= CNT_BIT;
                            if (rx_bidx == 3'd7) rx_state <= S_STOP;
                            else                 rx_bidx  <= rx_bidx + 1'b1;
                         end else rx_cnt <= rx_cnt - 1'b1;
                S_STOP:  if (rx_cnt == 0) begin
                            rx_byte  <= rx_sh;
                            rx_valid <= 1'b1;
                            rx_state <= S_IDLE;
                         end else rx_cnt <= rx_cnt - 1'b1;
                default: rx_state <= S_IDLE;
            endcase
        end
    end

    //-----------------------------------------------------------------
    // 2) 命令解析：2 字节帧，未知字符直接丢
    //-----------------------------------------------------------------
    localparam [7:0] C_T = 8'h54, C_V = 8'h56, C_M = 8'h4D, C_R = 8'h52,
                     C_A = 8'h41, C_B = 8'h42, C_S = 8'h53;

    reg       p_wait;                 // 正在等参数字节
    reg [7:0] p_cmd;
    reg       tx_req;
    reg [7:0] tx_d [0:6];             // {thr, view, mode, smooth, gain_r, gain_b, '\n'}

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            thr_o    <= THR_DEF;
            view_o   <= VIEW_DEF;
            mode_o   <= MODE_DEF;
            smooth_o <= SMOOTH_DEF;
            gain_r_o <= GAIN_R_DEF;
            gain_b_o <= GAIN_B_DEF;
            thr_valid_o <= 1'b0;
            smooth_valid_o <= 1'b0;
            p_wait   <= 1'b0;
            p_cmd    <= 8'd0;
            tx_req   <= 1'b0;
        end else begin
            tx_req <= 1'b0;
            thr_valid_o <= 1'b0;
            smooth_valid_o <= 1'b0;
            if (rx_valid) begin
                if (!p_wait) begin
                    case (rx_byte)
                        C_T, C_V, C_M, C_A, C_B, C_S: begin
                            p_cmd  <= rx_byte;
                            p_wait <= 1'b1;
                        end
                        C_R: begin
                            tx_d[0] <= thr_read_i;
                            tx_d[1] <= {6'b0, view_o};
                            tx_d[2] <= {6'b0, mode_o};
                            tx_d[3] <= gain_r_o;         // 回读里带上增益：
                            tx_d[4] <= gain_b_o;         // 调完不用再猜"到底生效没有"
                            tx_d[5] <= {6'b0, smooth_read_i};
                            tx_d[6] <= 8'h0A;
                            tx_req  <= 1'b1;
                        end
                        default: ;                        // 忽略（回车/换行/空格）
                    endcase
                end else begin
                    case (p_cmd)
                        C_T: begin
                            thr_o       <= rx_byte;
                            thr_valid_o <= 1'b1;
                        end
                        C_V:     view_o   <= rx_byte[1:0];
                        C_M:     mode_o   <= rx_byte[1:0];
                        C_S: begin
                            smooth_o       <= (rx_byte[1:0] > 2'd2) ? 2'd2 : rx_byte[1:0];
                            smooth_valid_o <= 1'b1;
                        end
                        C_A:     gain_r_o <= rx_byte;
                        C_B:     gain_b_o <= rx_byte;
                        default: ;
                    endcase
                    p_wait <= 1'b0;
                end
            end
        end
    end

    //-----------------------------------------------------------------
    // 3) 发送：'R' 的 7 字节回包。忙时来的新请求会被丢掉（调试口，可接受）
    //-----------------------------------------------------------------
    localparam [1:0] T_IDLE = 2'd0, T_START = 2'd1, T_DATA = 2'd2, T_STOP = 2'd3;

    reg [1:0]    tx_state;
    reg [CW-1:0] tx_cnt;
    reg [2:0]    tx_bidx;
    reg [7:0]    tx_sh;
    reg [2:0]    tx_idx;                               // 0..6，回包 7 字节
    reg          tx_line;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state <= T_IDLE;
            tx_cnt   <= {CW{1'b0}};
            tx_bidx  <= 3'd0;
            tx_sh    <= 8'd0;
            tx_idx   <= 3'd0;
            tx_line  <= 1'b1;                              // 空闲 = 高
        end else begin
            case (tx_state)
                T_IDLE: begin
                    tx_line <= 1'b1;
                    if (tx_req) begin
                        tx_sh    <= tx_d[0];
                        tx_idx   <= 3'd0;
                        tx_cnt   <= CNT_BIT;
                        tx_state <= T_START;
                    end
                end
                T_START: begin
                    tx_line <= 1'b0;
                    if (tx_cnt == 0) begin
                        tx_cnt   <= CNT_BIT;
                        tx_bidx  <= 3'd0;
                        tx_line  <= tx_sh[0];              // 第一个数据位
                        tx_state <= T_DATA;
                    end else tx_cnt <= tx_cnt - 1'b1;
                end
                T_DATA: begin
                    if (tx_cnt == 0) begin
                        tx_sh  <= {1'b0, tx_sh[7:1]};
                        tx_cnt <= CNT_BIT;
                        if (tx_bidx == 3'd7) begin
                            tx_line  <= 1'b1;              // 停止位
                            tx_state <= T_STOP;
                        end else begin
                            tx_bidx <= tx_bidx + 1'b1;
                            tx_line <= tx_sh[1];           // 移位后的新 LSB
                        end
                    end else tx_cnt <= tx_cnt - 1'b1;
                end
                T_STOP: begin
                    tx_line <= 1'b1;
                    if (tx_cnt == 0) begin
                        if (tx_idx == 3'd6) begin
                            tx_state <= T_IDLE;            // 7 字节发完
                        end else begin
                            tx_idx   <= tx_idx + 1'b1;
                            tx_sh    <= tx_d[tx_idx + 1'b1];
                            tx_cnt   <= CNT_BIT;
                            tx_state <= T_START;
                        end
                    end else tx_cnt <= tx_cnt - 1'b1;
                end
                default: tx_state <= T_IDLE;
            endcase
        end
    end

    assign tx_o = tx_line;

endmodule
