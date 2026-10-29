`timescale 1ns / 1ps
//=====================================================================
// gm6020_can_top —— GM6020 单电机 CAN 通信板级模块（VF-Ti60F225 用）
//---------------------------------------------------------------------
// 层次：
//   gm6020_can_top            ← 本文件：复位同步 / 上电静默 / 软启动 / 掉线保护 / 状态灯
//     └ gm6020_if             ← 协议层：指令打包、反馈解析、多圈累加（gm6020_if.v）
//         ├ can_tx            ← 物理层：位定时、位填充、CRC15（can_tx.v）
//         └ can_rx            ← 物理层：硬同步、去填充、CRC 校验（can_rx.v）
//
// 时钟：clk 必须是 96 MHz（clk_sys）。96/1 Mbps 整除，零误差。
//       绝对不能用 clk_pixel 74.25 MHz（74.25 非整数分频，0.34% 误差会偶发误帧）。
//
// 引脚（VF-Ti60F225，底板 DC3-40 排针 P4）：
//   can_tx_o -> FPGA 球 A9  = GPIOT_N_11 = User_IO34 = P4 第 39 脚
//   can_rx_i <- FPGA 球 B9  = GPIOT_P_11 = User_IO35 = P4 第 40 脚
//   FPGA 侧 bank 2B 是 1.8V，Interface Designer 里 I/O Standard 要选 1.8 V LVCMOS；
//   底板上这两路经过 TXS0108E 自动双向转换，出排针时已是 3.3V，直接进 SN65HVD230。
//
// 使用方式：
//   DEMO_MODE=1 → 不看外部输入，上电 10 ms 后把 TARGET_CMD 斜坡加到位（点灯自检用）
//   DEMO_MODE=0 → 由 cmd_i / enable_i 控制，接视觉支路的输出（三角形跟随用）
//
//   led_o 输出的是"逻辑亮"（1=亮），板上 LED 是低电平点亮，
//   所以在 example_top 里接的是： assign led_o[5:0] = ~gp6020_status;
//=====================================================================
module gm6020_can_top #(
    parameter integer CLK_HZ      = 96_000_000,
    parameter integer BAUD        = 1_000_000,
    parameter [2:0]   MOTOR_ID    = 3'd1,        // 电机拨码 ID
    parameter         CTRL_MODE   = 1'b0,        // 0 = 电压控制  1 = 电流控制
    parameter signed [15:0] TARGET_CMD = 16'sd0, // DEMO 模式的目标值
    parameter integer RAMP_STEP   = 50,          // 每帧变化量（1 kHz 帧率下 50 ≈ 240 ms 到 6000）
    parameter integer SILENT_MS   = 10,          // 上电静默，等 PLL 锁定
    parameter integer FAULT_MS    = 100,         // 曾经连上后又丢了这么久 → 松力
    parameter integer STRETCH_MS  = 20,          // 状态灯闪烁展宽
    parameter         DEMO_MODE   = 1'b1,
    //-------------------------------------------------------------
    // SEQ_MODE：三态自检序列（验证极性/停车是否受控）
    //   段 0: 0      停
    //   段 1: +CMD   正转
    //   段 2: 0      停
    //   段 3: -CMD   反转
    //   每段停留 SEQ_HOLD_MS 毫秒，四段循环。
    //   SEQ_MODE=1 时忽略 DEMO_MODE / TARGET_CMD / cmd_i。
    //-------------------------------------------------------------
    parameter         SEQ_MODE    = 1'b0,
    parameter integer SEQ_CMD     = 6000,
    parameter integer SEQ_HOLD_MS = 3000,
    //-------------------------------------------------------------
    // PIN_TEST：引脚定位模式（排针编号对不上时用，CAN 全部旁路）
    //   can_tx_o 输出 1 Hz 方波（0.5 s 高 / 0.5 s 低）
    //   LED1 = 该方波（和 TX 同步闪）
    //   LED2 = can_rx_i 的实时电平（跟随输入）
    //   用法：万用表直流档扫排针，电压在 0 / 3.3 V 之间慢速跳的那根就是 TX 脚；
    //         再用一根线把 TX 脚接到候选针，LED2 跟着亮灭 → 那根就是 RX 脚。
    //-------------------------------------------------------------
    parameter         PIN_TEST    = 1'b0
)(
    input  wire               clk,
    input  wire               rst_n,          // 低有效，内部做同步释放

    input  wire               can_rx_i,
    output wire               can_tx_o,

    input  wire signed [15:0] cmd_i,          // DEMO_MODE=0 时有效
    input  wire               enable_i,

    output wire [12:0]        angle_o,
    output wire signed [15:0] speed_o,
    output wire signed [15:0] cur_o,
    output wire [7:0]         temp_o,
    output wire signed [31:0] pos_cnt_o,
    output wire               link_ok_o,
    output wire [15:0]        bad_cnt_o,
    output reg  [5:0]         led_o         // 逻辑亮（1=亮）
);

//---------------------------------------------------------------- 复位同步
reg [2:0] rst_sync;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) rst_sync <= 3'b000;
    else        rst_sync <= {rst_sync[1:0], 1'b1};
end
wire rst_n_s = rst_sync[2];

//---------------------------------------------------------------- 上电静默
localparam integer SILENT_CLKS = (CLK_HZ/1000) * SILENT_MS;
reg [23:0] silent_cnt;
reg        started;

always @(posedge clk or negedge rst_n_s) begin
    if (!rst_n_s) begin
        silent_cnt <= 24'd0;
        started    <= 1'b0;
    end else if (!started) begin
        if (silent_cnt >= SILENT_CLKS-1) started <= 1'b1;
        else                             silent_cnt <= silent_cnt + 24'd1;
    end
end

//---------------------------------------------------------------- 三态自检序列
localparam integer SEG_CLKS = (CLK_HZ/1000) * SEQ_HOLD_MS;
localparam signed [15:0] SEQ_P = SEQ_CMD;      // 正转幅值
localparam signed [15:0] SEQ_N = -SEQ_CMD;     // 反转幅值（负号在这里取，避免位宽问题）

reg [1:0]  seg;
reg [31:0] seg_cnt;

always @(posedge clk or negedge rst_n_s) begin
    if (!rst_n_s) begin
        seg     <= 2'd0;
        seg_cnt <= 32'd0;
    end else if (!started) begin
        seg     <= 2'd0;
        seg_cnt <= 32'd0;
    end else if (seg_cnt >= SEG_CLKS-1) begin
        seg_cnt <= 32'd0;
        seg     <= seg + 2'd1;          // 2 bit 自然回绕：0→1→2→3→0
    end else begin
        seg_cnt <= seg_cnt + 32'd1;
    end
end

wire signed [15:0] seq_cmd = (seg == 2'd1) ? SEQ_P :
                             (seg == 2'd3) ? SEQ_N : 16'sd0;

//---------------------------------------------------------------- 协议层
wire        tx_busy, tx_done;
wire [15:0] rx_cnt, ack_err_cnt;
reg  signed [16:0] ramp;

// SEQ_MODE 优先；其次 DEMO_MODE 常量；最后外部视觉支路
wire signed [15:0] tgt_raw = SEQ_MODE ? seq_cmd :
                             (DEMO_MODE ? TARGET_CMD : cmd_i);

reg        ever_ok;
reg [23:0] lost_cnt;
localparam integer FAULT_CLKS = (CLK_HZ/1000) * FAULT_MS;

always @(posedge clk or negedge rst_n_s) begin
    if (!rst_n_s) begin
        ever_ok  <= 1'b0;
        lost_cnt <= 24'd0;
    end else if (link_ok_o) begin
        ever_ok  <= 1'b1;
        lost_cnt <= 24'd0;
    end else if (lost_cnt < FAULT_CLKS) begin
        lost_cnt <= lost_cnt + 24'd1;
    end
end

// 掉线保护：曾经连上过、现在断了超过 FAULT_MS → 松力。
// 做成组合条件而不是锁存：线重新插上、反馈恢复后自动解除，不用复位。
wire fault = ever_ok && !link_ok_o && (lost_cnt >= FAULT_CLKS);

// 目标值：DEMO 模式取常量，否则取外部；掉线保护生效时强制 0（松力）
wire signed [16:0] tgt_e = fault ? 17'sd0 : {tgt_raw[15], tgt_raw};
wire               run_en = started && !fault && (DEMO_MODE ? 1'b1 : enable_i);

// 每发完一帧，向目标逼近一步；两个方向都限速，避免上电猛甩
// 注意：差值用 18 bit 有符号量比较，别直接拿 17 bit 和 integer 参数比
//       —— Verilog 里只要有一个操作数无符号，比较就会变成无符号，负数会判错。
localparam signed [17:0] STEP = RAMP_STEP;
wire signed [17:0] d_up = $signed({tgt_e[16], tgt_e}) - $signed({ramp[16], ramp});
wire signed [17:0] d_dn = $signed({ramp[16], ramp}) - $signed({tgt_e[16], tgt_e});

always @(posedge clk or negedge rst_n_s) begin
    if (!rst_n_s) begin
        ramp <= 17'sd0;
    end else if (!run_en) begin
        ramp <= 17'sd0;
    end else if (tx_done) begin
        if (ramp < tgt_e)
            ramp <= (d_up > STEP) ? (ramp + STEP[16:0]) : tgt_e;
        else if (ramp > tgt_e)
            ramp <= (d_dn > STEP) ? (ramp - STEP[16:0]) : tgt_e;
    end
end

gm6020_if #(
    .CLK_HZ    (CLK_HZ),
    .BAUD      (BAUD),
    .CMD_HZ    (1_000),
    .TIMEOUT_MS(10)
) u_gm6020 (
    .clk         (clk),
    .rst_n       (rst_n_s),
    .can_rx_i    (can_rx_i),
    .can_tx_o    (can_tx_raw),
    .motor_id_i  (MOTOR_ID),
    .ctrl_mode_i (CTRL_MODE),
    .enable_i    (run_en),
    .cmd_i       (ramp[15:0]),
    .angle_o     (angle_o),
    .speed_o     (speed_o),
    .cur_o       (cur_o),
    .temp_o      (temp_o),
    .pos_cnt_o   (pos_cnt_o),
    .pos_valid_o (),
    .link_ok_o   (link_ok_o),
    .bad_cnt_o   (bad_cnt_o),
    .rx_cnt_o    (rx_cnt),
    .ack_err_cnt_o(ack_err_cnt),
    .tx_busy_o   (tx_busy),
    .tx_done_o   (tx_done),
    .loop_ok_o   (loop_ok),
    .loop_err_o  (loop_err),
    .tx_echo_o   (tx_echo)
);

//---------------------------------------------------------------- 自环 / 诊断信号
wire loop_ok, loop_err, tx_echo;

//---------------------------------------------------------------- 引脚定位模式
wire can_tx_raw;

localparam integer SQ_CLKS = CLK_HZ/2;      // 0.5 s 半周期 → 1 Hz 方波
reg [26:0] sq_cnt;
reg        pin_sq;

always @(posedge clk or negedge rst_n_s) begin
    if (!rst_n_s) begin
        sq_cnt <= 27'd0;
        pin_sq <= 1'b0;
    end else if (sq_cnt >= SQ_CLKS-1) begin
        sq_cnt <= 27'd0;
        pin_sq <= ~pin_sq;
    end else begin
        sq_cnt <= sq_cnt + 27'd1;
    end
end

// PIN_TEST 时把 TX 脚直接变成方波输出，旁路整个 CAN 发送链
assign can_tx_o = PIN_TEST ? pin_sq : can_tx_raw;

// ramp 绝对值，用来判断"指令是否真的爬到位了"
wire signed [16:0] ramp_abs = ramp[16] ? -ramp : ramp;
wire               ramp_live = (ramp_abs > 17'sd1000);

//---------------------------------------------------------------- 状态灯
// 现在是诊断映射：6 颗灯每颗都为一个具体故障点服务
//   LED1 心跳 2Hz          = FPGA 活着
//   LED2 常亮              = 发送时自己的位能在 RX 上看到（TX 物理链路通）
//   LED3 常亮              = ACK 槽无人应答（帧没上总线/收发器待机/无节点）★TX 侧铁证
//   LED4 常亮              = 掉线保护生效（曾被判定掉线，指令被强制清零）
//   LED5 常亮              = 出现过 CRC/格式错帧
//   LED6 常亮              = 整帧自收成功（CRC/ID/ACK 全对）或收到电机反馈
localparam integer BLINK_CLKS   = CLK_HZ/4;             // 0.25 s 翻转 → 2 Hz
localparam integer STRETCH_CLKS = (CLK_HZ/1000) * STRETCH_MS;

reg [25:0] blink_cnt;
reg        blink;
always @(posedge clk or negedge rst_n_s) begin
    if (!rst_n_s) begin
        blink_cnt <= 26'd0; blink <= 1'b0;
    end else if (blink_cnt >= BLINK_CLKS-1) begin
        blink_cnt <= 26'd0; blink <= ~blink;
    end else begin
        blink_cnt <= blink_cnt + 26'd1;
    end
end

reg [15:0] rx_cnt_d;
reg [23:0] tx_stretch, rx_stretch;

always @(posedge clk or negedge rst_n_s) begin
    if (!rst_n_s) begin
        rx_cnt_d   <= 16'd0;
        tx_stretch <= 24'd0;
        rx_stretch <= 24'd0;
    end else begin
        rx_cnt_d <= rx_cnt;
        if (tx_done)                     tx_stretch <= STRETCH_CLKS;
        else if (tx_stretch != 24'd0)    tx_stretch <= tx_stretch - 24'd1;
        if (rx_cnt != rx_cnt_d)          rx_stretch <= STRETCH_CLKS;
        else if (rx_stretch != 24'd0)    rx_stretch <= rx_stretch - 24'd1;
    end
end

always @(*) begin
    if (PIN_TEST) begin
        led_o[0] = pin_sq;               // 与 TX 脚完全同步的 1 Hz 方波
        led_o[1] = can_rx_i;             // RX 引脚实时电平
        led_o[2] = 1'b0;
        led_o[3] = 1'b0;
        led_o[4] = 1'b0;
        led_o[5] = 1'b0;
    end else begin
        led_o[0] = blink;                // 心跳，2 Hz 闪 = 时钟与复位正常
        led_o[1] = link_ok_o;            // 常亮 = 收到 GM6020 反馈（RX 方向通）
        led_o[2] = tx_echo;              // 常亮 = 发送时自己的位在 RX 上看得见（TX 物理链路通）
        led_o[3] = fault | (ack_err_cnt != 16'd0); // 亮 = ACK 槽无人应答（帧没上总线）或掉线保护
        led_o[4] = (bad_cnt_o != 16'd0); // 亮 = 出现过 CRC/格式错帧
        led_o[5] = loop_ok;              // 常亮 = 整帧自收成功（CRC/ID/ACK 全对）
    end
end

endmodule
