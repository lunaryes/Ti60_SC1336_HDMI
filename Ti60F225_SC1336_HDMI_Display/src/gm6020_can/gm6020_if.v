`timescale 1ns / 1ps
//=====================================================================
// gm6020_if —— GM6020 电机通信层（指令打包 + 反馈解析 + 多圈累加）
//---------------------------------------------------------------------
// 这是"协议层"：上面接视觉/控制给出的目标值，下面接 can_tx / can_rx。
// 板级胶合（节拍、软启动、限幅保护、状态灯）在 gm6020_can_top.v。
//
// 【报文】（RoboMaster 官方 CAN 协议）
//   电压控制  ID1~4 -> 0x1FF，ID5~7 -> 0x2FF   int16 大端，-30000..30000
//   电流控制  ID1~4 -> 0x1FE，ID5~7 -> 0x2FE   int16 大端，-16384..16384
//             电流模式必须在 RoboMaster Assistant 里打开"电流环开关"
//             （固件 >= 1.0.11.2），否则电机不动且不报错。
//   反馈帧    ID = 0x204 + 电机拨码 ID（ID1 -> 0x205），1 kHz
//             DATA[0:1] 机械角度 0..8191（13 bit，每圈回绕）
//             DATA[2:3] 转速 rpm   (int16)
//             DATA[4:5] 实际转矩电流 (int16，刻度同电流给定)
//             DATA[6]   温度 ℃
//             DATA[7]   保留
//
// 【本模块做的三件容易错的事】
//   1) 指令槽位：一帧 8 字节装 4 台电机，本机只填自己那一槽，其余补 0。
//      slot0 -> DATA[0:1]（bit 63:48），slot3 -> DATA[6:7]（bit 15:0）。
//   2) 13 bit 角度从 16 bit 大端里取低 13 位：
//      data_o[63:48] 是 DATA[0]:DATA[1] 拼成的 16 bit 值 A，
//      真正的角度是 A[12:0] = data_o[60:48]（不是 data_o[63:51]）。
//   3) 反馈是单圈绝对值，没有多圈计数器。过零处按 ±4096 判方向后累加，
//      1 kHz 下 320 rpm 只有 43.7 counts/样本，不会误判。
//      首帧直接以当前角度作为原点，避免从 0 起跳。
//
// 【报文长度与 ACK】
//   我们是总线上唯一的另一个节点，必须替 GM6020 的反馈帧回 ACK，
//   否则电机方的控制器会报 ACK 错并不断重发同一帧。
//   can_rx 的 ack_drive 正好在"CRC 界定符之后那一位"拉低，覆盖 ACK 槽；
//   此刻本方 can_tx 正处在自己的 ACK 槽（隐性），不会打架。
//   因此：can_tx_o = tx_bit & ~ack_drive（线与，显性优先）。
//=====================================================================
module gm6020_if #(
    parameter integer CLK_HZ       = 96_000_000,
    parameter integer BAUD         = 1_000_000,
    parameter integer CMD_HZ       = 1_000,     // 指令帧率，别超过 1 kHz
    parameter integer TIMEOUT_MS   = 10,        // 多久没反馈算掉线
    parameter integer LIMIT_VOLT   = 30000,     // 电压模式限幅（硬件上限 30000）
    parameter integer LIMIT_CUR    = 16384      // 电流模式限幅（对应 ±3A）
)(
    input  wire               clk,
    input  wire               rst_n,

    input  wire               can_rx_i,
    output wire               can_tx_o,

    input  wire [2:0]         motor_id_i,   // 拨码 ID，1..7
    input  wire               ctrl_mode_i,  // 0 = 电压(0x1FF/0x2FF)  1 = 电流(0x1FE/0x2FE)
    input  wire               enable_i,     // 0 = 发 0（松力），仍保持周期发送
    input  wire signed [15:0] cmd_i,        // 目标值（电压或电流，有符号）

    output reg  [12:0]        angle_o,      // 单圈机械角 0..8191
    output reg  signed [15:0] speed_o,      // rpm
    output reg  signed [15:0] cur_o,        // 实际转矩电流
    output reg  [7:0]         temp_o,       // ℃
    output reg  signed [31:0] pos_cnt_o,    // 多圈累加计数（上电位置为原点）
    output reg                pos_valid_o,  // 收到过至少一帧
    output reg                link_ok_o,    // 未超时
    output reg  [15:0]        bad_cnt_o,    // CRC/格式错帧数
    output reg  [15:0]        rx_cnt_o,     // 合法反馈帧数
    output wire [15:0]        ack_err_cnt_o,// 发送帧在 ACK 槽无人应答的次数
    output wire               tx_busy_o,
    output wire               tx_done_o,
    //-------------------------------------------------------------
    // 自环检测（诊断用）：自己发出去的指令帧又在自己 RX 上被完整收到，
    // 说明 FPGA 的 TX 脚 → 收发器 D → 总线 → 收发器 R → FPGA RX 整条路是通的，
    // 且发出去的帧 CRC 是正确的（否则 can_rx 不会给出 valid）。
    // 单节点也能成立：收发器 D 拉低会把总线拉成显性，R 随之变低。
    //-------------------------------------------------------------
    output reg                loop_ok_o,    // 收到过自己发的指令帧（CRC 通过）
    output reg                loop_err_o,   // 收到过 ID 相同但 CRC 错的帧
    //-------------------------------------------------------------
    // 物理回环（更底层、更宽松的一级诊断，不要求整帧解析成功）
    // 发送期间收发器会把 D 脚电平送到总线，R 脚再送回 FPGA 的 RX；
    // 因此链路完好时 tx_bit 与 can_rx_i 在发送窗口内逐拍一致（仅差几十 ns）。
    // 若模块没供电 / D 脚没接到 / FPGA 引脚没真正驱动，RX 会一直停在高电平，
    // 而 tx_bit 里有大量低电平位 → 不一致计数暴涨。
    //-------------------------------------------------------------
    output reg                tx_echo_o     // 1 = 发送时自己的位能在 RX 上看到
);

//---------------------------------------------------------------- 指令节拍
localparam integer CMD_DIV = (CLK_HZ + CMD_HZ/2) / CMD_HZ;

//---------------------------------------------------------------- 总线空闲门（★关键修复）
// 发帧前必须确认总线已空闲 ≥ 11 位（CAN 规范的帧间空间）且此刻仍为隐性。
// 没有这个门时是"盲发"：GM6020 以 1kHz 自主发 0x205 反馈帧，我们也以 1kHz
// 发 0x1FF，两路各占约 11% 总线时间，极易相撞——我们的帧头插进电机帧中间，
// 把对方的帧打坏（RX 收到半截 → bad_cnt 疯涨），电机检测到错误后重发，
// 错误计数飙升直至 bus-off，指令帧也全部作废 → 电机不转。
// 注：RX 线若断线悬空呈高电平，此门照常打开，不影响发送。
localparam integer BIT_CLKS_I = (CLK_HZ + (BAUD/2)) / BAUD;
localparam integer IDLE_CLKS  = BIT_CLKS_I * 11;

reg [15:0] idle_cnt;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)                    idle_cnt <= 16'd0;
    else if (!can_rx_i)            idle_cnt <= 16'd0;
    else if (idle_cnt < IDLE_CLKS) idle_cnt <= idle_cnt + 16'd1;
end
wire bus_idle = (idle_cnt >= IDLE_CLKS) && can_rx_i;

reg [31:0] cmd_div;
reg        cmd_tick;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cmd_div  <= 32'd0;
        cmd_tick <= 1'b0;
    end else if (cmd_div >= CMD_DIV-1) begin
        cmd_div  <= 32'd0;
        cmd_tick <= 1'b1;
    end else begin
        cmd_div  <= cmd_div + 32'd1;
        cmd_tick <= 1'b0;
    end
end

//---------------------------------------------------------------- 指令打包
wire       id_ok = (motor_id_i >= 3'd1) && (motor_id_i <= 3'd7);
wire [2:0] slot_x = (motor_id_i >= 3'd5) ? (motor_id_i - 3'd5)
                                          : (motor_id_i - 3'd1);
wire [1:0] slot  = slot_x[1:0];
wire       hi    =  (motor_id_i >= 3'd5);

wire [10:0] tx_id = ctrl_mode_i ? (hi ? 11'h2FE : 11'h1FE)
                                : (hi ? 11'h2FF : 11'h1FF);

// 限幅用 17 bit 中间量，比较不会溢出
wire signed [16:0] cmd_e = {cmd_i[15], cmd_i};
wire signed [16:0] lim_e = ctrl_mode_i ? LIMIT_CUR : LIMIT_VOLT;
wire signed [16:0] cmd_c = (cmd_e >  lim_e) ?  lim_e :
                           (cmd_e < -lim_e) ? -lim_e : cmd_e;
wire signed [15:0] cmd_f = (enable_i && id_ok) ? cmd_c[15:0] : 16'sd0;

reg [63:0] tx_data;
always @(*) begin
    tx_data = 64'd0;
    if (id_ok)
        tx_data[(3 - slot) * 16 +: 16] = cmd_f;   // slot0 -> bit63:48
end

// 发起发送：cmd_tick 打一个挂起标志，等总线空下来再发，避免丢帧
reg pend;
reg tx_req;

wire tx_busy, tx_done, tx_bit, ack_err;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pend   <= 1'b0;
        tx_req <= 1'b0;
    end else begin
        tx_req <= 1'b0;
        if (cmd_tick)                    pend <= 1'b1;
        else if (pend && !tx_busy && bus_idle) begin  // 总线空闲才发，避免撞掉电机的反馈帧
            tx_req <= 1'b1;
            pend   <= 1'b0;
        end
    end
end

can_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_can_tx (
    .clk      (clk),
    .rst_n    (rst_n),
    .req_i    (tx_req),
    .id_i     (tx_id),
    .data_i   (tx_data),
    .rx_i     (can_rx_i),
    .tx_o     (tx_bit),
    .busy_o   (tx_busy),
    .done_o   (tx_done),
    .ack_err_o(ack_err)
);

// ACK 错误计数：连续增长 = 发送的帧根本没上总线 / 总线上无应答方。
//   与 tx_echo 区分：ack_err 只在"ACK 槽总线仍为隐性"时计数，
//   对收发器待机（RS 悬空）、D 脚断线、FPGA 引脚未驱动这一类故障极其敏感。
reg [15:0] ack_err_cnt;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)                     ack_err_cnt <= 16'd0;
    else if (ack_err && ack_err_cnt != 16'hFFFF)
                                    ack_err_cnt <= ack_err_cnt + 16'd1;
end

assign tx_busy_o = tx_busy;
assign tx_done_o = tx_done;
assign ack_err_cnt_o = ack_err_cnt;

//---------------------------------------------------------------- 物理回环计数
// 只在 tx_busy 窗口内比较"本机正在驱动的位"和"RX 脚实际看到的位"。
// 一帧约 108 位 × 96 clk ≈ 10400 clk；链路完好时 hit≈10000、mis 只有边沿和
// ACK 槽带来的几百个。链路断开时 RX 恒为高，mis 会达到四五千。
reg [2:0]  tx_bit_d;
reg [15:0] echo_hit, echo_mis;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) tx_bit_d <= 3'b111;
    else        tx_bit_d <= {tx_bit_d[1:0], tx_bit};
end

wire tx_bit_dly = tx_bit_d[2];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        echo_hit  <= 16'd0;
        echo_mis  <= 16'd0;
        tx_echo_o <= 1'b0;
    end else if (tx_done) begin
        // 每帧结束判定并清零，避免计数跨帧累积掩盖问题
        if ((echo_hit > 16'd7000) && (echo_mis < 16'd800))
            tx_echo_o <= 1'b1;
        echo_hit <= 16'd0;
        echo_mis <= 16'd0;
    end else if (tx_busy) begin
        if (can_rx_i === tx_bit_dly) echo_hit <= echo_hit + 16'd1;
        else                         echo_mis <= echo_mis + 16'd1;
    end
end

//---------------------------------------------------------------- 反馈接收
// FILTER_EN=0：ID 期望值由 motor_id_i 决定，编译期常量过滤不够用，
// 统一在下面用 rx_id == rx_id_expected 判断。
wire [10:0] rx_id_expected = 11'h204 + {8'd0, motor_id_i};
wire [10:0] rx_id;
wire [63:0] rx_data;
wire        rx_valid, rx_err;
wire        ack_drive;

can_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD), .FILTER_EN(0)) u_can_rx (
    .clk      (clk),
    .rst_n    (rst_n),
    .rx_i     (can_rx_i),
    .id_o     (rx_id),
    .data_o   (rx_data),
    .valid_o  (rx_valid),
    .crc_err_o(rx_err),
    .ack_drive(ack_drive)
);

// 线与：本方发送位 与 接收方 ACK 拉低 相与，显性(0)优先
assign can_tx_o = tx_bit & ~ack_drive;

wire        rx_hit   = rx_valid && id_ok && (rx_id == rx_id_expected);
// 16 bit 大端里角度只有低 13 位有效
wire [15:0] ang_word = rx_data[63:48];
wire [12:0] f_angle  = ang_word[12:0];
wire signed [15:0] f_speed = rx_data[47:32];
wire signed [15:0] f_cur   = rx_data[31:16];
wire [7:0]         f_temp  = rx_data[15:8];

//---------------------------------------------------------------- 自环检测
// 收到 ID 等于本机指令 ID 的帧 = 自己发出去的那一帧绕回自己了
wire rx_self = rx_valid && (rx_id == tx_id);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        loop_ok_o  <= 1'b0;
        loop_err_o <= 1'b0;
    end else begin
        if (rx_self)                    loop_ok_o  <= 1'b1;
        if (rx_err && (rx_id == tx_id)) loop_err_o <= 1'b1;
    end
end

//---------------------------------------------------------------- 多圈累加
reg  [12:0]        ang_prev;
reg  signed [31:0] pos_cnt;
reg                first_seen;

wire signed [13:0] d_raw = $signed({1'b0, f_angle}) - $signed({1'b0, ang_prev});
wire signed [13:0] d_wrap = (d_raw >  14'sd4096) ? (d_raw - 14'sd8192) :
                            (d_raw < -14'sd4096) ? (d_raw + 14'sd8192) : d_raw;

//---------------------------------------------------------------- 掉线检测
localparam integer TO_CLKS = (CLK_HZ/1000) * TIMEOUT_MS;
reg [23:0] to_cnt;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        angle_o <= 13'd0; speed_o <= 16'sd0; cur_o <= 16'sd0; temp_o <= 8'd0;
        pos_cnt <= 32'sd0; ang_prev <= 13'd0; first_seen <= 1'b0;
        pos_valid_o <= 1'b0; link_ok_o <= 1'b0;
        bad_cnt_o <= 16'd0; rx_cnt_o <= 16'd0; to_cnt <= 24'd0;
        pos_cnt_o <= 32'sd0;
    end else begin
        pos_cnt_o <= pos_cnt;

        if (rx_err)
            bad_cnt_o <= bad_cnt_o + 16'd1;

        if (rx_hit) begin
            angle_o  <= f_angle;
            speed_o  <= f_speed;
            cur_o    <= f_cur;
            temp_o   <= f_temp;
            ang_prev <= f_angle;
            to_cnt   <= 24'd0;
            rx_cnt_o <= rx_cnt_o + 16'd1;
            pos_valid_o <= 1'b1;
            link_ok_o   <= 1'b1;
            if (!first_seen) begin
                // 上电第一帧：直接以当前位置为原点，不从 0 起跳
                first_seen <= 1'b1;
                pos_cnt    <= $signed({19'd0, f_angle});
            end else begin
                pos_cnt <= pos_cnt + {{18{d_wrap[13]}}, d_wrap};
            end
        end else if (to_cnt >= TO_CLKS) begin
            link_ok_o <= 1'b0;
            to_cnt    <= to_cnt;
        end else begin
            to_cnt <= to_cnt + 24'd1;
        end
    end
end

endmodule
