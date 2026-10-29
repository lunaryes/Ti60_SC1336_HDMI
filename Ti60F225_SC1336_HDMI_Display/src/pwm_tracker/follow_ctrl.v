`timescale 1ns / 1ps
//=====================================================================
// follow_ctrl —— 三角形跟随控制律（增量式积分 + 死区 + 斜率限幅）
//---------------------------------------------------------------------
// 为什么必须是"增量式"而不是"绝对位置式"：
//   PWM 模式没有反馈。你不知道电机轴现在指向哪里，也就无法把
//   "目标角度"换算成"绝对脉宽"。增量式的做法是：
//       每帧只算"该往哪边挪多少"，累加到当前脉宽上。
//   好处是起始脉宽错了也会自己收敛回来；坏处是必须靠视觉持续纠偏。
//   这也正是 CAN 闭环能解决、而 PWM 解决不了的那件事 —— 用 PWM
//   就接受"开环跟随"，别指望它抗负载扰动。
//
// 控制律（每帧执行一次，一帧 = 一次新的检测结果）：
//       e  = ex            （半格单位，右正左负）
//       if |e| > DEADZONE:
//           step = (e * GAIN_NUM) >>> GAIN_SHIFT      // 比例增益
//           step = clamp(step, -MAX_STEP, +MAX_STEP)  // 斜率限幅
//           acc  = acc + (DIR_INVERT ? -step : step)
//       acc = clamp(acc, PULSE_MIN, PULSE_MAX)
//
// 增益只能开到 1/4 的硬理由：
//   端到端延迟链 = 相机曝光 1 帧(16.7 ms) + 跟踪确认 3 帧(50 ms)
//                + PWM 更新(最多 20 ms) + 电机内部位置环(20~50 ms)
//                ≈ 50 ~ 90 ms
//   闭环带宽因此只有约 2 Hz。1:1 全量修正（GAIN_SHIFT=0）必然过冲振荡，
//   1/4（GAIN_SHIFT=2）在 60 Hz 帧率下是稳定的最优区间。
//
// GAIN_NUM 怎么算（换镜头/改 theta_max 时必须重算）：
//   unity gain 时"一个半格误差 -> 一个半格对应的脉宽增量"：
//       GAIN_NUM = PULSE_SPAN * (H_FOV_deg / GRID_W) / THETA_MAX_deg / 2
//   例：PULSE_SPAN = 184320-96000 = 88320 counts
//       H_FOV = 60 deg, GRID_W = 160 -> 1 格 = 0.375 deg，半格 = 0.1875 deg
//       THETA_MAX = 360 deg
//       GAIN_NUM = 88320 * 0.1875 / 360 = 46.0   （本设计取 44，留 5% 余量）
//   theta_max 若在 Assistant 里标定成 180 deg，GAIN_NUM 就要翻倍成 ~92。
//
// 丢失策略（配合 tri_target_extract.v 的"贴边即消失"特性）：
//   第 0 ~ LOST_HOLD 帧       ：完全冻结（目标可能只是被短时遮挡）
//   第 LOST_HOLD 帧之后       ：每帧朝 PULSE_MID 挪 RETURN_STEP，缓慢回中
//   回到 PULSE_MID 后仍没有   ：进入 SEEK 慢速扫描，来回找目标
//   一直没信号                ：在行程内来回扫（不会停死在某个角度）
//
// 为什么必须有 SEEK：
//   这是一个纯视觉伺服系统，没有别的传感器。如果上电时三角形根本
//   不在视野里（或目标跑出画面），"看不见 -> 不动 -> 更看不见"
//   会锁死。SEEK 用最慢的速度扫全程，等目标自己进画面。
//   把 SEEK_STEP 设为 0 就关闭这个行为（只在已知目标总在视野内时才这么用）。
//
// SELF_TEST=1：完全不用相机，每帧把脉宽从 1000 us 扫到 1920 us 再扫回来。
//   用来在焊完线、还没调视觉时先验证"PWM 通不通、电机动不动、方向对不对"。
//=====================================================================
module follow_ctrl #(
    parameter integer PULSE_MIN   = 96_000,    // 1000 us
    parameter integer PULSE_MAX   = 184_320,   // 1920 us
    parameter integer PULSE_MID   = 140_160,   // 1460 us，上电中位
    parameter integer GAIN_NUM    = 44,        // 见上方公式
    parameter integer GAIN_SHIFT  = 2,         // 1/4 增益
    parameter integer DEADZONE    = 3,         // 半格单位，1.5 格 = 12 像素
    parameter integer MAX_STEP    = 3000,      // 每帧最大脉宽变化（counts）
    parameter integer DIR_INVERT  = 0,         // 方向反了就置 1
    parameter integer LOST_HOLD   = 30,        // 丢目标后先冻结多少帧（0.5 s）
    parameter integer RETURN_STEP = 200,       // 回中速度（counts/帧）
    parameter integer SEEK_STEP   = 400,       // 回中后仍找不到时的扫描速度（0 = 关闭）
    parameter integer SEEK_DELAY  = 30,        // 回中后先停多少帧再开始扫（0.5 s）
    parameter integer SCAN_STEP   = 800,       // SELF_TEST 扫描速度（counts/帧）
    parameter         SELF_TEST   = 0
)(
    input  wire              clk,        // clk_sys 96 MHz
    input  wire              rst_n,

    input  wire signed [9:0] ex_i,       // 水平误差（半格单位）
    input  wire              found_i,    // 本帧是否找到三角形
    input  wire              tick_i,     // 每帧一个脉冲（来自 axis_cdc 的 valid_d）

    output reg  [20:0]       pulse_o,    // -> gm6020_pwm_gen.pulse_i
    output reg  [3:0]        status_o    // [0]找到 [1]死区 [2]回中 [3]到限位
);

//---------------------------------------------------------------- 组合：本帧增量
reg signed [31:0] acc;        // 当前脉宽（counts），32 bit 便于容错
reg [7:0]         lost;       // 连续丢失帧数
reg               scan_dir;   // SELF_TEST 扫描方向：1 向上
reg               seek;       // 已经回中，正在慢扫搜索目标
reg               seek_dir;   // 搜索方向：1 向上
reg [7:0]         seek_wait;  // 回中后等待计数

reg signed [31:0] prod_c;
reg signed [31:0] step_c;
reg signed [31:0] acc_n;
reg        [7:0]  lost_n;
reg               seek_n;
reg        [7:0]  seek_wait_n;

always @(*) begin
    // 比例项：ex * GAIN_NUM / 2^GAIN_SHIFT
    // GAIN_NUM 是常数，Efinity 会把它综合成移位加法树，不占 DSP。
    prod_c = ex_i * GAIN_NUM;
    step_c = prod_c >>> GAIN_SHIFT;

    if (step_c >  MAX_STEP) step_c =  MAX_STEP;
    if (step_c < -MAX_STEP) step_c = -MAX_STEP;

    if (DIR_INVERT != 0) step_c = -step_c;

    acc_n  = acc;
    lost_n = lost;
    seek_n = seek;
    seek_wait_n = seek_wait;

    if (SELF_TEST) begin
        // 自检扫描：不看视觉，纯斜坡往返
        if (scan_dir) acc_n = acc + SCAN_STEP;
        else          acc_n = acc - SCAN_STEP;
    end else if (found_i) begin
        lost_n = 8'd0;
        seek_n = 1'b0;
        seek_wait_n = 8'd0;
        if ((ex_i > DEADZONE) || (ex_i < -DEADZONE)) begin
            acc_n = acc + step_c;
        end
    end else begin
        // 目标丢失：先冻结 -> 再回中 -> 停一下 -> 最后慢扫
        if (lost != 8'hFF) lost_n = lost + 8'd1;
        if (lost >= LOST_HOLD) begin
            if (!seek) begin
                // 阶段 A：回到中位
                if      (acc > PULSE_MID + RETURN_STEP) acc_n = acc - RETURN_STEP;
                else if (acc < PULSE_MID - RETURN_STEP) acc_n = acc + RETURN_STEP;
                else begin
                    acc_n       = PULSE_MID;
                    seek_n      = 1'b1;
                    seek_wait_n = 8'd0;
                end
            end else if (seek_wait < SEEK_DELAY) begin
                // 阶段 B：停在中位等一会儿，目标可能正好出现在这里
                acc_n       = PULSE_MID;
                if (seek_wait != 8'hFF) seek_wait_n = seek_wait + 8'd1;
            end else if (SEEK_STEP != 0) begin
                // 阶段 C：在行程内来回慢扫，等目标重新进画面
                if (seek_dir) acc_n = acc + SEEK_STEP;
                else          acc_n = acc - SEEK_STEP;
            end
        end
    end

    // 机械限幅：永远不允许越界
    if (acc_n < PULSE_MIN) acc_n = PULSE_MIN;
    if (acc_n > PULSE_MAX) acc_n = PULSE_MAX;
end

//---------------------------------------------------------------- 时序
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        acc      <= PULSE_MID;
        lost     <= 8'd0;
        scan_dir <= 1'b1;
        seek     <= 1'b0;
        seek_dir <= 1'b1;
        seek_wait<= 8'd0;
        pulse_o  <= PULSE_MID;
        status_o <= 4'b0000;
    end else begin
        if (tick_i) begin
            acc       <= acc_n;
            lost      <= lost_n;
            seek      <= seek_n;
            seek_wait <= seek_wait_n;

            // 扫描方向在端点处翻转
            if (SELF_TEST) begin
                if      (acc_n >= PULSE_MAX) scan_dir <= 1'b0;
                else if (acc_n <= PULSE_MIN) scan_dir <= 1'b1;
            end else if (SEEK_STEP != 0) begin
                if      (acc_n >= PULSE_MAX) seek_dir <= 1'b0;
                else if (acc_n <= PULSE_MIN) seek_dir <= 1'b1;
            end

            if (SELF_TEST) begin
                status_o <= 4'b0010;              // 自检中
            end else begin
                status_o[0] <= found_i;
                status_o[1] <= found_i && !((ex_i > DEADZONE) || (ex_i < -DEADZONE));
                status_o[2] <= !found_i && (lost >= LOST_HOLD);
                status_o[3] <= (acc_n >= PULSE_MAX) || (acc_n <= PULSE_MIN);
            end

            pulse_o <= acc_n[20:0];
        end
    end
end

endmodule
