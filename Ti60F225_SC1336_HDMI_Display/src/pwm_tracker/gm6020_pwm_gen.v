`timescale 1ns / 1ps
//=====================================================================
// gm6020_pwm_gen —— GM6020 模拟 PWM 物理层（发送器）
//---------------------------------------------------------------------
// GM6020 官方 PWM 规格（RoboMaster GM6020 使用说明 v1.2）：
//   载波频率   50 Hz（周期 20 ms）
//   脉宽行程   1000 us .. 2000 us（本设计默认 1000 .. 1920 us）
//   映射关系   位置模式：1000 us -> 0 deg，1920 us -> theta_max
//   输入阻抗   高阻（uA 级），3.3 V TTL 即可驱动，不需要任何收发器
//   模式识别   电机自动识别控制信号，插 PWM 就走 PWM，不改固件
//
// 为什么必须跑 clk_sys 96 MHz：
//   96 MHz / 50 Hz  = 1 920 000   （整数）
//   96 MHz * 1 us   = 96 clk      （整数）
//   于是 1000 us = 96 000 counts，1920 us = 184 320 counts，
//   脉宽误差为 0，时间分辨率 10.42 ns（角度 0.0041 deg/count @360 deg）。
//   若用 clk_pixel 74.25 MHz：74.25/50 = 1 485 000 虽然整除，
//   但 1 us = 74.25 clk 不是整数，会产生 0.34% 的脉宽误差
//   （约 0.5 deg 的指向偏差）。这和 CAN 当初那个坑是同一个来源。
//
// 输出极性：
//   GM6020 要"正脉宽"（高电平持续时间表示角度），INVERT=0 即可。
//   若中间串了反相器 / 光耦 / 三极管，把 INVERT 置 1。
//
// 安全设计：
//   pulse_i 进比较前再做一次硬限幅。哪怕上游积分器溢出、
//   或者参数配错成 PULSE_MIN > PULSE_MAX，也绝不会输出
//   会让电机撞机械限位的脉宽。
//=====================================================================
module gm6020_pwm_gen #(
    parameter integer CLK_HZ    = 96_000_000,
    parameter integer PWM_HZ    = 50,
    parameter integer PULSE_MIN = 96_000,      // 1000 us
    parameter integer PULSE_MAX = 184_320,     // 1920 us
    parameter         INVERT    = 1'b0
)(
    input  wire        clk,          // 必须 96 MHz（clk_sys）
    input  wire        rst_n,

    input  wire [20:0] pulse_i,      // 目标脉宽（以 clk 计数表示）
    output reg         pwm_o,        // -> P4 第 39 脚（GPIOT_N_11 / IO34）
    output reg         period_tick_o,// 每个 PWM 周期一个脉冲（20 ms），给状态灯用
    output wire [20:0] count_o       // 当前周期内计数（调试 / ILA）
);

localparam integer PERIOD    = CLK_HZ / PWM_HZ;   // 1 920 000
localparam [20:0]  PERIOD_M1 = PERIOD - 1;        // 1 919 999

//---------------------------------------------------------------- 硬限幅
wire [20:0] p_hi = PULSE_MAX;
wire [20:0] p_lo = PULSE_MIN;

wire [20:0] pulse_c = (pulse_i > p_hi) ? p_hi :
                      (pulse_i < p_lo) ? p_lo : pulse_i;

//---------------------------------------------------------------- 载波
reg [20:0] cnt;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cnt           <= 21'd0;
        pwm_o         <= INVERT;
        period_tick_o <= 1'b0;
    end else begin
        if (cnt == PERIOD_M1) cnt <= 21'd0;
        else                  cnt <= cnt + 21'd1;

        // 比较用"本拍"的 cnt，输出再打一拍：
        // pwm_o 的高电平长度恰好等于 pulse_c 个时钟，边沿对齐无抖动。
        pwm_o         <= INVERT ^ (cnt < pulse_c);
        period_tick_o <= (cnt == PERIOD_M1);
    end
end

assign count_o = cnt;

endmodule
