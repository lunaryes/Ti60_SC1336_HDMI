`timescale 1ns / 1ps
//=====================================================================
// gm6020_pwm_top —— GM6020 三角形视觉跟随（PWM 通道）顶层
//---------------------------------------------------------------------
// 数据流：
//
//  clk_pixel 74.25 MHz                      clk_sys 96 MHz
//  ┌────────────────────────────────┐      ┌──────────────────────────────┐
//  │ tri_target_extract             │      │ follow_ctrl                  │
//  │  objects_i(8x36) + valid_i     │      │  增量式积分 + 死区 + 限幅     │
//  │      ↓ 选 class==1 且最近中心   │ CDC  │      ↓ pulse(counts)          │
//  │  ex(半格), found, frame_tick ──┼─11b─→│ gm6020_pwm_gen               │
//  └────────────────────────────────┘ 握手 │  50 Hz / 1000-1920 us ──→ P4-39│
//                                          └──────────────────────────────┘
//
// 上游接法（example_top.v:1404 已经存在的信号，一行都不用改）：
//      .objects_i (geometry_objects)   // [287:0] = 8 x 36 bit
//      .valid_i   (geometry_valid)     // [7:0]
//      .vsync_i   (shape_vs)           // 与 objects 同域，是识别流水线的 vsync
//
// 引脚：
//      pwm_o -> example_top 的 can_tx_o 端口 -> GPIOT_N_11 = P4 第 39 脚
//      GND   -> P4 第 30 脚
//      电机的 PWM 线 A（白）接 39，C（黑）接 30，B（灰）悬空。
//      P4 第 11 脚是 5 V，绝对不能碰。
//
// 状态灯 status_o（高有效，example_top 里接 LED 要取反，板载 LED 低电平点亮）：
//      [0] 本帧找到三角形
//      [1] 误差已进入死区（画面中心附近，稳定）
//      [2] 目标丢失，正在缓慢回中
//      [3] 脉宽已顶到行程端点（该查方向/增益/机械限位了）
//=====================================================================
module gm6020_pwm_top #(
    //------------------------------------------------------------------------ 视觉
    parameter integer MAX_OBJECTS = 8,
    parameter integer GRID_W      = 160,
    parameter integer GRID_H      = 90,
    parameter [3:0]   TRI_CLASS   = 4'd1,

    //------------------------------------------------------------------------ 电机行程
    // PULSE_MIN / PULSE_MAX 由 RoboMaster Assistant 里标定的行程决定：
    //   1000 us -> 0 deg，1920 us -> theta_max
    // 若只用到 ±90 deg，把 PULSE_MAX 改小（例如 140160+22080 = 162240），
    // 让同样的误差对应更小的转角，跟随会更稳。
    parameter integer PULSE_MIN   = 96_000,    // 1000 us
    parameter integer PULSE_MAX   = 184_320,   // 1920 us
    parameter integer PULSE_MID   = 140_160,   // 1460 us

    //------------------------------------------------------------------------ 控制律
    parameter integer GAIN_NUM    = 44,        // 见 follow_ctrl.v 顶部公式
    parameter integer GAIN_SHIFT  = 2,         // 1/4 增益
    parameter integer DEADZONE    = 3,         // 半格
    parameter integer MAX_STEP    = 3000,      // counts/帧
    parameter integer DIR_INVERT  = 0,
    parameter integer LOST_HOLD   = 30,
    parameter integer RETURN_STEP = 200,
    parameter integer SEEK_STEP   = 400,
    parameter integer SEEK_DELAY  = 30,
    parameter integer SCAN_STEP   = 800,
    parameter         SELF_TEST   = 0,
    parameter         CAL_MODE    = 0,         // 1 = 上电先做行程标定（见下）

    //--------------------------------------------------------------- 目标门限（阶段0 抗噪）
    parameter integer MIN_SIZE    = 2,         // 最小边（格）
    parameter integer MAX_SIZE    = 48,        // 最大边（格）
    parameter integer MIN_AREA    = 9,         // 最小面积（格^2）

    //------------------------------------------------------------------------ 恒速转
    // HOLD_PULSE != 0 时完全绕过视觉和自检，直接输出一个固定脉宽。
    // 是否"转"取决于电机当前的控制模式：
    //   速度模式：1500 us = 0 转速，>1500 正转，<1500 反转
    //             约 0.76 rpm / us（1920 us = +320 rpm，1000 us = -320 rpm）
    //   位置模式：脉宽 = 角度，恒定脉宽只会停在那个角度，不会持续转
    // 所以这一档同时也是一次模式探测：转 = 速度模式；停下 = 位置模式。
    parameter integer HOLD_PULSE  = 0,

    //------------------------------------------------------------------------ 慢速爬行档
    // HOLD_PULSE 是"固定脉宽"：位置模式下只会停在某个角度，不会持续转。
    // CREEP_MODE 让脉宽在 [CREEP_MIN, CREEP_MAX] 之间缓慢往返，两种模式都能看到连续转动：
    //   位置模式 -> 轴持续缓慢爬行（1400~1600us 约 78 deg 行程，约 10 deg/s）
    //   速度模式 -> 转速在 ±(偏离1500us量) 之间缓慢摆动（约 ±76 rpm）
    // 这是"让电机转起来"的安全验证档，不依赖任何视觉输入。
    parameter integer CREEP_MODE  = 0,
    parameter integer CREEP_MIN   = 134_400,   // 1400 us
    parameter integer CREEP_MAX   = 153_600,   // 1600 us
    parameter integer CREEP_STEP  = 48,        // counts / 20 ms -> 25 us/s

    //------------------------------------------------------------------------ 时钟
    parameter integer CLK_HZ      = 96_000_000,
    parameter integer PWM_HZ      = 50
)(
    // 视觉域
    input  wire        clk_pixel,
    input  wire        rst_pixel_n,
    input  wire        vsync_i,
    input  wire [MAX_OBJECTS*36-1:0] objects_i,
    input  wire [MAX_OBJECTS-1:0]    valid_i,

    // 电机域
    input  wire        clk_sys,
    input  wire        rst_sys_n,

    // 输出
    output wire        pwm_o,
    output wire [3:0]  status_o,

    // 观测（可选，接 ILA 或直接不接）
    output wire signed [9:0] ex_o,
    output wire              found_o,
    output wire [20:0]       pulse_o
);

//===================================================================== 1. 视觉域：取三角形
wire signed [9:0] ex_pix;
wire signed [9:0] ey_pix;
wire              found_pix;
wire              frame_pix;
wire [5:0]        obj_w, obj_h;

tri_target_extract #(
    .MAX_OBJECTS (MAX_OBJECTS),
    .GRID_W      (GRID_W     ),
    .GRID_H      (GRID_H     ),
    .TRI_CLASS   (TRI_CLASS  ),
    .MIN_SIZE    (MIN_SIZE   ),
    .MAX_SIZE    (MAX_SIZE   ),
    .MIN_AREA    (MIN_AREA   )
) u_tri_extract (
    .clk        (clk_pixel    ),
    .rst_n      (rst_pixel_n  ),
    .vsync_i    (vsync_i      ),
    .objects_i  (objects_i    ),
    .valid_i    (valid_i      ),
    .ex_o       (ex_pix       ),
    .ey_o       (ey_pix       ),
    .found_o    (found_pix    ),
    .size_w_o   (obj_w        ),
    .size_h_o   (obj_h        ),
    .frame_o    (frame_pix    )
);

//===================================================================== 2. 跨时钟域
// 打包 {found, ex} = 11 bit。ex 用补码，跨域后原样还原即可。
wire [10:0] cdc_data_s = {found_pix, ex_pix};
wire [10:0] cdc_data_d;
wire        cdc_valid_d;

axis_cdc #(.W(11)) u_cdc (
    .clk_s   (clk_pixel    ),
    .rst_s_n (rst_pixel_n  ),
    .data_s  (cdc_data_s   ),
    .valid_s (frame_pix    ),

    .clk_d   (clk_sys      ),
    .rst_d_n (rst_sys_n    ),
    .data_d  (cdc_data_d   ),
    .valid_d (cdc_valid_d  )
);

wire signed [9:0] ex_sys = cdc_data_d[9:0];
wire              found_sys = cdc_data_d[10];

//===================================================================== 3. 电机域：控制律
wire [20:0] pulse_c;

//---------------------------------------------------------------- 控制节拍
// 正常模式：tick 来自视觉帧（axis_cdc 的 valid_d），一帧更新一次。
// SELF_TEST：绝对不能依赖相机。相机没出图 -> vsync 不来 -> tick 不跳 ->
//   脉宽永远停在 PULSE_MID，现象是"电机一动不动"，很容易被误判成接线错。
//   所以自检时直接用 PWM 周期本身当节拍（50 Hz，20 ms 一次）。
wire pwm_period_tick;
wire ctrl_tick = SELF_TEST ? pwm_period_tick : cdc_valid_d;

wire [3:0] status_follow;   // follow_ctrl 的状态；恒速档不用它

follow_ctrl #(
    .PULSE_MIN   (PULSE_MIN  ),
    .PULSE_MAX   (PULSE_MAX  ),
    .PULSE_MID   (PULSE_MID  ),
    .GAIN_NUM    (GAIN_NUM   ),
    .GAIN_SHIFT  (GAIN_SHIFT ),
    .DEADZONE    (DEADZONE   ),
    .MAX_STEP    (MAX_STEP   ),
    .DIR_INVERT  (DIR_INVERT ),
    .LOST_HOLD   (LOST_HOLD  ),
    .RETURN_STEP (RETURN_STEP),
    .SEEK_STEP   (SEEK_STEP  ),
    .SEEK_DELAY  (SEEK_DELAY ),
    .SCAN_STEP   (SCAN_STEP  ),
    .SELF_TEST   (SELF_TEST  )
) u_follow (
    .clk      (clk_sys    ),
    .rst_n    (rst_sys_n  ),
    .ex_i     (ex_sys     ),
    .found_i  (found_sys  ),
    .tick_i   (ctrl_tick  ),
    .pulse_o  (pulse_c    ),
    .status_o (status_follow)
);

//===================================================================== 4. 首次上电行程标定（可选）
// GM6020 在 PWM 位置模式下必须做一次行程标定，否则 1000 us 不对应 0 deg、
// 1920 us 也不对应 theta_max，整套角度映射都是错的。
// 官方流程（RoboMaster Assistant 里也能手动点）：
//   把脉宽推到 1900~2100 us -> 绿灯常亮 3 s -> 再拉到 900~1100 us -> 绿灯慢闪
//
// CAL_MODE=1 时本模块上电自动跑这个序列，全程约 7 s，之后才交给追随逻辑。
// 标定结束的标志是电机绿灯由常亮转为慢闪，肉眼确认一次即可。
// 做过一次之后（电机掉电不丢标定）就把 CAL_MODE 改回 0。
localparam [20:0] CAL_PULSE_HI = 192_000;   // 2000 us
localparam [20:0] CAL_PULSE_LO = 96_000;    // 1000 us
localparam [31:0] CAL_MS_CYC   = CLK_HZ / 1000;
localparam [31:0] CAL_HI_CYC   = 3_000 * CAL_MS_CYC;    // 顶点保持 3 s
localparam [31:0] CAL_LO_CYC   = 4_000 * CAL_MS_CYC;    // 底点保持 4 s
localparam [31:0] CAL_END_CYC  = CAL_HI_CYC + CAL_LO_CYC;

wire [20:0] pulse_ctrl;

generate
if (CAL_MODE) begin : g_cal
    reg [31:0] cal_t;
    always @(posedge clk_sys or negedge rst_sys_n) begin
        if (!rst_sys_n)                 cal_t <= 32'd0;
        else if (cal_t < CAL_END_CYC)   cal_t <= cal_t + 32'd1;
    end

    wire cal_done = (cal_t >= CAL_END_CYC);
    wire cal_high = (cal_t <  CAL_HI_CYC);

    assign pulse_ctrl = cal_done ? pulse_c
                                 : (cal_high ? CAL_PULSE_HI : CAL_PULSE_LO);
end else begin : g_nocal
    assign pulse_ctrl = pulse_c;
end
endgenerate

// 恒速档 / 爬行档：优先级高于上面所有逻辑
localparam [20:0] HOLD_C   = HOLD_PULSE;
localparam [20:0] CREEP_LO = CREEP_MIN;
localparam [20:0] CREEP_HI = CREEP_MAX;
localparam [20:0] CREEP_S  = CREEP_STEP;

reg [20:0] creep_v;
reg        creep_dir;

always @(posedge clk_sys or negedge rst_sys_n) begin
    if (!rst_sys_n) begin
        creep_v   <= CREEP_LO;
        creep_dir <= 1'b1;
    end else if (CREEP_MODE && pwm_period_tick) begin
        if (creep_dir) begin
            if (creep_v + CREEP_S >= CREEP_HI) begin
                creep_v   <= CREEP_HI;
                creep_dir <= 1'b0;
            end else begin
                creep_v <= creep_v + CREEP_S;
            end
        end else begin
            if (creep_v <= CREEP_LO + CREEP_S) begin
                creep_v   <= CREEP_LO;
                creep_dir <= 1'b1;
            end else begin
                creep_v <= creep_v - CREEP_S;
            end
        end
    end
end

wire [20:0] pulse_final = (HOLD_PULSE != 0) ? HOLD_C   :
                          (CREEP_MODE != 0) ? creep_v  : pulse_ctrl;

// 状态灯：恒速/爬行档给固定图样，否则用追随状态
assign status_o = (HOLD_PULSE != 0) ? 4'b0100 :
                  (CREEP_MODE != 0) ? 4'b0010 : status_follow;

//===================================================================== 5. 电机域：PWM 物理层
// 标定时脉宽会顶到 2000 us，超过正常工作的 PULSE_MAX，
// 所以物理层的硬限幅要按标定范围放宽，否则 2000 us 会被削成 1920 us。
localparam [20:0] HARD_MIN = 96_000;
localparam integer HARD_MAX_I = (CAL_MODE != 0)      ? 192_000 :
                                (HOLD_PULSE > PULSE_MAX) ? HOLD_PULSE : PULSE_MAX;
localparam [20:0] HARD_MAX = HARD_MAX_I;

gm6020_pwm_gen #(
    .CLK_HZ    (CLK_HZ   ),
    .PWM_HZ    (PWM_HZ   ),
    .PULSE_MIN (HARD_MIN ),
    .PULSE_MAX (HARD_MAX )
) u_pwm_gen (
    .clk           (clk_sys   ),
    .rst_n         (rst_sys_n ),
    .pulse_i       (pulse_final),
    .pwm_o         (pwm_o     ),
    .period_tick_o (pwm_period_tick),
    .count_o       ()
);

assign ex_o    = ex_pix;
assign found_o = found_pix;
assign pulse_o = pulse_final;

endmodule
