`timescale 1ns / 1ps
//=====================================================================
// sobel_edge  —— 接口 I-B
//---------------------------------------------------------------------
// 消费 3x3 灰度邻域，输出 Sobel 梯度幅值 mag_o 与边缘标志 edge_o。
//
// 【公式】（docs/INTERFACE.md 冻结）
//   Gx = (p13 + 2*p23 + p33) - (p11 + 2*p21 + p31)      右列 - 左列
//   Gy = (p31 + 2*p32 + p33) - (p11 + 2*p12 + p13)      下行 - 上行
//   mag = |Gx| + |Gy|，饱和到 8bit（> 255 截到 255）
//   edge_o = (mag_o > thr_i)，thr_i 复位默认 40
//
// 【实现要点】
//   系数 2 用左移一位（位拼接）实现，无乘法器、不占 DSP。
//   |Gx| + |Gy| 上界 = 1020 + 1020 = 2040，用 11bit 承载后饱和。
//
// 【流水线】4 级，LATENCY = 4
//   S1: 四个方向加权和 c1/c3/r1/r3
//   S2: Gx、Gy
//   S3: |Gx|、|Gy|
//   S4: mag 求和 + 饱和 + 阈值比较
//   vsync / de 用同深度移位寄存器对齐，与 mag_o / edge_o 严格同拍。
//=====================================================================
module sobel_edge #(
    parameter LATENCY = 4
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        vsync_i,
    input  wire        de_i,
    input  wire [7:0]  p11, p12, p13,
    input  wire [7:0]  p21, p22, p23,
    input  wire [7:0]  p31, p32, p33,
    input  wire [7:0]  thr_i,

    output reg         vsync_o,
    output reg         de_o,
    output reg  [7:0]  mag_o,
    output reg         edge_o
);

    //-----------------------------------------------------------------
    // S1：四个方向的加权和（系数 2 用左移一位）
    //-----------------------------------------------------------------
    wire [9:0] c1 = {1'b0, p11} + {p21, 1'b0} + {1'b0, p31};   // 左列
    wire [9:0] c3 = {1'b0, p13} + {p23, 1'b0} + {1'b0, p33};   // 右列
    wire [9:0] r1 = {1'b0, p11} + {p12, 1'b0} + {1'b0, p13};   // 上行
    wire [9:0] r3 = {1'b0, p31} + {p32, 1'b0} + {1'b0, p33};   // 下行

    reg [9:0] c1_s1, c3_s1, r1_s1, r3_s1;
    reg       vsync_s1, de_s1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c1_s1 <= 10'd0; c3_s1 <= 10'd0;
            r1_s1 <= 10'd0; r3_s1 <= 10'd0;
            vsync_s1 <= 1'b0; de_s1 <= 1'b0;
        end else begin
            c1_s1 <= c1; c3_s1 <= c3;
            r1_s1 <= r1; r3_s1 <= r3;
            vsync_s1 <= vsync_i; de_s1 <= de_i;
        end
    end

    //-----------------------------------------------------------------
    // S2：Gx、Gy（有符号，范围 -1020 .. +1020）
    //-----------------------------------------------------------------
    reg signed [10:0] gx_s2, gy_s2;
    reg               vsync_s2, de_s2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gx_s2 <= 11'sd0; gy_s2 <= 11'sd0;
            vsync_s2 <= 1'b0; de_s2 <= 1'b0;
        end else begin
            gx_s2 <= $signed({1'b0, c3_s1}) - $signed({1'b0, c1_s1});
            gy_s2 <= $signed({1'b0, r3_s1}) - $signed({1'b0, r1_s1});
            vsync_s2 <= vsync_s1; de_s2 <= de_s1;
        end
    end

    //-----------------------------------------------------------------
    // S3：取绝对值
    //-----------------------------------------------------------------
    function [10:0] abs11;
        input signed [10:0] v;
        begin
            abs11 = v[10] ? (~v + 11'd1) : v;
        end
    endfunction

    reg [10:0] ax_s3, ay_s3;
    reg        vsync_s3, de_s3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ax_s3 <= 11'd0; ay_s3 <= 11'd0;
            vsync_s3 <= 1'b0; de_s3 <= 1'b0;
        end else begin
            ax_s3 <= abs11(gx_s2);
            ay_s3 <= abs11(gy_s2);
            vsync_s3 <= vsync_s2; de_s3 <= de_s2;
        end
    end

    //-----------------------------------------------------------------
    // S4：求和 + 饱和 + 阈值比较
    //-----------------------------------------------------------------
    wire [11:0] mag_sum = {1'b0, ax_s3} + {1'b0, ay_s3};       // 0 .. 2040
    wire [7:0]  mag_sat = (mag_sum > 12'd255) ? 8'hFF : mag_sum[7:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_o <= 1'b0;
            de_o    <= 1'b0;
            mag_o   <= 8'd0;
            edge_o  <= 1'b0;
        end else begin
            vsync_o <= vsync_s3;
            de_o    <= de_s3;
            mag_o   <= de_s3 ? mag_sat : 8'd0;
            edge_o  <= de_s3 & (mag_sat > thr_i);
        end
    end

endmodule
