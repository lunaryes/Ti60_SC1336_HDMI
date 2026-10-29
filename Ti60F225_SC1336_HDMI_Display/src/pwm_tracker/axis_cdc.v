`timescale 1ns / 1ps
//=====================================================================
// axis_cdc —— 74.25 MHz（clk_pixel）到 96 MHz（clk_sys）的握手式跨时钟域
//---------------------------------------------------------------------
// 为什么需要它：
//   视觉链路全部跑在 clk_pixel 74.25 MHz（HDMI 像素时钟）；
//   PWM 必须跑在 clk_sys 96 MHz（见 gm6020_pwm_gen.v 的注释）。
//   误差数据必须跨这道边界，而且必须是"整帧一致"的，
//   不能用简单的两级触发器打拍（多 bit 同时跳变会撕裂，
//   比如 ex 从 0x1FF 变成 0x000 时可能采到 0x1F0）。
//
// 协议：四相握手（req/ack 翻转）
//   1. 源域：valid_s 有效时锁存 data_hold，翻转 req，置 pend
//   2. 目的域：2FF 同步 req，检测到翻转后采样 data_hold，输出单拍 valid_d，回翻 ack
//   3. 源域：2FF 同步 ack，看到 ack == req 后清 pend，允许下一次发送
//   data_hold 从 req 翻转起一直保持到 ack 返回（>= 2 个目的域时钟），
//   所以目的域采样点数据一定是稳定的。
//
// 带宽：一帧一次（60 Hz），握手往返约 5 个时钟（~52 ns），
//       远小于 16.7 ms 的帧间隔，永远不会丢帧。
//       若上一次握手还没完成又来新的 valid_s，本次丢弃（下一帧自然会再来）。
//=====================================================================
module axis_cdc #(
    parameter integer W = 11
)(
    // 源域：clk_pixel
    input  wire         clk_s,
    input  wire         rst_s_n,
    input  wire [W-1:0] data_s,
    input  wire         valid_s,

    // 目的域：clk_sys
    input  wire         clk_d,
    input  wire         rst_d_n,
    output reg  [W-1:0] data_d,
    output reg          valid_d      // 单拍脉冲
);

//---------------------------------------------------------------- 源域
reg [W-1:0] data_hold;
reg         req;
reg         pend;
reg [2:0]   ack_sync_s;

always @(posedge clk_s or negedge rst_s_n) begin
    if (!rst_s_n) begin
        data_hold  <= {W{1'b0}};
        req        <= 1'b0;
        pend       <= 1'b0;
        ack_sync_s <= 3'd0;
    end else begin
        ack_sync_s <= {ack_sync_s[1:0], ack};

        if (valid_s && !pend) begin
            data_hold <= data_s;
            req       <= ~req;
            pend      <= 1'b1;
        end else if (pend && (ack_sync_s[2] == req)) begin
            pend      <= 1'b0;
        end
    end
end

//---------------------------------------------------------------- 目的域
reg [2:0] req_sync_d;
reg       ack;

always @(posedge clk_d or negedge rst_d_n) begin
    if (!rst_d_n) begin
        data_d     <= {W{1'b0}};
        valid_d    <= 1'b0;
        req_sync_d <= 3'd0;
        ack        <= 1'b0;
    end else begin
        req_sync_d <= {req_sync_d[1:0], req};
        valid_d    <= 1'b0;

        if (req_sync_d[2] != ack) begin   // req 发生翻转 -> 新数据到了
            data_d  <= data_hold;         // 此时 data_hold 已稳定 >= 2 拍
            valid_d <= 1'b1;
            ack     <= req_sync_d[2];
        end
    end
end

endmodule
