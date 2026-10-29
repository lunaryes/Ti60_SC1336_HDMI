`timescale 1ns / 1ps
//=====================================================================
// can_tx —— CAN 2.0A 标准帧发送器（11 bit ID / 数据帧 / 8 字节）
//---------------------------------------------------------------------
// 位定时：CLK_HZ/BAUD 必须整除。本项目 96MHz / 1Mbps = 96，零误差。
//         仿真时可把两者同比例缩小以加快速度，逻辑不变。
//
// 帧结构（逻辑位序号 fpos）：
//   0        SOF                      (dominant)
//   1..11    ID[10:0]                 (MSB first)
//   12       RTR   = 0                (数据帧)
//   13       IDE   = 0                (标准帧)
//   14       r0    = 0
//   15..18   DLC   = 4'h8             (8 字节)
//   19..82   DATA[0..7]               (MSB first，data_i[63:0])
//   83..97   CRC[14:0]                (MSB first)
//   之后不分段填充：CRC 界定符(1) + ACK(1) + ACK 界定符(1) + EOF(7) + IFS(3)
//
// 位填充：SOF..CRC 序列（fpos 0..97）范围内，连续 5 个相同位后插入 1 个反相位。
//         填充位不推进 fpos、不计入 CRC。
//
// CRC-15：poly = 0x4599（x^15+x^14+x^10+x^8+x^7+x^4+x^3+1，省去 x^15 项）
//         初值 0，不反射，无末尾异或。已用多项式长除法独立校验。
//=====================================================================
module can_tx #(
    parameter integer CLK_HZ = 96_000_000,
    parameter integer BAUD   = 1_000_000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        req_i,    // 单拍脉冲：发起一帧
    input  wire [10:0] id_i,     // 11 bit 标准帧 ID
    input  wire [63:0] data_i,   // data_i[63:56]=DATA[0] ... data_i[7:0]=DATA[7]
    input  wire        rx_i,     // 回读总线（收发器 R 脚）：用于 ACK 槽监测
    output reg         tx_o,     // 接收发器 D 脚
    output reg         busy_o,
    output reg         done_o,   // 一帧发完单拍脉冲
    output reg         ack_err_o // ACK 槽总线仍为隐性 = 无人应答（含自回环 ACK）
);

//---------------------------------------------------------------- bit timing
localparam integer BIT_CLKS = (CLK_HZ + (BAUD/2)) / BAUD;
localparam integer PW = (BIT_CLKS<=2)  ? 1 :
                        (BIT_CLKS<=4)  ? 2 :
                        (BIT_CLKS<=8)  ? 3 :
                        (BIT_CLKS<=16) ? 4 :
                        (BIT_CLKS<=32) ? 5 :
                        (BIT_CLKS<=64) ? 6 : 7;
localparam integer HALF = BIT_CLKS/2;
// ACK 槽采样点：该位 3/4 处（发送方 ACK 槽 = tail_cnt==1）。
// 应答方（GM6020 或本机 can_rx 的 ack_drive 自回环）最晚在槽位开始后
// 约半个位时间内拉低总线；3/4 处采样最稳妥。
localparam integer ACK_SMP = (BIT_CLKS*3)/4;

//---------------------------------------------------------------- states
localparam [1:0] ST_IDLE  = 2'd0,
                 ST_STUFF = 2'd1,
                 ST_TAIL  = 2'd2;

reg [1:0]    st;
reg [PW-1:0] ph;
reg [6:0]    fpos;         // 0..97 逻辑位序号
reg [3:0]    stuff_cnt;    // 当前连续相同位数，0..5
reg        last_bit;
reg [14:0] crc;
reg [4:0]  tail_cnt;       // 尾段 13 个 recessive 位
reg [10:0] id_sh;
reg [63:0] d_sh;
reg        nb;             // 下一个逻辑位

//---------------------------------------------------------------- CRC-15
function [14:0] crc_step;
    input [14:0] c;
    input        b;
    reg          x;
    begin
        x        = c[14] ^ b;
        crc_step = {c[13:0], 1'b0};
        if (x) crc_step = crc_step ^ 15'h4599;
    end
endfunction

//---------------------------------------------------------------- 取 fpos 处的逻辑位
function bit_at;
    input [6:0]  p;
    input [10:0] id;
    input [63:0] d;
    input [14:0] c;
    begin
        if      (p == 7'd0)  bit_at = 1'b0;            // SOF
        else if (p <= 7'd11) bit_at = id[7'd11 - p];   // ID[10]..ID[0]
        else if (p <= 7'd14) bit_at = 1'b0;            // RTR/IDE/r0
        else if (p <= 7'd18) bit_at = (p == 7'd15);    // DLC = 8
        else if (p <= 7'd82) bit_at = d[7'd82 - p];    // DATA MSB first
        else                 bit_at = c[7'd97 - p];    // CRC MSB first
    end
endfunction

//---------------------------------------------------------------- main
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        st <= ST_IDLE; ph <= 0; fpos <= 7'd0; stuff_cnt <= 4'd0;
        last_bit <= 1'b1; crc <= 15'd0; tail_cnt <= 5'd0;
        id_sh <= 11'd0; d_sh <= 64'd0; nb <= 1'b1;
        tx_o <= 1'b1; busy_o <= 1'b0; done_o <= 1'b0; ack_err_o <= 1'b0;
    end else begin
        done_o    <= 1'b0;
        ack_err_o <= 1'b0;

        case (st)
        //---------------------------------------------------- 等发起
        ST_IDLE: begin
            ph <= 0;
            if (req_i) begin
                id_sh     <= id_i;
                d_sh      <= data_i;
                fpos      <= 7'd1;                  // SOF 已发出
                crc       <= crc_step(15'd0, 1'b0); // SOF 计入 CRC
                last_bit  <= 1'b0;
                stuff_cnt <= 4'd1;
                tx_o      <= 1'b0;                  // SOF
                busy_o    <= 1'b1;
                st        <= ST_STUFF;
            end
        end

        //---------------------------------------------------- 填充区
        ST_STUFF: begin
            if (ph == BIT_CLKS-1) begin
                ph <= 0;
                if (stuff_cnt == 4'd5) begin
                    // 插入填充位：值与上一个相反，不推进 fpos
                    tx_o      <= ~last_bit;
                    last_bit  <= ~last_bit;
                    stuff_cnt <= 4'd1;
                end else if (fpos < 7'd98) begin
                    nb <= bit_at(fpos, id_sh, d_sh, crc);
                    tx_o      <= bit_at(fpos, id_sh, d_sh, crc);
                    stuff_cnt <= (bit_at(fpos, id_sh, d_sh, crc) == last_bit)
                                 ? (stuff_cnt + 1'b1) : 4'd1;
                    last_bit  <= bit_at(fpos, id_sh, d_sh, crc);
                    if (fpos <= 7'd82)
                        crc <= crc_step(crc, bit_at(fpos, id_sh, d_sh, crc));
                    fpos <= fpos + 1'b1;
                end else begin
                    // 填充区结束，转尾段（全 recessive）
                    tx_o     <= 1'b1;
                    st       <= ST_TAIL;
                    tail_cnt <= 5'd0;
                end
            end else begin
                ph <= ph + 1'b1;
            end
        end

        //---------------------------------------------------- 尾段 13 位
        // tail_cnt: 0=CRC界定符 1=ACK槽 2=ACK界定符 3..9=EOF 10..12=IFS
        ST_TAIL: begin
            if (ph == BIT_CLKS-1) begin
                ph <= 0;
                if (tail_cnt == 5'd12) begin
                    tx_o   <= 1'b1;
                    busy_o <= 1'b0;
                    done_o <= 1'b1;
                    st     <= ST_IDLE;
                end else begin
                    tail_cnt <= tail_cnt + 1'b1;
                end
            end else begin
                // ---- ACK 槽监测（★参照 FPGA-CAN 的完备性补齐）
                // 我们在 ACK 槽发隐性，正常情况下应答方把它压成显性，
                // 经收发器回读 rx_i 应为 0。若在 3/4 位处仍是 1：
                //   = 帧没有到达任何应答方（收发器损坏/待机/没接上/引脚错）
                //   = 或总线上没有任何 CAN 节点
                if (tail_cnt == 5'd1 && ph == ACK_SMP && rx_i)
                    ack_err_o <= 1'b1;
                ph <= ph + 1'b1;
            end
        end

        default: st <= ST_IDLE;
        endcase
    end
end

endmodule
