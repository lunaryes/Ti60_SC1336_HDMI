`timescale 1ns / 1ps
//=====================================================================
// uart_tx —— 极简 8N1 发送器（M6，片内直方图回传用）
//---------------------------------------------------------------------
// 与 uart_cmd 内部那段 TX 状态机**逻辑同构**，但抽成独立模块：
//   · uart_cmd 的协议/回包已经冻结，不去动它的 TX；
//   · 直方图要连续发 778 字节，需要 start/done 握手，不能复用命令口的定长回包。
//
// 位序：起始位 0，8 个数据位 LSB 先到，停止位 1。空闲电平为高。
// 分频算法与 uart_cmd 完全一致（四舍五入），这样同一个 BAUD 参数在两边
// 得到同一个 DIV，不会出现"回读正常、直方图乱码"的波特率漂移。
//
// 握手：
//   start_i 单拍脉冲 + data_i 同拍有效 → busy_o 拉高，tx_o 开始发；
//   停止位结束的那一拍 done_o 出单拍脉冲，busy_o 同时落回 0。
//   busy_o=1 时 start_i 被忽略，调用方自己保证不抢。
//=====================================================================
module uart_tx #(
    parameter integer CLK_FREQ = 74250000,   // clk_pixel
    parameter integer BAUD     = 460800
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start_i,
    input  wire [7:0] data_i,
    output reg        busy_o,
    output reg        tx_o,
    output reg        done_o
);

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

    // 与 uart_cmd 相同的写法：先取 CW 位部分选择再运算，避免 32 位表达式
    // 带来的 WIDTHTRUNC 警告。
    localparam [CW-1:0] CNT_BIT = DIV[CW-1:0] - 1'b1;

    localparam [1:0] U_IDLE = 2'd0, U_START = 2'd1, U_DATA = 2'd2, U_STOP = 2'd3;

    reg [1:0]    state;
    reg [CW-1:0] cnt;
    reg [2:0]    bidx;
    reg [7:0]    sh;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state  <= U_IDLE;
            cnt    <= {CW{1'b0}};
            bidx   <= 3'd0;
            sh     <= 8'd0;
            busy_o <= 1'b0;
            tx_o   <= 1'b1;             // 空闲 = 高
            done_o <= 1'b0;
        end else begin
            done_o <= 1'b0;             // 单拍脉冲
            case (state)
                U_IDLE: begin
                    tx_o <= 1'b1;
                    if (start_i) begin
                        sh     <= data_i;
                        busy_o <= 1'b1;
                        tx_o   <= 1'b0; // 起始位
                        cnt    <= CNT_BIT;
                        state  <= U_START;
                    end
                end
                U_START: begin
                    if (cnt == 0) begin
                        cnt   <= CNT_BIT;
                        bidx  <= 3'd0;
                        tx_o  <= sh[0]; // bit0
                        state <= U_DATA;
                    end else cnt <= cnt - 1'b1;
                end
                U_DATA: begin
                    if (cnt == 0) begin
                        sh  <= {1'b0, sh[7:1]};
                        cnt <= CNT_BIT;
                        if (bidx == 3'd7) begin
                            tx_o  <= 1'b1;          // 停止位
                            state <= U_STOP;
                        end else begin
                            bidx  <= bidx + 1'b1;
                            tx_o  <= sh[1];         // 移位后的下一位
                        end
                    end else cnt <= cnt - 1'b1;
                end
                U_STOP: begin
                    tx_o <= 1'b1;
                    if (cnt == 0) begin
                        busy_o <= 1'b0;
                        done_o <= 1'b1;
                        state  <= U_IDLE;
                    end else cnt <= cnt - 1'b1;
                end
                default: state <= U_IDLE;
            endcase
        end
    end

endmodule
