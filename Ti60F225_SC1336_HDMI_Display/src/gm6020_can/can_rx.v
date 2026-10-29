`timescale 1ns / 1ps
//=====================================================================
// can_rx —— CAN 2.0A 标准帧接收器（硬同步 + 位去填充 + CRC-15 校验）
//---------------------------------------------------------------------
// 同步策略：仅在 SOF 下降沿做硬同步，之后按标称位定时在每一位中心采样
//   （第 1 位在 0.5 个位时间后采，之后每 1 个位时间采一次）。
//   不做逐位重同步。前提：CLK_HZ/BAUD 为整数（96MHz/1Mbps=96，零误差），
//   且对方时钟偏差 < 0.5%。GM6020 内部为晶振，短帧 108 位累积漂移 < 0.1 位。
//   将来接长帧/多节点高速总线，需要补逐位重同步。
//
// 解析：SOF(0) ID(1..11) RTR(12) IDE(13) r0(14) DLC(15..18)
//       DATA(19..82) CRC(83..97) 界定符/ACK(98..100) EOF(101..107)
// 只接受 ID == ID_FILTER 的 8 字节数据帧。
//=====================================================================
module can_rx #(
    parameter integer CLK_HZ    = 96_000_000,
    parameter integer BAUD      = 1_000_000,
    parameter [10:0]  ID_FILTER = 11'h205,
    parameter integer FILTER_EN = 1        // 0 = 不做 ID 过滤，全部上报
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rx_i,
    output reg  [10:0] id_o,
    output reg  [63:0] data_o,     // data_o[63:56]=DATA[0] ... data_o[7:0]=DATA[7]
    output reg         valid_o,    // 收到一帧合法且 ID 匹配：单拍脉冲
    output reg         crc_err_o,   // CRC 或格式错：单拍脉冲
    // ACK 驱动：接收方必须在 ACK 槽拉低总线，否则发送方会不断重发直至报错。
    // 顶层用法：can_tx_pin = tx_bit & ~ack_drive;（线与，显性优先）
    output reg         ack_drive
);

localparam integer BIT_CLKS = (CLK_HZ + (BAUD/2)) / BAUD;
localparam integer PW = (BIT_CLKS<=2)  ? 1 :
                        (BIT_CLKS<=4)  ? 2 :
                        (BIT_CLKS<=8)  ? 3 :
                        (BIT_CLKS<=16) ? 4 :
                        (BIT_CLKS<=32) ? 5 :
                        (BIT_CLKS<=64) ? 6 : 7;
localparam integer HALF = BIT_CLKS/2;

localparam [1:0] ST_IDLE = 2'd0,
                 ST_RECV = 2'd1;

reg [1:0]    st;
reg [PW-1:0] ph;
reg [7:0]    idx;
reg [10:0]   id_sr;
reg [63:0]   d_sr;
reg [14:0]   crc_calc;
reg [14:0]   crc_rx;
reg [3:0]    run;
reg        last_raw;
reg        destuff_en;
reg        crc_ok;
reg        fmt_ok;
reg        rx_d;

wire rx_fall = rx_d & ~rx_i;
// 组合判断：当前采样点是否为位填充位（连续 5 个相同位之后的那一位）
wire is_stuff = destuff_en && (run == 4'd5);
// 采样点（位中心）
wire smp = (st == ST_RECV) && (ph == 0);

//---------------------------------------------------------------- ACK 驱动定时
// 时机：采到 CRC 界定符（idx=98）且 CRC 正确 -> 在"CRC 界定符/ACK"交界处
//       拉低，持续一个位时间，正好覆盖 ACK 槽。
//       不能在采样点立刻拉低：那样会把 CRC 界定符的后半位压成显性，
//       对方在 0.75 位处采样会判成格式错。
localparam [8:0] ACK_ON  = HALF;
localparam [8:0] ACK_OFF = HALF + BIT_CLKS;
reg [8:0] ack_t;
reg       ack_run;

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

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        st <= ST_IDLE; ph <= 0; idx <= 8'd0;
        id_sr <= 11'd0; d_sr <= 64'd0;
        crc_calc <= 15'd0; crc_rx <= 15'd0;
        run <= 4'd0; last_raw <= 1'b1; destuff_en <= 1'b0;
        crc_ok <= 1'b0; fmt_ok <= 1'b0;
        id_o <= 11'd0; data_o <= 64'd0;
        valid_o <= 1'b0; crc_err_o <= 1'b0; rx_d <= 1'b1;
        ack_t <= 9'd0; ack_run <= 1'b0; ack_drive <= 1'b0;
    end else begin
        valid_o   <= 1'b0;
        crc_err_o <= 1'b0;
        rx_d      <= rx_i;

        //-------------------------------------------- ACK 槽驱动
        if (smp && !is_stuff && (idx == 8'd98) && crc_ok) begin
            ack_run <= 1'b1;
            ack_t   <= 9'd0;
        end else if (ack_run) begin
            if (ack_t >= ACK_OFF) ack_run <= 1'b0;
            else                  ack_t   <= ack_t + 1'b1;
        end
        ack_drive <= ack_run && (ack_t >= ACK_ON) && (ack_t < ACK_OFF);

        case (st)
        //---------------------------------------------------- 等 SOF
        ST_IDLE: begin
            ph <= 0;
            if (rx_fall) begin
                ph         <= HALF - 1;
                idx        <= 8'd0;
                run        <= 4'd0;
                last_raw   <= 1'b1;
                destuff_en <= 1'b1;
                crc_calc   <= 15'd0;
                crc_rx     <= 15'd0;
                id_sr      <= 11'd0;
                d_sr       <= 64'd0;
                crc_ok     <= 1'b0;
                fmt_ok     <= 1'b1;     // 进入时乐观置位，出错再清零
                st         <= ST_RECV;
            end
        end

        //---------------------------------------------------- 收位
        ST_RECV: begin
            if (ph == 0) begin
                ph       <= BIT_CLKS - 1;
                last_raw <= rx_i;

                if (is_stuff) begin
                    // ---- 填充位：丢弃，不推进 idx，不参与 CRC
                    // ★ 游程必须置 1 而不是 0：填充位本身是"已发送的一位"，
                    //   它开启了长度为 1 的新游程；置 0 会与发送端差 1，
                    //   导致后续填充位整体错位（CRC 表现为右移一位）。
                    run <= 4'd1;
`ifdef CAN_DBG
                    $display("  STUFF  idx=%0d run=%0d t=%0t", idx, run, $time);
`endif
                end else begin
                    run <= (rx_i == last_raw) ? (run + 1'b1) : 4'd1;
`ifdef CAN_DBG
                    $display("A %0d %0d", idx, rx_i);
`endif

                    // ---- 字段路由
                    if (idx == 8'd0) begin
                        if (rx_i !== 1'b0) fmt_ok <= 1'b0;        // SOF 必须 dominant
                    end else if (idx <= 8'd11) begin
                        id_sr <= {id_sr[9:0], rx_i};
                    end else if (idx <= 8'd18) begin
                        // DLC/RTR/IDE/r0：只按 8 字节帧解析，不做长度分支
                    end else if (idx <= 8'd82) begin
                        d_sr <= {d_sr[62:0], rx_i};
                    end else if (idx <= 8'd97) begin
                        crc_rx <= {crc_rx[13:0], rx_i};
                    end else if (idx == 8'd98) begin
                        if (rx_i !== 1'b1) fmt_ok <= 1'b0;        // CRC 界定符
                        // ★ 去填充必须留到 CRC 界定符这一位才关：CRC 序列的最后
                        //   一位之后同样允许出现填充位，提前关会把填充位误当界定符。
                        destuff_en <= 1'b0;
                    end else if (idx == 8'd99) begin
`ifdef CAN_DBG
                        $display("  ACK rx_i=%0b ackdrive=%0b ackrun=%0d ackt=%0d t=%0t",
                                 rx_i, ack_drive, ack_run, ack_t, $time);
`endif
                        if (rx_i !== 1'b0) fmt_ok <= 1'b0;        // ACK 槽（对方拉低）
                    end else if (idx == 8'd100) begin
                        if (rx_i !== 1'b1) fmt_ok <= 1'b0;        // ACK 界定符
                    end

                    if (idx <= 8'd82)
                        crc_calc <= crc_step(crc_calc, rx_i);

                    if (idx == 8'd97) begin
                        crc_ok     <= (crc_calc == {crc_rx[13:0], rx_i});
`ifdef CAN_DBG
                        $display("  CRC calc=0x%04X rx=0x%04X ok=%0d t=%0t",
                                 crc_calc, {crc_rx[13:0], rx_i},
                                 (crc_calc == {crc_rx[13:0], rx_i}), $time);
`endif
                    end

                    // ---- 一帧收完（EOF 最后一位）
                    if (idx == 8'd107) begin
`ifdef CAN_DBG
                        $display("  END crc_ok=%0d fmt_ok=%0d id=0x%03X t=%0t",
                                 crc_ok, fmt_ok, id_sr, $time);
`endif
                        if (crc_ok && fmt_ok &&
                            ((FILTER_EN == 0) || (id_sr == ID_FILTER))) begin
                            id_o    <= id_sr;
                            data_o  <= d_sr;
                            valid_o <= 1'b1;
                        end else if (!crc_ok) begin
                            crc_err_o <= 1'b1;
                        end
                        st <= ST_IDLE;
                    end else begin
                        idx <= idx + 1'b1;
                    end
                end
            end else begin
                ph <= ph - 1'b1;
            end
        end

        default: st <= ST_IDLE;
        endcase
    end
end

endmodule
