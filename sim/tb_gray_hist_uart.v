`timescale 1ns / 1ps
//=====================================================================
// tb_gray_hist_uart —— 片内灰度直方图 + 串口回传的回归（M6）
//---------------------------------------------------------------------
// 为什么必须测：这块逻辑上板后只有"串口有没有回 778 字节"一个观察面，
//   帧边界、累加回放、bin 字节序、校验和任何一处错了都表现为"数据不对"，
//   而现场没有采集卡、没有 ILA，很难定位。所以在这里把口径全部钉死。
//
// 激励：
//   · 连续产生 16×16 的小帧（每帧 256 个 de 像素），帧间有消隐；
//   · 第一轮每帧的 Y = 0..255 各一次 → 每个 bin 应为 1；
//   · 第二轮每帧的 Y 全是 42     → bin42 应为 256，其余为 0。
//     （第二轮同时覆盖"同一个 bin 连续 256 个像素"的回放路径。）
//   · 直接给 hist_req_i 单拍（uart_cmd 的 'H' 通路由 tb_uart_cmd 单独测），
//     模块应等下一个 vsync 上升沿，统计完整一帧后再回传。
//
// 自检项：
//   1) 778 字节的 magic / 'H' / frame_id / frame_total / bin / checksum；
//   2) sum(bins) == frame_total == 256；
//   3) 两次测量的 frame_id 递增；
//   4) 回传期间 busy_o=1，发流期间 tx_busy_o=1；
//   5) 回传结束后 busy_o 落回 0（H_CLEAR 完成）。
//
// 参数取 CLK_FREQ=16MHz / BAUD=1M（DIV=16），不是真实值：这里要验的是
//   帧机和字节序，分频比本身已由 tb_uart_cmd 用真实 74.25MHz/460800 验过。
//   778 字节 × 10 位 × 16 拍 ≈ 12.4 万拍，两轮约 25 万拍，秒级跑完。
//
// 注意：本文件不要写形如 "// verilator ..." 的注释 —— 会被当成
//   metapragma，Verilator 直接编不过（本工程踩过一次）。
//=====================================================================
module tb_gray_hist_uart;

    localparam integer CLK_FREQ = 16000000;
    localparam integer BAUD     = 1000000;
    localparam integer DIV      = (CLK_FREQ + (BAUD / 2)) / BAUD;   // = 16
    localparam real    TCLK     = 1000.0 / 16.0;                    // 62.5 ns

    localparam integer H_ACT  = 16;
    localparam integer V_ACT  = 16;
    localparam integer H_BLANK = 4;
    localparam integer V_BLANK = 4;
    localparam integer H_LINE = H_ACT + H_BLANK;
    localparam integer VS_LEAD = 4;
    localparam integer FRAME_CYCLES = VS_LEAD + V_ACT * H_LINE
                                    + V_BLANK * H_LINE + 4;

    reg clk = 0;
    always #(TCLK / 2.0) clk = ~clk;

    reg        rst_n = 0;
    reg [7:0]  y_i = 0;
    reg        de_i = 0;
    reg        vsync_i = 0;
    reg        hist_req_i = 0;
    reg        pattern_sel = 0;

    wire       busy_o;
    wire       tx_busy_o;
    wire       tx;

    gray_hist_uart #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD     (BAUD)
    ) u_dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .y_i           (y_i),
        .de_i          (de_i),
        .vsync_i       (vsync_i),
        .hist_req_i    (hist_req_i),
        .cmd_tx_busy_i (1'b0),          // 本台不接 uart_cmd，恒为空闲
        .busy_o        (busy_o),
        .tx_busy_o     (tx_busy_o),
        .tx_o          (tx)
    );

    //-----------------------------------------------------------------
    // 帧激励：16×16 有效像素 + 消隐。vsync 是"帧有效电平"，
    // 上升沿在前，有效像素在 VS_LEAD 拍之后才出现（与真实时序一致）。
    //-----------------------------------------------------------------
    integer fcnt;
    integer row, col;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fcnt    <= 0;
            vsync_i <= 1'b0;
            de_i    <= 1'b0;
            y_i     <= 8'd0;
        end else begin
            if (fcnt == FRAME_CYCLES - 1) fcnt <= 0;
            else                          fcnt <= fcnt + 1;

            vsync_i <= (fcnt < VS_LEAD + V_ACT * H_LINE);
            de_i    <= 1'b0;
            y_i     <= 8'd0;

            if (fcnt >= VS_LEAD) begin
                row = (fcnt - VS_LEAD) / H_LINE;
                col = (fcnt - VS_LEAD) % H_LINE;
                if ((row < V_ACT) && (col < H_ACT)) begin
                    de_i <= 1'b1;
                    y_i  <= (pattern_sel == 1'b0)
                            ? ((row * H_ACT + col) & 8'hFF)   // 0..255 各一次
                            : 8'd42;                          // 全 42
                end
            end
        end
    end

    //-----------------------------------------------------------------
    // 主机侧 UART 接收（按 clk 拍计数，DUT 的位长恰好是 DIV 拍）
    //-----------------------------------------------------------------
    localparam integer RX_TIMEOUT = 6000;   // 拍；足够等 vsync + 一整帧

    integer   rb_i;          // 只给 recv_byte 用
    integer   rb_wt;
    reg [7:0] rx_tmp;
    reg       rx_ok;

    task recv_byte;
        begin
            rx_ok = 1'b1;
            rb_wt = 0;
            while (tx !== 1'b0 && rb_wt < RX_TIMEOUT) begin
                @(posedge clk);
                rb_wt = rb_wt + 1;
            end
            if (tx !== 1'b0) begin
                rx_ok = 1'b0;
                rx_tmp = 8'h00;
            end else begin
                // 起始位中点
                repeat (DIV / 2) @(posedge clk);
                // 8 个数据位，LSB 先到
                for (rb_i = 0; rb_i < 8; rb_i = rb_i + 1) begin
                    repeat (DIV) @(posedge clk);
                    rx_tmp[rb_i] = tx;
                end
                // 停止位中点
                repeat (DIV) @(posedge clk);
                if (tx !== 1'b1) rx_ok = 1'b0;
            end
        end
    endtask

    //-----------------------------------------------------------------
    // 一帧 778 字节的解析与比对
    //-----------------------------------------------------------------
    localparam integer NBYTES  = 778;
    localparam integer HDR_END = 8;
    localparam integer BIN_END = 776;

    reg [7:0] rbuf [0:NBYTES-1];

    integer   nerr = 0;
    integer   si, bi;
    integer   fid, total, sum_bins;
    reg [15:0] chk_calc, chk_recv;
    reg [23:0] bin_val;

    task fail;
        input [1023:0] msg;
        begin
            nerr = nerr + 1;
            $display("  FAIL: %0s", msg);
        end
    endtask

    task recv_stream;
        input integer want_fid;
        input integer mode;          // 0 = 每个 bin 1；1 = bin42=256
        begin
            for (si = 0; si < NBYTES; si = si + 1) begin
                recv_byte;
                if (!rx_ok) begin
                    fail("byte timeout / stop bit error");
                    $display("  (byte index %0d)", si);
                    si = NBYTES;     // 跳出循环
                end else begin
                    rbuf[si] = rx_tmp;
                end
                // 收完第一个字节时肯定还在 H_DUMP（最后一条停止位都没走完），
                // 直接查这两个标志，比另起一个监视进程稳（Verilator 对
                // "always 里只赋值、task 里读"的整型变量优化得很激进）。
                if (si == 0) begin
                    if (tx_busy_o !== 1'b1) fail("tx_busy_o not asserted in stream");
                    if (busy_o    !== 1'b1) fail("busy_o not asserted in stream");
                end
            end

            if (rbuf[0] !== 8'hA5) fail("magic0 != A5");
            if (rbuf[1] !== 8'h5A) fail("magic1 != 5A");
            if (rbuf[2] !== 8'h48) fail("cmd != 'H'");

            fid   = rbuf[3] + (rbuf[4] << 8);
            total = rbuf[5] + (rbuf[6] << 8) + (rbuf[7] << 16);
            if (fid !== want_fid) begin
                fail("frame_id mismatch");
                $display("  (got %0d want %0d)", fid, want_fid);
            end
            if (total !== 256) begin
                fail("frame_total != 256");
                $display("  (got %0d)", total);
            end

            chk_calc = 16'd0;
            for (si = 0; si < BIN_END; si = si + 1)
                chk_calc = chk_calc + rbuf[si];
            chk_recv = rbuf[776] + (rbuf[777] << 8);
            if (chk_calc !== chk_recv) begin
                fail("checksum mismatch");
                $display("  (calc %04x recv %04x)", chk_calc, chk_recv);
            end

            sum_bins = 0;
            for (bi = 0; bi < 256; bi = bi + 1) begin
                bin_val = rbuf[HDR_END + 3*bi]
                        + (rbuf[HDR_END + 3*bi + 1] << 8)
                        + (rbuf[HDR_END + 3*bi + 2] << 16);
                sum_bins = sum_bins + bin_val;
                if (mode == 0) begin
                    if (bin_val !== 24'd1) begin
                        fail("mode0 bin != 1");
                        $display("  (bin %0d = %0d)", bi, bin_val);
                    end
                end else begin
                    if (bi == 42) begin
                        if (bin_val !== 24'd256) begin
                            fail("mode1 bin42 != 256");
                            $display("  (bin42 = %0d)", bin_val);
                        end
                    end else if (bin_val !== 24'd0) begin
                        fail("mode1 non-42 bin != 0");
                        $display("  (bin %0d = %0d)", bi, bin_val);
                    end
                end
            end
            if (sum_bins !== total) begin
                fail("sum(bins) != frame_total");
                $display("  (sum %0d total %0d)", sum_bins, total);
            end

            // 回传结束后 H_CLEAR 还会忙 256 拍，等它落回空闲
            repeat (300) @(posedge clk);
            if (busy_o !== 1'b0) fail("busy_o did not return to 0");
        end
    endtask

    //-----------------------------------------------------------------
    // 请求：单拍脉冲
    //-----------------------------------------------------------------
    task send_req;
        begin
            @(posedge clk);
            hist_req_i = 1'b1;
            @(posedge clk);
            hist_req_i = 1'b0;
        end
    endtask

    //-----------------------------------------------------------------
    // 主流程
    //-----------------------------------------------------------------
    initial begin
        $display("tb_gray_hist_uart: DIV=%0d, frame=%0d cycles, stream=%0d bytes",
                 DIV, FRAME_CYCLES, NBYTES);

        rst_n = 1'b0;
        hist_req_i = 1'b0;
        pattern_sel = 1'b0;
        repeat (20) @(posedge clk);
        rst_n = 1'b1;

        // 等上电清零（256 拍）走完
        repeat (400) @(posedge clk);
        if (busy_o !== 1'b0) fail("busy_o != 0 after reset clear");

        // 第一轮：每个 bin 恰好 1 个像素
        pattern_sel = 1'b0;
        @(posedge vsync_i);
        repeat (20) @(posedge clk);
        send_req;
        repeat (3) @(posedge clk);
        if (busy_o !== 1'b1) fail("busy_o != 1 after request");
        recv_stream(1, 0);

        // 第二轮：同一 bin 连续 256 个像素（测回放/累加）
        pattern_sel = 1'b1;
        @(posedge vsync_i);
        repeat (20) @(posedge clk);
        send_req;
        repeat (3) @(posedge clk);
        if (busy_o !== 1'b1) fail("busy_o != 1 after second request");
        recv_stream(2, 1);

        if (nerr == 0) $display("GRAY_HIST_UART PASS");
        else           $display("GRAY_HIST_UART FAIL (%0d errors)", nerr);
        $finish;
    end

    // 总超时：两轮各约 13 万拍，给 100 万拍
    initial begin
        repeat (1000000) @(posedge clk);
        $display("GRAY_HIST_UART FAIL (global timeout)");
        $finish;
    end

endmodule
