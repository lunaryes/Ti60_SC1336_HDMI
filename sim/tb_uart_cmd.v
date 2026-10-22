`timescale 1ns / 1ps
//=====================================================================
// tb_uart_cmd —— 串口命令口的回归（M5 新增）
//---------------------------------------------------------------------
// 为什么必须测：这是全工程唯一一块"上板之后没法用波形看"的新逻辑——波特率分频、
//   收发状态机、2 字节帧解析，任何一处错了在板上都表现为"串口没反应"，而排查手段
//   只有串口本身（鸡生蛋）。所以先在仿真里把协议跑通。
//
// 参数用的是**真实值**（74.25 MHz / 460800），不是缩小版：分频比 161 本身就是要
//   验的东西之一，缩小了就没意义了。整段仿真约 30k 个时钟，不到一秒。
//   ★ 时钟周期必须与 CLK_FREQ 一致（13.468 ns），否则 DUT 的波特率跟着变，
//     而测试台的位时长是按 460800 算的 —— 两边对不上会全错。这是本台子最容易
//     被改坏的地方。
//
// 覆盖：复位默认值 / T·V·M·S·L 五条参数命令 / 'T'/'S' 写入单拍脉冲
//   （thr_valid_o / smooth_valid_o）/ 'S' 参数 >2 截断到 2 / 越界参数按位宽截断 /
//   干扰字节（回车、换行、空格）被忽略 / 未知命令**不会**把解析器卡在等参数状态 /
//   'R' 回读 8 字节（thr 来自 thr_read_i，即实际生效值）/ 'A'/'B' 调 AWB 增益 /
//   连发两帧（无帧间间隔）。
//
// 用法（sim/run.sh 已挂）： verilator --binary --timing ... ; ./Vtb_uart_cmd
//=====================================================================
module tb_uart_cmd;

    localparam integer CLK_FREQ = 74250000;
    localparam integer BAUD     = 460800;
    localparam integer DIV      = (CLK_FREQ + (BAUD / 2)) / BAUD;   // = 161，与 DUT 同一算法
    localparam real    TCLK     = 1000.0 / 74.25;        // 13.4680 ns
    localparam real    TBIT     = 1000000000.0 / BAUD;   // 2170.14 ns
    // DUT 的实际位时长 = 161 × TCLK = 2168.3 ns，比标称少 0.08%；
    // 收发都在位中点采样，10 位累积误差不到 0.1 位，可以忽略（这一项本身就是
    // "波特率误差够不够小"的实测）。

    reg  clk = 0, rst_n = 0;
    always #(TCLK / 2.0) clk = ~clk;

    reg         rx = 1'b1;          // 空闲为高
    wire        tx;
    wire [7:0]  thr_o;
    wire        thr_valid;            // 'T' 写入单拍（M7）
    wire [1:0]  view_o, mode_o;
    wire [1:0]  smooth_o;             // 'S' 平滑模式 0/1/2
    wire        smooth_valid;         // 'S' 写入单拍
    wire [7:0]  gain_r_o, gain_b_o;   // AWB 增益（1/64 定点）
    wire        link_o;               // 'L' 方向约束流式反馈开关
    wire        hist_req;             // 'H' 单拍（M6）
    wire [2:0] play_mode;
    wire tnr_enabled,status_req;
    wire [3:0] age_a,age_b;
    wire [7:0] replay_div;
    reg status_valid=0;
    wire history_req;
    wire [63:0] iir_tuning;
    reg history_valid=0;
    always @(posedge clk) if(!rst_n)history_valid<=0;else history_valid<=history_req;
    always @(posedge clk) if(!rst_n) status_valid<=0;else status_valid<=status_req;
    wire        tx_busy;              // TX 忙（含待发请求）

    // 'R' 回读的两个"实际生效值"输入。真实拓扑里它们来自按键控制器
    // （按键调过之后回读也准确）；本台子没有按键控制器，默认把 thr/smooth
    // 接回 uart_cmd 自己的输出，等价于"只有串口在改"。
    // ext_hold=1 时改用外部寄存器，专门验证回读取的是 read_i 而不是 tx_d 历史。
    reg  [7:0] ext_thr_read = 8'd0;
    reg  [1:0] ext_smooth_read = 2'd0;
    reg        ext_hold = 1'b0;
    wire [7:0] thr_read  = ext_hold ? ext_thr_read  : thr_o;
    wire [1:0] smooth_read = ext_hold ? ext_smooth_read : smooth_o;

    uart_cmd #(
        .CLK_FREQ (CLK_FREQ),
        .ENABLE_DIAGNOSTICS(1),
        .BAUD     (BAUD),
        .THR_DEF  (8'd12),          // 与 example_top 的 IMG_THR_DEF 一致（上板定的工作点）
        .VIEW_DEF (2'd0),
        .MODE_DEF (2'd3)
        // GAIN_R_DEF / GAIN_B_DEF / SMOOTH_DEF 用模块默认值
        // （112/96 = ×1.75/×1.5，smooth 复位 = 1 弱平滑）
    ) u_dut (
        .clk (clk), .rst_n (rst_n),
        .rx_i (rx), .tx_o (tx),
        .status_valid_i(status_valid),.app_status_i(32'h00180042),
        .status_req_o(status_req),.play_mode_o(play_mode),.tnr_enable_o(tnr_enabled),
        .history_req_o(history_req),.history_valid_i(history_valid),.history_status_i(32'h0012340c),
        .iir_tuning_o(iir_tuning),
        .strong_pixels_i(32'h00012345),.weak_pixels_i(32'h00023456),.bypass_pixels_i(32'h00034567),
        .age_a_o(age_a),.age_b_o(age_b),.replay_div_o(replay_div),
        .thr_read_i (thr_read), .smooth_read_i (smooth_read),
        .thr_o (thr_o), .thr_valid_o (thr_valid),
        .view_o (view_o), .mode_o (mode_o),
        .smooth_o (smooth_o), .smooth_valid_o (smooth_valid),
        .link_o   (link_o),
        .gain_r_o (gain_r_o), .gain_b_o (gain_b_o),
        .hist_req_o (hist_req), .tx_busy_o (tx_busy)
    );

    // 'T' 写入脉冲计数：每条 'T' 命令的参数帧应恰好出一个单拍脉冲，
    // 既不能漏（流水线收不到新阈值）也不能多（一帧写两次/误触发）。
    reg [7:0] thr_valid_cnt = 0;
    always @(posedge clk) begin
        if (thr_valid) thr_valid_cnt <= thr_valid_cnt + 1'b1;
    end

    // 'S' 写入脉冲计数，要求同 'T'。
    reg [7:0] smooth_valid_cnt = 0;
    always @(posedge clk) begin
        if (smooth_valid) smooth_valid_cnt <= smooth_valid_cnt + 1'b1;
    end

    // 'H' 是单拍脉冲。不能用"另一个 always 里置 sticky 标志、initial 里读"的
    // 写法：Verilator 5.029 会把那个 always 的赋值优化成函数内局部变量
    // （反汇编里能看到 `CData hist_seen; hist_seen = 1U;`，没有写回状态），
    // initial 里永远读到 0。改成在停止位期间按拍轮询，赋值和读取都在
    // 同一个 initial 进程里。
    integer hp_i;
    reg     hist_caught;

    task send_byte_poll_hist;
        input [7:0] b;
        begin
            hist_caught = 1'b0;
            rx = 1'b0;  #(TBIT);                    // 起始位
            for (hp_i = 0; hp_i < 8; hp_i = hp_i + 1) begin
                rx = b[hp_i];  #(TBIT);             // 8 个数据位
            end
            rx = 1'b1;                              // 停止位
            // DUT 在停止位中点（9.5 位）解析完 'H' 并出一拍 hist_req；
            // 这里从 9.0 位开始按拍采样，正好覆盖它。
            for (hp_i = 0; hp_i < DIV; hp_i = hp_i + 1) begin
                @(posedge clk);
                if (hist_req) hist_caught = 1'b1;
            end
        end
    endtask

    // ★ 循环变量必须各用各的：任务是静态作用域，recv_byte 里的 for 变量若与调用
    //   方共用同一个 integer，会把调用方的循环计数器冲掉（本台子第一版就踩了：
    //   外层的 `for (k...)` 被内层改成 8 之后立刻退出，回读的 4 个字节只对 1 个，
    //   还写飞到了 rb[8]，看起来像"协议解析错了"，其实是测试台的锅）。
    integer   ncase = 0, nerr = 0;
    integer   k;            // 只给 send_byte 用
    integer   kk;           // 只给 recv_byte 用
    integer   j;            // 只给调用方的循环用
    reg [7:0] rxb;
    reg [7:0] rb [0:7];         // 回包 8 字节 {thr, view, mode, gain_r, gain_b, smooth, link, '\n'}
    reg       rx_ok;        // recv_byte 的结果标志
    integer   wt;           // recv_byte 的超时计数

    //------------------------------------------------------------------
    // 主机侧：发一个字节（8N1，LSB 先到）
    //   gap = 停止位之后额外空几位（0 = 帧间不停顿，用来测连发）
    //------------------------------------------------------------------
    task send_byte;
        input [7:0] b;
        input integer gap;
        begin
            rx = 1'b0;  #(TBIT);                    // 起始位
            for (k = 0; k < 8; k = k + 1) begin
                rx = b[k];  #(TBIT);
            end
            rx = 1'b1;  #(TBIT);                    // 停止位
            if (gap > 0) #(TBIT * gap);
        end
    endtask

    //------------------------------------------------------------------
    // 发命令字节，但停止位只**拉高电平、不等它走完**就返回。
    //
    // ★ 只给 'R' 用，原因很关键：DUT 在停止位中点（≈9.5 位处）就结束收字节并
    //   立刻启动回包，回包的起始沿落在 9.5~10 位之间。若像普通命令那样走完
    //   停止位（10 位）再返回，接收端就错过了起始沿、从数据位中途"上车"，
    //   把 bit0 读成 bit1 —— 实测 thr=0x08 读成 0x84、view=0x03 读成 0x81，
    //   正好是整体错一位（第三、四字节因为重新对齐上沿反而是对的，极具迷惑性）。
    //   这里 9 位就返回，接收端从 9.0 位开始等沿，必然赶得上。
    //------------------------------------------------------------------
    task send_cmd_nostop;
        input [7:0] b;
        begin
            rx = 1'b0;  #(TBIT);                    // 起始位
            for (k = 0; k < 8; k = k + 1) begin
                rx = b[k];  #(TBIT);
            end
            rx = 1'b1;                              // 停止位电平，不等走完
        end
    endtask

    //------------------------------------------------------------------
    // 收一个字节，结果放 rxb；rx_ok = 0 表示等起始位超时（DUT 没回包）
    //
    // ★ 采样点**按 clk 数拍**，不用 #(TBIT)：DUT 的位长恰好是 DIV=161 拍，
    //   按拍数走则收发两侧零漂移。用 #(2170.14ns) 虽然 0.08% 的误差不至于采错，
    //   但一旦有人改动 BAUD/CLK_FREQ 使分频比变化，这里就会静默失配。
    // ★ 必须带超时：DUT 若因故不回包，`@(negedge tx)` 会把整个仿真挂死，
    //   在 run.sh 里表现为"卡住不结束"（本台子第一版就是这么挂的）。
    //   DIV-1+... 的取整见下：检测下降沿最多晚 1 拍，所以少等 1 拍。
    //------------------------------------------------------------------
    task recv_byte;
        begin
            rx_ok = 1'b1;
            wt    = 0;
            while (tx !== 1'b0 && wt < 20 * DIV) begin
                @(posedge clk);
                wt = wt + 1;
            end
            if (tx !== 1'b0) begin                 // 超时：DUT 没发
                rx_ok = 1'b0;
                rxb   = 8'hFF;
            end else begin
                repeat (DIV + DIV / 2 - 1) @(posedge clk);   // 1.5 位 → bit0 中点
                for (kk = 0; kk < 8; kk = kk + 1) begin
                    rxb[kk] = tx;
                    repeat (DIV) @(posedge clk);
                end
            end
        end
    endtask

    task chk;
        input [255:0] name;
        input [7:0]   got;
        input [7:0]   exp;
        begin
            ncase = ncase + 1;
            if (got !== exp) begin
                nerr = nerr + 1;
                $display("  !! FAIL: %0s = %0d（期望 %0d）", name, got, exp);
            end
        end
    endtask

    initial begin
        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (4) @(posedge clk);

        //--------------------------------------------------------------
        // 1) 复位默认值 = 参数传入的 THR_DEF/VIEW_DEF/MODE_DEF（不插串口也要能用）
        //--------------------------------------------------------------
        chk("复位 thr",  thr_o,  8'd12);
        chk("复位 view", {6'b0, view_o}, 8'd0);
        chk("复位 mode", {6'b0, mode_o}, 8'd3);
        chk("复位 smooth", {6'b0, smooth_o}, 8'd1);  // 默认弱平滑
        chk("复位 R 增益", gain_r_o, 8'd112);   // ×1.75，上板对白墙调出来的中性点
        chk("复位 B 增益", gain_b_o, 8'd96);    // ×1.5（沿用基线 384/256）
        chk("复位 link 开", {7'b0, link_o}, 8'd1);

        //--------------------------------------------------------------
        // 2) 'T' 设阈值（thr_low 由 RTL 内部取 >>1，不出现在这个口上）
        //--------------------------------------------------------------
        send_byte(8'h54, 1); send_byte(8'h20, 1);      // 'T' 0x20
        chk("T 0x20", thr_o, 8'd32);
        send_byte(8'h54, 1); send_byte(8'h00, 1);      // 'T' 0x00（边界：等于关掉阈值）
        chk("T 0x00", thr_o, 8'd0);
        chk("两条 T 各一个写入脉冲", thr_valid_cnt, 8'd2);

        //--------------------------------------------------------------
        // 3) 'V' / 'M'：参数只取低 2 位
        //--------------------------------------------------------------
        send_byte(8'h56, 1); send_byte(8'h02, 1);      // 'V' 0x02
        chk("V 0x02", {6'b0, view_o}, 8'd2);
        send_byte(8'h56, 1); send_byte(8'hFF, 1);      // 'V' 0xFF → 只取 [1:0] = 3
        chk("V 0xFF", {6'b0, view_o}, 8'd3);
        send_byte(8'h4D, 1); send_byte(8'h01, 1);      // 'M' 0x01
        chk("M 0x01", {6'b0, mode_o}, 8'd1);

        //--------------------------------------------------------------
        // 3b) 'S' 平滑模式：0 标准高斯 / 1 弱平滑 / 2 旁路；参数 >2 截断到 2。
        //     与 'T' 一样带 valid 单拍脉冲，按键控制器按它抢占按键写入。
        //--------------------------------------------------------------
        send_byte(8'h53, 1); send_byte(8'h00, 1);      // 'S' 0x00
        chk("S 0x00 标准高斯", {6'b0, smooth_o}, 8'd0);
        send_byte(8'h53, 1); send_byte(8'h02, 1);      // 'S' 0x02
        chk("S 0x02 旁路", {6'b0, smooth_o}, 8'd2);
        send_byte(8'h53, 1); send_byte(8'hFF, 1);      // 'S' 0xFF → [1:0]=3 → 截断 2
        chk("S 0xFF 截断到 2", {6'b0, smooth_o}, 8'd2);
        send_byte(8'h53, 1); send_byte(8'h01, 1);      // 'S' 0x01 恢复默认
        chk("S 0x01 弱平滑", {6'b0, smooth_o}, 8'd1);
        chk("四条 S 各一个写入脉冲", smooth_valid_cnt, 8'd4);
        chk("S 不影响 thr", thr_o, 8'd0);

        //--------------------------------------------------------------
        // 3c) 'L' 方向约束流式反馈开关：1 开 / 0 关，只取 bit0。
        //     上板"不重新综合对比开/关"就靠它，回读第 7 字节确认。
        //--------------------------------------------------------------
        send_byte(8'h4C, 1); send_byte(8'h00, 1);      // 'L' 0x00 → 关
        chk("L 0x00 关", {7'b0, link_o}, 8'd0);
        send_byte(8'h4C, 1); send_byte(8'hFE, 1);      // 'L' 0xFE → 只取 bit0 = 0
        chk("L 0xFE 只取 bit0", {7'b0, link_o}, 8'd0);
        send_byte(8'h4C, 1); send_byte(8'h01, 1);      // 'L' 0x01 → 开
        chk("L 0x01 开", {7'b0, link_o}, 8'd1);

        //--------------------------------------------------------------
        // 4) 干扰字节（回车/换行/空格）必须被忽略，不能当成命令或参数
        //--------------------------------------------------------------
        send_byte(8'h0D, 1); send_byte(8'h0A, 1); send_byte(8'h20, 1);
        chk("回车换行后 thr",  thr_o, 8'd0);
        chk("回车换行后 view", {6'b0, view_o}, 8'd3);
        chk("回车换行后 mode", {6'b0, mode_o}, 8'd1);

        //--------------------------------------------------------------
        // 5) 未知命令字符不能把解析器卡在"等参数"状态：
        //    两个未知字节应丢掉，不能进入等待参数状态。
        //    这一步过了，紧跟着的 'T' 0x08 才能正常生效。
        //--------------------------------------------------------------
        send_byte(8'h7E, 1); send_byte(8'h7F, 1);      // Both remain unknown commands.
        send_byte(8'h54, 1); send_byte(8'h08, 1);      // 'T' 0x08
        chk("未知命令后 T 0x08", thr_o, 8'd8);
        chk("未知命令不误出写入脉冲", thr_valid_cnt, 8'd3);

        //--------------------------------------------------------------
        // 6) 'R' 回读 8 字节 {thr, view, mode, gain_r, gain_b, smooth, link, '\n'}
        //    当前状态：thr=8、view=3、mode=1、smooth=1、link=1，增益默认 112/96
        //--------------------------------------------------------------
        send_cmd_nostop(8'h52);                         // 'R'
        for (j = 0; j < 8; j = j + 1) begin
            recv_byte;
            rb[j] = rxb;
            if (!rx_ok) begin
                nerr = nerr + 1;
                $display("  !! FAIL: 回读第 %0d 字节超时（DUT 未回包）@%0t", j, $time);
            end
        end
        chk("回读 thr",  rb[0], 8'd8);
        chk("回读 view", rb[1], 8'd3);
        chk("回读 mode", rb[2], 8'd1);
        chk("回读 R 增益复位值", rb[3], 8'd112);        // 112/64 = ×1.75（上板调的中性点）
        chk("回读 B 增益复位值", rb[4], 8'd96);         // 96/64 = ×1.5 = 原 384/256
        chk("回读 smooth", rb[5], 8'd1);
        chk("回读 link", rb[6], 8'd1);
        chk("回读结尾换行", rb[7], 8'h0A);

        //--------------------------------------------------------------
        // 7) AWB 增益 'A'/'B'（1/64 定点，量程 0..3.98×）
        //    这个旋钮是 2026-10-15 上板实测"白墙发青 = R 偏低"之后加的：
        //    调它要试好几轮，所以必须能运行时改 + 能从回读里确认。
        //--------------------------------------------------------------
        send_byte(8'h41, 1); send_byte(8'hA0, 1);      // 'A' 0xA0 = 160 → ×2.5
        chk("A 0xA0 → R 增益 ×2.5", gain_r_o, 8'd160);
        send_byte(8'h42, 1); send_byte(8'h40, 1);      // 'B' 0x40 =  64 → ×1.0
        chk("B 0x40 → B 增益 ×1.0", gain_b_o, 8'd64);
        send_byte(8'h41, 1); send_byte(8'h00, 1);      // 'A' 0x00 =   0 → 边界：R 通道全黑
        chk("A 0x00 → R 增益 0",  gain_r_o, 8'd0);
        send_byte(8'h41, 1); send_byte(8'hFF, 1);      // 'A' 0xFF = 255 → 量程上限 ×3.98
        chk("A 0xFF → R 增益上限", gain_r_o, 8'd255);
        send_byte(8'h41, 1); send_byte(8'h70, 1);      // 'A' 0x70 = 112 → 恢复默认
        send_byte(8'h42, 1); send_byte(8'h60, 1);      // 'B' 0x60 =  96 → 恢复默认
        chk("恢复 R 增益默认", gain_r_o, 8'd112);
        chk("恢复 B 增益默认", gain_b_o, 8'd96);

        //--------------------------------------------------------------
        // 8) 连发两帧（帧间零间隔）：接收状态机必须在停止位结束时及时回 IDLE
        //--------------------------------------------------------------
        send_byte(8'h54, 0); send_byte(8'h18, 0);      // 'T' 0x18
        send_byte(8'h56, 0); send_byte(8'h01, 0);      // 'V' 0x01
        send_byte(8'h00, 1);                            // 尾部补一位，让最后一帧走完
        chk("连发 thr",  thr_o, 8'd24);
        chk("连发 view", {6'b0, view_o}, 8'd1);
        chk("连发后写入脉冲总数", thr_valid_cnt, 8'd4);

        //--------------------------------------------------------------
        // 9) 改完增益再回读一次：**调参时唯一的确认手段**就是回包里这两个字节
        //    （板上转旋钮的时候，画面变化和回读值必须能对上）
        //--------------------------------------------------------------
        send_byte(8'h41, 1); send_byte(8'hC8, 1);      // 'A' 0xC8 = 200 → ×3.125
        send_byte(8'h42, 1); send_byte(8'h50, 1);      // 'B' 0x50 =  80 → ×1.25
        send_cmd_nostop(8'h52);
        for (j = 0; j < 8; j = j + 1) begin
            recv_byte;
            rb[j] = rxb;
            if (!rx_ok) begin
                nerr = nerr + 1;
                $display("  !! FAIL: 第二次回读第 %0d 字节超时 @%0t", j, $time);
            end
        end
        chk("回读改后的 R 增益", rb[3], 8'd200);
        chk("回读改后的 B 增益", rb[4], 8'd80);
        chk("回读 link 仍为开", rb[6], 8'd1);
        chk("回读改后仍以换行结尾", rb[7], 8'h0A);

        //--------------------------------------------------------------
        // 9b) 'R' 的 thr/smooth 字节取的是 thr_read_i / smooth_read_i
        //     （真实拓扑里来自按键控制器 = 实际生效值），不是 uart_cmd
        //     自己最后一次写入的 tx_d 历史。这一条防的就是"按键调过之后
        //     回读还显示串口旧值"那类问题。
        //--------------------------------------------------------------
        ext_thr_read    = 8'h2A;                        // 模拟按键控制器当前值
        ext_smooth_read = 2'd2;
        ext_hold        = 1'b1;
        send_cmd_nostop(8'h52);
        for (j = 0; j < 8; j = j + 1) begin
            recv_byte;
            rb[j] = rxb;
            if (!rx_ok) begin
                nerr = nerr + 1;
                $display("  !! FAIL: read_i 回读第 %0d 字节超时 @%0t", j, $time);
            end
        end
        chk("回读 thr 取自 read_i", rb[0], 8'h2A);
        chk("回读 smooth 取自 read_i", rb[5], 8'd2);
        ext_hold = 1'b0;
        repeat (DIV) @(posedge clk);

        //--------------------------------------------------------------
        // 10) 'H'：片内直方图请求（M6）。本模块只出单拍 hist_req，不回包；
        //     真正的 778 字节由 gray_hist_uart 发（另一个台子验）。
        //     'H' 若误走进 p_wait 或误触发 'R' 的回包，这里会抓到。
        //--------------------------------------------------------------
        send_byte_poll_hist(8'h48);             // 'H'
        chk("H → hist_req 单拍", {7'b0, hist_caught}, 8'd1);
        repeat (4 * DIV) @(posedge clk);        // 4 个字节时间
        chk("H 不回包（tx 保持空闲高）", {7'b0, tx}, 8'd1);
        chk("H 后 tx_busy 落回 0", {7'b0, tx_busy}, 8'd0);

        //--------------------------------------------------------------
        // 11) tx_busy_o：'R' 回包期间为 1，回包结束后回 0。
        //     直方图模块靠它避让在途回包，错了会在板上把 8 字节回包截断。
        //--------------------------------------------------------------
        send_cmd_nostop(8'h52);                 // 'R'
        repeat (2 * DIV) @(posedge clk);
        chk("R 回包期间 tx_busy", {7'b0, tx_busy}, 8'd1);
        for (j = 0; j < 8; j = j + 1) begin
            recv_byte;
            rb[j] = rxb;
            if (!rx_ok) begin
                nerr = nerr + 1;
                $display("  !! FAIL: tx_busy 用例回读第 %0d 字节超时 @%0t", j, $time);
            end
        end
        repeat (DIV) @(posedge clk);
        chk("R 回包结束 tx_busy", {7'b0, tx_busy}, 8'd0);

        //--------------------------------------------------------------
        send_byte(8'h50,1);send_byte(8'd2,1);
        send_byte(8'h4e,1);send_byte(8'd0,1);
        send_byte(8'h4a,1);send_byte(8'd3,1);
        send_byte(8'h4b,1);send_byte(8'd1,1);
        send_byte(8'h4f,1);send_byte(8'd4,1);
        chk("application replay mode",{5'd0,play_mode},8'd2);
        chk("application tnr disabled",{7'd0,tnr_enabled},8'd0);
        chk("application frame age",{4'd0,age_a},8'd3);
        chk("application reference age",{4'd0,age_b},8'd1);
        chk("application replay divider",replay_div,8'd4);
        send_cmd_nostop(8'h51);
        for(j=0;j<8;j=j+1) begin recv_byte;rb[j]=rxb;if(!rx_ok)nerr=nerr+1;end
        chk("Q application magic",rb[0],8'h56);chk("Q application mode",rb[1],8'd2);
        send_cmd_nostop(8'h44);
        for(j=0;j<8;j=j+1) begin recv_byte;rb[j]=rxb;if(!rx_ok)nerr=nerr+1;end
        chk("D mailbox magic",rb[0],8'h44);chk("D mailbox application",rb[1],8'h56);
        chk("D mailbox source",rb[3],8'h42);chk("D mailbox status",rb[5],8'h18);
        send_byte(8'h46,1);send_byte(8'd1,1);chk("F1 freezes",{5'd0,play_mode},8'd1);
        send_byte(8'h46,1);send_byte(8'd0,1);chk("F0 restores live",{5'd0,play_mode},8'd0);
        send_byte(8'h4f,1);send_byte(8'd0,1);chk("divider zero clamped",replay_div,8'd1);
        send_cmd_nostop(8'h49);
        for(j=0;j<8;j=j+1)begin recv_byte;rb[j]=rxb;if(!rx_ok)nerr=nerr+1;end
        chk("IIR status signature",rb[0],8'h49);chk("IIR status payload",rb[3],8'h0c);
        chk("IIR source low",rb[4],8'h34);chk("IIR source high",rb[5],8'h12);
        send_cmd_nostop(8'h55);
        for(j=0;j<8;j=j+1)begin recv_byte;rb[j]=rxb;if(!rx_ok)nerr=nerr+1;end
        chk("strong count signature",rb[0],8'h55);chk("strong count low",rb[3],8'h45);chk("strong count mid",rb[4],8'h23);
        send_cmd_nostop(8'h57);
        for(j=0;j<8;j=j+1)begin recv_byte;rb[j]=rxb;if(!rx_ok)nerr=nerr+1;end
        chk("weak count low",rb[3],8'h56);
        send_cmd_nostop(8'h58);
        for(j=0;j<8;j=j+1)begin recv_byte;rb[j]=rxb;if(!rx_ok)nerr=nerr+1;end
        chk("bypass count low",rb[3],8'h67);
        send_byte(8'h59,1);send_byte(8'h00,1);
        send_byte(8'h5a,1);send_byte(8'h02,1);
        chk("runtime strong weight",iir_tuning[7:0],8'h02);
        send_cmd_nostop(8'h5b);
        for(j=0;j<8;j=j+1)begin recv_byte;rb[j]=rxb;if(!rx_ok)nerr=nerr+1;end
        chk("tuning reply version",rb[2],8'h02);chk("tuning reply value",rb[4],8'h02);
        send_byte(8'h59,1);send_byte(8'h01,1);send_byte(8'h5a,1);send_byte(8'hff,1);
        chk("weight clamped",iir_tuning[15:8],8'h03);
        send_byte(8'h59,1);send_byte(8'h08,1);send_byte(8'h5a,1);send_byte(8'h99,1);
        chk("invalid index ignored",iir_tuning[7:0],8'h02);
        send_cmd_nostop(8'h5b);
        for(j=0;j<8;j=j+1)begin recv_byte;rb[j]=rxb;if(!rx_ok)nerr=nerr+1;end
        chk("invalid index flag",rb[6],8'h01);
        send_byte(8'h5d,1);
        if(iir_tuning!=64'h0818080405020201)begin nerr=nerr+1;$display("runtime defaults mismatch");end
        chk("restored strong weight",iir_tuning[7:0],8'h01);
        chk("restored weak weight",iir_tuning[15:8],8'h02);
        if (nerr == 0)
            $display("UART_CMD PASS（%0d 个用例，0 错误）", ncase);
        else
            $display("UART_CMD FAIL（%0d 个用例，%0d 个错误）", ncase, nerr);
        $finish;
    end

endmodule
