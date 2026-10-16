`timescale 1ns / 1ps
//=====================================================================
// tb_canny_thr_key —— 按键阈值控制器（canny_threshold_key_ctrl）的回归
//                     2026-10-16 接入 example_top 时新增
//---------------------------------------------------------------------
// 为什么必须测：从今天起**这一块是 Canny 强阈值的唯一来源**（串口那一路已在
//   example_top 里断开，见那里的 ★ 注释），而它的消抖窗口是算出来的：
//
//       CNT_MAX = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS - 1
//       真实参数 74.25 MHz × 20 ms = 1 484 999 → CNT_W = 21 位
//
//   算错一位，板上的表现是"按键没反应"或者"一按连跳好几档"；而 20 ms 是 148 万
//   拍，在波形窗口里根本看不出来 —— 这类错只能在仿真里量。
//
// 两个实例：
//   dut_fast —— 缩小参数（8 拍消抖）跑全部行为用例：抖动不认、按下沿只认一次、
//               按住不重复步进、上下限饱和、同时按、uart 软写优先级、脉冲宽度；
//   dut_real —— 真实参数（74.25 MHz / 20 ms）只跑两条：抖动不认、按够 20 ms 才
//               认（并把"按下沿 → 阈值变化"的拍数量出来，验 CNT_MAX 与真实时长
//               一致，而不是只验"能认出来"）。
//
// 口径与 RTL 的对应（改 RTL 时这几条要一起看）：
//   · 松开 = 2'b11、按下 = 0（板上 10k 上拉，K1/K2 分别是 GPIOL_P_02 / N_02）；
//   · 只有"稳定 1→0"才产生一个单拍脉冲，按住不放**不会**连续步进；
//   · K1 加、K2 减，步长 4，范围 4..48 内**饱和**（不回绕）；
//   · uart_thr_valid_i 优先于按键，且允许越界（8 位全量程，为串口调参留的）；
//     越界值之后按一次按键会先回到 THR_MAX，再按才继续递减。
//
// 用法（sim/run.sh 第 5 段已挂）：在工程根目录跑
//   $ verilator --binary --timing -Wno-fatal -Wno-lint --top-module tb_canny_thr_key \
//       Ti60F225_SC1336_HDMI_Display/src/control/canny_threshold_key_ctrl.v \
//       sim/tb_canny_thr_key.v -o Vtb_canny_thr_key --Mdir sim/build/obj_thr_key
//   ★ 注意这行注释不能写成 "// verilator ..."：Verilator 把行首 // 后的 verilator
//     也当成 metapragma（// verilator lint_off 那种），会报 "Unknown verilator
//     comment" 直接编不过。前面加个 $ 就绕开了。
//=====================================================================
module tb_canny_thr_key;

    //------------------------------------------------------------------
    // 快速实例：消抖窗口压到 8 拍（(8000/1000) × 1 - 1 = 7 → CNT_W = 3）
    //   时钟周期与 RTL 参数无关（它只数拍），10 ns 纯粹是仿真节拍。
    //------------------------------------------------------------------
    localparam integer FAST_HZ   = 8000;
    localparam integer FAST_DBMS = 1;
    localparam real    TFAST     = 10.0;
    localparam integer DB_CYC    = (FAST_HZ / 1000) * FAST_DBMS;   // = 8 拍

    reg        clk_f   = 0;
    always #(TFAST / 2.0) clk_f = ~clk_f;

    reg        rst_n_f  = 0;
    reg  [1:0] key_f    = 2'b11;
    reg        uart_v_f = 0;
    reg  [7:0] uart_d_f = 0;
    wire [7:0] thr_f, thrl_f;
    wire [1:0] press_f;

    canny_threshold_key_ctrl #(
        .CLK_FREQ_HZ (FAST_HZ),
        .DEBOUNCE_MS (FAST_DBMS),
        .THR_DEFAULT (8'd12),       // 与 example_top 的 IMG_THR_DEF 一致（上板工作点）
        .THR_MIN     (8'd4),
        .THR_MAX     (8'd48),
        .THR_STEP    (8'd4)
    ) dut_fast (
        .clk              (clk_f    ),
        .rst_n            (rst_n_f  ),
        .key_data         (key_f    ),
        .uart_thr_valid_i (uart_v_f ),
        .uart_thr_i       (uart_d_f ),
        .threshold_o      (thr_f    ),
        .threshold_low_o  (thrl_f   ),
        .key_press_o      (press_f  )
    );

    //------------------------------------------------------------------
    // 真实参数实例：74.25 MHz / 20 ms —— 与 example_top 里的例化逐个参数同值
    //------------------------------------------------------------------
    localparam real    TCLK     = 1000.0 / 74.25;   // 13.4680 ns
    localparam integer HALF_CYC = 74250 * 10;       // 10 ms = 742 500 拍（半个窗口）
    localparam integer FULL_CYC = 74250 * 20;       // 20 ms = 1 485 000 拍 = CNT_MAX + 1

    reg        clk_r   = 0;
    always #(TCLK / 2.0) clk_r = ~clk_r;

    reg        rst_n_r = 0;
    reg  [1:0] key_r   = 2'b11;
    wire [7:0] thr_r, thrl_r;

    canny_threshold_key_ctrl #(
        .CLK_FREQ_HZ (74250000),
        .DEBOUNCE_MS (20),
        .THR_DEFAULT (8'd12),
        .THR_MIN     (8'd4),
        .THR_MAX     (8'd48),
        .THR_STEP    (8'd4)
    ) dut_real (
        .clk              (clk_r        ),
        .rst_n            (rst_n_r      ),
        .key_data         (key_r        ),
        .uart_thr_valid_i (1'b0         ),      // 顶层也是这么接的（串口不参与阈值）
        .uart_thr_i       (8'd0         ),
        .threshold_o      (thr_r        ),
        .threshold_low_o  (thrl_r       ),
        .key_press_o      (             )
    );

    //------------------------------------------------------------------
    // 检查与激励
    //------------------------------------------------------------------
    integer ncase = 0, nerr = 0;

    task chk8;
        input [255:0] name;
        input [7:0]   got;
        input [7:0]   exp;
        begin
            ncase = ncase + 1;
            if (got !== exp) begin
                nerr = nerr + 1;
                $display("  !! FAIL: %0s = %0d（期望 %0d）@%0t", name, got, exp, $time);
            end
        end
    endtask

    task chki;
        input [255:0] name;
        input integer got;
        input integer exp;
        begin
            ncase = ncase + 1;
            if (got !== exp) begin
                nerr = nerr + 1;
                $display("  !! FAIL: %0s = %0d（期望 %0d）@%0t", name, got, exp, $time);
            end
        end
    endtask

    // thr_low 必须是 thr >> 1（Canny 双阈值的 2:1 口径），每处都顺手查一遍
    task chk_low;
        input [255:0] name;
        input [7:0]   got;
        input [7:0]   exp;
        begin
            ncase = ncase + 1;
            if (got !== {1'b0, exp[7:1]}) begin
                nerr = nerr + 1;
                $display("  !! FAIL: %0s 的 thr_low = %0d（期望 %0d）", name, got, exp >> 1);
            end
        end
    endtask

    task chk_thr;       // 快速实例：阈值 + thr_low
        input [255:0] name;
        input [7:0]   exp;
        begin
            chk8(name, thr_f, exp);
            chk_low(name, thrl_f, exp);
        end
    endtask

    task chk_thr_r;     // 真实实例：阈值 + thr_low
        input [255:0] name;
        input [7:0]   exp;
        begin
            chk8(name, thr_r, exp);
            chk_low(name, thrl_r, exp);
        end
    endtask

    // 按一下：按住 > 消抖窗口，松手后再等到电平稳定（下一按才认）
    task press_once;
        input integer idx;
        begin
            key_f[idx] = 1'b0;  repeat (2 * DB_CYC + 4) @(posedge clk_f);
            key_f[idx] = 1'b1;  repeat (2 * DB_CYC + 4) @(posedge clk_f);
        end
    endtask

    // 按住不放：同样只应产生 1 个按下沿事件
    task hold_long;
        input integer idx;
        input integer cycles;
        begin
            key_f[idx] = 1'b0;  repeat (cycles) @(posedge clk_f);
            key_f[idx] = 1'b1;  repeat (2 * DB_CYC + 4) @(posedge clk_f);
        end
    endtask

    // 抖动：只按下半个窗口宽 → 必须被忽略
    task glitch;
        input integer idx;
        begin
            key_f[idx] = 1'b0;  repeat (DB_CYC / 2) @(posedge clk_f);
            key_f[idx] = 1'b1;  repeat (2 * DB_CYC + 4) @(posedge clk_f);
        end
    endtask

    // key_press_o 的脉冲计数与"宽度"监视：
    //   n_press_wide 统计"连续两拍都为高"的次数 —— 该口按 RTL 契约必须是 1 拍脉冲，
    //   否则顶层拿它去触发别的逻辑时会被重复触发（这里先盯住）。
    integer n_press_k1 = 0, n_press_k2 = 0, n_press_wide = 0;
    reg     press_d = 0;
    always @(posedge clk_f) begin
        if (press_f[0])             n_press_k1 = n_press_k1 + 1;
        if (press_f[1])             n_press_k2 = n_press_k2 + 1;
        if (press_f != 2'b00 && press_d) n_press_wide = n_press_wide + 1;
        press_d = (press_f != 2'b00);
    end

    integer cyc_r;      // 真实实例：从按下沿到阈值变化的拍数

    //------------------------------------------------------------------
    // 主流程
    //------------------------------------------------------------------
    initial begin
        repeat (4) @(posedge clk_f);
        rst_n_f = 1;
        repeat (4) @(posedge clk_f);

        //--------------------------------------------------------------
        // 1) 复位默认值 = THR_DEFAULT：不按任何键就是上板工作点（12）
        //--------------------------------------------------------------
        chk_thr("复位阈值", 8'd12);

        //--------------------------------------------------------------
        // 2) 不按键保持：跑 50 拍，阈值和脉冲都不许动
        //--------------------------------------------------------------
        repeat (50) @(posedge clk_f);
        chk_thr("不按键保持", 8'd12);
        chk8("不按键时无脉冲", {6'b0, press_f}, 8'd0);

        //--------------------------------------------------------------
        // 3) 抖动（按下半个窗口宽）必须被忽略
        //    否则松手时的触点抖动会被当成多次按键，一按连跳好几档
        //--------------------------------------------------------------
        glitch(0); glitch(0); glitch(1);
        chk_thr("抖动不认", 8'd12);
        chki("抖动无 K1 脉冲", n_press_k1, 0);
        chki("抖动无 K2 脉冲", n_press_k2, 0);

        //--------------------------------------------------------------
        // 4) 正常按一下 K1 → 升一档（12 → 16），且正好 1 个脉冲
        //--------------------------------------------------------------
        press_once(0);
        chk_thr("K1 升一档", 8'd16);
        chki("K1 脉冲数 1", n_press_k1, 1);

        //--------------------------------------------------------------
        // 5) 按住不放 100 拍：只认按下那一刻，**不能**连续步进
        //--------------------------------------------------------------
        hold_long(0, 100);
        chk_thr("按住不放只升一档", 8'd20);
        chki("按住不放只 1 个脉冲", n_press_k1, 2);

        //--------------------------------------------------------------
        // 6) 连按到上限：饱和在 48，不回绕（回到最小值是最难查的一类错）
        //--------------------------------------------------------------
        press_once(0); press_once(0); press_once(0); press_once(0);   // 24 28 32 36
        press_once(0); press_once(0); press_once(0); press_once(0);   // 40 44 48 48
        chk_thr("连按到上限 48", 8'd48);
        press_once(0); press_once(0);
        chk_thr("上限饱和不回绕", 8'd48);

        //--------------------------------------------------------------
        // 7) K2 递减
        //--------------------------------------------------------------
        press_once(1);
        chk_thr("K2 降一档", 8'd44);
        chki("K2 脉冲数 1", n_press_k2, 1);

        //--------------------------------------------------------------
        // 8) 连按到下限：饱和在 4
        //--------------------------------------------------------------
        repeat (20) press_once(1);
        chk_thr("连按到下限 4", 8'd4);
        press_once(1);
        chk_thr("下限饱和不回绕", 8'd4);

        //--------------------------------------------------------------
        // 9) 同时按 K1 + K2：两路消抖同步完成 → case 落 default，阈值保持不动
        //    （同时按不该有方向性歧义：宁可不动，也不要随机往上或往下）
        //--------------------------------------------------------------
        key_f = 2'b00;  repeat (2 * DB_CYC + 4) @(posedge clk_f);
        key_f = 2'b11;  repeat (2 * DB_CYC + 4) @(posedge clk_f);
        chk_thr("同时按保持不变", 8'd4);

        //--------------------------------------------------------------
        // 10) uart 软写：立即生效（8 位全量程，允许越出按键范围），
        //     之后第一次按键把越界值拉回范围内 —— 顶层虽然没接串口，
        //     但这条通路是留给以后恢复串口调参的，行为要定住
        //--------------------------------------------------------------
        uart_d_f = 8'd200; uart_v_f = 1'b1; @(posedge clk_f); uart_v_f = 1'b0;
        repeat (2) @(posedge clk_f);
        chk_thr("uart 软写立即生效", 8'd200);
        press_once(1);
        chk_thr("越界值按一次 K2 回到上限 48", 8'd48);
        press_once(1);
        chk_thr("再按一次才继续降", 8'd44);

        //--------------------------------------------------------------
        // 11) 全过程：key_press_o 不许出现连续两拍高
        //--------------------------------------------------------------
        chki("key_press_o 始终是单拍脉冲", n_press_wide, 0);

        //--------------------------------------------------------------
        // 12) 真实参数（74.25 MHz / 20 ms）：时长与 CNT_MAX 必须对得上
        //     这条才是真正在验位宽/参数算式，前 11 条验的是行为
        //--------------------------------------------------------------
        repeat (4) @(posedge clk_r);
        rst_n_r = 1;
        repeat (4) @(posedge clk_r);
        chk_thr_r("真实参数复位", 8'd12);

        // a) 按 10 ms（半个窗口）：不认
        key_r[0] = 1'b0;  repeat (HALF_CYC) @(posedge clk_r);
        key_r[0] = 1'b1;  repeat (100) @(posedge clk_r);
        chk_thr_r("真实参数：10 ms 抖动不认", 8'd12);

        // b) 按住不放，量"按下沿 → 阈值变化"的拍数：
        //    期望 1 485 000（消抖）+ 2（同步）= 1 485 002，给 ±20 拍余量
        //    （量的是真实时长，不是"能不能认出来"—— 差 10 倍也算得出来）
        cyc_r = 0;
        key_r[0] = 1'b0;
        while (thr_r == 8'd12 && cyc_r < 3000000) begin
            @(posedge clk_r);
            cyc_r = cyc_r + 1;
        end
        key_r[0] = 1'b1;
        chk8("真实参数：按下后阈值变化", thr_r, 8'd16);
        chki("真实参数：按下延迟 = 20.00 ms ±20 拍",
             (cyc_r >= FULL_CYC + 2 - 20 && cyc_r <= FULL_CYC + 2 + 20) ? 1 : 0, 1);
        $display("  （实际延迟 %0d 拍 = %0.3f ms，期望 %0d 拍 = 20.000 ms）",
                 cyc_r, cyc_r * TCLK / 1000000.0, FULL_CYC + 2);
        repeat (4) @(posedge clk_r);
        chk_low("真实参数 thr_low", thrl_r, 8'd16);

        //--------------------------------------------------------------
        if (nerr == 0)
            $display("CANNY_THR_KEY PASS（%0d 个用例，0 错误）", ncase);
        else
            $display("CANNY_THR_KEY FAIL（%0d 个用例，%0d 个错误）", ncase, nerr);
        $finish;
    end

endmodule
