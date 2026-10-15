`timescale 1ns / 1ps
//=====================================================================
// tb_rgb_to_gray —— 灰度链的**颜色路径**回归（M5 新增）
//---------------------------------------------------------------------
// 为什么要单独测：集成回归 sim/tb_pipeline_top.v 灌的是 {g,g,g} 灰度图，
//   而 (77+150+29)*g >> 8 = g —— 颜色路径（三通道权重、通道顺序、AWB 削顶
//   后的取值）在集成回归里**从未被激励过**。M5 排查"红色格子没边"时，发现
//   自己没法用仿真回答"红块喂进去 Y 到底是多少"，只能靠离线脚本。这个台子
//   把那一步固化下来。
//
// 两组检查：
//   A) 定点字面值：把我们关心的那几个三元组（含 AWB 削顶后的红/灰/紫）的
//      期望 Y 写死，逐位相等。期望值来自 sim/color_probe.py 的同一套整数算子。
//   B) BT.601 容差：R,G,B 各取 4 档共 64 个组合，要求
//      |Y - (0.299R + 0.587G + 0.114B)| <= 1.5。这条**不是**拿 RTL 的公式
//      去比 RTL，而是拿标准去比 —— 通道接反（比如把 B 当 R）在这里必挂。
//   C) de=0 时必须输出 0（消隐期不给图像数据，与 rgb_to_gray.v 一致）。
//
// 用法（sim/run.sh 已挂）： verilator --binary --timing ... ; ./Vtb_rgb_to_gray
//=====================================================================
module tb_rgb_to_gray;

    localparam real TOL = 1.5;      // 整数近似相对 BT.601 的允许偏差
                                    // （纯蓝 28 vs 29.07 差了 1.07，见 A 组注释）

    reg  clk = 0, rst_n = 0;
    always #5 clk = ~clk;           // 100 MHz，只跑逻辑

    reg         vs = 0, hs = 0, de = 0;
    reg  [23:0] rgb = 24'h0;
    wire        vs_o, hs_o, de_o;
    wire [23:0] rgb_o;

    rgb_to_gray u_dut (
        .clk (clk), .rst_n (rst_n),
        .vs_i (vs), .hs_i (hs), .de_i (de), .rgb_i (rgb),
        .vs_o (vs_o), .hs_o (hs_o), .de_o (de_o), .rgb_o (rgb_o)
    );

    integer ncase = 0;
    integer nerr  = 0;
    integer i, j, k;
    integer r_c, g_c, b_c;

    //--------------------------------------------------------------
    // A) 定点字面值
    //--------------------------------------------------------------
    reg [23:0] avec [0:9];
    reg [ 7:0] aexp [0:9];
    initial begin
        avec[0] = 24'h000000; aexp[0] = 8'd0;     // 黑（数字笔画）
        avec[1] = 24'hFFFFFF; aexp[1] = 8'd255;   // 白
        avec[2] = 24'hFF0000; aexp[2] = 8'd76;    // 纯红
        avec[3] = 24'h00FF00; aexp[3] = 8'd149;   // 纯绿
        avec[4] = 24'h0000FF; aexp[4] = 8'd28;    // 纯蓝（BT.601 是 29.07，
                                                  //  整数近似低 1.07 —— TOL 就是为它留的）
        avec[5] = 24'h808080; aexp[5] = 8'd128;   // 中灰
        avec[6] = 24'hFF1C36; aexp[6] = 8'd99;    // 测试卡红块 AWB 后 (255,28,54)
        avec[7] = 24'hCC80C0; aexp[7] = 8'd158;   // 测试卡灰块 AWB 后 (204,128,192)
                                                  //  ★ AWB 把 128 灰拉成 (204,128,192)：
                                                  //    红/蓝通道被提亮，Y 从 128 → 158
        avec[8] = 24'hCC00FF; aexp[8] = 8'd90;    // 紫块 AWB 后 (204,0,255)
        avec[9] = 24'hFF00FF; aexp[9] = 8'd105;   // 品红
    end

    //--------------------------------------------------------------
    // 激励与检查：下降沿改激励，下一个上升沿 DUT 采样，再 +1ns 查输出
    //--------------------------------------------------------------
    task drive_check;
        input [23:0] v;
        input [7:0]  y_exp;
        input        by_expect;      // 1 = 查字面值，0 = 只喂（B 组另算）
        begin
            @(negedge clk);
            de  = 1'b1;
            vs  = 1'b1;
            rgb = v;
            @(posedge clk);
            #1;
            ncase = ncase + 1;
            if (de_o !== 1'b1 || vs_o !== 1'b1) begin
                nerr = nerr + 1;
                $display("  !! FAIL: de_o/vs_o 没有跟随（de_o=%b vs_o=%b）", de_o, vs_o);
            end else if (rgb_o[23:16] !== rgb_o[15:8] || rgb_o[15:8] !== rgb_o[7:0]) begin
                nerr = nerr + 1;
                $display("  !! FAIL: rgb_o = %06h 不是三通道相同的灰度", rgb_o);
            end else if (by_expect && rgb_o[7:0] !== y_exp) begin
                nerr = nerr + 1;
                $display("  !! FAIL: rgb_i=%06h  Y=%0d（期望 %0d）", v, rgb_o[7:0], y_exp);
            end
        end
    endtask

    real want, got;
    initial begin
        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);

        //----------------------------------------------------------
        // A) 字面值
        //----------------------------------------------------------
        for (i = 0; i < 10; i = i + 1)
            drive_check(avec[i], aexp[i], 1'b1);
        $display("  A 组（定点字面值）：%0d 个用例，累计错误 %0d", ncase, nerr);

        //----------------------------------------------------------
        // B) BT.601 容差：R/G/B 各 4 档
        //----------------------------------------------------------
        for (i = 0; i < 4; i = i + 1)
        for (j = 0; j < 4; j = j + 1)
        for (k = 0; k < 4; k = k + 1) begin
            r_c = i * 85;  g_c = j * 85;  b_c = k * 85;
            drive_check({r_c[7:0], g_c[7:0], b_c[7:0]}, 8'd0, 1'b0);
            want = 0.299 * r_c + 0.587 * g_c + 0.114 * b_c;
            got  = rgb_o[7:0];
            if (got - want > TOL || want - got > TOL) begin
                nerr = nerr + 1;
                $display("  !! FAIL: (%0d,%0d,%0d) Y=%0d 偏离 BT.601 的 %.2f 太多",
                         r_c, g_c, b_c, rgb_o[7:0], want);
            end
        end
        $display("  B 组（BT.601 容差 ±%.1f，64 个组合）：累计错误 %0d", TOL, nerr);

        //----------------------------------------------------------
        // C) 消隐期必须清零
        //----------------------------------------------------------
        @(negedge clk);
        de = 1'b0; vs = 1'b0; rgb = 24'hFFFFFF;
        @(posedge clk);
        #1;
        ncase = ncase + 1;
        if (de_o !== 1'b0 || rgb_o !== 24'h000000) begin
            nerr = nerr + 1;
            $display("  !! FAIL: de=0 时输出没清零（de_o=%b rgb_o=%06h）", de_o, rgb_o);
        end

        if (nerr == 0)
            $display("RGB2GRAY PASS（%0d 个用例，0 错误）", ncase);
        else
            $display("RGB2GRAY FAIL（%0d 个用例，%0d 个错误）", ncase, nerr);
        $finish;
    end

endmodule
