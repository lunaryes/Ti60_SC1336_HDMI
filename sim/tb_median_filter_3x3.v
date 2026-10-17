`timescale 1ns/1ps
//=====================================================================
// tb_median_filter_3x3 —— 3x3 中值滤波单元回归（2026-10-17 接入 Canny 前）
//---------------------------------------------------------------------
// 顶层集成台（tb_pipeline_top）里 DUT 与参考链用的是同一个模块，接错线能查
// 出来，但模块本身的算法口径查不出来。这里直接喂 9 个像素，验证：
//   1) 延迟 1 拍，de/vs 与 gray 同拍；
//   2) de=0 时 gray_o 清零；
//   3) IMPULSE_ONLY=1：只替换"局部极值且与中值差 >= DELTA"的中心点；
//      干净台阶边、差值不足 DELTA 的孤立点保持原值；
//   4) IMPULSE_ONLY=0：完整 3x3 中值，同样的小差值点也会被替换。
// 同一组激励同时喂两个 DUT（仅 IMPULSE_ONLY 不同），逐拍比对。
//=====================================================================
module tb_median_filter_3x3;

    localparam [7:0] DELTA = 8'd24;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg       vs = 0, de = 0;
    reg [7:0] p11, p12, p13;
    reg [7:0] p21, p22, p23;
    reg [7:0] p31, p32, p33;

    wire       vs_imp, de_imp, vs_full, de_full;
    wire [7:0] y_imp, y_full;

    median_filter_3x3 #(
        .IMPULSE_ONLY  (1),
        .IMPULSE_DELTA (DELTA)
    ) u_imp (
        .clk (clk), .rst_n (rst_n),
        .vsync_i (vs), .de_i (de),
        .p11 (p11), .p12 (p12), .p13 (p13),
        .p21 (p21), .p22 (p22), .p23 (p23),
        .p31 (p31), .p32 (p32), .p33 (p33),
        .vsync_o (vs_imp), .de_o (de_imp), .gray_o (y_imp)
    );

    median_filter_3x3 #(
        .IMPULSE_ONLY  (0),
        .IMPULSE_DELTA (DELTA)
    ) u_full (
        .clk (clk), .rst_n (rst_n),
        .vsync_i (vs), .de_i (de),
        .p11 (p11), .p12 (p12), .p13 (p13),
        .p21 (p21), .p22 (p22), .p23 (p23),
        .p31 (p31), .p32 (p32), .p33 (p33),
        .vsync_o (vs_full), .de_o (de_full), .gray_o (y_full)
    );

    integer ncase = 0, nerr = 0;

    task chk8;
        input [255:0] name;
        input [7:0]   got;
        input [7:0]   exp;
        begin
            ncase = ncase + 1;
            if (got !== exp) begin
                nerr = nerr + 1;
                $display("  !! FAIL: %0s = %0d（期望 %0d）@%0t",
                         name, got, exp, $time);
            end
        end
    endtask

    task chk1;
        input [255:0] name;
        input         got;
        input         exp;
        begin
            ncase = ncase + 1;
            if (got !== exp) begin
                nerr = nerr + 1;
                $display("  !! FAIL: %0s = %0d（期望 %0d）@%0t",
                         name, got, exp, $time);
            end
        end
    endtask

    // 在 negedge 驱动一组窗口，posedge 后检查两个 DUT 的寄存输出。
    task apply;
        input [7:0] a11, a12, a13;
        input [7:0] a21, a22, a23;
        input [7:0] a31, a32, a33;
        input [255:0] name;
        input [7:0] exp_imp;
        input [7:0] exp_full;
        begin
            @(negedge clk);
            {p11, p12, p13} = {a11, a12, a13};
            {p21, p22, p23} = {a21, a22, a23};
            {p31, p32, p33} = {a31, a32, a33};
            de = 1'b1; vs = 1'b1;
            @(posedge clk); #1;
            chk8({name, " (impulse)"},  y_imp,  exp_imp);
            chk8({name, " (full)"},     y_full, exp_full);
            chk1({name, " de 延迟 1 拍"}, de_imp, 1'b1);
            chk1({name, " vs 延迟 1 拍"}, vs_imp, 1'b1);
            @(negedge clk);
            de = 1'b0;
        end
    endtask

    initial begin
        {p11, p12, p13} = 24'd0;
        {p21, p22, p23} = 24'd0;
        {p31, p32, p33} = 24'd0;

        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);

        // 1) 均匀场：两个 DUT 都原样输出
        apply(8'd50, 8'd50, 8'd50,
              8'd50, 8'd50, 8'd50,
              8'd50, 8'd50, 8'd50,
              "uniform", 8'd50, 8'd50);

        // 2) 孤立亮点：中心是局部最大、差值 40 >= 24 → 两模式都替换为中值
        apply(8'd50, 8'd50, 8'd50,
              8'd50, 8'd90, 8'd50,
              8'd50, 8'd50, 8'd50,
              "bright impulse", 8'd50, 8'd50);

        // 3) 孤立暗点：差值 30 >= 24 → 两模式都替换
        apply(8'd50, 8'd50, 8'd50,
              8'd50, 8'd20, 8'd50,
              8'd50, 8'd50, 8'd50,
              "dark impulse", 8'd50, 8'd50);

        // 4) 小差值孤立点：中心 60、邻域 50，差值 10 < 24
        //    impulse 模式保持 60（不是"噪声"）；full 模式仍取中值 50
        apply(8'd50, 8'd50, 8'd50,
              8'd50, 8'd60, 8'd50,
              8'd50, 8'd50, 8'd50,
              "small delta", 8'd60, 8'd50);

        // 5) 干净竖直台阶边：中心与中值同值 → impulse 模式必须恒等
        apply(8'd40, 8'd60, 8'd60,
              8'd40, 8'd60, 8'd60,
              8'd40, 8'd60, 8'd60,
              "clean step", 8'd60, 8'd60);

        // 6) 斜边（中心值处于中值位置）：impulse 模式恒等
        apply(8'd40, 8'd40, 8'd60,
              8'd40, 8'd60, 8'd70,
              8'd60, 8'd70, 8'd70,
              "diagonal edge", 8'd60, 8'd60);

        // 7) de=0：gray_o 必须清零，de/vs 仍按 1 拍延迟
        @(negedge clk);
        {p11, p12, p13} = {8'd10, 8'd20, 8'd30};
        {p21, p22, p23} = {8'd40, 8'd90, 8'd60};
        {p31, p32, p33} = {8'd70, 8'd80, 8'd50};
        de = 1'b0; vs = 1'b1;
        @(posedge clk); #1;
        chk8("de=0 时 gray_o 清零 (impulse)", y_imp,  8'd0);
        chk8("de=0 时 gray_o 清零 (full)",    y_full, 8'd0);
        chk1("de=0 时 de_o 为 0",             de_imp, 1'b0);
        chk1("de=0 时 vs_o 仍延迟 1 拍",      vs_imp, 1'b1);

        if (nerr == 0)
            $display("MEDIAN PASS（%0d 个用例，0 错误）", ncase);
        else
            $display("MEDIAN FAIL（%0d 个用例，%0d 个错误）", ncase, nerr);
        $finish;
    end

endmodule
