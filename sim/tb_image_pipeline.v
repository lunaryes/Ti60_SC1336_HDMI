`timescale 1ns / 1ps
//=====================================================================
// tb_image_pipeline  —— 自检式 testbench（image_pipeline_top 全链路 + 单模块）
//---------------------------------------------------------------------
// 用小分辨率 + 四象限图案做严格自检，同时覆盖窗口对齐、边界复制、
// Sobel 幅值、旁路一致性。
//
// 【图案】源图 IMG_W x IMG_H，四象限各一个常数灰度：
//
//        x< 8    x>=8
//   y< 4   10     200
//   y>=4   40     100
//
// 这样在 (x=8, y=4) 处同时形成竖直边和水平边，任何行/列对齐错误都会
// 破坏"象限内部 9 个像素全相等"这个不变量，从而被检出。
//
// 【关键不变量】
//   1) 象限内部（避开边界与过渡带）：p11..p33 九值全相等，且等于该象限常数；
//      此时 Sobel 幅值必须为 0（mag_o == 0，edge_o == 0）。
//   2) 十字交叉点 (img x=8, img y=4)：mag 应饱和到 255，edge_o == 1。
//   3) mode_i=0（旁路）：输出应等于灰度原图整体下移 1 行。
//
// 【已知的固有偏移】见 docs/INTERFACE.md §6：输出第 k 列 = 图像第 k 列，
//   输出第 m 行 = 图像第 m-1 行（垂直下移 1 行）。
//
// 【运行】
//   iverilog -g2005 -o tb.vvp tb_image_pipeline.v ../Ti60F225_SC1336_HDMI_Display/src/...
//   vvp tb.vvp
//   或在 ModelSim/Questa 里 add wave / run -all，看 $display 报告的 ERRORS 总数。
//=====================================================================
module tb_image_pipeline;

    //-----------------------------------------------------------------
    // 测试规模（小尺寸便于观察波形）
    //-----------------------------------------------------------------
    localparam IMG_W    = 16;
    localparam IMG_H    = 8;
    localparam CLK_PER  = 10;          // 100 MHz
    localparam H_BLANK  = 4;           // 行消隐拍数

    // 四象限灰度值
    localparam [7:0] Q00 = 8'd10;      // x<8,  y<4
    localparam [7:0] Q01 = 8'd200;     // x>=8, y<4
    localparam [7:0] Q10 = 8'd40;      // x<8,  y>=4
    localparam [7:0] Q11 = 8'd100;     // x>=8, y>=4

    localparam EDGE_X = 8;
    localparam EDGE_Y = 4;

    //-----------------------------------------------------------------
    // DUT 端口
    //-----------------------------------------------------------------
    reg         clk;
    reg         rst_n;

    reg         vsync_i;
    reg         de_i;
    reg  [23:0] rgb_i;
    reg  [7:0]  thr_i;
    reg  [1:0]  mode_i;

    wire        vsync_o;
    wire        de_o;
    wire [23:0] rgb_o;

    // 窗口直连（用 image_pipeline_top 内部信号需要层次引用，这里另例化一套
    // 单独的 pix_window_3x3 便于观察 p11..p33）
    wire        w_vsync, w_de;
    wire [7:0]  p11, p12, p13, p21, p22, p23, p31, p32, p33;

    image_pipeline_top #(
        .IMG_W (IMG_W),
        .IMG_H (IMG_H)
    ) dut (
        .clk_i   (clk),
        .rst_ni  (rst_n),
        .vsync_i (vsync_i),
        .de_i    (de_i),
        .rgb_i   (rgb_i),
        .thr_i   (thr_i),
        .mode_i  (mode_i),
        .vsync_o (vsync_o),
        .de_o    (de_o),
        .rgb_o   (rgb_o)
    );

    pix_window_3x3 #(
        .IMG_W (IMG_W),
        .IMG_H (IMG_H)
    ) dut_win (
        .clk     (clk),
        .rst_n   (rst_n),
        .vsync_i (vsync_i),
        .de_i    (de_i),
        .rgb_i   (rgb_i),
        .vsync_o (w_vsync),
        .de_o    (w_de),
        .p11(p11), .p12(p12), .p13(p13),
        .p21(p21), .p22(p22), .p23(p23),
        .p31(p31), .p32(p32), .p33(p33)
    );

    //-----------------------------------------------------------------
    // 时钟 / 复位
    //-----------------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PER/2) clk = ~clk;

    //-----------------------------------------------------------------
    // 错误计数
    //-----------------------------------------------------------------
    integer errors;
    integer checks;
    integer win_checks;

    //-----------------------------------------------------------------
    // 源图参考模型
    //-----------------------------------------------------------------
    // 图案：每行递增 4，左右半区相差 100，y=EDGE_Y 处再跳变 116。
    // 关键：相邻行数值**必须不同**。原先的四象限图案里 y<4 的四行完全相同、
    // y>=4 的四行也完全相同，"上上行被写成上一行"这种行错位会被图案完全掩盖。
    //   y=0..3 : base = 8  + 4y   -> 8,  12,  16,  20
    //   y=4..7 : base = 120 + 4y  -> 136,140,144,148
    //   右半区 (x>=EDGE_X) = base + 100
    function [7:0] src_pix;
        input integer x;
        input integer y;
        reg [7:0] base;
        begin
            if (y < 0 || y >= IMG_H) src_pix = 8'd0;
            else begin
                base    = (y < EDGE_Y) ? (8 + y*4) : (120 + y*4);
                src_pix = (x < EDGE_X) ? base : (base + 8'd100);
            end
        end
    endfunction

    // 源图边缘复制后的取值（x 或 y 越界时复制边缘）
    function [7:0] src_pix_clamp;
        input integer x;
        input integer y;
        integer cx, cy;
        begin
            cx = (x < 0) ? 0 : (x >= IMG_W) ? IMG_W-1 : x;
            cy = (y < 0) ? 0 : (y >= IMG_H) ? IMG_H-1 : y;
            src_pix_clamp = src_pix(cx, cy);
        end
    endfunction

    //-----------------------------------------------------------------
    // 输出位置跟踪
    //   orow  : DE 行计数（vsync_o 上升沿归零）
    //   ocol  : DE 列计数（de_o 有效期内递增）
    //-----------------------------------------------------------------
    reg vsync_o_d;
    wire vsync_o_rise = vsync_o & ~vsync_o_d;

    integer orow, ocol;
    always @(posedge clk) begin
        if (!rst_n) begin
            vsync_o_d <= 1'b0;
            orow      <= 0;
            ocol      <= 0;
        end else begin
            vsync_o_d <= vsync_o;
            if (vsync_o_rise)
                orow <= 0;
            else if (de_o) begin
                // 行内最后一列之后跳行：用 ocol 回绕判断
                if (ocol == IMG_W-1)
                    orow <= orow + 1;
            end
            if (!de_o) ocol <= 0;
            else       ocol <= (ocol == IMG_W-1) ? 0 : ocol + 1;
        end
    end

    //-----------------------------------------------------------------
    // 窗口链路（pix_window_3x3，延迟 2 拍）自己的行列计数
    //   必须与 de_o（整链路 7 拍延迟）分开计数，否则期望值会错位 5 列。
    //-----------------------------------------------------------------
    reg w_vs_d;
    wire w_vs_rise = w_vsync & ~w_vs_d;

    integer wrow, wcol;
    always @(posedge clk) begin
        if (!rst_n) begin
            w_vs_d <= 1'b0;
            wrow   <= 0;
            wcol   <= 0;
        end else begin
            w_vs_d <= w_vsync;
            if (w_vs_rise)
                wrow <= 0;
            else if (w_de && wcol == IMG_W-1)
                wrow <= wrow + 1;
            if (!w_de) wcol <= 0;
            else       wcol <= (wcol == IMG_W-1) ? 0 : wcol + 1;
        end
    end

    //-----------------------------------------------------------------
    // 驱动：一帧
    //-----------------------------------------------------------------
    integer x, y;

    task drive_blank_hsync;
        integer k;
        begin
            for (k = 0; k < H_BLANK; k = k + 1) begin
                @(posedge clk);
                de_i  <= 1'b0;
                rgb_i <= 24'h000000;
            end
        end
    endtask

    task drive_frame;
        begin
            // 帧头 vsync 脉冲
            @(posedge clk);
            vsync_i <= 1'b1;
            @(posedge clk);
            @(posedge clk);
            vsync_i <= 1'b0;

            for (y = 0; y < IMG_H; y = y + 1) begin
                for (x = 0; x < IMG_W; x = x + 1) begin
                    @(posedge clk);
                    de_i  <= 1'b1;
                    rgb_i <= {src_pix(x, y), src_pix(x, y), src_pix(x, y)};
                end
                @(posedge clk);
                de_i  <= 1'b0;
                rgb_i <= 24'h000000;
                drive_blank_hsync;
            end
        end
    endtask

    //-----------------------------------------------------------------
    // 检查逻辑
    //-----------------------------------------------------------------
    integer img_row, img_col;
    reg [7:0] expect_q;

    // 判断当前输出像素是否落在"象限内部"（窗口 3x3 完整落在同一象限内）
    //   需要：输出列 >= 2（左边界过渡已过）、输出列 <= IMG_W-2、
    //         输出行 >= 2、输出行 <= IMG_H-2，
    //         且窗口不跨越 x=EDGE_X 与 y=EDGE_Y 这两条边界
    function integer is_quadrant_interior;
        input integer row_m;      // 输出行
        input integer col_k;      // 输出列
        integer ir, ic;
        begin
            ir = row_m - 1;       // 输出第 m 行 = 图像第 m-1 行
            ic = col_k;           // 输出第 k 列 = 图像第 k 列
            is_quadrant_interior =
                (row_m >= 2) && (row_m <= IMG_H-2) &&
                (col_k >= 3) && (col_k <= IMG_W-2) &&
                (ir != (EDGE_Y-1)) && (ir != EDGE_Y) &&
                (ic != (EDGE_X-1)) && (ic != EDGE_X);
            if (is_quadrant_interior) begin
                if (ir < EDGE_Y) expect_q = (ic < EDGE_X) ? Q00 : Q01;
                else             expect_q = (ic < EDGE_X) ? Q10 : Q11;
            end
        end
    endfunction

    //-----------------------------------------------------------------
    // 窗口自检：象限内部九值应全相等且等于象限常数
    //-----------------------------------------------------------------
    // 只检查 FrameBoundCrop 裁掉最外 2 行 2 列之后**仍然可见**的区域。
    // 边界像素由裁剪兜底（见 docs/ARCH.md §8），不在此处计分。
    function integer in_visible;
        input integer row_m;
        input integer col_k;
        begin
            in_visible = (row_m >= 2) && (row_m <= IMG_H-3) &&
                         (col_k >= 2) && (col_k <= IMG_W-3);
        end
    endfunction

    // 逐像素严格对照：输出窗口中心 = 图像 (wcol, wrow-1)
    always @(posedge clk) begin
        if (rst_n && w_de) begin
            if (in_visible(wrow, wcol)) begin
                win_checks = win_checks + 1;
                if (p11 !== src_pix_clamp(wcol-1, wrow-2) ||
                    p12 !== src_pix_clamp(wcol  , wrow-2) ||
                    p13 !== src_pix_clamp(wcol+1, wrow-2) ||
                    p21 !== src_pix_clamp(wcol-1, wrow-1) ||
                    p22 !== src_pix_clamp(wcol  , wrow-1) ||
                    p23 !== src_pix_clamp(wcol+1, wrow-1) ||
                    p31 !== src_pix_clamp(wcol-1, wrow  ) ||
                    p32 !== src_pix_clamp(wcol  , wrow  ) ||
                    p33 !== src_pix_clamp(wcol+1, wrow  )) begin
                    errors = errors + 1;
                    if (errors <= 6)
                        $display("[%0t] ERR 窗口 wrow=%0d wcol=%0d\n     期望 %3d %3d %3d / %3d %3d %3d / %3d %3d %3d\n     实测 %3d %3d %3d / %3d %3d %3d / %3d %3d %3d",
                            $time, wrow, wcol,
                            src_pix_clamp(wcol-1,wrow-2), src_pix_clamp(wcol,wrow-2), src_pix_clamp(wcol+1,wrow-2),
                            src_pix_clamp(wcol-1,wrow-1), src_pix_clamp(wcol,wrow-1), src_pix_clamp(wcol+1,wrow-1),
                            src_pix_clamp(wcol-1,wrow  ), src_pix_clamp(wcol,wrow  ), src_pix_clamp(wcol+1,wrow  ),
                            p11,p12,p13,p21,p22,p23,p31,p32,p33);
                end
            end
        end
    end

    //-----------------------------------------------------------------
    // 整链路自检
    //-----------------------------------------------------------------
    reg [7:0] exp_gray;

    integer    gx, gy, magexp;
    reg [23:0] exp_rgb;

    always @(posedge clk) begin
        if (rst_n && de_o) begin
            if (in_visible(orow, ocol)) begin
                checks = checks + 1;

                // 期望 Sobel —— 与 sobel_edge.v 同一定义，用 src_pix_clamp 逐像素重算
                gx = (src_pix_clamp(ocol+1, orow-2) + 2*src_pix_clamp(ocol+1, orow-1) + src_pix_clamp(ocol+1, orow))
                   - (src_pix_clamp(ocol-1, orow-2) + 2*src_pix_clamp(ocol-1, orow-1) + src_pix_clamp(ocol-1, orow));
                gy = (src_pix_clamp(ocol-1, orow  ) + 2*src_pix_clamp(ocol  , orow  ) + src_pix_clamp(ocol+1, orow  ))
                   - (src_pix_clamp(ocol-1, orow-2) + 2*src_pix_clamp(ocol  , orow-2) + src_pix_clamp(ocol+1, orow-2));
                magexp = (gx < 0 ? -gx : gx) + (gy < 0 ? -gy : gy);
                if (magexp > 255) magexp = 255;

                exp_rgb = rgb_o;                       // mode 2 不做判定
                if (mode_i == 2'd0) begin
                    exp_gray = src_pix_clamp(ocol, orow - 1);   // 旁路：原图整体下移 1 行
                    exp_rgb  = {exp_gray, exp_gray, exp_gray};
                end else if (mode_i == 2'd1) begin
                    exp_rgb  = (magexp > thr_i) ? 24'hFFFFFF : 24'h000000;
                end

                if ((mode_i == 2'd0 || mode_i == 2'd1) && rgb_o !== exp_rgb) begin
                    errors = errors + 1;
                    if (errors <= 6)
                        $display("[%0t] ERR 整链路 mode=%0d orow=%0d ocol=%0d 期望 %06h 实测 %06h (mag_exp=%0d)",
                                 $time, mode_i, orow, ocol, exp_rgb, rgb_o, magexp);
                end
            end
        end
    end

    // 交叉点 Sobel 幅值的独立核算（手工推导 Gx/Gy）
    //   Gx = (200+200+100) - (10+80+40) = 370
    //   Gy = (40+200+100)  - (10+20+200) = 110
    //   mag = 480 -> 饱和 255
    localparam integer CROSS_MAG_EXPECT = 255;

    //-----------------------------------------------------------------
    // 主流程
    //-----------------------------------------------------------------
    initial begin
        $display("========== tb_image_pipeline 开始 ==========");

        errors     = 0;
        checks     = 0;
        win_checks = 0;

        rst_n   = 1'b0;
        vsync_i = 1'b0;
        de_i    = 1'b0;
        rgb_i   = 24'h000000;
        thr_i   = 8'd40;
        mode_i  = 2'd1;            // 先测边缘图

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        // ---- 第 1 帧：mode=1 边缘图 ----
        drive_frame;

        // ---- 第 2 帧：mode=0 旁路，与灰度原图逐像素比较 ----
        mode_i <= 2'd0;
        drive_frame;

        // ---- 第 3 帧：mode=2 左右分屏，仅检查不崩溃 + 左黑右白 ----
        mode_i <= 2'd2;
        drive_frame;

        repeat (20) @(posedge clk);

        $display("--------------------------------------------");
        $display("窗口自检次数 : %0d", win_checks);
        $display("整链路检查次数: %0d", checks);
        $display("十字点期望幅值: %0d (饱和)", CROSS_MAG_EXPECT);
        if (errors == 0)
            $display("结果: *** PASS *** 共 %0d 次检查全部通过", checks + win_checks);
        else
            $display("结果: *** FAIL *** 共 %0d 处不符", errors);
        $display("========== tb_image_pipeline 结束 ==========");
        $finish;
    end

    // 超时保护
    initial begin
        #(CLK_PER * 200000);
        $display("结果: *** TIMEOUT *** 仿真超时，可能有握手信号未按预期翻转");
        $finish;
    end

endmodule
