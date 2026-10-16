`timescale 1ns/1ps
//=====================================================================
// canny_edge 验证台（当前对应 M3：全套 高斯→梯度→幅值→方向→NMS→双阈值滞后）
//---------------------------------------------------------------------
// 结构与真实链路一致：pix_window_3x3 → canny_edge，输入是灰度流。
// 激励图像由 sim/canny_ref.py 生成（同一套近似的 Python 参考模型），
// 本测试台只负责按时序把图像灌进去、把指定帧的输出写出来。
//
// 运行方式（见 sim/run.sh）：
//   ./obj_dir/Vtb_canny_edge +STIM=xxx.hex +OUT=xxx.txt
//
// 本地检查（不依赖 Python）：
//   1) 对外延迟：src_de 上升沿 → canny_de 上升沿 必须是 WIN_LAT + LATENCY
//   2) de = 0 的区段：mag_o / edge_o 必须为 0
//   3) 每行有效像素数 = IMG_W（列对齐、不丢不重）
//   4) 抓取从"本帧第一个输出行的行首"开始、抓满一帧（见下面的抓取注释）
//=====================================================================
module tb_canny_edge;

    parameter IMG_W = 64;
    parameter IMG_H = 32;

    // 延迟常量：改 RTL 时这里要跟着改（有意写死，避免"改了 RTL 测试台自动跟随"
    // 而失去延迟回归的意义）
    localparam WIN_LAT   = 2;                  // pix_window_3x3 冻结延迟
    localparam CANNY_LAT = 12;                 // canny_edge M3 延迟（M2/M3 同为 12）
    localparam EXP_LAT   = WIN_LAT + CANNY_LAT;

    localparam H_BLANK   = 6;                  // 行消隐（>= 2 拍的前提要求）
    localparam VS_LEAD   = 4;
    localparam VS_TAIL   = 6;
    localparam H_LINE    = IMG_W + H_BLANK;
    localparam ACT_END   = VS_LEAD + IMG_H*H_LINE;
    localparam FRAME     = ACT_END + VS_TAIL + 2;
    localparam CAP_FRAME = 2;                  // 抓第 3 帧（行缓存已填满）

    // 阈值域是内部 mag8 = mag>>3（方案 §3.8）：16 对应真实 |Gx|+|Gy| = 128。
    // ★ 与 sobel_edge 的 thr_i（先饱和到 255 再比）不可混用。
    // ★★ M5 重标定：40/20 → 16/8。可用量程只有 0..96（3x3 高斯把 Sobel 的 4A
    //    阶跃响应压成 3A，mag8 ≈ 3·ΔY/8），thr=40 实际要求 ΔY ≥ 107 = 满幅 42%，
    //    这就是"只有高对比度才出边"的根因。必须与 sim/canny_ref.py、
    //    image_pipeline_top.v 的默认值、example_top.v 的 IMG_THR_DEF 四处同步。
    localparam THR_HIGH = 16;                  // 强边阈值（ΔY ≥ 43 = 满幅 17%）
    localparam THR_LOW  = 8;                   // 弱边阈值（>= 它的才有资格被连接）

    //------------------------------------------------------------------
    // 时钟 / 复位
    //------------------------------------------------------------------
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    //------------------------------------------------------------------
    // 激励图像 + 时序
    //------------------------------------------------------------------
    reg [7:0] mem [0:IMG_W*IMG_H-1];

    reg [1023:0] stim_f, out_f;
    initial begin
        if (!$value$plusargs("STIM=%s", stim_f)) stim_f = "stim.hex";
        if (!$value$plusargs("OUT=%s",  out_f))  out_f  = "out.txt";
        $readmemh(stim_f, mem);
    end

    integer fcyc = 0, frame = 0;
    reg        src_vs = 0, src_de = 0;
    integer    src_row = 0, src_col = 0;
    reg  [7:0] src_gray = 0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fcyc <= 0; frame <= 0;
            src_vs <= 1'b0; src_de <= 1'b0;
            src_row <= 0; src_col <= 0;
        end else begin
            fcyc <= (fcyc == FRAME-1) ? 0 : fcyc + 1;
            if (fcyc == FRAME-1) frame <= frame + 1;

            src_vs <= (fcyc < ACT_END + VS_TAIL);      // 帧有效电平

            if (fcyc >= VS_LEAD && fcyc < ACT_END) begin
                src_row <= (fcyc - VS_LEAD) / H_LINE;
                if (((fcyc - VS_LEAD) % H_LINE) < IMG_W) begin
                    src_de   <= 1'b1;
                    src_col  <= (fcyc - VS_LEAD) % H_LINE;
                end else begin
                    src_de   <= 1'b0;
                end
            end else begin
                src_de <= 1'b0;
            end
        end
    end

    always @(*) begin
        if (src_de && src_row < IMG_H)
            src_gray = mem[src_row*IMG_W + src_col];
        else
            src_gray = 8'd0;
    end

    //------------------------------------------------------------------
    // DUT：pix_window_3x3 → canny_edge
    //------------------------------------------------------------------
    wire        win_vs, win_de;
    wire [7:0]  p11, p12, p13, p21, p22, p23, p31, p32, p33;

    pix_window_3x3 #(
        .IMG_W (IMG_W)
    ) u_win (
        .clk     (clk),
        .rst_n   (rst_n),
        .vsync_i (src_vs),
        .de_i    (src_de),
        .gray_i  (src_gray),
        .vsync_o (win_vs),
        .de_o    (win_de),
        .p11 (p11), .p12 (p12), .p13 (p13),
        .p21 (p21), .p22 (p22), .p23 (p23),
        .p31 (p31), .p32 (p32), .p33 (p33)
    );

    wire        canny_vs, canny_de;
    wire [7:0]  canny_mag;
    wire        canny_edge;

    canny_edge #(
        .IMG_W   (IMG_W),
        .LATENCY (CANNY_LAT)
    ) u_canny (
        .clk     (clk),
        .rst_n   (rst_n),
        .vsync_i (win_vs),
        .de_i    (win_de),
        .p11 (p11), .p12 (p12), .p13 (p13),
        .p21 (p21), .p22 (p22), .p23 (p23),
        .p31 (p31), .p32 (p32), .p33 (p33),
        .thr_high_i (THR_HIGH[7:0]),
        .thr_low_i  (THR_LOW[7:0]),
        .smooth_mode_i (2'd0),
        .vsync_o (canny_vs),
        .de_o    (canny_de),
        .mag_o   (canny_mag),
        .edge_o  (canny_edge)
    );

    //------------------------------------------------------------------
    // 检查 1：对外延迟
    //------------------------------------------------------------------
    integer de_rise_i = -1, de_rise_o = -1;
    reg src_de_d = 0, canny_de_d = 0;

    always @(posedge clk) begin
        src_de_d   <= src_de;
        canny_de_d <= canny_de;

        if (src_de && !src_de_d && de_rise_i < 0) de_rise_i = fcyc;
        if (canny_de && !canny_de_d && de_rise_i >= 0 && de_rise_o < 0 && fcyc > de_rise_i)
            de_rise_o = fcyc;
    end

    //------------------------------------------------------------------
    // 检查 2：消隐期输出必须为 0
    //------------------------------------------------------------------
    integer blank_err = 0;

    always @(posedge clk) begin
        if (rst_n && !canny_de && (canny_mag !== 8'd0 || canny_edge !== 1'b0))
            blank_err = blank_err + 1;
    end

    //------------------------------------------------------------------
    // 检查 3：每行有效像素数 + 输出抓取
    //------------------------------------------------------------------
    integer fd = 0;
    integer row_pix = 0;                 // 当前输出行的有效像素数
    integer row_cnt = 0;                 // 当前是输出第几行
    integer short_rows = 0, long_rows = 0;
    integer cap_cnt = 0;
    reg     cap_on  = 0;                 // 抓取使能

    always @(posedge clk) begin
        if (!rst_n) begin
            row_pix = 0; row_cnt = 0; cap_on = 0;
        end else begin
            if (canny_de && !canny_de_d) begin
                // 上一行结束，检查它的像素数
                if (row_cnt > 0) begin
                    if (row_pix < IMG_W) short_rows = short_rows + 1;
                    if (row_pix > IMG_W) long_rows  = long_rows  + 1;
                end
                row_pix = 0;
                row_cnt = row_cnt + 1;
            end
            if (canny_de) row_pix = row_pix + 1;

            // 抓取必须从"本帧第一个输出行的行首"开始，不能直接按帧计数器抓：
            // 上一帧最后一行的输出还有 1 拍落在本帧开头（流水线尾巴），直接抓会
            // 让整个数组整体错开 1 个像素（看起来像每行右移 1 列）。
            if (canny_de && !canny_de_d && frame == CAP_FRAME && fcyc > 4)
                cap_on = 1'b1;

            if (cap_on && canny_de && fd != 0) begin
                $fwrite(fd, "%02x %0d\n", canny_mag, canny_edge);
                cap_cnt = cap_cnt + 1;
                if (cap_cnt == IMG_W*IMG_H) cap_on = 1'b0;
            end
        end
    end

    //------------------------------------------------------------------
    // 主流程
    //------------------------------------------------------------------
    initial begin
        #20;
        rst_n = 1;
        fd = $fopen(out_f, "w");
        if (fd == 0) begin
            $display("FATAL: 打不开输出文件 %0s", out_f);
            $finish;
        end

        #(FRAME*10*(CAP_FRAME+1) + 200);

        $display("LATENCY src_de->canny_de = %0d (期望 %0d)", de_rise_o - de_rise_i, EXP_LAT);
        $display("BLANK_ERR %0d", blank_err);
        $display("ROWS %0d SHORT %0d LONG %0d", row_cnt, short_rows, long_rows);
        $display("CAPTURED %0d (期望 %0d)", cap_cnt, IMG_W*IMG_H);
        $fclose(fd);

        if ((de_rise_o - de_rise_i) != EXP_LAT) $display("FAIL: 延迟不符");
        if (blank_err != 0)                     $display("FAIL: 消隐期输出非 0");
        if (short_rows != 0 || long_rows != 0)  $display("FAIL: 行像素数不对");
        if (cap_cnt != IMG_W*IMG_H)             $display("FAIL: 抓取像素数不对");

        $finish;
    end

endmodule
