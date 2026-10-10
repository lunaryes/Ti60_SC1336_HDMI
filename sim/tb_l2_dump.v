`timescale 1ns / 1ps
//=====================================================================
// tb_l2_dump —— L2 逐像素验证的激励 / 转储 testbench
//---------------------------------------------------------------------
// 把一整幅源图喂给 image_pipeline_top，并把「输入像素流」与「输出像素流」
// 逐拍转储成文本文件，交给**独立实现的 Python 参考模型**
// (tools/l2_ref_model.py) 逐像素比对。这就是 PR 模板要求的 L2 验证。
//
// 【与 L1 的区别】
//   L1 (sim/tb_image_pipeline.v) = 期望值在 Verilog 里内联，16x8 小图、单一图案；
//   L2 (本文件 + Python)         = 期望值由**外部 Python 从同一幅源图独立重算**，
//                                  图更大、图案更丰富，并且同时转储两条独立参数
//                                  的流水线实例：EDGE_BINARY=1（二值边缘图）与
//                                  EDGE_BINARY=0（梯度幅值灰度图），
//                                  即不仅校验阈值化之后的 1 bit，也校验完整 mag 数值。
//
// 【图案】上半幅平滑斜坡（低梯度，考非饱和算术），下半幅确定性伪随机
//   （高梯度，考饱和到 255 的路径）。图案不需要 Python 复现——输入像素流已一并转储。
//
// 【固有偏移】输出第 m 行 = 图像第 m-1 行（垂直下移 1 行），与 docs/INTERFACE.md §6 一致。
//
// 【用法】在 repo/ 根目录：
//     iverilog -g2005 -o sim/tb_l2.vvp <src 列表> sim/tb_l2_dump.v
//     vvp sim/tb_l2.vvp +MODE=1 +THR=40
//   或直接用 sim/l2_verify.sh 一键跑全部模式/阈值组合。
//
// 【产物】sim/l2_in.txt        输入像素流（每行一个 6 位十六进制像素，仅 de_i=1 时写）
//         sim/l2_out_bin.txt   EDGE_BINARY=1 实例的输出流
//         sim/l2_out_mag.txt   EDGE_BINARY=0 实例的输出流
//         'V' 单独一行 = vsync_o 上升沿（帧分隔符）
//=====================================================================
module tb_l2_dump;

    localparam IMG_W    = 64;
    localparam IMG_H    = 48;
    localparam CLK_PER  = 10;      // 100 MHz
    localparam H_BLANK  = 4;
    localparam NFRAMES  = 4;       // 第 1 帧预热（行 bank 未满），Python 取最后一整帧

    //-----------------------------------------------------------------
    // 测试平台信号
    //-----------------------------------------------------------------
    reg         clk;
    reg         rst_n;

    reg         vsync_i;
    reg         de_i;
    reg  [23:0] rgb_i;
    reg  [7:0]  thr_i;
    reg  [1:0]  mode_i;

    wire        vsync_o_bin, de_o_bin;
    wire [23:0] rgb_o_bin;
    wire        vsync_o_mag, de_o_mag;
    wire [23:0] rgb_o_mag;

    //-----------------------------------------------------------------
    // 两条独立参数实例：二值边缘图 / 梯度幅值灰度图
    //-----------------------------------------------------------------
    image_pipeline_top #(
        .IMG_W       (IMG_W),
        .IMG_H       (IMG_H),
        .EDGE_BINARY (1)
    ) dut_bin (
        .clk_i   (clk),   .rst_ni  (rst_n),
        .vsync_i (vsync_i), .de_i  (de_i),  .rgb_i (rgb_i),
        .thr_i   (thr_i),   .mode_i (mode_i),
        .vsync_o (vsync_o_bin), .de_o (de_o_bin), .rgb_o (rgb_o_bin)
    );

    image_pipeline_top #(
        .IMG_W       (IMG_W),
        .IMG_H       (IMG_H),
        .EDGE_BINARY (0)
    ) dut_mag (
        .clk_i   (clk),   .rst_ni  (rst_n),
        .vsync_i (vsync_i), .de_i  (de_i),  .rgb_i (rgb_i),
        .thr_i   (thr_i),   .mode_i (mode_i),
        .vsync_o (vsync_o_mag), .de_o (de_o_mag), .rgb_o (rgb_o_mag)
    );

    //-----------------------------------------------------------------
    // 时钟 / 复位
    //-----------------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PER/2) clk = ~clk;

    //-----------------------------------------------------------------
    // 源图：上半幅平滑斜坡 + 下半幅确定性伪随机
    //   注意相邻行必须不同（否则行错位会被图案掩盖，这是 L1 踩过的坑）。
    //-----------------------------------------------------------------
    function [7:0] src_pix;
        input integer x;
        input integer y;
        integer v;
        begin
            if (y < IMG_H/2)
                v = (x*3 + y*5) & 32'hFF;                                  // 平滑斜坡
            else
                v = ((x*37) + (y*91) + (x*y*5) + ((x ^ y)*17)) & 32'hFF;   // 伪随机
            src_pix = v[7:0];
        end
    endfunction

    //-----------------------------------------------------------------
    // 转储文件
    //-----------------------------------------------------------------
    integer fd_in, fd_bin, fd_mag;
    integer n_in, n_bin, n_mag;

    reg vs_d_bin, vs_d_mag;

    always @(posedge clk) begin
        if (rst_n && de_i) begin
            $fwrite(fd_in, "%06h\n", rgb_i);
            n_in = n_in + 1;
        end

        vs_d_bin <= vsync_o_bin;
        vs_d_mag <= vsync_o_mag;

        if (vsync_o_bin & ~vs_d_bin) $fwrite(fd_bin, "V\n");
        if (de_o_bin) begin
            $fwrite(fd_bin, "%06h\n", rgb_o_bin);
            n_bin = n_bin + 1;
        end

        if (vsync_o_mag & ~vs_d_mag) $fwrite(fd_mag, "V\n");
        if (de_o_mag) begin
            $fwrite(fd_mag, "%06h\n", rgb_o_mag);
            n_mag = n_mag + 1;
        end
    end

    //-----------------------------------------------------------------
    // 驱动
    //-----------------------------------------------------------------
    integer x, y, f;

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
            @(posedge clk);
            vsync_i <= 1'b1;
            @(posedge clk);
            @(posedge clk);
            vsync_i <= 1'b0;

            for (y = 0; y < IMG_H; y = y + 1) begin
                for (x = 0; x < IMG_W; x = x + 1) begin
                    @(posedge clk);
                    de_i  <= 1'b1;
                    rgb_i <= {src_pix(x,y), src_pix(x,y), src_pix(x,y)};
                end
                @(posedge clk);
                de_i  <= 1'b0;
                rgb_i <= 24'h000000;
                drive_blank_hsync;
            end
        end
    endtask

    //-----------------------------------------------------------------
    // 主流程
    //-----------------------------------------------------------------
    integer mode_arg, thr_arg;

    initial begin
        if (!$value$plusargs("MODE=%d", mode_arg)) mode_arg = 1;
        if (!$value$plusargs("THR=%d",  thr_arg))  thr_arg  = 40;

        $display("========== tb_l2_dump 开始 (MODE=%0d THR=%0d) ==========", mode_arg, thr_arg);

        fd_in  = $fopen("sim/l2_in.txt",      "w");
        fd_bin = $fopen("sim/l2_out_bin.txt", "w");
        fd_mag = $fopen("sim/l2_out_mag.txt", "w");

        n_in = 0; n_bin = 0; n_mag = 0;

        rst_n   = 1'b0;
        vsync_i = 1'b0;
        de_i    = 1'b0;
        rgb_i   = 24'h000000;
        thr_i   = thr_arg[7:0];
        mode_i  = mode_arg[1:0];

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        for (f = 0; f < NFRAMES; f = f + 1)
            drive_frame;

        repeat (20) @(posedge clk);

        $fclose(fd_in);
        $fclose(fd_bin);
        $fclose(fd_mag);

        $display("转储完成：输入 %0d 像素，bin 输出 %0d 像素，mag 输出 %0d 像素",
                 n_in, n_bin, n_mag);
        $display("每帧应有 %0d 像素（%0d 帧）", IMG_W*IMG_H, NFRAMES);
        $display("========== tb_l2_dump 结束 ==========");
        $finish;
    end

    initial begin
        #(CLK_PER * 500000);
        $display("结果: *** TIMEOUT ***");
        $finish;
    end

endmodule
