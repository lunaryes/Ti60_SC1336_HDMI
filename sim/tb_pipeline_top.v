`timescale 1ns/1ps
//=====================================================================
// image_pipeline_top 验证台（M4：接入 Canny 之后的集成回归）
//---------------------------------------------------------------------
// 激励：把灰图当 {g,g,g} 灌进去 —— rgb_to_gray 的 Y = (77+150+29)*g>>8 = g，
//       正好原值还原，所以边缘图可以和 tb_canny_edge 用同一套 .hex 图样。
// 结构：同一个输入同时喂
//         · u_bin —— EDGE_BINARY = 1（真实工程配置）
//         · u_mag —— EDGE_BINARY = 0（{mag,mag,mag}）
//         · 旁路参考链 pix_window_3x3 + canny_edge（阈值 40/20，与顶层
//           thr_i = 40 → 40/20 的映射一致，映射写错这里就会报）
// 自检项（全在 Verilog 里，不依赖 Python；算法本身的精度归 tb_canny_edge.v）：
//   1) 延迟：src_de 上升沿 → bin_de 上升沿 = IMG_LAT；vsync 同理且**高电平宽度
//      不变**（送到 rgb2dvi 的 VS 不能被压窄，否则显示器锁不住帧）
//   2) mode 0 旁路：bin_rgb 与 rgb_i 延迟 IMG_LAT 逐像素一致，**每个时钟沿都比**
//      （含消隐期：原图路径是纯寄存器延迟，image_mix 又不在 de=0 时清零）
//   3) mode 1：bin_rgb = 参考链 edge 的 {FF,FF,FF}/0；u_mag = {mag,mag,mag}
//   4) mode 2/3：第 < IMG_W/2 列 = 原图、其余 = 边缘图，逐行逐点查
//   5) 帧间靠 mode 只在帧首切换，输出与输入逐拍对齐（不做帧号对齐）
//
// 运行： ./obj_dir/Vtb_pipeline_top +STIM=xxx.hex     （见 sim/run.sh）
//=====================================================================
module tb_pipeline_top;

    parameter IMG_W = 64;
    parameter IMG_H = 32;

    // 改 RTL 延迟时这里要跟着改（有意写死，避免"改了 RTL 测试台自动跟随"）
    localparam IMG_LAT  = 16;                  // image_pipeline_top.LATENCY
                                               // （内部 = 原图 15 拍 + image_mix 1 拍）
    localparam THR_HIGH = 8'd40;               // = 顶层 thr_i
    localparam THR_LOW  = 8'd20;               // = 顶层内部 thr_i >> 1

    localparam H_BLANK  = 6;
    localparam VS_LEAD  = 4;
    localparam VS_TAIL  = 6;
    localparam H_LINE   = IMG_W + H_BLANK;
    localparam ACT_END  = VS_LEAD + IMG_H*H_LINE;
    localparam FRAME    = ACT_END + VS_TAIL + 2;
    localparam NFRAME   = 6;                   // 帧 2/3/4 分别是 mode 1/2/3

    localparam HALF_W   = IMG_W / 2;           // 分屏分界列（与 image_mix 一致）

    //------------------------------------------------------------------
    // 时钟 / 复位
    //------------------------------------------------------------------
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    //------------------------------------------------------------------
    // 激励图像 + 时序（与 tb_canny_edge 同一套）
    //------------------------------------------------------------------
    reg [7:0] mem [0:IMG_W*IMG_H-1];

    reg [1023:0] stim_f;
    initial begin
        if (!$value$plusargs("STIM=%s", stim_f)) stim_f = "stim.hex";
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

            src_vs <= (fcyc < ACT_END + VS_TAIL);

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

    wire [23:0] src_rgb = {src_gray, src_gray, src_gray};

    //------------------------------------------------------------------
    // mode 调度：帧 0/1 = 0（旁路）、2 = 1（边缘）、3 = 2（分屏）、4 = 3（同 2）
    //   只在帧首切换，切换点落在帧消隐里，输出侧逐拍对齐不受影响
    //------------------------------------------------------------------
    reg [1:0] mode = 2'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)         mode <= 2'd0;
        else if (fcyc == 0) mode <= (frame == 2) ? 2'd1 :
                                   (frame == 3) ? 2'd2 :
                                   (frame == 4) ? 2'd3 : 2'd0;
    end

    //------------------------------------------------------------------
    // DUT 1：EDGE_BINARY = 1（真实工程配置）
    //------------------------------------------------------------------
    wire        bin_vs, bin_de;
    wire [23:0] bin_rgb;

    image_pipeline_top #(
        .IMG_W (IMG_W), .IMG_H (IMG_H),
        .EDGE_BINARY (1), .LATENCY (IMG_LAT)
    ) u_bin (
        .clk_i (clk), .rst_ni (rst_n),
        .vsync_i (src_vs), .de_i (src_de), .rgb_i (src_rgb),
        .thr_i (THR_HIGH), .mode_i (mode),
        .vsync_o (bin_vs), .de_o (bin_de), .rgb_o (bin_rgb)
    );

    //------------------------------------------------------------------
    // DUT 2：EDGE_BINARY = 0（{mag,mag,mag}），只查 mode 1
    //------------------------------------------------------------------
    wire        mag_vs, mag_de;
    wire [23:0] mag_rgb;

    image_pipeline_top #(
        .IMG_W (IMG_W), .IMG_H (IMG_H),
        .EDGE_BINARY (0), .LATENCY (IMG_LAT)
    ) u_mag (
        .clk_i (clk), .rst_ni (rst_n),
        .vsync_i (src_vs), .de_i (src_de), .rgb_i (src_rgb),
        .thr_i (THR_HIGH), .mode_i (mode),
        .vsync_o (mag_vs), .de_o (mag_de), .rgb_o (mag_rgb)
    );

    //------------------------------------------------------------------
    // 旁路参考链：与顶层内部那条 edge 链逐拍同拍。
    //   第一级要照抄 rgb_to_gray 的 1 拍（顶层喂给 pix_window 的是灰度寄存后的
    //   值，不是 rgb_i），否则整条参考链会早 1 拍，mag 图会在每行的跳变处失配。
    //------------------------------------------------------------------
    reg src_vs_q = 0, src_de_q = 0;
    reg [7:0] src_gray_q = 0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            src_vs_q <= 1'b0; src_de_q <= 1'b0; src_gray_q <= 8'd0;
        end else begin
            src_vs_q <= src_vs; src_de_q <= src_de; src_gray_q <= src_gray;
        end
    end

    wire        rwin_vs, rwin_de;
    wire [7:0]  rp11, rp12, rp13, rp21, rp22, rp23, rp31, rp32, rp33;

    pix_window_3x3 #(.IMG_W (IMG_W)) u_ref_win (
        .clk (clk), .rst_n (rst_n),
        .vsync_i (src_vs_q), .de_i (src_de_q), .gray_i (src_gray_q),
        .vsync_o (rwin_vs), .de_o (rwin_de),
        .p11 (rp11), .p12 (rp12), .p13 (rp13),
        .p21 (rp21), .p22 (rp22), .p23 (rp23),
        .p31 (rp31), .p32 (rp32), .p33 (rp33)
    );

    wire        ref_vs, ref_de, ref_edge;
    wire [7:0]  ref_mag;

    canny_edge #(.IMG_W (IMG_W), .LATENCY (12)) u_ref_canny (
        .clk (clk), .rst_n (rst_n),
        .vsync_i (rwin_vs), .de_i (rwin_de),
        .p11 (rp11), .p12 (rp12), .p13 (rp13),
        .p21 (rp21), .p22 (rp22), .p23 (rp23),
        .p31 (rp31), .p32 (rp32), .p33 (rp33),
        .thr_high_i (THR_HIGH), .thr_low_i (THR_LOW),
        .vsync_o (ref_vs), .de_o (ref_de),
        .mag_o (ref_mag), .edge_o (ref_edge)
    );

    // 参考链的输出还要再晚 1 拍才是顶层出口：中间隔着 image_mix 那唯一一级输出
    // 寄存器（它不按 de 清零，所以连消隐期也逐拍对应）
    reg [23:0] ref_edge_rgb = 24'h000000;
    reg [23:0] ref_mag_rgb  = 24'h000000;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ref_edge_rgb <= 24'h000000;
            ref_mag_rgb  <= 24'h000000;
        end else begin
            ref_edge_rgb <= ref_edge ? 24'hFFFFFF : 24'h000000;
            ref_mag_rgb  <= {ref_mag, ref_mag, ref_mag};
        end
    end

    //------------------------------------------------------------------
    // 原图黄金值：rgb_i 延迟 IMG_LAT 拍（顶层内部 ORIG_LAT 拍 + image_mix 1 拍）
    //------------------------------------------------------------------
    reg [23:0] src_dly [0:IMG_LAT-1];

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            for (k = 0; k < IMG_LAT; k = k + 1) src_dly[k] <= 24'h000000;
        else begin
            src_dly[0] <= src_rgb;
            for (k = 1; k < IMG_LAT; k = k + 1) src_dly[k] <= src_dly[k-1];
        end
    end

    wire [23:0] ref_orig_rgb = src_dly[IMG_LAT-1];

    //------------------------------------------------------------------
    // 检查 1：延迟 + vsync 高电平宽度
    //------------------------------------------------------------------
    integer de_rise_i = -1, de_rise_o = -1;
    reg     src_de_d = 0, bin_de_d = 0;

    integer vs_i_rise = -1, vs_o_rise = -1;
    integer vs_i_hi = 0, vs_o_hi = 0, vs_i_last = 0, vs_o_last = 0;
    reg     src_vs_d = 0, bin_vs_d = 0;
    integer vs_width_err = 0;

    always @(posedge clk) begin
        src_de_d <= src_de;
        bin_de_d <= bin_de;
        src_vs_d <= src_vs;
        bin_vs_d <= bin_vs;

        if (src_de && !src_de_d && de_rise_i < 0) de_rise_i = fcyc;
        if (bin_de && !bin_de_d && de_rise_i >= 0 && de_rise_o < 0 && fcyc > de_rise_i)
            de_rise_o = fcyc;

        // vsync 上升沿
        if (src_vs && !src_vs_d && vs_i_rise < 0) vs_i_rise = fcyc;
        if (bin_vs && !bin_vs_d && vs_i_rise >= 0 && vs_o_rise < 0 && fcyc > vs_i_rise)
            vs_o_rise = fcyc;

        // vsync 高电平拍数（上升沿到下降沿，与平移无关）
        if (src_vs) vs_i_hi = vs_i_hi + 1;
        if (bin_vs) vs_o_hi = vs_o_hi + 1;
        if (src_vs_d && !src_vs) begin
            if (vs_i_last != 0 && vs_i_last != vs_i_hi) vs_width_err = vs_width_err + 1;
            vs_i_last = vs_i_hi; vs_i_hi = 0;
        end
        if (bin_vs_d && !bin_vs) begin
            if (vs_o_last != 0 && vs_o_last != vs_i_last) vs_width_err = vs_width_err + 1;
            vs_o_last = vs_o_hi; vs_o_hi = 0;
        end
    end

    //------------------------------------------------------------------
    // 检查 2..4：按 mode 比对（mode_d = 产生这一拍输出的那个 mode）
    //------------------------------------------------------------------
    reg [1:0] mode_d = 2'd0;

    integer err_bypass = 0, err_edge = 0, err_mag = 0, err_split = 0;
    integer out_pix = 0;                       // 本行已输出的像素数（1 基计数）
    integer out_col = 0;                       // 本行当前输出像素的 0 基列号
    integer row_cnt = 0, short_rows = 0, long_rows = 0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode_d <= 2'd0; out_pix = 0; out_col = 0; row_cnt = 0;
        end else begin
            mode_d <= mode;


            // 行内计数（与 image_mix 的 col_cnt 对齐：输出第 k 拍 = 第 k 列）
            if (bin_de && !bin_de_d) begin
                if (row_cnt > 0) begin
                    if (out_pix < IMG_W) short_rows = short_rows + 1;
                    if (out_pix > IMG_W) long_rows  = long_rows  + 1;
                end
                out_pix = 0;
                out_col = 0;
                row_cnt = row_cnt + 1;
            end else if (bin_de) begin
                out_col = out_col + 1;
            end
            if (bin_de) out_pix = out_pix + 1;

            case (mode_d)
                2'd0: begin
                    // 旁路：每个沿都比，含消隐期
                    if (bin_rgb !== ref_orig_rgb) err_bypass = err_bypass + 1;
                end
                2'd1: begin
                    if (bin_rgb !== ref_edge_rgb) err_edge = err_edge + 1;
                    if (mag_rgb !== ref_mag_rgb)  err_mag  = err_mag  + 1;
                end
                2'd2, 2'd3: begin
                    if (bin_de) begin
                        if (out_col < HALF_W) begin
                            if (bin_rgb !== ref_orig_rgb) err_split = err_split + 1;
                        end else begin
                            if (bin_rgb !== ref_edge_rgb) err_split = err_split + 1;
                        end
                    end
                end
                default: ;
            endcase
        end
    end

    //------------------------------------------------------------------
    // 主流程
    //------------------------------------------------------------------
    integer fails = 0;

    initial begin
        #20;
        rst_n = 1;

        #(FRAME*10*NFRAME + 200);

        $display("LATENCY  src_de->bin_de = %0d (期望 %0d)", de_rise_o - de_rise_i, IMG_LAT);
        $display("LATENCY  src_vs->bin_vs = %0d (期望 %0d)", vs_o_rise - vs_i_rise, IMG_LAT);
        $display("VSYNC    高电平宽度不符的帧数 %0d", vs_width_err);
        $display("ROWS     %0d SHORT %0d LONG %0d", row_cnt, short_rows, long_rows);
        $display("MODE0    旁路逐像素失配 %0d 点", err_bypass);
        $display("MODE1    边缘图失配 %0d 点 / mag 图失配 %0d 点", err_edge, err_mag);
        $display("MODE2/3  分屏失配 %0d 点", err_split);

        if ((de_rise_o - de_rise_i) != IMG_LAT)    begin fails=fails+1; $display("FAIL: de 延迟不符"); end
        if ((vs_o_rise - vs_i_rise)  != IMG_LAT)   begin fails=fails+1; $display("FAIL: vsync 延迟不符"); end
        if (vs_width_err != 0)                     begin fails=fails+1; $display("FAIL: vsync 宽度变了"); end
        if (short_rows != 0 || long_rows != 0)     begin fails=fails+1; $display("FAIL: 每行像素数不对"); end
        if (err_bypass != 0)                       begin fails=fails+1; $display("FAIL: mode 0 旁路不逐像素一致"); end
        if (err_edge != 0 || err_mag != 0)         begin fails=fails+1; $display("FAIL: mode 1 边缘图/mag 图不符"); end
        if (err_split != 0)                        begin fails=fails+1; $display("FAIL: mode 2/3 分屏不符"); end

        if (fails == 0) $display("TOP PASS");
        else            $display("TOP FAIL (%0d 项)", fails);
        $finish;
    end

endmodule
