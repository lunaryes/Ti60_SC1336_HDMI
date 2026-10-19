`timescale 1ns/1ps
//=====================================================================
// image_pipeline_top 验证台（M5：view_i 四视图 + 阈值重标定之后的集成回归）
//---------------------------------------------------------------------
// 激励：把灰图当 {g,g,g} 灌进去 —— rgb_to_gray 的 Y = (77+150+29)*g>>8 = g，
//       正好原值还原，所以边缘图可以和 tb_canny_edge 用同一套 .hex 图样。
// 结构：同一个输入同时喂
//         · u_dut —— image_pipeline_top（真实工程配置）
//         · 旁路参考链 pix_window_3x3 + median_filter_3x3 + pix_window_3x3 +
//           canny_edge（阈值 16/8，与顶层 thr_i = 16 → 16/8 的映射一致，
//           映射写错这里就会报）
// 自检项（全在 Verilog 里，不依赖 Python；算法本身的精度归 tb_canny_edge.v）：
//   1) 延迟：src_de 上升沿 → bin_de 上升沿 = IMG_LAT；vsync 同理且**高电平宽度
//      不变**（送到 rgb2dvi 的 VS 不能被压窄，否则显示器锁不住帧）
//   2) mode 0 旁路：bin_rgb 与 rgb_i 延迟 IMG_LAT 逐像素一致，**每个时钟沿都比**
//      （含消隐期：原图路径是纯寄存器延迟，image_mix 又不在 de=0 时清零）
//   3) mode 1 的四种 view 各对一遍参考链：
//        view 0 二值边缘图 / 1 mag8 图（min(mag8<<1,255)）/ 2 六状态假彩图 / 3 Y 图
//        view 2 的六色：白 = 强边，绿 = 仅因切向传播保留，蓝 = 弱边且滞回连通，
//        黄 = 弱边但未连通，暗红 = NMS 抑制，黑 = 无响应
//      link_en_i 也逐帧调度：同一批 view 在开/关两种状态下各跑一遍，DUT 与
//      参考链吃同一个开关值；绿色只在开的时候出现
//   4) mode 2/3：第 < IMG_W/2 列 = 中值后灰度图、其余 = 边缘图，逐行逐点查
//      （2026-10-17 起左半从原图改为灰度图，黄金值用 ref_y_rgb）
//   5) 帧间靠 mode/view 只在帧首切换，输出与输入逐拍对齐（不做帧号对齐）
//
//   注：view 3（Y 图）的黄金值取自参考链的中值输出（不是原图）—— 激励是
//       {g,g,g}，但 IMPULSE_ONLY 中值仍可能改掉孤立极值点。它验证的是 y_dly
//       相对中值输出的延迟深度对不对（差一拍就整帧错），不是灰度公式；灰度
//       公式由 tb_rgb_to_gray.v 单独验。
//
// 运行： ./obj_dir/Vtb_pipeline_top +STIM=xxx.hex     （见 sim/run.sh）
//=====================================================================
module tb_pipeline_top;

    parameter IMG_W = 64;
    parameter IMG_H = 32;

    // 改 RTL 延迟时这里要跟着改（有意写死，避免"改了 RTL 测试台自动跟随"）
    localparam IMG_LAT  = 19;                  // image_pipeline_top.LATENCY
                                               // （内部 = 原图 18 拍 + image_mix 1 拍）
    localparam Y_DLY    = 14;                  // 中值输出到 can_de 的对齐拍数
                                               // （= ORIG_LAT - 1 - 灰度1 - 窗A2 - 中值1）
    localparam THR_HIGH = 8'd16;               // = 顶层 thr_i（M5：由 40 改为 16）
    localparam THR_LOW  = 8'd8;                // = 顶层内部 thr_i >> 1

    localparam H_BLANK  = 6;
    localparam VS_LEAD  = 4;
    localparam VS_TAIL  = 6;
    localparam H_LINE   = IMG_W + H_BLANK;
    localparam ACT_END  = VS_LEAD + IMG_H*H_LINE;
    localparam FRAME    = ACT_END + VS_TAIL + 2;
    localparam NFRAME   = 12;                  // 帧 0/1 旁路，2..11 见下面的调度

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
    integer      expect_green = 0;              // +EXPECT_GREEN=1：本图样必须出绿色
    initial begin
        if (!$value$plusargs("STIM=%s", stim_f)) stim_f = "stim.hex";
        if (!$value$plusargs("EXPECT_GREEN=%d", expect_green)) expect_green = 0;
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
    // mode / view 调度（M5：view_i 改成运行时端口，原来靠 EDGE_BINARY 例化两个
    //   DUT、现在一个 DUT 逐帧切 view 就够）
    //   帧 0/1 = 旁路(0/0)、2 = 边缘(view 0)、3 = 分屏(2)、4 = 画中画(3)、
    //   5 = mag8 图(view 1)、6 = 假彩图(view 2)、7 = Y 图(view 3)、
    //   8 = 边缘(view 0, link 开)、9 = 假彩图(view 2, link 开)、
    //   10 = mag8 图(view 1, link 开)、11 = 再回旁路(link 开)
    //   只在帧首切换，切换点落在帧消隐里，输出侧逐拍对齐不受影响
    //------------------------------------------------------------------
    reg [1:0] mode = 2'd0;
    reg [1:0] view = 2'd0;
    reg       link_en = 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode    <= 2'd0;
            view    <= 2'd0;
            link_en <= 1'b0;
        end else if (fcyc == 0) begin
            case (frame)
                2:       begin mode <= 2'd1; view <= 2'd0; link_en <= 1'b0; end // 二值边缘图
                3:       begin mode <= 2'd2; view <= 2'd0; link_en <= 1'b0; end // 分屏
                4:       begin mode <= 2'd3; view <= 2'd0; link_en <= 1'b0; end // 画中画（同分屏）
                5:       begin mode <= 2'd1; view <= 2'd1; link_en <= 1'b0; end // mag8 图
                6:       begin mode <= 2'd1; view <= 2'd2; link_en <= 1'b0; end // 六状态假彩图（关）
                7:       begin mode <= 2'd1; view <= 2'd3; link_en <= 1'b0; end // Y 图
                8:       begin mode <= 2'd1; view <= 2'd0; link_en <= 1'b1; end // 二值边缘图（开）
                9:       begin mode <= 2'd1; view <= 2'd2; link_en <= 1'b1; end // 假彩图（开，绿）
                10:      begin mode <= 2'd1; view <= 2'd1; link_en <= 1'b1; end // mag8 图（开）
                11:      begin mode <= 2'd0; view <= 2'd0; link_en <= 1'b1; end // 旁路（开关应无影响）
                default: begin mode <= 2'd0; view <= 2'd0; link_en <= 1'b0; end // 旁路
            endcase
        end
    end

    //------------------------------------------------------------------
    // DUT（真实工程配置）。输出信号沿用 M4 的 bin_* 名字，现在它就是唯一的 DUT。
    //------------------------------------------------------------------
    wire        bin_vs, bin_de;
    wire [23:0] bin_rgb;

    image_pipeline_top #(
        .IMG_W (IMG_W), .IMG_H (IMG_H),
        .LATENCY (IMG_LAT)
    ) u_dut (
        .clk_i (clk), .rst_ni (rst_n),
        .vsync_i (src_vs), .de_i (src_de), .rgb_i (src_rgb),
        .thr_i (THR_HIGH), .mode_i (mode), .view_i (view),
        .smooth_mode_i (2'd0),                  // 0 = 标准高斯，参考链同值
        .link_en_i (link_en),                   // 逐帧调度，参考链同值
        .vsync_o (bin_vs), .de_o (bin_de), .rgb_o (bin_rgb)
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

    // 中值级：参数必须与 u_dut 的默认值一致（IMPULSE_ONLY=1 / DELTA=24）
    wire        rmed_vs, rmed_de;
    wire [7:0]  rmed_y;

    median_filter_3x3 u_ref_median (
        .clk (clk), .rst_n (rst_n),
        .vsync_i (rwin_vs), .de_i (rwin_de),
        .p11 (rp11), .p12 (rp12), .p13 (rp13),
        .p21 (rp21), .p22 (rp22), .p23 (rp23),
        .p31 (rp31), .p32 (rp32), .p33 (rp33),
        .vsync_o (rmed_vs), .de_o (rmed_de),
        .gray_o (rmed_y)
    );

    // 中值输出的是单像素流，Canny 需要再构一级 3x3 窗口
    wire        rwin2_vs, rwin2_de;
    wire [7:0]  rq11, rq12, rq13, rq21, rq22, rq23, rq31, rq32, rq33;

    pix_window_3x3 #(.IMG_W (IMG_W)) u_ref_win2 (
        .clk (clk), .rst_n (rst_n),
        .vsync_i (rmed_vs), .de_i (rmed_de), .gray_i (rmed_y),
        .vsync_o (rwin2_vs), .de_o (rwin2_de),
        .p11 (rq11), .p12 (rq12), .p13 (rq13),
        .p21 (rq21), .p22 (rq22), .p23 (rq23),
        .p31 (rq31), .p32 (rq32), .p33 (rq33)
    );

    wire        ref_vs, ref_de, ref_edge, ref_link;
    wire [7:0]  ref_mag, ref_mag8;
    wire [1:0]  ref_cls;

    canny_edge #(.IMG_W (IMG_W), .LATENCY (12)) u_ref_canny (
        .clk (clk), .rst_n (rst_n),
        .vsync_i (rwin2_vs), .de_i (rwin2_de),
        .p11 (rq11), .p12 (rq12), .p13 (rq13),
        .p21 (rq21), .p22 (rq22), .p23 (rq23),
        .p31 (rq31), .p32 (rq32), .p33 (rq33),
        .thr_high_i (THR_HIGH), .thr_low_i (THR_LOW),
        .smooth_mode_i (2'd0),                  // 与 u_dut 同值
        .link_en_i (link_en),                   // 与 u_dut 同值
        .vsync_o (ref_vs), .de_o (ref_de),
        .mag_o (ref_mag), .edge_o (ref_edge), .link_o (ref_link),
        .mag8_o (ref_mag8), .cls_o (ref_cls)
    );

    // 参考链的输出还要再晚 1 拍才是顶层出口：中间隔着 image_mix 那唯一一级输出
    // 寄存器（它不按 de 清零，所以连消隐期也逐拍对应）
    // 四种 view 的黄金值（组合值与顶层同名逻辑一一对应，改一处必须改两处）
    wire [8:0]  ref_mag8x2  = {1'b0, ref_mag8} << 1;
    wire [7:0]  ref_mag8v   = ref_mag8x2[8] ? 8'hFF : ref_mag8x2[7:0];
    // 六状态：强边白 / 传播保留绿 / 滞回连通蓝 / 未连通黄 / NMS 抑制暗红 / 无响应黑
    // ★ 优先级与 image_pipeline_top 的 cls_rgb 必须逐字一致：绿只看 ref_link，
    //   蓝才看 ref_edge（link 像素本身 edge 也是 1）。
    wire [23:0] ref_cls_rgb = (ref_cls == 2'b11) ? 24'hFFFFFF :
                              (ref_cls == 2'b10) ? (ref_link ? 24'h00FF00 :
                                                    ref_edge ? 24'h0000FF :
                                                               24'hFFFF00) :
                              (ref_cls == 2'b01) ? 24'h800000 : 24'h000000;

    reg [23:0] ref_edge_rgb = 24'h000000;
    reg [23:0] ref_mag8_rgb = 24'h000000;
    reg [23:0] ref_cls_d_rgb = 24'h000000;
    reg [23:0] ref_y_rgb    = 24'h000000;   // 在下面 src_dly 声明之后再驱动

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ref_edge_rgb  <= 24'h000000;
            ref_mag8_rgb  <= 24'h000000;
            ref_cls_d_rgb <= 24'h000000;
        end else begin
            ref_edge_rgb  <= ref_edge ? 24'hFFFFFF : 24'h000000;
            ref_mag8_rgb  <= {ref_mag8v, ref_mag8v, ref_mag8v};
            ref_cls_d_rgb <= ref_cls_rgb;
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

    // view 3 的黄金值：顶层进 image_mix 的是 y_dly[Y_DLY-1]，即中值输出再延
    // Y_DLY = 14 拍（中值在第 4 拍，+14 = ORIG_LAT = 18 拍）；再经 image_mix
    // 那 1 拍寄存器才是顶层出口，所以这里同样延 14 拍后再寄一次 = IMG_LAT。
    reg [7:0] rmed_dly [0:Y_DLY-1];

    integer m;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            for (m = 0; m < Y_DLY; m = m + 1) rmed_dly[m] <= 8'd0;
        else begin
            rmed_dly[0] <= rmed_y;
            for (m = 1; m < Y_DLY; m = m + 1)
                rmed_dly[m] <= rmed_dly[m-1];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ref_y_rgb <= 24'h000000;
        else        ref_y_rgb <= {rmed_dly[Y_DLY-1], rmed_dly[Y_DLY-1],
                                  rmed_dly[Y_DLY-1]};
    end

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
    // 检查 2..5：按 mode/view 比对（mode_d/view_d = 产生这一拍输出的那一组）
    //------------------------------------------------------------------
    reg [1:0] mode_d = 2'd0;
    reg [1:0] view_d = 2'd0;

    integer err_bypass = 0, err_edge = 0, err_split = 0;
    integer err_mag8 = 0, err_cls = 0, err_y = 0;
    integer ref_green = 0;                     // 参考链在 view2 判为"传播保留"的像素数
    integer out_pix = 0;                       // 本行已输出的像素数（1 基计数）
    integer out_col = 0;                       // 本行当前输出像素的 0 基列号
    integer row_cnt = 0, short_rows = 0, long_rows = 0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode_d <= 2'd0; view_d <= 2'd0; out_pix = 0; out_col = 0; row_cnt = 0;
        end else begin
            mode_d <= mode;
            view_d <= view;


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
                    case (view_d)
                        2'd0: if (bin_rgb !== ref_edge_rgb)  err_edge = err_edge + 1;
                        2'd1: if (bin_rgb !== ref_mag8_rgb)  err_mag8 = err_mag8 + 1;
                        2'd2: begin
                            // 用与 bin_rgb 同拍的黄金值计数（ref_cls_d_rgb 已寄存 1 拍）
                            if (ref_cls_d_rgb == 24'h00FF00) ref_green = ref_green + 1;
                            if (bin_rgb !== ref_cls_d_rgb) err_cls = err_cls + 1;
                        end
                        2'd3: if (bin_rgb !== ref_y_rgb)     err_y    = err_y    + 1;
                        default: ;
                    endcase
                end
                2'd2, 2'd3: begin
                    if (bin_de) begin
                        if (out_col < HALF_W) begin
                            // 左半 = 中值后的灰度图（{Y,Y,Y}），不是原图
                            if (bin_rgb !== ref_y_rgb) err_split = err_split + 1;
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
        $display("VIEW0    二值边缘图失配 %0d 点", err_edge);
        $display("VIEW1    mag8 图失配 %0d 点", err_mag8);
        $display("VIEW2    六状态假彩图失配 %0d 点（参考链绿色 %0d 点）", err_cls, ref_green);
        $display("VIEW3    Y 图失配 %0d 点", err_y);
        $display("MODE2/3  分屏失配 %0d 点", err_split);

        if ((de_rise_o - de_rise_i) != IMG_LAT)    begin fails=fails+1; $display("FAIL: de 延迟不符"); end
        if ((vs_o_rise - vs_i_rise)  != IMG_LAT)   begin fails=fails+1; $display("FAIL: vsync 延迟不符"); end
        if (vs_width_err != 0)                     begin fails=fails+1; $display("FAIL: vsync 宽度变了"); end
        if (short_rows != 0 || long_rows != 0)     begin fails=fails+1; $display("FAIL: 每行像素数不对"); end
        if (err_bypass != 0)                       begin fails=fails+1; $display("FAIL: mode 0 旁路不逐像素一致"); end
        if (err_edge != 0)                         begin fails=fails+1; $display("FAIL: view 0 二值边缘图不符"); end
        if (err_mag8 != 0)                         begin fails=fails+1; $display("FAIL: view 1 mag8 图不符"); end
        if (err_cls  != 0)                         begin fails=fails+1; $display("FAIL: view 2 六状态假彩图不符"); end
        // faint_link（+EXPECT_GREEN=1）必须真的出现绿色，否则 link 开关没生效
        if (expect_green != 0 && ref_green == 0)
            begin fails=fails+1; $display("FAIL: view2 link 开的帧没有绿色像素"); end
        if (err_y    != 0)                         begin fails=fails+1; $display("FAIL: view 3 Y 图不符"); end
        if (err_split != 0)                        begin fails=fails+1; $display("FAIL: mode 2/3 分屏不符"); end

        if (fails == 0) $display("TOP PASS");
        else            $display("TOP FAIL (%0d 项)", fails);
        $finish;
    end

endmodule
