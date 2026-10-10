`timescale 1ns / 1ps
//=====================================================================
// pix_window_3x3  —— 接口 I-B
//---------------------------------------------------------------------
// 把灰度像素流变成 3x3 邻域窗口（p11..p33），供 Sobel / 后续滤波共享。
//
// 【存储结构：3 个 bank 按行轮转 + 同步读】
//   初版用两个数组 line_buf_1/line_buf_2 做"异步读 + 级联搬移"：
//       line_buf_2[col] <= line_buf_1[col];
//       line_buf_1[col] <= gray_i;
//   综合结果 20605 FF + 17377 LUT + 0 RAM —— 异步读推断不出 BRAM，两个
//   1280x8 存储器被整体打散成寄存器，占掉全器件约 1/4 的逻辑资源。
//   现改为 3 个 1280x8 的 bank 按行轮转，每个 bank 单时钟、1 读 1 写
//   （标准 SDP 写法）：
//       bank(sel)    = 当前行 n     （写）
//       bank(sel-1)  = 上一行 n-1   （读）
//       bank(sel-2)  = 上上行 n-2   （读）
//   行轮转顺带免掉了"级联搬移"这份工作：不必再把 line_buf_1 的内容搬进
//   line_buf_2，读到的永远是"同一列、更老的行"，三行天然对齐、零错位。
//
// 【为什么不用 src/isp/Line_Shift_RAM_8Bit.v】
//   README 铁律 1 要求行缓存"复制一份到 src/image/ 再改造"。评估后未采用
//   该原语（src/isp/ 下的原件一行未改）：
//     (1) 它实现的是"延迟 1 行 + 1 像素"（内部地址模 DATA_DEPTH+1 循环、
//         INIT_ADDR = DATA_DEPTH），相邻两行之间固定差 1 个像素，必须像
//         VIP_Matrix_Generate_3X3_8Bit 那样用 pixel_cnt 特判去补，容易埋坑；
//     (2) 存储深度写成 2**ADDR_WIDTH，1280 点的行要占 2048 深，浪费约 40%。
//   本模块每个 bank 深度恰为 IMG_W，列天然对齐，无需任何特判。
//
// 【对外延迟】LATENCY = 2 拍，数据与 vsync_o / de_o 严格同拍：
//     第 1 拍：bank 同步读（q0/q1/q2）
//     第 2 拍：窗口寄存器（p11..p33）
//   下游 image_pipeline_top 的 ORIG_LAT（原图旁路延迟）必须把这 2 拍算进去。
//
// 【空间对齐】输出窗口中心 = 与输出同拍进入的那一像素的"上一行、左一列"。
//   即边缘图相对原图整体右下偏移 1 行 1 列（与初版一致）；这两行两列会被
//   FrameBoundCrop 的 SKIP_ROWS/SKIP_COLS = 2 抹黑，实际不可见。
//
// 【边界】左边界：每行输出第 0 列（窗口中心在第 0 列、左邻列不存在）时，
//   整窗复制为**最近的有效列**（第 1 列）的三行像素，不做补零，对齐
//   docs/INTERFACE.md §3 "复制边缘像素、不补零"。
//   （比初版复制第 0 列略有出入：预测式读地址使"行首读侧那一拍"拿到的是
//   第 1 列的数据，要去到第 0 列得再加一级缓存；该列落在裁剪黑边内不可见，
//   故不做。）
//   行方向（每帧最上前两行）不做特殊处理：读到的 bank 残留是上一帧内容，
//   同样落在裁剪区内，不可见。
//   行尾那一拍读地址回绕到 0，于是"末列的右邻"取到第 0 列，输出末列（及
//   其右邻列）数据不对；同样落在裁剪黑边内，故不做特判（冻结成末列会破坏
//   "行尾发 0、行首读第 1 列"这一对齐前提）。
//
// 【前提】每行 DE 拉低后至少 2 拍消隐：行轮转在 DE 下降沿翻页，读地址
//   在消隐期归零，需要一拍才生效。本设计 H_TOTAL - H_DISP = 370 拍，富余。
//
// 【vsync 约定】接口约定 vsync_i 为"帧有效电平"（整个有效画面期间恒高），
//   本模块内部自己做帧首上升沿检测（vsync_start），两种约定都能工作
//   （输入本来就是单拍脉冲时，vsync_d 恰为 0，等价于原行为）。
//=====================================================================
module pix_window_3x3 #(
    parameter integer IMG_W = 1280
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       vsync_i,
    input  wire       de_i,
    input  wire [7:0] gray_i,

    output wire       vsync_o,
    output wire       de_o,
    output reg  [7:0] p11,
    output reg  [7:0] p12,
    output reg  [7:0] p13,
    output reg  [7:0] p21,
    output reg  [7:0] p22,
    output reg  [7:0] p23,
    output reg  [7:0] p31,
    output reg  [7:0] p32,
    output reg  [7:0] p33
);

    localparam integer  AW       = (IMG_W <= 2) ? 1 : $clog2(IMG_W);
    localparam [AW-1:0] COL_ZERO = {AW{1'b0}};
    localparam [AW-1:0] COL_LAST = IMG_W - 1;

    //-----------------------------------------------------------------
    // 1) 帧首检测 + 握手延迟（2 拍，与数据同延迟）
    //    这是本模块最容易踩的坑：若让 de_o 只延迟 1 拍而数据延迟 2 拍，
    //    两者不一致会让整幅图右移 1 列并丢掉最右 1 列。
    //-----------------------------------------------------------------
    reg       vsync_d, de_d;
    reg [1:0] vsync_sr, de_sr;

    wire vsync_start = vsync_i && !vsync_d;     // 帧首单拍
    wire de_r        = de_sr[0];                // 读侧使能（比 de_i 晚 1 拍）
    wire line_start_r = de_sr[0] && !de_sr[1];  // 读侧行首

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_d  <= 1'b0;
            de_d     <= 1'b0;
            vsync_sr <= 2'b00;
            de_sr    <= 2'b00;
        end else begin
            vsync_d  <= vsync_i;
            de_d     <= de_i;
            vsync_sr <= {vsync_sr[0], vsync_i};
            de_sr    <= {de_sr[0],    de_i};
        end
    end

    assign vsync_o = vsync_sr[1];
    assign de_o    = de_sr[1];

    //-----------------------------------------------------------------
    // 2) 列计数：DE 期间 0..IMG_W-1，消隐期归零
    //    写地址用它；读地址在下一节由它推出。
    //-----------------------------------------------------------------
    reg [AW-1:0] col_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            col_cnt <= COL_ZERO;
        else if (!de_i)
            col_cnt <= COL_ZERO;                // 消隐期归零 -> 行首读地址为 0
        else if (col_cnt == COL_LAST)
            col_cnt <= COL_ZERO;
        else
            col_cnt <= col_cnt + 1'b1;
    end

    //-----------------------------------------------------------------
    // 3) 行 bank 轮转：行尾（DE 下降沿）翻页，帧首归零
    //-----------------------------------------------------------------
    reg [1:0] bank_sel;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bank_sel <= 2'd0;
        else if (vsync_start)
            bank_sel <= 2'd0;
        else if (de_d && !de_i)                 // 行尾翻页
            bank_sel <= (bank_sel == 2'd2) ? 2'd0 : bank_sel + 1'b1;
    end

    //   bank_prev1 = (sel+2)%3 -> 上一行 n-1
    //   bank_prev2 = (sel+1)%3 -> 上上行 n-2
    // 这两个式子必须不同：写错会让"上上行"读到"上一行"，Sobel 的 Gy 全程算错。
    wire [1:0] bank_prev1 = (bank_sel == 2'd0) ? 2'd2 : bank_sel - 1'b1;
    wire [1:0] bank_prev2 = (bank_sel == 2'd2) ? 2'd0 : bank_sel + 1'b1;

    //-----------------------------------------------------------------
    // 4) 预测式读地址：提前一拍发 col+1
    //    当拍发 col+1，下一拍读出的正是"下一拍那一列"的历史行数据，于是
    //    rd_prev1/rd_prev2 与当拍 gray_i 落在同一列，三行对齐、零错位。
    //    行尾与消隐期发 0：既避开 col+1 越界，又让每行第一拍读到第 0 列。
    //    读写地址相差 1，且分行在不同 bank，不存在同址读写冲突。
    //-----------------------------------------------------------------
    wire [AW-1:0] rd_addr = (!de_i || col_cnt == COL_LAST) ? COL_ZERO
                                                           : (col_cnt + 1'b1);

    //-----------------------------------------------------------------
    // 5) 3 个行 bank（单时钟、1 读 1 写，标准 SDP 推断写法；不做复位）
    //-----------------------------------------------------------------
    reg [7:0] bank0 [0:IMG_W-1];
    reg [7:0] bank1 [0:IMG_W-1];
    reg [7:0] bank2 [0:IMG_W-1];

    reg [7:0] q0, q1, q2;

    always @(posedge clk) begin
        if (de_i && bank_sel == 2'd0) bank0[col_cnt] <= gray_i;
        q0 <= bank0[rd_addr];
    end

    always @(posedge clk) begin
        if (de_i && bank_sel == 2'd1) bank1[col_cnt] <= gray_i;
        q1 <= bank1[rd_addr];
    end

    always @(posedge clk) begin
        if (de_i && bank_sel == 2'd2) bank2[col_cnt] <= gray_i;
        q2 <= bank2[rd_addr];
    end

    wire [7:0] rd_prev1 = (bank_prev1 == 2'd0) ? q0 :
                          (bank_prev1 == 2'd1) ? q1 : q2;   // 行 n-1，当前列
    wire [7:0] rd_prev2 = (bank_prev2 == 2'd0) ? q0 :
                          (bank_prev2 == 2'd1) ? q1 : q2;   // 行 n-2，当前列

    //-----------------------------------------------------------------
    // 6) 窗口寄存器（读侧）
    //    rowN_d1 / rowN_d2 是各行"当前列"的 1/2 拍历史，正好是左邻、左二邻。
    //    窗口中心 = 上一行、左一列：p2x 取行 n-1，p1x 取行 n-2，p3x 取行 n。
    //-----------------------------------------------------------------
    reg [7:0] row1_d1, row1_d2;     // 行 n-2 的 1/2 拍历史
    reg [7:0] row2_d1, row2_d2;     // 行 n-1 的 1/2 拍历史
    reg [7:0] row3_d1, row3_d2;     // 行 n   的 1/2 拍历史

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row1_d1 <= 8'd0; row1_d2 <= 8'd0;
            row2_d1 <= 8'd0; row2_d2 <= 8'd0;
            row3_d1 <= 8'd0; row3_d2 <= 8'd0;

            p11 <= 8'd0; p12 <= 8'd0; p13 <= 8'd0;
            p21 <= 8'd0; p22 <= 8'd0; p23 <= 8'd0;
            p31 <= 8'd0; p32 <= 8'd0; p33 <= 8'd0;
        end else if (vsync_start) begin
            // 帧首清历史，避免首帧前几拍读到未定义的残值
            row1_d1 <= 8'd0; row1_d2 <= 8'd0;
            row2_d1 <= 8'd0; row2_d2 <= 8'd0;
            row3_d1 <= 8'd0; row3_d2 <= 8'd0;
        end else if (de_r) begin
            if (line_start_r) begin
                // 行首：左邻缺失，整窗复制第 0 列（docs/INTERFACE.md §3）
                p11 <= rd_prev2; p12 <= rd_prev2; p13 <= rd_prev2;
                p21 <= rd_prev1; p22 <= rd_prev1; p23 <= rd_prev1;
                p31 <= gray_i;   p32 <= gray_i;   p33 <= gray_i;

                row1_d1 <= rd_prev2; row1_d2 <= rd_prev2;
                row2_d1 <= rd_prev1; row2_d2 <= rd_prev1;
                row3_d1 <= gray_i;   row3_d2 <= gray_i;
            end else begin
                p11 <= row1_d2; p12 <= row1_d1; p13 <= rd_prev2;
                p21 <= row2_d2; p22 <= row2_d1; p23 <= rd_prev1;
                p31 <= row3_d2; p32 <= row3_d1; p33 <= gray_i;

                row1_d2 <= row1_d1; row1_d1 <= rd_prev2;
                row2_d2 <= row2_d1; row2_d1 <= rd_prev1;
                row3_d2 <= row3_d1; row3_d1 <= gray_i;
            end
        end
    end

endmodule
