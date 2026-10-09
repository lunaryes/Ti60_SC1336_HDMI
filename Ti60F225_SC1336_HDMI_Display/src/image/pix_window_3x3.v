`timescale 1ns / 1ps
//=====================================================================
// pix_window_3x3  —— 接口 I-B
//---------------------------------------------------------------------
// 把 RGB888 像素流变成 3x3 灰度邻域窗口，输出 p11..p33 共 9 个 8bit 像素。
//
// 【前置条件】
//   rgb_i 已经是 {Y,Y,Y}（灰度化由上游 rgb_to_gray 完成，见 docs/RGB_TO_GRAY.md）。
//   本模块只取 R 通道作为亮度 Y，不重复做灰度化，省下 3 个常数乘法器。
//
// 【为什么不用 src/isp/Line_Shift_RAM_8Bit.v】
//   README 铁律 1 要求行缓存"复制一份到 src/image/ 再改造"。复制件保留在
//   src/image/image_line_shift_ram_8bit.v 作参考，但本模块未采用它：
//     (1) 该原语实际实现的是"延迟 1 行 + 1 像素"（内部地址模 DATA_DEPTH+1 循环，
//         INIT_ADDR = DATA_DEPTH），相邻两行之间有固定 1 像素错位，必须像
//         VIP_Matrix_Generate_3X3_8Bit 那样用 pixel_cnt 特判去补，容易埋坑；
//     (2) 存储深度写成 2**ADDR_WIDTH，1280 点的行要占 2048 深，浪费约 40%。
//   本模块改用「3 个 bank 按行轮转 + 预测式读地址」结构：读地址提前一拍发
//   col+1，使读出的历史像素与当拍输入落在同一列，三行天然对齐、零错位，
//   每个 bank 深度恰为 IMG_W。
//   src/isp/ 下的原件一行都没有改动。
//
// 【时序】
//   vsync_o / de_o 与数据同延迟 = 2 拍。数据路径内部延迟：垂直 1 行 + 水平 2 拍。
//   DE 有效期内输出的第 k 列恰好对应图像第 k 列，输出的第 m 行对应图像第 m-1 行
//   （整幅只有 1 行的垂直偏移）。
//   下游 FrameBoundCrop 配置 SKIP_ROWS=2/SKIP_COLS=2，最外 2 行 2 列会被抹黑，
//   故 1 行垂直偏移与边界过渡区都不可见。
//   由于窗口需要"下一行"数据，最后一行的边缘幅值不会被计算；该行的窗口右邻
//   同样取不到，落在 FrameBoundCrop 的空白区内。
//
// 【边界】本模块**不做边界特殊处理**（docs/INTERFACE.md §3 已据此修订，
//   原"复制边缘像素、不补零"条款**作废**）。最外圈像素的窗口会取到越界值，
//   由下游 FrameBoundCrop 的 SKIP_ROWS=2 / SKIP_COLS=2 裁剪切边统一兜底，
//   裁剪后可见区域内的窗口 100% 正确。
//   实测越界行为（sim/tb_window_boundary.v，图案 pix=row*16+col，详见 ARCH.md §11.4）：
//     输出首行：窗口"上一行 / 上上行"读到**上一帧末两行的残留**
//     输出首列：窗口"左邻列"取到**第 1 列**（而非复制第 0 列）
//     输出末列：整窗被替换为**第 0 列**的值（行尾读地址回绕）
//   下面 center_bad / left_bad 两个分支**保留原样**：它们用"当拍 col"做判据，
//   而输出比输入滞后 2 拍，所以实际只作用在输出行的**最后两列**——同样落在裁剪区内。
//   保留是为了不动已通过综合与仿真的 RTL。**不要把它们当成"边界已实现"。**
//   将来若要真做：判据须改用**输出侧列号**并新增行方向判据，
//   不能简单给 col 加偏移（col 在行尾会归零，偏移跨行时不成立）。
//
// 【握手】vsync_o / de_o 延迟 2 拍，与数据严格同拍
//
// 【LATENCY】垂直 1 行、水平 2 拍（供 image_pipeline_top 对齐旁路）
//
// 【资源】3 x IMG_W x 8bit = 3 x 1280 x 8 = 30 kbit，应推断为 BRAM。
//         只例化一次，Sobel 与中值滤波共享同一组 p11..p33。
//=====================================================================
module pix_window_3x3 #(
    parameter IMG_W = 1280,
    parameter IMG_H = 720
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        vsync_i,
    input  wire        de_i,
    input  wire [23:0] rgb_i,

    output wire        vsync_o,
    output wire        de_o,
    output wire [7:0]  p11, p12, p13,   // 上一行 左 中 右
    output wire [7:0]  p21, p22, p23,   // 当前行 左 中 右（窗口中心）
    output wire [7:0]  p31, p32, p33    // 下一行 左 中 右
);

    localparam AW        = 11;                    // IMG_W <= 2**AW
    localparam [AW-1:0] COL_ZERO = {AW{1'b0}};
    localparam [AW-1:0] COL_ONE  = {{(AW-1){1'b0}}, 1'b1};
    localparam [AW-1:0] COL_LAST = IMG_W - 1;

    //-----------------------------------------------------------------
    // 握手信号：延迟 = 数据延迟 = 2 拍。
    // 这是本模块最容易踩的坑：若让 de_o = de_i（0 延迟）而数据延迟 2 拍，
    // 两者不一致会让整幅图右移 2 列、并丢掉最右 2 列。
    // 让握手信号与数据同延迟后，输出 DE 窗口内第 k 列恰好就是图像第 k 列。
    //-----------------------------------------------------------------
    reg [1:0] vsync_sr, de_sr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_sr <= 2'b00;
            de_sr    <= 2'b00;
        end else begin
            vsync_sr <= {vsync_sr[0], vsync_i};
            de_sr    <= {de_sr[0],    de_i};
        end
    end

    assign vsync_o = vsync_sr[1];
    assign de_o    = de_sr[1];

    //-----------------------------------------------------------------
    // 亮度输入：rgb_i = {Y,Y,Y}，取 R 通道
    //-----------------------------------------------------------------
    wire [7:0] y_in = rgb_i[23:16];

    //-----------------------------------------------------------------
    // 列计数：DE 有效期内 0 .. IMG_W-1，DE 无效时归零
    //-----------------------------------------------------------------
    reg [AW-1:0] col;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            col <= COL_ZERO;
        else if (!de_i)
            col <= COL_ZERO;
        else if (col == COL_LAST)
            col <= COL_ZERO;
        else
            col <= col + 1'b1;
    end

    //-----------------------------------------------------------------
    // 行 bank 轮转：写入 bank = 行号 % 3
    //   bank(row_bank)       = 当前行   n
    //   bank((row_bank+2)%3) = 上一行   n-1
    //   bank((row_bank+1)%3) = 上上行   n-2
    // 在 DE 下降沿（行尾）翻页，vsync 上升沿复位，保证每帧起点一致。
    //-----------------------------------------------------------------
    reg de_d, vsync_d;
    reg [1:0] row_bank;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_d     <= 1'b0;
            vsync_d  <= 1'b0;
            row_bank <= 2'd0;
        end else begin
            de_d    <= de_i;
            vsync_d <= vsync_i;

            if (vsync_i & ~vsync_d)
                row_bank <= 2'd0;
            else if (de_d & ~de_i)
                row_bank <= (row_bank == 2'd2) ? 2'd0 : row_bank + 1'b1;
        end
    end

    //   bank_prev1 = (row_bank + 2) % 3  -> 上一行 n-1
    //   bank_prev2 = (row_bank + 1) % 3  -> 上上行 n-2
    // 注意：这两个式子必须**不同**。曾经把 bank_prev2 误写成 (row_bank-1)%3，
    // 结果与 bank_prev1 恒等，"上上行"读到的是"上一行"，Sobel 的 Gy 全程算错。
    wire [1:0] bank_prev1 = (row_bank == 2'd0) ? 2'd2 : row_bank - 1'b1;
    wire [1:0] bank_prev2 = (row_bank == 2'd2) ? 2'd0 : row_bank + 1'b1;

    //-----------------------------------------------------------------
    // 预测式读地址：提前一拍发 col+1，使读出的数据与当拍输入同列。
    // 行尾那一拍发 0，使下一行第一拍读到的正是该行第 0 列。
    // 读 bank 与写 bank 不同，且读写地址相差 1，不存在同址冲突。
    //
    // 注意：行尾**回绕到 0 是有意的**，它承担"下一行第 0 列对齐"的职责。
    // 副作用是输出最后一列的"右邻"会取到第 0 列（而非复制末列）。
    // 该副作用属"不做边界处理"的一部分，由 FrameBoundCrop 兜底，故**决定不修**——
    // 冻结成 COL_LAST 会破坏整幅列对齐。详见文件头【边界】与 docs/ARCH.md §11.4。
    //-----------------------------------------------------------------
    wire [AW-1:0] rd_addr = (col == COL_LAST) ? COL_ZERO : (col + 1'b1);

    //-----------------------------------------------------------------
    // 3 个行缓存 bank（单时钟、1 读 1 写，标准 SDP 推断写法）
    //-----------------------------------------------------------------
    reg [7:0] bank0 [0:IMG_W-1];
    reg [7:0] bank1 [0:IMG_W-1];
    reg [7:0] bank2 [0:IMG_W-1];

    reg [7:0] q0, q1, q2;

    always @(posedge clk) begin
        if (de_i && row_bank == 2'd0) bank0[col] <= y_in;
        q0 <= bank0[rd_addr];
    end

    always @(posedge clk) begin
        if (de_i && row_bank == 2'd1) bank1[col] <= y_in;
        q1 <= bank1[rd_addr];
    end

    always @(posedge clk) begin
        if (de_i && row_bank == 2'd2) bank2[col] <= y_in;
        q2 <= bank2[rd_addr];
    end

    wire [7:0] rd_prev1 = (bank_prev1 == 2'd0) ? q0 :
                          (bank_prev1 == 2'd1) ? q1 : q2;   // 行 n-1
    wire [7:0] rd_prev2 = (bank_prev2 == 2'd0) ? q0 :
                          (bank_prev2 == 2'd1) ? q1 : q2;   // 行 n-2

    //-----------------------------------------------------------------
    // 2 拍列历史
    //   a = 行 n   （窗口的"下一行"）
    //   b = 行 n-1 （窗口的"当前行"，即中心）
    //   c = 行 n-2 （窗口的"上一行"）
    //   当拍值 = 右邻列；1 拍历史 = 中心列；2 拍历史 = 左邻列
    //-----------------------------------------------------------------
    reg [7:0] a1, a2, b1, b2, c1, c2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a1 <= 8'd0; a2 <= 8'd0;
            b1 <= 8'd0; b2 <= 8'd0;
            c1 <= 8'd0; c2 <= 8'd0;
        end else if (de_i) begin
            a1 <= y_in;      a2 <= a1;
            b1 <= rd_prev1;  b2 <= b1;
            c1 <= rd_prev2;  c2 <= c1;
        end
    end

    //-----------------------------------------------------------------
    // 边界标志 ——【已知与设计意图不符，保留原样，不构成边界实现】
    //   判据用的是"当拍输入列号 col"，但输出比输入**滞后 2 拍**：
    //     wcol = 0 时模块内 col = 2；wcol = 15 时 col = 0（行尾归零）
    //   因此这两个标志实际只在**输出行的最后两列**（wcol = 14/15）生效，
    //   而真正需要保护的首两列（wcol = 0/1）完全不生效 —— 方向正好搞反。
    //   保留理由：作用区落在 FrameBoundCrop 的 2 行 2 列裁剪内，不影响可见输出；
    //   且不动已通过综合与仿真的 RTL。详见文件头【边界】与 docs/ARCH.md §11.4。
    //   将来若要真做：判据须改用**输出侧列号**，不能简单给 col 加偏移
    //   （col 在行尾会归零，偏移跨行时不成立）。
    //-----------------------------------------------------------------
    wire center_bad = (col == COL_ZERO);
    wire left_bad   = (col <= COL_ONE);

    //-----------------------------------------------------------------
    // 窗口寄存器
    //-----------------------------------------------------------------
    reg [7:0] w11, w12, w13;
    reg [7:0] w21, w22, w23;
    reg [7:0] w31, w32, w33;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w11 <= 8'd0; w12 <= 8'd0; w13 <= 8'd0;
            w21 <= 8'd0; w22 <= 8'd0; w23 <= 8'd0;
            w31 <= 8'd0; w32 <= 8'd0; w33 <= 8'd0;
        end else begin
            // 缺省：左 = 2 拍历史，中 = 1 拍历史，右 = 当拍
            w11 <= c2; w12 <= c1; w13 <= rd_prev2;   // 上一行 n-2
            w21 <= b2; w22 <= b1; w23 <= rd_prev1;   // 当前行 n-1
            w31 <= a2; w32 <= a1; w33 <= y_in;       // 下一行 n

            if (left_bad) begin                      // 左邻缺失 -> 复制中心
                w11 <= c1;
                w21 <= b1;
                w31 <= a1;
            end

            if (center_bad) begin                    // 中心列不存在 -> 整窗复制右邻
                w11 <= rd_prev2; w12 <= rd_prev2; w13 <= rd_prev2;
                w21 <= rd_prev1; w22 <= rd_prev1; w23 <= rd_prev1;
                w31 <= y_in;     w32 <= y_in;     w33 <= y_in;
            end
        end
    end

    assign p11 = w11; assign p12 = w12; assign p13 = w13;
    assign p21 = w21; assign p22 = w22; assign p23 = w23;
    assign p31 = w31; assign p32 = w32; assign p33 = w33;

endmodule
