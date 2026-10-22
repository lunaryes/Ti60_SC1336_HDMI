`timescale 1ns/1ps
//=====================================================================================
// image_sbs_resizer —— 左右分屏：左半 = 灰度原图，右半 = 灰度原图 + 边缘叠加
//-------------------------------------------------------------------------------------
// 【注意：文件名/模块名沿用 image_sbs_resizer，但本版已经没有任何缩放】
//   之所以不改名：这个文件已经登记在 Ti60_Demo.xml 的 design_files 里，
//   改名要同步改工程文件，容易在 IDE 打开状态下来回踩坑。
//   "sbs" = side by side（左右分屏），仍然准确；"resizer" 已名不副实。
//
// 【功能】
//   显示光栅 1280x720 @74.25MHz，按列一分为二，**像素 1:1 直接映射、不缩放**：
//       x =    0 ..  639 : 左半 ← 原图的左半（只去色，不做任何处理）
//       x =  640 .. 1279 : 右半 ← 原图的右半（保留灰度 + 白色边缘叠加）
//   也就是说：屏幕像素 (x,y) 对应原图像素 (x,y)，左右两半拼起来就是整幅图，
//   区别只在于右半上面"画"了 Sobel 边缘。
//
// 【为什么现在不需要行缓存了】
//   上一版是"整幅图压进半个屏幕"（水平 2:1），一行 1280 个像素要写进 640 深的
//   行缓存，所以需要两块 640x24 的乒乓缓存 + 跨行读出。
//   本版是 1:1，第 y 行输出用的就是第 y 行输入，不需要回看任何历史行，
//   所以整个模块退化成一个「打 8 拍对齐 + 三选一 MUX」，零 RAM。
//   代价是每半屏只看到"半幅图"，看不到整幅图的整体效果。
//
// 【为什么要打 8 拍】
//   边缘来自 image_pipeline_top，那条通路比原图晚 LATENCY = 8 拍
//   （rgb_to_gray 1 + pix_window_3x3 2 + sobel_edge 4 + image_mix 1）。
//   左半的原图必须延迟同样的拍数，否则左右两半会错开 8 个像素，
//   在屏幕中缝处能明显看到上下断开。
//   实测对齐关系：edge_flag_i 与 rgb_i 延迟 8 拍后的像素属于同一个源像素。
//
// 【灰度公式与流水线保持一致】
//   Y = (R*77 + G*150 + B*29) >> 8，与 src/image/rgb_to_gray.v 完全相同。
//   这点很关键：左右两半如果用了不同的灰度系数，中缝两侧亮度会有台阶。
//   乘法器放在第 1 级寄存器的前面（前一级是 ADC/AWB 的输出寄存器），
//   后面还有 7 级寄存器垫着，组合路径不会压在 74.25MHz 的关键路径上。
//
// 【延迟】
//   vs_o / hs_o / de_o / rgb_o 全部比输入晚 LAT 拍，四者严格同拍。
//
// 【参数】
//   H_ACTIVE : 水平有效像素，1280（与 lcd_para.v 的 H_DISP 一致），
//              分界列 = H_ACTIVE/2 = 640，必须是偶数
//   LAT      : 对齐延迟拍数，必须等于 image_pipeline_top 的 LATENCY（当前 8）
//   EDGE_RGB : 边缘叠加颜色，默认纯白 24'hFF_FF_FF
//=====================================================================================
module image_sbs_resizer #(
    parameter H_ACTIVE = 1280,
    parameter LAT      = 8,
    parameter EDGE_RGB = 24'hFF_FF_FF
)(
    input  wire        clk_i,
    input  wire        rst_ni,

    // ---- 主通路：显示光栅 + AWB 之后的原图（实时，0 延迟）----
    input  wire        vs_i,
    input  wire        hs_i,
    input  wire        de_i,
    input  wire [23:0] rgb_i,

    // ---- 处理通路：二值边缘标志（比原图晚 LAT 拍，与 pipe_rgb 同拍）----
    //   1 = 该像素是 Sobel 边缘。注意这一路只借它的"时间对齐"，不用它的数据。
    input  wire        edge_flag_i,

    // ---- 输出：1280x720 光栅 ----
    output wire        vs_o,
    output wire        hs_o,
    output wire        de_o,
    output wire [23:0] rgb_o
);

    localparam [11:0] HALF = H_ACTIVE >> 1;         // 分界列 640

    //---------------------------------------------------------------------------------
    // 1) 灰度：与 src/image/rgb_to_gray.v 完全相同的系数
    //---------------------------------------------------------------------------------
    wire [7:0]  r_i = rgb_i[23:16];
    wire [7:0]  g_i = rgb_i[15: 8];
    wire [7:0]  b_i = rgb_i[ 7: 0];

    wire [15:0] y_sum = (r_i * 8'd77) + (g_i * 8'd150) + (b_i * 8'd29);
    wire [7:0]  y_in  = y_sum[15:8];

    //---------------------------------------------------------------------------------
    // 2) 对齐延迟链：y / vs / hs / de 各 LAT 级
    //    sr[0] 是"灰度寄存器"那一级（第 1 拍），sr[LAT-1] 是第 LAT 拍。
    //    延迟链不复位到 0 也安全（全部靠 de_o 门控），但这里复位干净些，
    //    免得仿真出现一堆 X。
    //---------------------------------------------------------------------------------
    reg [7:0] y_sr  [0:LAT-1];
    reg       vs_sr [0:LAT-1];
    reg       hs_sr [0:LAT-1];
    reg       de_sr [0:LAT-1];

    integer k;

    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            for (k = 0; k < LAT; k = k + 1) begin
                y_sr [k] <= 8'd0;
                vs_sr[k] <= 1'b0;
                hs_sr[k] <= 1'b0;
                de_sr[k] <= 1'b0;
            end
        end else begin
            y_sr [0] <= y_in;
            vs_sr[0] <= vs_i;
            hs_sr[0] <= hs_i;
            de_sr[0] <= de_i;

            for (k = 1; k < LAT; k = k + 1) begin
                y_sr [k] <= y_sr [k-1];
                vs_sr[k] <= vs_sr[k-1];
                hs_sr[k] <= hs_sr[k-1];
                de_sr[k] <= de_sr[k-1];
            end
        end
    end

    wire        de_o_i   = de_sr[LAT-1];
    wire [7:0]  y_o      = y_sr [LAT-1];
    wire [23:0] gray_rgb = {y_o, y_o, y_o};

    assign vs_o = vs_sr[LAT-1];
    assign hs_o = hs_sr[LAT-1];
    assign de_o = de_o_i;

    //---------------------------------------------------------------------------------
    // 3) 输出列计数：de_o 期间 0..H_ACTIVE-1，消隐期归零
    //    计数与数据同拍（第 1 个有效像素对应的计数值为 0）。
    //---------------------------------------------------------------------------------
    reg [11:0] x_out;

    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni)
            x_out <= 12'd0;
        else if (!de_o_i)
            x_out <= 12'd0;
        else
            x_out <= x_out + 12'd1;
    end

    //---------------------------------------------------------------------------------
    // 4) 合成
    //    左半：纯灰度
    //    右半：灰度，边缘像素处替换成 EDGE_RGB
    //    组合逻辑只有一级 MUX，下游 FrameBoundCrop 的输入寄存器就是它的落脚点。
    //---------------------------------------------------------------------------------
    wire left_half = (x_out < HALF);

    assign rgb_o = left_half        ? gray_rgb                    :
                   edge_flag_i      ? EDGE_RGB                    :
                                      gray_rgb                     ;

endmodule
