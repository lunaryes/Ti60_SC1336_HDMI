# RGB 转灰度代码文档

本文档面向当前 `Ti60F225 + SC1336 + HDMI 1280x720` 工程，说明如何在已经完成 Bayer 转 RGB 和手动白平衡之后，把 RGB888 视频流转换为灰度 RGB888，再送入后级裁边和 HDMI 输出。

参考摄像头工程：

```text
e:\qianrushi\VF-Ti60F225_HDK_V2.1_20251009\VF-Ti60F225_HDK_V2.1_20251009\VF-Ti60F225_HDK_V2.1_20251009\07-FPGA_HDL_Image-2023.2\03-1_Ti60_SC130GS_MIPIx4_HDMI_1280720
```

当前工程关键链路：

```text
MIPI CSI RAW8
  -> Sensor_Image_XYCrop
  -> DDR
  -> lcd_driver 读出 RAW8
  -> VIP_RAW8_RGB888
  -> manual awb
  -> RGB 转灰度
  -> FrameBoundCrop
  -> rgb2dvi
  -> HDMI
```

推荐插入点在 `example_top.v` 的手动白平衡之后、`FrameBoundCrop` 之前。这个位置的信号已经是 RGB888，且仍在 `clk_pixel` 单时钟域内，改动范围最小。

## 1. 灰度公式

使用常见定点亮度公式：

```text
Y = 0.299R + 0.587G + 0.114B
Y = (77*R + 150*G + 29*B) >> 8
```

系数 `77 + 150 + 29 = 256`，因此右移 8 位后保持 8 bit 亮度。输出为了兼容 HDMI RGB888，采用：

```text
gray_rgb = {Y, Y, Y}
```

## 2. 可综合 Verilog 模块

建议新增文件：

```text
Ti60F225_SC1336_HDMI_Display/src/image/rgb_to_gray.v
```

代码如下：

```verilog
`timescale 1ns / 1ps

module rgb_to_gray (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        vs_i,
    input  wire        hs_i,
    input  wire        de_i,
    input  wire [23:0] rgb_i,

    output reg         vs_o,
    output reg         hs_o,
    output reg         de_o,
    output reg  [23:0] rgb_o
);

    wire [7:0] r_i = rgb_i[23:16];
    wire [7:0] g_i = rgb_i[15:8];
    wire [7:0] b_i = rgb_i[7:0];

    wire [15:0] y_sum =
        (r_i * 8'd77) +
        (g_i * 8'd150) +
        (b_i * 8'd29);

    wire [7:0] y = y_sum[15:8];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vs_o  <= 1'b0;
            hs_o  <= 1'b0;
            de_o  <= 1'b0;
            rgb_o <= 24'h000000;
        end else begin
            vs_o  <= vs_i;
            hs_o  <= hs_i;
            de_o  <= de_i;
            rgb_o <= de_i ? {y, y, y} : 24'h000000;
        end
    end

endmodule
```

## 3. `example_top.v` 接线方式

原始白平衡输出信号：

```verilog
wire [7:0] w_rgb_r;
wire [7:0] w_rgb_g;
wire [7:0] w_rgb_b;

assign w_rgb_r = w_rgb_r_mult[17:16] ? 8'hFF : w_rgb_r_mult[15: 8];
assign w_rgb_g = w_rgb_pre_g;
assign w_rgb_b = w_rgb_b_mult[17:16] ? 8'hFF : w_rgb_b_mult[15: 8];
```

在 `FrameBoundCrop` 实例化之前新增灰度模块信号：

```verilog
wire        gray_vs;
wire        gray_hs;
wire        gray_de;
wire [23:0] gray_rgb;

rgb_to_gray u_rgb_to_gray (
    .clk   (clk_pixel),
    .rst_n (rstn_pixel),

    .vs_i  (w_rgb_vsync),
    .hs_i  (w_rgb_hsync),
    .de_i  (w_rgb_href),
    .rgb_i ({w_rgb_r, w_rgb_g, w_rgb_b}),

    .vs_o  (gray_vs),
    .hs_o  (gray_hs),
    .de_o  (gray_de),
    .rgb_o (gray_rgb)
);
```

然后把 `FrameBoundCrop` 输入从原 RGB 改成灰度 RGB：

```verilog
FrameBoundCrop #(
    .SKIP_ROWS  (2),
    .SKIP_COLS  (2),
    .TOTAL_ROWS (720),
    .TOTAL_COLS (1280)
) inst2_FrameCrop (
    .clk_i  (clk_pixel),
    .rst_i  (~rstn_pixel),

    .vs_i   (gray_vs),
    .hs_i   (gray_hs),
    .de_i   (gray_de),
    .data_i (gray_rgb),

    .vs_o   (boundcrop_vs),
    .hs_o   (boundcrop_hs),
    .de_o   (boundcrop_de),
    .data_o (boundcrop_data)
);
```

## 4. 工程文件注意事项

如果新增 `src/image/rgb_to_gray.v`，需要确认 `Ti60_Demo.xml` 的工程源文件列表包含该文件。否则综合时会报找不到 `rgb_to_gray` 模块。

推荐目录：

```text
Ti60F225_SC1336_HDMI_Display/src/image/
```

如果当前工程还没有 `src/image/`，可以新建目录；也可以临时放到 `src/isp/`，但从功能归类上更建议放在 `src/image/`。

## 5. 时序与资源

- 时钟域：只使用 `clk_pixel`，不引入新跨时钟域。
- 输入格式：`rgb_i = {R[7:0], G[7:0], B[7:0]}`。
- 输出格式：`rgb_o = {Y[7:0], Y[7:0], Y[7:0]}`。
- 延迟：输出相对输入固定延迟 1 个 `clk_pixel`。
- 同步信号：`vs/hs/de` 同步寄存 1 拍，与 `rgb_o` 对齐。
- 资源：3 个常数乘法，可由综合器映射为 DSP 或移位加法逻辑。

## 6. 快速检查点

1. HDMI 画面应变为黑白图像，不应出现彩色残留。
2. 图像边界仍由 `FrameBoundCrop` 处理，四周 2 像素黑边属于现有设计。
3. 如果画面整体偏暗，可以把输出改为四舍五入：

```verilog
wire [7:0] y = (y_sum + 16'd128) >> 8;
```

4. 如果综合器对乘法资源比较敏感，可改成移位加法近似：

```verilog
Y = (R>>2) + (R>>5) + (G>>1) + (G>>4) + (B>>4) + (B>>5);
```

但推荐优先使用 `77/150/29` 版本，灰度效果更稳定。
