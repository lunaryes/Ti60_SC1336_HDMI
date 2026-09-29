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

## 2. image/rgb_to_gray.v

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

然后把 `FrameBoundCrop` **输入从原 RGB 改成灰度 RGB**：

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


## 5. 时序与资源

- 时钟域：只使用 `clk_pixel`，不引入新跨时钟域。
- 输入格式：`rgb_i = {R[7:0], G[7:0], B[7:0]}`。
- 输出格式：`rgb_o = {Y[7:0], Y[7:0], Y[7:0]}`。
- 延迟：输出相对输入固定延迟 1 个 `clk_pixel`。
- 同步信号：`vs/hs/de` 同步寄存 1 拍，与 `rgb_o` 对齐。
- 资源：3 个常数乘法，可由综合器映射为 DSP 或移位加法逻辑。

 如果画面整体偏暗，可以把输出改为四舍五入：

```verilog
wire [7:0] y = (y_sum + 16'd128) >> 8;
```

4. 如果综合器对乘法资源比较敏感，可改成移位加法近似：

```verilog
Y = (R>>2) + (R>>5) + (G>>1) + (G>>4) + (B>>4) + (B>>5);
```
