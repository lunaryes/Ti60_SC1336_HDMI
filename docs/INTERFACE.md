# 接口文档

> 本文件是接口的唯一来源。

## 1. 模块一览

| 模块 | 文件 | 作用 | 接口 |
| --- | --- | --- | --- |
| `pix_window_3x3` | `src/image/pix_window_3x3.v` | 灰度化 + 双行缓存 + 3×3 窗口，输出 9 个灰度像素 | I-B |
| `sobel_edge` | `src/image/sobel_edge.v` | Sobel 梯度、饱和、阈值比较 | I-B |
| `image_mix` | `src/display/image_mix.v` | 原图与边缘图的分屏 / 画中画 / 叠加合成 | I-C |
| `image_pipeline_top` | `src/image/image_pipeline_top.v` | 流水线顶层，串联上述模块并做旁路 | I-A |

## 2. 全局约定

**端口顺序**（所有时序模块统一）

```
clk, rst_n, vsync_i, de_i, <data_in>, <ctrl_in>, vsync_o, de_o, <data_out>
```

- `vsync` = 帧有效，`de` = 像素有效；只处理 `de=1` 的像素。
- `data_o` 与 `vsync_o/de_o` 严格同拍；延迟固定，写在模块头注释里。
- 位宽显式写出，如 `[7:0]`、`[23:0]`。

**像素格式**

| 阶段 | 格式 |
| --- | --- |
| 算法入口（AWB 之后） | RGB888 = `{R[7:0], G[7:0], B[7:0]}` |
| 灰度 | Y8 |
| Sobel 幅值 | mag8，饱和到 255 |
| 边缘标志 | 1 bit，`edge = mag > thr` |
| 算法出口 | RGB888 |

**时钟域**：算法全部在 `clk_pixel`（74.4 MHz）单域。`clk_sys` / `cmos_pclk` / `clk_pixel_2x|10x` 不放算法。

## 3. I-B `pix_window_3x3`

把 RGB888 像素流变成 3×3 灰度邻域，灰度化在本模块内完成。

```verilog
module pix_window_3x3 #(
    parameter IMG_W   = 1280,
    parameter LATENCY = 0        // 实现后填写并固定
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        vsync_i,
    input  wire        de_i,
    input  wire [23:0] rgb_i,
    output wire        vsync_o,
    output wire        de_o,
    output wire [7:0]  p11, p12, p13,   // 上一行 左 中 右
    output wire [7:0]  p21, p22, p23,   // 当前行 左 中 右
    output wire [7:0]  p31, p32, p33    // 下一行 左 中 右
);
```

- 灰度（冻结）：`Y = (77*R + 150*G + 29*B) >> 8`
- 边界（冻结）：首末行、首末列复制边缘像素，不补零。
- 只例化一次；Sobel 与中值并行消费同一组 `p11..p33`。

## 4. I-B `sobel_edge`

消费 9 个灰度像素，输出梯度幅值与边缘标志。

```verilog
module sobel_edge #(
    parameter LATENCY = 0        // 实现后填写并固定
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        vsync_i,
    input  wire        de_i,
    input  wire [7:0]  p11, p12, p13,
    input  wire [7:0]  p21, p22, p23,
    input  wire [7:0]  p31, p32, p33,
    input  wire [7:0]  thr_i,
    output wire        vsync_o,
    output wire        de_o,
    output wire [7:0]  mag_o,
    output wire        edge_o
);
```

- 幅值（冻结）：`mag = |Gx| + |Gy|`，饱和到 8 bit。
- `edge_o = (mag_o > thr_i)`；`thr_i` 复位默认 40。

## 5. I-C `image_mix`

把原图与边缘图合成为一路 RGB888。

```verilog
module image_mix #(
    parameter IMG_W = 1280,
    parameter IMG_H = 720
)(
    input  wire        clk_i,
    input  wire        rst_ni,
    input  wire        vsync_i,
    input  wire        de_i,
    input  wire [23:0] orig_rgb_i,   // 原图，与边缘图同拍
    input  wire [23:0] edge_rgb_i,   // 边缘图
    input  wire [1:0]  mode_i,
    output reg         vsync_o,
    output reg         de_o,
    output reg  [23:0] rgb_o
);
```

| `mode_i` | 输出 |
| --- | --- |
| 0 | 原图 |
| 1 | 边缘图 |
| 2 | 左半原图 + 右半边缘图 |
| 3 | 画中画：边缘图缩放至右下角 1/4 |

- 输出寄存 1 拍，延迟固定为 1。
- `mode_i` 为 2/3 时用行计数器，`IMG_W` / `IMG_H` 必须参数化。

## 6. I-A `image_pipeline_top`

顶层流水线，插在 AWB 之后、`FrameBoundCrop` 之前。

```verilog
module image_pipeline_top #(
    parameter IMG_W = 1280,
    parameter IMG_H = 720
)(
    input  wire        clk_i,        // clk_pixel
    input  wire        rst_ni,
    input  wire        vsync_i,
    input  wire        de_i,
    input  wire [23:0] rgb_i,
    input  wire [7:0]  thr_i,        // 默认 40
    input  wire [1:0]  mode_i,       // 含义见 §5
    output wire        vsync_o,
    output wire        de_o,
    output wire [23:0] rgb_o
);
```

- `mode_i = 0` 时输出与输入逐像素一致（旁路），用于回归对比。
- 对外延迟 `LATENCY` 固定，供 `example_top` 对齐。
