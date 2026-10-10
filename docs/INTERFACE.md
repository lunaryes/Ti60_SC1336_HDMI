# 接口文档

> 本文件是接口的唯一来源。

## 1. 模块一览

| 模块 | 文件 | 作用 | 接口 |
| --- | --- | --- | --- |
| `rgb_to_gray` | `src/image/rgb_to_gray.v` | RGB888 → Y8 灰度（`{Y,Y,Y}`），公式见 `RGB_TO_GRAY.md` | I-B |
| `pix_window_3x3` | `src/image/pix_window_3x3.v` | 3×3 灰度邻域窗口（3 bank 行缓存按行轮转），输入已是 Y8 | I-B |
| `sobel_edge` | `src/image/sobel.v` | Sobel 梯度、饱和、阈值比较 | I-B |
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

**时钟域**：算法全部在 `clk_pixel`（74.25 MHz）单域。`clk_sys` / `cmos_pclk` / `clk_pixel_2x|10x` 不放算法。

## 3. I-B `pix_window_3x3`

把 Y8 灰度像素流变成 3×3 邻域窗口。灰度化不在这里做（已冻结移到上一级 `rgb_to_gray`，实现见 `RGB_TO_GRAY.md`）。

```verilog
module pix_window_3x3 #(
    parameter IMG_W = 1280
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        vsync_i,
    input  wire        de_i,
    input  wire [7:0]  gray_i,          // Y8，与输出同列对齐
    output wire        vsync_o,
    output wire        de_o,
    output reg  [7:0]  p11, p12, p13,   // 上一行 左 中 右
    output reg  [7:0]  p21, p22, p23,   // 当前行 左 中 右
    output reg  [7:0]  p31, p32, p33    // 下一行 左 中 右
);
```

- 灰度公式（冻结，在 `rgb_to_gray` 内）：`Y = (77*R + 150*G + 29*B) >> 8`
- 延迟（冻结）：**LATENCY = 2 拍** —— bank 同步读 1 拍 + 窗口寄存 1 拍；`p11..p33` 与 `vsync_o/de_o` 严格同拍。
- 空间对齐：输出窗口中心 = 与输出同拍进入的那一像素的**上一行、左一列**；即边缘图相对原图整体右下偏移 1 行 1 列。这两行两列落在 `FrameBoundCrop` 的 `SKIP_ROWS/SKIP_COLS = 2` 黑边内，不可见。
- 存储：3 个 `IMG_W × 8 bit` bank 按行轮转（1 写 2 读，标准 SDP 写法，推断为 BRAM），实测 128 FF + 149 LUT + 6 个 EFX_RAM10。不用 `src/isp/Line_Shift_RAM_8Bit.v`：它是"延迟 1 行 + 1 像素"（相邻两行固定错 1 列，要 `pixel_cnt` 特判去补），且深度按 2 的幂开、1280 点要占 2048 深，浪费约 40%；`src/isp/` 下原件一行未改。
- 边界（冻结）：**行首整窗复制最近的有效列**，不补零。行方向前两行与行尾末列不做特判——它们落在 `FrameBoundCrop` 的黑边内，同样不可见。
- 只例化一次；Sobel 与中值并行消费同一组 `p11..p33`。

## 4. I-B `sobel_edge`

消费 9 个灰度像素，输出梯度幅值与边缘标志。

```verilog
module sobel_edge #(
    parameter LATENCY = 4        // S1~S4 四级流水
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        vsync_i,
    input  wire        de_i,
    input  wire [7:0]  p11, p12, p13,
    input  wire [7:0]  p21, p22, p23,
    input  wire [7:0]  p31, p32, p33,
    input  wire [7:0]  thr_i,
    output reg         vsync_o,
    output reg         de_o,
    output reg  [7:0]  mag_o,
    output reg         edge_o
);
```

- 幅值（冻结）：`mag = |Gx| + |Gy|`，饱和到 8 bit；`Gx` 取右列减左列、`Gy` 取下行减上行。
- `edge_o = (mag_o > thr_i)`；`thr_i` 复位默认 40。
- 延迟（冻结）：**LATENCY = 4 拍**，与 `mag_o / edge_o` 严格同拍。系数 2 用移位实现，不占 DSP。

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
| 2 | 左半原图 + 右半边缘图（分界列 `IMG_W/2`） |
| 3 | 画中画：边缘图缩放至右下角 1/4 —— **暂未实现，当前等同 mode 2** |

- 输出寄存 1 拍，延迟固定为 1。
- `IMG_W` / `IMG_H` 必须参数化；当前实现只用列计数器 `col_cnt`（`col_cnt < IMG_W/2` 取原图，否则取边缘图）。
- 顶层建议默认 `mode = 2'd3`；在 mode 3 实现前，上电画面是分屏。

**mode 3 暂未实现的原因**：流式流水线在输出第 y 行时输入只有第 y 行，而右下角要显示整幅边缘图的 1/4 缩放需要回看约 `IMG_H/2` = 360 行，即 640×360×24 ≈ 5.5 Mbit 的帧缓存；Ti60F225 全部 EBR 只有约 2.56 Mbit，放不下（`docs/ARCH.md` §4 的行缓存预算只有约 30 kbit）。后续可选做法：右下象限 1:1 直通边缘图（零缓存），或加一块 1024×24 行缓存做水平 2:1 抽取。两种都只改 `image_mix` 的 case 分支，端口与顶层接线不动。

## 6. I-A `image_pipeline_top`

顶层流水线，插在 AWB 之后、`FrameBoundCrop` 之前。内部串联
`rgb_to_gray(1 拍) → pix_window_3x3(2 拍) → sobel_edge(4 拍) → image_mix(1 拍)`。

```verilog
module image_pipeline_top #(
    parameter IMG_W       = 1280,
    parameter IMG_H       = 720,
    parameter EDGE_BINARY = 1,       // 1 = 二值边缘图（白线黑底）；0 = {mag,mag,mag} 灰度梯度图
    parameter LATENCY     = 8        // 对外延迟（拍）
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

- 对外延迟（冻结）：**LATENCY = 8 拍** = 1 + 2 + 4 + 1；`rgb_o` 与 `vsync_o/de_o` 严格同拍，`vsync_o` 宽度与输入相同（送到 `rgb2dvi` 的 VS 必须是完整帧同步电平，压成单拍显示器锁不住帧）。
- 原图走一条 7 拍纯寄存器延迟（不含行缓存），所以 `mode_i = 0` 时输出与输入**逐像素一致**（只差固定 8 拍），用于回归对比。
- 集成约束（`example_top`）：`hsync` 必须按**同一 LATENCY** 延迟后再送 `FrameBoundCrop`（当前 8 拍），否则 `rgb2dvi` 的 HS 与 DE 错位、HDMI 画面横移。`FrameBoundCrop → rgb2dvi` 及其下游一切不动。
