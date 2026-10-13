# 接口文档

> 本文件是接口的唯一来源。

## 1. 模块一览

| 模块 | 文件 | 作用 | 接口 |
| --- | --- | --- | --- |
| `rgb_to_gray` | `src/image/rgb_to_gray.v` | RGB888 → Y8 灰度（`{Y,Y,Y}`），公式见 `RGB_TO_GRAY.md` | I-B |
| `pix_window_3x3` | `src/image/pix_window_3x3.v` | 3×3 灰度邻域窗口（3 bank 行缓存按行轮转），输入已是 Y8 | I-B |
| `sobel_edge` | `src/image/sobel.v` | Sobel 梯度、饱和、阈值比较（**已被 `canny_edge` 替换**，文件保留作对照，见 §4） | I-B |
| `canny_edge` | `src/image/canny_edge.v` | 硬件近似 Canny：高斯 → Sobel → 方向量化 → NMS → 双阈值滞后，见 §4.5 | I-B |
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
| 梯度幅值 | `mag`（= abs(Gx) + abs(Gy)，0..2040），输出时饱和到 255（`mag_o`）；内部判据用 `mag8 = mag >> 3`（0..255，**不饱和**） |
| 边缘标志 | 1 bit，`canny_edge`：NMS + 双阈值滞后；`sobel_edge`（已替换）：`mag > thr` |
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
- 空间对齐：输出窗口中心 = 与输出同拍进入的那一像素的**上一行、左一列**；实测（2026-10-13，`sim/canny_ref.py` 逐点比对）偏移是**下移 1 行、0 列**——窗口的"左一列"与流水线自身的滞后恰好抵消，列方向不偏移。原文写的"右下偏移 1 行 1 列"里行数对、列数不对，已按实测订正。（边缘图相对原图的行偏移还要算上 `canny_edge` 内部两级窗口，见 §4.5。）
- 存储：3 个 `IMG_W × 8 bit` bank 按行轮转（1 写 2 读，标准 SDP 写法，推断为 BRAM），实测 128 FF + 149 LUT + 6 个 EFX_RAM10。不用 `src/isp/Line_Shift_RAM_8Bit.v`：它是"延迟 1 行 + 1 像素"（相邻两行固定错 1 列，要 `pixel_cnt` 特判去补），且深度按 2 的幂开、1280 点要占 2048 深，浪费约 40%；`src/isp/` 下原件一行未改。
- 边界（冻结）：**行首整窗复制最近的有效列**，不补零。行方向前两行与行尾末列不做特判——它们落在 `FrameBoundCrop` 的黑边内，同样不可见。
- 只例化一次；边缘检测（原 Sobel → 现 `canny_edge`）与后续中值并行消费同一组 `p11..p33`。

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
- **已被 `canny_edge` 就地替换**（2026-10-13）：`sobel.v` 文件与本节保留，供分支/图样对照，工程里仍列着但已无实例。

## 4.5 I-B `canny_edge`

硬件近似版 Canny（方案见 `CANNY_PLAN.md`），插槽与原 `sobel_edge` 完全相同，端口只差阈值口。

```verilog
module canny_edge #(
    parameter IMG_W   = 1280,
    parameter LATENCY = 12       // S1~S12 十二级流水
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        vsync_i,
    input  wire        de_i,
    input  wire [7:0]  p11, p12, p13,
    input  wire [7:0]  p21, p22, p23,
    input  wire [7:0]  p31, p32, p33,
    input  wire [7:0]  thr_high_i,   // 强边阈值，mag8 = mag>>3 域
    input  wire [7:0]  thr_low_i,    // 弱边阈值，同一域
    output reg         vsync_o,
    output reg         de_o,
    output reg  [7:0]  mag_o,
    output reg         edge_o
);
```

- 算法（冻结）：`g` = 3×3 高斯 `[1 2 1; 2 4 2; 1 2 1]/16`（移位加法，`>>4` 精确）→ 在高斯场上做 Sobel →
  `mag = |Gx| + |Gy|` → 方向量化（只用一个常数 106/256 ≈ tan 22.5°）→ NMS（平局保留）→ 双阈值 + 滞后。
- `mag_o` = 未做 NMS、未过阈值的饱和幅值 `min(mag, 255)`；`edge_o` = `strong || (weak && 8 邻域内有 strong)`，
  其中 `strong = NMS 过 && mag8 >= thr_high_i`、`weak = NMS 过 && mag8 >= thr_low_i`。
- **阈值域（冻结，重要）**：判据在内部 `mag8 = mag >> 3`（0..255，不饱和）上，**与 `sobel_edge` 的
  `thr_i`（先饱和到 255 再比）不可混用**。顶层 `thr_i` 默认 40 映射成 `thr_high/thr_low = 40/20`。
- 延迟（冻结）：**LATENCY = 12 拍**，`vsync_o / de_o / mag_o / edge_o` 严格同拍。
- 空间对齐（实测）：相对原图**下移 3 行、0 列**——`pix_window_3x3` 1 行 + 高斯场窗口 1 行 + 梯度场窗口 1 行。
- 边界（实测，比对时留边）：输出左侧 0..3 列受行首整窗复制影响；帧首第 0..5 行是行缓存没填满的暂态
  （`mag` 第 5 行起干净、`edge` 第 6 行起干净）；末两列有 `mag` = 32/16 的已知小台阶（末列抽头读到消隐期的 0，
  `mag8` = 2..4，低于弱阈值，**不产生 edge**），该现象在本分支接入前就存在（`pix_window_3x3` 单独实例同样有）。
- 资源：额外 2 个高斯场 `1280×8` bank（梯度场另需 2 个 `1280×14` bank），实测值待 M5 综合后补。

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
`rgb_to_gray(1 拍) → pix_window_3x3(2 拍) → canny_edge(12 拍) → image_mix(1 拍)`。

```verilog
module image_pipeline_top #(
    parameter IMG_W       = 1280,
    parameter IMG_H       = 720,
    parameter EDGE_BINARY = 1,       // 1 = 二值边缘图（白线黑底）；0 = {mag,mag,mag} 灰度梯度图
    parameter LATENCY     = 16       // 对外延迟（拍）
)(
    input  wire        clk_i,        // clk_pixel
    input  wire        rst_ni,
    input  wire        vsync_i,
    input  wire        de_i,
    input  wire [23:0] rgb_i,
    input  wire [7:0]  thr_i,        // 默认 40 → 内部 thr_high/thr_low = 40/20
    input  wire [1:0]  mode_i,       // 含义见 §5
    output wire        vsync_o,
    output wire        de_o,
    output wire [23:0] rgb_o
);
```

- 对外延迟（冻结，2026-10-13 由 M4 更新）：**LATENCY = 16 拍** = 1 + 2 + 12 + 1；`rgb_o` 与
  `vsync_o/de_o` 严格同拍，`vsync_o` 宽度与输入相同（送到 `rgb2dvi` 的 VS 必须是完整帧同步电平，压成单拍显示器锁不住帧）。
  （接 Canny 之前是 8 拍 = 1 + 2 + 4 + 1，`ORIG_LAT` 7 → 15。）
- 原图走一条 15 拍纯寄存器延迟（不含行缓存），所以 `mode_i = 0` 时输出与输入**逐像素一致**（只差固定 16 拍），用于回归对比。
- 阈值口只保留一个 8 bit（接口 I-A 不变）：`thr_high = thr_i`、`thr_low = thr_i >> 1`（2:1），见 §4.5 的阈值域说明。
- 集成约束（`example_top`）：`hsync` 必须按**同一 LATENCY** 延迟后再送 `FrameBoundCrop`
  （当前 16 拍，用 `IMG_LAT` 与顶层 `LATENCY` 共用一个 localparam），否则 `rgb2dvi` 的 HS 与 DE 错位、HDMI 画面横移。
  `FrameBoundCrop → rgb2dvi` 及其下游一切不动。
- 已知可见性问题（待 M5 上板定）：边缘图比原图下移 3 行（§4.5），超出 `FrameBoundCrop` 的 `SKIP_ROWS = 2`，
  帧首还有 3 行暂态落在可见区。两种处理：把 `SKIP_ROWS` 加到 6（整幅黑边变厚 6 行），或接受这几行。
