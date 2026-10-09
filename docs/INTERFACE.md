# 接口文档

> 本文件是接口的唯一来源。

## 1. 模块一览

| 模块 | 文件 | 作用 | 接口 | 状态 |
| --- | --- | --- | --- | --- |
| `pix_window_3x3` | `src/image/pix_window_3x3.v` | 3×3 灰度窗口（灰度化在上游完成），输出 9 个灰度像素 | I-B | 已实现 |
| `sobel_edge` | `src/image/sobel_edge.v` | Sobel 梯度、饱和、阈值比较 | I-B | 已实现 |
| `image_mix` | `src/display/image_mix.v` | 原图与边缘图的分屏合成 | I-C | 已实现（mode 0/1/2） |
| `image_pipeline_top` | `src/image/image_pipeline_top.v` | 流水线顶层，串联上述模块并做旁路 | I-A | 已实现 |
| `sync_delay_align` | `src/common/sync_delay_align.v` | 通用延迟对齐，把旁路路径与算法路径打齐 | — | 已实现 |
| `image_line_shift_ram_8bit` | `src/image/image_line_shift_ram_8bit.v` | `src/isp/Line_Shift_RAM_8Bit.v` 的参考副本 | — | 未采用，不入工程 |

## 2. 全局约定

**端口顺序**（所有时序模块统一）

```
clk, rst_n, vsync_i, de_i, <data_in>, <ctrl_in>, vsync_o, de_o, <data_out>
```

- `vsync` = 帧有效，`de` = 像素有效；只处理 `de=1` 的像素。
- `data_o` 与 `vsync_o/de_o` 严格同拍；延迟固定，写在模块头注释里。
- 位宽显式写出，如 `[7:0]`、`[23:0]`。

**握手信号延迟必须等于数据延迟**（实测踩坑记录）

流水线各级**不得**让 `de_o = de_i` 而数据多延迟 N 拍。两者不一致会让整幅图横向
偏移 N 列并丢掉最右 N 列。各级统一做法：`vsync_o` / `de_o` 用与数据同深度的移位
寄存器延迟。各级延迟如下，`example_top` 需按总和处理 `hsync`：

| 模块 | 数据/握手延迟 |
| --- | --- |
| `pix_window_3x3` | 2 拍（+ 垂直 1 行） |
| `sobel_edge` | 4 拍 |
| `image_mix` | 1 拍 |
| `image_pipeline_top` 合计 | **7 拍**（+ 垂直 1 行） |

**`hsync` 必须同步延迟 7 拍**：`image_pipeline_top` 的端口没有 `hs`，因此
`example_top` 里要用 `sync_delay_align #(.DEPTH(7), .DW(1))` 把 `gray_hs` 也延迟
7 拍，接到 `FrameBoundCrop.hs_i`。否则 HS 与 DE 失去同步，图像会横移 7 个像素。

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

把 RGB888 像素流变成 3×3 灰度邻域。

**灰度化位置（本次修订）**：灰度**不在本模块内做**。上游 `rgb_to_gray` 已经把
画面转成 `{Y,Y,Y}`，本模块只取 R 通道作为亮度，省下 3 个常数乘法器。
因此 `rgb_i` 的端口位宽保持 24 bit 不变，但**前置条件是 `rgb_i[23:16]` 就是 Y**。

```verilog
module pix_window_3x3 #(
    parameter IMG_W = 1280,
    parameter IMG_H = 720        // 保留未用：本模块不做行方向边界处理（见下方"边界"）
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        vsync_i,
    input  wire        de_i,
    input  wire [23:0] rgb_i,           // 前置条件：{Y,Y,Y}
    output wire        vsync_o,
    output wire        de_o,
    output wire [7:0]  p11, p12, p13,   // 上一行 左 中 右（图像第 m-1 行）
    output wire [7:0]  p21, p22, p23,   // 当前行 左 中 右（窗口中心，图像第 m 行）
    output wire [7:0]  p31, p32, p33    // 下一行 左 中 右（图像第 m+1 行）
);
```

- 灰度公式（仍冻结，但已由 `rgb_to_gray` 在上游实现）：`Y = (77*R + 150*G + 29*B) >> 8`
- **边界（2026-10-09 修订；原"复制边缘像素、不补零"条款作废）**：
  本模块**不做任何边界特殊处理**。最外圈像素的窗口会取到越界值，
  **由下游 `FrameBoundCrop` 的 `SKIP_ROWS=2` / `SKIP_COLS=2` 裁剪切边统一兜底**；
  裁剪后可见区域内的窗口完全正确（已逐像素仿真验证，见 `docs/ARCH.md` §11.3）。
  三处越界行为（实测，详见 `docs/ARCH.md` §11.4）：
  - **输出首行**：窗口的"上一行 / 上上行"读到**上一帧末两行的残留**，非复制本帧第 0 行
  - **输出首列**：窗口的"左邻列"取到**第 1 列**，非复制第 0 列
  - **输出末列**：整窗被替换为**第 0 列**的值（行尾读地址回绕，见下方"存储"）
  - 另外代码中**没有行方向判据**，"首行 / 末行复制"从未存在
  需要改变这三处行为时，改动点在 `FrameBoundCrop` 的裁剪行数，**不在本模块**。
  模块内遗留的 `center_bad` / `left_bad` 两个分支**不构成边界实现**（原因见模块头注释）。
- **LATENCY = 2 拍**（垂直 1 行）。`vsync_o` / `de_o` 同延迟 2 拍。
  输出 DE 窗口内第 k 列就是图像第 k 列；输出第 m 行对应图像第 m-1 行。
- 存储：3 个行缓存 bank（按行轮转）+ 预测式读地址（提前一拍发 `col+1`），
  三行天然零错位，每 bank 深度恰为 `IMG_W`。**未使用 `src/isp/Line_Shift_RAM_8Bit.v`**
  （原因见模块头注释），`src/isp/` 原件未改动。
  注：读地址在**行尾回绕到 0**，这是**有意的**——它承担"使下一行第 0 列对齐"的职责。
  副作用是输出末列的"右邻"取到第 0 列，归入上述"不做边界处理"。
  **不要把它改成冻结在 `COL_LAST`**，那会破坏整幅图的列对齐。
- 只例化一次；Sobel 与中值并行消费同一组 `p11..p33`。

## 4. I-B `sobel_edge`

消费 9 个灰度像素，输出梯度幅值与边缘标志。

```verilog
module sobel_edge #(
    parameter LATENCY = 4
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

- 幅值（冻结）：`Gx = (p13+2*p23+p33) - (p11+2*p21+p31)`，`Gy = (p31+2*p32+p33) - (p11+2*p12+p13)`，
  `mag = |Gx| + |Gy|`，饱和到 8 bit。
- `edge_o = (mag_o > thr_i)`；`thr_i` 复位默认 40（由 `example_top` 保持缺省值）。
- **LATENCY = 4 拍**，4 级流水：加权和 → Gx/Gy → 取绝对值 → 求和+饱和+阈值。
- 系数 2 用左移一位（位拼接）实现，**无乘法器、不占 DSP**。
- `|Gx| + |Gy|` 上界 2040，用 11 bit 承载后饱和到 255。

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

| `mode_i` | 输出 | 状态 |
| --- | --- | --- |
| 0 | 原图 | 已实现 |
| 1 | 边缘图 | 已实现 |
| 2 | 左半原图 + 右半边缘图 | 已实现 |
| 3 | 画中画：边缘图缩放至右下角 1/4 | **本轮未实现**，暂回落为边缘图 |

- 输出寄存 1 拍，延迟固定为 1。
- **前置条件**：`orig_rgb_i` 与 `edge_rgb_i` 必须同拍。对齐由 `image_pipeline_top`
  用 `sync_delay_align` 保证，本模块不做任何延迟补偿。
- `mode_i` 为 2/3 时用输入列计数器比较 `IMG_W/2`，`IMG_W` 参数化。
- mode 3 的资源账：右下角 1/4 缩放需要一块 640×360 ≈ 1.8 Mbit 的缓冲，且要在
  写当前帧的同时读出来输出，BRAM 占用可能顶到预算，故后置。

## 6. I-A `image_pipeline_top`

顶层流水线，插在 AWB 之后、`FrameBoundCrop` 之前（全工程唯一插入点）。

```verilog
module image_pipeline_top #(
    parameter IMG_W       = 1280,
    parameter IMG_H       = 720,
    parameter EDGE_BINARY = 1,       // 1 = 二值边缘图；0 = 梯度幅值灰度图
    parameter LATENCY     = 7        // 对外延迟（拍）
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

- **旁路对齐（关键，文档原稿没写清）**：`mode_i = 0` 时输出与输入一致，靠的不是
  简单地把 `rgb_i` 接到 MUX。原图取 `pix_window_3x3` 输出的中心像素 `p22`
  （与 Sobel 的窗口中心天然同源同拍，省掉一整块行缓存），再用
  `sync_delay_align #(.DEPTH(4))` 补上 `sobel_edge` 的 4 级流水延迟。
  两条路径延迟完全相同后才进 `image_mix` 的 MUX。
- **对外延迟 `LATENCY = 7` 拍**（+ 垂直 1 行），即
  `vsync_o/de_o` 相对 `vsync_i/de_i` 延迟 7 拍。`example_top` 必须把 `hsync`
  也延迟 7 拍（见 §2）。
- **回归判据修正**：由于固有的垂直 1 行延迟，`mode_i = 0` 的输出与基线**不是逐
  像素一致**，而是**整体下移 1 行**。该行落在 `FrameBoundCrop` 的最外 2 行空白
  区内，不可见，但回归对比时要把这 1 行偏移算进去。
- 边缘图默认输出**二值图**（`edge_o ? 白 : 黑`），这样 `thr_i` 的效果直观可见；
  改 `EDGE_BINARY = 0` 可切换为梯度幅值灰度图（铅笔素描效果）。
