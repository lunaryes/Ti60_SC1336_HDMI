# 接口冻结表（INTERFACE.md）

> 本文件是**唯一**的接口来源。任何模块的端口、位宽、`LATENCY`、像素格式以本文件为准。
> 改这里必须走 `rfc/` 分支 + 2 人 approve。
>
> 版本：v1.0（冻结）

---

## 0. 为什么只有这么少的接口

拆分粒度的目的是**让不同的人可以并行开工且不用互相等待**，而不是"让每个算法算子都成为一个模块"。
把行缓存、窗口、Sobel、阈值各拆成一个模块，会得到 4 个接口 + 4 个延迟参数，
集成时要把它们一个个对齐——**接口成本远大于收益**。

因此本设计只保留 **3 条跨人接口**，其余全部收在单个负责人内部：

| 接口 | 谁 ↔ 谁 | 是否需要冻结 | 出现条件 |
| --- | --- | --- | --- |
| **I-A 系统接口** | `example_top` ↔ `image_pipeline_top` | ✅ 必须冻结 | 始终 |
| **I-B 窗口接口** | 窗口发生器 ↔ Sobel / 中值 | ✅ 冻结 | 窗口与算法由不同人负责时 |
| **I-C 合成接口** | `image_pipeline_top` ↔ 显示合成 | ✅ 冻结 | 做分屏 / 画中画 / 叠加时 |

**如果某个交付单元由同一个人从头写到尾，它内部的子模块之间不需要任何接口约定**——
内部想拆几个 `always` 块、几个内部 module、几个文件，都由该负责人自己定。

---

## I-A 系统接口（唯一对外的硬接口）

```verilog
module image_pipeline_top #(
    parameter IMG_W = 1280,
    parameter IMG_H = 720
)(
    input  wire        clk_i,          // clk_pixel，单时钟域
    input  wire        rst_ni,         // 异步复位，低有效
    input  wire        vsync_i,        // 帧有效
    input  wire        de_i,           // 像素有效
    input  wire [23:0] rgb_i,          // {R[7:0], G[7:0], B[7:0]}，来自 AWB 之后
    input  wire [7:0]  thr_i,          // Sobel 阈值，默认 40
    input  wire [1:0]  mode_i,         // 0=原图 1=边缘 2=分屏 3=画中画
    output wire        vsync_o,
    output wire        de_o,
    output wire [23:0] rgb_o           // 接 FrameBoundCrop
);
```

- 6 个输入 + 3 个输出，就这一组。
- **`mode_i = 0` 时输出必须与输入逐像素一致**（旁路，用于回归对比）。
- `LATENCY`（`vsync_o/de_o` 相对 `vsync_i/de_i` 的延迟）由实现方决定并写入本文件，
  但**必须固定**，因为顶层拿它做对齐。

## I-B 窗口接口（窗口发生器 ↔ 消费者）

窗口发生器把一路 RGB888 像素流变成 3×3 邻域，**灰度化也在里面完成**，
这样 Sobel / 中值滤波 / Canny 三个消费者都能直接复用，且都不需要自己再实现行缓存。

```verilog
module pix_window_3x3 #(
    parameter IMG_W = 1280
)(
    input  wire        clk, rst_n,
    input  wire        vsync_i, de_i,
    input  wire [23:0] rgb_i,
    output wire        vsync_o, de_o,
    output wire [7:0]  p11, p12, p13,   // 上一行  左 中 右
                       p21, p22, p23,   // 当前行  左 中 右
                       p31, p32, p33    // 下一行  左 中 右
);
```

**灰度化公式（冻结）**

```
Y = (77*R + 150*G + 29*B) >> 8        // BT.601 整数近似，权重和 = 256
```

**边界策略（冻结）**：图像第 0/1 行与第 0/1 列采用**边缘复制**；
第 `IMG_W-1`/`IMG_W-2` 行同理复制边缘，不补零（避免产生假边缘）。TB 必须覆盖这四种边界。

**消费者约定**：`sobel_edge` 与中值滤波都**直接吃这 9 个像素**，
中值滤波器把 `p22` 换成 9 像素中值后输出单像素，因此它**不需要自己的行缓存**。

## I-C 合成接口（流水线顶层 ↔ 显示合成）

```verilog
module image_mix #(
    parameter IMG_W = 1280,
    parameter IMG_H = 720
)(
    input  wire        clk_i, rst_ni,
    input  wire        vsync_i, de_i,   // orig 与 edge 已严格同拍
    input  wire [23:0] orig_rgb_i,      // 原始彩色图
    input  wire [23:0] edge_rgb_i,      // 边缘图（灰度或着色后）
    input  wire [1:0]  mode_i,
    output reg         vsync_o, de_o,
    output reg  [23:0] rgb_o
);
```

合成模式：

| `mode_i` | 行为 |
| --- | --- |
| 0 | 输出 `orig_rgb_i` |
| 1 | 输出 `edge_rgb_i` |
| 2 | 左半原图 + 右半边缘图 |
| 3 | 画中画：原图为主，边缘图缩放到右下角 1/4 |

> mode 2/3 需要行计数器，`IMG_W`/`IMG_H` 必须参数化。

---

## 统一视频流端口顺序（所有模块遵守）

时序类模块一律按下面的顺序声明端口，便于阅读与审查：

```
clk, rst_n, vsync_i, de_i, <data_in...>, <ctrl_in...>, vsync_o, de_o, <data_out...>
```

- 信号语义：`vsync` = 帧有效，`de` = 像素有效；**只处理 `de=1` 的像素**。
- 数据延迟与同步信号延迟必须**严格相等**（`data_o` 与 `vsync_o/de_o` 同拍），TB 里断言。
- 位宽一律显式 `[n-1:0]`，禁止出现 `[7:0]` 之外的魔法数字。

## 像素格式约定

| 阶段 | 格式 | 说明 |
| --- | --- | --- |
| AWB 之后（算法入口） | RGB888，24 bit，`{R,G,B}` | 高字节是 R |
| 灰度 | Y8，8 bit | 见上方公式 |
| Sobel 幅值 | mag8，8 bit（饱和） | `mag = |Gx| + |Gy|`，超过 255 截断为 255 |
| 边缘判定 | 1 bit | `edge = (mag > thr_i)` |
| 算法出口 | RGB888，24 bit | 边缘图为灰度三通道复制，或着色后 |

## 时钟域

| 时钟 | 频率 | 允许放算法？ |
| --- | --- | --- |
| `clk_pixel` | 74.4 MHz | ✅ **唯一算法域** |
| `clk_sys` | 96 MHz | ❌ |
| `cmos_pclk` | 采集像素时钟 | ❌ |
| `clk_pixel_2x` / `10x` | 148.5 / 742.5 MHz | ❌ |

`Ti60_Demo.pt.sdc` 已把 `clk_sys ↔ clk_pixel`、`clk_sys ↔ cmos_pclk` 声明为 false path，
且已定义 `clk_pixel`。**因此新增算法不需要写任何时序约束，只需要确认没有引入新的跨域路径。**
