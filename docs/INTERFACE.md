> 顶层固定启用方向插值与灰度抗锯齿，移除 aa_mode_i；公共显示延迟仍为23拍。EDGE_AA 仅用于编译期兼容测试。

# 接口文档

> 本文件是接口的唯一来源。

## 固定灰度抗锯齿集成（当前板级默认）

`example_top` 固定设置 `EDGE_AA=1`，采用方向插值 NMS、子像素峰值定位和灰度覆盖显示（原 mode2）。
没有运行时 AA 模式输入；复位后即生效。`view=0` 输出灰度抗锯齿边缘，其余诊断视图保留。
Canny 的 `aa_o[11:0]` 随 `edge_o` 同拍输出几何信息：主轴、象限、偏移符号、Q4 偏移及 Q4 斜率。
插值 NMS 的前侧严格比较、后侧非严格比较，避免平坦峰的重复保留。
辅助模块与渲染器直接放在现有 canny_edge.v、image_pipeline_top.v，无需增加工程源文件。

显示增加3×3窗口和运算寄存器共4拍，板级 `IMG_LAT=23`，HS、RGB、灰度及分屏统一对齐。
窗口额外增加一行空间位移，`FrameBoundCrop.SKIP_ROWS=3`，`SKIP_COLS=2` 不变。
原图也经过相同窗口中心选择，不能按旧19拍纯寄存器旁路解释其空间位置。
模块参数 `EDGE_AA=0` 保留编译期旧算法兼容；下文19拍、纯寄存器原图旁路和平局保留的描述仅适用于该配置。
板级固定启用时使用23拍、插值判决和上述公共显示对齐，不支持串口切回旧路径。

## 1. 模块一览

| 模块                 | 文件                             | 作用                                                                                          | 接口 |
| -------------------- | -------------------------------- | --------------------------------------------------------------------------------------------- | ---- |
| `rgb_to_gray`        | `src/image/rgb_to_gray.v`        | RGB888 → Y8 灰度（`{Y,Y,Y}`），公式见 `RGB_TO_GRAY.md`                                        | I-B  |
| `pix_window_3x3`     | `src/image/pix_window_3x3.v`     | 3×3 灰度邻域窗口（3 bank 行缓存按行轮转），输入已是 Y8                                        | I-B  |
| `sobel_edge`         | `src/image/sobel.v`              | Sobel 梯度、饱和、阈值比较（**已被 `canny_edge` 替换**，文件保留作对照，见 §4）               | I-B  |
| `canny_edge`         | `src/image/canny_edge.v`         | 硬件近似 Canny：高斯 → Sobel → 方向量化 → NMS → 双阈值滞后 →（可选）方向约束流式反馈，见 §4.5 | I-B  |
| `image_mix`          | `src/display/image_mix.v`        | 原图与边缘图的分屏 / 画中画 / 叠加合成                                                        | I-C  |
| `image_pipeline_top` | `src/image/image_pipeline_top.v` | 流水线顶层，串联上述模块并做旁路                                                              | I-A  |
| `uart_cmd`           | `src/common/uart_cmd.v`          | 串口命令口（调试）：运行时改 `thr / view / mode / smooth`，见 §7                              | —    |

## 2. 全局约定

**端口顺序**（所有时序模块统一）

```text
clk, rst_n, vsync_i, de_i, <data_in>, <ctrl_in>, vsync_o, de_o, <data_out>
```

- `vsync` = 帧有效，`de` = 像素有效；只处理 `de=1` 的像素。
- `data_o` 与 `vsync_o/de_o` 严格同拍；延迟固定，写在模块头注释里。
- 位宽显式写出，如 `[7:0]`、`[23:0]`。

### 像素格式

| 阶段                 | 格式                                                                                                                                                               |
| -------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 算法入口（AWB 之后） | RGB888 = `{R[7:0], G[7:0], B[7:0]}`                                                                                                                                |
| 灰度                 | Y8                                                                                                                                                                 |
| 梯度幅值             | `mag`（= abs(Gx) + abs(Gy)，0..2040），输出时饱和到 255（`mag_o`）；内部判据用 `mag8 = mag >> 3`（**不饱和**，但**实际只用到 0..96 横竖 / 0..140 对角**，见 §4.5） |
| 边缘标志             | 1 bit，`canny_edge`：NMS + 双阈值滞后；`sobel_edge`（已替换）：`mag > thr`                                                                                         |
| 算法出口             | RGB888                                                                                                                                                             |

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
- 2026-10-17 起在流水线里**例化两次**：第一级给 `median_filter_3x3` 提供
  `p11..p33`，中值输出单像素流后再构第二级窗口给 `canny_edge`。中值不再与
  边缘检测并行消费同一组像素——"并行"只能让中值作用于显示，救不了 Canny 的噪声。

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
    input  wire [7:0]  thr_high_i,   // 强边阈值，mag8 = mag>>3 域（**可用量程只有 0..96**）
    input  wire [7:0]  thr_low_i,    // 弱边阈值，同一域
    input  wire [1:0]  smooth_mode_i,// 0 标准高斯 / 1 弱平滑 / 2 旁路（vsync 上升沿锁存）
    input  wire        link_en_i,    // 方向约束流式反馈开关（vsync 上升沿锁存，2026-10-19）
    output reg         vsync_o,
    output reg         de_o,
    output reg  [7:0]  mag_o,
    output reg         edge_o,
    output reg         link_o,       // 仅因切向传播才保留的弱边（调试观测，view=2 绿）
    output reg  [7:0]  mag8_o,       // 未饱和的 mag8（调试观测，M5 增量）
    output reg  [1:0]  cls_o         // 判决分类（调试观测，M5 增量）
);
```

- 算法（冻结）：`g` = 3×3 高斯 `[1 2 1; 2 4 2; 1 2 1]/16`（移位加法，`>>4` 精确）→ 在高斯场上做 Sobel →
  `mag = |Gx| + |Gy|` → 方向量化（只用一个常数 106/256 ≈ tan 22.5°）→ NMS（平局保留）→ 双阈值 + 滞后。
- `smooth_mode_i`（2026-10-17 增量）：`0` 标准高斯 `gsum>>4`；`1` 弱平滑
  `(p22 + gaussian + 1)>>1`；`2` 完全旁路 `p22`。在 `vsync_i` 上升沿锁存，
  帧内切换无接缝；三档延迟同为 12 拍。默认值由按键控制器给 `1`。
- `mag_o` = 未做 NMS、未过阈值的饱和幅值 `min(mag, 255)`；`edge_o` = `strong || (weak && 8 邻域内有 strong) || link`，
  其中 `strong = NMS 过 && mag8 >= thr_high_i`、`weak = NMS 过 && mag8 >= thr_low_i`。
- `link_en_i / link_o`（2026-10-19 增量，只增不改）：`link_en_i=1` 时，弱边若沿**自己的切向**
  接到一个 raster 顺序上已经保留的边缘（`strong`、单遍滞后连上的弱边、或已传播的弱边），
  也保留；`link_en_i=0` 时 `edge_o` 与加此功能前逐位一致。
  · 切向邻居（梯度方向 → 边缘走向）：dir `00`→上、`01`→左、`10`→右上、`11`→左上；
    还要求邻居与本像素的梯度扇区相同或相邻（`00→10→01→11` 角度序，`135°` 与 `0°` 相邻，
    差 2 个扇区 = 90° 拐角，不连）。
  · 反馈存 3 bit/像素 `{dir, edge}`，两个 `IMG_W×3` bank 按行 ping-pong（当前行写、上一行读）。
  · **单向递推**：每条弱链只能从 raster 顺序更早的一端长出来（起点必须是已有的 edge），
    并沿扫描方向继续生长（竖直链向下、水平链向右）；反方向不传播，没有种子的孤立弱链
    仍整条丢弃。前 7 行、左 3 列、右 2 列禁止传播（帧首/列边界暂态），
    参考模型 `sim/canny_ref.py` 用同样的门限。
  · `link_o` 只标"本来会被丢掉、因传播才留下"的弱边（already-strong / 已由单遍滞后保留的像素不置位）。
  · 不增加延迟：`LATENCY` 仍是 12，`link_o` 与 `edge_o` 同拍。
- **阈值域（冻结，重要；2026-10-15 重标定）**：判据在内部 `mag8 = mag >> 3`（不饱和）上，**与 `sobel_edge` 的
  `thr_i`（先饱和到 255 再比）不可混用**。
  ★★ **这个口的有效量程只有 0..96（横竖边）/ 0..140（对角边），不是 0..255**：3×3 高斯把 Sobel 的
  4A 阶跃响应压成了 3A，满对比度（ΔY=255）的横竖边 `mag` 只有 768。按 0..255 折算会**高估 2.7 倍**。
  反查（横竖边，`sim/color_probe.py` 实测）：`thr` 8/16/24/32/40/48 → 要求 ΔY ≥ 21/42/63/85/106/127。
  **标定值 `thr_high = 12`、`thr_low = 6`**（顶层 `thr_i` 复位默认 12，映射成 12/6；2026-10-15 上板扫出来的工作点）；**`thr_high` 不要超过 48**。
  原来的默认值 40 实际要求满幅 42% 的对比度，这就是 2026-10-15 那次"只有高对比度才出边"的根因。
- `mag8_o` / `cls_o`（M5 增量端口，只增不改）：`mag8_o` = 真实的 `mag >> 3`（与 `mag_o` 同拍同像素），
  用来**在屏幕上直接量阈值域** —— `mag_o` 在 `mag8 > 32` 就饱和，阈值平面（16 → mag=128）落在它的
  显示动态范围里刚好能看见，但 40（mag=320）就看不见了，这正是当初调不准阈值的原因。
  `cls_o`：`00` = `mag8 < thr_low`（无响应）／`01` = 有梯度但 NMS 没过／`10` = 弱边／`11` = 强边。
- 延迟（冻结）：**LATENCY = 12 拍**，`vsync_o / de_o / mag_o / edge_o / link_o / mag8_o / cls_o` 严格同拍。
- 空间对齐（实测）：相对原图**下移 3 行、0 列**——`pix_window_3x3` 1 行 + 高斯场窗口 1 行 + 梯度场窗口 1 行。
- 边界（实测，比对时留边）：输出左侧 0..3 列受行首整窗复制影响；帧首第 0..5 行是行缓存没填满的暂态
  （`mag` 第 5 行起干净、`edge` 第 6 行起干净）；末两列有 `mag` = 32/16 的已知小台阶（末列抽头读到消隐期的 0，
  `mag8` = 2..4，低于弱阈值，**不产生 edge**），该现象在本分支接入前就存在（`pix_window_3x3` 单独实例同样有）。
- 资源：额外 2 个高斯场 `1280×8` bank（梯度场另需 2 个 `1280×14` bank）。
  2026-10-19 的流式反馈再加 2 个 `1280×3` bank，**实测共 2 块 EFX_RAM10**
  （每个 bank 1 块），**不增加流水拍数**；整条链路的综合增量
  见 `docs/ARCH.md` §4 的 M8 表。

## 5. I-C `image_mix`

把原图、灰度图与边缘图合成为一路 RGB888。

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
    input  wire [23:0] gray_rgb_i,   // 中值后灰度图 {Y,Y,Y}，与边缘图同拍
    input  wire [23:0] edge_rgb_i,   // 边缘图
    input  wire [1:0]  mode_i,
    output reg         vsync_o,
    output reg         de_o,
    output reg  [23:0] rgb_o
);
```

| `mode_i` | 输出                                                            |
| -------- | --------------------------------------------------------------- |
| 0        | 原图                                                            |
| 1        | 边缘图                                                          |
| 2        | 左半**灰度图** + 右半边缘图（分界列 `IMG_W/2`）                 |
| 3        | 画中画：边缘图缩放至右下角 1/4 —— **暂未实现，当前等同 mode 2** |

- 输出寄存 1 拍，延迟固定为 1；三路输入必须逐像素同拍。
- `IMG_W` / `IMG_H` 必须参数化；当前实现只用列计数器 `col_cnt`（`col_cnt < IMG_W/2` 取 `gray_rgb_i`，否则取边缘图）。
- 顶层建议默认 `mode = 2'd3`；在 mode 3 实现前，上电画面是分屏。

**mode 3 暂未实现的原因**：流式流水线在输出第 y 行时输入只有第 y 行，而右下角要显示整幅边缘图的 1/4 缩放需要回看约 `IMG_H/2` = 360 行，即 640×360×24 ≈ 5.5 Mbit 的帧缓存；Ti60F225 全部 EBR 只有约 2.56 Mbit，放不下（`docs/ARCH.md` §4 的行缓存预算只有约 30 kbit）。后续可选做法：右下象限 1:1 直通边缘图（零缓存），或加一块 1024×24 行缓存做水平 2:1 抽取。两种都只改 `image_mix` 的 case 分支，端口与顶层接线不动。

## 6. I-A `image_pipeline_top`

顶层流水线，插在 AWB 之后、`FrameBoundCrop` 之前。内部串联
`rgb_to_gray(1 拍) → pix_window_3x3(2 拍) → median_filter_3x3(1 拍) →
pix_window_3x3(2 拍) → canny_edge(12 拍) → image_mix(1 拍)`。
中值输出是单像素流，Canny 需要 3×3 邻域，所以中值后还要再构一级窗口
（2026-10-17 接入，用于压掉 thr 调低后的孤立噪点）。

```verilog
module image_pipeline_top #(
    parameter IMG_W       = 1280,
    parameter IMG_H       = 720,
    parameter LATENCY     = 19,      // 对外延迟（拍）
    parameter integer MEDIAN_IMPULSE_ONLY  = 1,      // 1 = 只替换孤立脉冲
    parameter [7:0]   MEDIAN_IMPULSE_DELTA = 8'd24   // 替换阈值（灰度差）
)(
    input  wire        clk_i,        // clk_pixel
    input  wire        rst_ni,
    input  wire        vsync_i,
    input  wire        de_i,
    input  wire [23:0] rgb_i,
    input  wire [7:0]  thr_i,        // 复位默认 12 → 内部 thr_high/thr_low = 12/6（见 §4.5）
    input  wire [1:0]  mode_i,       // 含义见 §5
    input  wire [1:0]  view_i,       // 边缘支路显示什么，见下面【view_i】
    input  wire [1:0]  smooth_mode_i,// Canny 前置平滑：0 标准高斯 / 1 弱平滑 / 2 旁路
    input  wire        link_en_i,    // 方向约束流式反馈：1 开 / 0 退回单遍滞后
    output wire        vsync_o,
    output wire        de_o,
    output wire [23:0] rgb_o,
    output wire [7:0]  dbg_y_o,      // M6 增量：中值后的 Y（Canny 实际输入）
    output wire        dbg_de_o,     // M6 增量：与 dbg_y_o 同拍
    output wire        dbg_vs_o      // M6 增量：与 dbg_y_o 同拍
);
```

- 对外延迟（2026-10-17 由中值接入更新）：**LATENCY = 19 拍** = 1 + 2 + 1 + 2 + 12 + 1；`rgb_o` 与
  `vsync_o/de_o` 严格同拍，`vsync_o` 宽度与输入相同（送到 `rgb2dvi` 的 VS 必须是完整帧同步电平，压成单拍显示器锁不住帧）。
  （接 Canny 之前是 8 拍 = 1 + 2 + 4 + 1；接 Canny 后 16 拍；加中值后 19 拍。）
- 原图走一条 18 拍纯寄存器延迟（不含行缓存），所以 `mode_i = 0` 时输出与输入**逐像素一致**（只差固定 19 拍），用于回归对比。
- 阈值口只保留一个 8 bit（接口 I-A 不变）：`thr_high = thr_i`、`thr_low = thr_i >> 1`（2:1），见 §4.5 的阈值域说明。
  **默认 12**（40 → 16 → 12：40 是标定错，16 是量纲改对，12 是 2026-10-15 上板扫出来的工作点）；
  复位值写在 `example_top.v` 的 `IMG_THR_DEF`，可在板上用串口改（§7）。
- `view_i`（M5 增量端口，只增不改）：边缘支路显示什么，**不影响延迟**（四种视图都在 `image_mix` 之前合成、由它统一寄存）。

  | `view_i` | 边缘支路内容                                                                                                  | 用途                                                                                                                          |
  | -------- | ------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
  | 0        | 二值边缘图（白线黑底）                                                                                        | 默认，出图用                                                                                                                  |
  | 1        | `mag8` 灰度图 = `min(mag8<<1, 255)`                                                                           | **量阈值余量**：thr=16 落在灰度 32，满对比度黑边（mag8=96）落在 192                                                           |
  | 2        | 六状态假彩图：黑=无响应／深红=被 NMS 抑制／黄=弱边未连通／蓝=弱边且滞回连通／**绿=仅因切向传播保留**／白=强边 | **判死器**，一眼分清"阈值问题""NMS 问题""滞后连接问题"还是"传播连接问题"；关 `link_en_i` 后绿色应全部消失，其余颜色逐像素不变 |
  | 3        | Y 灰度图（算法眼里的世界，中值后）                                                                            | **量 ΔY**：屏幕两侧灰度差查 §4.5 的标尺表就知道该不该出边（此视图不做空间对齐，比边缘图高 3 行）                              |

- `smooth_mode_i`：Canny 前置平滑选择，`0` 标准 1:2:1 高斯、`1` 弱平滑 `(p22+高斯)/2`、`2` 完全旁路。
  `canny_edge` 在 `vsync_i` 上升沿锁存该值，**帧内切换不会出现半帧接缝**；三种模式延迟相同（12 拍），
  所以 `LATENCY` 不随平滑模式变化。默认由按键控制器给 `1`。

- `link_en_i`（2026-10-19 增量，只增不改）：方向约束流式反馈开关，`1` 开（复位默认）/ `0` 退回
  原来的单遍滞后。同样在 `canny_edge` 的 `vsync_i` 上升沿锁存，**帧内切换无接缝、不影响延迟**。
  由 `uart_cmd` 的 `'L' <byte>` 运行时改（§7），不用重新综合即可对比开/关；
  `view=2` 下绿色 = 仅因该功能保留的弱边。关掉时 `edge_o` 与加此功能前逐位一致。

- `dbg_y_o / dbg_de_o / dbg_vs_o`（M6 增量端口，只增不改）：直接引出
  **中值滤波之后**、Canny 支路实际看到的 `Y/de/vs`，用来接片内 256 bin 直方图
  （`gray_hist_uart`，§7）。纯组合直连已有寄存器输出，**不增加延迟、不增资源**；
  之所以不从 `example_top` 里再例化一份 `rgb_to_gray`，是为了保证统计口径与
  Canny 完全一致，也避免多 3 个 DSP。`dbg_vs_o` 是 Y 支路的帧同步电平（高有效），
  直方图模块用它的上升沿切帧。要量原始 `rgb_to_gray` 输出，改接 `gray_y/gray_de/gray_vs`。
- 集成约束（`example_top`）：`hsync` 必须按**同一 LATENCY** 延迟后再送 `FrameBoundCrop`
  （当前 19 拍，用 `IMG_LAT` 与顶层 `LATENCY` 共用一个 localparam），否则 `rgb2dvi` 的 HS 与 DE 错位、HDMI 画面横移。
  `FrameBoundCrop → rgb2dvi` 及其下游一切不动。
- 已知可见性问题（待上板定）：边缘图比原图下移 4 行（两级外部窗口 + Canny 内部两级窗口），
  超出 `FrameBoundCrop` 的 `SKIP_ROWS = 2`，帧首还有 4 行暂态落在可见区（比加中值前多 1 行）。
  两种处理：把 `SKIP_ROWS` 加到 6（整幅黑边变厚 6 行），或接受这几行。

## 7. 串口命令口 `uart_cmd`（调试用，非流水线接口）

把 `thr / view / mode / smooth` 从"编译期 localparam"变成"运行时可调"，现场调参不用重新综合
（一次综合约 10 分钟）。**不接入流水线数据通路**，所以不属于 I-A/I-B/I-C 任何一条接口。

```verilog
module uart_cmd #(
    parameter integer CLK_FREQ = 74250000,   // clk_pixel
    parameter integer BAUD     = 460800,     // 分频 161，误差 -0.08%
    parameter [7:0]   THR_DEF  = 8'd12,      // = example_top 的 IMG_THR_DEF
    parameter [1:0]   VIEW_DEF = 2'd0,
    parameter [1:0]   MODE_DEF = 2'd3,
    parameter [1:0]   SMOOTH_DEF = 2'd1,
    parameter         LINK_DEF = 1'b1        // 方向约束流式反馈复位值
)(
    input  wire       clk, rst_n, rx_i,      // rx_i 空闲为高
    output wire       tx_o,
    input  wire [7:0] thr_read_i,    // 实际生效阈值（来自按键控制器），'R' 回读用
    input  wire [1:0] smooth_read_i, // 实际生效平滑模式
    output reg  [7:0] thr_o,
    output reg        thr_valid_o,   // 'T' 写入单拍脉冲，与 thr_o 同拍有效
    output reg  [1:0] view_o,
    output reg  [1:0] mode_o,
    output reg  [1:0] smooth_o,
    output reg        smooth_valid_o,// 'S' 写入单拍脉冲，与 smooth_o 同拍有效
    output reg        link_o,        // 'L' 写入：方向约束流式反馈开关
    output reg  [7:0] gain_r_o,
    output reg  [7:0] gain_b_o,
    output reg        hist_req_o,    // 'H'：单拍脉冲，请求一次直方图（M6）
    output wire       tx_busy_o      // TX 忙（含已排队的 tx_req），给直方图让线（M6）
);
```

| 命令（`T/V/M/A/B/S/L` 为 2 字节帧；`R/H` 为单字节） | 作用                                                                                                                                                                                     |
| --------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `'T' <byte>`                                        | **写入即生效**：`thr_high = byte`（`thr_low = byte>>1`），`thr_valid_o` 出一个单拍脉冲送按键控制器（串口优先于按键，量程 0..255）                                                        |
| `'V' <byte>`                                        | `view = byte[1:0]`（§6 表）                                                                                                                                                              |
| `'M' <byte>`                                        | `mode = byte[1:0]`（§5 表）                                                                                                                                                              |
| `'S' <byte>`                                        | `smooth = byte[1:0]`（>2 按 2 截断），`smooth_valid_o` 出一个单拍脉冲送按键控制器（串口优先于 K1）                                                                                       |
| `'L' <byte>`                                        | `link_en = byte[0]`（2026-10-19）：`1` 开方向约束流式反馈 / `0` 退回单遍滞后。**运行时对比开/关**，不用重新综合；`view=2` 下绿色就是开的时候新增的传播保留像素                           |
| `'A' <byte>`                                        | `gain_r = byte`（AWB 的 **R 通道增益**，1/64 定点：`byte/64`，量程 0..3.98×）。复位值 **112** = ×1.75（2026-10-15 上板对白墙二分调出的中性点；改前写死的是 `409/256` ≈ ×1.59，偏青）     |
| `'B' <byte>`                                        | `gain_b = byte`（AWB 的 B 通道增益，同上）。复位值 **96** = ×1.5 = 老的 `384/256`                                                                                                        |
| `'R'`                                               | 回读，回发 8 字节 `{thr, view, mode, gain_r, gain_b, smooth, link, '\n'}`；`thr`/`smooth` 直接采 `thr_read_i`/`smooth_read_i`，即**按键调过之后的实际生效值**；`link` 是当前流式反馈开关 |
| `'H'`                                               | 请求一次**片内 Y 直方图**；立即无回包，等下一个 vsync 统计完整一帧后回 778 字节（§7.1）                                                                                                  |
| 其他                                                | 一律忽略（回车/换行/空格不会误触发）                                                                                                                                                     |

> **阈值/平滑来源（2026-10-17 起：按键 + 串口双路）**：`image_pipeline_top.thr_i` 与
> `smooth_mode_i` 接 `canny_threshold_key_ctrl`，20 ms 消抖：
> **K1 循环平滑** `1→2→0→1`；**K2 循环阈值** 4..48、**步长 1**，48 再按回到 4
> （回绕，不是饱和）。串口 `'T'`/`'S'` 通过 `thr_valid_o`/`smooth_valid_o`
> 直接写入，**优先于按键且阈值不受步长限制**；串口阈值在 4..48 内时 K2 从该值
> 继续 +1，越界（含 0..3、49..255）时下一次 K2 先回到 4。
> `'R'` 的第 1/6 字节直接采实际生效寄存器，按键改过之后也准确；第 7 字节是 `link_en`。
> `view` / `mode` / `link` / AWB 增益仍由串口实时生效。

> `'A'` / `'B'` 是 2026-10-15 上板实测"**白墙在正常室内白灯下发青**"（= 相机链路 R 通道偏低）
> 之后加的：调这个系数要好几个来回，所以做成运行时旋钮而不是改 `example_top.v` 里的常数反复综合。
> **默认值**：R **112** = ×1.75（比旧的写死值 ×1.59 高约 9.5%，是 10-15 对白墙
> 调出来的中性点），B **96** = ×1.5（与旧的 `384/256` 相同）。不插串口时就用这两个值。
> 调参流程：发 `'A' <byte>` → 发 `'R'` 确认第 4 个字节是刚发的值 → 看画面（对着**白墙/白纸**调，
> 目标是中性白）。`byte/64` 换算：×2.0 = 128(0x80)，×2.5 = 160(0xA0)，×3.0 = 192(0xC0)。

> **★ 参数是"原始字节"，不是十进制文本——上板最容易踩的坑（2026-10-15 实际踩了）**
>
> **注意**：下面用 `T` 举例只是因为它的参数最容易被写成文本；
> 同样的字节规则适用于 `V`/`M`/`S`/`L`/`A`/`B`，它们都会改画面。
>
> `T`/`V`/`M` 后面**紧跟的那个字节就是参数本身**，协议里没有分隔符。想让
> `thr` 变成 16 必须发 `54 10` 两个字节。如果按人话发
> `T 16`（四个字节 `54 20 31 36`），被吃掉的参数是
> **空格 `0x20` = 32**：`thr` 会**静默地**变成 32（这个值还在合法量程内，画面上只是边更少），
> 后面的 `1`/`6` 作为未知命令被忽略。`V 02` 同理 → `view = 0x20 & 3 = 0`，
> **假彩图视图从来没生效过**，屏幕一直是 view 0 的二值边缘图——"边缘只有白边没有别的颜色"
> 是 view 0 的定义，不是观察结果。
>
> 上一段"回车/换行/空格不会误触发"只对**命令字位置**成立；空格出现在**参数位置**时，
> 它就是一个合法的参数值 `0x20`。串口助手请用**十六进制发送模式**：
>
> | 想做的事 | 发送字节 | 回读确认 |
> | --- | --- | --- |
> | 回读当前状态 | `52` | 期望 `0C 00 03 70 60 01 01 0A`（复位后：thr=12、view=0、mode=3、R 增益 112、B 增益 96、smooth=1、link=1） |
> | `thr=5`（画面立即生效） | `54 05` | 第 1 字节 `05` |
> | `thr=6`（画面立即生效） | `54 06` | 第 1 字节 `06` |
> | `thr=7`（画面立即生效） | `54 07` | 第 1 字节 `07` |
> | `view=2`（六状态假彩图） | `56 02` | 第 2 字节 `02` |
> | `view=3`（Y 图，量 ΔY 用） | `56 03` | 第 2 字节 `03` |
> | 关闭流式反馈（对比旧行为） | `4C 00` | 第 7 字节 `00`；view=2 的绿色全部消失 |
> | 打开流式反馈（默认） | `4C 01` | 第 7 字节 `01`；view=2 重新出现绿色 |
> | `mode=0`（旁路原图） | `4D 00` | 第 3 字节 `00` |
> | `mode=2`（左灰度右边缘） | `4D 02` | 第 3 字节 `02` |
> | 平滑 S0 标准高斯 | `53 00` | 第 6 字节 `00` |
> | 平滑 S2 完全旁路 | `53 02` | 第 6 字节 `02` |
> | AWB R 增益 ×2.5 | `41 A0` | 第 4 字节 `A0` |
> | AWB R 增益回到复位默认 ×1.75 | `41 70` | 第 4 字节 `70` |
> | AWB B 增益 ×1.5（默认） | `42 60` | 第 5 字节 `60` |
>
> **纪律：每发一条会改画面的参数命令（`T`/`V`/`M`/`S`/`L`/`A`/`B`），跟一个 `52` 回读**，
> 回包 `{thr, view, mode, gain_r, gain_b, smooth, link, '\n'}` 逐字节核对。
> 下一版 `uart_cmd` 计划给 `T`/`V`/`M` 加自动回显并接受十进制文本（`CANNY_PLAN.md` §12.2），
> 在那之前只能靠这条纪律。

- 时钟：`clk_pixel`（74.25 MHz），与 `image_pipeline_top` **同域，不需要 CDC**。
- 引脚：沿用 `example_top` 早已声明、`Ti60_Demo.peri.xml` 早已分配的 `uart_rx_i / uart_tx_o`
  （`GPIOT_N_16 / GPIOT_P_16`）——这对引脚在本工程里**此前从未被任何 RTL 引用**，本次才接上。
  板上需要 USB-UART 桥（TTL 侧）。
- 复位值 = `example_top` 的 `IMG_THR_DEF / IMG_VIEW_DEF / IMG_MODE_DEF`，**不插串口也正常工作**。
- 两处刻意的省略（见 `uart_cmd.v` 注释）：没有单独设 `thr_low` 的命令字（等滞回按 `CANNY_PLAN.md`
  §3.7 重做再加；`'L'` 已被流式反馈开关占用）；参数帧没有超时保护（调试口，可接受）。
- **回包时机（写主机端脚本时要注意）**：DUT 在收到 `'R'` 的**停止位中点**（≈9.5 位）就结束收字节并
  立即起发，回包的 8 个字节是**背靠背连发**的（字节间隔 = 1 个停止位）。也就是说回包的第一个起始沿
  可能比主机"发完这一帧"还早半个位 —— `rx`（主机→DUT）与 `tx`（DUT→主机）是两根线、互不冲突，
  普通 USB-UART 的接收端是常开的，不会有问题；但**主机端若写成"先发完再打开接收"就会错过第一个
  字节的开头**（现象是回读值整体错位一位，如 0x08 读成 0x84）。仿真台第一版正是这么错的，
  详见 `sim/tb_uart_cmd.v` 里 `send_cmd_nostop` 的注释。
- 回归：`sim/tb_uart_cmd.v`（`sim/run.sh` 第 4 段，用真实参数 74.25 MHz / 460800），
  变异测试 `sim/mut_uart.py`（确认这个台子真的会 FAIL，不是假绿）。

### 7.1 片内灰度直方图回传（`gray_hist_uart`，M6）

没有 HDMI 采集卡时，用这条路径量 Canny 实际输入的 Y 分布。
Y 来自 `image_pipeline_top` 的 `dbg_y_o/dbg_de_o/dbg_vs_o`（§6），统计的是
**3×3 中值之后、Canny 支路同一份灰度**，不是另算的近似（2026-10-17 起；
要量原始 `rgb_to_gray` 输出需改接 `gray_y`）。

`'H'` 到达后模块等下一个 `vsync` 上升沿，统计**完整一帧**，然后占用 TX 线回传：

| 字节       | 内容                                                                       |
| ---------- | -------------------------------------------------------------------------- |
| `0..2`     | `A5 5A 48`（magic + 命令 `'H'`）                                           |
| `3..4`     | `frame_id`，16 bit 小端，每测一帧 +1                                       |
| `5..7`     | `frame_total`，24 bit 小端，本帧统计到的 de 像素数（1280×720 应为 921600） |
| `8..775`   | 256 个 bin × 3 字节小端（20 bit），`bin[i]` = 本帧 Y==i 的像素数           |
| `776..777` | checksum，16 bit 小端 = `sum(bytes[0..775]) & 0xFFFF`                      |

- 长度固定 778 字节，460800 8N1 下约 **16.9 ms**；加上等下一个 vsync（≤ 2 帧），
  从发 `'H'` 到收完通常 **< 50 ms**。
- 每次 `'H'` 只测一帧，回传后自动清零再等下一条命令；`sum(bins)` 应等于
  `frame_total`，可用来验帧边界和累加。
- **仲裁**：测量/回传期间直方图的 `busy_o` 把 `uart_cmd` 的 RX 强制拉成空闲，
  这期间发的命令会被**丢弃**（不排队）；回传前会等 `uart_cmd.tx_busy_o` 落低，
  不会和 `'R'` 的回包撞线。流程是：发 `'H'` → 收满 778 字节 → 再发别的命令。
- 主机脚本：`python sim/read_hist.py --port COM5`（Windows 走 Win32 API、
  Linux/WSL 走 termios，**不需要 pyserial**）。它发 `'H'`、校验
  magic / checksum / `sum(bins)==frame_total`，并用与 `sim/gray_hist.py`
  相同的口径打印直方图和 p1/p50/p99/削顶比例；`--json` 存结果，
  `--file` 可离线重放已保存的流。
- 资源（2026-10-17 `efx_map` 实测）：`u_gray_hist` **225 FF / 56 ADD /
  1 EFX_RAM10 / 0 DSP**，其中 256×20 bit 的 `hist_mem` 占 1 块 RAM10，
  `u_uart_tx` 21 FF。详见 `docs/ARCH.md` §4「M6b」。
- **局限**：只有 256 个 bin，**算不了** `mag8` 过阈比例和空间分布。
  要回答“低对比度人脸轮廓能不能出边”，仍要抓 `view=3` 的整帧图跑
  `sim/gray_hist.py --roi/--grid`；直方图只负责回答“整体偏暗/削顶”。
- 回归：`sim/tb_gray_hist_uart.v`（`sim/run.sh` 第 6 段：帧边界、同 bin 连续
  累加、字节序、checksum、`sum(bins)==total`）；`'H'` 命令本身在
  `sim/tb_uart_cmd.v` 第 10 项。
