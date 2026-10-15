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
| `uart_cmd` | `src/common/uart_cmd.v` | 串口命令口（调试）：运行时改 `thr / view / mode`，见 §7 | — |

## 2. 全局约定

**端口顺序**（所有时序模块统一）

```text
clk, rst_n, vsync_i, de_i, <data_in>, <ctrl_in>, vsync_o, de_o, <data_out>
```

- `vsync` = 帧有效，`de` = 像素有效；只处理 `de=1` 的像素。
- `data_o` 与 `vsync_o/de_o` 严格同拍；延迟固定，写在模块头注释里。
- 位宽显式写出，如 `[7:0]`、`[23:0]`。

### 像素格式

| 阶段 | 格式 |
| --- | --- |
| 算法入口（AWB 之后） | RGB888 = `{R[7:0], G[7:0], B[7:0]}` |
| 灰度 | Y8 |
| 梯度幅值 | `mag`（= abs(Gx) + abs(Gy)，0..2040），输出时饱和到 255（`mag_o`）；内部判据用 `mag8 = mag >> 3`（**不饱和**，但**实际只用到 0..96 横竖 / 0..140 对角**，见 §4.5） |
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
    input  wire [7:0]  thr_high_i,   // 强边阈值，mag8 = mag>>3 域（**可用量程只有 0..96**）
    input  wire [7:0]  thr_low_i,    // 弱边阈值，同一域
    output reg         vsync_o,
    output reg         de_o,
    output reg  [7:0]  mag_o,
    output reg         edge_o,
    output reg  [7:0]  mag8_o,       // 未饱和的 mag8（调试观测，M5 增量）
    output reg  [1:0]  cls_o         // 判决分类（调试观测，M5 增量）
);
```

- 算法（冻结）：`g` = 3×3 高斯 `[1 2 1; 2 4 2; 1 2 1]/16`（移位加法，`>>4` 精确）→ 在高斯场上做 Sobel →
  `mag = |Gx| + |Gy|` → 方向量化（只用一个常数 106/256 ≈ tan 22.5°）→ NMS（平局保留）→ 双阈值 + 滞后。
- `mag_o` = 未做 NMS、未过阈值的饱和幅值 `min(mag, 255)`；`edge_o` = `strong || (weak && 8 邻域内有 strong)`，
  其中 `strong = NMS 过 && mag8 >= thr_high_i`、`weak = NMS 过 && mag8 >= thr_low_i`。
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
- 延迟（冻结）：**LATENCY = 12 拍**，`vsync_o / de_o / mag_o / edge_o / mag8_o / cls_o` 严格同拍。
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
    parameter LATENCY     = 16       // 对外延迟（拍）
)(
    input  wire        clk_i,        // clk_pixel
    input  wire        rst_ni,
    input  wire        vsync_i,
    input  wire        de_i,
    input  wire [23:0] rgb_i,
    input  wire [7:0]  thr_i,        // 复位默认 12 → 内部 thr_high/thr_low = 12/6（见 §4.5）
    input  wire [1:0]  mode_i,       // 含义见 §5
    input  wire [1:0]  view_i,       // 边缘支路显示什么，见下面【view_i】
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
  **默认 12**（40 → 16 → 12：40 是标定错，16 是量纲改对，12 是 2026-10-15 上板扫出来的工作点）；
  复位值写在 `example_top.v` 的 `IMG_THR_DEF`，可在板上用串口改（§7）。
- `view_i`（M5 增量端口，只增不改）：边缘支路显示什么，**不影响延迟**（四种视图都在 `image_mix` 之前合成、由它统一寄存）。

  | `view_i` | 边缘支路内容 | 用途 |
  | --- | --- | --- |
  | 0 | 二值边缘图（白线黑底） | 默认，出图用 |
  | 1 | `mag8` 灰度图 = `min(mag8<<1, 255)` | **量阈值余量**：thr=16 落在灰度 32，满对比度黑边（mag8=96）落在 192 |
  | 2 | 四类假彩图：黑=无响应／深红=被 NMS 抑制／蓝=弱边／白=强边 | **判死器**，一眼分清"阈值问题"还是"逻辑问题" |
  | 3 | Y 灰度图（算法眼里的世界） | **量 ΔY**：屏幕两侧灰度差查 §4.5 的标尺表就知道该不该出边（此视图不做空间对齐，比边缘图高 3 行） |

- 集成约束（`example_top`）：`hsync` 必须按**同一 LATENCY** 延迟后再送 `FrameBoundCrop`
  （当前 16 拍，用 `IMG_LAT` 与顶层 `LATENCY` 共用一个 localparam），否则 `rgb2dvi` 的 HS 与 DE 错位、HDMI 画面横移。
  `FrameBoundCrop → rgb2dvi` 及其下游一切不动。
- 已知可见性问题（待 M5 上板定）：边缘图比原图下移 3 行（§4.5），超出 `FrameBoundCrop` 的 `SKIP_ROWS = 2`，
  帧首还有 3 行暂态落在可见区。两种处理：把 `SKIP_ROWS` 加到 6（整幅黑边变厚 6 行），或接受这几行。

## 7. 串口命令口 `uart_cmd`（调试用，非流水线接口）

把 `thr / view / mode` 从"编译期 localparam"变成"运行时可调"，现场调参不用重新综合
（一次综合约 10 分钟）。**不接入流水线数据通路**，所以不属于 I-A/I-B/I-C 任何一条接口。

```verilog
module uart_cmd #(
    parameter integer CLK_FREQ = 74250000,   // clk_pixel
    parameter integer BAUD     = 460800,     // 分频 161，误差 -0.08%
    parameter [7:0]   THR_DEF  = 8'd12,      // = example_top 的 IMG_THR_DEF
    parameter [1:0]   VIEW_DEF = 2'd0,
    parameter [1:0]   MODE_DEF = 2'd3
)(
    input  wire       clk, rst_n, rx_i,      // rx_i 空闲为高
    output wire       tx_o,
    output reg  [7:0] thr_o,
    output reg  [1:0] view_o,
    output reg  [1:0] mode_o
);
```

| 命令（2 字节帧：命令字符 + 1 字节参数） | 作用 |
| --- | --- |
| `'T' <byte>` | `thr_high = byte`（`thr_low` 自动 = `byte>>1`，保持 2:1） |
| `'V' <byte>` | `view = byte[1:0]`（§6 表） |
| `'M' <byte>` | `mode = byte[1:0]`（§5 表） |
| `'A' <byte>` | `gain_r = byte`（AWB 的 **R 通道增益**，1/64 定点：`byte/64`，量程 0..3.98×）。复位值 **112** = ×1.75（2026-10-15 上板对白墙二分调出的中性点；改前写死的是 `409/256` ≈ ×1.59，偏青） |
| `'B' <byte>` | `gain_b = byte`（AWB 的 B 通道增益，同上）。复位值 **96** = ×1.5 = 老的 `384/256` |
| `'R'` | 回读，回发 6 字节 `{thr, view, mode, gain_r, gain_b, '\n'}` |
| 其他 | 一律忽略（回车/换行/空格不会误触发） |

> `'A'` / `'B'` 是 2026-10-15 上板实测"**白墙在正常室内白灯下发青**"（= 相机链路 R 通道偏低）
> 之后加的：调这个系数要好几个来回，所以做成运行时旋钮而不是改 `example_top.v` 里的常数反复综合。
> **默认行为不变**：复位值 102/96 与老常数只差 0.25%/0%，不插串口时画面与改动前一致。
> 调参流程：发 `'A' <byte>` → 发 `'R'` 确认第 4 个字节是刚发的值 → 看画面（对着**白墙/白纸**调，
> 目标是中性白）。`byte/64` 换算：×2.0 = 128(0x80)，×2.5 = 160(0xA0)，×3.0 = 192(0xC0)。

> **★ 参数是"原始字节"，不是十进制文本——上板最容易踩的坑（2026-10-15 实际踩了）**
>
> `T`/`V`/`M` 后面**紧跟的那个字节就是参数本身**，协议里没有分隔符。想设 `thr=16`
> 必须发 `54 10` 两个字节。如果按人话发 `T 16`（四个字节 `54 20 31 36`），被吃掉的参数是
> **空格 `0x20` = 32**：`thr` 会**静默地**变成 32（这个值还在合法量程内，所以看不出错），
> 后面的 `1`/`6` 作为未知命令被忽略。`V 02` 同理 → `view = 0x20 & 3 = 0`，
> **假彩图视图从来没生效过**，屏幕一直是 view 0 的二值边缘图——"边缘只有白边没有别的颜色"
> 是 view 0 的定义，不是观察结果。
>
> 上一段"回车/换行/空格不会误触发"只对**命令字位置**成立；空格出现在**参数位置**时，
> 它就是一个合法的参数值 `0x20`。串口助手请用**十六进制发送模式**：
>
> | 想做的事 | 发送字节 | 回读确认 |
> | --- | --- | --- |
> | 回读当前状态 | `52` | 期望 `0C 00 03 70 60 0A`（复位后：thr=12、view=0、mode=3、R 增益 112、B 增益 96） |
> | `thr=16`（`thr_low` 自动 8） | `54 10` | 第 1 字节 `10` |
> | `thr=8`（弱边 4，噪声余量最小） | `54 08` | 第 1 字节 `08` |
> | `view=2`（四类假彩图） | `56 02` | 第 2 字节 `02` |
> | `view=3`（Y 图，量 ΔY 用） | `56 03` | 第 2 字节 `03` |
> | `mode=0`（旁路原图） | `4D 00` | 第 3 字节 `00` |
> | AWB R 增益 ×2.5 | `41 A0` | 第 4 字节 `A0` |
> | AWB R 增益回到复位默认 ×1.75 | `41 70` | 第 4 字节 `70` |
> | AWB B 增益 ×1.5（默认） | `42 60` | 第 5 字节 `60` |
>
> **纪律：每发一条参数命令，跟一个 `52` 回读**，回包 `{thr, view, mode, gain_r, gain_b, '\n'}`
> 逐字节核对。
> 下一版 `uart_cmd` 计划给 `T`/`V`/`M` 加自动回显并接受十进制文本（`CANNY_PLAN.md` §12.2），
> 在那之前只能靠这条纪律。

- 时钟：`clk_pixel`（74.25 MHz），与 `image_pipeline_top` **同域，不需要 CDC**。
- 引脚：沿用 `example_top` 早已声明、`Ti60_Demo.peri.xml` 早已分配的 `uart_rx_i / uart_tx_o`
  （`GPIOT_N_16 / GPIOT_P_16`）——这对引脚在本工程里**此前从未被任何 RTL 引用**，本次才接上。
  板上需要 USB-UART 桥（TTL 侧）。
- 复位值 = `example_top` 的 `IMG_THR_DEF / IMG_VIEW_DEF / IMG_MODE_DEF`，**不插串口也正常工作**。
- 两处刻意的省略（见 `uart_cmd.v` 注释）：没有单独设 `thr_low` 的 `'L'`（等滞回按 `CANNY_PLAN.md`
  §3.7 重做再加）；参数帧没有超时保护（调试口，可接受）。
- **回包时机（写主机端脚本时要注意）**：DUT 在收到 `'R'` 的**停止位中点**（≈9.5 位）就结束收字节并
  立即起发，回包的 4 个字节是**背靠背连发**的（字节间隔 = 1 个停止位）。也就是说回包的第一个起始沿
  可能比主机"发完这一帧"还早半个位 —— `rx`（主机→DUT）与 `tx`（DUT→主机）是两根线、互不冲突，
  普通 USB-UART 的接收端是常开的，不会有问题；但**主机端若写成"先发完再打开接收"就会错过第一个
  字节的开头**（现象是回读值整体错位一位，如 0x08 读成 0x84）。仿真台第一版正是这么错的，
  详见 `sim/tb_uart_cmd.v` 里 `send_cmd_nostop` 的注释。
- 回归：`sim/tb_uart_cmd.v`（`sim/run.sh` 第 4 段，用真实参数 74.25 MHz / 460800），
  变异测试 `sim/mut_uart.py`（确认这个台子真的会 FAIL，不是假绿）。
