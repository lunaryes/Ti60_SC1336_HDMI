# 架构说明（ARCH.md）

## 1. 基线数据流（现成，不动）

```
SC1336 摄像头
   │ cmos_pclk
   ▼
CMOS_Capture_RAW_Gray ──► Sensor_Image_XYCrop ──► ddr_rw_ctrl
                                                     │
                                          （跨时钟域 FIFO，已有）
                                                     ▼
                                              DDR3 (DdrCtrl IP)
                                                     │
                                                     ▼  clk_pixel
                                          bayer2rgb / VIP_RAW8_RGB888
                                               │  内含 VIP_Matrix_Generate_3X3_8Bit
                                               │      └─ u0/u1_Line_Shift_RAM_8Bit  ← 活链路，只读
                                               ▼
                                             AWB / 色彩处理
                                               │
                                               ▼
                                        ★ 算法流水线插入点 ★
                                               │
                                               ▼
                                          FrameBoundCrop
                                               │
                                               ▼
                                     lcd_driver / rgb2dvi / TMDS
                                               ▼
                                          HDMI 1280×720
```

**关键结论**：算法只需插在 AWB 之后、`FrameBoundCrop` 之前这一个位置。
采集链路和显示链路都不用改，因此两条链路的人可以全程并行、互不阻塞。

## 2. 新增流水线内部结构

```
rgb_i[23:0] ──► rgb_to_gray ──► pix_window_3x3 ──► median_filter_3x3 ──┐
                                                                       ▼
                                                            pix_window_3x3 ──► canny_edge ──► mag/edge
                                                                                                    │
                                                                     mode 选择 / 着色 / 旁路 MUX ◄──┘
                                                                         │
                                                                   image_mix
                                                                         ▼
                                                            rgb_o[23:0] ──► FrameBoundCrop
```

中值滤波**串在 Canny 之前**（2026-10-17 接入）：`median_filter_3x3` 吃前级
`pix_window_3x3` 的 9 个像素、输出**一个**中值像素，而 Canny 需要中值后的 3×3
邻域，所以中值之后必须再例化一级 `pix_window_3x3`。代价是**多一组行缓存
（3 × 1280 × 8 bit，实测 6 块 RAM10 —— 每个 1280 深的 bank 要 2 块 1024 深
RAM10，3 个 bank 共 6 块）和多 3 拍延迟**（中值 1 + 后级窗口 2）。
早期的"中值与 Canny 并行、共享 9 像素、零额外 BRAM"方案只能让中值作用于
显示，救不了 Canny 的噪声，已废弃。

`canny_edge` 内部比 `sobel_edge` 多两级 3×3 窗口，需要自己的行历史：

- 3×3 高斯把 3 行压成 1 行，但 Sobel 还要高斯场的上下各一行 → **高斯场 2 个
  `1280×8` bank**（4 块 EFX_RAM10）；
- NMS 要梯度场的 3×3 邻域 → **梯度场 2 个 `1280×14` bank**（字宽 14 =
  `{mag[11:0], dir[1:0]}`，方案 §11 决议 3 的精确版）。

两级都是"同步读 + 预测读地址（发 col+1）"的 SDP 写法（§6 的坑）。

Canny 前置高斯有**三档运行时模式**（2026-10-17 从 `debug_canny_hardware_diagnosis`
分支合入）：`0` 标准 1:2:1 高斯、`1` 弱平滑 `(p22 + 高斯)/2`、`2` 完全旁路。
`canny_edge` 在 `vsync_i` 上升沿锁存模式，避免切换落在帧中间；三档走同一组
流水寄存器，**延迟不变（12 拍）**，代价只有几个 2:1/3:1 MUX。

## 3. 时钟域

全部在 `clk_pixel`（74.25 MHz）单域。详见 `INTERFACE.md` §2。

## 4. 资源预算（初步）

| 资源 | 预估占用 | 说明 |
| --- | --- | --- |
| 行缓存 BRAM | 3 × 1280 × 8 bit ≈ 30 kbit | 第一级 `pix_window_3x3`（中值输入），按行轮转、1 写 2 读 |
| 行缓存 BRAM（中值后新增） | 3 × 1280 × 8 bit ≈ 30 kbit | 第二级 `pix_window_3x3`（Canny 输入），2026-10-17 加中值后需要 |
| 行缓存 BRAM（Canny 新增） | 2 × 1280 × 8 + 2 × 1280 × 14 ≈ 56 kbit | 高斯场窗口 + 梯度场窗口，各 2 个 bank（见 §2） |
| 乘法器 / DSP | 3（灰度化）+ 0 | 灰度系数是常数、可用移位加法；Sobel/Canny 的系数 2 与 106 全用移位加法，**不占 DSP** |
| 逻辑 LUT | 数百~千级 | 边缘检测流水 + 合成；Canny 预估比 Sobel 多 300~600 |

目标总占用不超过 Ti60F225 的 70%，由集成阶段持续跟踪。

### 实测（2026-10-15，Efinity 2026.1 `efx_map`，工程含全部外设；M5 落定后的最新一次）

器件合计（map 日志的 Resource Summary，两次同一口径，可直接相减）：

| 资源 | M4（含 `canny_edge`，10-13 日志） | M5（本次） | M5 增量 |
| --- | --- | --- | --- |
| `EFX_FF` | 3859 | 3979 | **+120** |
| `EFX_LUT4` | 4680 | 4913 | **+233** |
| `EFX_ADD` | 1221 | 1221 | 0 |
| `EFX_SRL8` | 578 | 594 | +16 |
| `EFX_RAM10` | 66 | 66 | **0** |
| `EFX_DSP24` / `EFX_DSP48` | 3 / 1 | 3 / 1 | 0 |

M5 的 +120 FF / +233 LUT 就是 `mag8_o`/`cls_o`、4 种调试视图（含 14×8 的 Y 延迟链）和
`uart_cmd` 的全部代价，预案估的是"合计 < 200 FF"，实际 120。**RAM 一块没多。**

**M6（2026-10-15，AWB 的 R/B 增益改成运行时寄存器）** —— 同一口径，可与上表直接相减：

| 资源 | M5 | **M6** | 增量 |
| --- | --- | --- | --- |
| `EFX_FF` | 3979 | 4004 | **+25** |
| `EFX_LUT4` | 4913 | 4932 | +19 |
| `EFX_ADD` | 1221 | 1212 | −9 |
| `EFX_SRL8` | 594 | 594 | 0 |
| `EFX_RAM10` | 66 | 66 | 0 |
| `EFX_DSP24` | 3 | **5** | **+2** |
| `EFX_DSP48` | 1 | 本次报告未列出（记 0） | — |

+2 个 DSP 是 `example_top.v` 里 AWB 的两个增益从**常数乘法**（`* 409`）变成**变量乘法**（8×8）——
这是"现场调白平衡"必须付的代价。`u_uart_cmd` 由 79 FF 涨到 **96 FF**（两个 8 位增益寄存器 +
`tx_idx` 由 2 位扩到 3 位），`u_image_pipeline` 709 FF（+2，边界优化抖动，非功能变化）。

**M6b（2026-10-17，按键阈值控制器 + 片内灰度直方图）** —— 同一口径，与上表直接相减：

| 资源 | M6（10-15） | 本次 | 增量 |
| --- | --- | --- | --- |
| `EFX_FF` | 4004 | 4288 | **+284** |
| `EFX_LUT4` | 4932 | 5401 | +469 |
| `EFX_ADD` | 1212 | 1315 | +103 |
| `EFX_SRL8` | 594 | 594 | 0 |
| `EFX_RAM10` | 66 | 67 | **+1** |
| `EFX_DSP24` | 5 | 5 | 0 |

10-15 之后新增的是两块独立功能，按 post-map 网表拆开：

- `u_canny_thr_key`（按键阈值控制器）：**50 FF / 47 ADD**；
- `u_gray_hist`（直方图 + `u_uart_tx`）：**225 FF / 56 ADD / 1 RAM10**；
- `u_uart_cmd` 96 → **104 FF**（`'H'` 命令 + `tx_busy_o`）；
- `u_image_pipeline` 709 → **698 FF**（−11，边界优化抖动，非功能变化）。

256×20 bit 的 `hist_mem` 被推断成 **1 块 EFX_RAM10**（`VERI-2571: extracting RAM for
identifier 'hist_mem'`），比预算的 2 块省一块。`dbg_y/de/vs` 三个抽头是纯组合直连，
没有引入额外寄存器（流水线 FF 反而少了 11）。

分子树（`python3 sim/count_map.py outflow/Ti60_Demo.map.v`，数 post-map 网表的实例路径）：

| 子树（实例路径） | FF | ADD | SRL8 | RAM10 | DSP |
| --- | --- | --- | --- | --- | --- |
| `u_image_pipeline`（整条流水线） | 707 | 298 | 67 | 16 | 3 |
| ├ `u_canny_edge` | 494 | 257 | 3 | 10 | 0 |
| ├ `u_pix_window_3x3` | 63 | 0 | 0 | 6 | 0 |
| ├ `u_rgb_to_gray` | 0 | 31 | 0 | 0 | 3 |
| └ `u_image_mix` | 11 | 10 | 0 | 0 | 0 |
| `u_uart_cmd`（顶层，不在流水线内） | 79 | 0 | 0 | 0 | 0 |

三条读表须知（都是这次实测撞出来的，不写下来下次还会撞）：

1. **"每个模块用了多少 LUT"从 post-map 网表里数不出来，别浪费时间**：优化会把组合逻辑
   改写成合成名（`LUT__12345`、`CutToMuxOpt_*`），**丢掉层次路径**。实测路径保留率
   FF/ADD/RAM10 = 100%，LUT4 = **0.2%**。所以分模块表里没有 LUT 列。
2. **网表数和日志数不能混用**：网表里只有 1817 个 FF，而日志说 3979 —— 硬核/IP 内部的
   单元没有展开进网表。比"这次改了多少"用日志，比"哪个模块贵"用网表。
3. Canny 的代价集中在两级新增窗口：`u_canny_edge` 自己 494 FF + 10 块 RAM10，
   流水线的 RAM10 由 6（只有 3×3 窗口）涨到 16 —— 与 §2 的预算一致。

> **本节原有的一行 2026-10-10 实测已撤掉**（`u_pix_window_3x3` 128 FF / 149 LUT / 6 RAM10，
> `u_image_pipeline` 294 FF / 243 LUT / 6 RAM10）。撤掉的原因：其中 RAM10 = 6 与本次一致，
> 但 **FF/LUT 两列用现在的方法复现不出来**（本次 `u_pix_window_3x3` 是 63 FF），当时的统计
> 口径已无从考证；留着会和可复现的数字打架，历史值看 git。
> 连带地，"接 Sobel 时 294 FF、换 Canny 预估 +200"这类推断也不再引用 —— 看上面的实测增量。

**M7（2026-10-17，Canny 前串入 3×3 中值 + UART `'T'` 阈值回接；
同日合入 `debug_canny_hardware_diagnosis` 的平滑三档、五状态假彩图、分屏左灰度）** —— 同一口径：

| 资源 | M6b | **M7（合并后）** | 增量 |
| --- | --- | --- | --- |
| `EFX_FF` | 4288 | 4472 | **+184** |
| `EFX_LUT4` | 5401 | 6222 | **+821** |
| `EFX_ADD` | 1315 | 1343 | **+28** |
| `EFX_SRL8` | 594 | 594 | 0 |
| `EFX_RAM10` | 67 | 73 | **+6** |
| `EFX_DSP24` | 5 | 5 | 0 |

- 上表是合入平滑三档、五状态假彩图、分屏左灰度**之后**的 map（2026-10-17 16:33）。
  相对合并前同一天的 M7 测量（4463 FF / 6141 LUT4 / 1333 ADD），这三项分别
  **+9 / +81 / +10**；SRL8 / RAM10 / DSP 不变。ADD 的 +10 是弱平滑档的
  `(p22 + gaussian + 1) >> 1`，LUT 的 +81 主要来自五状态译码和分屏三选一。
- 第二级 `pix_window_3x3`（`u_pix_window_post_median`，综合截断为
  `u_pix_window_post_media`）：**62 FF / 6 RAM10**，与第一级（63 FF / 6 RAM10）
  基本同价；RAM 增量正好是它。
- `u_median_filter_3x3`：**18 ADD / 0 RAM / 0 DSP**。post-map 里它的输出寄存器
  被优化成父层网络名（该 bucket 显示 0 FF），**不要据此认为中值没有寄存器**；
  按器件总量看，FF 的 +184 主要是第二级窗口（+62）、Y 视图延迟链 12→14 拍
  （+16）和边界优化搬移。
- LUT **+821** 是这次的大头，来自中值的比较/选择网络、第二级窗口的地址逻辑、
  平滑三档 MUX 与五状态/分屏显示逻辑；这一项只能由器件总量相减得到，
  post-map 本次带路径 LUT 保留率 **0%**。
- 顶层 `u_image_pipeline` 子树 868 FF / 326 ADD / 67 SRL8 / 22 RAM10 / 3 DSP。
  子模块行（canny 496 / 窗口 63 / 窗口 62 / median 0）**加不出 868**：map 会把
  端口处的寄存器改成父层网络名（本次 `p11..p33` 占 144 FF、`q11..q33` 占 72 FF）。
  比"哪个模块贵"看子模块行，比"这次改了多少"看上面的器件总量。

**重跑方法**（工程里**没有**现成的综合脚本，`work_syn/` 是工具产物、随时会被清掉）：

```bat
:: 必须先 call setup.bat，否则 efx_map 启动即 EXCEPTION_ACCESS_VIOLATION（见 §6）
call C:\Efinity\2026.1\bin\setup.bat
efx_map --project Ti60_Demo --root example_top --device Ti60F225 --family Titanium ^
        --prj <工程目录>\Ti60_Demo.xml --work-dir <工程目录>\work_syn ^
        --output-dir <工程目录>\outflow --write-efx-verilog <…>\outflow\Ti60_Demo.map.v ^
        --write-premap-module <…>\outflow\Ti60_Demo.elab.vdb ^
        --binary-db <…>\outflow\Ti60_Demo.vdb ^
        --veri_option "verilog_mode=verilog_2k,vhdl_mode=vhdl_2008" ^
        --I <工程目录> --I <工程目录>\ip\<每个 IP 子目录，共 11 个>
```

### M5 调试可观测性的增量（预估 → 实测）

| 改动 | 预估 | 实测 |
| --- | --- | --- |
| `canny_edge` 加 `mag8_o` / `cls_o` | +10 FF／级（S11、S12 各一组），LUT 可忽略 | 含在 `u_canny_edge` 的 494 FF 里 |
| 4 种调试视图（Y 延迟链 14×8 + 24 bit 4:1 mux） | 112 FF + ≈50 LUT | 见上表器件增量 |
| `uart_cmd`（新建） | ≈100~150 LUT + 数十 FF，无 RAM/无 DSP | **79 FF**、0 RAM、0 DSP（`u_uart_cmd` 行） |
| **合计** | < 200 FF | **+120 FF / +233 LUT / +16 SRL8 / +0 RAM** |

代价是：`view_i != 0` 时边缘支路不再输出二值边缘图，且这些新增口/视图**只在调试时有意义**；
量很小（+120 FF ≈ 全器件 3%），不值得做编译期开关。

改 BRAM 版之前，`u_pix_window_3x3` 是 20605 FF + 17377 LUT + 0 RAM —— 两行缓存用
异步读写成寄存器数组，推断不出 BRAM，占掉全器件约 1/4 的逻辑。**行缓存必须用
同步读**（读地址打在寄存器上），这是本工程踩过的坑，别再写成组合读。

## 5. 插入点接线（`example_top.v` 唯一改动处）

```verilog
// 伪代码，仅供示意；真实信号名以基线代码为准
localparam IMG_LAT = 19;             // = image_pipeline_top 的 LATENCY，两处必须一致
image_pipeline_top #(
    .IMG_W (1280),
    .IMG_H (720),
    .LATENCY (IMG_LAT)
) u_image_pipeline (
    .clk_i  (clk_pixel), .rst_ni (rst_n),
    .vsync_i(vs_de_from_awb), .de_i(de_from_awb), .rgb_i(rgb_from_awb),
    .thr_i  (r_img_thr),      .mode_i(r_img_mode), .view_i(r_img_view),
    .smooth_mode_i(r_smooth),
    .vsync_o(vs_to_crop),     .de_o  (de_to_crop), .rgb_o(rgb_to_crop)
);
```

M5 起 `thr` / `mode` / `view` 三个口不是 localparam 而是**带复位值的寄存器**，复位值 =
`IMG_THR_DEF / IMG_MODE_DEF / IMG_VIEW_DEF`，运行时由 `src/common/uart_cmd.v`（串口命令口）
改写 —— 现场调阈值不用重新综合。2026-10-17 起 `'T'`/`'S'` 接回数据通路：
`uart_cmd.thr_o/thr_valid_o → canny_threshold_key_ctrl.uart_thr_i/uart_thr_valid_i`，
`smooth_o/smooth_valid_o → uart_smooth_i/uart_smooth_valid_i`。板上 **K1 循环平滑
（1→2→0）、K2 循环阈值（4..48，步长 1，48 后回 4）**；串口写优先于按键。
串口口与流水线同域（`clk_pixel`），详见 `INTERFACE.md` §7 与
`docs/UART_COMMAND_REFERENCE.md`。

- 加**旁路 MUX**：`mode_i = 0` 时输出必须与基线逐像素一致，用于回归对比。
- hsync 也要按同一 `IMG_LAT` 延迟后再送 `FrameBoundCrop`（`r_hs_dly` 用 `IMG_LAT`
  参数化），否则 `rgb2dvi` 的 HS 与 DE 错位、HDMI 画面横移。
- 该文件由集成负责人独占修改。

## 6. 已知陷阱（来自基线代码核实）

| 陷阱 | 说明 |
| --- | --- |
| `src/axi4_ctrl.v` 有两个版本 | 工程编译的是 `src/axi/axi4_ctrl.v`（15868 B，参数化）。根下 `src/axi4_ctrl.v`（16483 B）未被引用，改它不会有任何效果 |
| `src/` 里有文件不在工程里 | 2026-10-17 复核：`src/` 共 43 个 `.v`，`Ti60_Demo.xml` 引用 38 个（另加根目录的 `example_top.v`）；未引用的 5 个是 `src/axi4_ctrl.v`（重复件）、`src/axi/axi4_ctrl_tb.v`、`src/dsi/dsi_init_tb.v`、`src/isp/VIP_RAW8_RGB888_TMPL.v`（后三个是 testbench/模板）和 `src/image/histogram_equalization.v`（2026-10-17 从 `debug_canny_hardware_diagnosis` 合入的备用模块，**尚未例化**，要用时先接进 `image_pipeline_top` 再挂 XML）。新加的 RTL 必须同时挂进 `Ti60_Demo.xml`，否则"改了 src 里文件但板上无变化"（`canny_edge.v`、`uart_cmd.v`、`median_filter_3x3.v` 等都已挂上） |
| `.pt.sdc` 是生成的 | 文件头写明 `# Efinity Interface Designer SDC`，点一次 Generate 就覆盖 |
| `src/isp/` 两个模块是活链路 | 见 `README.md` 铁律第 1 条 |
| **`canny_edge` 的 thr 口可用量程只有 0..96** | 判据域 `mag8 = mag>>3` 是按 2040 折算 0..255 的，但 3×3 高斯把 Sobel 的 4A 阶跃响应压成 3A，满对比度横竖边的 `mag` 只有 768。把 thr 当 "n/255" 会高估 2.7 倍（M5 事故根因：默认值 40 = 要求 42% 对比度）。标尺表见 `CANNY_PLAN.md` §3.8 与 `canny_edge.v` 头注释 |
| **`mag_o` 在 `mag8 > 32` 就饱和，不能用来调阈值** | `mag_o = min(mag,255)`，而 `mag8 > 32 ⟺ mag > 255`。thr=16 → mag=128 还看得见，thr=40 → mag=320 就是纯白一片。**量阈值要用 `view_i = 1`（`mag8` 图）**，它显示 `min(mag8<<1,255)` |
| **`uart_rx_i` / `uart_tx_o` 早已布线但从未被引用** | `example_top.v` 声明、`Ti60_Demo.peri.xml` 分配（`GPIOT_N_16`/`GPIOT_P_16`），全工程此前没有任何 RTL 用它。M5 的 `src/common/uart_cmd.v` 是第一个用户；板上要接 USB-UART 桥（TTL 侧） |
| **`efx_*` 必须先 `call setup.bat`** | 缺 `EFINITY_HOME`/`PATH` 时 `efx_map` 在打印 banner 之后直接 `EXCEPTION_ACCESS_VIOLATION reading memory at (nil)` 段错误 —— **看着像 RTL/工程有问题，其实是环境问题**（试过 `--help`、干净环境、cmd 启动都不是原因）。正确姿势：`cmd /c "call C:\Efinity\2026.1\bin\setup.bat && <efx 命令>"`。重跑综合的完整命令行见 §4 |
| **`start.cmd` 已过期** | 它写死 `c:\efinity\2022.2\bin\efx_run.bat`（本机装的是 2026.1），并且**开头就 `rm -rf outflow work_pnr work_pt work_syn`** —— 直接跑会删掉现有的综合/布局布线产物。要用它先把版本号改对，并想清楚要不要删 |
