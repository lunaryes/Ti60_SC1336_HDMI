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
rgb_i[23:0] ──► pix_window_3x3 ──┬──► sobel_edge ──► mag/edge
                                 │                        │
                                 └──► (可选) 中值滤波 ────┘
                                                          ▼
                                              mode 选择 / 着色 / 旁路 MUX
                                                          │
                                              edge_display_mix
                                                          ▼
                                                    rgb_o[23:0] ──► FrameBoundCrop
```

`pix_window_3x3` 只例化一次，Sobel 与中值**共享同一组 9 个像素**（并行消费），
所以中值滤波器不需要额外 BRAM。

## 3. 时钟域

全部在 `clk_pixel`（74.4 MHz）单域。详见 `INTERFACE.md` §2。

## 4. 资源预算（初步）

| 资源 | 预估占用 | 说明 |
| --- | --- | --- |
| 行缓存 BRAM | 2 × 1280 × 8 bit ≈ 20 kbit | `pix_window_3x3` 的两行历史数据 |
| 乘法器 / DSP | 3（灰度化）+ 4~8（Sobel） | 灰度系数是常数，可用移位加法代替乘法器 |
| 逻辑 LUT | 数百~千级 | Sobel 4 级流水 + 合成 |

目标总占用不超过 Ti60F225 的 70%，由集成阶段持续跟踪。

## 5. 插入点接线（`example_top.v` 唯一改动处）

```verilog
// 伪代码，仅供示意；真实信号名以基线代码为准
image_pipeline_top #(
    .IMG_W (1280),
    .IMG_H (720)
) u_image_pipeline (
    .clk_i  (clk_pixel), .rst_ni (rst_n),
    .vsync_i(vs_de_from_awb), .de_i(de_from_awb), .rgb_i(rgb_from_awb),
    .thr_i  (thr),            .mode_i(mode),
    .vsync_o(vs_to_crop),     .de_o  (de_to_crop), .rgb_o(rgb_to_crop)
);
```

- 加**旁路 MUX**：`mode_i = 0` 时输出必须与基线逐像素一致，用于回归对比。
- 该文件由集成负责人独占修改。

## 6. 已知陷阱（来自基线代码核实）

| 陷阱 | 说明 |
| --- | --- |
| `src/axi4_ctrl.v` 有两个版本 | 工程编译的是 `src/axi/axi4_ctrl.v`（15868 B，参数化）。根下 `src/axi4_ctrl.v`（16483 B）未被引用，改它不会有任何效果 |
| `src/` 有 6 个文件不在工程里 | `src/` 共 33 个文件，`Ti60_Demo.xml` 只引用 27 个。"改了 src 里文件但板上无变化"通常就是这个原因 |
| `.pt.sdc` 是生成的 | 文件头写明 `# Efinity Interface Designer SDC`，点一次 Generate 就覆盖 |
| `src/isp/` 两个模块是活链路 | 见 `README.md` 铁律第 1 条 |

---

## 7. 实现状态（本轮）

| 模块 | 状态 |
| --- | --- |
| `src/image/rgb_to_gray.v` | 上一轮已完成，保持不动 |
| `src/image/pix_window_3x3.v` | 本轮实现（LATENCY = 2 拍） |
| `src/image/sobel_edge.v` | 本轮实现（LATENCY = 4 拍） |
| `src/display/edge_display_mix.v` | 本轮实现（mode 0/1/2；mode 3 待实现）。**2026-10-10 由 `image_mix` 改名**，腾出 `src/display/image_mix.v` 给所有者分支 |
| `src/common/sync_delay_align.v` | 本轮实现 |
| `src/image/image_pipeline_top.v` | 本轮实现（对外 LATENCY = 7 拍） |
| `src/image/image_line_shift_ram_8bit.v` | `Line_Shift_RAM_8Bit` 参考副本，**未采用、不入工程** |
| `sim/tb_image_pipeline.v` | 自检式 testbench，待有仿真器时运行 |

`example_top.v` 的唯一改动：在 `rgb_to_gray` 与 `FrameBoundCrop` 之间插入
`image_pipeline_top`，并把 `hsync` 用 `sync_delay_align #(.DEPTH(7))` 延迟 7 拍。
`Ti60_Demo.xml` 已登记 5 个新增源文件。

## 8. 实现阶段新发现的陷阱

| 陷阱 | 说明 |
| --- | --- |
| **握手信号延迟必须等于数据延迟** | 若模块让 `de_o = de_i`（0 延迟）而数据多延迟 N 拍，输出画面会整幅横移 N 列并丢掉最右 N 列。各级统一用与数据同深度的移位寄存器延迟握手信号 |
| **`hsync` 必须单独补延迟** | `image_pipeline_top` 端口里没有 `hs`，若不管它，HS 与 DE 会错开 7 拍，图像横移 7 个像素。必须在 `example_top` 里补 `sync_delay_align` |
| **旁路不能直接接 MUX** | 旁路路径取 `pix_window_3x3` 的输出 `p22`（与 Sobel 的窗口中心同源同拍，省一块行缓存），再补 `sobel_edge` 的 4 拍 |
| **固有 1 行垂直延迟** | 3x3 窗口需要"下一行"数据，末行永远算不出边缘。因此 `mode=0` 的输出相对基线**整体下移 1 行**，不是逐像素一致。回归对比要把这 1 行算进去 |
| **`mode_i` 被硬编码会触发死代码消除** | `example_top` 里 `assign edge_mode = 2'd1` 是常量，综合器把 `edge_display_mix`（原名 `image_mix`）的 `mode 2` 分支（含列计数器）整个裁掉了，报 `col[*] unreachable`。要在板上切模式，`mode_i` 必须由真实寄存器/输入驱动（见 §10） |
| **行缓存不要照抄 `Line_Shift_RAM_8Bit`** | 它实现的是"延迟 1 行 + 1 像素"，相邻行有固定 1 像素错位，必须特判去补；且存储深度写成 `2**ADDR_WIDTH`，1280 点行占 2048 深。改用 3 个 bank 按行轮转 + 预测式读地址（读地址提前一拍发 `col+1`），三行零错位 |

## 9. 综合结果（Efinity 2026.1.132，Ti60F225 C4）

工具链：`E:\qianrushi\efinity-2026.1.132-windows-x64\efinity\2026.1\bin\efx_run.bat`
（与 `Ti60_Demo.xml` 的 `sw_version="2026.1.132"` 匹配；仓库原 `start.cmd` 里
的 `c:\efinity\2022.2` 是过期路径，已修正）

下表为**修掉 §11 两个 RTL bug 之后**重新综合的结果（2026-10-09 19:39，Ti60F225 C4）。

| 项目 | 结果 |
| --- | --- |
| map / interface / pnr / pgm | **全 PASS**，bitstream 已生成 |
| 时序 | **0 个负 slack**。最差 slack **0.026 ns** 出现在 `ddr3_ctl_axi` 的 `tac_clk` 域（DDR3 控制器），**与算法流水线无关**；算法所在的 `clk_pixel` 域 setup slack = **8.341 ns**，余量充裕 |
| 跨时钟域 | 无新增 CDC（算法全在 `clk_pixel` 单域，符合铁律 4） |
| XLR（逻辑） | 6916 / 60800 = **11.38%**（修复 `bank_prev2` 后 +61，因为该 mux 不再被优化掉） |
| Memory Blocks | 56 / 256 = **21.88%**；其中算法流水线只占 **6 块** |
| DSP Blocks | 4 / 160 = 2.50%；**Sobel 用 0 个 DSP**（系数 2 用移位实现） |

结论：资源远低于 70% 预算目标，`mode 3`（画中画，约需 1.8 Mbit 缓冲 ≈ 额外几十块 BRAM）
在预算内是可以做的。

## 10. 待决问题

1. **`mode_i` / `thr_i` 怎么接**：`example_top` 当前把 `mode` 硬编码为 1、`thr` 硬编码为 40。
   板上没有 DIP 开关，`example_top` 端口里也没有用户按键（`nrst` 是注释掉的），
   只有 `led_o[5:0]` 和 `uart_rx_i/uart_tx_o` 可用。
   要上板切换，可选：
   - 走 UART 命令（需要写一个接收模块，零风险，不动 `.peri.xml`）
   - 用计数器定时自动轮换 mode/thr 做演示（最简单）
   - 加真实按键 → 必须由集成负责人在 Interface Designer 里重新生成
     `.peri.xml` / `.pt.sdc`（禁止手改）
2. **`mode 3`（画中画）** 本轮未实现，资源预算允许，需要 640×360 缓冲与读写仲裁。
3. **回归对比**：`mode=0` 与基线存在固有 1 行垂直偏移，对比脚本要按此对齐。
4. **仿真**：已用本机 D 盘的 Icarus Verilog 14.0 跑通，全部 288 次检查通过（详见 §11）。
   一键复跑：`sh sim/run.sh`。
5. ~~边界策略要不要补实现~~ —— **已决（2026-10-09）：不补实现，改契约**。
   `docs/INTERFACE.md` §3 原"复制边缘像素"条款**作废**，改写为"本模块不做边界处理，
   由 `FrameBoundCrop` 裁剪切边兜底"。逐点实测证据见 §11.4。

## 11. 功能仿真验证（Icarus Verilog 14.0）

仓库原本没有仿真器。本轮发现**本机 D 盘其实已经装了 Icarus Verilog 14.0**
（`D:\下载迁移文件夹\iverilog\bin`，含 `iverilog.exe` + `vvp.exe`），只是没加入
`PATH`，所以此前 `which iverilog` 查不到。用它把 `sim/tb_image_pipeline.v`
真正跑了起来，并据此修掉两个 RTL bug。

一键复跑（在 repo/ 根目录）：

```sh
sh sim/run.sh
# 或指定自己的仿真器路径
IVB=/path/to/iverilog/bin sh sim/run.sh
```

### 11.1 testbench 自身的两个坑（不先修就没法信仿真结论）

| 坑 | 症状 | 修法 |
| --- | --- | --- |
| **延迟基准用错** | 窗口自检拿 `de_o`（整链路 7 拍延迟）当基准，而窗口数据只延迟 2 拍，期望值整体错位 5 列，首轮报 81 处不符 | 给窗口链路单独用 `w_de` 计 `wrow/wcol`，与 `orow/ocol` 分开 |
| **图案掩盖行错位** | 原四象限图案里 y<4 的四行完全相同、y>=4 的四行也完全相同，"上上行被写成上一行"这种行错位完全看不出来 | 图案改为**每行递增 4**（相邻行必不同），检查从"九值全相等"改为**逐像素对照 `src_pix_clamp`** |

### 11.2 仿真抓出的 RTL bug

| # | 位置 | 问题 | 后果 |
| --- | --- | --- | --- |
| 1 | `pix_window_3x3.v` `bank_prev2` | 原式 `(row_bank==1)?0:(row_bank==2)?1:2` 实际等于 `(row_bank-1)%3`，与 `bank_prev1` **恒等** | "上上行"永远读到"上一行"，Sobel 的 `Gy` 分量全程算错 |
| 2 | `pix_window_3x3.v` `rd_addr` | 行尾 `rd_addr = (col==COL_LAST) ? 0 : col+1` 把"右邻"回绕到**第 0 列** | 最后一列的窗口右邻取错（实测输出第 15 列拿到第 0 列的值）。**决定不修**，见下 |

修正：`bank_prev2 = (row_bank+1) % 3` —— **已落地**（仿真与综合均通过）。

#2 的 `rd_addr` **决定保留原样，不修**：该地址同时承担"使下一行第 0 列对齐"的职责，
把它冻结成 `COL_LAST` 会破坏整幅图的列对齐。末列右邻取错被归入"不做边界处理"，
由 `FrameBoundCrop` 兜底（见 §11.4）。

**变异测试（证明测试有效）**：把 `bank_prev2` 改回错误版本重跑，仿真报 **154 处不符**，
且报错内容正是"上上行的期望 8 被写成了 12"——说明新图案确实具备检出能力，
PASS 不是因为测试太弱。

### 11.3 结果

```
窗口自检次数  : 144
整链路检查次数: 144
结果: *** PASS *** 共 288 次检查全部通过
```

覆盖范围：3x3 窗口逐像素对齐（含行/列错位）、Sobel 幅值与阈值逐像素重算、
旁路逐像素一致（`mode=0`）、`mode=1` 二值边缘图。

### 11.4 边界行为 —— 已决：不做处理，由裁剪兜底

**结论**：`pix_window_3x3` **不做任何边界特殊处理**；最外圈像素的窗口取到越界值，
由下游 `FrameBoundCrop`（`SKIP_ROWS=2` / `SKIP_COLS=2`）裁剪切边兜底。
`docs/INTERFACE.md` §3 原"复制边缘像素、不补零"条款据此**作废**（2026-10-09 决定）。

**逐点实测**（`sim/tb_window_boundary.v`；16×8、两帧，图案 `pix = row*16 + col`，
每个像素值唯一。表中数值即 `row*16+col`）：

| 位置 | 旧契约期望 | 实测 | 症状 |
| --- | --- | --- | --- |
| 首行 `wrow=0 wcol=0` | `0 0 1 / 0 0 1 / 0 0 1` | `113 113 113 / 81 81 81 / 0 0 1` | 前两行是**上一帧残留**（帧1 行7=113、行5=81） |
| 首行 `wrow=0 wcol=2` | `1 2 3 / 1 2 3 / 1 2 3` | `113 114 115 / 81 82 83 / 1 2 3` | 列对、行错 |
| 中行 `wrow=4 wcol=0` | `32 32 33 / 48 48 49 / 64 64 65` | `33 33 33 / 49 49 49 / 64 64 65` | 左列＋中心列取到**列1**（应列0） |
| 中行 `wrow=4 wcol=1` | `32 33 34 / 48 49 50 / 64 65 66` | `33 33 34 / 49 49 50 / 64 65 66` | 左列取错 |
| 末列 `wrow=4 wcol=15` | `46 47 47 / 62 63 63 / 78 79 79` | `32 32 32 / 48 48 48 / 0 0 0` | `center_bad` 生效→**整窗被替换**，地址回绕到第 0 列 |
| 末行 `wrow=7 wcol=0` | `80 80 81 / 96 96 97 / 112 112 113` | `81 81 81 / 97 97 97 / 112 112 113` | 左列取错 |

**6 / 6 全部与旧契约不符。**

**根因（判据错位 2 列）**：`center_bad = (col==0)`、`left_bad = (col<=1)` 用的是
**当拍输入列号**，而窗口输出比输入**滞后 2 拍**——实测 `wcol=0` 时模块内 `col=2`；
`wcol=15` 时 `col=0`（行尾归零）。判据因此**整体错位 2 列**，实际只作用在
**输出行的最后两列**（`wcol=14/15`），而**首两列 `wcol=0/1` 完全不被保护**，
方向正好搞反。另外代码中**没有任何行方向判据**，"首行 / 末行复制"这一半从未存在。

**为什么此前没暴露**：三处偏差全落在 `FrameBoundCrop` 裁掉的 2 行 2 列内，板上不可见；
`tb_image_pipeline.v` 的 288 次检查也只覆盖 `col ∈ [2, IMG_W-3]`、`row ∈ [2, IMG_H-3]`。
**所以 288 次 PASS 不代表边界契约成立。**

**若将来要真做**：判据须改用**输出侧列号**并新增行方向判据，
而不是简单给 `col` 加偏移——`col` 在行尾会归零，偏移跨行时不成立。

### 11.5 如何在别处使用 iverilog

Icarus Verilog 官方 Windows 安装包（含 GTKWave 波形查看器）：
<https://bleyer.org/icarus/> —— 当前最新为 `iverilog-v14-20260804-x64_setup.exe`。
本机已装，无需重装；要让命令在任意目录可用：

```sh
export PATH="/d/下载迁移文件夹/iverilog/bin:$PATH"     # Git Bash
# 之后：
iverilog -g2005 -o sim/tb.vvp <源文件列表> && vvp sim/tb.vvp
```

