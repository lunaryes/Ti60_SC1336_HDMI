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
rgb_i[23:0] ──► rgb_to_gray ──► pix_window_3x3 ──┬──► sobel_edge ──► mag/edge
                                                 │                        │
                                                 └──► (可选) 中值滤波 ────┘
                                                                          ▼
                                                              mode 选择 / 着色 / 旁路 MUX
                                                                          │
                                                                    image_mix
                                                                          ▼
                                                             rgb_o[23:0] ──► FrameBoundCrop
```

`pix_window_3x3` 只例化一次，Sobel 与中值**共享同一组 9 个像素**（并行消费），
所以中值滤波器不需要额外 BRAM。

## 3. 时钟域

全部在 `clk_pixel`（74.25 MHz）单域。详见 `INTERFACE.md` §2。

## 4. 资源预算（初步）

| 资源 | 预估占用 | 说明 |
| --- | --- | --- |
| 行缓存 BRAM | 3 × 1280 × 8 bit ≈ 30 kbit | `pix_window_3x3` 的 3 个行 bank，按行轮转、1 写 2 读 |
| 乘法器 / DSP | 3（灰度化）+ 4~8（Sobel） | 灰度系数是常数，可用移位加法代替乘法器 |
| 逻辑 LUT | 数百~千级 | Sobel 4 级流水 + 合成 |

目标总占用不超过 Ti60F225 的 70%，由集成阶段持续跟踪。

**实测（2026-10-10，Efinity 2026.1 `efx_map`，工程含全部外设）**：

| 模块 | FF | LUT | RAM |
| --- | --- | --- | --- |
| `u_pix_window_3x3` | 128 | 149 | 6 × EFX_RAM10 |
| `u_image_pipeline`（整条流水线） | 294 | 243 | 6 × EFX_RAM10 |

改 BRAM 版之前，`u_pix_window_3x3` 是 20605 FF + 17377 LUT + 0 RAM —— 两行缓存用
异步读写成寄存器数组，推断不出 BRAM，占掉全器件约 1/4 的逻辑。**行缓存必须用
同步读**（读地址打在寄存器上），这是本工程踩过的坑，别再写成组合读。

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
