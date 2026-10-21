# 赛题四 · 基于 FPGA 的实时图像边缘检测系统

易灵思 Titanium Ti60F225 + SC1336 摄像头 + HDMI 1280×720 实时边缘检测。

本版使用全屏 CCL 字符分割、常规/细线双采样、中文标注与位置跟踪，保留图形 CCL/Moore 检测。
支持数字 0～9、大写 A～Z，最多同时显示 4 个字符；O/0 等歧义允许拒识，不是整页通用 OCR。
新版使用范围见 [字符识别重写说明](docs/OCR_REWRITE_20261021.md)，
实测范围及硬件时序见 [重写验证记录](docs/OCR_REWRITE_VALIDATION_20261021.md)。旧版资料保留在 [OCR V3说明](docs/OCR_V3_CAMERA.md)，
Canny基线/像素保真验证见 [回归记录](docs/CANNY_BASELINE_AUDIT.md)。
UART `4F 00`/`4F 01` 可立即关闭/开启识别标注，便于对比同一画面的Canny边缘。

---

## 1. 仓库布局

```text
07-1_Ti60_SC1336_HDMI_1280720/          ← 仓库根（git 从这一层开始跟踪）
├── Ti60F225_SC1336_HDMI_Display/       ← Efinity 工程（基线，尽量只读）
│   ├── example_top.v                   ← 唯一顶层模块【集成负责人独占】
│   ├── Ti60_Demo.xml                   ← 工程定义 / 源文件列表【集成负责人独占】
│   ├── Ti60_Demo.peri.xml              ← 引脚与外设配置（Interface Designer 生成，禁手改）
│   ├── Ti60_Demo.pt.sdc                ← 时钟与跨域约束（Interface Designer 生成，禁手改）
│   ├── start.cmd
│   ├── ip/                             ← 11 个 IP 实例（只读，勿手改）
│   └── src/
│       ├── axi/ cmos_i2c/ dsi/ hdmi_ip/ isp/ lvds/    ← 基线代码，只读
│       ├── common/                     ← 【新增】通用模块
│       ├── image/                      ← 【新增】图像算法模块
│       └── display/                    ← 【新增】显示合成模块
├── docs/                               ← 架构与接口文档
└── sim/                                ← Verilator 回归（Verilator + Python 参考模型）
    ├── run.sh                          ← 唯一入口：bash sim/run.sh [图样名...]
    ├── canny_ref.py                    ← 参考模型 + 图样 + 逐像素比对
    ├── color_probe.py                  ← 离线探针：AWB→灰度→梯度→mag8 标尺与色卡判决
    ├── gray_hist.py                    ← 离线：原始灰度直方图 / mag8 余量 / 灰度增强收益评估
    ├── read_hist.py                    ← 在线：发 'H' 读片内 256 bin 直方图（无采集卡路径）
    ├── count_map.py                    ← 从综合网表统计各子系统资源（`docs/ARCH.md` §4 的数据来源）
    ├── tb_canny_edge.v                 ← canny_edge 算法回归
    ├── tb_pipeline_top.v               ← 顶层集成回归（延迟/vsync/旁路/四种视图/分屏左灰度）
    ├── tb_rgb_to_gray.v                ← 灰度链颜色路径回归
    ├── tb_uart_cmd.v                   ← 串口命令口回归（真实参数 74.25MHz/460800）
    ├── tb_gray_hist_uart.v             ← 片内直方图回归（帧边界/累加/778 字节流/校验和）
    ├── tb_median_filter_3x3.v          ← 中值滤波单元回归（两种模式/1 拍延迟/边保持）
    └── mut_uart.py                     ← 串口台的变异测试（手动跑，确认它真的会 FAIL）
```

`work_syn/` `work_pnr/` `work_pt/` `ooc/` `outflow/` `ip/*/ipm/` `ip/*/Testbench/` `ip/*/*_devkit/`
都是工具构建产物，已被 `.gitignore` 排除，本地存在但不入库。

## tips

1. **`src/isp/Line_Shift_RAM_8Bit.v` 和 `src/isp/VIP_Matrix_Generate_3X3_8Bit.v` 禁止修改。**
   已从综合网表确认它们是当前摄像头画面能显示的活链路（`bayer2rgb/u_VIP_Matrix_Generate_3X3_8Bit/u0_|u1_Line_Shift_RAM_8Bit`）。
   为 Sobel 去改公共行缓存，会让 Bayer 解马赛克挂掉、画面花屏且极难定位。需要行缓存就**复制一份到 `src/image/` 再改造**。
2. **`Ti60_Demo.peri.xml` 和 `Ti60_Demo.pt.sdc` 禁止手改**，一点 Interface Designer 的 Generate 就被覆盖。
3. **`example_top.v` 与 `Ti60_Demo.xml` 是集成负责人独占**，其他人不要动。
4. **算法流水线全部放在 `clk_pixel` 单时钟域内**，不得引入新的跨时钟域路径。
5. **`src/axi/axi4_ctrl.v` 才是被编译的那一份**；`src/axi4_ctrl.v`（同名模块的旧版）不在工程中，改它不会有任何效果。
6. **改了 RTL 就跑回归**：`bash sim/run.sh`（Verilator + Python 参考模型，七段：
   13 个图样算法比对（另抽测平滑模式 1/2）/ 13 个图样顶层集成 / 灰度链颜色路径 / 串口命令口 /
   按键/平滑控制器 / 片内直方图 / 中值滤波单元，64×32 约 2 分钟）。它在 WSL 里跑，
   需要 `verilator >= 5.0` 和 `python3`。
   只跑某几个图样：`bash sim/run.sh lines noise`；跑真实分辨率：`IMG_W=1280 IMG_H=720 bash sim/run.sh`
   （约 10 分钟，Python 参考模型是瓶颈）。
   **不要同时开两个 run.sh**：`sim/build/` 是共享工作目录，会互相覆盖激励，症状是
   `$readmem file address beyond bounds` 之类完全指错方向的报错。脚本用锁目录会直接拒绝并发。
7. **Canny 的阈值口刻度不是 0..255，有效量程只有 0..96**（横竖边）。默认 12（要求 ΔY ≥ 32 ≈ 满幅 13%；
   2026-10-15 上板扫出来的工作点，之前是 16），
   **不要超过 48**。调参、排查"边缘出不来"之前先看 `docs/CANNY_PLAN.md` §3.8 的标尺表，
   或直接跑 `python3 sim/color_probe.py`。板上 **K1 循环平滑（标准高斯→弱平滑→旁路）、
   K2 循环阈值（范围 4..48，步长 1，48 后回 4）**，或用串口 `'T' <byte>` 直接写
   8 位任意值（优先于按键）。完整命令表见 `docs/UART_COMMAND_REFERENCE.md`。
   **串口是二进制协议，不是文本**：`view=3` 要发十六进制 `56 03`，发成 `V 3` 会被吃掉参数里的
   空格（`0x20`=32）而静默变成 `view = 0x20 & 3 = 0` —— 表现成"命令发出去了但没生效"。
   字节表见 `INTERFACE.md` §7，**每发一条会改画面的参数命令跟一个 `52` 回读核对**
   （回包 7 字节：`thr view mode gain_r gain_b smooth '\n'`；`thr`/`smooth` 是实际生效值，
   按键调过之后也准确）。
8. **AWB 增益也是运行时可调的**（`41 XX` = R 增益、`42 XX` = B 增益，`byte/64`）：现场"白墙发青"
   = 相机链路 R 偏低，对着白墙/白纸把 R 调到中性即可，不用改常数重新综合。
   复位默认 **112/96 = ×1.75/×1.5**，其中 ×1.75 是 2026-10-15 上板对白墙二分调出来的中性点
   （老的写死值是 ×1.59，偏青）。
9. **量原始灰度分布、判断要不要加灰度增强**：两种口径，按现场条件选一种。

   **A. 没有采集卡：片内直方图（2026-10-17 新增）**
   串口发单字节 `48`（`'H'`）。FPGA 统计 **3×3 中值之后、Canny 实际看到的那份 Y**
   的完整一帧，回传 778 字节（magic `A5 5A 48` + frame_id + frame_total + 256×20 bit
   bins + checksum）。主机侧跑：

   ```bash
   python sim/read_hist.py --port COM5
   ```

   Windows 走 Win32 API、Linux/WSL 走 termios，**不需要 pyserial**；`--json` 存结果，
   `--file` 离线重放已保存的流。脚本按与 `gray_hist.py` 相同的口径打印 256 级直方图、
   p1/p50/p99、削顶比例，并判读“偏暗 / 过曝 / 黑电平抬高 / 动态范围铺满”。
   发 `'H'` 后**不要再发别的命令**，等 778 字节收完（通常 < 50 ms）；这期间
   `uart_cmd` 的 RX 被直方图模块占住，命令会被丢弃。这条路径只给全局分布，
   **不给** mag8 过阈比例和空间分布。

   **B. 有采集卡：抓整帧图，跑离线分析**
   串口发 `4D 01` + `56 03`（mode=1, view=3）让 HDMI 输出 RTL 的 Y 图，
   用采集卡以 **1280×720、无损 PNG、Full range** 抓一帧；或发 `4D 00`（mode=0）
   抓 AWB 之后的原图。**不要用手机拍屏 / JPEG**（屏幕 gamma、摩尔纹、DCT 会改掉
   直方图和局部梯度）。然后：

   ```bash
   python sim/gray_hist.py <抓图.png> --roi ... --grid 3 --sim "gamma:0.7"
   ```

   除了直方图/分位数，它还能给 mag8 过阈比例、空间网格和增强方案仿真 ——
   这是唯一能回答“低对比度人脸轮廓能不能出边”的口径。

   两种口径的共同判读：p99 距 255 还有余量且 `>=254` 的像素 < 0.5% → 数字增强有空间；
   `>=254` 超过 0.5% → 先降 sensor 曝光/增益，任何 LUT 都救不回已削顶的高光。
   正常光 / 强光 / 盖镜头三组一起测，横向对比能分开“曝光不足”和“局部对比度不足”。
   增强落点：`rgb_to_gray` 之后、第一级 `pix_window_3x3` 之前做 256×8 LUT；同步读 BRAM 会 +1 拍，
   **LATENCY 19→20 要同步改 `IMG_LAT` 和 hsync 延迟**。根因仍是 SC1336 固定曝光、无 AE 环
   （`docs/RGB_TO_GRAY.md` §4.1）。
