# 赛题四 · 基于 FPGA 的实时图像边缘检测系统

易灵思 Titanium Ti60F225 + SC1336 摄像头 + HDMI 1280×720 实时边缘检测。

目标平台为既有基线工程 `Ti60F225_SC1336_HDMI_Display`（采集 / DDR 帧缓存 / Bayer 解马赛克 / HDMI 输出全部现成），
本仓库在其基础上**插入一段图像算法流水线**，并把它组织成可多人并行开发的形态。

---

## 1. 仓库布局

```
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
├── sim/                                ← 仿真：TB、激励向量、运行脚本
└── tools/                              ← Python 参考模型与比对工具
```

`work_syn/` `work_pnr/` `work_pt/` `ooc/` `outflow/` `ip/*/ipm/` `ip/*/Testbench/` `ip/*/*_devkit/`
都是工具构建产物，已被 `.gitignore` 排除，本地存在但不入库。

## 2. 快速上手

### 只想跑仿真（算法组 / 验证组，**不需要装 Efinity**）

```bash
# 在仓库根执行；需要 iverilog（免费）或 ModelSim
bash sim/scripts/run_iverilog.sh src/image/pix_window_3x3.v sim/tb/tb_pix_window_3x3.v
```

### 要综合上板（集成负责人）

1. 双击 `Ti60F225_SC1336_HDMI_Display/start.cmd` 打开 Efinity 工程
2. 新写的 `.v` 必须登记进 `Ti60_Demo.xml` 的 `<efx:design_file>` 列表，否则综合器看不到它
3. Synthesize → Place & Route → Program

> ⚠️ **Efinity 不像 Vivado 会扫描目录**。`src/` 下的 `.v` 不一定被编译——
> "我改了这个文件但板子没变化"，最常见的原因就是这个文件不在 `Ti60_Demo.xml` 里。

## 3. 分工速览

| 交付单元 | 文件 | 负责人 |
| --- | --- | --- |
| U1 3×3 像素窗口发生器 | `src/image/pix_window_3x3.v` | 算法 A |
| U2 Sobel 边缘检测 | `src/image/sobel_edge.v` | 算法 B |
| U3 流水线顶层与系统集成 | `src/image/image_pipeline_top.v` + `example_top.v` + `Ti60_Demo.xml` | 集成负责人 |
| U4 显示合成 | `src/display/image_mix.v` | 显示 |
| 验证与工具（横向） | `sim/**` + `tools/**` | 验证 |

详细的任务、里程碑与协作规范见仓库外的两份计划文档：
`赛题四_边缘检测_任务拆分与协作开发计划.md`、`赛题四_模块划分与文件归属.md`。

## 4. 铁律（每条都是踩过坑换来的）

1. **`src/isp/Line_Shift_RAM_8Bit.v` 和 `src/isp/VIP_Matrix_Generate_3X3_8Bit.v` 禁止修改。**
   已从综合网表确认它们是当前摄像头画面能显示的活链路（`bayer2rgb/u_VIP_Matrix_Generate_3X3_8Bit/u0_|u1_Line_Shift_RAM_8Bit`）。
   为 Sobel 去改公共行缓存，会让 Bayer 解马赛克挂掉、画面花屏且极难定位。需要行缓存就**复制一份到 `src/image/` 再改造**。
2. **`Ti60_Demo.peri.xml` 和 `Ti60_Demo.pt.sdc` 禁止手改**，一点 Interface Designer 的 Generate 就被覆盖。
3. **`example_top.v` 与 `Ti60_Demo.xml` 是集成负责人独占**，其他人不要动。
4. **算法流水线全部放在 `clk_pixel` 单时钟域内**，不得引入新的跨时钟域路径。
5. **`src/axi/axi4_ctrl.v` 才是被编译的那一份**；`src/axi4_ctrl.v`（同名模块的旧版）不在工程中，改它不会有任何效果。
