# 赛题四 · 基于 FPGA 的实时图像边缘检测系统

易灵思 Titanium Ti60F225（C4）+ SC1336 摄像头 + HDMI 1280×720 实时边缘检测。

基线停在灰度转换（`rgb_to_gray`）之后。本仓库在其后接上 **3x3 窗口 → Sobel → 显示合成**
整条流水线，并配套 L1 / L2 两级可复现验证。

---

## 1. 快速开始

```sh
git clone https://github.com/lunaryes/Ti60_SC1336_HDMI.git
cd Ti60_SC1336_HDMI

sh sim/run.sh          # L1 模块仿真：期望末行 *** PASS ***，退出码 0
sh sim/l2_verify.sh    # L2 逐像素比对：期望 *** L2 PASS ***
```

工具链：Efinity 2026.1.132、Icarus Verilog 14.0、Python 3。
工程主文件：`Ti60F225_SC1336_HDMI_Display/Ti60_Demo.xml`（Efinity → File → Open Project）。

**动手前请先读 [CONTRIBUTING.md](CONTRIBUTING.md)** —— 里面写了三条铁律、验证流程和坑速查。

---

## 2. 仓库布局

```
Ti60_SC1336_HDMI/                       ← 仓库根（git 从这一层开始跟踪）
├── CONTRIBUTING.md                     ← 贡献指南【先读这个】
├── .github/
│   ├── CODEOWNERS                      ← 文件归属
│   ├── PULL_REQUEST_TEMPLATE.md
│   └── ISSUE_TEMPLATE/{task,bug}.yml
├── Ti60F225_SC1336_HDMI_Display/       ← Efinity 工程（基线，尽量只读）
│   ├── example_top.v                   ← 唯一顶层模块【集成负责人独占】
│   ├── Ti60_Demo.xml                   ← 工程定义 / 源文件列表【集成负责人独占】
│   ├── Ti60_Demo.peri.xml              ← 引脚与外设配置（Interface Designer 生成，禁手改）
│   ├── Ti60_Demo.pt.sdc                ← 时钟与跨域约束（Interface Designer 生成，禁手改）
│   ├── start.cmd
│   ├── ip/                             ← 11 个 IP 实例（只读，勿手改）
│   └── src/
│       ├── axi/ cmos_i2c/ dsi/ hdmi_ip/ isp/ lvds/   ← 基线代码，只读
│       ├── common/                     ← sync_delay_align.v（可参数化拍数的对齐）
│       ├── image/                      ← rgb_to_gray.v（基线）+ 3x3 窗口 / Sobel / 流水线顶层
│       └── display/                    ← 原图与边缘图合成（含 mode 3 画中画）
├── docs/                               ← 架构与接口文档
│   ├── INTERFACE.md                    ← 冻结接口契约（模块、端口、LATENCY、公式）
│   ├── ARCH.md                         ← 结构、资源、实现陷阱、综合结果
│   ├── L2_VERIFY.md                    ← L2 验证报告
│   └── RGB_TO_GRAY.md                  ← 基线灰度转换说明
├── rfc/                                ← 设计决策记录（为什么这么定、否掉了哪些方案）
├── sim/                                ← testbench 与一键复现脚本
│   ├── run.sh                          ← L1 一键复跑
│   ├── l2_verify.sh                    ← L2 一键复跑（8 组 MODE×THR）
│   ├── tb_image_pipeline.v             ← 逐像素对照自检
│   ├── tb_window_boundary.v            ← 边界行为基线
│   ├── tb_l2_dump.v                    ← L2 像素流转储 TB
│   └── l2_report.txt                   ← L2 报告（验证证据，入库）
└── tools/
    └── l2_ref_model.py                 ← 独立 Python 参考模型（不读 RTL 代码）
```

`work_syn/` `work_pnr/` `work_pt/` `ooc/` `outflow/` `ip/*/ipm/` `ip/*/Testbench/` `ip/*/*_devkit/`
都是工具构建产物，已被 `.gitignore` 排除，本地存在但不入库。

---

## 3. 文档索引

| 你想知道 | 看 |
| --- | --- |
| 现在的接口长什么样（端口、`LATENCY`、公式） | [`docs/INTERFACE.md`](docs/INTERFACE.md) |
| 结构、资源占用、踩过哪些坑 | [`docs/ARCH.md`](docs/ARCH.md) |
| 验证到什么程度、还没覆盖什么 | [`docs/L2_VERIFY.md`](docs/L2_VERIFY.md) |
| **当时为什么这么定**、否掉了哪些方案 | [`rfc/`](rfc/README.md) |
| 怎么开发、怎么验证、怎么提 PR | [`CONTRIBUTING.md`](CONTRIBUTING.md) |

---

## 4. tips

1. **`src/isp/Line_Shift_RAM_8Bit.v` 和 `src/isp/VIP_Matrix_Generate_3X3_8Bit.v` 禁止修改。**
   已从综合网表确认它们是当前摄像头画面能显示的活链路（`bayer2rgb/u_VIP_Matrix_Generate_3X3_8Bit/u0_|u1_Line_Shift_RAM_8Bit`）。
   为 Sobel 去改公共行缓存，会让 Bayer 解马赛克挂掉、画面花屏且极难定位。需要行缓存就**复制一份到 `src/image/` 再改造**。
2. **`Ti60_Demo.peri.xml` 和 `Ti60_Demo.pt.sdc` 禁止手改**，一点 Interface Designer 的 Generate 就被覆盖。
3. **`example_top.v` 与 `Ti60_Demo.xml` 是集成负责人独占**，其他人不要动。
4. **算法流水线全部放在 `clk_pixel` 单时钟域内**，不得引入新的跨时钟域路径。
5. **`src/axi/axi4_ctrl.v` 才是被编译的那一份**；`src/axi4_ctrl.v`（同名模块的旧版）不在工程中，改它不会有任何效果。
6. **新增 `.v` 文件必须登记进 `Ti60_Demo.xml`**，否则板上不会有任何变化；
   而 `efx_run` 每次都会规范化重写该文件，提交前记得 `git checkout --` 恢复。

---

## 5. 贡献

先读 [`CONTRIBUTING.md`](CONTRIBUTING.md)。要点：

- 不要直接往 `main` 推，走 `feature-*` 分支 + PR
- 提交信息用 Conventional Commits，**正文写"为什么"**，一次提交只做一件事
- L1（`sh sim/run.sh`）+ L2（`sh sim/l2_verify.sh`）跑过再提 PR，并附报告路径
- 改冻结接口或影响多人的决策，先在 [`rfc/`](rfc/README.md) 落一篇 RFC

提任务用 [`task.yml`](.github/ISSUE_TEMPLATE/task.yml)，报缺陷用 [`bug.yml`](.github/ISSUE_TEMPLATE/bug.yml)。
