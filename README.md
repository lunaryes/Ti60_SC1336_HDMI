# 赛题四 · 基于 FPGA 的实时图像边缘检测系统

易灵思 Titanium Ti60F225 + SC1336 摄像头 + HDMI 1280×720 实时边缘检测。

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
