# 贡献指南（CONTRIBUTING.md）

**赛题四 · 基于 FPGA 的实时图像边缘检测系统**
易灵思 Titanium Ti60F225（C4）+ SC1336 摄像头 + HDMI 1280×720

本文只写**这个仓库特有的规矩**。通用 Git / Verilog 常识不重复，请自行掌握。

---

## 0. 三分钟上手

```sh
git clone https://github.com/lunaryes/Ti60_SC1336_HDMI.git
cd Ti60_SC1336_HDMI
git checkout -b feature-<你的主题>

sh sim/run.sh          # L1 自检，期望末行 "*** PASS ***"，退出码 0
sh sim/l2_verify.sh    # L2 与 Python 参考模型逐像素比对，期望 "*** L2 PASS ***"
```

工具链与版本（务必对齐，版本不一致 Efinity 会弹升级提示）：

| 工具 | 版本 | 位置 / 覆盖方式 |
| --- | --- | --- |
| Efinity | 2026.1.132 | `E:\qianrushi\efinity-2026.1.132-windows-x64\efinity\2026.1\bin\efx_run.bat` |
| Icarus Verilog | 14.0 | 默认 `D:\下载迁移文件夹\iverilog\bin`，用 `IVB=<路径>` 覆盖 |
| Python | 3.x | `sh sim/l2_verify.sh` 自动查找，用 `PYTHON=<路径>` 覆盖 |

工程主文件是 `Ti60F225_SC1336_HDMI_Display/Ti60_Demo.xml`（Efinity → File → Open Project）。

---

## 1. 三条铁律

### 1.1 这些文件禁止修改

| 文件 | 原因 |
| --- | --- |
| `src/isp/Line_Shift_RAM_8Bit.v` | 已从综合网表确认它是摄像头画面能显示的**活链路**（`bayer2rgb/u_VIP_Matrix_Generate_3X3_8Bit/u0_\|u1_Line_Shift_RAM_8Bit`）。为 Sobel 改公共行缓存会让 Bayer 解马赛克挂掉，**画面花屏且极难定位** |
| `src/isp/VIP_Matrix_Generate_3X3_8Bit.v` | 同上 |
| `Ti60_Demo.peri.xml` | Interface Designer 生成物，一点 Generate 就被覆盖。改它等于白改 |
| `Ti60_Demo.pt.sdc` | 同上（文件头自带 `# Efinity Interface Designer SDC`） |
| `ip/` 下的一切 | IP 实例，只读 |

**需要行缓存就复制一份到 `src/image/` 再改造** —— 我们自己的 `image_line_shift_ram_8bit.v` 就是这么来的。

### 1.2 新增 `.v` 文件必须登记进工程

只把文件放进 `src/` **完全没有用**，板上不会有任何变化。必须手工把 `<efx:design_file name="src/.../xxx.v" .../>` 加进
`Ti60F225_SC1336_HDMI_Display/Ti60_Demo.xml`。

> ⚠️ 每次跑 `efx_run` 或 GUI 点 Generate，Efinity 都会**规范化重写** `Ti60_Demo.xml`（属性顺序、自闭合标签全变）。
> 提交前用 `git checkout -- Ti60F225_SC1336_HDMI_Display/Ti60_Demo.xml` 恢复成手工维护的干净版本，只保留你要加的那一行。

### 1.3 算法全部放在 `clk_pixel` 单时钟域内

不得引入新的跨时钟域路径。DDR3 / AWB / HDMI 上下游是既有域，改动需单独评估。

---

## 2. 文件归属与目录约定

| 路径 | 归属 | 说明 |
| --- | --- | --- |
| `example_top.v`、`Ti60_Demo.xml`、`start.cmd` | **集成负责人独占** | 其他人不要动 |
| `src/image/` | 算法模块 | 3x3 窗口、Sobel 等 |
| `src/display/` | 显示合成 | 原图/边缘图合成、画中画 |
| `src/common/` | 通用模块 | 延迟对齐等 |
| `src/isp/` `src/axi/` `src/cmos_i2c/` `src/hdmi_ip/` `src/lvds/` `src/dsi/` | 基线只读 | 改动需 2 人 approve |
| `sim/`、`tools/` | 验证与工具 | testbench、参考模型、一键脚本 |
| `docs/` | 文档 | 接口契约、架构、验证报告 |
| `rfc/` | 设计决策记录 | 见 §7 |

**模块命名**：新增模块名不得与基线已有模块重名，否则综合/仿真会直接看到两个同名 `module` 而报错。
如果发现目标路径已被他人占用，**改自己的名字**（例如把 `image_mix` 改为 `edge_display_mix`），不要改名别人的文件。

> 顺带一条：`src/axi/axi4_ctrl.v` 才是被编译的那一份；`src/axi4_ctrl.v`（同名模块的旧版）不在工程中，改它没有任何效果。

### 2.1 交付单元

开任务 Issue 时选的就是下面这些单元。每个单元**可独立认领、可独立验证**。

| 单元 | 名称 | 主要文件 | 状态 |
| --- | --- | --- | --- |
| **U1** | 3x3 像素窗口发生器 | `src/image/pix_window_3x3.v` | 已完成 |
| **U2** | Sobel 边缘检测 | `src/image/sobel_edge.v` | 已完成 |
| **U3** | 流水线顶层与系统集成 | `src/image/image_pipeline_top.v`、`example_top.v` 接入 | 已完成 |
| **U4** | 显示合成 | `src/display/edge_display_mix.v` | 已完成（含 mode 3 画中画） |
| **U5** | 可选扩展 | 中值滤波 / 按键切模式 / 测试图样 / Canny / 目标识别 | 未开始 |
| **V1** | 验证与工具 | `sim/`、`tools/`、`docs/L2_VERIFY.md` | 已有 L1 + L2 |
| **infra** | 基础设施 | `src/common/`、工程登记、文档、流程与仓库治理 | 持续 |

U1–U4 的接口契约见 `docs/INTERFACE.md`，延迟推导与实现陷阱见 `docs/ARCH.md`，
当时为什么这么定见 `rfc/`。

---

## 3. 分支与提交

### 3.1 分支命名

```
feature-<主题>      新功能        feature-edge-detect-pipeline
fix-<现象>          修 bug        fix-hsync-off-by-7
docs-<主题>         纯文档        docs-l2-verify-report
```

不要直接往 `main` 推。

### 3.2 提交信息

采用 Conventional Commits，**类型与摘要用英文关键词，正文用中文**：

```
<type>(<scope>): <一句话摘要，不超过 50 字>

- 关键实现决策（为什么这么做，而不是做了什么）
- 踩到的坑与根因
- 验证方式与结果

Refs: #<issue>      如有关联 Issue
```

| type | 用途 |
| --- | --- |
| `feat` | 新功能 |
| `fix` | 修缺陷 |
| `refactor` | 不改行为的重构 |
| `test` | testbench、参考模型、验证脚本 |
| `docs` | 文档 |
| `chore` | 构建、路径、配置 |

`scope` 用模块域：`image` / `display` / `verify` / `top` / `sim`。

真实示例（可直接参考）：

```
feat(display): edge_display_mix 实现 mode 3 画中画（1 bit 二值 + 2:1 抽取）

全彩画中画需 640x360x24 = 5.53 Mbit，超出 Ti60 全部 EBR（2.56 Mbit），
故降级为 1 bit 二值 + 2:1 抽取，缓冲降到 640x360/8 = 28800 B ≈ 23 块 EBR。

- 写侧位序：第 1 个样点落 bit0、第 8 个落 bit7，必须用右移拼接。
  左移拼接会得到 MSB 优先、整体反位（实测踩过）。
- 读侧地址：BRAM 自身有 1 拍读延迟，地址必须用组合预测地址直喂，
  不能再插一级地址寄存器，否则整体错位一个取样点（实测踩过）。
```

**一次提交只做一件事**，不要混装 RTL + 文档 + 无关格式化。

---

## 4. 开发与验证流程

验证分三级，**上一级不过不要进下一级**。

| 级别 | 内容 | 命令 | 通过标准 |
| --- | --- | --- | --- |
| **L1** | 模块级仿真 | `sh sim/run.sh` | `*** PASS ***`，288 项检查全过，退出码 0 |
| **L2** | 算法正确性 | `sh sim/l2_verify.sh` | `*** L2 PASS ***`，契约区逐像素 0 不符，退出码 0 |
| **L3** | 集成综合 / 上板 | `efx_run.bat Ti60_Demo.xml` | map / interface / pnr / pgm / bitstream 全 PASS，时序 0 负 slack |

L2 的判定依据是**独立的 Python 参考模型**（`tools/l2_ref_model.py`），它**不读 RTL 代码**，只按
`docs/INTERFACE.md` 的冻结公式重算。**不允许**用"拿 RTL 输出跟 RTL 自己比"来冒充 L2。

跑完把结果写进 `docs/L2_VERIFY.md` 或 PR 描述里的「验证证据」栏，附报告路径。

### 提交前自检

```sh
sh sim/run.sh && sh sim/l2_verify.sh && git status --short
```

最后一条输出应为空（**不要提交仿真产物**：`.vvp`、`.vcd`、像素流转储都已被 `.gitignore` 排除；
但 `sim/l2_report.txt` 是验证证据，**要提交**）。

---

## 5. 公共文件纪律

`example_top.v`、`Ti60_Demo.xml`、`*.peri.xml`、`*.pt.sdc` 是集成负责人独占。如果你确实需要改：

1. **先在 Issue 里声明**，说明改哪几行、为什么；
2. 改完后**在 PR 里勾选**「本次为集成负责人的公共文件提交」；
3. `example_top.v` 里任何**延迟改动**都必须同步补 `hsync`：数据路径补了 N 拍，`hsync` 就必须补 N 拍，
   否则整幅横移 N 列并丢掉最右 N 列。

---

## 6. 文档要求

改了行为就要同步文档，**同一提交内完成**：

- 端口、`LATENCY`、参数变了 → `docs/INTERFACE.md`
- 结构、插入点、资源、陷阱变了 → `docs/ARCH.md`
- 验证结论变了 → `docs/L2_VERIFY.md`

每个新增模块的**文件头注释**必须包含：功能、`LATENCY` 推导过程、边界策略、参数说明。四样缺一不可。

---

## 7. RFC 流程

什么时候写 RFC：**会改变已冻结接口、或影响多人的决策**。例如插入点选择、`LATENCY` 契约、
模块命名与所有权、资源降级方案。

- 目录：`rfc/`，命名 `NNNN-短标题.md`（四位序号，连字符，全小写）
- 模板：`rfc/0000-template.md`
- 流程：**Draft** → 提交 Issue 讨论 → 2 人 approve 后改状态为 **Accepted** → 实现
- 前端接口变更**必须**先有 Accepted 的 RFC，才能动 `docs/INTERFACE.md` 的冻结章节
- 已被推翻的 RFC **不要删**，把状态改成 `Superseded by #NNNN` 并写清原因（历史本身就是资产）

---

## 8. 坑速查

| 现象 | 根因 | 处置 |
| --- | --- | --- |
| 板上毫无变化 | 新增 `.v` 没登记进 `Ti60_Demo.xml` | 见 §1.2 |
| `Ti60_Demo.xml` 出现几百行 diff | `efx_run` / GUI Generate 规范化重写 | `git checkout --` 恢复 |
| 现有比特流没了 | `start.cmd` 开头就 `rmdir /s /q outflow work_syn work_pnr work_pt` | 别随手跑 `start.cmd` |
| 画面整体横移 N 列、右边缘丢 N 列 | 握手指令延迟 ≠ 数据延迟 | `hsync` 用 `sync_delay_align` 补同样的拍数 |
| 画面花屏、色彩错乱 | 动了 `src/isp/` 的公共行缓存 | 回滚，复制副本到 `src/image/` 再改 |
| 两个同名 `module` 报错 | 与基线或他人分支撞名 | 改自己的模块名 |
| 综合报告里某模块只有几个 FF / 0 RAM | 输入被常量绑定，整块被死代码消除（DCE） | 检查顶层有没有把 `mode_i` / `thr_i` 写死 |
| 改了 `src/axi4_ctrl.v` 无效果 | 被编译的是 `src/axi/axi4_ctrl.v` | 改对的那一份 |

---

## 9. 提问与求助

优先开 Issue（模板见 `.github/ISSUE_TEMPLATE/`）：

- 开发任务用 `task.yml`（要求写清交付物、接口、DoD）
- 缺陷用 `bug.yml`（要求附复现命令、分支与 commit）

描述现象时请给**客观证据**：波形截图、`$fwrite` 转储的原始值、报错原文。避免"好像不对"。
