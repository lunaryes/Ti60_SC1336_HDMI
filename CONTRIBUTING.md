# 协作开发规范

面向 GitHub 多人协作。约定优先于个人偏好——有冲突先改这份文档，再改代码。

---

## 1. 分支模型

```
main        ← 只接受来自 develop 的合并，且每个里程碑打一次 tag
  └ develop ← 日常集成分支，所有 feature 合入这里
      └ feature/<unit>-<short-desc>   例：feature/u1-pix-window
      └ fix/<short-desc>              例：fix/threshold-saturation
      └ docs/<short-desc>             例：docs/interface-freeze
      └ chore/<short-desc>            例：chore/prj-add-sources
      └ rfc/<short-desc>              接口变更专用，例：rfc/window-latency-change
```

**规则**

- `main` 与 `develop` 禁止直接 push，只能通过 PR。
- 一个分支只做一件事，生命周期尽量不超过 3 天。
- 合并前必须 `git pull --rebase origin develop`，保持线性历史。
- 接口变更必须走 `rfc/` 分支，并在 PR 描述里列出受影响的模块与文件，需 **2 人 approve**。

## 2. 提交信息（Conventional Commits）

```
<type>(<scope>): <subject>
```

| type | 用途 |
| --- | --- |
| `feat` | 新增模块或功能 |
| `fix` | 修 bug |
| `docs` | 文档 |
| `chore` | 工程配置、构建脚本、源文件登记 |
| `test` | 测试台、激励、验证脚本 |
| `refactor` | 不改变行为的重构 |
| `rfc` | 接口变更 |

`scope` 用模块名或目录名：`pix_window`、`sobel`、`prj`、`sim` 等。示例：

```
feat(pix_window): parametric 2-row line buffer with clken and row-edge replication
fix(sobel): saturate |Gx|+|Gy| to 8 bit to stop wraparound at high contrast
chore(prj): add U1/U2 sources to Efinity project
docs(interface): freeze image_pipeline_top port list
```

## 3. 代码风格（Verilog-2001）

- 模块一个文件一个，文件名 = 模块名，小写下划线。
- 端口一律 `wire`/`reg` 显式声明类型，禁止隐式线网。
- 参数化：**图像宽度、位宽、阈值位宽禁止硬编码**，一律 `parameter`。
- 时序逻辑统一 `always @(posedge clk or negedge rst_n)`，异步复位、低有效。
- 每级流水在模块头注释里写清**总延迟 `LATENCY`** 及其推导，并在 TB 里断言。
- 组合逻辑避免锁存器：`if` 必须带 `else`，`case` 必须有 `default`。
- 禁止使用 `initial` 做综合逻辑（只允许在 TB 中使用）。

模块头注释模板：

```verilog
// ============================================================
//  <module_name>  —— <一句话功能>
//  功能   ：...
//  延迟   ：LATENCY = <n> 拍（推导：...）
//  边界   ：第 0/1 行、第 0/1 列如何处理（复制边缘 / 补零）
//  参数   ：IMG_W 图像宽度；DW 像素位宽；...
//  时钟域 ：clk_pixel（单域，无跨时钟逻辑）
// ============================================================
```

## 4. 三级工作流（大多数开发不需要装 Efinity）

| 阶段 | 需要 Efinity | 做什么 |
| --- | --- | --- |
| **L1 模块开发 + 仿真** | 否 | 写 1 个 `.v` + 1 个 TB，用 iverilog / ModelSim 跑 |
| **L2 算法正确性比对** | 否 | 与 Python 参考模型逐像素比对，出比对报告 |
| **L3 集成综合 + 上板** | 是 | 登记源文件 → 综合 → 布线 → 烧录（仅集成负责人操作） |

PR 的验证证据只需 L1/L2；L3 证据在里程碑节点由集成负责人补。

## 5. 公共文件纪律（消除唯一的冲突源）

以下文件**只能由集成负责人修改**，任何其他 PR 附带其改动将被要求 rebase 剔除：

- `Ti60F225_SC1336_HDMI_Display/example_top.v`
- `Ti60F225_SC1336_HDMI_Display/Ti60_Demo.xml`
- `Ti60F225_SC1336_HDMI_Display/Ti60_Demo.peri.xml`
- `Ti60F225_SC1336_HDMI_Display/Ti60_Demo.pt.sdc`

新写的 `.v` **开发期不要登记进工程**。由集成负责人在里程碑边界批量登记，提交信息固定为
`chore(prj): add U<x> sources to Efinity project`。

若某人（误）打开 Interface Designer 并点了 Generate，`git status` 会显示 `*.peri.xml` / `*.pt.sdc` 被改动——
**先 `git checkout --` 还原，再确认是否有意为之**，不要把无意的重新生成提交上去。

## 6. PR 要求

- 标题遵循 Conventional Commits。
- 描述里关联 Issue（`Closes #12`）。
- 附验证证据：仿真日志、波形截图或比对报告；纯文档改动可写"无"。
- 新增模块必须附对应 TB。
- 至少 1 人 review；涉及接口或公共文件的 PR 需 2 人。
- 自查清单见 PR 模板，逐条勾选。
