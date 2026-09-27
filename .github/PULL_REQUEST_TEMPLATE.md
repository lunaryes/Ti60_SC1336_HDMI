## 关联 Issue

Closes #

## 变更内容

<!-- 一句话说清改了什么；列表说明关键实现决策 -->

-

## 验证证据

<!-- L1 仿真日志 / L2 与 Python 参考模型比对结果 / L3 上板截图。纯文档改动写"无" -->

- [ ] L1 模块仿真通过（命令：`bash sim/scripts/run_iverilog.sh ...`）
- [ ] L2 与 Python 参考模型逐像素比对通过（附报告路径）
- [ ] L3 综合 / 上板验证（里程碑节点补，普通 PR 可留空）
- 证据位置：

## 接口与文档

- [ ] 未变更已冻结接口（如变更，已关联 `rfc/` Issue 并获 2 人 approve）
- [ ] 模块头注释包含：功能、`LATENCY` 推导、边界策略、参数说明
- [ ] 相关文档已同步（`docs/INTERFACE.md`、`docs/ARCH.md` 或模块说明）

## 公共文件纪律

- [ ] 本次 PR **未**修改 `example_top.v` / `Ti60_Demo.xml` / `*.peri.xml` / `*.pt.sdc`
      （如是集成负责人的登记提交，请勾选下一项）
- [ ] 本次为集成负责人的公共文件提交，已在 Issue 中提前声明
- [ ] 未修改 `src/isp/Line_Shift_RAM_8Bit.v`、`src/isp/VIP_Matrix_Generate_3X3_8Bit.v`（基线活链路，只读）
- [ ] 未新增跨时钟域逻辑（算法全部在 `clk_pixel` 单域内）

## 风险与遗留

- [ ] 已知限制 / 未覆盖的边界情况（如无则写"无"）
