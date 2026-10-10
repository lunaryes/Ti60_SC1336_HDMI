## 关联 Issue

Closes #

## 变更内容

<!-- 一句话说清改了什么；列表说明关键实现决策（为什么这么做，而不是做了什么） -->

-

## 验证证据

<!-- L1 / L2 的输出摘要 + 报告路径；L3 上板截图。纯文档改动写"无" -->

- [ ] **L1** 模块仿真通过 —— `sh sim/run.sh`（期望末行 `*** PASS ***`，退出码 0）
- [ ] **L2** 与 Python 参考模型逐像素比对通过 —— `sh sim/l2_verify.sh`（期望 `*** L2 PASS ***`）
- [ ] **L3** 集成综合 / 上板验证（里程碑节点补，普通 PR 可留空）
- 证据位置：（例：`sim/l2_report.txt`、`docs/L2_VERIFY.md`）

> 复现命令一律以仓库根为工作目录。CI 未接入，以上均需在本地跑通。

## 接口与文档

- [ ] 未变更已冻结接口；**如变更**，已关联 `rfc/` 中状态为 Accepted 的 RFC，并获 2 人 approve
- [ ] 新增/修改模块的文件头注释包含四要素：功能、`LATENCY` 推导、边界策略、参数说明
- [ ] 相关文档已**在本 PR 内**同步（`docs/INTERFACE.md` / `docs/ARCH.md` / `docs/L2_VERIFY.md`）
- [ ] 若属「会改变冻结接口或影响多人」的决策，已在 `rfc/` 落一篇 RFC，并在 `rfc/README.md` 索引表登记

## 工程登记与公共文件纪律

- [ ] 新增的 `.v` 文件已登记进 `Ti60_Demo.xml`
      （只把文件放进 `src/` 是**无效**的，板上不会有任何变化。
      若本 PR 不做登记，请注明由谁在哪个 Issue 中补 —— 未登记 = 不进比特流）
- [ ] `Ti60_Demo.xml` 已 `git checkout --` 恢复为手工维护版本，
      未混入 `efx_run` / GUI Generate 的规范化重写（会产生几百行无意义 diff）
- [ ] 本次 PR **未**修改 `example_top.v` / `Ti60_Demo.xml` / `start.cmd`
      （如是集成负责人的登记提交，请勾选下一项）
- [ ] 本次为集成负责人的公共文件提交，已在 Issue 中提前声明
- [ ] 若改了 `example_top.v` 里的数据路径延迟，`hsync` 已同步补上**相同拍数**
      （漏了就会整幅横移 N 列并丢失最右 N 列）
- [ ] 未修改 `src/isp/Line_Shift_RAM_8Bit.v`、`src/isp/VIP_Matrix_Generate_3X3_8Bit.v`（基线活链路，只读）
- [ ] 未修改 `Ti60_Demo.peri.xml` / `Ti60_Demo.pt.sdc`（Interface Designer 生成物，禁手改）
- [ ] 未新增跨时钟域逻辑（算法全部在 `clk_pixel` 单域内）

## 风险与遗留

- [ ] 已知限制 / 未覆盖的边界情况（如无则写"无"）
