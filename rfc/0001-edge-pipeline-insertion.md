# RFC 0001 — 边缘检测流水线的插入点与 7 拍延迟契约

| 项 | 值 |
| --- | --- |
| 状态 | Accepted（2026-10-09）→ Implemented |
| 作者 | @yqiiu |
| 日期 | 2026-10-09 |
| 关联 | 无 |
| 影响范围 | `example_top.v`、`src/image/image_pipeline_top.v`、`src/common/sync_delay_align.v`、`docs/INTERFACE.md` §2/§3/§4/§6 |

---

## 1. 背景

基线（提交 `1fc9192`「完成灰度转换」）的数据通路是：

```
CM-1336 → AWB → rgb_to_gray → FrameBoundCrop(SKIP_ROWS(2), SKIP_COLS(2),
                                          TOTAL_ROWS(720), TOTAL_COLS(1280))
        → rgb2dvi / hdmi_tx_ip → HDMI
```

`rgb_to_gray` 之后**画面就是灰度图了**，边缘检测没有任何位置。

三个既有事实约束了插入点的选择：

1. `FrameBoundCrop` 的裁剪语义是 `w_skip = (rc_w < SKIP_COLS) || (rc_w >= TOTAL_COLS - SKIP_COLS)`，
   即**每边各裁 2 行 2 列**：可见区 = 行 `[2, 717]`、列 `[2, 1277]`。它同时承担了"把边界的无效像素扔掉"的职责。
2. 3x3 窗口必须拿到**下一行**才能算中心行，天然引入 **1 行垂直延迟**，所以**末行永远算不出来**。
3. `src/isp/Line_Shift_RAM_8Bit.v` 与 `src/isp/VIP_Matrix_Generate_3X3_8Bit.v` 是当前画面能显示的
   活链路（`bayer2rgb/u_VIP_Matrix_Generate_3X3_8Bit/u0_|u1_Line_Shift_RAM_8Bit`），**禁止修改**。

## 2. 问题

边缘检测流水线**插在哪一段**，以及它的延迟如何与既有握手指令对齐？

## 3. 备选方案

### 方案 A：插在 `rgb_to_gray` 之后、`FrameBoundCrop` 之前 ← 采用

- 做法：在 `rgb_to_gray` 与 `FrameBoundCrop` 之间插入 `image_pipeline_top`，
  对外 `LATENCY = 7`（窗口 2 + Sobel 4 + 合成 1）；`hsync` 用 `sync_delay_align #(.DEPTH(7), .DW(1))` 单独补 7 拍。
- 优点：
  - 输入已经是灰度（`Y = (77R + 150G + 29B) >> 8`），Sobel 直接可用，**不需要再算一次灰度**；
  - 下游 `FrameBoundCrop` 会把边界无效像素裁掉，因此**本模块完全不需要做边界处理**——
    窗口越界时输出什么都会被裁掉；
  - 该段仍在 `clk_pixel` 单时钟域内，不引入跨域路径。
- 缺点：3x3 的固有 1 行垂直延迟会让 `mode 0`（原图直通）相对基线**整体下移 1 行**。
- 代价 / 风险：需要改 `example_top.v`（集成负责人独占文件），且 `hsync` 必须同步补延迟，漏了就横移。

### 方案 B：插在 `FrameBoundCrop` 之后

- 做法：先裁剪再检测，把流水线串在裁剪与 `rgb2dvi` 之间。
- 优点：输入本身就是"干净"的可见区，不用依赖下游兜底边界。
- 缺点：**致命**。裁剪后每行只剩 1276 列，3x3 窗口在最外两列缺邻域，会在可见区内产生一圈假边缘；
  而且 `FrameBoundCrop` 的行输出时序与 DE 的关系在裁剪后不再简单，重新对齐握手的成本更高。
- 否决理由：把"边界兜底"的职责从下游挪到本模块，既扩大了本模块职责，又在**可见区内**引入了伪影。

### 方案 C：插在 AWB 之前或 `rgb_to_gray` 之前

- 做法：在彩色域直接做边缘检测。
- 缺点：需要对 R/G/B 三个通道各做一次 3x3 窗口与 Sobel，或者先转灰度——
  **等于把 `rgb_to_gray` 重做一遍**，资源翻三倍，且与基线已冻结的灰度公式存在不一致风险。
- 否决理由：重复计算，且违反"复用基线成果"的原则。

### 方案 D：不做 `hsync` 延迟对齐，直接沿用原 `hsync`

- 做法：只给数据路径补移位，握手指令原样透传。
- 缺点：数据比 `hsync` 晚 7 拍，**整幅画面横移 7 列，最右 7 列丢失**。
- 否决理由：这是最容易漏、也最难定位的一类错误——画面看起来"能显示"，只是位置不对，
  很容易被误判为摄像头或裁剪参数问题。必须用 `sync_delay_align` 显式补上。

## 4. 决策

**采用方案 A**：插入点在 `rgb_to_gray` 与 `FrameBoundCrop` 之间，对外 `LATENCY = 7`，
`hsync` 用 `sync_delay_align #(.DEPTH(7), .DW(1))` 补齐 7 拍。

依据：

1. 输入已是灰度，复用基线成果，零重复计算；
2. 边界无效像素由下游 `FrameBoundCrop` 兜底，本模块职责单一，**不做边界处理**；
3. 全程 `clk_pixel` 单域，不新增跨时钟域路径。

拍数分解（`docs/INTERFACE.md` §2 为权威来源）：

| 级 | 模块 | LATENCY |
| --- | --- | --- |
| 1 | `pix_window_3x3` | 2 |
| 2 | `sobel_edge` | 4 |
| 3 | `edge_display_mix` | 1 |
| — | **合计** | **7** |

## 5. 影响

- 需要改的文件：`example_top.v`（插入例化 + `hsync` 延迟）、`Ti60_Demo.xml`（登记 5 个新增源文件）
- 对已冻结接口的影响：新增接口，**不改动**基线任何既有接口
- 资源影响：XLR 6924/60800 = 11.39%、Memory 56/256 = 21.88%、DSP 4/160 = 2.50%（Ti60F225 C4，Efinity 2026.1.132）
- 对验证的影响：
  - L1 新增 `sim/tb_image_pipeline.v`（逐像素对照）与 `sim/tb_window_boundary.v`（边界基线）
  - L2 新增 `sim/l2_verify.sh` + `tools/l2_ref_model.py`，覆盖 `MODE 0/1/2/3 × THR 40/100` 共 8 组

## 6. 未覆盖 / 已知限制

1. **固有 1 行垂直延迟**：3x3 需要"下一行"，末行算不出来。因此 `mode 0`（原图直通）相对基线整体下移 1 行。
   这不影响可见区正确性（`FrameBoundCrop` 每边裁 2 行），但**画面比例上确实差了一行**，属已知接受项。
2. **边界像素未定义**：窗口越界时输出的具体值不做保证，理由是这些位置必被裁剪，属"不可见"。
   如果将来把 `SKIP_ROWS` / `SKIP_COLS` 调到小于 2，该假设即失效。
3. 未评估把流水线拆到多个时钟域以换取时序余量的方案（当前单域时序余量充足，无必要）。

## 7. 实现位置

- 提交：`298e3d9`（算法模块）、`97544f0`（显示合成 + 延迟对齐）、`29a2026`（顶层接入）
- 代码：`src/image/{pix_window_3x3,sobel_edge,image_pipeline_top}.v`、`src/common/sync_delay_align.v`
- 验证证据：`sim/l2_report.txt`、`docs/L2_VERIFY.md`
