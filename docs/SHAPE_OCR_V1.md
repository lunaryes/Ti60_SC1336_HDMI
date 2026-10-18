# 简易图形与字符识别 V1

本目录是一套与现有 Canny、串口和顶层解耦的实验模块。它不会修改现有视频链路，便于在独立分支中评审和合并。

## 识别范围

- 数字：`0`～`9`
- 大写英文字母：`A`～`Z`
- 图形：圆、椭圆、三角形、正方形、矩形、菱形、水平线、垂直线、十字、叉号、上/右/下/左箭头

这里的“准确”是指受控场景：一个符号完整落在固定 ROI 内，前景和背景已经二值化，字体与模板接近，没有任意角度旋转和严重透视。任意字体、手写体、小写字母和自然场景 OCR 不属于本版本范围。

## 数据通路

```text
灰度/二值化前景
    -> fixed_roi_glyph_sampler（固定 ROI 归一化为 8x8）
    -> symbol_template_matcher（50 类汉明距离）
    -> class_id/ascii/距离/拒识
    -> recognition_overlay（ROI、放大符号、置信条）
```

字符识别应接阈值化的灰度前景，而不是直接接 Canny 边缘。建议前景定义为 `gray < bin_threshold`；白底黑字时黑字输出 1。图形若使用实心或线框卡片，也使用相同二值输入。直接接 `edge_o` 时，模板也必须改成对应的轮廓形式。

## 文件

| 文件 | 作用 |
| --- | --- |
| `fixed_roi_glyph_sampler.v` | 将固定 ROI 按占用率降采样为 8×8 位图 |
| `symbol_template_rom.v` | 50 个数字、字母和图形模板 |
| `symbol_template_matcher.v` | 逐模板计算汉明距离，输出最佳和次佳结果 |
| `simple_shape_ocr.v` | 采样器与匹配器封装 |
| `recognition_overlay.v` | 在视频上画 ROI、识别符号和置信条 |
| `tb_symbol_template_matcher.v` | 50 个原模板、50 个两位扰动和空输入自检 |
| `sim/check_symbol_templates.py` | 50 个原模板及全部 3200 种单格扰动的静态可分性检查 |

## 类别编号

| class_id | 类别 |
| ---: | --- |
| 0 | 未识别/拒识 |
| 1～10 | `0`～`9` |
| 11～36 | `A`～`Z` |
| 37 | 圆 |
| 38 | 椭圆 |
| 39 | 三角形 |
| 40 | 正方形 |
| 41 | 矩形 |
| 42 | 菱形 |
| 43 | 水平线 |
| 44 | 垂直线 |
| 45 | 十字 `+` |
| 46 | 叉号 `×` |
| 47～50 | 上、右、下、左箭头 |

## 拒识规则

匹配器不会无条件选一个最像的类别。只有同时满足以下条件才输出识别结果：

```text
best_distance <= MAX_DISTANCE
second_distance - best_distance >= MIN_MARGIN
```

默认允许 64 个网格中最多 12 个不同，同时要求第一名至少领先第二名 2 格。现场应使用测试卡采集混淆矩阵后再调整；门限越宽松越容易误识别，越严格越容易输出 `UNKNOWN`。

## 顶层接入建议

1. 在灰度输出后增加可调二值阈值：`foreground = gray_y < bin_threshold`。
2. 实例化 `simple_shape_ocr`，初始 ROI 为画面中央 128×128。
3. 将结果寄存到下一帧再显示，避免帧内变化。
4. 在最终 HDMI RGB 输出前实例化 `recognition_overlay`。
5. 第一轮只用打印的统一字体卡片标定 `CELL_HITS_MIN`、`MAX_DISTANCE` 和二值阈值。

本版本使用固定 ROI，目的是先验证算法和显示闭环。自动寻找多个目标需要另加连通域标记、外接框排序和每目标归一化，不应与第一版同时引入。

不依赖 HDL 工具的模板检查：

```bash
python sim/check_symbol_templates.py
```

## 仿真

在安装了 Verilator 的 WSL 或 Linux 环境中：

```bash
verilator --binary --timing -Wno-fatal \
  --top-module tb_symbol_template_matcher \
  Ti60F225_SC1336_HDMI_Display/src/recognition/symbol_template_rom.v \
  Ti60F225_SC1336_HDMI_Display/src/recognition/symbol_template_matcher.v \
  sim/tb_symbol_template_matcher.v \
  -o Vtb_symbol_template_matcher --Mdir sim/build/obj_symbol

sim/build/obj_symbol/Vtb_symbol_template_matcher
```

期望输出（`noisy rejects` 允许非零；有歧义时拒识优于误识）：

```text
SYMBOL_MATCHER PASS: 101 cases, noisy rejects=...
```
