# tools/ — Python 参考模型与比对工具（验证组负责）

不参与综合，不被 Efinity 引用。用途是给 RTL 一个"标准答案"。

## 计划中的工具

| 脚本 | 作用 |
| --- | --- |
| `gen_vectors.py` | 生成测试图（棋盘、灰阶、同心圆、真实照片）→ `sim/vectors/*.hex` |
| `ref_model.py` | 用 NumPy 实现灰度化 + Sobel（边界复制策略一致）→ 期望输出 |
| `compare.py` | 逐像素比对 RTL 输出与参考模型，输出差异图与统计 |

## 依赖

```
python >= 3.10
numpy
pillow    # 仅在读入真实照片时需要
```

## 用法（计划）

```bash
python tools/gen_vectors.py --pattern checker --size 64x64 --out sim/vectors/checker.hex
python tools/ref_model.py  --in sim/vectors/checker.hex --out sim/vectors/checker_expected.hex
# 跑完 RTL 仿真后
python tools/compare.py --rtl sim/out/rtl_out.hex --ref sim/vectors/checker_expected.hex
```

## 关键要求

- **边界策略必须与 RTL 完全一致**（边缘复制，不是补零），否则比对会失败但其实是模型错了。
- 定点运算用整数，不要用浮点后取整——`>>8` 是截断不是四舍五入，两者结果会差 1。
- 比对失败时**先输出差异像素的坐标和值**，再打印统计。定位靠的是坐标，不是统计数。
