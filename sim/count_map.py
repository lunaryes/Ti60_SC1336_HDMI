#!/usr/bin/env python3
"""从 Efinity 的 post-map 网表里按层次前缀统计资源（ARCH §4 的数据来源）。

网表是**扁平化**的（只有一个 module example_top），但单元的实例名保留了层次路径
（`\\u_image_pipeline/u_canny/...`），所以按名字前缀分组即可得到每个子树的资源。

★ 只能数 FF / ADD / SRL8 / RAM10 / DSP，**不能数 LUT**：EFX_MAP 的优化会把组合逻辑
  改写成合成名（`LUT__12345`、`CutToMuxOpt_*`），丢掉层次路径。实测保留率
  FF/ADD/RAM10 = 100%，LUT4 = 0.2%。脚本最后会把这个保留率打出来自证。

先有网表再跑（工程里没有现成的综合驱动脚本，启动方式见 ARCH.md §4 注）：
    python3 sim/count_map.py Ti60F225_SC1336_HDMI_Display/outflow/Ti60_Demo.map.v
"""
import re
import sys
from collections import defaultdict

CELLS = ("EFX_FF", "EFX_LUT4", "EFX_ADD", "EFX_SRL8", "EFX_RAM10", "EFX_DSP48", "EFX_DSP24")

txt = open(sys.argv[1], encoding="utf-8", errors="replace").read()
# 单元实例一律是转义标识符形式： EFX_XXX \path/name
# ★ 必须要求这个反斜杠：文件里还有形如 `EFX_LUT4(*)` 的注释/连接文本，
#   宽松的正则会把它们也数进去（第一版就踩了，LUT4 全被算成 0）。
hits = re.findall(r"\b(EFX_[A-Z0-9]+)\s+\\([^\s;(]+)", txt)

def bucket(name, depth):
    parts = name.split("/")
    return "/".join(parts[:depth]) if len(parts) > depth else name

lvl1 = defaultdict(lambda: defaultdict(int))
lvl2 = defaultdict(lambda: defaultdict(int))
for cell, name in hits:
    if cell not in CELLS:
        continue
    lvl1[bucket(name, 1)][cell] += 1
    lvl2[bucket(name, 2)][cell] += 1

def show(title, table, filt=None, mincells=0):
    print("\n== %s ==" % title)
    print("%-40s %6s %6s %6s %6s %6s %5s" % ("inst", "FF", "LUT4", "ADD", "SRL8", "RAM10", "DSP"))
    rows = []
    for k, v in table.items():
        tot = sum(v.values())
        if tot < mincells:
            continue
        if filt and not any(s in k for s in filt):
            continue
        rows.append((tot, k, v))
    for tot, k, v in sorted(rows, reverse=True):
        print("%-40s %6d %6d %6d %6d %6d %5d" % (
            k[:40], v.get("EFX_FF", 0), v.get("EFX_LUT4", 0), v.get("EFX_ADD", 0),
            v.get("EFX_SRL8", 0), v.get("EFX_RAM10", 0),
            v.get("EFX_DSP24", 0) + v.get("EFX_DSP48", 0)))

show("顶层（第一段路径）", lvl1, mincells=50)
show("新增的算法/显示子树（第二段路径）", lvl2,
     filt=("image", "canny", "gray", "pix", "mix", "uart", "sobel"), mincells=1)

tot = defaultdict(int)
for cell, _ in hits:
    if cell in CELLS:
        tot[cell] += 1
print("\n== 全器件合计（网表里数出来的，可与 map 日志的 Resource Summary 对照）==")
for c in CELLS:
    print("  %-12s %d" % (c, tot[c]))

# ★ 重要限制：EFX_MAP 的优化过程会把一部分单元改名成合成名（LUT__12345、
#   CutToMuxOpt_*），**丢掉原来的层次路径**。带路径的单元能按子树统计，
#   合成名的不能 —— 所以"每个模块用了多少 LUT"从 post-map 网表里**根本数不出来**
#   （LUT 几乎全是合成名），FF/RAM/DSP 基本保留路径，可以数。
#   这个数字就是诚实度指标：合成名占比越低，下面的分模块表越可信。
print("\n== 层次名 vs 合成名（分模块表的可信度）==")
for c in ("EFX_FF", "EFX_LUT4", "EFX_ADD", "EFX_RAM10"):
    esc = len(re.findall(r"\b%s\s+\\[^\s;(]+" % c, txt))
    syn = len(re.findall(r"\b%s\s+(?!\\)[^\s;(]+" % c, txt))
    keep = 100.0 * esc / (esc + syn) if esc + syn else 0.0
    print("  %-10s 带路径 %5d  合成名 %5d  → 保留率 %5.1f%%" % (c, esc, syn, keep))
