#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""color_probe —— 色卡判决探针（M5 新增）

把 M5 那次"只有高对比度才出边"事故背后的**背表**脚本化：给一队原始 RGB，
一口气算出 AWB 削顶 → rgb_to_gray → 高斯 → Sobel → mag8 的**精确定点结果**，
再对照阈值给出判决。上板之前就能知道"这个颜色在 thr=N 下到底出不出边"。

为什么需要它：`canny_ref.py` 是"实现对不对"的逐像素回归，参考模型与 DUT 共享
同一组阈值常量，**结构上验证不了阈值合不合适**（M5 事故就是这么漏过去的）。
这个脚本补的就是"标定"这一维：它不跟任何 DUT 比，只回答"满幅多少对比度才过阈值"。

用法：
    python3 sim/color_probe.py                    # 标尺表 + 内置 8 色测试卡
    python3 sim/color_probe.py --thr 8 16 24 40   # 指定几档阈值看判决
    python3 sim/color_probe.py --rgb 237,28,36 --name 红   # 加自己的实测色
    python3 sim/color_probe.py --raw-rgb 12,200,80        # 未过 AWB 的原始值

复用 canny_ref.py 的整数算子，保证与 RTL 逐位一致（那套算子已经被
tb_canny_edge 的逐像素回归钉死）。
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import canny_ref as C          # noqa: E402


# ---------------------------------------------------------------------
# 1) AWB 与灰度：与 RTL 逐位一致
# ---------------------------------------------------------------------
def awb_clip(rgb):
    """example_top.v 的手动白平衡：R×409/256、B×384/256，溢出即饱和到 255。

    RTL 的写法是"乘完看 bit[17:16] 是否为 1"（= 乘积 >= 65536），等价于
        R' = 255        if R >= 161
        R' = (R*409)>>8 otherwise
    所以红系**亮**的时候会被压平成一个值，色块内部的层次会丢，但"饱和色块 vs
    黑字"这对端点不受影响（黑字 R=0 → R'=0）。
    """
    r, g, b = rgb
    return (min(r * 409, 0xFFFF) >> 8,
            g,
            min(b * 384, 0xFFFF) >> 8)


def gray(rgb):
    """rgb_to_gray.v：y = (77R + 150G + 29B) >> 8（无舍入）"""
    r, g, b = rgb
    return (r * 77 + g * 150 + b * 29) >> 8


# ---------------------------------------------------------------------
# 2) 标尺：ΔY → mag8，直接用模型算，不写解析式
# ---------------------------------------------------------------------
# 用"黑字 0 → 色块 Y"这对端点算，跟测试卡上数字边缘的实际情况一致：
# 背景是色块（Y>0），笔画是黑（Y=0），阶跃幅度就是 Y 本身。
# 取解析式 mag8 = 3·ΔY/8 也行，但那样算不出 >>4 截断带来的 ±1 偏差，
# 而调阈值恰恰是 1 个 LSB 就要命的场合（品红当初就卡在 39/40）。
_RULER_CACHE = {}


def step_mag8(dy, diag=False, lo=0):
    """幅度为 dy 的阶跃边（竖边 / 45° 边）在模型里产生的 mag8 峰值"""
    key = (dy, diag, lo)
    if key in _RULER_CACHE:
        return _RULER_CACHE[key]
    W, H = 48, 24
    hi = min(lo + dy, 255)
    img = [[(hi if (r - c) >= 0 else lo) if diag else (hi if c >= W // 2 else lo)
            for c in range(W)] for r in range(H)]
    mags, _ = C.ref_grad(img, W, H)
    c0 = H // 2 if diag else W // 2
    peak = max(mags[r][c] >> 3
               for r in range(4, H - 4) for c in range(c0 - 3, c0 + 4))
    _RULER_CACHE[key] = peak
    return peak


def min_dy_for(thr, diag=False):
    """反查：要让一条边达到 mag8 >= thr，两侧灰度差至少要多少"""
    for dy in range(1, 256):
        if step_mag8(dy, diag) >= thr:
            return dy
    return None


# ---------------------------------------------------------------------
# 3) 内置测试卡（原始 RGB，上机前的标称值；实测值用 --rgb 覆盖）
# ---------------------------------------------------------------------
CARD = [
    ("白",   (255, 255, 255)),
    ("黄",   (255, 255, 0)),
    ("青",   (0, 255, 255)),
    ("绿",   (0, 255, 0)),
    ("灰",   (128, 128, 128)),
    ("品红", (255, 0, 255)),
    ("红",   (237, 28, 36)),
    ("紫",   (128, 0, 255)),
    ("蓝",   (0, 0, 255)),
    ("黑",   (0, 0, 0)),
]

# 现场色卡（2026-10-15 上板实测所用；标称 RGB 由集成负责人提供，屏幕显示/图片压缩有轻微偏差）
# ★ 这张卡上**红 (220,30,30) 与深紫 (110,30,200) 的标称 Y 完全并列**（都是 99，mag8 都是 37），
#   而板上深紫明显更难 —— "标称并列、实测分家"是判别**通道性衰减 vs 整体欠曝**的天然对照对：
#   红的 R=220 ≥ 161 会被 AWB 削顶（77/99 的 Y 来自饱和通道，上游 R 再弱也咬不动），
#   深紫的 R=110 < 161 不削顶、随上游线性缩水。详见 CANNY_PLAN §10。
CARD_USER = [
    ("红",   (220, 30, 30)),
    ("亮黄", (255, 240, 0)),
    ("亮绿", (0, 210, 70)),
    ("青",   (80, 220, 240)),
    ("深紫", (110, 30, 200)),
    ("品红", (255, 40, 180)),
    ("白",   (255, 255, 255)),
    ("中灰", (140, 140, 140)),
]


def report_ruler(thrs):
    print("=" * 78)
    print("标尺：竖边 / 45° 边，模型实测（不是解析式 mag8 = 3·ΔY/8，含 >>4 截断）")
    print("-" * 78)
    print("  ΔY  |  竖边 mag8 |  45° mag8 | 占满幅 | 判决（thr = %s）"
          % " / ".join(str(t) for t in thrs))
    for dy in (8, 16, 22, 32, 43, 54, 64, 86, 107, 128, 160, 192, 255):
        m_v, m_d = step_mag8(dy), step_mag8(dy, diag=True)
        verdict = " ".join("强" if m_v >= t else ("弱" if m_v >= t // 2 else "-")
                           for t in thrs)
        print("  %3d  |    %3d     |    %3d    |  %3.0f%%  | %s"
              % (dy, m_v, m_d, dy * 100.0 / 255, verdict))
    print("  （弱 = 落在 [thr/2, thr) 里：会进 NMS，但不贴强边就整条丢 —— 见 CANNY_PLAN §3.7）")


def report_reverse(thrs):
    print()
    print("反查：thr_high = N 时，横竖边两侧至少要多大的灰度差才判强边")
    print("-" * 78)
    print("  thr :", "".join("%6d" % t for t in thrs))
    print("  ΔY≥ :", "".join("%6d" % min_dy_for(t) for t in thrs))
    print("  占幅 :", "".join("%5.0f%%" % (min_dy_for(t) * 100.0 / 255) for t in thrs))
    print("  ★ thr 的可用量程只有 0..96（横竖，对角约 140）—— 3x3 高斯把 Sobel 的")
    print("    4A 阶跃响应压成了 3A。把 thr 当成 'n/255 的对比度' 会高估 2.7 倍，")
    print("    这正是 M5 事故（默认值 40 = 要求 42% 对比度）；经验值 16（17%）。")


def report_card(card, thrs):
    print()
    print("=" * 78)
    print("测试卡判决（黑字 0 → 色块 Y 的阶跃；\"–\" = mag8 < thr/2，连弱边都算不上）")
    print("-" * 78)
    print("  色块 | 原始 RGB        | AWB 后 RGB      | Y  | mag8 | 占幅 | 判决（thr = %s）"
          % " / ".join(str(t) for t in thrs))
    rows = []
    for name, rgb in card:
        r2 = awb_clip(rgb)
        y = gray(r2)
        m8 = step_mag8(y) if y > 0 else 0
        verdict = " ".join("强" if m8 >= t else ("弱" if m8 * 2 >= t else "–") for t in thrs)
        print("  %-4s | %-15s | %-15s | %3d | %4d | %3.0f%% | %s"
              % (name, "%3d,%3d,%3d" % rgb, "%3d,%3d,%3d" % r2, y, m8,
                 m8 * 100.0 / 96, verdict))
        rows.append((name, y, m8))
    print("  占幅 = mag8 / 96（竖边的物理满量程），不是 mag8/255。")
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--thr", type=int, nargs="*", default=[8, 16, 24, 40],
                    help="要对照判决的 thr_high 档位（默认 8 16 24 40）")
    ap.add_argument("--rgb", action="append", default=[],
                    help="追加色块，格式 R,G,B（已过 AWB 的显示域）")
    ap.add_argument("--raw-rgb", action="append", default=[],
                    help="追加色块，格式 R,G,B（未过 AWB 的原始域）")
    ap.add_argument("--no-ruler", action="store_true")
    ap.add_argument("--card", choices=("std", "user"), default="std",
                    help="std = 内置通用色卡；user = 现场实测色卡（CANNY_PLAN §10 的对照对在这张上）")
    args = ap.parse_args()

    thrs = sorted(set(args.thr))
    card = list(CARD_USER if args.card == "user" else CARD)
    for s in args.rgb:
        card.append(("自定", tuple(int(x) for x in s.split(","))))
    for s in args.raw_rgb:
        card.append(("自定raw", tuple(int(x) for x in s.split(","))))

    if not args.no_ruler:
        report_ruler(thrs)
        report_reverse(thrs)
    rows = report_card(card, thrs)

    # 默认值 16 的余量表：最差的那块还差多少 / 还剩多少（上板就看这个数）
    print()
    print("默认 thr=16 的余量（mag8 - 16；负数 = 判不出强边，只会是弱边或全无）：")
    margin = [(m8 - 16, name) for (name, y, m8) in rows if m8 > 0]
    for d, name in sorted(margin):
        print("    %-6s %+4d" % (name, d))

    print()
    print("★ 上板对照：把 thr 逐级降/升，色块应在**预测的 mag8 值**附近翻面（强↔弱）。")
    print("  若某色块在 thr 降到 8 时仍然全黑，说明问题不在阈值，去查 AWB 削顶 / 等亮度色")
    print("  （判决表见这两处注释：canny_edge.v 头注释的标尺表、CANNY_PLAN §3.8）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
