#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gray_hist —— 原始灰度分布 / 灰度增强收益评估（离线，不参与 run.sh 回归）

为什么需要它
------------
“正常光照偏暗、强光过曝”是**采集端曝光**问题，但要不要在算法前加灰度增强、
加哪一种，不能靠肉眼判断。这个脚本把三件事量化：

  1. 算法实际看到的灰度分布 —— 与 RTL 逐位同构：Y = (77R + 150G + 29B) >> 8；
  2. 这些灰度在 Canny 判据域里还能产生多少梯度 —— 3x3 高斯 -> Sobel -> mag8 = mag>>3；
  3. 每种候选增强（gain / levels / gamma / 直方图均衡）把“过阈像素比例”抬高多少，
     同时把平坦区噪声抬高多少、把高光多削掉多少。

它回答的是“**值不值得加、加哪一种**”，不是“增强后的边缘图好不好看”。
最终效果仍以上板为准；但“直方图有没有余量、增强会不会先削顶/先起噪”在这里就能判死。

采集口径（这一步错了，后面全错）
--------------------------------
* **最可信**：串口发 `4D 01` + `56 03`（mode=1, view=3），让 HDMI 输出 RTL 算出的
  Y 图；用 HDMI 采集卡以 **1280x720、无损 PNG、full range** 抓一帧。三通道相等时
  脚本会明确提示“这是 RTL 的 Y 图”。
* 也可以发 `4D 00`（mode=0，旁路原图）抓 AWB 之后的 RGB —— 这正是算法输入。
  注意：mode 0 是纯基线通路，不含任何本流水线的处理。
* **不要**用手机拍屏幕：屏幕 gamma、摩尔纹、手机自动曝光会把直方图整个改掉。
* **不要**用 JPEG：8x8 DCT 会改变局部梯度，mag8 统计不可信。必须 PNG/BMP。
* 采集工具若有 range 选项，选 **Full / PC range**；limited range（16..235）
  会把直方图两端压掉，脚本只能给警告、无法还原。
* 同一天分别抓：正常光、强光、盖镜头（噪声地板）。多张图一起跑，脚本会给横向对比表。

用法
----
    python sim/gray_hist.py normal.png
    python sim/gray_hist.py normal.png strong.png --grid 3
    python sim/gray_hist.py normal.png --roi 0.3,0.2,0.7,0.8 \\
        --sim "gamma:0.7" --sim "levels:16,235" --sim "eq" --plot hist.png
    python sim/gray_hist.py frame.raw --raw 1280x720 --fmt y8

`--sim` 语法：用 `>` 串联多级，每级 `操作:参数`，按顺序执行、每级都量化回 8 bit
（模拟真实 LUT 链，不是浮点理想变换）：

    gain:1.3                  线性乘 1.3（超 255 削顶）
    levels:16,235             黑白场拉伸：(y-16)*255/(235-16)
    levels:auto               用当前整幅的 p1/p99 当黑白场（仿真方便；硬件要帧统计）
    gamma:0.7                 幂函数 255*(y/255)^0.7（<1 提暗部，>1 压暗部）
    eq                        按当前直方图做全局均衡（整幅，不是 ROI）

注意
----
* 阈值映射表来自 `docs/INTERFACE.md` §4.5：横竖边 mag8 ≈ 3*ΔY/8，
  thr=12 要求 ΔY ≥ 32（满幅 12.5%）。对角边大约再高 1.45 倍。
* 边界 4 像素不参与梯度统计（RTL 行缓存边界行为与理想 edge-pad 不同）。
* 统计口径是“算法输入端”，不是显示器观感。曝光问题的根因在
  `src/cmos_i2c/I2C_SC1336_1280720_Config.v`（3e01/3e02/3e08 固定值，无 AE 环）。
"""

import argparse
import json
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageOps

# 默认评估的阈值档位（mag8 域）。第一项是“主阈值”，仿真表只显示它。
THR_DEF = "4,8,12,16,24,32,40,48"
MAIN_THR = 12
MARGIN = 4          # 梯度统计排除的边界像素（RTL 行缓存边界不参与比对）

# INTERFACE.md §4.5 的实测反查表（横竖边、thr -> 最小 ΔY）。脚本不硬编码它，
# 而是用同一条 mag8_field 在阶跃图上反查（与 sim/color_probe.py 的模型一致）；
# 这张表只作为函数自检的断言。
THR_DY_TABLE = {8: 21, 16: 42, 24: 63, 32: 85, 40: 106, 48: 127}


def init_console():
    """Windows 下 Python 默认按控制台代码页（常见 cp936）输出，而本工程在
    VS Code / Git Bash / WSL 里看的是 UTF-8 —— 不强制一下，中文全是乱码。
    输出重定向到文件时也统一成 UTF-8，方便贴进文档。
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            enc = (stream.encoding or "").lower()
            if "utf" not in enc:
                stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass


def die(msg):
    sys.stderr.write("gray_hist: %s\n" % msg)
    sys.exit(2)


def warn(msg):
    sys.stdout.write("  !! %s\n" % msg)


# ---------------------------------------------------------------------
# 1) 读图 / 灰度：与 RTL 逐位同构
# ---------------------------------------------------------------------
def rtl_luma(rgb):
    """rgb_to_gray.v：Y = (77R + 150G + 29B) >> 8（无舍入）

    77+150+29=256，所以 R=G=B 时输出恒等于输入 —— view=3 的 Y 图用这个公式
    算出来与原值逐位一致，不需要单独的“已经是灰度”分支。
    """
    r = rgb[..., 0].astype(np.uint32)
    g = rgb[..., 1].astype(np.uint32)
    b = rgb[..., 2].astype(np.uint32)
    return ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8)


def parse_size(s):
    try:
        w, h = s.lower().split("x")
        w, h = int(w), int(h)
    except Exception:
        die("--raw 需要 WxH，例如 1280x720")
    if w <= 0 or h <= 0:
        die("--raw 尺寸非法：%s" % s)
    return w, h


def load_frame(path, raw_spec=None, raw_fmt="y8"):
    """返回 (Y8 ndarray, info dict)。"""
    if not os.path.isfile(path):
        die("找不到文件：%s" % path)

    if raw_spec:
        w, h = parse_size(raw_spec)
        n = w * h
        if raw_fmt == "y8":
            need = n
            if os.path.getsize(path) < need:
                die("raw 文件只有 %d 字节，Y8 %dx%d 需要 %d"
                    % (os.path.getsize(path), w, h, need))
            y = np.fromfile(path, dtype=np.uint8, count=need).reshape(h, w)
            rgb = None
            mode = "RAW Y8"
        elif raw_fmt == "rgb888":
            need = n * 3
            if os.path.getsize(path) < need:
                die("raw 文件只有 %d 字节，RGB888 %dx%d 需要 %d"
                    % (os.path.getsize(path), w, h, need))
            rgb = np.fromfile(path, dtype=np.uint8, count=need).reshape(h, w, 3)
            y = rtl_luma(rgb)
            mode = "RAW RGB888"
        else:
            die("--fmt 只支持 y8 / rgb888")
        return y, {"mode": mode, "size": (w, h), "rgb": rgb}

    try:
        im = Image.open(path)
        im = ImageOps.exif_transpose(im)
    except Exception as e:
        die("读图失败：%s（%s）" % (path, e))

    mode = im.mode
    if mode in ("I;16", "I;16L", "I;16B", "I"):
        a = np.asarray(im).astype(np.float64)
        mx = float(a.max()) if a.size else 255.0
        if mx > 255.0:
            warn("输入是 16 bit（max=%.0f），按 max 线性压到 8 bit；"
                 "直方图绝对值只能当相对量看" % mx)
            a = a * (255.0 / mx)
        y = np.clip(np.rint(a), 0, 255).astype(np.uint8)
        rgb = None
        mode = "%s -> 8bit" % mode
    else:
        rgb = np.asarray(im.convert("RGB"), dtype=np.uint8)
        y = rtl_luma(rgb)

    return y, {"mode": mode, "size": im.size, "rgb": rgb}


def channel_check(rgb):
    """判断抓的是不是 RTL 的 Y 图（R=G=B），返回 (equal_frac, max_diff)。"""
    if rgb is None:
        return None, None
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    eq = np.mean((r == g) & (g == b))
    md = int(max(np.abs(r.astype(np.int16) - g).max(),
                 np.abs(g.astype(np.int16) - b).max()))
    return float(eq), md


def autocrop(y, max_frac=0.35, black_level=2):
    """裁掉采集卡/显示器加的黑边（每边最多裁 35%）。

    只认“>=99.5% 像素 <= black_level”的连续边行/边列。真实画面顶部本来就是黑的
    会被一起裁掉，直方图影响可忽略；不想要就加 --no-autocrop。
    """
    h, w = y.shape

    def black(line):
        return float(np.mean(line <= black_level)) >= 0.995

    top = 0
    while top < int(h * max_frac) and black(y[top, :]):
        top += 1
    bot = h
    while bot > h - int(h * max_frac) and black(y[bot - 1, :]):
        bot -= 1
    left = 0
    while left < int(w * max_frac) and black(y[:, left]):
        left += 1
    right = w
    while right > w - int(w * max_frac) and black(y[:, right - 1]):
        right -= 1
    if bot - top < 8 or right - left < 8:
        return y, (0, 0, w, h)
    if (top, left, bot, right) == (0, 0, h, w):
        return y, (0, 0, w, h)
    return y[top:bot, left:right], (left, top, right, bot)


def parse_roi(s, h, w):
    try:
        vals = [float(x) for x in s.split(",")]
    except Exception:
        die("--roi 需要 x0,y0,x1,y1（像素或 0..1 归一化）")
    if len(vals) != 4:
        die("--roi 需要 4 个数")
    if all(0.0 <= v <= 1.0 for v in vals):
        x0, y0, x1, y1 = (int(round(vals[0] * w)), int(round(vals[1] * h)),
                          int(round(vals[2] * w)), int(round(vals[3] * h)))
        unit = "归一化"
    else:
        x0, y0, x1, y1 = (int(round(vals[0])), int(round(vals[1])),
                          int(round(vals[2])), int(round(vals[3])))
        unit = "像素"
    x0, x1 = max(0, min(x0, w)), max(0, min(x1, w))
    y0, y1 = max(0, min(y0, h)), max(0, min(y1, h))
    if x1 - x0 < 8 or y1 - y0 < 8:
        die("--roi 太小（至少 8x8）：%s" % s)
    return (x0, y0, x1, y1), unit


# ---------------------------------------------------------------------
# 2) 直方图 / 统计
# ---------------------------------------------------------------------
def hist_stats(v):
    """v: 一维 uint8 数组。返回统计字典。"""
    v64 = v.astype(np.int64)
    n = int(v64.size)
    hist = np.bincount(v64, minlength=256).astype(np.int64)
    p = hist.astype(np.float64) / max(n, 1)
    nz = p[p > 0]
    entropy = float(-(nz * np.log2(nz)).sum())
    q = np.percentile(v64, [0.1, 1, 5, 10, 25, 50, 75, 90, 95, 99, 99.9])
    return {
        "n": n,
        "mean": float(v64.mean()),
        "std": float(v64.std()),
        "min": int(v64.min()),
        "max": int(v64.max()),
        "entropy": entropy,
        "hist": hist,
        "p0_1": float(q[0]), "p1": float(q[1]), "p5": float(q[2]),
        "p10": float(q[3]), "p25": float(q[4]), "p50": float(q[5]),
        "p75": float(q[6]), "p90": float(q[7]), "p95": float(q[8]),
        "p99": float(q[9]), "p99_9": float(q[10]),
        "le1_pct": 100.0 * float(np.mean(v64 <= 1)),
        "eq255_pct": 100.0 * float(np.mean(v64 == 255)),
        "ge254_pct": 100.0 * float(np.mean(v64 >= 254)),
        "lt16_pct": 100.0 * float(np.mean(v64 < 16)),
        "gt240_pct": 100.0 * float(np.mean(v64 > 240)),
        "span_p1_p99": float(q[9] - q[1]),
    }


def print_histogram(stats, width=42):
    hist = stats["hist"]
    total = stats["n"]
    groups = hist.reshape(16, 16).sum(axis=1)
    pcts = groups * 100.0 / max(total, 1)
    mx = float(pcts.max()) if len(pcts) else 0.0
    scale = mx / width if mx > 0 else 1.0
    print("    灰度区间      像素数      占比   分布（每格 16 级，横轴线性）")
    for i in range(16):
        bar = "#" * int(round(pcts[i] / scale)) if scale > 0 else ""
        print("    %3d..%3d  %10d  %7.2f%%   %s"
              % (i * 16, i * 16 + 15, int(groups[i]), pcts[i], bar))


def print_stats(stats):
    print("    均值 %.1f   标准差 %.1f   最小 %d   最大 %d   熵 %.2f/8 bit"
          % (stats["mean"], stats["std"], stats["min"], stats["max"], stats["entropy"]))
    print("    分位  p0.1 %.0f  p1 %.0f  p5 %.0f  p10 %.0f  p25 %.0f  "
          "p50 %.0f  p75 %.0f  p90 %.0f  p95 %.0f  p99 %.0f  p99.9 %.0f"
          % (stats["p0_1"], stats["p1"], stats["p5"], stats["p10"], stats["p25"],
             stats["p50"], stats["p75"], stats["p90"], stats["p95"],
             stats["p99"], stats["p99_9"]))
    print("    削顶  <=1: %.2f%%   =255: %.2f%%   >=254: %.2f%%   "
          "<16: %.2f%%   >240: %.2f%%"
          % (stats["le1_pct"], stats["eq255_pct"], stats["ge254_pct"],
             stats["lt16_pct"], stats["gt240_pct"]))
    print("    有效范围  p1..p99 = %.0f..%.0f（span %.0f = 满幅的 %.0f%%）"
          % (stats["p1"], stats["p99"], stats["span_p1_p99"],
             stats["span_p1_p99"] * 100.0 / 255.0))


# ---------------------------------------------------------------------
# 3) 梯度：与 canny_edge 理想链逐位同构
# ---------------------------------------------------------------------
def mag8_field(y):
    """Y8 -> 3x3 高斯(/16) -> Sobel -> |Gx|+|Gy| -> >>3。

    边界用 edge-pad；统计时排除 MARGIN 像素，所以与 RTL 的行缓存边界差异
    不影响结论。内部像素与 sim/canny_ref.py 的 ref_grad 逐位一致（已对拍）。
    """
    p = np.pad(y.astype(np.int32), 1, mode="edge")
    g = (p[:-2, :-2] + 2 * p[:-2, 1:-1] + p[:-2, 2:] +
         2 * p[1:-1, :-2] + 4 * p[1:-1, 1:-1] + 2 * p[1:-1, 2:] +
         p[2:, :-2] + 2 * p[2:, 1:-1] + p[2:, 2:]) >> 4
    q = np.pad(g, 1, mode="edge")
    gx = ((q[:-2, 2:] + 2 * q[1:-1, 2:] + q[2:, 2:]) -
          (q[:-2, :-2] + 2 * q[1:-1, :-2] + q[2:, :-2]))
    gy = ((q[2:, :-2] + 2 * q[2:, 1:-1] + q[2:, 2:]) -
          (q[:-2, :-2] + 2 * q[:-2, 1:-1] + q[:-2, 2:]))
    return ((np.abs(gx) + np.abs(gy)) >> 3).astype(np.int32)


def grad_metrics(mag, mask, thrs):
    v = mag[mask]
    q = np.percentile(v, [50, 90, 95, 99]) if v.size else [0, 0, 0, 0]
    return {
        "n": int(v.size),
        "p50": float(q[0]), "p90": float(q[1]),
        "p95": float(q[2]), "p99": float(q[3]),
        "max": int(v.max()) if v.size else 0,
        "pass": {int(t): 100.0 * float(np.mean(v >= t)) for t in thrs},
    }


def box_sum(a, k):
    """k x k 邻域和（edge-pad），用积分图，避免 Python 滑窗。"""
    pad = k // 2
    p = np.pad(a.astype(np.float64), pad, mode="edge")
    ii = np.zeros((p.shape[0] + 1, p.shape[1] + 1), dtype=np.float64)
    ii[1:, 1:] = p.cumsum(0).cumsum(1)
    h, w = a.shape
    return (ii[k:k + h, k:k + w] - ii[0:h, k:k + w]
            - ii[k:k + h, 0:w] + ii[0:h, 0:w])


def local_std(y, k=5):
    a = y.astype(np.float64)
    s = box_sum(a, k)
    s2 = box_sum(a * a, k)
    n = float(k * k)
    mean = s / n
    var = np.maximum(s2 / n - mean * mean, 0.0)
    return np.sqrt(var)


def flat_mask_of(y, roi_mask):
    """“平坦区”掩膜：原图 5x5 局部标准差最小的 20%，并把阈值夹在 [1.5, 8]。

    用途是给增强仿真一个**噪声地板**代理：真实噪声地板应该盖镜头量（见文档），
    这里只回答“增强会不会把本来平坦的区域推过弱阈值”。
    """
    std = local_std(y)
    vals = std[roi_mask]
    if vals.size == 0:
        return np.zeros_like(y, dtype=bool), 0.0
    thr = float(np.percentile(vals, 20))
    thr = float(np.clip(thr, 1.5, 8.0))
    return (std <= thr) & roi_mask, thr


# ---------------------------------------------------------------------
# 4) 增强仿真
# ---------------------------------------------------------------------
def parse_chain(s):
    ops = []
    for part in s.split(">"):
        part = part.strip()
        if not part:
            continue
        name, sep, arg = part.partition(":")
        name = name.strip().lower()
        if name not in ("gain", "levels", "gamma", "eq"):
            die("未知增强操作：%s（支持 gain / levels / gamma / eq）" % name)
        if name != "eq" and not sep:
            die("增强操作缺参数：%s（例如 gamma:0.7）" % part)
        ops.append((name, arg.strip()))
    return ops


def apply_chain(y, ops):
    """按顺序执行，每一级都量化回 uint8（模拟真实 LUT 链）。"""
    out = y
    for name, arg in ops:
        if name == "levels":
            if arg.strip().lower() == "auto":
                lo, hi = [float(x) for x in np.percentile(out, [1, 99])]
            else:
                try:
                    lo, hi = [float(x) for x in arg.split(",")]
                except Exception:
                    die("levels 参数格式是 lo,hi 或 auto（例如 levels:16,235）")
            if hi <= lo:
                die("levels 的 hi 必须大于 lo：%s" % arg)
            out = (out.astype(np.float32) - lo) * (255.0 / (hi - lo))
        elif name == "gamma":
            try:
                g = float(arg)
            except Exception:
                die("gamma 参数是正数（例如 gamma:0.7）")
            if g <= 0:
                die("gamma 必须 > 0")
            out = 255.0 * np.power(out.astype(np.float32) / 255.0, g)
        elif name == "gain":
            try:
                k = float(arg)
            except Exception:
                die("gain 参数是正数（例如 gain:1.3）")
            if k <= 0:
                die("gain 必须 > 0")
            out = out.astype(np.float32) * k
        else:  # eq
            hist = np.bincount(out.ravel(), minlength=256).astype(np.float64)
            cdf = np.cumsum(hist)
            if cdf[-1] <= 0:
                continue
            lut = np.clip(np.rint(255.0 * cdf / cdf[-1]), 0, 255).astype(np.uint8)
            out = lut[out]
            continue
        out = np.clip(np.rint(out), 0, 255).astype(np.uint8)
    return out


def variant_metrics(y, roi_mask, flat_mask, thrs):
    st = hist_stats(y[roi_mask])
    mag = mag8_field(y)
    gm = grad_metrics(mag, roi_mask, thrs)
    fm = grad_metrics(mag, flat_mask, thrs)
    return {
        "mean": st["mean"], "p5": st["p5"], "p95": st["p95"],
        "span": st["span_p1_p99"],
        "ge254_pct": st["ge254_pct"], "le1_pct": st["le1_pct"],
        "mag_p90": gm["p90"], "mag_p99": gm["p99"],
        "flat_p95": fm["p95"], "flat_p99": fm["p99"],
        "pass": gm["pass"],
    }


def variant_label(ops):
    if not ops:
        return "原图"
    return ">".join(n + ("" if n == "eq" else ":" + a) for n, a in ops)


# ---------------------------------------------------------------------
# 5) 报告
# ---------------------------------------------------------------------
_STEP_CACHE = {}
_DY_CACHE = {}


def step_mag8(dy):
    """幅度为 dy 的竖边在模型里产生的 mag8 峰值（与 color_probe.step_mag8 同口径）。"""
    dy = int(dy)
    if dy not in _STEP_CACHE:
        w, h = 48, 24
        img = np.zeros((h, w), dtype=np.uint8)
        img[:, w // 2:] = dy
        mag = mag8_field(img)
        _STEP_CACHE[dy] = int(mag[4:h - 4, w // 2 - 3:w // 2 + 4].max())
    return _STEP_CACHE[dy]


def dy_for_thr(thr):
    """横竖边：thr -> 最小 ΔY。直接跑模型反查，不依赖解析近似。

    解析式 mag8 ≈ 3*ΔY/8 会因为 >>4 截断差 1 级，而调阈值恰恰是 1 LSB
    就要命的场合（见 CANNY_PLAN §3.8），所以这里宁可多算几十次。
    """
    thr = int(thr)
    if thr not in _DY_CACHE:
        for dy in range(1, 256):
            if step_mag8(dy) >= thr:
                _DY_CACHE[thr] = dy
                break
        else:
            _DY_CACHE[thr] = 255
    return _DY_CACHE[thr]


def print_gradient(gm, thrs, main_thr, scope):
    print("    mag8（%s，排除边界 %d px）：p50 %.0f  p90 %.0f  p95 %.0f  "
          "p99 %.0f  max %d"
          % (scope, MARGIN, gm["p50"], gm["p90"], gm["p95"], gm["p99"], gm["max"]))
    line = "    过强阈值比例（%s）：" % scope
    line += "  ".join("thr=%d: %.2f%%" % (t, gm["pass"][t]) for t in thrs)
    print(line)
    print("    参考：thr=%d 对应横竖边 ΔY ≥ %d（满幅 %.1f%%）；"
          "对角边约再高 1.45 倍"
          % (main_thr, dy_for_thr(main_thr), dy_for_thr(main_thr) * 100.0 / 255.0))


def print_grid(y, mag, roi, n, main_thr):
    x0, y0, x1, y1 = roi
    print("    区域均值 / p5..p95 / 削顶%% / thr=%d 过阈%%（%dx%d 网格）"
          % (main_thr, n, n))
    cw = (x1 - x0) / n
    ch = (y1 - y0) / n
    header = "        " + "".join("%22s" % ("列%d" % c) for c in range(n))
    print(header)
    for r in range(n):
        cells = []
        for c in range(n):
            cx0 = int(round(x0 + c * cw))
            cx1 = int(round(x0 + (c + 1) * cw))
            cy0 = int(round(y0 + r * ch))
            cy1 = int(round(y0 + (r + 1) * ch))
            sub = y[cy0:cy1, cx0:cx1]
            mg = mag[cy0:cy1, cx0:cx1]
            if sub.size == 0:
                cells.append(" " * 22)
                continue
            p5, p95 = np.percentile(sub, [5, 95])
            cells.append("%5.0f %3.0f..%3.0f %5.1f%% %5.2f%%"
                         % (sub.mean(), p5, p95,
                            100.0 * np.mean(sub >= 254),
                            100.0 * np.mean(mg >= main_thr)))
        print("    行%d  " % r + "".join("%22s" % c for c in cells))


def print_sim_table(sims, thrs, main_thr):
    print("    %-30s %6s %5s %5s %5s %8s %8s %8s %9s %8s"
          % ("方案", "mean", "p5", "p95", "span", ">=254%", "mag8p90",
             "mag8p99", "平坦p95", "过阈%"))
    print("    " + "-" * 105)
    for s in sims:
        print("    %-30s %6.1f %5.0f %5.0f %5.0f %7.2f%% %8.0f %8.0f "
              "%9.1f %7.2f%%"
              % (s["label"], s["mean"], s["p5"], s["p95"], s["span"],
                 s["ge254_pct"], s["mag_p90"], s["mag_p99"],
                 s["flat_p95"], s["pass"][main_thr]))
    print("    过阈%% = thr=%d 下 ROI 内 mag8 达标的像素比例；"
          "平坦p95 = 原图最平坦 20%% 区域增强后的 mag8 p95" % main_thr)
    print("    弱阈值 thr_low = thr_high/2 = %d；平坦p95 接近或超过它时，"
          "平坦区会开始出现弱边（NMS/滞后后未必可见，但余量已变薄）"
          % max(1, main_thr // 2))


def verdict(stats, gm, sims, main_thr, roi_label, grid_used):
    """给出“要不要增强、先修什么”的保守结论。"""
    lines = []
    hi = stats["ge254_pct"]
    p1, p50, p99 = stats["p1"], stats["p50"], stats["p99"]

    # 1) 过曝
    if hi > 0.5 or (stats["p99_9"] >= 254 and p99 >= 245):
        lines.append("高光削顶：>=254 的像素占 %.2f%%，p99=%.0f。"
                     "已经到 255 的层次任何数字增强都救不回来。" % (hi, p99))
        lines.append("  先降采集曝光/增益（I2C_SC1336_1280720_Config.v 的 "
                     "3e01/3e02/3e08），再做任何 LUT；否则增强只会把更多像素推进削顶区。")
    # 2) 欠曝 / 余量
    if p99 < 170 and hi < 0.5:
        lines.append("上端余量：p99=%.0f，距 255 还有 %.0f 级（满幅的 %.0f%% 未用）。"
                     % (p99, 255 - p99, (255 - p99) * 100.0 / 255.0))
    if p50 < 90:
        lines.append("整体偏暗：中位数 p50=%.0f。整体增益或提暗部的 gamma 有明确收益。"
                     % p50)
    # 3) 黑位。注意：重度过曝的画面 p1 也会很高（整幅都亮），那不是“发灰”，
    #    所以必须先排除削顶，否则会给出“用过曝图去拉黑场”这种反向建议。
    if p1 > 32 and hi < 0.5 and p99 < 245:
        lines.append("黑位抬高：p1=%.0f > 32，画面发灰/低对比。"
                     "优先用 levels 把黑场压回去，而不是整体 gain。" % p1)
    # 4) 范围已经用满
    if p1 < 8 and p99 > 220 and hi < 0.5:
        lines.append("动态范围已基本用满（p1=%.0f, p99=%.0f, 削顶 %.2f%%）："
                     "全局增强的收益有限，问题更可能在局部对比度、光照或对焦。"
                     % (p1, p99, hi))
    # 5) 梯度余量
    lines.append("thr=%d（弱边 %d）下 ROI 过阈比例 %.2f%%；"
                 "横竖边要 ΔY ≥ %d。若人脸轮廓所在 ROI 的比例明显低于全幅，"
                 "说明是**局部**欠曝/低对比，优先补光或做 ROI 相关处理，"
                 "不要用全局增益硬拉。"
                 % (main_thr, max(1, main_thr // 2), gm["pass"][main_thr],
                    dy_for_thr(main_thr)))

    # 6) 仿真对比
    if len(sims) > 1:
        base = sims[0]
        safe = []
        for s in sims[1:]:
            dclip = s["ge254_pct"] - base["ge254_pct"]
            noise_ok = s["flat_p95"] < max(4.0, main_thr // 2)
            if dclip <= 0.5 and noise_ok:
                safe.append(s)
        best = max(safe, key=lambda s: s["pass"][main_thr]) if safe else None
        gain = (best["pass"][main_thr] - base["pass"][main_thr]) if best else 0.0
        # 收益门槛：至少 +0.1 个百分点，且相对提升 >=10%。否则“推荐”没有意义
        # （噪声/量化抖动都能造成这个量级的变化）。
        meaningful = (best is not None and
                      gain >= max(0.1, 0.10 * base["pass"][main_thr]))
        if meaningful:
            lines.append("保守候选：%s —— ROI 过阈 %.2f%% -> %.2f%%（%+.2f 个百分点），"
                         "削顶增加 %.2f 个百分点，平坦区 p95 %.0f -> %.0f。"
                         % (best["label"], base["pass"][main_thr],
                            best["pass"][main_thr], gain,
                            best["ge254_pct"] - base["ge254_pct"],
                            base["flat_p95"], best["flat_p95"]))
        elif best is not None:
            lines.append("仿真里没有哪个增强方案能在“不明显增加削顶、不把平坦区噪声"
                         "推到弱阈值附近”的前提下，把 ROI 过阈比例实质抬高"
                         "（最好的 %s 只有 %+.2f 个百分点）—— 全局灰度增强对这个"
                         "场景帮助有限，先解决曝光/补光/对焦。"
                         % (best["label"], gain))
        else:
            lines.append("所有仿真方案都不满足“削顶增加 <=0.5%% 且平坦区 p95 < 弱阈值”"
                         "这条保守判据：先修曝光/补光，再考虑增强。")
        risky = [s for s in sims[1:]
                 if s["ge254_pct"] - base["ge254_pct"] > 0.5]
        if risky:
            lines.append("会把高光明显推爆的方案：%s。正常光下也许可用，"
                         "强光下不要上。"
                         % "、".join(s["label"] for s in risky))

    # 7) 实现建议
    lines.append("实现位置：rgb_to_gray 之后、pix_window_3x3 之前对 Y 做 256x8 LUT。"
                 "同步读 BRAM LUT 会多 1 拍，需把 image_pipeline_top 的 LATENCY "
                 "16->17，并同步 example_top 的 hsync 延迟（IMG_LAT）。")
    lines.append("根因提醒：SC1336 的曝光/增益是开机写死的固定值，全工程没有 AE 环。"
                 "数字增强只能在“没有削顶、有余量”时补对比度；"
                 "正常光偏暗 + 强光过曝这种跨场景问题，最终仍应做 sensor 侧曝光控制。")
    if grid_used:
        lines.append("网格表用于判断空间不均匀：若某几格明显更暗/过阈比例更低，"
                     "先怀疑光照、镜头暗角或对焦，而不是全局直方图。")
    return lines


# ---------------------------------------------------------------------
# 6) 可选绘图（不依赖 matplotlib）
# ---------------------------------------------------------------------
def save_plot(path, name, stats, mag_vals, thrs, main_thr):
    """两个面板都用同一 ROI：灰度直方图 + mag8 直方图（阈值线）。"""
    W, H = 960, 560
    img = Image.new("RGB", (W, H), (255, 255, 255))
    d = ImageDraw.Draw(img)
    d.text((12, 10), "gray_hist: %s" % name, fill=(0, 0, 0))

    # 面板 1：灰度直方图（sqrt 刻度，避免削顶峰淹没主体）
    x0, y0, x1, y1 = 60, 50, W - 30, 280
    d.rectangle([x0, y0, x1, y1], outline=(0, 0, 0))
    hist = stats["hist"].astype(np.float64)
    mx = math.sqrt(max(hist.max(), 1.0))
    bw = (x1 - x0) / 256.0
    for i in range(256):
        hgt = (math.sqrt(hist[i]) / mx) * (y1 - y0 - 4)
        if hgt <= 0:
            continue
        d.line([x0 + i * bw, y1, x0 + i * bw, y1 - hgt], fill=(70, 90, 160))
    for val, col, lab in ((stats["p1"], (0, 120, 0), "p1"),
                          (stats["p50"], (0, 0, 0), "p50"),
                          (stats["p99"], (200, 0, 0), "p99")):
        xx = x0 + val * bw
        d.line([xx, y0, xx, y1], fill=col)
        d.text((min(xx + 2, x1 - 18), y0 + 2), lab, fill=col)
    for g in range(0, 256, 32):
        xx = x0 + g * bw
        d.line([xx, y1, xx, y1 + 4], fill=(0, 0, 0))
        d.text((xx - 6, y1 + 6), str(g), fill=(0, 0, 0))
    d.text((x0, y1 + 20), "Y (RTL formula), sqrt scale", fill=(0, 0, 0))

    # 面板 2：mag8（Canny 判据域）
    x0, y0, x1, y1 = 60, 330, W - 30, 520
    d.rectangle([x0, y0, x1, y1], outline=(0, 0, 0))
    h2 = np.bincount(mag_vals.ravel(), minlength=256).astype(np.float64)
    h2 = h2[:161]  # 0..160，>160 罕见（对角满量程约 140）
    mx2 = math.sqrt(max(h2.max(), 1.0))
    bw2 = (x1 - x0) / 161.0
    for i in range(161):
        hgt = (math.sqrt(h2[i]) / mx2) * (y1 - y0 - 4)
        if hgt <= 0:
            continue
        d.line([x0 + i * bw2, y1, x0 + i * bw2, y1 - hgt], fill=(160, 90, 40))
    for t in thrs:
        if t > 160:
            continue
        xx = x0 + t * bw2
        col = (200, 0, 0) if t == main_thr else (150, 150, 150)
        d.line([xx, y0, xx, y1], fill=col)
        d.text((min(xx + 2, x1 - 24), y0 + 2), "t%d" % t, fill=col)
    for g in range(0, 161, 20):
        xx = x0 + g * bw2
        d.line([xx, y1, xx, y1 + 4], fill=(0, 0, 0))
        d.text((xx - 6, y1 + 6), str(g), fill=(0, 0, 0))
    d.text((x0, y1 + 20), "mag8 = (|Gx|+|Gy|)>>3, sqrt scale", fill=(0, 0, 0))

    img.save(path)


# ---------------------------------------------------------------------
# 7) 主流程
# ---------------------------------------------------------------------
def analyze_one(path, args, thrs, main_thr):
    y_full, info = load_frame(path, args.raw, args.fmt)
    h, w = y_full.shape
    eq_frac, max_diff = channel_check(info["rgb"])

    if args.no_autocrop:
        y, crop = y_full, (0, 0, w, h)
    else:
        y, crop = autocrop(y_full)
    h2, w2 = y.shape

    if args.roi:
        roi, unit = parse_roi(args.roi, h2, w2)
        roi_label = "ROI %d,%d,%d,%d（%s，相对裁边后）" % (roi + (unit,))
    else:
        roi = (0, 0, w2, h2)
        roi_label = "全幅 %dx%d" % (w2, h2)
    x0, y0, x1, y1 = roi
    roi_mask = np.zeros((h2, w2), dtype=bool)
    roi_mask[y0:y1, x0:x1] = True
    # 梯度统计再排除边界
    inner = np.zeros_like(roi_mask)
    inner[max(y0, MARGIN):max(y0, y1 - MARGIN),
          max(x0, MARGIN):max(x1 - MARGIN, x0)] = True
    roi_inner = roi_mask & inner
    if not roi_inner.any():
        roi_inner = roi_mask

    print()
    print("=" * 96)
    print("gray_hist  |  %s" % path)
    print("=" * 96)
    print("[0] 输入")
    print("    格式 %s   原尺寸 %dx%d   载入后 %dx%d"
          % (info["mode"], info["size"][0], info["size"][1], w, h))
    if crop != (0, 0, w, h):
        print("    自动裁边：left %d, top %d, right %d, bottom %d -> %dx%d"
              % (crop[0], crop[1], crop[2], crop[3], w2, h2))
    print("    %s" % roi_label)
    if eq_frac is not None:
        if eq_frac >= 0.999 and max_diff <= 1:
            print("    通道相等：是（R=G=B）—— 这是 RTL 的 Y 图（view=3）抓图，最可信")
        else:
            print("    通道相等：否（相等像素 %.2f%%，最大通道差 %d）—— 按 RTL 公式算 Y"
                  % (eq_frac * 100.0, max_diff))
    lo, hi = np.percentile(y, [0.1, 99.9])
    # 只在“内容本来应该铺满、却刚好卡在 16..235”时警告。暗场景的 hi 天然很低，
    # 不能因为 hi < 235 就报 limited range（第一版就这么误报过）。
    if lo >= 13 and 228 <= hi <= 243 and (hi - lo) >= 190:
        warn("疑似 HDMI limited range（p0.1=%.0f, p99.9=%.0f，卡在 16..235 附近）："
             "采集端请设 Full/PC range，否则直方图两端被压缩" % (lo, hi))
    if path.lower().endswith((".jpg", ".jpeg")):
        warn("JPEG 是有损压缩，8x8 DCT 会改变局部梯度；请改用无损 PNG/BMP 重抓")
    if abs(w2 - 1280) > 8 or abs(h2 - 720) > 8:
        warn("有效画面不是 1280x720（%dx%d）：若采集卡做了缩放，局部梯度会被"
             "重采样改变，mag8 统计只能当相对参考" % (w2, h2))

    stats = hist_stats(y[roi_mask])
    mag = mag8_field(y)
    gm = grad_metrics(mag, roi_inner, thrs)
    flat_mask, flat_thr = flat_mask_of(y, roi_inner)
    fm = grad_metrics(mag, flat_mask, thrs)

    print("[1] 灰度直方图（%s，%d 像素）" % (roi_label, stats["n"]))
    print_histogram(stats)
    print_stats(stats)

    print("[2] 空间分布")
    if args.grid:
        print_grid(y, mag, roi, args.grid, main_thr)
    else:
        print("    未启用网格；加 --grid 3 可看 3x3 区域的亮度/过阈比例差异")

    print("[3] 梯度 / Canny 可用性（RTL 同构：高斯 -> Sobel -> mag>>3）")
    print_gradient(gm, thrs, main_thr, "ROI" if args.roi else "全幅")
    print("    平坦区代理（原图 5x5 局部 std <= %.1f，占 ROI %.0f%%）："
          "mag8 p50 %.0f  p95 %.0f  p99 %.0f"
          % (flat_thr, 100.0 * float(flat_mask.sum()) / max(int(roi_inner.sum()), 1),
             fm["p50"], fm["p95"], fm["p99"]))
    print("    （真实噪声地板应盖镜头抓一帧来量；这里只用最平坦像素做相对比较）")

    # 仿真：原图 + 用户方案
    sims = []
    base = {
        "label": "原图",
        "mean": stats["mean"], "p5": stats["p5"], "p95": stats["p95"],
        "span": stats["span_p1_p99"], "ge254_pct": stats["ge254_pct"],
        "le1_pct": stats["le1_pct"],
        "mag_p90": gm["p90"], "mag_p99": gm["p99"],
        "flat_p95": fm["p95"], "flat_p99": fm["p99"],
        "pass": gm["pass"],
    }
    sims.append(base)
    for chain in args.sim:
        ops = parse_chain(chain)
        y2 = apply_chain(y, ops)
        vm = variant_metrics(y2, roi_inner, flat_mask, thrs)
        vm["label"] = variant_label(ops)
        sims.append(vm)

    print("[4] 增强仿真（ROI 内，逐级量化回 8 bit）")
    print_sim_table(sims, thrs, main_thr)

    print("[5] 结论")
    for line in verdict(stats, gm, sims, main_thr, roi_label, bool(args.grid)):
        print("    " + line)

    if args.plot:
        base_name = os.path.splitext(os.path.basename(path))[0]
        out = args.plot
        if len(args.images) > 1 and not out.lower().endswith(".png"):
            out = "%s_%s.png" % (out, base_name)
        elif len(args.images) > 1:
            root, ext = os.path.splitext(out)
            out = "%s_%s%s" % (root, base_name, ext)
        save_plot(out, os.path.basename(path), stats, mag[roi_mask], thrs, main_thr)
        print("    直方图已保存：%s" % out)

    result = {
        "file": os.path.abspath(path),
        "input": {"mode": info["mode"], "size": list(info["size"]),
                  "channels_equal": None if eq_frac is None else bool(eq_frac >= 0.999 and max_diff <= 1)},
        "crop": list(crop),
        "roi": list(roi),
        "stats": {k: v for k, v in stats.items() if k != "hist"},
        "gradient": gm,
        "flat": {"threshold": flat_thr,
                 "coverage_pct": 100.0 * float(flat_mask.sum()) / max(int(roi_inner.sum()), 1),
                 "p50": fm["p50"], "p95": fm["p95"], "p99": fm["p99"]},
        "variants": sims,
    }
    return result


def main():
    init_console()
    ap = argparse.ArgumentParser(
        description="原始灰度分布 / Canny 阈值余量 / 灰度增强收益评估（离线）")
    ap.add_argument("images", nargs="+", help="PNG/BMP（推荐）或 raw 文件")
    ap.add_argument("--raw", metavar="WxH", help="按原始裸数据读取，如 1280x720")
    ap.add_argument("--fmt", choices=("y8", "rgb888"), default="y8",
                    help="raw 格式（默认 y8）")
    ap.add_argument("--roi", help="x0,y0,x1,y1（像素或 0..1 归一化）")
    ap.add_argument("--grid", type=int, default=0, metavar="N",
                    help="打印 NxN 空间分布表（如 3）")
    ap.add_argument("--thr", default=THR_DEF,
                    help="评估的 mag8 阈值，逗号分隔（默认 %s；第一项为主阈值）" % THR_DEF)
    ap.add_argument("--sim", action="append", default=[],
                    help="增强方案，可多次给；用 > 串联，如 \"levels:16,235>gamma:0.7\"")
    ap.add_argument("--plot", metavar="PNG", help="保存直方图（多图时自动加文件名后缀）")
    ap.add_argument("--json", metavar="FILE", help="把统计结果写成 JSON")
    ap.add_argument("--no-autocrop", action="store_true", help="不自动裁黑边")
    args = ap.parse_args()

    if args.raw and len(args.images) > 1:
        die("--raw 一次只能处理一个文件")

    thrs = []
    for s in args.thr.split(","):
        s = s.strip()
        if not s:
            continue
        try:
            t = int(s)
        except Exception:
            die("--thr 里有非整数：%s" % s)
        if not 1 <= t <= 255:
            die("--thr 必须在 1..255：%d" % t)
        thrs.append(t)
    if not thrs:
        die("--thr 不能为空")
    for t in thrs:
        if t > 96:
            warn("thr=%d 超出横竖边的有效量程 0..96（对角边约到 140），"
                 "这一档很可能永远不达标（见 INTERFACE.md §4.5）" % t)
    main_thr = thrs[0]

    if args.grid and args.grid < 1:
        die("--grid 必须 >= 1")

    results = [analyze_one(p, args, thrs, main_thr) for p in args.images]

    # 多图横向对比
    if len(results) > 1:
        print()
        print("=" * 96)
        print("横向对比（同一口径：裁边后 ROI，thr=%d）" % main_thr)
        print("=" * 96)
        print("%-34s %6s %5s %5s %5s %8s %8s %9s %8s"
              % ("文件", "mean", "p5", "p50", "p99", ">=254%", "mag8p90",
                 "平坦p95", "过阈%"))
        print("-" * 105)
        for r in results:
            s = r["stats"]
            print("%-34s %6.1f %5.0f %5.0f %5.0f %7.2f%% %8.0f %9.1f %7.2f%%"
                  % (os.path.basename(r["file"]), s["mean"], s["p5"], s["p50"],
                     s["p99"], s["ge254_pct"], r["gradient"]["p90"],
                     r["flat"]["p95"], r["gradient"]["pass"][main_thr]))
        print("提示：正常光 / 强光 / 盖镜头 三张一起跑，最容易看出“曝光问题”"
              "和“局部对比度问题”的边界。")

    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(results, f, ensure_ascii=False, indent=2)
        print("\n统计结果已写入：%s" % args.json)

    return 0


if __name__ == "__main__":
    sys.exit(main())
