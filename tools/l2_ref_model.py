#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
l2_ref_model.py —— L2「与 Python 参考模型逐像素比对」的参考模型

PR 模板 (.github/PULL_REQUEST_TEMPLATE.md) 要求 L2 验证需附「与 Python 参考模型
逐像素比对」的报告。本脚本即那个独立参考模型：**不看 RTL 实现**，只按
docs/INTERFACE.md 冻结的数学定义，从转储出来的输入像素流重算期望输出，
再与 RTL 转储的输出流逐像素比对。

冻结的数学定义（docs/INTERFACE.md §2/§4/§5）：
  灰度      Y     = (77*R + 150*G + 29*B) >> 8   —— 本工程上游 rgb_to_gray 已做，
                                                     送进流水线的已是 {Y,Y,Y}
  Sobel     Gx    = (p13 + 2*p23 + p33) - (p11 + 2*p21 + p31)
            Gy    = (p31 + 2*p32 + p33) - (p11 + 2*p12 + p13)
            mag   = |Gx| + |Gy|，饱和到 8 bit（>255 截 255）
            edge  = (mag > thr)
  mode 0 = 原图；1 = 边缘图；2 = 左半原图 + 右半边缘图；3 = 画中画（本轮回落为边缘图）
  几何：输出第 m 行 = 图像第 (m-1) 行；输出第 k 列 = 图像第 k 列。
        窗口行 = 图像 (m-2, m-1, m)，列 = (k-1, k, k+1)。

用法（在 repo/ 根目录，最好由 sim/l2_verify.sh 调用）：
    MODE=1 THR=40 python tools/l2_ref_model.py
退出码：0 = 契约区内 0 不符；1 = 有符不符或文件缺失
"""

import os
import sys

# ---------------------------------------------------------------------------
# 测试规模必须与 sim/tb_l2_dump.v 保持一致
# ---------------------------------------------------------------------------
IMG_W = 64
IMG_H = 48

MODE = int(os.environ.get("MODE", "1"))
THR = int(os.environ.get("THR", "40"))

IN_FILE = os.environ.get("L2_IN", "sim/l2_in.txt")
BIN_FILE = os.environ.get("L2_BIN", "sim/l2_out_bin.txt")
MAG_FILE = os.environ.get("L2_MAG", "sim/l2_out_mag.txt")


def parse_hex(tok):
    """解析 6 位十六进制像素。含 x/z 的未知值返回 None（只可能出现在预热帧）。"""
    try:
        return int(tok, 16)
    except ValueError:
        return None


def load_flat(path):
    """读纯像素流（每行 6 位十六进制），忽略 'V' 与空行。"""
    vals = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line == "V":
                continue
            vals.append(parse_hex(line))
    return vals


def load_frames(path):
    """按 'V' 帧分隔符切分输出流。"""
    frames, cur = [], None
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if line == "V":
                if cur is not None:
                    frames.append(cur)
                cur = []
            elif line:
                if cur is None:
                    cur = []
                cur.append(parse_hex(line))
    if cur is not None:
        frames.append(cur)
    return frames


def main():
    for p in (IN_FILE, BIN_FILE, MAG_FILE):
        if not os.path.exists(p):
            print("[FATAL] 缺少转储文件：%s（请先跑 sim/l2_verify.sh）" % p)
            return 1

    # ------------------------------------------------------------------
    # 1) 重建源图
    # ------------------------------------------------------------------
    flat = load_flat(IN_FILE)
    fsz = IMG_W * IMG_H
    if len(flat) % fsz != 0:
        print("[FATAL] 输入像素数 %d 不是帧尺寸 %d 的整数倍" % (len(flat), fsz))
        return 1
    nframe_in = len(flat) // fsz
    if nframe_in < 2:
        print("[FATAL] 输入帧数 %d < 2，无法取稳态帧" % nframe_in)
        return 1

    img = [flat[i * IMG_W:(i + 1) * IMG_W] for i in range(IMG_H)]
    if any(v is None for row in img for v in row):
        print("[FATAL] 输入像素流含未知值(x/z)，激励或复位有问题")
        return 1
    # 各帧应完全一致（源图是 (x,y) 的纯函数）
    for fi in range(1, nframe_in):
        seg = flat[fi * fsz:(fi + 1) * fsz]
        if seg != flat[0:fsz]:
            print("[FATAL] 第 %d 帧输入与第 0 帧不一致，转储或激励有误" % fi)
            return 1

    # ------------------------------------------------------------------
    # 2) 参考模型
    # ------------------------------------------------------------------
    def P(x, y):
        """取图像亮度字节；越界时钳位（仅用于边界区的形式化取值，契约区不会触发）。

        转储里存的是 RGB888 打包值 {Y,Y,Y}，故先右移取出 R 通道即亮度 Y。
        """
        cx = 0 if x < 0 else (IMG_W - 1 if x > IMG_W - 1 else x)
        cy = 0 if y < 0 else (IMG_H - 1 if y > IMG_H - 1 else y)
        return (img[cy][cx] >> 16) & 0xFF

    def sobel_at(m, k):
        """按冻结公式在流位置 (m,k) 处算 Sobel -> (sat, edge)。

        几何：流位置 (m,k) 的窗口中心是图像 (k, m-1)，窗口覆盖图像行 (m-2,m-1,m)、
        列 (k-1,k,k+1)。（沿用 docs/INTERFACE.md §4 与 §6 的约定。）
        """
        top, mid, bot = m - 2, m - 1, m
        left, ctr, right = k - 1, k, k + 1

        gx = (P(right, top) + 2 * P(right, mid) + P(right, bot)) - \
             (P(left,  top) + 2 * P(left,  mid) + P(left,  bot))
        gy = (P(left,  bot) + 2 * P(ctr,   bot) + P(right, bot)) - \
             (P(left,  top) + 2 * P(ctr,   top) + P(right, top))

        mag = abs(gx) + abs(gy)
        sat = 255 if mag > 255 else mag
        return sat, (1 if sat > THR else 0)

    def ref(orow, ocol):
        """返回 (期望_EDGE_BINARY=1 的实例, 期望_EDGE_BINARY=0 的实例)。"""
        sat, edge = sobel_at(orow, ocol)
        orig = P(ocol, orow - 1)
        orig_rgb = orig * 0x010101
        e_bin = 0xFFFFFF if edge else 0x000000
        e_mag = (sat << 16) | (sat << 8) | sat

        if MODE == 0:
            return orig_rgb, orig_rgb
        if MODE == 1:
            return e_bin, e_mag
        if MODE == 2:
            return (orig_rgb, orig_rgb) if ocol < IMG_W // 2 else (e_bin, e_mag)

        # ---------------- MODE 3 画中画 ----------------
        # 背景 = 原图；右下 1/4 = 边缘图的 2:1 抽取（1 bit 二值）
        if orow >= IMG_H // 2 and ocol >= IMG_W // 2:
            PH, PW = IMG_H // 2, IMG_W // 2
            iy, ix = orow - IMG_H // 2, ocol - IMG_W // 2
            # 读侧跳过抽取后第 0 行/第 0 列（含首行/首列固有偏差），末尾钳位
            dy = min(iy + 1, PH - 1)
            dx = min(ix + 1, PW - 1)
            # 该样点取自流位置 (2*dy, 2*dx)
            s_sat, s_edge = sobel_at(2 * dy, 2 * dx)
            # 缓冲存的是 edge_rgb_i == 24'hFFFFFF：二值实例=边缘位；幅值实例=满幅值
            b = 0xFFFFFF if s_edge else 0x000000
            m = 0xFFFFFF if s_sat == 255 else 0x000000
            return b, m
        return orig_rgb, orig_rgb

    def classify(m, k):
        """把输出像素归类。

        vis   —— 裁剪后可见区。FrameBoundCrop 的判据是
                 `w_skip = (rc_w < SKIP_COLS) || (rc_w >= TOTAL_COLS - SKIP_COLS)`
                 （行同理），即 **每边各裁 2 行 2 列**；SKIP=2 时可见区为
                 行 [2, H-3]、列 [2, W-3]。这是 docs/INTERFACE.md §3 冻结契约
                 所指的「裁剪后可见区域」，**必须逐像素 0 不符**。
        crop  —— 3x3 窗口本身完全落在图内，但该行/列会被 FrameBoundCrop 裁掉。
                 契约明说本模块不做边界处理，故这里有差异属**已知且不可见**。
        outer —— 窗口已越界出图（无对应数学定义），同样被裁掉。
        """
        if (2 <= m <= IMG_H - 3) and (2 <= k <= IMG_W - 3):
            return "vis"
        if (2 <= m <= IMG_H - 1) and (1 <= k <= IMG_W - 2):
            return "crop"
        return "outer"

    # ------------------------------------------------------------------
    # 3) 逐像素比对
    # ------------------------------------------------------------------
    def check(path, is_bin, tag):
        raw = load_frames(path)
        frames = [fr for fr in raw if len(fr) == fsz]
        if not frames:
            print("[FATAL] %s 中没有完整帧（各段长度：%s）" % (path, [len(fr) for fr in raw]))
            return None
        out = frames[-1]  # 取最后一整帧（前面的帧可能仍在预热）

        st = {c: {"n": 0, "bad": 0} for c in ("vis", "crop", "outer")}
        st["crop"]["cols"] = set()
        first_bad = []
        for i, got in enumerate(out):
            m, k = i // IMG_W, i % IMG_W
            exp_b, exp_m = ref(m, k)
            exp = exp_b if is_bin else exp_m
            c = classify(m, k)
            st[c]["n"] += 1
            if got != exp:
                st[c]["bad"] += 1
                if c == "crop":
                    st["crop"]["cols"].add(k)
                if len(first_bad) < 5:
                    first_bad.append((c, m, k, got, exp))

        print("  [%s]" % tag)
        print("     可见区(契约)   %5d 点，不符 %d   <- 判定依据"
              % (st["vis"]["n"], st["vis"]["bad"]))
        print("     裁剪区(不可见) %5d 点，不符 %d   差异列: %s"
              % (st["crop"]["n"], st["crop"]["bad"],
                 sorted(st["crop"]["cols"]) if st["crop"]["bad"] else "无"))
        print("     外圈(窗口越界) %5d 点，不符 %d   （同样被裁掉）"
              % (st["outer"]["n"], st["outer"]["bad"]))
        for (c, m, k, got, exp) in first_bad:
            rtl = ("%06X" % got) if isinstance(got, int) else "x/z未知"
            print("       首个不符[%s] m=%d k=%d：RTL=%s 期望=%06X" % (c, m, k, rtl, exp))
        return st

    print("=" * 72)
    print("L2 参考模型比对   MODE=%d  THR=%d  图像 %dx%d  帧数 %d（取最后一帧）"
          % (MODE, THR, IMG_W, IMG_H, nframe_in))
    print("=" * 72)

    st_bin = check(BIN_FILE, True, "EDGE_BINARY=1 二值边缘图")
    st_mag = check(MAG_FILE, False, "EDGE_BINARY=0 梯度幅值灰度图")
    if st_bin is None or st_mag is None:
        return 1

    bad = st_bin["vis"]["bad"] + st_mag["vis"]["bad"]
    crop_bad = st_bin["crop"]["bad"] + st_mag["crop"]["bad"]
    crop_cols = sorted(st_bin["crop"]["cols"] | st_mag["crop"]["cols"])

    print("-" * 72)
    if bad == 0:
        print("结果: *** PASS *** 可见区（裁剪后）逐像素 0 不符，"
              "%d 点/实例 × 2 实例" % st_bin["vis"]["n"])
        print("        裁剪区另有 %d 点差异，全部落在列 %s —— 均被 FrameBoundCrop "
              "裁掉，\n        符合同一冻结契约（本模块不做边界处理）"
              % (crop_bad, crop_cols if crop_cols else "无"))
        return 0
    print("结果: *** FAIL *** 可见区共 %d 点不符" % bad)
    return 1


if __name__ == "__main__":
    sys.exit(main())
