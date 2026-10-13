#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
canny_edge 参考模型 + 回归驱动（当前对应 M2：高斯 + 梯度 + 幅值 + 方向量化 + NMS）

参考模型刻意用**与 RTL 完全相同的近似**（方案 §8.1 的要求），否则比对必然不一致：
    g   = (p11 + 2p12 + p13 + 2p21 + 4p22 + 2p23 + p31 + 2p32 + p33) >> 4
    Gx  = (g13 + 2g23 + g33) - (g11 + 2g21 + g31)
    Gy  = (g31 + 2g32 + g33) - (g11 + 2g12 + g13)
    mag = |Gx| + |Gy|（内部 12bit 不饱和，输出饱和到 8bit）
    dir = |Gy|*256 <  |Gx|*106            → 0（0°，梯度近水平）
          |Gx|*106 <  |Gy|*256            → 1（90°，梯度近竖直）
          否则 Gx、Gy 同号 → 2（45°）／异号 → 3（135°）
    NMS = mag 中心 >= 沿 dir 的两个邻居（平局保留）
    edge = NMS 通过 且 饱和幅值 > thr
不做双阈值滞后——那是 M3 的事。

比对方式：参考模型给出"中心落在 (r,c) 的理想值"，RTL 输出与它之间只差一个固定的
整体偏移 (EXP_DR, EXP_DC)（流水线固有的行列偏移，写成常量）。比对区留 5~6 圈边，
避开行缓存边界口径和帧首暂态，要求**区域内逐像素相等**——失配即失败。
★ 偏移不能"每次搜出来"：周期性图样在任何平移下都自洽，搜出来的最优偏移会退化成
"少比几个点"（实测过：checker 会搜到 (-8,-16) 且 0 失配），那样回归就失去意义。
失败时会额外打印"自由搜索得到的最佳偏移"作为诊断，用来看是整体错位还是逐点算错。

用法（见 sim/run.sh）：
    python3 sim/canny_ref.py --exe sim/build/obj_dir/Vtb_canny_edge
"""

import argparse
import os
import random
import re
import subprocess
import sys

THR = 40        # 与 tb_canny_edge.v / example_top 的默认阈值一致

# M2 流水线的固有空间偏移：RTL 输出 (r,c) 上的像素 = 理想参考 (r+DR, c+DC)。
# 由窗口链推出：pix_window_3x3 的中心偏 (-1,0) → canny 内的高斯窗口再 (-1,0)
# → 梯度窗口再 (-1,0)；列方向两级的读地址都是预测式（col+1），列对齐。
# ★ 写成常量而不是"每次搜出来"：周期性图样（thin/checker/step_*）本身在任何平移下
#   都自洽，搜出来的偏移会退化成"少比几个点"，那样就失去回归意义了。
EXP_DR, EXP_DC = -3, 0


# ---------------------------------------------------------------------
# 工具
# ---------------------------------------------------------------------
def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def gauss_at(img, W, H, r, c):
    """3x3 高斯 [1 2 1; 2 4 2; 1 2 1]/16，边界复制（边界值不参与比对，见 mask）"""
    def px(dr, dc):
        return img[clamp(r + dr, 0, H - 1)][clamp(c + dc, 0, W - 1)]
    s = (px(-1, -1) + 2 * px(-1, 0) + px(-1, 1) +
         2 * px(0, -1) + 4 * px(0, 0) + 2 * px(0, 1) +
         px(1, -1) + 2 * px(1, 0) + px(1, 1))
    return s >> 4


def ref_grad(img, W, H):
    """理想梯度场：返回 (mag, dir) 两个二维表，中心落在 (r,c)

    mag : |Gx| + |Gy|，未饱和（0..2040）
    dir : 方向扇区，与 RTL 的 S8 同一判据
          0 = 0°（梯度近水平）1 = 90°（近竖直）2 = 45°（Gx、Gy 同号）3 = 135°
    """
    g = [[gauss_at(img, W, H, r, c) for c in range(W)] for r in range(H)]

    def gp(dr, dc):
        return g[clamp(r + dr, 0, H - 1)][clamp(c + dc, 0, W - 1)]

    mags = [[0] * W for _ in range(H)]
    dirs = [[0] * W for _ in range(H)]
    for r in range(H):
        for c in range(W):
            gx = (gp(-1, 1) + 2 * gp(0, 1) + gp(1, 1)) - (gp(-1, -1) + 2 * gp(0, -1) + gp(1, -1))
            gy = (gp(1, -1) + 2 * gp(1, 0) + gp(1, 1)) - (gp(-1, -1) + 2 * gp(-1, 0) + gp(-1, 1))
            ax, ay = abs(gx), abs(gy)
            mags[r][c] = ax + ay
            if ay * 256 < ax * 106:
                dirs[r][c] = 0
            elif ax * 106 < ay * 256:
                dirs[r][c] = 1
            else:
                dirs[r][c] = 2 if (gx < 0) == (gy < 0) else 3
    return mags, dirs


def ref_mag(img, W, H):
    """理想 M1 输出：mag 饱和到 8bit"""
    mags, _ = ref_grad(img, W, H)
    return [[min(m, 255) for m in row] for row in mags]


# NMS 方向 → 两个邻居的相对坐标（与 RTL 的 S11 case 一一对应）
NB = {0: ((0, -1), (0, 1)),      # 00 左右：w21 / w23
      1: ((-1, 0), (1, 0)),      # 01 上下：w12 / w32
      2: ((-1, -1), (1, 1)),     # 10 对角线：w11 / w33
      3: ((-1, 1), (1, -1))}     # 11 反对角线：w13 / w31


def ref_nms(mags, dirs, W, H):
    """理想 NMS：中心幅值 >= 两个邻居幅值则保留（平局保留，与 RTL 的 >= 一致）"""
    keep = [[0] * W for _ in range(H)]
    for r in range(H):
        for c in range(W):
            (d1, d2) = NB[dirs[r][c]]
            n1 = mags[clamp(r + d1[0], 0, H - 1)][clamp(c + d1[1], 0, W - 1)]
            n2 = mags[clamp(r + d2[0], 0, H - 1)][clamp(c + d2[1], 0, W - 1)]
            m = mags[r][c]
            keep[r][c] = 1 if (m >= n1 and m >= n2) else 0
    return keep


def ref_edge(img, W, H, thr):
    """理想 M2 边缘图：通过 NMS 且 饱和幅值 > thr"""
    mags, dirs = ref_grad(img, W, H)
    keep = ref_nms(mags, dirs, W, H)
    return [[1 if (keep[r][c] and min(mags[r][c], 255) > thr) else 0
             for c in range(W)] for r in range(H)]


def sobel_mag(img, W, H):
    """现有 sobel_edge 的幅值（不做高斯），用来对比高斯有没有把细节抹过头"""
    def px(dr, dc):
        return img[clamp(r + dr, 0, H - 1)][clamp(c + dc, 0, W - 1)]

    out = [[0] * W for _ in range(H)]
    for r in range(H):
        for c in range(W):
            gx = (px(-1, 1) + 2 * px(0, 1) + px(1, 1)) - (px(-1, -1) + 2 * px(0, -1) + px(1, -1))
            gy = (px(1, -1) + 2 * px(1, 0) + px(1, 1)) - (px(-1, -1) + 2 * px(-1, 0) + px(-1, 1))
            out[r][c] = min(abs(gx) + abs(gy), 255)
    return out


# ---------------------------------------------------------------------
# 图样
# ---------------------------------------------------------------------
def pat_uniform(W, H):
    return [[128] * W for _ in range(H)]


def pat_impulse(W, H):
    img = [[0] * W for _ in range(H)]
    img[H // 2][W // 2] = 255
    return img


def pat_lines(W, H):
    """竖直 / 水平 / 45° / 135° 细线 + 一条 2 像素宽竖线（方案 §8 用例 3、4）"""
    img = [[60] * W for _ in range(H)]
    for r in range(H):
        img[r][W // 4] = 220                      # 竖直 1 像素
    for c in range(W):
        img[H // 4][c] = 220                      # 水平 1 像素
    for r in range(H):                            # 45°: r - c = -16
        c = r + 16
        if 0 <= c < W:
            img[r][c] = 220
    for r in range(H):                            # 135°: r + c = 48
        c = 48 - r
        if 0 <= c < W:
            img[r][c] = 220
    for r in range(H):                            # 2 像素宽竖线
        img[r][3 * W // 4] = 220
        img[r][3 * W // 4 + 1] = 220
    return img


def pat_thin(W, H):
    """1 像素 / 2 像素细线各一条，背景 60（看高斯会不会把它抹平）"""
    img = [[60] * W for _ in range(H)]
    for r in range(H):
        img[r][W // 3] = 240
        img[r][2 * W // 3] = 240
        img[r][2 * W // 3 + 1] = 240
    return img


def pat_noise(W, H, seed=12345):
    rnd = random.Random(seed)
    return [[rnd.randrange(256) for _ in range(W)] for _ in range(H)]


def pat_checker(W, H, cell=4):
    return [[255 if ((r // cell) + (c // cell)) % 2 else 0 for c in range(W)] for r in range(H)]


def pat_step(W, H, orient):
    """阶跃（半平面）图样：垂直于该方向扫描时，理想 NMS 只应剩 1 个像素宽"""
    img = [[60] * W for _ in range(H)]
    for r in range(H):
        for c in range(W):
            if orient == "v":
                on = c >= W // 2                       # 竖直边界
            elif orient == "h":
                on = r >= H // 2                       # 水平边界
            elif orient == "45":
                on = (r - c) >= 0                      # 45° 边界（r-c = 0）
            else:
                on = (r + c) >= (W + H) // 2           # 135° 边界（r+c = 中值）
            img[r][c] = 220 if on else 60
    return img


# 阶跃图样几何：边界方向 along、垂直（扫描）方向 perp
STEP_GEO = {
    "v":   ((1, 0), (0, 1)),
    "h":   ((0, 1), (1, 0)),
    "45":  ((1, 1), (1, -1)),
    "135": ((1, -1), (1, 1)),
}


def pat_step_center(W, H, orient):
    """边界上的参考点（各方向都取画面中部）"""
    if orient == "45":
        return (H // 2, H // 2)
    if orient == "135":
        return (H // 2, (W + H) // 2 - H // 2)
    return (H // 2, W // 2)


def measure_width(getter, W, H, orient, n_lines=11, half=5):
    """沿垂直于边界的方向来回扫，量边缘像素连续段的最大长度

    getter(r, c) 越界应返回 None。返回最大宽度（0 = 这些扫描线上没有边缘）。
    """
    (ar, ac), (pr, pc) = STEP_GEO[orient]
    cr, cc = pat_step_center(W, H, orient)
    worst = 0
    for t in range(-(n_lines // 2), n_lines // 2 + 1):
        run = best = 0
        for s in range(-half, half + 1):
            r, c = cr + t * ar + s * pr, cc + t * ac + s * pc
            if not (0 <= r < H and 0 <= c < W):
                continue
            if getter(r, c):
                run += 1
                best = max(best, run)
            else:
                run = 0
        worst = max(worst, best)
    return worst


PATTERNS = {
    "uniform":  pat_uniform,
    "impulse":  pat_impulse,
    "lines":    pat_lines,
    "thin":     pat_thin,
    "noise":    pat_noise,
    "checker":  pat_checker,
    "step_v":   lambda W, H: pat_step(W, H, "v"),
    "step_h":   lambda W, H: pat_step(W, H, "h"),
    "step_45":  lambda W, H: pat_step(W, H, "45"),
    "step_135": lambda W, H: pat_step(W, H, "135"),
}


# ---------------------------------------------------------------------
# 仿真驱动
# ---------------------------------------------------------------------
def write_hex(path, img, W, H):
    with open(path, "w") as f:
        for r in range(H):
            for c in range(W):
                f.write("%02x\n" % (img[r][c] & 0xFF))


def run_tb(exe, build, stim, out, W, H):
    cmd = [exe, "+STIM=%s" % stim, "+OUT=%s" % out]
    res = subprocess.run(cmd, cwd=build, capture_output=True, text=True, timeout=600)
    return res.stdout + res.stderr


def parse_out(path, W, H):
    mag = [[None] * W for _ in range(H)]
    edge = [[None] * W for _ in range(H)]
    vals = []
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) == 2:
                vals.append((int(parts[0], 16), int(parts[1])))
    if len(vals) != W * H:
        raise RuntimeError("输出像素数 %d != %d" % (len(vals), W * H))
    for i, (m, e) in enumerate(vals):
        r, c = divmod(i, W)
        mag[r][c] = m
        edge[r][c] = e
    return mag, edge


def search_offset(dut, ref, W, H, margin=4, dr_range=8, dc_range=16):
    """找整体偏移 (dr,dc)：dut[r][c] 应等于 ref[r+dr][c+dc]"""
    best = None
    for dr in range(-dr_range, dr_range + 1):
        for dc in range(-dc_range, dc_range + 1):
            err = 0
            n = 0
            for r in range(margin, H - margin):
                rr = r + dr
                if rr < 2 or rr > H - 3:
                    continue
                for c in range(margin, W - margin):
                    cc = c + dc
                    if cc < 2 or cc > W - 3:
                        continue
                    n += 1
                    if dut[r][c] != ref[rr][cc]:
                        err += 1
            if n == 0:
                continue
            if best is None or err < best[0]:
                best = (err, n, dr, dc)
    return best


def verify_offset(dut, ref, W, H, dr, dc, margin=5):
    """固定偏移下逐点比对：dut[r][c] 应等于 ref[r+dr][c+dc]。返回 (err, n, 前几个失配)"""
    err = 0
    n = 0
    bad = []
    for r in range(margin, H - margin):
        rr = r + dr
        if rr < 2 or rr > H - 3:
            continue
        for c in range(margin, W - margin):
            cc = c + dc
            if cc < 2 or cc > W - 3:
                continue
            n += 1
            if dut[r][c] != ref[rr][cc]:
                err += 1
                if len(bad) < 8:
                    bad.append((r, c, dut[r][c], rr, cc, ref[rr][cc]))
    return err, n, bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", default="sim/build/obj_dir/Vtb_canny_edge")
    ap.add_argument("--build", default="sim/build")
    ap.add_argument("names", nargs="*", help="只跑指定图样（默认全跑）")
    ap.add_argument("--W", type=int, default=64)
    ap.add_argument("--H", type=int, default=32)
    args = ap.parse_args()

    W, H = args.W, args.H
    build = args.build
    os.makedirs(build, exist_ok=True)

    names = args.names if args.names else list(PATTERNS.keys())
    for n in names:
        if n not in PATTERNS:
            print("未知图样：%s（可选：%s）" % (n, ", ".join(PATTERNS)))
            return 2
    fails = 0

    for name in names:
        img = PATTERNS[name](W, H)
        stim = os.path.join(build, "stim_%s.hex" % name)
        out = os.path.join(build, "out_%s.txt" % name)
        write_hex(stim, img, W, H)

        log = run_tb(os.path.abspath(args.exe), build,
                     os.path.basename(stim), os.path.basename(out), W, H)

        lat = re.search(r"LATENCY src_de->canny_de = (-?\d+) \(期望 (\d+)\)", log)
        blank = re.search(r"BLANK_ERR (\d+)", log)
        rows = re.search(r"ROWS (\d+) SHORT (\d+) LONG (\d+)", log)
        capped = re.search(r"CAPTURED (\d+) \(期望 (\d+)\)", log)

        print("=== %s ===" % name)
        if lat:
            print("  延迟 %s 拍（期望 %s）  消隐期错误 %s  行数 %s 短%s 长%s  抓取 %s/%s" % (
                lat.group(1), lat.group(2), blank.group(1),
                rows.group(1), rows.group(2), rows.group(3),
                capped.group(1), capped.group(2)))
        else:
            print("  !! 没拿到测试台状态输出，原始日志：")
            print(log)
            fails += 1
            continue

        dut_mag, dut_edge = parse_out(os.path.join(build, os.path.basename(out)), W, H)
        ref = ref_mag(img, W, H)

        # mag / edge 都按"流水线固有偏移"这个常量比对
        # 比对区留 5 圈边：最外 2 行 2 列是行缓存/边界复制口径差异，第 3、4 行是
        # 帧首行缓存还没填满的暂态，而 NMS 又要再看一圈邻居，所以取 5。
        dr, dc = EXP_DR, EXP_DC
        err, n, bad = verify_offset(dut_mag, ref, W, H, dr, dc, margin=5)
        print("  对齐偏移 (dr,dc) = (%d,%d)（写死的流水线偏移）；比对 %d 点，失配 %d 点"
              % (dr, dc, n, err))

        # edge 的比对区再收一圈（margin=6）：edge 是 NMS 判决，要看中心像素的
        # 邻居；第 5 行的邻居还是帧首暂态算出来的值，判决可以不一样。
        edge_ref = ref_edge(img, W, H, THR)
        eerr, en, ebad = verify_offset(dut_edge, edge_ref, W, H, dr, dc, margin=6)
        print("  edge 比对 %d 点，失配 %d 点（参考 = NMS + mag_sat > %d）" % (en, eerr, THR))

        if err != 0 or eerr != 0:
            fails += 1
            print("  !! FAIL")
            for (r, c, v, rr, cc, rv) in bad[:6]:
                print("     mag dut(%d,%d)=%d  ref(%d,%d)=%d" % (r, c, v, rr, cc, rv))
            for (r, c, v, rr, cc, rv) in ebad[:6]:
                print("     edge dut(%d,%d)=%d  ref(%d,%d)=%d" % (r, c, v, rr, cc, rv))
            # 顺带报一下"最佳偏移"作参考：若最佳偏移不是常量偏移，说明是整体错位；
            # 若相等，说明是逐点算错（或图样自对称）
            best = search_offset(dut_mag, ref, W, H, margin=5)
            if best:
                print("     （参考）mag 自由搜偏移 (%d,%d) 失配 %d/%d"
                      % (best[2], best[3], best[0], best[1]))
        else:
            print("  OK")

        # 细化率：过阈值像素 → 通过 NMS 的像素（都在同一片有效区内统计）
        over = keep = 0
        for r in range(4, H - 4):
            for c in range(4, W - 4):
                if ref[r][c] > THR:
                    over += 1
                if edge_ref[r][c]:
                    keep += 1
        print("  NMS 细化：过阈值 %d → 保留 %d（%s）"
              % (over, keep, "%.0f%%" % (100.0 * keep / over) if over else "n/a"))

        # 单像素宽度：阶跃图样沿垂直方向扫描，量边缘像素的连续段长度
        if name.startswith("step"):
            orient = name.split("_")[1]
            dw = measure_width(lambda r, c: dut_edge[r - dr][c - dc]
                               if (0 <= r - dr < H and 0 <= c - dc < W) else None,
                               W, H, orient)
            rw = measure_width(lambda r, c: edge_ref[r][c], W, H, orient)
            print("  边缘宽度（%s 方向）：DUT 最大 %d 像素 / 参考最大 %d 像素" % (orient, dw, rw))
            if dw > rw:
                fails += 1
                print("  !! FAIL: NMS 细化不如参考模型")

        # 统计：与"不做高斯的 sobel"对比
        sob = sobel_mag(img, W, H)
        cmax = max(max(row) for row in ref)
        smax = max(max(row) for row in sob)
        cover = sum(1 for r in range(H) for c in range(W) if ref[r][c] > THR)
        scover = sum(1 for r in range(H) for c in range(W) if sob[r][c] > THR)
        print("  mag 峰值：canny %d / sobel %d；过阈值像素：canny %d / sobel %d"
              % (cmax, smax, cover, scover))

    print()
    print("总计：%d 个图样，失败 %d 个" % (len(names), fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
