#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
canny_edge 参考模型 + 回归驱动（当前对应 M1：高斯 + Sobel 梯度 + 幅值）

参考模型刻意用**与 RTL 完全相同的近似**（方案 §8.1 的要求），否则比对必然不一致：
    g   = (p11 + 2p12 + p13 + 2p21 + 4p22 + 2p23 + p31 + 2p32 + p33) >> 4
    Gx  = (g13 + 2g23 + g33) - (g11 + 2g21 + g31)
    Gy  = (g31 + 2g32 + g33) - (g11 + 2g12 + g13)
    mag = |Gx| + |Gy|，饱和到 8bit
不做方向量化 / NMS / 双阈值滞后——那些是 M2 / M3 的事。

比对方式：参考模型给出"中心落在 (r,c) 的理想值"，RTL 输出与它之间只允许差一个
固定的整体偏移 (dr,dc)（流水线固有的行列偏移）。驱动脚本在小范围内搜这个偏移，
要求找到的偏移能让**整个有效区域逐像素相等**，找不到就是错的（这会同时抓住
"偏移不对"和"算得不对"两类问题，也顺便验证输出没有被拉伸/错位）。

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


def ref_mag(img, W, H):
    """理想 M1 输出：mag 饱和到 8bit，中心落在 (r,c)"""
    g = [[gauss_at(img, W, H, r, c) for c in range(W)] for r in range(H)]

    def gp(dr, dc):
        return g[clamp(r + dr, 0, H - 1)][clamp(c + dc, 0, W - 1)]

    out = [[0] * W for _ in range(H)]
    for r in range(H):
        for c in range(W):
            gx = (gp(-1, 1) + 2 * gp(0, 1) + gp(1, 1)) - (gp(-1, -1) + 2 * gp(0, -1) + gp(1, -1))
            gy = (gp(1, -1) + 2 * gp(1, 0) + gp(1, 1)) - (gp(-1, -1) + 2 * gp(-1, 0) + gp(-1, 1))
            out[r][c] = min(abs(gx) + abs(gy), 255)
    return out


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


PATTERNS = {
    "uniform": pat_uniform,
    "impulse": pat_impulse,
    "lines":   pat_lines,
    "thin":    pat_thin,
    "noise":   pat_noise,
    "checker": pat_checker,
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

        # mag 逐像素比对（整体偏移由搜索确定）
        best = search_offset(dut_mag, ref, W, H)
        if best is None:
            print("  !! 无有效比对区域")
            fails += 1
            continue
        err, n, dr, dc = best
        print("  对齐偏移 (dr,dc) = (%d,%d)；比对 %d 点，失配 %d 点" % (dr, dc, n, err))

        # edge 与 mag 的一致性（edge = mag > THR）
        edge_err = sum(1 for r in range(H) for c in range(W)
                       if dut_edge[r][c] != (1 if dut_mag[r][c] > THR else 0))
        print("  edge 与 mag 不一致 %d 点（thr=%d）" % (edge_err, THR))

        if err != 0 or edge_err != 0:
            fails += 1
            print("  !! FAIL")
            # 打印前几个失配点，便于定位
            shown = 0
            for r in range(4, H - 4):
                for c in range(4, W - 4):
                    rr, cc = r + dr, c + dc
                    if 2 <= rr <= H - 3 and 2 <= cc <= W - 3 and dut_mag[r][c] != ref[rr][cc]:
                        print("     dut(%d,%d)=%d  ref(%d,%d)=%d" % (r, c, dut_mag[r][c], rr, cc, ref[rr][cc]))
                        shown += 1
                        if shown >= 6:
                            break
                if shown >= 6:
                    break
        else:
            print("  OK")

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
