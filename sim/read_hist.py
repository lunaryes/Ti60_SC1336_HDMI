#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""read_hist —— 读片内 256 bin 灰度直方图（无采集卡路径）

背景
----
现场没有 HDMI 采集卡，抓不到无损帧。FPGA 里新增的
`src/common/gray_hist_uart.v` 直接统计 3×3 中值之后、Canny 实际看到的 Y
（2026-10-17 中值接入后抽头随之后移），收到串口 'H'（0x48）后测完整
一帧，回传 778 字节：

    [0..2]    A5 5A 48           magic + 命令
    [3..4]    frame_id           16 bit LE
    [5..7]    frame_total        24 bit LE（本帧 de 像素数）
    [8..775]  256 × 3 字节       bin0..bin255，每 bin 20 bit LE
    [776..777] checksum          16 bit LE = bytes[0..775] 之和 mod 65536

这个脚本做三件事：发 'H'、读满 778 字节、按 `gray_hist.py` 同一口径打印
直方图和分位数。它**不能**算 mag8 过阈比例/空间分布——那需要整帧像素，
不是 256 个 bin 能恢复的；需要那一项时仍要抓 Y 图跑 `gray_hist.py`。

依赖
----
numpy + Pillow（复用 `gray_hist.py` 的统计/打印口径，它 import 了 PIL；
直方图本身不需要图像，但这样两边数字完全一致）。串口访问**不需要 pyserial**：
  · Windows：ctypes 调 Win32 API；
  · Linux/WSL：termios + select，标准库。

用法
----
    python sim/read_hist.py --port COM5
    python sim/read_hist.py --port /dev/ttyUSB0 --json hist.json
    python sim/read_hist.py --file hist_stream.bin --save-raw copy.bin

板上前提
--------
* 位流已下载，摄像头在出图（发 'R' 能收到 6 字节回包就说明串口通）；
* 波特率 460800 8N1（与 uart_cmd / gray_hist_uart 一致）；
* 发 'H' 后**不要**再发别的命令，等 778 字节读完（约 17 ms 数据 + 最多
  2 帧等待 ≈ 50 ms）。脚本自己会等 3 s。
"""

import argparse
import ctypes
import json
import os
import sys
import time

import numpy as np

# 与脚本同目录；从仓库根运行也能 import
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gray_hist as G


MAGIC = bytes([0xA5, 0x5A, 0x48])
STREAM_LEN = 778


# ---------------------------------------------------------------------
# 串口：Windows（Win32 API，无 pyserial）
# ---------------------------------------------------------------------
def _open_serial_windows(port, baud, timeout):
    from ctypes import wintypes

    GENERIC_READ = 0x80000000
    GENERIC_WRITE = 0x40000000
    OPEN_EXISTING = 3
    INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value

    class DCB(ctypes.Structure):
        _fields_ = [
            ("DCBlength", wintypes.DWORD),
            ("BaudRate", wintypes.DWORD),
            ("fFlags", wintypes.DWORD),
            ("wReserved", wintypes.WORD),
            ("XonLim", wintypes.WORD),
            ("XoffLim", wintypes.WORD),
            ("ByteSize", wintypes.BYTE),
            ("Parity", wintypes.BYTE),
            ("StopBits", wintypes.BYTE),
            ("XonChar", ctypes.c_char),
            ("XoffChar", ctypes.c_char),
            ("ErrorChar", ctypes.c_char),
            ("EofChar", ctypes.c_char),
            ("EvtChar", ctypes.c_char),
            ("wReserved1", wintypes.WORD),
        ]

    class COMMTIMEOUTS(ctypes.Structure):
        _fields_ = [
            ("ReadIntervalTimeout", wintypes.DWORD),
            ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
            ("ReadTotalTimeoutConstant", wintypes.DWORD),
            ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
            ("WriteTotalTimeoutConstant", wintypes.DWORD),
        ]

    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    # 64 位 Windows 上句柄是 8 字节。不设 argtypes，ctypes 会按 C int
    # （4 字节）传参，句柄被截断，ReadFile/WriteFile 直接失败 —— 这是
    # ctypes 调 Win32 最典型的坑，必须逐个函数钉死类型。
    kernel32.CreateFileW.restype = wintypes.HANDLE
    kernel32.CreateFileW.argtypes = [
        wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
        wintypes.LPVOID, wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE,
    ]
    kernel32.SetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(DCB)]
    kernel32.SetCommState.restype = wintypes.BOOL
    kernel32.SetCommTimeouts.argtypes = [
        wintypes.HANDLE, ctypes.POINTER(COMMTIMEOUTS)]
    kernel32.SetCommTimeouts.restype = wintypes.BOOL
    kernel32.SetupComm.argtypes = [
        wintypes.HANDLE, wintypes.DWORD, wintypes.DWORD]
    kernel32.SetupComm.restype = wintypes.BOOL
    kernel32.PurgeComm.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    kernel32.PurgeComm.restype = wintypes.BOOL
    kernel32.ReadFile.argtypes = [
        wintypes.HANDLE, wintypes.LPVOID, wintypes.DWORD,
        ctypes.POINTER(wintypes.DWORD), wintypes.LPVOID]
    kernel32.ReadFile.restype = wintypes.BOOL
    kernel32.WriteFile.argtypes = [
        wintypes.HANDLE, wintypes.LPCSTR, wintypes.DWORD,
        ctypes.POINTER(wintypes.DWORD), wintypes.LPVOID]
    kernel32.WriteFile.restype = wintypes.BOOL
    kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
    kernel32.CloseHandle.restype = wintypes.BOOL

    if port.upper().startswith("COM") and not port.startswith("\\\\.\\"):
        path = "\\\\.\\" + port
    else:
        path = port

    h = kernel32.CreateFileW(path, GENERIC_READ | GENERIC_WRITE,
                             0, None, OPEN_EXISTING, 0, None)
    if h == INVALID_HANDLE_VALUE or h is None:
        err = ctypes.get_last_error()
        raise OSError(err, "打不开串口 %s（WinError %d）。"
                           "检查端口号是否被别的终端占用。" % (port, err))

    dcb = DCB()
    dcb.DCBlength = ctypes.sizeof(DCB)
    dcb.BaudRate = baud
    # fBinary=1, fDtrControl=1(bit4), fRtsControl=1(bit12)；8N1、无流控
    dcb.fFlags = 0x0001 | 0x0010 | 0x1000
    dcb.ByteSize = 8
    dcb.Parity = 0
    dcb.StopBits = 0
    if not kernel32.SetCommState(h, ctypes.byref(dcb)):
        err = ctypes.get_last_error()
        kernel32.CloseHandle(h)
        raise OSError(err, "SetCommState 失败（WinError %d）" % err)

    to = COMMTIMEOUTS()
    to.ReadIntervalTimeout = 50
    to.ReadTotalTimeoutMultiplier = 0
    to.ReadTotalTimeoutConstant = 100
    to.WriteTotalTimeoutMultiplier = 0
    to.WriteTotalTimeoutConstant = 1000
    kernel32.SetCommTimeouts(h, ctypes.byref(to))
    kernel32.SetupComm(h, 4096, 4096)
    kernel32.PurgeComm(h, 0x0004 | 0x0008)   # TXCLEAR | RXCLEAR

    return ("win", h, kernel32)


def _read_chunk_windows(ctx):
    _, h, kernel32 = ctx
    buf = ctypes.create_string_buffer(4096)
    n = ctypes.c_ulong(0)
    ok = kernel32.ReadFile(h, buf, 4096, ctypes.byref(n), None)
    if not ok:
        return b""
    return buf.raw[:n.value]


def _write_windows(ctx, data):
    _, h, kernel32 = ctx
    n = ctypes.c_ulong(0)
    ok = kernel32.WriteFile(h, data, len(data), ctypes.byref(n), None)
    if not ok:
        raise OSError("WriteFile 失败")


def _close_windows(ctx):
    _, h, kernel32 = ctx
    kernel32.CloseHandle(h)


# ---------------------------------------------------------------------
# 串口：POSIX（termios，标准库）
# ---------------------------------------------------------------------
def _open_serial_posix(port, baud, timeout):
    import termios

    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    # iflag：8N1、无软/硬件流控、不做字符转换
    attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK
                  | termios.ISTRIP | termios.INLCR | termios.IGNCR
                  | termios.ICRNL | termios.IXON | termios.IXOFF)
    attrs[1] &= ~termios.OPOST
    attrs[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
    attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON
                  | termios.ISIG | termios.IEXTEN)
    speed = getattr(termios, "B%d" % baud, None)
    if speed is None:
        raise OSError("termios 没有 B%d 常量，不支持波特率 %d"
                      % (baud, baud))
    attrs[4] = speed
    attrs[5] = speed
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return ("posix", fd, None)


def _read_chunk_posix(ctx):
    import select

    _, fd, _ = ctx
    r, _, _ = select.select([fd], [], [], 0.05)
    if not r:
        return b""
    try:
        return os.read(fd, 4096)
    except OSError:
        return b""


def _write_posix(ctx, data):
    _, fd, _ = ctx
    os.write(fd, data)


def _close_posix(ctx):
    _, fd, _ = ctx
    os.close(fd)


# ---------------------------------------------------------------------
# 读流
# ---------------------------------------------------------------------
def read_stream(port, baud, timeout):
    if os.name == "nt":
        ctx = _open_serial_windows(port, baud, timeout)
        read_chunk, write, close = (_read_chunk_windows, _write_windows,
                                    _close_windows)
    else:
        ctx = _open_serial_posix(port, baud, timeout)
        read_chunk, write, close = (_read_chunk_posix, _write_posix,
                                    _close_posix)
    try:
        write(ctx, b"H")
        deadline = time.monotonic() + timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = read_chunk(ctx)
            if chunk:
                buf.extend(chunk)
                # 收到 magic 之后要凑满 778 字节
                pos = buf.find(MAGIC)
                if pos >= 0 and len(buf) - pos >= STREAM_LEN:
                    return bytes(buf[pos:pos + STREAM_LEN])
            else:
                time.sleep(0.01)
        pos = buf.find(MAGIC)
        if pos >= 0:
            raise TimeoutError(
                "只收到 %d 字节（从 magic 起），还差 %d 字节。"
                % (len(buf) - pos, STREAM_LEN - (len(buf) - pos)))
        raise TimeoutError(
            "%.1f s 内没收到 magic A5 5A 48。收到的前 16 字节：%s\n"
            "  排查：位流是否下载 / 摄像头是否在出图 / 波特率是否 460800 /\n"
            "        TX-RX 是否交叉 / 串口是否被别的终端占用 / 发 'R' 能否回 6 字节。"
            % (timeout, buf[:16].hex(" ")))
    finally:
        close(ctx)


def parse_stream(data):
    if len(data) < STREAM_LEN:
        raise ValueError("流长度 %d < %d" % (len(data), STREAM_LEN))
    if data[0:3] != MAGIC:
        raise ValueError("magic 不对：%s" % data[0:3].hex(" "))
    frame_id = data[3] | (data[4] << 8)
    total = data[5] | (data[6] << 8) | (data[7] << 16)
    bins = []
    for i in range(256):
        o = 8 + 3 * i
        bins.append(data[o] | (data[o + 1] << 8) | (data[o + 2] << 16))
    chk_calc = sum(data[0:776]) & 0xFFFF
    chk_recv = data[776] | (data[777] << 8)
    if chk_calc != chk_recv:
        raise ValueError("checksum 不符：算得 %04X，收到 %04X"
                         % (chk_calc, chk_recv))
    bins = np.array(bins, dtype=np.int64)
    s = int(bins.sum())
    return {
        "frame_id": frame_id,
        "total": total,
        "bins": bins,
        "sum_bins": s,
        "checksum": chk_recv,
    }


# ---------------------------------------------------------------------
# 报告
# ---------------------------------------------------------------------
def _ensure_parent(path):
    d = os.path.dirname(os.path.abspath(path))
    if d:
        os.makedirs(d, exist_ok=True)


def report(frame, out_json=None, save_raw=None, raw_data=None):
    bins = frame["bins"]
    total = frame["total"]
    s = frame["sum_bins"]

    print("=" * 96)
    print("片上 Y 直方图（gray_hist_uart：3×3 中值之后的 Y，Canny 实际输入）")
    print("=" * 96)
    print("  帧号 %d    frame_total %d    sum(bins) %d    checksum %04X OK"
          % (frame["frame_id"], total, s, frame["checksum"]))
    if s != total:
        G.warn("sum(bins) != frame_total（%d vs %d）—— RTL 累加有问题，"
               "这组数据不可信" % (s, total))
    if total != 1280 * 720:
        G.warn("frame_total = %d，不是 1280×720=921600；如果摄像头/裁剪变了"
               "可以忽略，否则说明 de 统计漏了/多了" % total)
    if s == 0:
        print("  直方图全 0（没有统计到任何 de 像素）—— 检查 vsync/de 是否接对")
        return

    # 用与 gray_hist.py 完全相同的 hist_stats 口径（重建一个等价数组）
    v = np.repeat(np.arange(256, dtype=np.uint8), bins.astype(np.int64))
    stats = G.hist_stats(v)

    print()
    print("[1] 灰度直方图")
    G.print_histogram(stats)
    print()
    print("[2] 统计（与 gray_hist.py 同口径）")
    G.print_stats(stats)

    print()
    print("[3] 判读（仅直方图）")
    ge254 = stats["ge254_pct"]
    p1, p50, p99 = stats["p1"], stats["p50"], stats["p99"]
    if ge254 > 0.5:
        print("  !! 强光过曝：>=254 占 %.2f%%（> 0.5%%）。" % ge254)
        print("     已削顶的高光，任何数字增益/gamma 都救不回来 —— 先降 sensor 曝光/增益")
        print("     （src/cmos_i2c/I2C_SC1336_1280720_Config.v 的 3e01/3e02）。")
        print("     这段场景里，亮块与更亮块的边界已经消失；黑字 vs 亮块仍可能出边。")
    elif p99 < 170:
        print("  整体偏暗、有提升空间：p99=%.0f，>=254 仅 %.2f%%。" % (p99, ge254))
        print("     → 数字增强（gain / levels / gamma）有空间；但先确认是“正常光”")
        print("       而不是曝光没配对 —— 欠曝把 ΔY 等比压小，阈值却不变。")
    elif p50 < 90:
        print("  中位偏暗：p50=%.0f，p1=%.0f，p99=%.0f。" % (p50, p1, p99))
        print("     → 暗部占比大；gamma<1 或抬黑场可能比纯 gain 更合适。")
    elif p1 > 32 and ge254 < 0.5 and p99 < 245:
        print("  黑电平被抬高：p1=%.0f（正常应接近 0），p99=%.0f。" % (p1, p99))
        print("     → 像是采集/显示链路 limited range（16..235）或 AWB 抬了黑场；")
        print("       先确认 HDMI 采集是 Full range，再考虑拉黑场。")
    elif p1 < 8 and p99 > 220 and ge254 < 0.5:
        print("  动态范围已经铺满：p1=%.0f，p99=%.0f，>=254 %.2f%%。"
              % (p1, p99, ge254))
        print("     → 全局增强的空间有限；如果局部轮廓仍不清晰，问题更可能在")
        print("       局部对比度/对焦/噪声，而不是整体亮度。")
    else:
        print("  分布基本正常：p1=%.0f  p50=%.0f  p99=%.0f  >=254 %.2f%%"
              % (p1, p50, p99, ge254))
        print("     → 没有明显的整体欠曝/削顶；若边缘仍出不来，优先看局部 ΔY。")

    print()
    print("  注意：本工具只有 256 个 bin，**算不了** mag8 过阈比例和空间分布。")
    print("        要回答“低对比度人脸轮廓能不能过阈值”，用采集卡抓 view=3 的 Y 图，")
    print("        再跑： python sim/gray_hist.py frame.png --roi ... --grid 3")
    print("        三张对比（正常光 / 强光 / 盖镜头）能分开曝光问题和局部对比度问题。")

    if out_json:
        obj = {
            "source": "gray_hist_uart",
            "frame_id": frame["frame_id"],
            "frame_total": total,
            "sum_bins": s,
            "checksum": frame["checksum"],
            "bins": bins.tolist(),
            "stats": {
                k: (sv.tolist() if isinstance(sv, np.ndarray) else sv)
                for k, sv in stats.items()
            },
        }
        _ensure_parent(out_json)
        with open(out_json, "w", encoding="utf-8") as f:
            json.dump(obj, f, ensure_ascii=False, indent=2)
        print("\n统计结果已写入：%s" % out_json)

    if save_raw and raw_data is not None:
        _ensure_parent(save_raw)
        with open(save_raw, "wb") as f:
            f.write(raw_data)
        print("原始 778 字节流已写入：%s" % save_raw)


def main():
    G.init_console()
    ap = argparse.ArgumentParser(
        description="读片内 256 bin 灰度直方图（'H' 命令，778 字节）")
    ap.add_argument("--port", help="串口，如 COM5 或 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--timeout", type=float, default=3.0,
                    help="等流的秒数（默认 3 s，覆盖 2 帧 + 回传）")
    ap.add_argument("--file", help="不从串口读，解析已保存的 778 字节流")
    ap.add_argument("--json", help="把直方图与统计写成 JSON")
    ap.add_argument("--save-raw", help="把收到的 778 字节原样保存")
    args = ap.parse_args()

    if not args.file and not args.port:
        ap.error("要么给 --port，要么给 --file")

    if args.file:
        with open(args.file, "rb") as f:
            blob = f.read()
        pos = blob.find(MAGIC)
        if pos < 0:
            print("!! 文件里找不到 magic A5 5A 48：%s" % args.file)
            return 1
        if len(blob) - pos < STREAM_LEN:
            print("!! 文件从 magic 起只有 %d 字节，不足 %d"
                  % (len(blob) - pos, STREAM_LEN))
            return 1
        data = blob[pos:pos + STREAM_LEN]
    else:
        try:
            data = read_stream(args.port, args.baud, args.timeout)
        except Exception as e:
            print("!! 读流失败：%s" % e)
            return 1

    try:
        frame = parse_stream(data)
    except ValueError as e:
        print("!! 解析失败：%s" % e)
        print("   原始数据前 16 字节：%s" % data[:16].hex(" "))
        return 1

    report(frame, out_json=args.json, save_raw=args.save_raw, raw_data=data)
    return 0


if __name__ == "__main__":
    sys.exit(main())
