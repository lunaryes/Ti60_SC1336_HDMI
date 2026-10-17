#!/usr/bin/env python3
"""tb_uart_cmd 的变异测试：故意改坏 RTL，确认测试台真的会 FAIL。

为什么要有这个：仿真台"永远 PASS"是所有回归里最贵的一种失败模式。
tb_uart_cmd 的命令用例都靠 thr_o/view_o/mode_o 这几个寄存器做判据，
如果哪天接收通路整体失效（比如 rx_valid 不再脉冲），这些寄存器会停在
复位默认值上 —— 而复位默认值恰好又被第一组用例单独检查过，容易出现
"看起来测了、其实什么都没测"的假绿。这里用多个必死变异封住这个口子
（当前 9 个：回包字段互换/错值、命令写错通道、'S' 截断与脉冲、接收通路失效等）。

用法（不进 run.sh：每个变异都要完整编译一次 Verilator 台，合计约 15 分钟；
只在改动 uart_cmd.v 或 tb_uart_cmd.v 后手动跑）：
    python3 sim/mut_uart.py
"""
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "Ti60F225_SC1336_HDMI_Display", "src", "common", "uart_cmd.v")
TB = os.path.join(ROOT, "sim", "tb_uart_cmd.v")

# 每个变异：(名字, [(原文, 替换)], 期望在输出里看到的判定)
MUTANTS = [
    (
        "回包把 view/mode 写反",
        [("tx_d[1] <= {6'b0, view_o};", "tx_d[1] <= {6'b0, mode_o};"),
         ("tx_d[2] <= {6'b0, mode_o};", "tx_d[2] <= {6'b0, view_o};")],
        "FAIL",
    ),
    (
        "T 命令写入 +1",
        [("thr_o       <= rx_byte;", "thr_o       <= rx_byte + 1'b1;")],
        "FAIL",
    ),
    (
        "AWB 增益写进了相反的通道（'A' 命令改到 gain_b_o）",
        [("gain_r_o <= rx_byte;", "gain_b_o <= rx_byte;")],
        "FAIL",
    ),
    (
        "'S' 写入不再出 valid 脉冲",
        [("smooth_valid_o <= 1'b1;", "smooth_valid_o <= 1'b0;")],
        "FAIL",
    ),
    (
        "'S' 参数不再截断到 2",
        [("smooth_o       <= (rx_byte[1:0] > 2'd2) ? 2'd2\n                                                                     : rx_byte[1:0];",
          "smooth_o       <= rx_byte[1:0];")],
        "FAIL",
    ),
    (
        "回包把 smooth 写成固定 0",
        [("tx_d[5] <= {6'b0, smooth_read_i};", "tx_d[5] <= {6'b0, 2'd0};")],
        "FAIL",
    ),
    (
        "回包 thr 不再取实际生效值（改用 uart_cmd 自己的寄存器）",
        [("tx_d[0] <= thr_read_i;", "tx_d[0] <= thr_o;")],
        "FAIL",
    ),
    (
        "回包里的 R/B 增益写反",
        [("tx_d[3] <= gain_r_o;", "tx_d[3] <= gain_b_o;"),
         ("tx_d[4] <= gain_b_o;", "tx_d[4] <= gain_r_o;")],
        "FAIL",
    ),
    (
        "接收完成不再置 rx_valid（整条接收通路失效）",
        [("rx_valid <= 1'b1;\n                            rx_state <= S_IDLE;",
          "rx_valid <= 1'b0;\n                            rx_state <= S_IDLE;")],
        "FAIL",
    ),
]


def main():
    src = open(SRC, encoding="utf-8").read()
    nbad = 0
    for name, subs, want in MUTANTS:
        text = src
        for old, new in subs:
            if old not in text:
                print("  !! 变异「%s」的锚点没找到（RTL 改了？）：%r" % (name, old[:50]))
                nbad += 1
                continue
            text = text.replace(old, new, 1)
        d = tempfile.mkdtemp(prefix="mut_uart_")
        m = os.path.join(d, "uart_cmd.v")
        open(m, "w", encoding="utf-8").write(text)
        obj = os.path.join(d, "obj")
        r = subprocess.run(
            ["verilator", "--binary", "--timing", "-Wno-fatal", "-Wno-lint",
             "--top-module", "tb_uart_cmd", m, TB, "-o", "Vmut", "--Mdir", obj],
            capture_output=True, text=True)
        if r.returncode != 0:
            print("  !! 变异「%s」编译失败" % name)
            nbad += 1
            continue
        try:
            out = subprocess.run([os.path.join(obj, "Vmut")], capture_output=True,
                                 text=True, timeout=180).stdout
        except subprocess.TimeoutExpired:
            out = "TIMEOUT"
        lines = [l for l in out.splitlines() if "FAIL" in l or "PASS" in l]
        if out == "TIMEOUT":
            got = "FAIL"          # 挂死也算被抓住（但不该发生：recv_byte 带超时）
        elif not lines:
            got = "NOOUT"
        else:
            got = "FAIL" if "FAIL" in lines[-1] else "PASS"
        ok = (got == want)
        print("  %s 变异「%s」→ %s（期望 %s）" % ("OK  " if ok else "!!  ", name, got, want))
        if lines:
            print("        %s" % lines[-1])
        if not ok:
            nbad += 1
    print("变异测试：%s（共 %d 个）" % ("全部被抓住" if nbad == 0 else "%d 个没抓住" % nbad,
                                       len(MUTANTS)))
    return 1 if nbad else 0


if __name__ == "__main__":
    sys.exit(main())
