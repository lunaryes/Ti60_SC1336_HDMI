#!/usr/bin/env bash
# ============================================================
#  L1 模块仿真入口（iverilog，无需安装 Efinity）
#
#  用法：
#    bash sim/scripts/run_iverilog.sh <DUT.v> <TB.v> [更多.v ...]
#
#  例：
#    bash sim/scripts/run_iverilog.sh \
#        Ti60F225_SC1336_HDMI_Display/src/image/pix_window_3x3.v \
#        sim/tb/tb_pix_window_3x3.v
#
#  波形输出到 sim/out/<TB名>.vcd，可用 GTKWave 打开。
# ============================================================
set -euo pipefail

if [ $# -lt 2 ]; then
    echo "用法: $0 <DUT.v> <TB.v> [更多.v ...]" >&2
    exit 2
fi

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT_DIR="$ROOT/sim/out"
mkdir -p "$OUT_DIR"

SRCS=("$@")

# TB 文件名（不含扩展名）作为顶层与输出名
TB_PATH="${SRCS[${#SRCS[@]}-1]}"
TB_NAME="$(basename "$TB_PATH" .v)"
VVP="$OUT_DIR/$TB_NAME.vvp"
LOG="$OUT_DIR/$TB_NAME.log"

echo "[1/3] 语法检查 (iverilog -Wall -g2005)"
iverilog -g2005 -Wall \
    -I "$ROOT/sim/tb" \
    -I "$ROOT/Ti60F225_SC1336_HDMI_Display/src" \
    -s "$TB_NAME" \
    -o "$VVP" \
    "${SRCS[@]/#/$ROOT/}" 2>&1 | tee "$LOG"
# 注：上面的路径拼装假设调用时传的是相对仓库根的路径，请在仓库根执行本脚本。

echo "[2/3] 运行仿真"
vvp "$VVP" 2>&1 | tee -a "$LOG"

echo "[3/3] 完成。日志: $LOG"
if ls "$OUT_DIR"/*.vcd >/dev/null 2>&1; then
    echo "      波形: $OUT_DIR/*.vcd"
fi
