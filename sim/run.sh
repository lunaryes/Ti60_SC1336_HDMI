#!/bin/sh
# =====================================================================
# 一键编译并运行边缘检测流水线的自检 testbench
# ---------------------------------------------------------------------
# 用法（在 repo/ 根目录）：
#     sh sim/run.sh
#
# 覆盖仿真器路径：
#     IVB=/path/to/iverilog/bin sh sim/run.sh
#
# 依赖：Icarus Verilog 14.0（iverilog.exe + vvp.exe）
#   本机已装于 D:\下载迁移文件夹\iverilog\bin
#   若未加入 PATH，用绝对路径调用即可（无需重装）。
#   安装包（含 GTKWave）: https://bleyer.org/icarus/
#
# 退出码：0 = 全部检查通过；非 0 = 有不符或编译失败
# =====================================================================
set -e

IVB=${IVB:-/d/下载迁移文件夹/iverilog/bin}
SRC=Ti60F225_SC1336_HDMI_Display/src

if [ ! -x "$IVB/iverilog.exe" ]; then
    echo "找不到 iverilog.exe：$IVB" >&2
    echo "请用 IVB=<iverilog的bin目录> sh sim/run.sh 指定，或安装见 https://bleyer.org/icarus/" >&2
    exit 2
fi

echo "== 编译 =="
"$IVB/iverilog.exe" -g2005 -o sim/tb.vvp \
    "$SRC/image/pix_window_3x3.v" \
    "$SRC/image/sobel_edge.v" \
    "$SRC/display/image_mix.v" \
    "$SRC/common/sync_delay_align.v" \
    "$SRC/image/image_pipeline_top.v" \
    sim/tb_image_pipeline.v \
    $(: 忽略 timescale 继承告警)

echo "== 运行 =="
"$IVB/vvp.exe" sim/tb.vvp

echo
echo "== 边界行为基线（预期 6/6 与旧契约不符，见 docs/ARCH.md §11.4）=="
"$IVB/iverilog.exe" -g2005 -o sim/tb_bnd.vvp \
    "$SRC/image/pix_window_3x3.v" \
    sim/tb_window_boundary.v
"$IVB/vvp.exe" sim/tb_bnd.vvp | sed -n '/^\[帧/p'
rm -f sim/tb_bnd.vvp
