#!/bin/sh
# =====================================================================
# L2 验证一键脚本：RTL 转储 -> Python 参考模型逐像素比对
# ---------------------------------------------------------------------
# 用法（在 repo/ 根目录）：
#     sh sim/l2_verify.sh
#
# 覆盖仿真器路径：   IVB=/path/to/iverilog/bin sh sim/l2_verify.sh
# 覆盖 Python 路径： PYTHON=/path/to/python        sh sim/l2_verify.sh
#
# 做什么：
#   1. 编译 sim/tb_l2_dump.v + 全部算法源文件
#   2. 对 MODE=0/1/2/3 × THR=40/100 各跑一次仿真，转储输入/输出像素流
#   3. 调 tools/l2_ref_model.py 用独立参考模型逐像素比对
#   4. 汇总写 sim/l2_report.txt，退出码 0 = 全部通过
#
# 依赖：Icarus Verilog 14.0；Python 3。
# =====================================================================
set -e

IVB=${IVB:-/d/下载迁移文件夹/iverilog/bin}
SRC=Ti60F225_SC1336_HDMI_Display/src
REPORT=sim/l2_report.txt

if [ ! -x "$IVB/iverilog.exe" ]; then
    echo "找不到 iverilog.exe：$IVB" >&2
    echo "请用 IVB=<iverilog的bin目录> sh sim/l2_verify.sh 指定" >&2
    exit 2
fi

# --- 找 Python 3（优先 PATH，其次托管运行时） -------------------------
if [ -z "$PYTHON" ]; then
    if command -v python >/dev/null 2>&1; then
        PYTHON=python
    elif command -v python3 >/dev/null 2>&1; then
        PYTHON=python3
    else
        PYTHON="C:/Users/HUAWEI/.workbuddy/binaries/python/versions/3.13.12/python.exe"
    fi
fi

echo "== 编译 L2 testbench =="
"$IVB/iverilog.exe" -g2005 -o sim/tb_l2.vvp \
    "$SRC/image/pix_window_3x3.v" \
    "$SRC/image/sobel_edge.v" \
    "$SRC/display/edge_display_mix.v" \
    "$SRC/common/sync_delay_align.v" \
    "$SRC/image/image_pipeline_top.v" \
    sim/tb_l2_dump.v

: > "$REPORT"
rc=0

for MODE in 0 1 2 3; do
    for THR in 40 100; do
        echo
        echo "===== 仿真 MODE=$MODE THR=$THR ====="
        "$IVB/vvp.exe" sim/tb_l2.vvp +MODE=$MODE +THR=$THR | sed -n 's/^/  /p'

        echo "----- 参考模型比对 MODE=$MODE THR=$THR -----"
        if MODE=$MODE THR=$THR "$PYTHON" tools/l2_ref_model.py > sim/l2_tmp.txt 2>&1; then
            ok=0
        else
            ok=1
        fi
        cat sim/l2_tmp.txt | tee -a "$REPORT"
        [ "$ok" -eq 0 ] || rc=1
    done
done

echo
echo "== L2 汇总 =="
if [ "$rc" -eq 0 ]; then
    echo "*** L2 PASS *** 全部 MODE/THR 组合在契约区逐像素 0 不符；报告：$REPORT"
else
    echo "*** L2 FAIL *** 存在不符，详见 $REPORT"
fi
rm -f sim/tb_l2.vvp
exit "$rc"
