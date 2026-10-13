#!/usr/bin/env bash
#=====================================================================
# canny_edge 仿真回归：Verilator 编 RTL + Python 参考模型逐像素比对
#
#   用法： bash sim/run.sh              # 全部图样
#          bash sim/run.sh lines noise  # 只跑指定图样
#
#   依赖： verilator >= 5.0、python3（本工程在 WSL 里跑）
#   产物： sim/build/（激励、输出、obj_dir；可随时删除，不要提交）
#
#   图样与参考模型都在 sim/canny_ref.py 里；两个文件的近似必须与 RTL
#   完全一致，否则比对必然不一致（方案 §8.1）。
#=====================================================================
set -euo pipefail
cd "$(dirname "$0")/.."

SRC=Ti60F225_SC1336_HDMI_Display/src/image
TB=sim/tb_canny_edge.v
BUILD=sim/build
TOP=tb_canny_edge
IMG_W=${IMG_W:-64}
IMG_H=${IMG_H:-32}

mkdir -p "$BUILD"

verilator --binary --timing -Wno-fatal -Wno-lint \
    --top-module "$TOP" \
    -GIMG_W="$IMG_W" -GIMG_H="$IMG_H" \
    "$SRC/pix_window_3x3.v" "$SRC/canny_edge.v" "$TB" \
    -o "V$TOP" --Mdir "$BUILD/obj_dir"

python3 sim/canny_ref.py \
    --exe "$BUILD/obj_dir/V$TOP" \
    --build "$BUILD" \
    --W "$IMG_W" --H "$IMG_H" \
    "$@"
