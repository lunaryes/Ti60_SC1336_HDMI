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
TB_TOP=sim/tb_pipeline_top.v
TOP2=tb_pipeline_top
IMG_W=${IMG_W:-64}
IMG_H=${IMG_H:-32}

mkdir -p "$BUILD"

#---------------------------------------------------------------------
# 1) canny_edge 算法回归（Verilator + Python 参考模型逐像素比对）
#---------------------------------------------------------------------
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

#---------------------------------------------------------------------
# 2) 顶层集成回归（M4）：延迟 / vsync 宽度 / mode 0 旁路 / mode 1 边缘图 /
#    mode 2·3 分屏。图样复用上面生成的同一套 stim_*.hex，纯 Verilog 自检。
#---------------------------------------------------------------------
verilator --binary --timing -Wno-fatal -Wno-lint \
    --top-module "$TOP2" \
    -GIMG_W="$IMG_W" -GIMG_H="$IMG_H" \
    "$SRC/rgb_to_gray.v" "$SRC/pix_window_3x3.v" "$SRC/canny_edge.v" \
    "$SRC/image_pipeline_top.v" "$SRC/../display/image_mix.v" "$TB_TOP" \
    -o "V$TOP2" --Mdir "$BUILD/obj_top"

if [ "$#" -gt 0 ]; then
    STS=(); for n in "$@"; do STS+=("$BUILD/stim_$n.hex"); done
else
    STS=("$BUILD"/stim_*.hex)
fi

npass=0; nfail=0
for s in "${STS[@]}"; do
    name=$(basename "$s" .hex); name=${name#stim_}
    out=$("$BUILD/obj_top/V$TOP2" +STIM="$s" 2>&1)
    if echo "$out" | grep -q '^TOP PASS'; then
        npass=$((npass + 1))
        printf '  %-12s OK\n' "$name"
    else
        nfail=$((nfail + 1))
        printf '  %-12s !! FAIL\n' "$name"
        echo "$out" | sed 's/^/      /'
    fi
done
echo "顶层集成（tb_pipeline_top）：$npass 个图样通过，$nfail 个失败"
[ "$nfail" -eq 0 ] || exit 1
