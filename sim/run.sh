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
# 0) 串行保护：sim/build 是**共享的可变工作目录**（激励、DUT 输出、编译产物
#    全在里面），两个 run.sh 同时跑会互相覆盖 stim_*.hex。症状极具迷惑性：
#    尺寸不同的那一路报 `$readmem file address beyond bounds`（大图样被小
#    图样覆盖），看起来像激励生成器坏了、像 RTL 越界，唯独不像"并发"。
#    用 mkdir 做原子锁（比 flock 可移植）。
#---------------------------------------------------------------------
LOCK="$BUILD/.lock"
if ! mkdir "$LOCK" 2>/dev/null; then
    echo "!! 另一个 sim/run.sh 正在跑（锁目录 $LOCK 已存在）。"
    echo "   sim/build 不能并发使用，等它跑完；确认没有在跑就 rm -rf $LOCK"
    exit 1
fi
trap 'rmdir "$LOCK" 2>/dev/null || true' EXIT

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

#---------------------------------------------------------------------
# 3) 灰度链颜色路径（M5）：定点点值 + BT.601 容差 + 消隐清零
#    集成回归灌的是 {g,g,g}，颜色路径在那里从未被激励过（见 tb_rgb_to_gray.v）
#---------------------------------------------------------------------
verilator --binary --timing -Wno-fatal -Wno-lint \
    --top-module tb_rgb_to_gray \
    "$SRC/rgb_to_gray.v" sim/tb_rgb_to_gray.v \
    -o Vtb_rgb_to_gray --Mdir "$BUILD/obj_gray"

"$BUILD/obj_gray/Vtb_rgb_to_gray"

#---------------------------------------------------------------------
# 4) 串口命令口（M5）：协议 + 分频 + 帧解析。用真实参数（74.25MHz/460800）
#    这是全工程唯一"上板后没法用波形排查"的新逻辑，先在仿真里跑通协议。
#---------------------------------------------------------------------
verilator --binary --timing -Wno-fatal -Wno-lint \
    --top-module tb_uart_cmd \
    "$SRC/../common/uart_cmd.v" sim/tb_uart_cmd.v \
    -o Vtb_uart_cmd --Mdir "$BUILD/obj_uart"

"$BUILD/obj_uart/Vtb_uart_cmd"
