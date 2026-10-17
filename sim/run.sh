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

# 1b) 平滑模式 1（弱平滑）/ 2（旁路）：tb 的 +SMOOTH 与参考模型的 --smooth
#     同步切档，验证 canny_edge 新加的三选一 MUX（2026-10-17 合并）。
#     图样取三种有代表性的：孤立脉冲（看平滑是否真的被旁路）、噪声、细线。
python3 sim/canny_ref.py \
    --exe "$BUILD/obj_dir/V$TOP" \
    --build "$BUILD" \
    --W "$IMG_W" --H "$IMG_H" --smooth 1 \
    impulse noise thin
python3 sim/canny_ref.py \
    --exe "$BUILD/obj_dir/V$TOP" \
    --build "$BUILD" \
    --W "$IMG_W" --H "$IMG_H" --smooth 2 \
    impulse noise thin

#---------------------------------------------------------------------
# 2) 顶层集成回归（M4）：延迟 / vsync 宽度 / mode 0 旁路 / mode 1 边缘图 /
#    mode 2·3 分屏。图样复用上面生成的同一套 stim_*.hex，纯 Verilog 自检。
#---------------------------------------------------------------------
verilator --binary --timing -Wno-fatal -Wno-lint \
    --top-module "$TOP2" \
    -GIMG_W="$IMG_W" -GIMG_H="$IMG_H" \
    "$SRC/rgb_to_gray.v" "$SRC/pix_window_3x3.v" "$SRC/median_filter_3x3.v" \
    "$SRC/canny_edge.v" \
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

#---------------------------------------------------------------------
# 5) 按键阈值控制器（2026-10-16 接入顶层后新增）：消抖窗口 / 按下沿 / 饱和 /
#    同时按 / uart 软写优先级 / 脉冲宽度，外加真实参数（74.25MHz × 20ms）下
#    "按下 → 阈值变化"的**实际拍数**。这一块现在是 Canny 阈值的唯一来源，
#    且 20 ms 在波形上没法肉眼核对，所以这里必须量。
#    这一段查 PASS 行：3)、4) 只看程序退出码，控制不住"自检没跑完就退出"。
#---------------------------------------------------------------------
verilator --binary --timing -Wno-fatal -Wno-lint \
    --top-module tb_canny_thr_key \
    "$SRC/../control/canny_threshold_key_ctrl.v" sim/tb_canny_thr_key.v \
    -o Vtb_canny_thr_key --Mdir "$BUILD/obj_thr_key"

out=$("$BUILD/obj_thr_key/Vtb_canny_thr_key" 2>&1)
echo "$out" | grep -v '^$'
if echo "$out" | grep -q '^CANNY_THR_KEY PASS'; then
    echo "按键阈值控制器（tb_canny_thr_key）：PASS"
else
    echo "按键阈值控制器（tb_canny_thr_key）：!! FAIL"
    exit 1
fi

#---------------------------------------------------------------------
# 6) 片内灰度直方图 + 串口回传（M6）：帧边界 / 同 bin 累加 / 778 字节流 /
#    校验和。参数用 16MHz/1Mbaud（DIV=16）缩短仿真到秒级；真实分频比
#    （74.25MHz/460800）由第 4 段的串口台覆盖。
#    这一段查 PASS 行：只看退出码控制不住"自检没跑完就退出"。
#---------------------------------------------------------------------
verilator --binary --timing -Wno-fatal -Wno-lint \
    --top-module tb_gray_hist_uart \
    "$SRC/../common/uart_tx.v" "$SRC/../common/gray_hist_uart.v" sim/tb_gray_hist_uart.v \
    -o Vtb_gray_hist_uart --Mdir "$BUILD/obj_hist"

out=$("$BUILD/obj_hist/Vtb_gray_hist_uart" 2>&1)
echo "$out" | grep -v '^$'
if echo "$out" | grep -q '^GRAY_HIST_UART PASS'; then
    echo "片内直方图（tb_gray_hist_uart）：PASS"
else
    echo "片内直方图（tb_gray_hist_uart）：!! FAIL"
    exit 1
fi

#---------------------------------------------------------------------
# 7) 3x3 中值滤波单元（2026-10-17 接入 Canny 前）：顶层集成台里 DUT 与参考链
#    是同一个模块，接错线能查，算法口径查不出来。这里同一组 9 像素同时喂
#    IMPULSE_ONLY=1/0 两个实例，验证 1 拍延迟、de=0 清零、孤立点替换阈值、
#    干净台阶边保持原值。这一段查 PASS 行（理由同 5、6 段）。
#---------------------------------------------------------------------
verilator --binary --timing -Wno-fatal -Wno-lint \
    --top-module tb_median_filter_3x3 \
    "$SRC/median_filter_3x3.v" sim/tb_median_filter_3x3.v \
    -o Vtb_median_filter_3x3 --Mdir "$BUILD/obj_median"

out=$("$BUILD/obj_median/Vtb_median_filter_3x3" 2>&1)
echo "$out" | grep -v '^$'
if echo "$out" | grep -q '^MEDIAN PASS'; then
    echo "中值滤波单元（tb_median_filter_3x3）：PASS"
else
    echo "中值滤波单元（tb_median_filter_3x3）：!! FAIL"
    exit 1
fi
