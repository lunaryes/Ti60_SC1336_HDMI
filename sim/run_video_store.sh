#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p sim/build
verilator --binary --timing -Wno-fatal -Wno-lint \
  --top-module tb_ddr_video_store \
  Ti60F225_SC1336_HDMI_Display/src/image/ddr_video_store.v \
  sim/tb_ddr_video_store.sv --Mdir sim/build/obj_video -o Vtb_ddr_video_store
sim/build/obj_video/Vtb_ddr_video_store
