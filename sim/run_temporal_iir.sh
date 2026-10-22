#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p sim/build
verilator --binary --timing -Wno-fatal -Wno-lint --top-module tb_temporal_history \
 Ti60F225_SC1336_HDMI_Display/src/image/gray_temporal_iir.v \
 Ti60F225_SC1336_HDMI_Display/src/image/temporal_ddr_history.v \
 Ti60F225_SC1336_HDMI_Display/src/image/temporal_axi_mux.v \
 sim/tb_temporal_history.sv --Mdir sim/build/obj_history -o Vtb_temporal_history
sim/build/obj_history/Vtb_temporal_history
verilator --binary --timing -Wno-fatal -Wno-lint --top-module tb_ddr_startup \
 Ti60F225_SC1336_HDMI_Display/src/axi/AXI4_AWARMux.v \
 sim/tb_ddr_startup.sv --Mdir sim/build/obj_startup -o Vtb_ddr_startup
sim/build/obj_startup/Vtb_ddr_startup
