# Shape recognition rewrite

Base: codex/shape-ocr-core at a60c493. Replaces the former OCR profile with shape-only recognition. The 16 obsolete character, template matcher and combined overlay modules have been deleted.

## Implementation

The camera, DDR/IIR, Canny and HDMI paths are preserved. Capture pools 8x8 cells using an edge count with a default threshold of four pixels, replacing single-pixel OR pooling. Thin contours grazing a cell corner can be lost; MIN_EDGE_PIXELS controls this tradeoff.

The geometry engine uses eight-connected components, row spans, area extent, four normalized quadrants and sampled row widths instead of morphological closing, Moore tracing and RDP vertex counts. Supported classes: triangle, square, rectangle, circle, ellipse, pentagon, plus and diamond (class 9, not a general quadrilateral). Unmatched regions are rejected. Dense solid shapes can use single-run rows. Per-pixel component tags isolate nested and previously scanned objects.

Two or more endpoints reject a component. One is allowed for rasterized acute triangle tips. This rejects the tested open U contour, but is not a general proof of closure. Queue exhaustion or an oversized component aborts that analysis with an empty result. A four-million-cycle watchdog bounds processing.

Three matching analyses confirm a track. An already confirmed track tolerates one missed analysis and expires on the second. Analyses may skip video frames while busy. UART ocr_enable is retained as an interface name and now controls only shape annotations.

## Validation

Run: python sim/shape_regression/run.py

Icarus Verilog is required. Tools are found on PATH or through IVERILOG_BIN; this machine also has a fallback at F:/codex/tools/iverilog/bin. Generated test vectors and executables are ignored.

27 geometry cases passed, including all eight classes, multiple positions, three triangle orientations, solid regions, blank/noise/line/open-U rejection, oversized clutter, nested and separate objects, and internal speckles. Separate capture and tracker simulations passed thresholding, frame replacement, busy exclusion, confirmation, gaps, class changes and expiry. Shape top and overlay compile together; all 50 project source references exist.

On 2026-10-23, Efinity 2026.1.132 completed map, interface, placement/routing and bitstream generation from these sources. Map RAM usage: 171/256 blocks. Final analyzed clock relationships have positive setup and hold slack; pixel-clock setup slack is 3.158 ns and hold slack 0.027 ns. Unmatched CSI constraints remain, so these results do not establish complete timing coverage.

The generated outflow_shape_program/Ti60_Demo.bit was successfully downloaded to the Ti60 via JTAG SRAM at 6 MHz; device ID 0x10660A79. Flash was not written. The user reported a positive result after programming. No measured camera false-positive rate is claimed. Arbitrary rotation, perspective, blur and overlapping shapes are not guaranteed by the synthetic tests.

Build log: Ti60F225_SC1336_HDMI_Display/build_shape_program.log (local, reproducible). Historical build_gap.log predates this rewrite and is not the programming build log.
