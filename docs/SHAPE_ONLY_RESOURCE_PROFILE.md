> Current profile update: temporal IIR has been removed. Median feeds Canny directly; application IMG_LAT=24. Historical IIR/resource records below refer to earlier revisions.

# Shape-only resource profile

The active Ti60_Demo.xml project builds shape recognition, Chinese shape labels,
Canny, DDR frame storage and temporal IIR. Digit and uppercase-letter detection
are disabled by removing their top-level instances and project source entries.
All 25 recognition RTL source files remain unchanged in src/recognition.

The shape overlay receives geometry_valid directly. Character detection,
character overlay, object fusion and the legacy OCR status overlay are absent.
The video timing path is unchanged; the overlay is combinational.
UART E followed by binary 0/1 disables/enables shape annotations (default on).
UART O retains its DDR replay-divider behavior.

## Synthesis validation

Efinity 2026.1, Ti60F225, example_top, map PASS (100 seconds).

| Resource | Combined profile | Shape-only profile |
| --- | ---: | ---: |
| RAM10 | 307 | 184 / 256 (71.875%) |
| LUT4 | 42940 | 21929 |
| FF | 21242 | 12235 |
| ADD | 5212 | 4258 |
| SRL8 | 632 | 632 |
| DSP48 | 80 | 71 |
| DSP24 | 24 | 24 |

These are synthesis primitive counts, not final placed XLR/DSP block counts.
Reports are generated under outflow/ and excluded from Git. The earlier combined
synthesis reports are preserved locally under outflow/before_shape_only/.

## Restoring character recognition

Commit 81f2676 contains the combined top-level wiring and complete recognition
source registration. Restore the character-related instances and source entries
from that revision when re-enabling OCR; preserve subsequent unrelated changes.
Do not restore the full combined implementation unchanged for Ti60: its 307
RAM10 blocks exceed the 256-block device budget. Optimize the 320x180 character
capture, label map and component statistics before re-enabling it. The shared
shape RAM, tracker, font and label modules remain registered in the active project.

## Physical implementation validation

Interface generation PASS; placement and routing PASS (178 seconds).
Packed resources: XLR 31706/60800 (52.15%), RAM 184/256 (71.88%),
DSP blocks 87/160 (54.38%). All analyzed setup and hold clock relationships
have positive slack. clk_pixel setup slack is 2.824 ns, hold slack 0.026 ns;
clk_sys setup slack is 3.817 ns. Across the reported clock relationships,
minimum setup slack is 0.309 ns and minimum hold slack is 0.026 ns.
Existing unmatched CSI port/clock SDC warnings remain; these results do not
establish timing coverage for unmatched constraints. No board validation or
bitstream generation was performed in this optimization run.

Reports: outflow/Ti60_Demo.place.rpt and outflow/Ti60_Demo.timing.rpt.

## Default display mode

Display mode M=3 is now left-half grayscale and right-half original RGB with
bright yellow (FFFF00) edges. It uses corresponding halves of the full frame;
it does not scale two complete images side by side. Mode 3 forces the edge tap
regardless of V so grayscale display selection cannot mask the entire original.
Antialiased edge coverage blends yellow with original RGB (alpha/255). No new RAM or output
pipeline stage is added. Modes 0/1/2 retain original/edge/gray-edge split behavior.
The shape annotations remain controlled by UART E.

Select using `python sim/video_control.py --port COM6 --layout overlay` or send
binary bytes `4D 03`. This is also the FPGA reset default. These display changes
require rebuilding the bitstream and downloading it before they affect the board.

## Overlay spatial alignment correction

A full-pipeline horizontal step regression exposed a four-row RGB/edge offset:
the uncorrected original became white on output row 21 while yellow edges
appeared on rows 24/25. Median grayscale became white on row 22.
Mode 3 now delays original RGB by four active rows and median grayscale by
three active rows before the common AA renderer. The corrected grayscale
transition is row 25 and the edge straddles rows 24/25. Vertical step edges
remain on columns 43/44. Measurements use a 64x48 input step at row20/column44.

Synchronous read-before-write line stores infer BRAM. Frame-start validity
suppresses previous-frame RAM contents; unavailable top rows are black.
The renderer adds its existing common one-row displacement to every stream.
Modes 0/1/2 retain their spatial behavior. All display modes gain one common
clock for synchronous line reads, so top-level IMG_LAT changes from 26 to 27.
The recognizer taps and DDR/IIR control paths are unchanged.

Regression: sim/tb_overlay_alignment.sv, run with and without +VERTICAL.
Synthesis PASS: RAM10 212/256 (82.81%), LUT4 22017, FF 12345;
DSP48 71 and DSP24 24. The added 28 RAM10 blocks are the physical mapping
of 4x1280x24 RGB and 3x1280x8 gray active-pixel line stores.

Full compile (map/interface/pnr/pgm) PASS for the spatial correction. All reported
setup/hold clock relationships are positive; clk_pixel setup slack 2.229 ns,
hold slack 0.026 ns. Existing unmatched CSI SDC warnings remain. JTAG download
was attempted but no USB target was detected; board visual verification is pending.

## Selective cloud shape rewrite integration

Integrated the shape-recognition changes from origin/codex/shape-ocr-core
commit 9b3a2f3 on top of local display/alignment commit 71bb59a.
Only shape_frame_capture.v, shape_geometry_engine.v and shape_geometry_top.v
were imported. Capture uses an 8x8 edge-count threshold (default 4); geometry
uses eight-connected components, spans, extent, quadrants and sampled widths.
Supported classes are triangle, square, rectangle, circle, ellipse, pentagon,
plus and diamond. Three matching analyses confirm a shape track.

All 25 recognition RTL files remain in the repository. The cloud deletion of
character RTL was intentionally excluded. The existing shared Chinese-label
interface and template ROM registration are retained for source compatibility.
The shape class IDs are unchanged. Display mode 3, yellow edges, four/three-row
RGB/grayscale compensation, IMG_LAT=27 and DDR/IIR paths remain unchanged.

The cloud regression was extracted to ignored outflow/remote_shape_tests and
adapted to local Verilator. All 27 geometry cases, capture threshold/frame/busy
checks and tracker confirmation/gap/class-change/expiry checks passed. Tests,
fixtures and generated artifacts from the cloud commit were not imported.

Merged-profile Efinity map PASS (239 seconds), RAM10 199/256 (77.73%),
compared with 212 blocks before the rewrite. This integration has not yet been
placed/routed, converted to a new bitstream, or programmed. Earlier timing and
board results above refer to their explicitly named earlier revisions.

## Temporal IIR removal (2026-10-26)

Removed temporal_ddr_history, gray_temporal_iir and temporal_axi_mux from the
active project. RAW frame storage connects directly to the calibrated DDR
interface. The three-clock external temporal path is removed from the image
pipeline; HDMI HS alignment changes from 27 to 24 clocks. Four/three-row overlay
alignment is unchanged. Spatial median and Canny smoothing remain.

RAW frame freezing, history age selection, replay, comparison and difference
are retained. UART Q keeps its eight-byte format with byte2 reserved zero.
Legacy N/Y/Z parameter frames are consumed without effect; IIR status/counter
and tuning queries no longer respond. The control tool no longer sends them.
Old IIR RTL and standalone tests are retained only as historical sources.

Validation: Efinity 2026.1 map PASS (127 seconds), RAM10 181/256 (70.70%),
LUT4 21582, FF 13637, DSP48 32, DSP24 30. Before removal RAM10 was 197.
UART regression: 66 cases PASS. Horizontal and vertical full-pipeline overlay
checks PASS, including exact 24-clock VS/DE latency. No placement/routing,
bitstream generation or board programming has been performed for this change.

## Coverage-weighted yellow overlay

Mode3 now computes round((original*(255-alpha)+yellow*alpha)/255), using
8-bit AA coverage alpha and yellow=FFFF00. Zero coverage preserves original RGB
exactly; full coverage produces exact yellow. Integer constant-division reduction
uses rounded products rather than a general divider. Three 8x8 products are
combinational; no new RAM, output stage or UART command is added.

Validation: tb_yellow_coverage exhaustively checks 65536 pixel/alpha combinations
against independent integer /255 arithmetic, plus left grayscale and timing.
Horizontal/vertical full-pipeline overlay and 24-clock VS/DE regressions PASS.
This change has not been synthesized, placed/routed or programmed yet; earlier
resource reports are for the previous hard-yellow overlay revision.
