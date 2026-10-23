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
Any nonzero antialiased edge coverage selects yellow. No new RAM or output
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
