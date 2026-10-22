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
