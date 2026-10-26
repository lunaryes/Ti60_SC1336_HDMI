# SC1336 exposure trial: 900 half-lines

Branch: codex/sc1336-exposure-900. Base: e38384c (no temporal IIR,
coverage-weighted yellow overlay). Only the sensor exposure values change:

| Register | Before | Trial |
| --- | --- | --- |
| 3E01 | 5D | 38 |
| 3E02 | 80 | 40 |

The existing 3E00 configuration/reset assumption is unchanged. With 3E00=0,
exposure code changes from 0x5D8 (1496) to 0x384 (900 half-lines).
The proposed 10 ms duration is conditional on actual sensor half-line timing;
it has not been measured here. This is not a guarantee of flicker elimination
for arbitrary lamp PWM frequencies or display refresh patterns.

Analog gain (3E08=1F), white balance, Canny threshold (12), spatial smoothing,
image processing and display alignment are unchanged. Expected unsaturated
linear exposure ratio is 900/1496=60.16% (about 39.84% lower, not 60% lower).

Purpose: compare horizontal banding, false horizontal edges, contour continuity
and dark/low-contrast target detection under the same light and camera position.
No board programming or visual validation is included in the bitstream build.
To restore prior exposure, return to codex/shape-ocr-core or revert the two LUT
values; reloading the previous FPGA configuration initializes the original LUT.
