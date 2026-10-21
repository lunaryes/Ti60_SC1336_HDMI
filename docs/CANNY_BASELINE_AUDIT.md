# Canny baseline preservation audit

Compared the on-disk project `E:\fpga\Ti60_SC1336_HDMI` against the Git object
`cdd6a8e` (2026-10-17 stable Canny branch) on 2026-10-21. The V3 staging area
does not carry replacements for any of these six Canny/display source files.

The hashes below are SHA-256 of the bytes read directly from `git show
cdd6a8e:Ti60F225_SC1336_HDMI_Display/<path>`, compared against the on-disk
file after replacing CRLF line endings with LF. All six are identical after
line-ending normalization; the raw-file hashes are reported separately so
line-ending differences are not confused with source changes.

| Path | Baseline SHA-256 and current LF-normalized SHA-256 | Same |
| --- | --- | --- |
| `src/image/canny_edge.v` | `1B54DD7C712E442A95410545326A3D9509CC1ED7839F53A997468733281A5C3F` | yes |
| `src/image/image_pipeline_top.v` | `1F024E99C34473C3F27F0E089B76E50D03A9B78E7DC4A2F30B9C13C51A81E4C2` | yes |
| `src/image/pix_window_3x3.v` | `2897F4BEEC98B6FF4D36C9E659AB010D25FAB352BA27D95CC133750D1EAB1D18` | yes |
| `src/image/median_filter_3x3.v` | `5A3531426DCC4148750839DAECC35EA3E748E1DFCAE299C7D810B4DE521B97CE` | yes |
| `src/image/rgb_to_gray.v` | `B7A9D07B2CEEBBBD9EDC5A2EA47D0092A6CB8A07F33EBFB7971677F7D3D56B1E` | yes |
| `src/display/image_mix.v` | `98B6C3D3565DA3549B5D9EFC2522138EE51585DF3716B28C6E0D93DC97A865A9` | yes |

Current raw-file SHA-256 values:

| Path | Raw SHA-256 |
| --- | --- |
| `src/image/canny_edge.v` | `8A030F7DD59D479470A1755F74C4FAABAA63F931D7D3CD944327FDEA8B53C3FE` |
| `src/image/image_pipeline_top.v` | `E5E68BBF7546E21C582CC40C2A7A896970D2F4C8C6F7C04E885A958E2A7CC58D` |
| `src/image/pix_window_3x3.v` | `E71ADD2023DA0D375B1F5538DC04840ADEA5A1755CB7473860F0435446A383B7` |
| `src/image/median_filter_3x3.v` | `20E6EC4DA5403DF7B3DC385D9E1D6F29C22600A58CEEB5B4634E548A013084E7` |
| `src/image/rgb_to_gray.v` | `B7A9D07B2CEEBBBD9EDC5A2EA47D0092A6CB8A07F33EBFB7971677F7D3D56B1E` |
| `src/display/image_mix.v` | `3B71C0706CD3D4537378AD14D3C45CED697ECC4963AB30C3E24AE5446AA05201` |

The existing hardware defaults also match the baseline: high threshold 12,
low threshold `high >> 1`, smoothing mode 1, button step 1, and image latency
19 clocks. OCR is a read-only consumer of the median grayscale tap.

These source checks do not prove that a live camera scene, exposure or prior
UART/button settings match a previous demonstration. Reprogramming resets
runtime settings. Compare the same stationary scene with identical UART
readback settings before attributing a visual difference to the Canny core.

`sim/tb_overlay_passthrough.v` separately verifies RGB isolation: complete
1280x720 frames in three candidate/recognition states; disabled overlays and
blanking RGB pass through unchanged, and enabled annotations cannot change
pixels outside documented borders, icons and status rectangles. The overlay
does not own DE, HSYNC or VSYNC, so synchronization bypass is structural.

Validation result with Icarus Verilog (`iverilog -g2012 -Wall`, `vvp`):

```text
OVERLAY PASSTHROUGH PASS: frames=3 active_pixels=2764800 protected_checks=5488997 annotated_pixels=37724
```

Compilation produced no warnings. The checks also cover a stale
`recognized_i=1` with `bbox_valid_i=0`: no green template or old candidate
box is rendered. These are RTL checks; hardware scene/exposure verification
still requires a stationary input scene and identical live settings.
