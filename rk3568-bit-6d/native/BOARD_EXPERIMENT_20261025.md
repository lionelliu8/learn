# Whole-board pose experiment — 2026-10-25

Existing calibration frozen: SHA256
`d46815c4f2f3cdfc6cfe247a400acfcd87075839a464fe172035b7d244805459`.
No refit, scale compensation, depth offset, or smoothing was applied.

Build: Ubuntu cross/native-build, C++ OpenCV 4.5.5; deployment:
`/userdata/6D-native/pose6d-board-v2`. Original `pose6d` preserved.
Both builds passed the existing four-rotation RGA marker self-test.
New whole-board path tested with real camera frames; legacy marker mode also
completed a 15-frame regression run. No startup service was changed.

| Run | Frames | Valid after 10 warmup | Mean valid 4K RMS | Output FPS | Processing p50/p95 |
|---|---:|---:|---:|---:|---:|
| 01 (full Y copy) | 180 | 161/170 | 0.826 px | 14.87 | 64.29/72.08 ms |
| 02 (no board in view) | 180 | 0/170 | N/A | 24.94 | 35.17/45.18 ms |
| 03 (no Y copy) | 240 | 194/230 | 0.948 px | 15.12 | 60.17/77.41 ms |

Raw logs are on board at `/userdata/6D-native/board-test-20261025-0N.jsonl`
and on Windows under `.review/` with the same filenames. Regenerate statistics
with `python3 native/analyze_board.py PATH_TO_LOG`.

These are different scene placements, not a controlled speed comparison.
Run 03 center XYZ standard deviations were 5.16/11.32/7.24 mm, with peak-to-peak
25.65/53.02/23.81 mm. Physical motion and algorithm noise were not independently
separated; do not advertise these values as intrinsic sensor repeatability.
Accepted-frame RMS excludes rejected frames and is not absolute accuracy.
84.3% valid rate means this is an operational experimental mode, not yet a
stable production pose source. The earlier 8–15 mm manual-distance discrepancies
remain unresolved. No claim of millimeter accuracy or maximum hardware
performance is supported.

Next controlled experiment: mechanically fix camera and board, record an
unchanged pose, inspect rejected frames and spatial corner residuals, then
evaluate translation/rotation repeatability. Only afterwards compare against
an independently measured geometric reference. This system estimates the known
printed board pose, not arbitrary unmarked object pose.
