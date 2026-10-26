# RK3568 Buildroot C++ 6D deployment

The target has Buildroot glibc 2.35, OpenCV Python without ArUco and no ROS2. This executable uses cross-built OpenCV 4.5.5/contrib (static core/imgproc/calib3d/aruco), with static compiler runtimes and the board's installed librga shared library. It does not replace system OpenCV/libstdc++ and does not install ROS2.

Actual board results and suggested parameters are recorded in BOARD_VALIDATION.md.

## Build on Ubuntu22.04 x86_64

Toolchain: g++-aarch64-linux-gnu (GCC11), ARM64 glibc2.35 sysroot. Run `/home/llx/6D/native/build_cross.sh`. Downloads and source trees are under `/home/llx/6D/cross/`; build log is `/home/llx/6D/cross_build.log`. OpenCV source tags are 4.5.5; source tar SHA256 and package checksums should be retained with deployment records. librga header path comes from the previous RK3568 SDK; library is copied from the target /usr/lib/librga.so.2.1.0. Board self-test verifies this pairing, not just linkage.

## Run

For a persistent window on the board's existing Weston desktop, run `python3 view_live.py`.
Q/ESC or Ctrl+C stops it. This uses the installed Python OpenCV Qt/Wayland UI;
the C++ process still owns capture, RGA and detection. Preview is grayscale,
annotated with IDs and calibration status, refreshed every 3 processed frames.
It is a diagnostic viewer, not a zero-copy display or full-rate camera feed.
Default viewer logging goes to `/dev/null` to avoid filling storage; pass
`--output unique.jsonl` to record. `--frames 0` means continuous capture.
Continuous runs retain only the last 4096 timing samples for summary percentiles.

```sh
./run.sh --self-test
./run.sh --self-test --rga
./run.sh --frames 120 --output cpu.jsonl
./run.sh --rga --frames 120 --output rga.jsonl
./run.sh --rga --frames 120 --preview --output poses.jsonl
```

Use the same scene and camera mode, run serially, compare summaries and per-frame timings. P50/P95 are processing time (after dequeue through detection/PnP); output_fps includes capture waiting but is not sensor exposure-to-consumer latency. Discard warmup samples for serious benchmarking. `--threads 1/2/4` sets OpenCV threads; compare before choosing.

The input device must already be configured for NV12 on the single-plane V4L2 multiplanar API. The program validates format/geometry, mmaps four buffers, attempts DMA-BUF export, drains at most four completed buffers to retain the latest, and returns the active capture buffer only after synchronous preprocessing. CPU path resizes only Y. RGA resizes NV12 to NV12 and uses destination Y directly. Output memory is reused. Preview is an optional grayscale PGM saved every30 frames; visualization is excluded from processing_ms but affects output_fps.

RGA uses an exported source DMA fd when available, otherwise source virtual address; destination is CPU virtual memory. This is not an end-to-end zero-copy NPU pipeline. No RGB conversion needed for ArUco. Explicit --rga never silently falls back to CPU. Current RGA path may be slower than CPU for this task and must be measured. No NPU model is in this marker-based system.

## Metric pose

### Whole-board experiment (2026-10-25)

The optional `--charuco` mode uses the existing 7x5 DICT_4X4_50 board:
30 mm squares, 22 mm markers, OpenCV 4.5.x layout. It does not recalibrate.
RGA downsizes the image for marker detection, then marker corners are mapped
back to the original Y plane and refined. Intrinsics-aware ChArUco interpolation
and joint PnP run in original 3840x2160 coordinates. The V4L2 buffer remains
dequeued until refinement finishes; no full-frame CPU copy is needed.

```sh
cd /userdata/6D-native
./pose6d-board-v2 --rga --charuco --width 640 --height 360 --threads 2 \
  --calibration calibration/camera.yaml --marker-length 0.022 \
  --frames 180 --preview --output NEW_UNIQUE_NAME.jsonl
python3 analyze_board.py NEW_UNIQUE_NAME.jsonl
```

The extra JSON `board` field reports `valid`, corner count, 4K RMS,
`center_xyz_m` and `rvec_rad`. Translation is the physical board CENTER in
camera optical coordinates (x right, y down, z forward); the existing per-marker
output has its own marker-centered frame. Require at least 12 corners, positive
depth and RMS <=2 pixels. This is an initial quality gate, not an accuracy
guarantee. Missing/rejected boards have no valid translation. Planar pose
ambiguity remains; no temporal smoothing or robot-base transform is applied.

Keep camera and board still for the summary's standard deviation to measure
repeatability. It does not measure absolute accuracy. The existing manual
distance checks differed by about 8–15 mm; those errors have not been corrected
or disproved by small reprojection residuals. The calibration file is unchanged.

For calibration capture, create a fresh image directory and pass
`--snapshot-dir /userdata/6D-calibration/SESSION/images` to `view_live.py`.
Send `kill -USR1 PID_OF_POSE6D` to save the next original, unrotated 4K Y frame
as lossless PGM. Signals are coalesced; wait for `SNAPSHOT_SAVED` before the next
request. Preview/marker processing continues. Snapshot disk I/O affects timing;
do not benchmark while recording. Offline ChArUco calibration accepts PGM and
`--root`, `--square-mm`, `--marker-mm`; use measured printed dimensions.

By default the program only detects marker IDs; it never fabricates camera intrinsics. To enable metric pose:

```sh
./run.sh --rga --calibration camera.yaml --marker-length 0.08 --frames 300
```

camera.yaml is the JSON-compatible YAML produced by project tools. It must match the actual camera mode and use plumb_bob. Length is the marker black outer edge in metres (not ChArUco square length or paper size). All DICT_4X4_50 marker detections are assumed to have this physical size. Output XYZ and rvec are in the camera optical frame, not robot base frame. PnP rejects invalid/high reprojection error poses; planar/front-facing ambiguity still exists and real-world accuracy requires measurement. Camera intrinsics/hand-eye calibration and model-based RKNN are not supplied.

No autostart is installed. Ctrl+C stops capture and releases DMA fds/mappings. Use unique --output paths if results should be preserved; log files are truncated on start. Do not run two camera processes on the same video node. Neither input resolution nor existing media graph is changed automatically.
