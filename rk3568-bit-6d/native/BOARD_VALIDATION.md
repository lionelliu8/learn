# RK3568 validation 2026-10-24

Target the target board; Buildroot glibc2.35; ARM64. Binary compiled in Ubuntu22.04 at /home/llx/6D/cross/deploy/pose6d, deployed under /userdata/6D-native. OpenCV4.5.5+contrib static, board system RGA API reports1.3.1, librga soname2. No system OpenCV or ROS2 installation changed.

SHA256SUMS verified on target. CPU and RGA synthetic tests each passed ID0 at rotations0/90/180/270 with known0.240m depth, estimated approximately0.24027m. This is an algorithm smoke test, not real camera calibration accuracy.

Real camera input: /dev/video0 RKISP mainpath, NV12 one-plane via multiplanar API, 3840x2160, stride3840, size12441600, four MMAP buffers, DMA-BUF source export succeeded. No real calibration supplied, no markers detected in the observed scene; only detection performance validated on real frames.

| Backend | Detection size | Threads | Frames | Output FPS | Processing P50 ms | Processing P95 ms |
|---|---|---:|---:|---:|---:|---:|
| CPU | 960x540 | 1 | 120 | 9.92 | 106.04 | 136.87 |
| RGA | 960x540 | 1 | 120 | 9.72 | 98.04 | 118.02 |
| RGA | 640x360 | 2 | 90 | 23.13 | 40.50 | 45.66 |
| RGA | 640x360 | 4 | 90 | 21.84 | 43.80 | 53.89 |
| CPU | 640x360 | 2 | 90 | 17.41 | 53.78 | 61.63 |

Sequential live-camera tests; scene and automatic exposure can change. These are observed throughput/processing statistics, not a controlled accuracy benchmark or camera-to-robot end-to-end latency. Initial-frame warmup included in executable summary; analyze_logs.py can discard first10 frames. Lower detection resolution can miss small markers. More threads or hardware acceleration alone do not guarantee speedup. No NPU task exists in this ArUco pipeline.

CPU/RGA preview images were retrieved and visually checked: both show the same physical scene with no geometric corruption. Current scene is very dark/noisy, so lighting/exposure should be improved before marker or calibration accuracy testing. RGA/CPU interpolation differs; observed FPS is not proof of equivalent marker recall.

Suggested live demo:

```sh
cd /userdata/6D-native
./run.sh --rga --width 640 --height 360 --threads 2 --frames 300 --output live.jsonl
```

For genuine metric pose add --calibration /actual/camera.yaml --marker-length 0.08 (replace with measured black marker edge). Do not use synthetic self-test K for the camera. Original calibration mode must match the actual ISP crop/focal mode. Preview: add --preview, get latest.pgm. Ctrl+C releases camera buffers. Results are not published via ROS2 on Buildroot; JSONL contains detections and, with calibration, XYZ/rvec and reprojection error in camera frame.
