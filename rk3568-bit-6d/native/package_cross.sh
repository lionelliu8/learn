#!/usr/bin/env bash
set -euo pipefail
ROOT="${POSE6D_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
DEPLOY="$ROOT/cross/deploy"
aarch64-linux-gnu-strip --strip-unneeded "$DEPLOY/pose6d"
cp "$ROOT/cross/sources/opencv-4.5.5/LICENSE" "$DEPLOY/LICENSE.opencv"
cp "$ROOT/cross/sources/opencv_contrib-4.5.5/LICENSE" "$DEPLOY/LICENSE.opencv_contrib"
tar -cf "$DEPLOY/source.tar" -C "$ROOT" native src/monocular_6d_pose/include
(cd "$DEPLOY" && sha256sum pose6d source.tar > SHA256SUMS)
tar -cf "$ROOT/6D_native_aarch64.tar" -C "$DEPLOY" .
ls -lh "$ROOT/6D_native_aarch64.tar"
