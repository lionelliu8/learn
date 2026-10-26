#!/usr/bin/env bash
set -euo pipefail
ROOT="${POSE6D_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
CROSS="$ROOT/cross"
exec > "$ROOT/cross_build.log" 2>&1
mkdir -p "$CROSS/sources" "$CROSS/rga/lib" "$CROSS/opencv-build" "$CROSS/native-build"
if [ ! -d "$CROSS/sources/opencv-4.5.5" ]; then
 tar -xzf "$CROSS/opencv-4.5.5.tar.gz" -C "$CROSS/sources"
fi
if [ ! -d "$CROSS/sources/opencv_contrib-4.5.5" ]; then
 tar -xzf "$CROSS/opencv_contrib-4.5.5.tar.gz" -C "$CROSS/sources"
fi
cmake -S "$CROSS/sources/opencv-4.5.5" -B "$CROSS/opencv-build" \
 -DCMAKE_TOOLCHAIN_FILE="$ROOT/native/aarch64.cmake" \
 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$CROSS/opencv-install" \
 -DOPENCV_EXTRA_MODULES_PATH="$CROSS/sources/opencv_contrib-4.5.5/modules" \
 -DBUILD_LIST=core,imgproc,calib3d,aruco -DBUILD_SHARED_LIBS=OFF \
 -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF \
 -DBUILD_opencv_apps=OFF -DBUILD_opencv_python2=OFF -DBUILD_opencv_python3=OFF \
 -DBUILD_JAVA=OFF -DWITH_OPENCL=OFF -DWITH_IPP=OFF -DWITH_ITT=OFF \
 -DWITH_TBB=OFF -DWITH_OPENMP=OFF -DWITH_FFMPEG=OFF -DWITH_GSTREAMER=OFF \
 -DWITH_GTK=OFF -DWITH_QT=OFF -DWITH_V4L=OFF -DWITH_LAPACK=OFF \
 -DWITH_EIGEN=OFF -DBUILD_ZLIB=ON -DWITH_PROTOBUF=OFF \
 -DWITH_JPEG=OFF -DWITH_PNG=OFF -DWITH_TIFF=OFF -DWITH_WEBP=OFF -DWITH_OPENEXR=OFF \
 -DCPU_BASELINE=NEON -DCPU_DISPATCH= -DENABLE_PRECOMPILED_HEADERS=OFF
cmake --build "$CROSS/opencv-build" --parallel 3
# OpenCV 4.5.5 exports quirc/ADE even with objdetect/GAPI excluded.
# Build these small archives so find_package can validate imported targets.
cmake --build "$CROSS/opencv-build" --target quirc ade --parallel 3
cmake --install "$CROSS/opencv-build"
cmake -S "$ROOT/native" -B "$CROSS/native-build" \
 -DCMAKE_TOOLCHAIN_FILE="$ROOT/native/aarch64.cmake" -DCMAKE_BUILD_TYPE=Release \
 -DOpenCV_DIR="$CROSS/opencv-install/lib/cmake/opencv4" \
 -DRGA_INCLUDE_DIR="$CROSS/rga/include" -DRGA_LIBRARY="$CROSS/rga/lib/librga.so" \
 -DCMAKE_INSTALL_PREFIX="$CROSS/deploy"
cmake --build "$CROSS/native-build" --parallel 3
cmake --install "$CROSS/native-build"
file "$CROSS/deploy/pose6d"
aarch64-linux-gnu-readelf -d "$CROSS/deploy/pose6d"
sha256sum "$CROSS/deploy/pose6d" > "$CROSS/deploy/SHA256SUMS"
echo CROSS_BUILD_SUCCESS
