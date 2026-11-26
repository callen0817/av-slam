#!/usr/bin/env bash
# Builds the native Linux / NVIDIA Jetson AV SLAM Desktop application (AVSlamDesktop)
# Prerequisites (Ubuntu / JetPack):
#   sudo apt-get update && sudo apt-get install -y build-essential cmake ninja-build git libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build/desktop_linux"

cmake -S "${REPO_ROOT}" -B "${BUILD_DIR}" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DAVSLAM_BUILD_DESKTOP_APP=ON \
    -DAVSLAM_BUILD_IOS_APP=OFF \
    -DAVSLAM_BUILD_TESTS=OFF \
    -DAVSLAM_BUILD_AB_BENCHMARK=OFF

cmake --build "${BUILD_DIR}" --target AVSlamDesktop -j"$(nproc)"
echo "Built Linux/Jetson AV SLAM Desktop: ${BUILD_DIR}/AVSlamDesktop"
