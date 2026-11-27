#!/usr/bin/env bash
# Builds the parallel Phase 3 Sensor-Agnostic AV-SLAM Linux/Jetson app & test suite
# Runs on branch feature/sensor-agnostic-slam without disturbing main (ScanAR production baseline).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build/agnostic_linux"

cmake -S "${REPO_ROOT}" -B "${BUILD_DIR}" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DAVSLAM_BUILD_DESKTOP_APP=ON \
    -DAVSLAM_BUILD_IOS_APP=OFF \
    -DAVSLAM_BUILD_TESTS=ON \
    -DAVSLAM_BUILD_AB_BENCHMARK=OFF

cmake --build "${BUILD_DIR}" --target AVSlamDesktop avslam_agnostic_test avslam_cslam_depth_test avslam_tests -j"$(nproc)"
"${BUILD_DIR}/avslam_agnostic_test"
"${BUILD_DIR}/avslam_cslam_depth_test"
echo "Built parallel Sensor-Agnostic AV-SLAM Linux/Jetson app: ${BUILD_DIR}/AVSlamDesktop"
