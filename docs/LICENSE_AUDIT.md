# AV SLAM — Open-Source & Commercial IP Licensing Audit

## 1. Executive Summary
**AV SLAM** (`avslam_core` and `AVSlam.app`) is **100% first-party proprietary C++17 and Objective-C++ software** owned by Artificial Vision Inc.

- **Zero GPLv2 / GPLv3 / LGPL / AGPL code** is compiled or linked into `avslam_core` or the shipping iOS / Jetson / Desktop binaries.
- **Zero upstream code** from `hku-mars/FAST-LIVO2`, `FAST-LIO`, `ikd-Tree`, `rpg_vikit`, `PCL`, `OpenCV`, `GTSAM`, or `ROS` is included or linked.
- The legacy `FastLivo2Engine.cpp` file exists strictly for optional host-only regression A/B benchmarking (`avslam_ab_benchmark`, disabled in all shipping iOS and commercial builds via `-DAVSLAM_BUILD_AB_BENCHMARK=OFF`).

---

## 2. Component-by-Component IP & License Matrix

| Subsystem | Implementation | Origin / License | Commercial App Store & Enterprise Status |
| :--- | :--- | :--- | :--- |
| **AV-SLAM Core Engine** | `AvSlamEngine`, `AvImuPreintegrator`, `AvSurfelMap` | **100% First-Party Proprietary C++17** | **Full Proprietary Commercial Ownership** |
| **Point Cloud Filter & Cleanup Suite** | `PointCloudFilters`, `PointCloudCleanup` | **100% First-Party Proprietary C++17** | **Full Proprietary Commercial Ownership** |
| **Hardware-Agnostic Sensor Layer** | `SensorRigProfile`, `ISensorStreamAdapter`, `AiryDecoder/Discovery/UdpReceiver` | **100% First-Party Proprietary C++17** (Public UDP specs) | **Full Proprietary Commercial Ownership** |
| **Lie Group & Linear Algebra** | `include/avscan/math/SophusLie.hpp` ($SO(3), SE(3)$, Jacobi Eigen, Cholesky $LL^T$) | **100% First-Party Proprietary C++17** | **Full Proprietary Commercial Ownership** |
| **Targetless & Online Auto-Calibration** | `TargetlessCalibrator`, `OnlineAutoCalibrator` | **100% First-Party Proprietary C++17** | **Full Proprietary Commercial Ownership** |
| **Loop Closure, GICP & Pose Graph Backend** | `ScanContext`, `GicpVerifier`, `PoseGraphOptimizer` | **100% First-Party Proprietary C++17** | **Full Proprietary Commercial Ownership** |
| **Terrestrial & Hybrid Flash Mode** | `TerrestrialStationManager`, `RotationGeometry` | **100% First-Party Proprietary C++17** | **Full Proprietary Commercial Ownership** |
| **Point Cloud & Spatial Exporters** | PLY, PCD v0.7, ASTM E2807-11 E57, AV Walkthrough Bundle | **100% First-Party Proprietary C++17** | **Full Proprietary Commercial Ownership** |
| **iOS Native App & Metal Renderer** | `ios/AVScan/` (`Metal`, `AVFoundation`, `UIKit`, `ARKit`, `CoreMotion`) | **First-Party ObjC++ / Apple SDK License** | **Commercial App Store & Enterprise Safe** |
