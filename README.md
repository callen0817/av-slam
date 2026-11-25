# AV SLAM — Proprietary Multi-Sensor Mobile & Cross-Platform 3D SLAM Engine

[![Engine](https://img.shields.io/badge/Engine-AV--SLAM%20Proprietary%20v2.0-gold)](docs/AV_SLAM_ARCHITECTURE.md)
[![License](https://img.shields.io/badge/IP-100%25%20First--Party%20Zero--GPL-brightgreen)](docs/LICENSE_AUDIT.md)
[![Platform](https://img.shields.io/badge/Platforms-iOS%20%7C%20Jetson%20%7C%20Desktop-orange)](#cross-platform-core)
[![BundleID](https://img.shields.io/badge/iOS%20Bundle-com.artificialvision.avslam-blue)](#side-by-side-testing)

**AV SLAM** is Artificial Vision's proprietary, zero-GPL, zero-external-dependency C++17 SLAM engine and native iOS/iPadOS application designed for **hardware-agnostic external 3D LiDAR + IMU + RGB camera + RTK GNSS reality capture**.

It runs as a **parallel side-by-side product** alongside `AV_iOS` (`com.artificialvision.avscan`), allowing direct live field A/B comparisons on the exact same iPad/iPhone without modifying `AV_iOS`.

---

## Core Proprietary Innovations (`AV-SLAM v2.0`)

1. **On-Manifold Lie-Group IMU Preintegration (`AvImuPreintegrator`)**
   - Compresses 200 Hz IMU packets into compact $(\Delta R, \Delta v, \Delta p)$ factors with exact analytical bias Jacobians and block-sparse 3x3 covariance updates, paired with O(1)/point piecewise-linear twist motion compensation.
2. **Tightly-Coupled 17-DOF Hybrid Manifold Cholesky MAP Solver + 3-Level Visual Fusion (`AvSlamEngine`)**
   - Operates on $SO(3) \times \mathbb{R}^{12} \times S^2$ (exact geodesic gravity manifold conserving $\|g\| \equiv 9.80665\,\text{m/s}^2$).
   - Fuses LiDAR planar/linear residuals AND **coarse-to-fine 3-level grayscale pyramid 8x8 direct photometric visual factors** directly into the exact same 17x17 Cholesky $LL^T$ normal equations — stabilizing texture-rich corridors even during geometric LiDAR degeneracy.
   - Supports **Graduated Non-Convexity (GNC) Cauchy** weighting, **Anisotropic Subspace-Projected Degeneracy Mitigation**, and configurable **20–40 Hz Sub-Sweep Micro-Updates** (`sub_sweep_splits = 1..4`) for aggressive handheld/drone motion.
3. **Exact Coupled Block-Cholesky Pose Graph + Native RTK GNSS / AprilTag / Flash Anchor Factors (`PoseGraphOptimizer`)**
   - Solves full coupled $6N \times 6N$ Block-Cholesky Levenberg-Marquardt globally on $SE(3)$ (with automatic Gauss-Seidel fallback for ultra-large graphs), fusing sequential odometry, verified loop closures, **3-DOF RTK GNSS ENU** constraints, **6-DOF AprilTag / Control Points**, and **Single-Shot Flash Scan Anchors**.
4. **Dual-Resolution O(1) Sufficient-Statistics Surfel Map (`AvSurfelMap`)**
   - Fine (`0.38 m`) + Coarse (`0.76 m`) hash surfel pyramid storing 10-scalar incremental sufficient statistics (zero per-cell point arrays).
   - Westin shape classification providing both **Point-to-Plane** and **Point-to-Line** geometric constraints plus **Live Ray-Visibility Dynamic Carving** and **Motor-Free Single-Shot Flash Scan Anchoring**.
5. **Integrated Point Cloud Conditioning & Native External Sensor Drivers**
   - Clean-room zero-dependency UDP adapters behind `ISensorStreamAdapter` for **RoboSense Airy**, **Livox Mid-360** (UDP 56301/56401), and **Ouster OS0/OS1** (UDP 7502/7503), plus rig presets for **Hesai XT/JT** and **Apple ARKit dToF**.
   - Single-pass real-time front-end conditioning (Range/Self-Mask, Ring Flying-Pixel Veil rejection, flat open-addressing Adaptive Range-Stratified Centroid Voxels) + post-scan ROR/SOR/surfel-guided surface denoising.

---

## Side-by-Side Testing

- **iOS App:** Builds `build/AVSlam.ipa` (`com.artificialvision.avslam`, Display Name `AV SLAM`) — installs directly alongside `AVScan.ipa` (`com.artificialvision.avscan`) on the same iPad/iPhone.
- **Host Unit & Live Regression Suite:** `avslam_tests.exe`, `avslam_live_regression_test.exe`, `avslam_terrestrial_test.exe`, `avslam_autocalib_test.exe`.
- **Automated A/B Benchmark:** `avslam_ab_benchmark.exe` replays the exact same live `.avraw` scan through both `AvSlamEngine` and `AV_iOS`'s `FastLivo2Engine` side-by-side.
