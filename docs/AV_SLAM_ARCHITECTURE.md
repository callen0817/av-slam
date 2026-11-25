# AV-SLAM — Proprietary Unified Factor-Manifold SLAM Architecture

**Owner:** Artificial Vision Inc.  
**License:** 100% First-Party Proprietary C++17 (Zero GPL, Zero External SLAM Dependencies)  
**Target Platforms:** Apple iOS / iPadOS (`arm64`), NVIDIA Jetson Orin (`AV scanAR`), Desktop Windows / Linux / macOS (`AV Scan`)

---

## 1. Executive IP Summary (Why AV-SLAM Is Proprietary & Distinct)

Unlike open-source academic ports (such as GPLv2 `hku-mars/FAST-LIVO2`, which relies on sequential 18-state ES-IEKF updates over `ikd-Tree` and ROS middleware), **AV-SLAM** is a ground-up, zero-dependency, hardware-agnostic C++17 SLAM engine built specifically for **mobile, edge, and desktop multi-sensor reality capture**:

| Architectural Layer | Legacy FAST-LIVO2 Formulation | Proprietary AV-SLAM Architecture |
| :--- | :--- | :--- |
| **Inertial Propagation** | Per-sample (200 Hz) dense 18x18 Kalman covariance multiplies | **On-Manifold Lie-Group IMU Preintegration** (`AvImuPreintegrator`) with block-sparse 3x3 covariance & analytical bias Jacobians + single sweep-level propagation |
| **State Manifold & Solver** | Unconstrained 18-state ES-IEKF (3D gravity requires manual norm clamping) with dense 18x18 Gauss-Jordan inversion | **Minimal 17-DOF Hybrid Manifold** ($SO(3) \times \mathbb{R}^{12} \times S^2$ geodesic gravity) solved via **Cholesky $LL^T$ MAP Optimization** with **Graduated Non-Convexity (GNC) Cauchy** weighting, **Tightly-Coupled 3-Level Grayscale Pyramid Direct Visual Factors**, and configurable **20–40 Hz Sub-Sweep Micro-Updates** (`sub_sweep_splits = 1..4`) |
| **Degeneracy Handling** | Binary warning flag (`is_degenerate`) | **Anisotropic Subspace-Projected Degeneracy Mitigation** — injects directional IMU prior stiffening strictly along under-constrained geometric eigenvectors, reinforced by fused visual photometric gradients |
| **Global Backend (PGO)** | Diagonal Gauss-Seidel relaxation | **Exact Coupled $6N \times 6N$ Block-Cholesky Levenberg-Marquardt** (`PoseGraphOptimizer`) fusing odometry, verified loop closures, **3-DOF RTK GNSS ENU**, **6-DOF AprilTag / Control Points**, and **Single-Shot Flash Scan Anchors** |
| **Local Map Representation** | Single-scale `ikd-Tree` or point-list voxel cache recomputed from scratch | **Dual-Resolution O(1) Sufficient-Statistics Surfel Map** (`AvSurfelMap`, Fine 0.38 m + Coarse 0.76 m tiers) with **Westin Planar + Linear-Edge** classification & **Live Ray-Visibility Dynamic Carving** |
| **Point Cloud Conditioning** | Minimal stride / single voxel downsample | **Built-In Multi-Stage Filter Pipeline** (`PointCloudFilters` + `PointCloudCleanup`): Single-pass Range/Self-Mask + Ring Flying-Pixel Veil Rejection, Flat Open-Addressing Adaptive Range-Stratified Centroids, Fast O(N) ROR/SOR, Edge-Preserving Surfel Surface Denoising |
| **Hardware Support** | Hardcoded per-sensor ROS config | **Hardware-Agnostic Sensor Rig Abstraction** (`SensorRigProfile` + `ISensorStreamAdapter`) with **clean-room native UDP adapters** for **RoboSense Airy**, **Livox Mid-360** (UDP 56301/56401), and **Ouster OS0/OS1** (UDP 7502/7503), plus presets for **Hesai XT/JT** and **Apple ARKit dToF** |
| **Flash Scan vs Terrestrial** | Conflated with 6x60 deg motorized tripod workflow | **Decoupled Motor-Free Single-Shot Flash Scan** (1-tap stationary accumulation + `FLASH_ANCHOR` PGO factor + `injectTerrestrialAnchor`, zero tripod/motor) separate from standalone **Mode B Terrestrial 360°** (6x60 deg detent/motorized head) |

---

## 2. Shared Cross-Platform Core (`avslam_core`)

One single C++17 static library (`avslam_core`) powers all three Artificial Vision products:
1. **AV SLAM (iOS / iPadOS)** — Native Objective-C++ + Metal 3D renderer + USB-C Ethernet external LiDAR + internal/USB-C UVC external RGB camera (`com.artificialvision.avslam`, installs side-by-side with `AV Scan`).
2. **AV scanAR (NVIDIA Jetson Orin / Linux arm64)** — Links `avslam_core` directly with the RoboSense Airy + Orbbec Gemini 336L rig profile (`scanar_v4_airy_336l_usb`).
3. **AV Scan (Desktop Workstation)** — Links `avslam_core` for multi-sensor ingestion, offline `.avraw` reprocessing, terrestrial registration, and ASTM E57 / PLY / PCD / Walkthrough Bundle generation.

---

## 3. Side-by-Side Testing & Commercial Verification

This repository (`AV_SLAM`) runs completely in parallel with `AV_iOS`:
- **Distinct iOS App Bundle:** `com.artificialvision.avslam` (`AV SLAM`) installs directly next to `com.artificialvision.avscan` (`AV Scan`) on the same iPad/iPhone for live field A/B testing.
- **Automated A/B Benchmark (`avslam_ab_benchmark`):** Replays identical recorded `.avraw` sensor captures through both `AvSlamEngine` and legacy `FastLivo2Engine` to verify frame latency, registration RMSE, ceiling/floor sharpness, trajectory consistency, and zero tracking loss.
