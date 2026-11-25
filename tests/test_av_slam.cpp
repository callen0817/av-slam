#include "avscan/backend/PoseGraphOptimizer.hpp"
#include "avscan/filters/PointCloudFilters.hpp"
#include "avscan/sensors/LivoxMid360Adapter.hpp"
#include "avscan/sensors/OusterAdapter.hpp"
#include "avscan/sensors/SensorRigProfile.hpp"
#include "avscan/slam/AvImuPreintegrator.hpp"
#include "avscan/slam/AvSlamEngine.hpp"
#include "avscan/slam/AvSurfelMap.hpp"
#include <cmath>
#include <cstdio>

bool run_test_av_slam() {
    using namespace avscan;

    // 1. Verify SO(3) and SE(3) Lie-group algebra
    const math::Vec3d omega(0.15, -0.25, 0.35);
    const math::SO3d R = math::SO3d::exp(omega);
    if ((omega - R.log()).norm() > 1e-7) {
        std::printf("  [AV-SLAM FAIL] SO3 exp/log round-trip mismatch\n");
        return false;
    }

    // 2. Verify On-Manifold IMU Preintegrator analytical bias Jacobians against finite differences!
    {
        slam::AvImuPreintegrator preint;
        std::vector<core::ImuMeasurement> imus;
        for (int k = 0; k <= 20; ++k) {
            core::ImuMeasurement m;
            m.timestamp_sec = 1.0 + k * 0.005;
            m.gyro_rads = math::Vec3d(0.25, -0.15, 0.40);
            m.accel_mps2 = math::Vec3d(0.4, -0.3, 9.80665 + 0.2);
            imus.push_back(m);
        }
        core::SlamState s0;
        s0.timestamp_sec = 1.0;
        s0.gravity_mps2 = {0.0, 0.0, -9.80665};
        core::SlamState sp0;
        std::vector<slam::MotionKnot> knots;
        const auto fac0 = preint.preintegrateInterval(s0, imus, nullptr, 1.10, sp0, knots);

        // Perturb gyro bias by eps and compare finite-difference vs first-order analytical correction
        const math::Vec3d dbg(1e-3, -0.8e-3, 1.2e-3);
        const math::Vec3d dba(2e-3, -1.5e-3, 1.0e-3);
        core::SlamState s1 = s0;
        s1.bias_gyro_rads += dbg;
        s1.bias_accel_mps2 += dba;
        core::SlamState sp1;
        const auto fac1 = preint.preintegrateInterval(s1, imus, nullptr, 1.10, sp1, knots);

        const math::SO3d dR_approx = fac0.correctedDeltaR(dbg);
        const double rot_err = (fac1.delta_R.inverse() * dR_approx).log().norm();
        const double vel_err = (fac1.delta_v - fac0.correctedDeltaV(dbg, dba)).norm();
        const double pos_err = (fac1.delta_p - fac0.correctedDeltaP(dbg, dba)).norm();
        if (rot_err > 1e-4 || vel_err > 2e-4 || pos_err > 2e-4) {
            std::printf("  [AV-SLAM FAIL] IMU preintegration bias Jacobians error (rot=%.6f, vel=%.6f, pos=%.6f)\n",
                        rot_err, vel_err, pos_err);
            return false;
        }
    }

    // 3. Verify Hardware-Agnostic Sensor Rig Presets (Airy, Mid-360, Ouster, Hesai, ARKit)
    {
        const auto airy = sensors::GenericSensorRigConfig::presetRoboSenseAiry();
        const auto mid360 = sensors::GenericSensorRigConfig::presetLivoxMid360();
        const auto ouster = sensors::GenericSensorRigConfig::presetOusterOs0();
        const auto hesai = sensors::GenericSensorRigConfig::presetHesaiXt32();
        const auto arkit = sensors::GenericSensorRigConfig::presetAppleArkitDepth();
        if (airy.hardware_type != sensors::LidarHardwareType::ROBOSENSE_AIRY ||
            mid360.hardware_type != sensors::LidarHardwareType::LIVOX_MID360 ||
            ouster.hardware_type != sensors::LidarHardwareType::OUSTER_OS0_OS1 ||
            hesai.hardware_type != sensors::LidarHardwareType::HESAI_XT_JT ||
            arkit.hardware_type != sensors::LidarHardwareType::APPLE_ARKIT_DTOF) {
            std::printf("  [AV-SLAM FAIL] Sensor rig preset enum mismatch\n");
            return false;
        }
    }

    // 4. Verify Proprietary Point Cloud Filter Pipeline (Flying-Pixel Veil, Self-Mask, ROR, SOR, Surfel Denoise)
    {
        std::vector<core::LidarPointXYZIRT> raw;
        // Generate a clean wall at x = 4.0 m on ring 10, plus 2 injected flying-pixel veil spikes + 2 airborne dust points
        for (int i = 0; i < 60; ++i) {
            core::LidarPointXYZIRT pt;
            pt.x = 4.0;
            pt.y = -1.5 + i * 0.05;
            pt.z = 0.0;
            pt.range_m = static_cast<float>(std::sqrt(pt.x * pt.x + pt.y * pt.y));
            pt.ring = 10;
            pt.intensity = 120.0f;
            raw.push_back(pt);
        }
        // Inject a mid-air flying-pixel spike between consecutive returns on ring 10
        raw[20].x = 2.2;
        raw[20].range_m = static_cast<float>(std::sqrt(raw[20].x * raw[20].x + raw[20].y * raw[20].y));
        // Add a second ring so max_ring > 0
        for (int i = 0; i < 60; ++i) {
            core::LidarPointXYZIRT pt;
            pt.x = 4.0;
            pt.y = -1.5 + i * 0.05;
            pt.z = 0.10;
            pt.range_m = static_cast<float>(std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z));
            pt.ring = 11;
            raw.push_back(pt);
        }
        size_t fp_removed = 0;
        auto cleaned = filters::PointCloudFilters::filterFlyingPixels(raw, 0.18, 0.35, &fp_removed);
        if (fp_removed < 1) {
            std::printf("  [AV-SLAM FAIL] Flying-pixel veil filter failed to reject isolated depth spike\n");
            return false;
        }

        // Verify ROR removes isolated airborne dust outlier while preserving dense planar wall
        std::vector<core::LidarPointXYZIRT> cloud;
        for (int ix = 0; ix < 20; ++ix) {
            for (int iy = 0; iy < 20; ++iy) {
                core::LidarPointXYZIRT pt;
                pt.x = 3.12 + ((ix + iy) % 2 == 0 ? 0.008 : -0.008);  // +/- 8 mm planar noise (interior of voxel)
                pt.y = ix * 0.04;
                pt.z = iy * 0.04;
                cloud.push_back(pt);
            }
        }
        core::LidarPointXYZIRT dust;
        dust.x = 1.0; dust.y = 5.0; dust.z = 4.5;
        cloud.push_back(dust);

        size_t ror_removed = 0;
        auto ror_out = filters::PointCloudFilters::radiusOutlierRemoval(cloud, 0.08, 3, &ror_removed);
        if (ror_removed != 1 || ror_out.size() != 400) {
            std::printf("  [AV-SLAM FAIL] ROR removed=%zu (expected 1), out=%zu\n", ror_removed, ror_out.size());
            return false;
        }

        // Verify surfel-guided surface denoising tightens the +/- 8 mm noisy wall
        size_t snapped = 0;
        auto denoised = filters::PointCloudFilters::surfelGuidedSurfaceDenoise(ror_out, 0.25, 0.70, 0.005, 0.75, &snapped);
        double sum_dev_after = 0.0;
        for (const auto& pt : denoised) {
            sum_dev_after += std::abs(pt.x - 3.12);
        }
        const double mean_dev_after = sum_dev_after / static_cast<double>(denoised.size());
        if (snapped < 300 || mean_dev_after >= 0.0035) {
            std::printf("  [AV-SLAM FAIL] Surfel surface denoising insufficient (snapped=%zu, mean_dev=%.5f)\n",
                        snapped, mean_dev_after);
            return false;
        }
    }

    // 5. Verify Dual-Resolution AvSurfelMap live dynamic ray carving
    {
        slam::AvSurfelMap smap;
        // Insert a temporary obstacle at (2.0, 0.0, 0.0)
        std::vector<math::Vec3d> transient;
        for (int i = 0; i < 8; ++i) {
            transient.emplace_back(2.0 + 0.01 * i, 0.01 * (i % 3), 0.01 * (i / 3));
        }
        smap.insertWorldPoints(transient, 1);

        // Now cast 30 rays from origin (0,0,0) through (2.0, 0.0, 0.0) hitting a permanent wall at (4.5, 0.0, 0.0)
        std::vector<math::Vec3d> behind_wall;
        for (int k = 0; k < 30; ++k) {
            behind_wall.emplace_back(4.75, 0.005 * (k % 3), 0.005 * (k % 2));
        }
        for (int f = 0; f < 5; ++f) {
            smap.carveFreeSpaceRays(math::Vec3d(0.0, 0.0, 0.0), behind_wall, 1);
        }
        if (!smap.isPointDynamic(math::Vec3d(2.02, 0.01, 0.01))) {
            std::printf("  [AV-SLAM FAIL] Live free-space ray carving failed to flag dynamic transient cell\n");
            return false;
        }
    }

    // 6. Verify AvSlamEngine full 17-DOF LIO tracking on 3-plane room corner + exact S^2 gravity norm conservation
    {
        slam::AvSlamEngine engine;
        if (engine.getLicenseIdentifier().find("AV-SLAM Proprietary") == std::string::npos) {
            std::printf("  [AV-SLAM FAIL] Unexpected license identifier: %s\n", engine.getLicenseIdentifier().c_str());
            return false;
        }

        for (int f = 0; f < 6; ++f) {
            sync::SyncBundle bundle;
            const double t0 = f * 0.10;
            const double t1 = t0 + 0.10;
            bundle.lidar_frame.frame_id = static_cast<uint64_t>(f + 1);
            bundle.lidar_frame.timestamp_start_sec = t0;
            bundle.lidar_frame.timestamp_end_sec = t1;

            for (int k = 0; k < 20; ++k) {
                core::ImuMeasurement imu;
                imu.timestamp_sec = t0 + k * 0.005;
                imu.accel_mps2 = {0.0, 0.0, 9.80665};
                imu.gyro_rads = {0.0, 0.0, 0.0};
                bundle.imu_measurements.push_back(imu);
            }

            // Generate 3 orthogonal planes: floor (z=-1), front wall (x=4), side wall (y=3)
            for (int u = -10; u <= 10; ++u) {
                for (int v = -10; v <= 10; ++v) {
                    core::LidarPointXYZIRT p1;
                    p1.x = 2.0 + u * 0.08; p1.y = v * 0.08; p1.z = -1.0; p1.timestamp_sec = t0 + 0.02;
                    bundle.lidar_frame.points.push_back(p1);

                    core::LidarPointXYZIRT p2;
                    p2.x = 4.0; p2.y = u * 0.08; p2.z = v * 0.08; p2.timestamp_sec = t0 + 0.05;
                    bundle.lidar_frame.points.push_back(p2);

                    core::LidarPointXYZIRT p3;
                    p3.x = 2.0 + u * 0.08; p3.y = 3.0; p3.z = v * 0.08; p3.timestamp_sec = t0 + 0.08;
                    bundle.lidar_frame.points.push_back(p3);
                }
            }

            core::LidarFrame body_f, world_f;
            slam::EstimatorMetrics metrics;
            if (!engine.processBundle(bundle, body_f, world_f, metrics)) {
                std::printf("  [AV-SLAM FAIL] processBundle returned false on frame %d\n", f);
                return false;
            }
        }

        const auto st = engine.getCurrentState();
        const double g_err = std::abs(st.gravity_mps2.norm() - 9.80665);
        if (st.T_world_body.t.norm() > 0.05 || g_err > 1e-6 || engine.getSurfelMap().numValidPlanarSurfels() < 10) {
            std::printf("  [AV-SLAM FAIL] Room tracking check failed (pos=%.4f, g_err=%.2e, planes=%zu)\n",
                        st.T_world_body.t.norm(), g_err, engine.getSurfelMap().numValidPlanarSurfels());
            return false;
        }

        // Test Hybrid Terrestrial Flash-Station anchor injection
        std::vector<core::LidarPointXYZIRT> anchor_pts;
        for (int u = -5; u <= 5; ++u) {
            for (int v = -5; v <= 5; ++v) {
                core::LidarPointXYZIRT pt;
                pt.x = 3.0; pt.y = u * 0.1; pt.z = v * 0.1;
                anchor_pts.push_back(pt);
            }
        }
        engine.injectTerrestrialAnchor(math::SE3d(math::SO3d(), math::Vec3d(1.5, -0.5, 0.0)), anchor_pts);
        if ((engine.getCurrentState().T_world_body.t - math::Vec3d(1.5, -0.5, 0.0)).norm() > 1e-6) {
            std::printf("  [AV-SLAM FAIL] Terrestrial anchor injection did not lock pose\n");
            return false;
        }
    }

    // 7. Verify AV-SLAM v2.0 Exact Block-Cholesky PGO with RTK GNSS ENU & AprilTag 6-DOF Control-Point Factors
    {
        backend::PoseGraphOptimizer pgo;
        const int N = 25;
        // True trajectory moves +1.0 m/keyframe along X; odometry drifts linearly by +0.08 m/step in Y & +0.04 m/step in Z
        for (int i = 0; i < N; ++i) {
            core::KeyframeSubmap kf;
            kf.keyframe_id = static_cast<uint64_t>(i + 1);
            kf.timestamp_sec = i * 1.0;
            kf.T_world_keyframe = math::SE3d(math::SO3d(), math::Vec3d(i * 1.0, i * 0.08, i * 0.04));
            pgo.addKeyframeSubmap(kf, {150.0, 150.0, 150.0, 600.0, 600.0, 600.0});
        }
        // Add 3 RTK GNSS ENU measurements along true X axis (y=0, z=0) + 1 AprilTag 6-DOF control point at end
        if (!pgo.addRtkGnssFactor(10, math::Vec3d(9.0, 0.0, 0.0), 0.02, 0.03) ||
            !pgo.addRtkGnssFactor(18, math::Vec3d(17.0, 0.0, 0.0), 0.02, 0.03) ||
            !pgo.addControlPointPoseFactor(25, math::SE3d(math::SO3d(), math::Vec3d(24.0, 0.0, 0.0)))) {
            std::printf("  [AV-SLAM FAIL] Failed to add RTK GNSS / Control-Point factors to PGO\n");
            return false;
        }
        const auto rep = pgo.optimizeGlobalPoseGraph(25);
        const auto& end_pose = pgo.getSubmaps().back().T_world_keyframe;
        const double end_err = (end_pose.t - math::Vec3d(24.0, 0.0, 0.0)).norm();
        if (!rep.used_exact_cholesky || !rep.converged || rep.num_global_factors != 3 || end_err > 0.05) {
            std::printf("  [AV-SLAM FAIL] Exact Cholesky PGO + RTK/GCP check failed (exact=%d, conv=%d, gf=%zu, err=%.4fm)\n",
                        rep.used_exact_cholesky ? 1 : 0, rep.converged ? 1 : 0, rep.num_global_factors, end_err);
            return false;
        }
    }

    // 8. Verify AV-SLAM v2.0 Clean-Room Livox Mid-360 UDP Protocol Decoder + AvSlamEngine Ingestion
    {
        sensors::LivoxMid360Decoder decoder(0.10);
        std::vector<core::LidarPointXYZIRT> synth;
        for (int i = 0; i < 96; ++i) {
            core::LidarPointXYZIRT p;
            p.x = 3.5 + 0.01 * (i % 12);
            p.y = -1.2 + 0.03 * i;
            p.z = 0.25 * ((i % 5) - 2);
            p.intensity = 140.0f;
            synth.push_back(p);
        }
        auto pkt1 = sensors::LivoxMid360Decoder::buildCartesianHighPacket(1000000000ULL, 1000, synth);
        auto pkt2 = sensors::LivoxMid360Decoder::buildCartesianHighPacket(1120000000ULL, 1000, synth);
        bool completed = false;
        core::LidarFrame decoded;
        if (!decoder.decodePointCloudPacket(pkt1.data(), pkt1.size(), completed, decoded) || completed) {
            std::printf("  [AV-SLAM FAIL] Livox Mid-360 first packet decode error\n");
            return false;
        }
        if (!decoder.decodePointCloudPacket(pkt2.data(), pkt2.size(), completed, decoded) || !completed || decoded.points.size() != 192) {
            std::printf("  [AV-SLAM FAIL] Livox Mid-360 frame completion failed (pts=%zu)\n", decoded.points.size());
            return false;
        }
        if (std::abs(decoded.points[0].x - 3.5) > 0.002 || std::abs(decoded.points[0].y + 1.2) > 0.002) {
            std::printf("  [AV-SLAM FAIL] Livox Mid-360 decoded coordinate mismatch (%.4f, %.4f)\n",
                        decoded.points[0].x, decoded.points[0].y);
            return false;
        }
        core::ImuMeasurement imu_in, imu_out;
        imu_in.gyro_rads = {0.12, -0.08, 0.25};
        imu_in.accel_mps2 = {0.15, -0.10, 9.80665};
        auto imu_pkt = sensors::LivoxMid360Decoder::buildImuPacket(1050000000ULL, imu_in);
        if (!decoder.decodeImuPacket(imu_pkt.data(), imu_pkt.size(), imu_out) ||
            (imu_out.gyro_rads - imu_in.gyro_rads).norm() > 1e-4 ||
            (imu_out.accel_mps2 - imu_in.accel_mps2).norm() > 1e-3) {
            std::printf("  [AV-SLAM FAIL] Livox Mid-360 IMU decode mismatch\n");
            return false;
        }
    }

    // 9. Verify AV-SLAM v2.0 Clean-Room Ouster OS0/OS1 UDP Protocol Decoder (LiDAR Columns + IMU)
    {
        sensors::OusterSensorConfig ocfg;
        ocfg.pixels_per_column = 32;
        ocfg.columns_per_packet = 16;
        ocfg.columns_per_frame = 512;
        sensors::OusterDecoder odec(ocfg);

        auto opkt1 = odec.buildSyntheticLidarPacket(10, 0, 2000000000ULL, 5.0);
        auto opkt2 = odec.buildSyntheticLidarPacket(11, 0, 2100000000ULL, 5.0);
        bool o_done = false;
        core::LidarFrame oframe;
        if (!odec.decodeLidarPacket(opkt1.data(), opkt1.size(), o_done, oframe) || o_done) {
            std::printf("  [AV-SLAM FAIL] Ouster first packet decode failed\n");
            return false;
        }
        if (!odec.decodeLidarPacket(opkt2.data(), opkt2.size(), o_done, oframe) || !o_done || oframe.points.size() != 32 * 16) {
            std::printf("  [AV-SLAM FAIL] Ouster frame roll-over decode failed (pts=%zu)\n", oframe.points.size());
            return false;
        }
        core::ImuMeasurement oimu_in, oimu_out;
        oimu_in.gyro_rads = {-0.05, 0.10, -0.02};
        oimu_in.accel_mps2 = {0.0, 0.2, 9.80665};
        auto oimu_pkt = sensors::OusterDecoder::buildSyntheticImuPacket(2050000000ULL, oimu_in);
        if (!odec.decodeImuPacket(oimu_pkt.data(), oimu_pkt.size(), oimu_out) ||
            (oimu_out.gyro_rads - oimu_in.gyro_rads).norm() > 1e-4 ||
            (oimu_out.accel_mps2 - oimu_in.accel_mps2).norm() > 1e-3) {
            std::printf("  [AV-SLAM FAIL] Ouster IMU UDP decode mismatch\n");
            return false;
        }
    }

    // 10. Verify AV-SLAM v2.0 Tightly-Coupled 3-Level Pyramid Direct Visual Fusion inside 17-DOF Cholesky Solver
    {
        slam::AvSlamConfig vcfg;
        vcfg.enable_visual_fusion = true;
        vcfg.enable_rgb_colorization = true;
        vcfg.visual_pyramid_levels = 3;
        vcfg.sensor_rig.T_imu_lidar = math::SE3d::Identity();
        vcfg.sensor_rig.camera_extrinsics.verified = true;
        vcfg.sensor_rig.camera_extrinsics.confidence_score = 0.96;
        // Camera looks along +X_lidar: +Z_cam = +X_lidar, +X_cam = -Y_lidar, +Y_cam = -Z_lidar
        math::Mat3d R_cam_lidar;
        R_cam_lidar(0, 0) = 0.0; R_cam_lidar(0, 1) = -1.0; R_cam_lidar(0, 2) = 0.0;
        R_cam_lidar(1, 0) = 0.0; R_cam_lidar(1, 1) =  0.0; R_cam_lidar(1, 2) = -1.0;
        R_cam_lidar(2, 0) = 1.0; R_cam_lidar(2, 1) =  0.0; R_cam_lidar(2, 2) = 0.0;
        vcfg.sensor_rig.camera_extrinsics.T_cam_lidar = math::SE3d(math::SO3d(R_cam_lidar), math::Vec3d(0.0, 0.0, 0.0));

        slam::AvSlamEngine vengine(vcfg);
        const int W = 160, H = 120;
        std::vector<uint8_t> gray(static_cast<size_t>(W * H));
        std::vector<uint8_t> rgb(static_cast<size_t>(W * H * 3));
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                const double val = 128.0 + 95.0 * std::sin(x * 0.28) * std::cos(y * 0.28);
                const uint8_t g = static_cast<uint8_t>(std::clamp(val, 10.0, 245.0));
                gray[static_cast<size_t>(y * W + x)] = g;
                rgb[static_cast<size_t>((y * W + x) * 3 + 0)] = g;
                rgb[static_cast<size_t>((y * W + x) * 3 + 1)] = g;
                rgb[static_cast<size_t>((y * W + x) * 3 + 2)] = g;
            }
        }

        bool saw_visual_update = false;
        size_t max_vis_patches = 0;
        for (int f = 0; f < 5; ++f) {
            sync::SyncBundle b;
            const double t0 = f * 0.10, t1 = t0 + 0.10;
            b.lidar_frame.frame_id = static_cast<uint64_t>(f + 1);
            b.lidar_frame.timestamp_start_sec = t0;
            b.lidar_frame.timestamp_end_sec = t1;
            for (int k = 0; k < 20; ++k) {
                core::ImuMeasurement imu;
                imu.timestamp_sec = t0 + k * 0.005;
                imu.accel_mps2 = {0.0, 0.0, 9.80665};
                b.imu_measurements.push_back(imu);
            }
            for (int u = -10; u <= 10; ++u) {
                for (int v = -10; v <= 10; ++v) {
                    core::LidarPointXYZIRT p1; p1.x = 2.0 + u * 0.08; p1.y = v * 0.08; p1.z = -1.0; p1.timestamp_sec = t0 + 0.02;
                    core::LidarPointXYZIRT p2; p2.x = 4.0; p2.y = u * 0.10; p2.z = v * 0.08; p2.timestamp_sec = t0 + 0.05;
                    core::LidarPointXYZIRT p3; p3.x = 2.0 + u * 0.08; p3.y = 3.0; p3.z = v * 0.08; p3.timestamp_sec = t0 + 0.08;
                    b.lidar_frame.points.push_back(p1);
                    b.lidar_frame.points.push_back(p2);
                    b.lidar_frame.points.push_back(p3);
                }
            }
            core::RgbFrame rf;
            rf.timestamp_sec = t0 + 0.05;
            rf.exposure_duration_sec = 0.01;
            rf.intrinsics.width = W;
            rf.intrinsics.height = H;
            rf.intrinsics.fx = 120.0;
            rf.intrinsics.fy = 120.0;
            rf.intrinsics.cx = 80.0;
            rf.intrinsics.cy = 60.0;
            rf.gray_bytes = gray;
            rf.rgb_bytes = rgb;
            b.rgb_frame = std::move(rf);

            core::LidarFrame bf, wf;
            slam::EstimatorMetrics m;
            if (!vengine.processBundle(b, bf, wf, m)) {
                std::printf("  [AV-SLAM FAIL] Visual fusion processBundle returned false on frame %d\n", f);
                return false;
            }
            if (m.visual_update_applied) {
                saw_visual_update = true;
                max_vis_patches = std::max(max_vis_patches, m.visual_patch_correspondences);
            }
        }
        if (!saw_visual_update || max_vis_patches < 4 || vengine.getCurrentState().T_world_body.t.norm() > 0.05) {
            std::printf("  [AV-SLAM FAIL] 3-Level Pyramid Direct Visual Fusion failed (applied=%d, patches=%zu, pos=%.4fm)\n",
                        saw_visual_update ? 1 : 0, max_vis_patches, vengine.getCurrentState().T_world_body.t.norm());
            return false;
        }
    }

    // 11. Verify AV-SLAM v2.0 Sub-Sweep 20 Hz Micro-Update Mode (sub_sweep_splits = 2)
    {
        slam::AvSlamConfig scfg;
        scfg.sub_sweep_splits = 2;
        slam::AvSlamEngine sengine(scfg);
        for (int f = 0; f < 5; ++f) {
            sync::SyncBundle b;
            const double t0 = f * 0.10, t1 = t0 + 0.10;
            b.lidar_frame.frame_id = static_cast<uint64_t>(f + 1);
            b.lidar_frame.timestamp_start_sec = t0;
            b.lidar_frame.timestamp_end_sec = t1;
            for (int k = 0; k <= 20; ++k) {
                core::ImuMeasurement imu;
                imu.timestamp_sec = t0 + k * 0.005;
                imu.accel_mps2 = {0.0, 0.0, 9.80665};
                b.imu_measurements.push_back(imu);
            }
            for (int u = -10; u <= 10; ++u) {
                for (int v = -10; v <= 10; ++v) {
                    const double dt_frac = (u + 10) / 20.0;
                    core::LidarPointXYZIRT p1; p1.x = 2.0 + u * 0.08; p1.y = v * 0.08; p1.z = -1.0; p1.timestamp_sec = t0 + dt_frac * 0.095;
                    core::LidarPointXYZIRT p2; p2.x = 4.0; p2.y = u * 0.08; p2.z = v * 0.08; p2.timestamp_sec = t0 + dt_frac * 0.095;
                    core::LidarPointXYZIRT p3; p3.x = 2.0 + u * 0.08; p3.y = 3.0; p3.z = v * 0.08; p3.timestamp_sec = t0 + dt_frac * 0.095;
                    b.lidar_frame.points.push_back(p1);
                    b.lidar_frame.points.push_back(p2);
                    b.lidar_frame.points.push_back(p3);
                }
            }
            core::LidarFrame bf, wf;
            slam::EstimatorMetrics m;
            if (!sengine.processBundle(b, bf, wf, m)) {
                std::printf("  [AV-SLAM FAIL] Sub-sweep 20 Hz processBundle returned false on frame %d\n", f);
                return false;
            }
        }
        const auto sst = sengine.getCurrentState();
        if (sst.T_world_body.t.norm() > 0.05 || std::abs(sst.gravity_mps2.norm() - 9.80665) > 1e-6) {
            std::printf("  [AV-SLAM FAIL] Sub-sweep 20 Hz stability check failed (pos=%.4fm)\n", sst.T_world_body.t.norm());
            return false;
        }
    }

    return true;
}
