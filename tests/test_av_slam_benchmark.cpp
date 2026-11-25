#include "avscan/backend/GicpVerifier.hpp"
#include "avscan/backend/PoseGraphOptimizer.hpp"
#include "avscan/backend/ScanContext.hpp"
#include "avscan/filters/PointCloudFilters.hpp"
#include "avscan/processing/PointCloudCleanup.hpp"
#include "avscan/quality/QualityMonitor.hpp"
#include "avscan/sensors/AiryUdpReceiver.hpp"
#include "avscan/slam/AvSlamEngine.hpp"
#include "avscan/slam/FastLivo2Engine.hpp"
#include "avscan/sync/TimeSynchronizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace avscan;

namespace {

double percentile(std::vector<double> vals, double p) {
    if (vals.empty()) return 0.0;
    const size_t idx = std::min(vals.size() - 1, static_cast<size_t>(std::floor(p * static_cast<double>(vals.size() - 1))));
    std::nth_element(vals.begin(), vals.begin() + static_cast<std::ptrdiff_t>(idx), vals.end());
    return vals[idx];
}

struct BenchmarkResult {
    std::string engine_name;
    std::string license_id;
    size_t decoded_sweeps{0};
    size_t decoded_imus{0};
    size_t frames_processed{0};
    size_t keyframes{0};
    size_t loop_closures{0};
    size_t tracking_lost{0};
    double wall_sec{0.0};
    double avg_slam_ms{0.0};
    double p95_slam_ms{0.0};
    double median_rmse_m{0.0};
    double p90_rmse_m{0.0};
    double traveled_m{0.0};
    double traj_z_span_m{0.0};
    double ceiling_band_live_m{0.0};
    double ceiling_band_pgo_m{0.0};
    double floor_band_live_m{0.0};
    double floor_band_refined_m{0.0};
    double ceiling_band_refined_m{0.0};
    size_t unified_pts{0};
    size_t refined_pts{0};
    size_t wall_cells{0};
    math::SE3d final_pose{math::SE3d::Identity()};
    math::Vec3d final_gravity{0.0, 0.0, -9.80665};
    std::vector<math::SE3d> sampled_poses;

    // AV-SLAM specific breakdown
    slam::AvSlamRuntimeStats av_stats{};
};

BenchmarkResult runBenchmarkForEngine(const std::string& raw_path, bool use_av_slam) {
    BenchmarkResult res;
    res.engine_name = use_av_slam ? "AV-SLAM (Proprietary)" : "FastLivo2 (Legacy AV_iOS)";

    sensors::AiryUdpReceiver receiver;
    sync::TimeSynchronizer synchronizer;

    std::unique_ptr<slam::ISlamEstimator> estimator;
    slam::AvSlamEngine* av_ptr = nullptr;
    if (use_av_slam) {
        auto eng = std::make_unique<slam::AvSlamEngine>();
        av_ptr = eng.get();
        estimator = std::move(eng);
    } else {
        estimator = std::make_unique<slam::FastLivo2Engine>();
    }
    res.license_id = estimator->getLicenseIdentifier();

    backend::ScanContextDatabase sc_db;
    backend::GicpVerificationConfig gicp_cfg;
    gicp_cfg.voxel_resolution_m = 0.40;
    gicp_cfg.max_correspondence_dist_m = 0.30;
    gicp_cfg.max_iterations = 20;
    gicp_cfg.min_inlier_overlap_ratio = 0.50;
    gicp_cfg.max_fitness_rmse_m = 0.07;
    gicp_cfg.max_source_points = 4000;
    gicp_cfg.allow_centroid_fallback = false;
    backend::GicpVerifier gicp_verifier(gicp_cfg);
    backend::PoseGraphOptimizer pose_graph;
    quality::QualityMonitor quality_monitor;

    uint64_t next_kf_id = 1;
    uint64_t last_loop_kf_id = 0;
    double total_traveled_m = 0.0;
    double unclosed_dist_m = 0.0;
    math::Vec3d prev_pos(0.0, 0.0, 0.0);
    bool has_prev_pos = false;
    std::unordered_map<uint64_t, double> kf_traveled_dist;

    double slam_ms_total = 0.0;
    std::vector<double> frame_times_ms;
    std::vector<double> frame_rmses;
    std::vector<double> traj_z;
    std::vector<double> map_z_samples;
    map_z_samples.reserve(500000);

    auto processPending = [&]() {
        sync::SyncBundle bundle;
        while (synchronizer.popSyncBundle(bundle)) {
            core::LidarFrame deskewed_body;
            core::LidarFrame world_frame;
            slam::EstimatorMetrics metrics;
            if (!estimator->processBundle(bundle, deskewed_body, world_frame, metrics)) {
                continue;
            }
            res.frames_processed++;
            slam_ms_total += metrics.processing_time_ms;
            frame_times_ms.push_back(metrics.processing_time_ms);

            const auto state = estimator->getCurrentState();
            res.final_pose = state.T_world_body;
            res.final_gravity = state.gravity_mps2;
            if (res.frames_processed <= 15 || (res.frames_processed % 100) == 1 || res.frames_processed == 431) {
                res.sampled_poses.push_back(state.T_world_body);
            }
            if (has_prev_pos) {
                const double step = (state.T_world_body.t - prev_pos).norm();
                if (step < 2.0) {
                    total_traveled_m += step;
                    unclosed_dist_m += step;
                }
            }
            prev_pos = state.T_world_body.t;
            has_prev_pos = true;
            traj_z.push_back(state.T_world_body.t.z);
            if (metrics.geometric_correspondences > 0) {
                frame_rmses.push_back(metrics.geometric_rmse_m);
            }

            const auto qrep = quality_monitor.evaluateFrameQuality(state, metrics, unclosed_dist_m);
            if (qrep.tracking_lost) res.tracking_lost++;

            for (size_t i = 0; i < world_frame.points.size(); i += 10) {
                map_z_samples.push_back(world_frame.points[i].z);
            }

            if (metrics.keyframe_emitted) {
                core::KeyframeSubmap sm;
                sm.keyframe_id = next_kf_id++;
                sm.timestamp_sec = bundle.lidar_frame.timestamp_end_sec;
                sm.T_world_keyframe = state.T_world_body;
                sm.local_points = deskewed_body.points;
                pose_graph.addKeyframeSubmap(sm);
                kf_traveled_dist[sm.keyframe_id] = total_traveled_m;

                const auto sc_desc = backend::ScanContextDatabase::computeDescriptor(
                    sm.keyframe_id, sm.timestamp_sec, sm.local_points, bundle.rgb_frame, state.T_world_body.so3);
                const auto loop_cand = sc_db.detectLoopClosure(sc_desc);
                sc_db.addKeyframeDescriptor(sc_desc);

                const bool cooldown_ok = (last_loop_kf_id == 0) ||
                                         ((sm.keyframe_id > last_loop_kf_id) &&
                                          ((sm.keyframe_id - last_loop_kf_id) >= 10));
                if (loop_cand.valid && cooldown_ok) {
                    for (const auto& target_sm : pose_graph.getSubmaps()) {
                        if (target_sm.keyframe_id == loop_cand.match_keyframe_id) {
                            const uint64_t kf_gap = (sm.keyframe_id > target_sm.keyframe_id)
                                                        ? (sm.keyframe_id - target_sm.keyframe_id)
                                                        : 0;
                            const double dist_gap = total_traveled_m - kf_traveled_dist[target_sm.keyframe_id];
                            const double spatial_dist = (sm.T_world_keyframe.t - target_sm.T_world_keyframe.t).norm();
                            if (kf_gap >= 30 && dist_gap >= 8.0 && spatial_dist < 4.5) {
                                const math::SE3d init_guess = target_sm.T_world_keyframe.inverse() * sm.T_world_keyframe;
                                const auto ver = gicp_verifier.verifyLoopConstraint(
                                    sm.local_points, target_sm.local_points, init_guess);
                                if (ver.verified) {
                                    const math::SE3d delta = init_guess.inverse() * ver.T_target_source;
                                    const double drot_deg = delta.so3.log().norm() * (180.0 / math::kPi);
                                    const double dtrans = delta.t.norm();
                                    if (drot_deg < 20.0 && dtrans < 2.0) {
                                        pose_graph.addVerifiedLoopFactor(
                                            target_sm.keyframe_id, sm.keyframe_id,
                                            ver.T_target_source, ver.information_diagonal, true);
                                        pose_graph.optimizeGlobalPoseGraph(15);
                                        last_loop_kf_id = sm.keyframe_id;
                                        unclosed_dist_m = 0.0;
                                    }
                                }
                            }
                            break;
                        }
                    }
                }
            }
        }
    };

    receiver.setCallbacks(
        [&](const core::LidarFrame& frame) {
            synchronizer.pushLidarFrame(frame, frame.timestamp_end_sec);
            processPending();
        },
        [&](const core::ImuMeasurement& imu) {
            synchronizer.pushImu(imu, imu.timestamp_sec);
        },
        [&](const core::AiryFactoryCalibration& calib) {
            estimator->setFactoryCalibration(calib);
        });

    const auto wall0 = std::chrono::steady_clock::now();
    receiver.replayRawFile(raw_path, res.decoded_sweeps, res.decoded_imus);
    processPending();
    const auto wall1 = std::chrono::steady_clock::now();

    res.wall_sec = std::chrono::duration<double>(wall1 - wall0).count();
    res.avg_slam_ms = (res.frames_processed > 0) ? (slam_ms_total / res.frames_processed) : 0.0;
    res.p95_slam_ms = percentile(frame_times_ms, 0.95);
    res.median_rmse_m = percentile(frame_rmses, 0.50);
    res.p90_rmse_m = percentile(frame_rmses, 0.90);
    res.traveled_m = total_traveled_m;
    res.keyframes = pose_graph.getSubmaps().size();
    res.loop_closures = pose_graph.numLoopClosures();

    if (!traj_z.empty()) {
        const double zmin = *std::min_element(traj_z.begin(), traj_z.end());
        const double zmax = *std::max_element(traj_z.begin(), traj_z.end());
        res.traj_z_span_m = zmax - zmin;
    }

    pose_graph.optimizeGlobalPoseGraph(25);
    const auto unified = pose_graph.buildGloballyConsistentPointCloud(1, 0.015);
    res.unified_pts = unified.size();

    const auto fp = quality::QualityMonitor::generateLiveFloorPlan(
        unified, pose_graph.getSubmaps(), pose_graph.getFactors(), 0.08);
    for (uint8_t c : fp.occupancy_cells) {
        if (c == 1) res.wall_cells++;
    }

    res.ceiling_band_live_m = percentile(map_z_samples, 0.99) - percentile(map_z_samples, 0.90);
    res.floor_band_live_m = percentile(map_z_samples, 0.02) - percentile(map_z_samples, 0.01);

    std::vector<double> unified_z;
    unified_z.reserve(unified.size() / 4 + 1);
    for (size_t i = 0; i < unified.size(); i += 4) unified_z.push_back(unified[i].z);
    res.ceiling_band_pgo_m = percentile(unified_z, 0.99) - percentile(unified_z, 0.90);

    // Run full AV-SLAM map refinement pipeline on AV-SLAM output to measure final deliverable cleanliness
    if (use_av_slam) {
        filters::MapRefinementFilterConfig rcfg;
        rcfg.fusion_voxel_m = 0.015;
        auto refined = filters::PointCloudFilters::runFullMapRefinement(unified, rcfg);
        res.refined_pts = refined.size();
        std::vector<double> rz;
        rz.reserve(refined.size() / 4 + 1);
        for (size_t i = 0; i < refined.size(); i += 4) rz.push_back(refined[i].z);
        res.ceiling_band_refined_m = percentile(rz, 0.99) - percentile(rz, 0.90);
        res.floor_band_refined_m = percentile(rz, 0.02) - percentile(rz, 0.01);
        if (av_ptr) {
            res.av_stats = av_ptr->getRuntimeStats();
        }
    }

    return res;
}

}  // namespace

int main(int argc, char** argv) {
    std::string raw_path;
    if (argc > 1 && argv[1][0] != '\0') {
        raw_path = argv[1];
    } else {
#ifdef AVSCAN_SOURCE_DIR
        raw_path = std::string(AVSCAN_SOURCE_DIR) +
                   "/tests/live scans/Projects/LEVEL_01_20261123_194518/scans/Scan_01_20261123_194518/raw_capture.avraw";
#else
        raw_path = "tests/live scans/Projects/LEVEL_01_20261123_194518/scans/Scan_01_20261123_194518/raw_capture.avraw";
#endif
    }

    if (!std::filesystem::exists(raw_path)) {
        std::printf("[SKIP] Live capture not found at %s\n", raw_path.c_str());
        return 0;
    }

    std::printf("===========================================================================\n");
    std::printf("  HEAD-TO-HEAD A/B BENCHMARK: AV-SLAM (Proprietary) vs FastLivo2 (Legacy)\n");
    std::printf("  Capture: %s\n", raw_path.c_str());
    std::printf("===========================================================================\n\n");

    const auto baseline = runBenchmarkForEngine(raw_path, false);
    const auto avslam   = runBenchmarkForEngine(raw_path, true);

    const double speedup_x = (avslam.avg_slam_ms > 1e-6) ? (baseline.avg_slam_ms / avslam.avg_slam_ms) : 0.0;
    const double rmse_delta_mm = (avslam.median_rmse_m - baseline.median_rmse_m) * 1000.0;
    const double end_pose_diff_m = (avslam.final_pose.t - baseline.final_pose.t).norm();
    const double end_rot_diff_deg = (baseline.final_pose.so3.inverse() * avslam.final_pose.so3).log().norm() * (180.0 / math::kPi);

    std::printf("%-34s | %-20s | %-20s\n", "Metric", "FastLivo2 (AV_iOS)", "AV-SLAM (Proprietary)");
    std::printf("-----------------------------------+----------------------+----------------------\n");
    std::printf("%-34s | %-20zu | %-20zu\n", "Frames Processed", baseline.frames_processed, avslam.frames_processed);
    std::printf("%-34s | %-17.2f ms | %-17.2f ms\n", "Avg SLAM Compute / Frame", baseline.avg_slam_ms, avslam.avg_slam_ms);
    std::printf("%-34s | %-17.2f ms | %-17.2f ms\n", "P95 SLAM Compute / Frame", baseline.p95_slam_ms, avslam.p95_slam_ms);
    std::printf("%-34s | %-18.2f s | %-18.2f s\n", "Total Replay Wall Time", baseline.wall_sec, avslam.wall_sec);
    std::printf("%-34s | %-18.4f m | %-18.4f m\n", "Median Point-to-Surface RMSE", baseline.median_rmse_m, avslam.median_rmse_m);
    std::printf("%-34s | %-18.4f m | %-18.4f m\n", "P90 Point-to-Surface RMSE", baseline.p90_rmse_m, avslam.p90_rmse_m);
    std::printf("%-34s | %-18.2f m | %-18.2f m\n", "Trajectory Length", baseline.traveled_m, avslam.traveled_m);
    std::printf("%-34s | %-18.3f m | %-18.3f m\n", "Trajectory Z Span", baseline.traj_z_span_m, avslam.traj_z_span_m);
    std::printf("%-34s | %-18.3f m | %-18.3f m\n", "Ceiling Band p99-p90 (Live)", baseline.ceiling_band_live_m, avslam.ceiling_band_live_m);
    std::printf("%-34s | %-18.3f m | %-18.3f m\n", "Ceiling Band p99-p90 (Post-PGO)", baseline.ceiling_band_pgo_m, avslam.ceiling_band_pgo_m);
    std::printf("%-34s | %-18.3f m | %-18.3f m\n", "Floor Band p02-p01 (Live)", baseline.floor_band_live_m, avslam.floor_band_live_m);
    std::printf("%-34s | %-20zu | %-20zu\n", "Tracking Lost Frames", baseline.tracking_lost, avslam.tracking_lost);
    std::printf("%-34s | %-20zu | %-20zu\n", "Keyframes Emitted", baseline.keyframes, avslam.keyframes);
    std::printf("%-34s | %-20zu | %-20zu\n", "Unified 1.5cm Map Points", baseline.unified_pts, avslam.unified_pts);
    std::printf("%-34s | %-20zu | %-20zu\n", "Floorplan Wall Cells", baseline.wall_cells, avslam.wall_cells);
    std::printf("-----------------------------------+----------------------+----------------------\n\n");

    std::printf("AV-SLAM Stage Timing Breakdown (avg/frame):\n");
    std::printf("  Preintegration + Twist Deskew : %.2f ms\n", avslam.av_stats.avg_preint_deskew_ms);
    std::printf("  Real-Time Point Cloud Filters : %.2f ms (flying pixels rejected: %zu)\n",
                avslam.av_stats.avg_filter_ms, avslam.av_stats.total_flying_pixels_removed);
    std::printf("  17-DOF Cholesky MAP Solver    : %.2f ms\n", avslam.av_stats.avg_map_solve_ms);
    std::printf("  Dual-Tier Surfel Map + Carving: %.2f ms (fine=%zu, coarse=%zu, dynamic=%zu)\n",
                avslam.av_stats.avg_surfel_update_ms,
                avslam.av_stats.fine_surfel_cells,
                avslam.av_stats.coarse_surfel_cells,
                avslam.av_stats.flagged_dynamic_cells);
    std::printf("  Post-Scan Filtered Cloud      : %zu pts (ceil=%.3fm, floor=%.3fm)\n\n",
                avslam.refined_pts, avslam.ceiling_band_refined_m, avslam.floor_band_refined_m);

    std::printf("Head-to-Head Summary:\n");
    std::printf("  AV-SLAM Compute Speedup       : %.2fx (%.2f ms vs %.2f ms)\n",
                speedup_x, avslam.avg_slam_ms, baseline.avg_slam_ms);
    std::printf("  Median RMSE Delta             : %+.2f mm (negative = AV-SLAM sharper)\n", rmse_delta_mm);
    std::printf("  Final Pose Agreement          : %.3f m trans, %.2f deg rot\n", end_pose_diff_m, end_rot_diff_deg);
    std::printf("  Final Gravity Tilt from -Z    : FastLivo2=%.2f deg | AV-SLAM=%.2f deg\n",
                std::acos(std::clamp(-baseline.final_gravity.z / baseline.final_gravity.norm(), -1.0, 1.0)) * (180.0 / math::kPi),
                std::acos(std::clamp(-avslam.final_gravity.z / avslam.final_gravity.norm(), -1.0, 1.0)) * (180.0 / math::kPi));
    const size_t n_ck = std::min(baseline.sampled_poses.size(), avslam.sampled_poses.size());
    for (size_t i = 0; i < n_ck; ++i) {
        const double dp = (baseline.sampled_poses[i].t - avslam.sampled_poses[i].t).norm();
        const double dr = (baseline.sampled_poses[i].so3.inverse() * avslam.sampled_poses[i].so3).log().norm() * (180.0 / math::kPi);
        std::printf("    [Checkpoint %zu] dPos=%.3fm dRot=%.2f deg | FL2=(%.2f,%.2f,%.2f) AV=(%.2f,%.2f,%.2f)\n",
                    i, dp, dr,
                    baseline.sampled_poses[i].t.x, baseline.sampled_poses[i].t.y, baseline.sampled_poses[i].t.z,
                    avslam.sampled_poses[i].t.x, avslam.sampled_poses[i].t.y, avslam.sampled_poses[i].t.z);
    }
    std::printf("  AV-SLAM License               : %s\n", avslam.license_id.c_str());

    // Strict commercial parity & quality gates
    bool ok = true;
    if (avslam.frames_processed < baseline.frames_processed) {
        std::printf("[FAIL] AV-SLAM processed fewer frames than baseline\n");
        ok = false;
    }
    if (avslam.tracking_lost > 0) {
        std::printf("[FAIL] AV-SLAM lost tracking on %zu frames\n", avslam.tracking_lost);
        ok = false;
    }
    if (avslam.median_rmse_m > 0.060) {
        std::printf("[FAIL] AV-SLAM median RMSE %.4f m exceeded 0.060 m gate\n", avslam.median_rmse_m);
        ok = false;
    }
    if (avslam.avg_slam_ms > 25.0) {
        std::printf("[FAIL] AV-SLAM avg frame time %.2f ms exceeded 25 ms mobile budget\n", avslam.avg_slam_ms);
        ok = false;
    }

    if (ok) {
        std::printf("\n=== A/B BENCHMARK VERDICT: AV-SLAM PASSED ALL PARITY & QUALITY GATES ===\n");
    }
    return ok ? 0 : 1;
}
