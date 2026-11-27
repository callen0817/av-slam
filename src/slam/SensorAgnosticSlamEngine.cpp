#include "avscan/slam/SensorAgnosticSlamEngine.hpp"
#include <algorithm>
#include <cmath>

namespace avscan {
namespace slam {

SensorAgnosticSlamEngine::SensorAgnosticSlamEngine(const AvSlamConfig& config)
    : core_engine_(config) {
    reset();
}

void SensorAgnosticSlamEngine::setSensorRigConfig(const sensors::GenericSensorRigConfig& rig) {
    core_engine_.setSensorRigConfig(rig);
    cam_extrinsics_ = rig.camera_extrinsics;
}

void SensorAgnosticSlamEngine::setExtrinsicCalibration(const core::ExtrinsicCalibrationProfile& profile) {
    cam_extrinsics_ = profile;
    core_engine_.setExtrinsicCalibration(profile);
}

void SensorAgnosticSlamEngine::setDepthProvider(std::shared_ptr<sensors::IDepthProvider> provider) {
    depth_provider_ = std::move(provider);
}

void SensorAgnosticSlamEngine::reset() {
    core_engine_.reset();
    agnostic_status_ = AgnosticSlamStatus{};
    has_prev_mono_frame_ = false;
    prev_mono_frame_ = core::RgbFrame{};
    mono_state_ = core::SlamState{};
    mono_trajectory_.clear();
    using_mono_fallback_ = false;
    has_gnss_anchor_ = false;
    gnss_world_offset_m_ = {0.0, 0.0, 0.0};
}

std::string SensorAgnosticSlamEngine::describeChannelMask(uint32_t mask) {
    if (mask == SENSOR_NONE) return "IDLE";
    std::string out;
    auto add = [&](const char* tok) {
        if (!out.empty()) out += " + ";
        out += tok;
    };
    if (mask & SENSOR_LIDAR)        add("LiDAR");
    if (mask & SENSOR_DEPTH_CAMERA) add("Depth");
    if (mask & SENSOR_RGB_CAMERA)   add("RGB");
    if (mask & SENSOR_IMU)          add("IMU");
    if (mask & SENSOR_GNSS_RTK)     add("GPS/RTK");
    return out;
}

void SensorAgnosticSlamEngine::fuseGnssRtkMeasurement(const sensors::GnssRtkMeasurement& fix) {
    if (!fix.valid) return;
    core::SlamState st = getCurrentState();

    if (!has_gnss_anchor_) {
        gnss_world_offset_m_ = st.T_world_body.t - fix.enu_position_m;
        has_gnss_anchor_ = true;
    }

    const math::Vec3d target_world = fix.enu_position_m + gnss_world_offset_m_;
    const math::Vec3d residual = target_world - st.T_world_body.t;
    agnostic_status_.last_gnss_residual_m = residual.norm();
    agnostic_status_.latest_gnss_quality = fix.fix_quality;
    agnostic_status_.gnss_rtk_locked = (fix.fix_quality == sensors::GnssFixQuality::RTK_FIXED ||
                                        fix.fix_quality == sensors::GnssFixQuality::RTK_FLOAT);

    // Covariance-weighted Kalman blend (RTK_FIXED ~2cm pulls strongly, single GPS ~2.5m pulls gently)
    const double h_var = std::max(1e-4, fix.horizontal_sigma_m * fix.horizontal_sigma_m);
    const double v_var = std::max(1e-4, fix.vertical_sigma_m * fix.vertical_sigma_m);
    const double prior_var = std::max(0.02, st.position_uncertainty_m * st.position_uncertainty_m);
    const double k_xy = std::clamp(prior_var / (prior_var + h_var), 0.02, 0.85);
    const double k_z  = std::clamp(prior_var / (prior_var + v_var), 0.01, 0.65);

    math::SE3d fused = st.T_world_body;
    fused.t.x += k_xy * residual.x;
    fused.t.y += k_xy * residual.y;
    fused.t.z += k_z  * residual.z;

    if (using_mono_fallback_) {
        mono_state_.T_world_body = fused;
    } else {
        core_engine_.resetPose(fused);
    }
    agnostic_status_.gnss_factors_applied++;
}

bool SensorAgnosticSlamEngine::runMonocularDirectStep(const core::RgbFrame& rgb,
                                                      const std::vector<core::ImuMeasurement>& imus,
                                                      core::LidarFrame& out_body_frame,
                                                      core::LidarFrame& out_world_frame,
                                                      EstimatorMetrics& out_metrics) {
    using_mono_fallback_ = true;
    out_body_frame = core::LidarFrame{};
    out_world_frame = core::LidarFrame{};
    out_metrics = EstimatorMetrics{};

    // Integrate IMU angular velocity if an IMU is paired with the monocular camera
    if (!imus.empty()) {
        double prev_t = (mono_state_.timestamp_sec > 0.0) ? mono_state_.timestamp_sec : imus.front().timestamp_sec;
        for (const auto& m : imus) {
            double dt = std::clamp(m.timestamp_sec - prev_t, 0.0, 0.05);
            prev_t = m.timestamp_sec;
            if (dt > 1e-6) {
                mono_state_.T_world_body.so3 = mono_state_.T_world_body.so3 * math::SO3d::exp(m.gyro_rads * dt);
            }
        }
    }

    // Lightweight direct gradient phase-shift visual odometry when no LiDAR/Depth is present
    if (has_prev_mono_frame_ &&
        prev_mono_frame_.intrinsics.width == rgb.intrinsics.width &&
        prev_mono_frame_.intrinsics.height == rgb.intrinsics.height &&
        rgb.intrinsics.width >= 32 && rgb.intrinsics.height >= 32) {
        const int w = rgb.intrinsics.width;
        const int h = rgb.intrinsics.height;
        double sum_gx2 = 0.0, sum_gy2 = 0.0, sum_gxt = 0.0, sum_gyt = 0.0;
        for (int y = 4; y < h - 4; y += 4) {
            for (int x = 4; x < w - 4; x += 4) {
                size_t idx = static_cast<size_t>(y * w + x);
                double gx = 0.5 * (static_cast<double>(rgb.gray_bytes[idx + 1]) - static_cast<double>(rgb.gray_bytes[idx - 1]));
                double gy = 0.5 * (static_cast<double>(rgb.gray_bytes[idx + w]) - static_cast<double>(rgb.gray_bytes[idx - w]));
                double gt = static_cast<double>(rgb.gray_bytes[idx]) - static_cast<double>(prev_mono_frame_.gray_bytes[idx]);
                sum_gx2 += gx * gx;
                sum_gy2 += gy * gy;
                sum_gxt += gx * gt;
                sum_gyt += gy * gt;
            }
        }
        double du = (sum_gx2 > 1e3) ? std::clamp(-sum_gxt / sum_gx2, -6.0, 6.0) : 0.0;
        double dv = (sum_gy2 > 1e3) ? std::clamp(-sum_gyt / sum_gy2, -6.0, 6.0) : 0.0;
        const double fx = std::max(100.0, rgb.intrinsics.fx);
        const double fy = std::max(100.0, rgb.intrinsics.fy);
        if (imus.empty()) {
            // Estimate yaw/pitch directly from optical flow when IMU-less
            mono_state_.T_world_body.so3 = mono_state_.T_world_body.so3 *
                math::SO3d::exp(math::Vec3d(dv / fy, -du / fx, 0.0));
        }
        // Emit sparse high-gradient visual landmarks into the unified point cloud
        for (int y = 8; y < h - 8; y += 12) {
            for (int x = 8; x < w - 8; x += 12) {
                size_t idx = static_cast<size_t>(y * w + x);
                double gx = std::abs(static_cast<double>(rgb.gray_bytes[idx + 1]) - static_cast<double>(rgb.gray_bytes[idx - 1]));
                double gy = std::abs(static_cast<double>(rgb.gray_bytes[idx + w]) - static_cast<double>(rgb.gray_bytes[idx - w]));
                if (gx + gy < 25.0) continue;
                constexpr double kNominalDepthM = 2.5;
                core::LidarPointXYZIRT p;
                p.x = (x - rgb.intrinsics.cx) * kNominalDepthM / fx;
                p.y = (y - rgb.intrinsics.cy) * kNominalDepthM / fy;
                p.z = kNominalDepthM;
                p.timestamp_sec = rgb.timestamp_sec;
                if (rgb.rgb_bytes.size() >= static_cast<size_t>(w * h * 3)) {
                    p.r = rgb.rgb_bytes[idx * 3 + 0];
                    p.g = rgb.rgb_bytes[idx * 3 + 1];
                    p.b = rgb.rgb_bytes[idx * 3 + 2];
                    p.colorized = true;
                }
                out_body_frame.points.push_back(p);
                math::Vec3d pw = mono_state_.T_world_body.transform(math::Vec3d(p.x, p.y, p.z));
                p.x = pw.x; p.y = pw.y; p.z = pw.z;
                out_world_frame.points.push_back(p);
            }
        }
        out_metrics.visual_update_applied = true;
    }

    prev_mono_frame_ = rgb;
    has_prev_mono_frame_ = true;
    mono_state_.timestamp_sec = rgb.timestamp_sec;
    mono_state_.status = core::TrackingStatus::TRACKING_LIVO;
    mono_trajectory_.push_back(mono_state_);
    agnostic_status_.monocular_frames_tracked++;
    agnostic_status_.frames_processed++;
    out_metrics.frames_processed = mono_trajectory_.size();
    return true;
}

bool SensorAgnosticSlamEngine::processUniversalBundle(const UniversalSensorBundle& bundle,
                                                      core::LidarFrame& out_body_frame,
                                                      core::LidarFrame& out_world_frame,
                                                      EstimatorMetrics& out_metrics) {
    const uint32_t mask = bundle.activeChannelMask();
    agnostic_status_.active_channels = mask;
    agnostic_status_.mode_summary = describeChannelMask(mask);

    if (mask == SENSOR_NONE) return false;

    // 1. If an asynchronous IDepthProvider (Core ML / ONNX-TRT) is attached and we have RGB, submit it
    std::optional<sensors::DepthFrame> active_depth = bundle.depth_frame;
    if (!active_depth.has_value() && depth_provider_ && depth_provider_->isAvailable() && bundle.rgb_frame.has_value()) {
        depth_provider_->submitRgbFrame(bundle.rgb_frame.value(), cam_extrinsics_.T_cam_lidar);
        sensors::DepthFrame inferred;
        if (depth_provider_->popLatestDepthFrame(inferred) && inferred.valid()) {
            // If LiDAR is also present, anchor inferred depth to metric scale!
            if (bundle.lidar_frame.has_value() && !bundle.lidar_frame->points.empty()) {
                sensors::LidarAnchoredDepthAligner::anchorToLidarPoints(
                    inferred, bundle.lidar_frame->points, cam_extrinsics_.T_cam_lidar);
            }
            if (inferred.is_metric_anchored || inferred.units == sensors::DepthUnits::METERS) {
                active_depth = std::move(inferred);
            }
        }
    }

    // 2. Build unified SyncBundle for AvSlamEngine whenever LiDAR OR DepthFrame is available
    sync::SyncBundle sync_bundle;
    sync_bundle.imu_measurements = bundle.imu_measurements;
    sync_bundle.has_valid_imu_coverage = (bundle.imu_measurements.size() >= 2);
    sync_bundle.rgb_frame = bundle.rgb_frame;

    bool has_geometry = false;
    if (bundle.lidar_frame.has_value() && !bundle.lidar_frame->points.empty()) {
        sync_bundle.lidar_frame = bundle.lidar_frame.value();
        has_geometry = true;
    }

    // Fuse or substitute back-projected DepthFrame points (Orbbec 336L, Apple LiDAR, or anchored Core ML/ONNX depth)
    if (active_depth.has_value() && active_depth->valid()) {
        auto depth_pts_cam = sensors::LidarAnchoredDepthAligner::backProjectToPointCloud(
            active_depth.value(), bundle.rgb_frame, has_geometry ? 8 : 4, 0.25f, 10.0f, 0.30f);
        if (!depth_pts_cam.empty()) {
            const math::SE3d T_lidar_cam = active_depth->T_cam_lidar.inverse();
            if (!has_geometry) {
                sync_bundle.lidar_frame.frame_id = bundle.sequence_id;
                sync_bundle.lidar_frame.timestamp_start_sec = active_depth->timestamp_sec;
                sync_bundle.lidar_frame.timestamp_end_sec = active_depth->timestamp_sec;
            }
            for (auto& pt : depth_pts_cam) {
                math::Vec3d pl = T_lidar_cam.transform(math::Vec3d(pt.x, pt.y, pt.z));
                pt.x = pl.x; pt.y = pl.y; pt.z = pl.z;
                sync_bundle.lidar_frame.points.push_back(pt);
            }
            has_geometry = true;
            agnostic_status_.depth_frames_fused++;
        }
    }

    bool ok = false;
    if (has_geometry) {
        using_mono_fallback_ = false;
        ok = core_engine_.processBundle(sync_bundle, out_body_frame, out_world_frame, out_metrics);
        if (ok) agnostic_status_.frames_processed++;
    } else if (bundle.rgb_frame.has_value()) {
        ok = runMonocularDirectStep(bundle.rgb_frame.value(), bundle.imu_measurements,
                                    out_body_frame, out_world_frame, out_metrics);
    }

    // 3. Apply geodetic GPS / RTK factor if a valid fix arrived in this bundle
    if (ok && bundle.gnss_fix.has_value() && bundle.gnss_fix->valid) {
        fuseGnssRtkMeasurement(bundle.gnss_fix.value());
    }
    return ok;
}

core::SlamState SensorAgnosticSlamEngine::getCurrentState() const {
    return using_mono_fallback_ ? mono_state_ : core_engine_.getCurrentState();
}

std::vector<core::SlamState> SensorAgnosticSlamEngine::getTrajectory() const {
    return using_mono_fallback_ ? mono_trajectory_ : core_engine_.getTrajectory();
}

}  // namespace slam
}  // namespace avscan
