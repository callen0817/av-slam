#include "avscan/slam/SensorAgnosticSlamEngine.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace avscan {
namespace slam {

namespace {

inline double sampleGrayBilinear(const std::vector<uint8_t>& img, int w, int h, double x, double y) {
    int x0 = static_cast<int>(std::floor(x));
    int y0 = static_cast<int>(std::floor(y));
    if (x0 < 0 || y0 < 0 || x0 + 1 >= w || y0 + 1 >= h) return 0.0;
    double fx = x - x0;
    double fy = y - y0;
    const size_t i00 = static_cast<size_t>(y0 * w + x0);
    const size_t i10 = i00 + 1;
    const size_t i01 = i00 + static_cast<size_t>(w);
    const size_t i11 = i01 + 1;
    double v0 = img[i00] * (1.0 - fx) + img[i10] * fx;
    double v1 = img[i01] * (1.0 - fx) + img[i11] * fx;
    return v0 * (1.0 - fy) + v1 * fy;
}

struct TrackedFeature {
    double u0{0.0}, v0{0.0};
    double u1{0.0}, v1{0.0};
    double du{0.0}, dv{0.0};
};

// Grid-spaced corner extraction + Lucas-Kanade optical flow with MAD outlier rejection
std::vector<TrackedFeature> trackSparseFeaturesLK(const core::RgbFrame& prev_rgb,
                                                  const core::RgbFrame& curr_rgb) {
    std::vector<TrackedFeature> tracked;
    const int w = curr_rgb.intrinsics.width;
    const int h = curr_rgb.intrinsics.height;
    if (w < 32 || h < 32 || prev_rgb.intrinsics.width != w || prev_rgb.intrinsics.height != h) return tracked;
    if (prev_rgb.gray_bytes.size() < static_cast<size_t>(w * h) ||
        curr_rgb.gray_bytes.size() < static_cast<size_t>(w * h)) {
        return tracked;
    }

    const int cols = 12;
    const int rows = 9;
    const int cell_w = std::max(12, (w - 20) / cols);
    const int cell_h = std::max(12, (h - 20) / rows);

    struct Seed { double u, v, score; };
    std::vector<Seed> seeds;
    seeds.reserve(static_cast<size_t>(cols * rows));

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            int x0 = 10 + c * cell_w;
            int y0 = 10 + r * cell_h;
            int x1 = std::min(w - 10, x0 + cell_w);
            int y1 = std::min(h - 10, y0 + cell_h);
            double best_score = 350.0;
            int bx = -1, by = -1;
            for (int y = y0; y < y1; y += 3) {
                for (int x = x0; x < x1; x += 3) {
                    size_t idx = static_cast<size_t>(y * w + x);
                    double gx = 0.5 * (static_cast<double>(prev_rgb.gray_bytes[idx + 1]) -
                                       static_cast<double>(prev_rgb.gray_bytes[idx - 1]));
                    double gy = 0.5 * (static_cast<double>(prev_rgb.gray_bytes[idx + w]) -
                                       static_cast<double>(prev_rgb.gray_bytes[idx - w]));
                    // Cornerness proxy: strong gradient in both axes or strong edge
                    double sc = gx * gx + gy * gy;
                    if (sc > best_score) {
                        best_score = sc;
                        bx = x;
                        by = y;
                    }
                }
            }
            if (bx >= 0) seeds.push_back({static_cast<double>(bx), static_cast<double>(by), best_score});
        }
    }

    // Track each seed with 5x5 patch Gauss-Newton Lucas-Kanade
    tracked.reserve(seeds.size());
    for (const auto& s : seeds) {
        double u = s.u;
        double v = s.v;
        bool valid = true;
        for (int iter = 0; iter < 5; ++iter) {
            double hxx = 0.0, hxy = 0.0, hyy = 0.0, bx = 0.0, by = 0.0;
            for (int dy = -2; dy <= 2; ++dy) {
                for (int dx = -2; dx <= 2; ++dx) {
                    double px = s.u + dx;
                    double py = s.v + dy;
                    double qx = u + dx;
                    double qy = v + dy;
                    if (qx < 3.0 || qy < 3.0 || qx > w - 4.0 || qy > h - 4.0) {
                        valid = false;
                        break;
                    }
                    double i0 = sampleGrayBilinear(prev_rgb.gray_bytes, w, h, px, py);
                    double i1 = sampleGrayBilinear(curr_rgb.gray_bytes, w, h, qx, qy);
                    double gx = 0.5 * (sampleGrayBilinear(curr_rgb.gray_bytes, w, h, qx + 1.0, qy) -
                                       sampleGrayBilinear(curr_rgb.gray_bytes, w, h, qx - 1.0, qy));
                    double gy = 0.5 * (sampleGrayBilinear(curr_rgb.gray_bytes, w, h, qx, qy + 1.0) -
                                       sampleGrayBilinear(curr_rgb.gray_bytes, w, h, qx, qy - 1.0));
                    double err = i1 - i0;
                    hxx += gx * gx;
                    hxy += gx * gy;
                    hyy += gy * gy;
                    bx += gx * err;
                    by += gy * err;
                }
                if (!valid) break;
            }
            if (!valid) break;
            double det = hxx * hyy - hxy * hxy;
            if (det < 1e-3) {
                valid = false;
                break;
            }
            double step_u = -(hyy * bx - hxy * by) / det;
            double step_v = -(-hxy * bx + hxx * by) / det;
            if (!std::isfinite(step_u) || !std::isfinite(step_v) ||
                std::abs(step_u) > 12.0 || std::abs(step_v) > 12.0) {
                valid = false;
                break;
            }
            u += step_u;
            v += step_v;
            if (step_u * step_u + step_v * step_v < 0.01) break;
        }
        double du = u - s.u;
        double dv = v - s.v;
        if (valid && std::hypot(du, dv) < 28.0) {
            tracked.push_back({s.u, s.v, u, v, du, dv});
        }
    }

    if (tracked.size() < 8) return tracked;

    // Median + MAD outlier rejection
    std::vector<double> dus, dvs;
    dus.reserve(tracked.size());
    dvs.reserve(tracked.size());
    for (const auto& f : tracked) {
        dus.push_back(f.du);
        dvs.push_back(f.dv);
    }
    auto nth_med = [](std::vector<double>& v) {
        size_t m = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + m, v.end());
        return v[m];
    };
    double med_u = nth_med(dus);
    double med_v = nth_med(dvs);

    std::vector<double> errs;
    errs.reserve(tracked.size());
    for (const auto& f : tracked) {
        errs.push_back(std::hypot(f.du - med_u, f.dv - med_v));
    }
    double mad = std::max(1.2, nth_med(errs) * 2.5);

    std::vector<TrackedFeature> inliers;
    inliers.reserve(tracked.size());
    for (const auto& f : tracked) {
        if (std::hypot(f.du - med_u, f.dv - med_v) <= mad) {
            inliers.push_back(f);
        }
    }
    return inliers;
}

}  // namespace

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
    last_mono_kf_pose_ = math::SE3d::Identity();
    has_mono_kf_ = false;
    mono_trajectory_.clear();
    using_mono_fallback_ = false;
    has_prev_depth_rgb_ = false;
    prev_depth_rgb_ = core::RgbFrame{};
    last_depth_rgb_ts_ = 0.0;
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

    const int w = rgb.intrinsics.width;
    const int h = rgb.intrinsics.height;
    const double fx = std::max(150.0, rgb.intrinsics.fx);
    const double fy = std::max(150.0, rgb.intrinsics.fy);
    const double cx = (rgb.intrinsics.cx > 1.0) ? rgb.intrinsics.cx : (w * 0.5);
    const double cy = (rgb.intrinsics.cy > 1.0) ? rgb.intrinsics.cy : (h * 0.5);

    // 1. Integrate IMU angular velocity when an IMU (e.g. Surface/Host IMU or 336L IMU) is paired
    math::Vec3d d_rot_body{0.0, 0.0, 0.0};
    if (!imus.empty()) {
        double prev_t = (mono_state_.timestamp_sec > 0.0) ? mono_state_.timestamp_sec : imus.front().timestamp_sec;
        for (const auto& m : imus) {
            double dt = std::clamp(m.timestamp_sec - prev_t, 0.0, 0.05);
            prev_t = m.timestamp_sec;
            if (dt > 1e-6) {
                d_rot_body = d_rot_body + m.gyro_rads * dt;
            }
        }
        mono_state_.T_world_body.so3 = mono_state_.T_world_body.so3 * math::SO3d::exp(d_rot_body);
    }

    // 2. Sparse Lucas-Kanade feature tracking + MAD outlier rejection + parallax 3D triangulation
    if (has_prev_mono_frame_) {
        auto inliers = trackSparseFeaturesLK(prev_mono_frame_, rgb);
        if (inliers.size() >= 8) {
            double sum_du = 0.0, sum_dv = 0.0;
            for (const auto& f : inliers) {
                sum_du += f.du;
                sum_dv += f.dv;
            }
            const double mean_du = sum_du / static_cast<double>(inliers.size());
            const double mean_dv = sum_dv / static_cast<double>(inliers.size());

            // In FLU body frame: +X forward, +Y left, +Z up.
            // Camera optical: +z_cam = +X_body, +x_cam = -Y_body, +y_cam = -Z_body.
            // Yaw left (+Z_body) shifts pixels RIGHT (+du). Pitch down (+Y_body) shifts pixels UP (-dv).
            double dyaw = 0.0;
            double dpitch = 0.0;
            if (imus.empty()) {
                dyaw = std::clamp(mean_du / fx, -0.08, 0.08);
                dpitch = std::clamp(-mean_dv / fy, -0.08, 0.08);
                // Damp tiny noise when stationary (ZUPT for monocular)
                if (std::hypot(mean_du, mean_dv) < 0.35) {
                    dyaw = 0.0;
                    dpitch = 0.0;
                }
                mono_state_.T_world_body.so3 = mono_state_.T_world_body.so3 *
                    math::SO3d::exp(math::Vec3d(0.0, dpitch, dyaw));
            } else {
                dyaw = d_rot_body.z;
                dpitch = d_rot_body.y;
            }

            // Compute residual translational flow after compensating pure rotation
            double sum_res_u = 0.0, sum_res_v = 0.0, sum_div = 0.0;
            size_t div_cnt = 0;
            for (const auto& f : inliers) {
                const double rot_u = dyaw * fx;
                const double rot_v = -dpitch * fy;
                const double ru = f.du - rot_u;
                const double rv = f.dv - rot_v;
                sum_res_u += ru;
                sum_res_v += rv;
                const double rx = (f.u0 - cx) / fx;
                const double ry = (f.v0 - cy) / fy;
                const double r2 = rx * rx + ry * ry;
                if (r2 > 0.02) {
                    sum_div += (rx * (ru / fx) + ry * (rv / fy)) / r2;
                    div_cnt++;
                }
            }
            const double res_u = sum_res_u / static_cast<double>(inliers.size());
            const double res_v = sum_res_v / static_cast<double>(inliers.size());
            const double radial_div = (div_cnt > 0) ? (sum_div / static_cast<double>(div_cnt)) : 0.0;

            // Gentle bounded translation update (only when motion exceeds stationary deadband)
            math::Vec3d dt_body{0.0, 0.0, 0.0};
            if (std::abs(radial_div) > 0.002 || std::hypot(res_u, res_v) > 0.8) {
                constexpr double kNominalSceneScaleM = 2.0;
                dt_body.x = std::clamp(radial_div * kNominalSceneScaleM, -0.06, 0.06);
                dt_body.y = std::clamp((res_u / fx) * kNominalSceneScaleM, -0.05, 0.05);
                dt_body.z = std::clamp((res_v / fy) * kNominalSceneScaleM, -0.03, 0.03);
                mono_state_.T_world_body.t = mono_state_.T_world_body.t +
                    (mono_state_.T_world_body.so3 * dt_body);
            }

            const double baseline = dt_body.norm();
            // Triangulate tracked features into FLU body frame (and world frame) with color
            for (const auto& f : inliers) {
                const double rot_u = dyaw * fx;
                const double rot_v = -dpitch * fy;
                const double parallax_px = std::hypot(f.du - rot_u, f.dv - rot_v);
                double depth_m = 2.2;
                if (baseline > 0.004 && parallax_px > 0.4) {
                    depth_m = std::clamp((fx * baseline) / parallax_px, 0.55, 6.0);
                }
                const double x_cam = (f.u1 - cx) * depth_m / fx;
                const double y_cam = (f.v1 - cy) * depth_m / fy;
                const double z_cam = depth_m;

                // Convert camera optical -> FLU body frame (+X forward, +Y left, +Z up)
                core::LidarPointXYZIRT p;
                p.x = z_cam;
                p.y = -x_cam;
                p.z = -y_cam;
                p.range_m = static_cast<float>(std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z));
                p.timestamp_sec = rgb.timestamp_sec;
                p.intensity = 180.0f;

                int iu = std::clamp(static_cast<int>(std::round(f.u1)), 0, w - 1);
                int iv = std::clamp(static_cast<int>(std::round(f.v1)), 0, h - 1);
                size_t pix = static_cast<size_t>(iv * w + iu);
                if (rgb.rgb_bytes.size() >= static_cast<size_t>(w * h * 3)) {
                    p.r = rgb.rgb_bytes[pix * 3 + 0];
                    p.g = rgb.rgb_bytes[pix * 3 + 1];
                    p.b = rgb.rgb_bytes[pix * 3 + 2];
                    p.colorized = true;
                }
                out_body_frame.points.push_back(p);
                math::Vec3d pw = mono_state_.T_world_body.transform(math::Vec3d(p.x, p.y, p.z));
                p.x = pw.x; p.y = pw.y; p.z = pw.z;
                out_world_frame.points.push_back(p);
            }
            out_metrics.visual_update_applied = true;
        }
    }

    // Emit proper keyframes so WAL, trajectory, and Tier-2 cleanup work for Monocular-only!
    const double kf_trans = (mono_state_.T_world_body.t - last_mono_kf_pose_.t).norm();
    const double kf_rot_deg =
        (last_mono_kf_pose_.so3.inverse() * mono_state_.T_world_body.so3).log().norm() * math::kRadToDeg;
    if (!out_body_frame.points.empty() && (!has_mono_kf_ || kf_trans >= 0.20 || kf_rot_deg >= 10.0)) {
        out_metrics.keyframe_emitted = true;
        last_mono_kf_pose_ = mono_state_.T_world_body;
        has_mono_kf_ = true;
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
            if (bundle.lidar_frame.has_value() && !bundle.lidar_frame->points.empty()) {
                sensors::LidarAnchoredDepthAligner::anchorToLidarPoints(
                    inferred, bundle.lidar_frame->points, cam_extrinsics_.T_cam_lidar);
            }
            if (inferred.is_metric_anchored || inferred.units == sensors::DepthUnits::METERS) {
                active_depth = std::move(inferred);
            }
        }
    }

    // 2. Build unified SyncBundle for AvSlamEngine
    sync::SyncBundle sync_bundle;
    sync_bundle.imu_measurements = bundle.imu_measurements;
    sync_bundle.has_valid_imu_coverage = (bundle.imu_measurements.size() >= 2);
    sync_bundle.rgb_frame = bundle.rgb_frame;

    const bool has_lidar = (bundle.lidar_frame.has_value() && !bundle.lidar_frame->points.empty());
    bool has_geometry = false;

    if (has_lidar) {
        // CRITICAL: When metric 360-deg LiDAR (Airy/Livox/Ouster) is present, keep LiDAR geometry
        // 100% pure for AvSlamEngine ICP so noisy/uncalibrated narrow-FOV depth camera frames
        // NEVER poison LiDAR pose estimation or create double-wall ghosting!
        sync_bundle.lidar_frame = bundle.lidar_frame.value();
        has_geometry = true;
        if (active_depth.has_value() && active_depth->valid()) {
            agnostic_status_.depth_frames_fused++;
        }
    } else if (active_depth.has_value() && active_depth->valid()) {
        // Depth-only / RGB-D standalone mode (e.g. Orbbec Gemini 336L without LiDAR):
        // Range-gate stereo depth to 0.30..4.50 m (avoids quadratic far-field stereo noise)
        // and convert optical camera coordinates (+Z fwd, +X right, +Y down) to FLU body frame
        // (+X fwd, +Y left, +Z up) so AvSlamEngine gravity/floor leveling & ICP work natively.
        auto depth_pts_cam = sensors::LidarAnchoredDepthAligner::backProjectToPointCloud(
            active_depth.value(), bundle.rgb_frame, 4, 0.30f, 4.50f, 0.30f);
        if (!depth_pts_cam.empty()) {
            sync_bundle.lidar_frame.frame_id = bundle.sequence_id;
            sync_bundle.lidar_frame.timestamp_start_sec = active_depth->timestamp_sec;
            sync_bundle.lidar_frame.timestamp_end_sec = active_depth->timestamp_sec;
            sync_bundle.lidar_frame.points.reserve(depth_pts_cam.size());
            for (auto& pt : depth_pts_cam) {
                const double x_cam = pt.x;
                const double y_cam = pt.y;
                const double z_cam = pt.z;
                pt.x = z_cam;
                pt.y = -x_cam;
                pt.z = -y_cam;
                sync_bundle.lidar_frame.points.push_back(pt);
            }
            has_geometry = true;
            agnostic_status_.depth_frames_fused++;

            // If no hardware IMU is attached during Depth-only mode, synthesize a smooth
            // optical-flow gyro prior from consecutive RGB frames so rotations don't slip!
            if (sync_bundle.imu_measurements.size() < 2 && bundle.rgb_frame.has_value()) {
                if (has_prev_depth_rgb_) {
                    double dt = active_depth->timestamp_sec - last_depth_rgb_ts_;
                    if (dt > 0.005 && dt < 0.25) {
                        auto inliers = trackSparseFeaturesLK(prev_depth_rgb_, bundle.rgb_frame.value());
                        if (inliers.size() >= 8) {
                            double sdu = 0.0, sdv = 0.0;
                            for (const auto& f : inliers) { sdu += f.du; sdv += f.dv; }
                            double mdu = sdu / static_cast<double>(inliers.size());
                            double mdv = sdv / static_cast<double>(inliers.size());
                            double fx = std::max(150.0, bundle.rgb_frame->intrinsics.fx);
                            double fy = std::max(150.0, bundle.rgb_frame->intrinsics.fy);
                            double wz = (std::abs(mdu) > 0.35) ? std::clamp((mdu / fx) / dt, -2.5, 2.5) : 0.0;
                            double wy = (std::abs(mdv) > 0.35) ? std::clamp((-mdv / fy) / dt, -2.5, 2.5) : 0.0;
                            core::ImuMeasurement m0, m1;
                            m0.timestamp_sec = last_depth_rgb_ts_;
                            m1.timestamp_sec = active_depth->timestamp_sec;
                            m0.gyro_rads = m1.gyro_rads = math::Vec3d(0.0, wy, wz);
                            m0.accel_mps2 = m1.accel_mps2 = math::Vec3d(0.0, 0.0, 9.81);
                            sync_bundle.imu_measurements = {m0, m1};
                            sync_bundle.has_valid_imu_coverage = true;
                        }
                    }
                }
                prev_depth_rgb_ = bundle.rgb_frame.value();
                last_depth_rgb_ts_ = active_depth->timestamp_sec;
                has_prev_depth_rgb_ = true;
            }
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
