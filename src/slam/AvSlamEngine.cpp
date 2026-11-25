#include "avscan/slam/AvSlamEngine.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace avscan {
namespace slam {

namespace {

constexpr double kGravityMag = AvImuPreintegrator::kGravityMag;

}  // namespace

AvSlamEngine::AvSlamEngine(const AvSlamConfig& config)
    : config_(config), surfel_map_(config.map_config) {
    preintegrator_.setNoiseModel(config_.sensor_rig.noise);
    // Align front-end range envelope with active sensor rig noise model
    config_.front_end_filters.min_range_m = config_.sensor_rig.noise.min_valid_range_m;
    config_.front_end_filters.max_range_m = config_.sensor_rig.noise.max_valid_range_m;
    config_.front_end_filters.enable_self_mask_box = config_.sensor_rig.noise.enable_self_mask_box;
    config_.front_end_filters.self_mask_min_m = config_.sensor_rig.noise.self_mask_min_m;
    config_.front_end_filters.self_mask_max_m = config_.sensor_rig.noise.self_mask_max_m;
    config_.map_config.range_noise_sigma_m = config_.sensor_rig.noise.range_noise_sigma_m;
    surfel_map_.setConfig(config_.map_config);
    reset();
}

void AvSlamEngine::setSensorRigConfig(const sensors::GenericSensorRigConfig& rig) {
    const double d_rot_deg =
        (config_.sensor_rig.T_imu_lidar.so3.inverse() * rig.T_imu_lidar.so3).log().norm() * math::kRadToDeg;
    config_.sensor_rig = rig;
    preintegrator_.setNoiseModel(rig.noise);
    config_.front_end_filters.min_range_m = rig.noise.min_valid_range_m;
    config_.front_end_filters.max_range_m = rig.noise.max_valid_range_m;
    config_.front_end_filters.enable_self_mask_box = rig.noise.enable_self_mask_box;
    config_.front_end_filters.self_mask_min_m = rig.noise.self_mask_min_m;
    config_.front_end_filters.self_mask_max_m = rig.noise.self_mask_max_m;
    config_.front_end_filters.enable_flying_pixel_filter = rig.is_ring_ordered;
    config_.map_config.range_noise_sigma_m = rig.noise.range_noise_sigma_m;
    surfel_map_.setConfig(config_.map_config);
    // If an external sensor delivers a late DIFOP/calibration packet shifting T_imu_lidar > 5 deg during startup,
    // flush stale pre-calibration surfels so they never twist early tracking!
    if (d_rot_deg > 5.0 && trajectory_.size() <= 15) {
        surfel_map_.clear();
    }
}

void AvSlamEngine::setFrontEndFilterConfig(const filters::FrontEndFilterConfig& filter_cfg) {
    config_.front_end_filters = filter_cfg;
}

void AvSlamEngine::setFactoryCalibration(const core::AiryFactoryCalibration& calib) {
    // Seamless adapter from legacy RoboSense Airy DIFOP struct into hardware-agnostic rig config
    const auto prev_cam = config_.sensor_rig.camera_extrinsics;
    auto rig = sensors::GenericSensorRigConfig::fromAiryFactoryCalibration(calib);
    rig.camera_extrinsics = prev_cam;
    setSensorRigConfig(rig);
}

void AvSlamEngine::setExtrinsicCalibration(const core::ExtrinsicCalibrationProfile& profile) {
    config_.sensor_rig.camera_extrinsics = profile;
}

void AvSlamEngine::resetPose(const math::SE3d& new_T_world_body) {
    state_.T_world_body = new_T_world_body;
    state_.velocity_mps = {0.0, 0.0, 0.0};
    state_.status = core::TrackingStatus::TRACKING_LIO;
}

void AvSlamEngine::reset() {
    state_ = core::SlamState{};
    state_.gravity_mps2 = {0.0, 0.0, -kGravityMag};
    state_.status = core::TrackingStatus::UNINITIALIZED;

    P_ = Mat17::Zero();
    for (int i = 0; i < 3; ++i) {
        P_(i + 0, i + 0)   = 1e-4;  // Orientation variance (rad^2)
        P_(i + 3, i + 3)   = 1e-4;  // Position variance (m^2)
        P_(i + 6, i + 6)   = 1e-2;  // Velocity variance (m^2/s^2)
        P_(i + 9, i + 9)   = 1e-4;  // Gyro bias variance
        P_(i + 12, i + 12) = 1e-3;  // Accel bias variance
    }
    P_(15, 15) = 1e-4;              // S^2 gravity tangent variance (m^2/s^4)
    P_(16, 16) = 1e-4;

    surfel_map_.clear();
    init_imu_accum_.clear();
    has_last_imu_ = false;
    preintegrator_.setAccelScale(1.0);
    trajectory_.clear();
    recent_pose_history_.clear();
    last_keyframe_pose_ = math::SE3d::Identity();
    has_first_keyframe_ = false;
    stats_ = AvSlamRuntimeStats{};
}

void AvSlamEngine::injectTerrestrialAnchor(const math::SE3d& T_world_anchor,
                                           const std::vector<core::LidarPointXYZIRT>& anchor_local_points) {
    if (anchor_local_points.empty()) return;
    std::vector<math::Vec3d> world_pts;
    world_pts.reserve(anchor_local_points.size());
    for (const auto& pt : anchor_local_points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
        world_pts.push_back(T_world_anchor.transform(math::Vec3d(pt.x, pt.y, pt.z)));
    }
    surfel_map_.insertWorldPoints(world_pts, stats_.total_frames + 1, 4.0, true);
    state_.T_world_body = T_world_anchor;
    state_.velocity_mps = {0.0, 0.0, 0.0};
    // Tighten pose uncertainty to anchor rigidity
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < kStateDim; ++j) {
            P_(i, j) = P_(j, i) = 0.0;
        }
        P_(i, i) = 1e-5;
    }
}

AvSlamEngine::GravityBasis AvSlamEngine::computeGravityTangentBasis(const math::Vec3d& g) {
    const math::Vec3d g_hat = g.normalized();
    const math::Vec3d ref = (std::abs(g_hat.x) < 0.85) ? math::Vec3d(1.0, 0.0, 0.0) : math::Vec3d(0.0, 1.0, 0.0);
    GravityBasis basis;
    basis.b1 = (ref - g_hat * ref.dot(g_hat)).normalized();
    basis.b2 = g_hat.cross(basis.b1).normalized();
    return basis;
}

math::Vec3d AvSlamEngine::gravityBoxPlus(const math::Vec3d& g, const GravityBasis& basis, double dg0, double dg1) {
    const math::Vec3d u = basis.b1 * dg0 + basis.b2 * dg1;
    const double u_norm = u.norm();
    const double alpha = u_norm / kGravityMag;
    if (alpha < 1e-8) {
        return (g + u).normalized() * kGravityMag;
    }
    // Exact geodesic motion on S^2 sphere of radius kGravityMag (conserves ||g|| == 9.80665 exactly!)
    const double g_norm = g.norm();
    const math::Vec3d g_unit = (g_norm > 1e-6) ? (g / g_norm) : math::Vec3d(0.0, 0.0, -1.0);
    return (g_unit * std::cos(alpha) + (u / u_norm) * std::sin(alpha)) * kGravityMag;
}

math::Vec2d AvSlamEngine::gravityBoxMinus(const math::Vec3d& g_target, const math::Vec3d& g_base, const GravityBasis& basis) {
    const math::Vec3d g0 = g_base.normalized();
    const math::Vec3d g1 = g_target.normalized();
    const double cos_a = std::clamp(g0.dot(g1), -1.0, 1.0);
    const double alpha = std::acos(cos_a);
    const math::Vec3d perp = g1 - g0 * cos_a;
    const double p_norm = perp.norm();
    if (p_norm < 1e-10 || alpha < 1e-10) {
        return {0.0, 0.0};
    }
    const math::Vec3d u = perp * ((kGravityMag * alpha) / p_norm);
    return {basis.b1.dot(u), basis.b2.dot(u)};
}

bool AvSlamEngine::choleskyDecompose(const Mat17& A, Mat17& out_L) {
    out_L = Mat17::Zero();
    for (int i = 0; i < kStateDim; ++i) {
        for (int j = 0; j <= i; ++j) {
            double sum = A(i, j);
            for (int k = 0; k < j; ++k) {
                sum -= out_L(i, k) * out_L(j, k);
            }
            if (i == j) {
                if (sum <= 1e-15 || !std::isfinite(sum)) return false;
                out_L(i, j) = std::sqrt(sum);
            } else {
                out_L(i, j) = sum / out_L(j, j);
            }
        }
    }
    return true;
}

void AvSlamEngine::choleskySolve(const Mat17& L, const double b[kStateDim], double x[kStateDim]) {
    double y[kStateDim]{};
    // Forward substitution: L * y = b
    for (int i = 0; i < kStateDim; ++i) {
        double s = b[i];
        for (int k = 0; k < i; ++k) s -= L(i, k) * y[k];
        y[i] = s / L(i, i);
    }
    // Backward substitution: L^T * x = y
    for (int i = kStateDim - 1; i >= 0; --i) {
        double s = y[i];
        for (int k = i + 1; k < kStateDim; ++k) s -= L(k, i) * x[k];
        x[i] = s / L(i, i);
    }
}

bool AvSlamEngine::choleskyInverseFromL(const Mat17& L, Mat17& out_inv) {
    // Invert lower-triangular L -> Linv, then compute A^-1 = Linv^T * Linv
    Mat17 Linv = Mat17::Zero();
    for (int i = 0; i < kStateDim; ++i) {
        if (std::abs(L(i, i)) < 1e-14) return false;
        Linv(i, i) = 1.0 / L(i, i);
        for (int j = 0; j < i; ++j) {
            double s = 0.0;
            for (int k = j; k < i; ++k) {
                s -= L(i, k) * Linv(k, j);
            }
            Linv(i, j) = s / L(i, i);
        }
    }
    out_inv = Mat17::Zero();
    for (int i = 0; i < kStateDim; ++i) {
        for (int j = i; j < kStateDim; ++j) {
            double s = 0.0;
            for (int k = j; k < kStateDim; ++k) {
                s += Linv(k, i) * Linv(k, j);
            }
            out_inv(i, j) = out_inv(j, i) = s;
        }
    }
    return true;
}

bool AvSlamEngine::solve6x6CholeskyOrGauss(double A[6][6], const double b[6], double x[6]) {
    double L[6][6]{};
    bool spd = true;
    for (int i = 0; i < 6 && spd; ++i) {
        for (int j = 0; j <= i; ++j) {
            double s = A[i][j];
            for (int k = 0; k < j; ++k) s -= L[i][k] * L[j][k];
            if (i == j) {
                if (s <= 1e-14) { spd = false; break; }
                L[i][j] = std::sqrt(s);
            } else {
                L[i][j] = s / L[j][j];
            }
        }
    }
    if (spd) {
        double y[6]{};
        for (int i = 0; i < 6; ++i) {
            double s = b[i];
            for (int k = 0; k < i; ++k) s -= L[i][k] * y[k];
            y[i] = s / L[i][i];
        }
        for (int i = 5; i >= 0; --i) {
            double s = y[i];
            for (int k = i + 1; k < 6; ++k) s -= L[k][i] * x[k];
            x[i] = s / L[i][i];
        }
        return true;
    }
    return false;
}

bool AvSlamEngine::initializeImuGravity(const std::vector<core::ImuMeasurement>& imus) {
    for (const auto& m : imus) {
        init_imu_accum_.push_back(m);
    }
    if (init_imu_accum_.size() < 20) {
        state_.status = core::TrackingStatus::INITIALIZING_IMU;
        return false;
    }

    math::Vec3d mean_acc(0.0, 0.0, 0.0);
    math::Vec3d mean_gyro(0.0, 0.0, 0.0);
    double max_gyro = 0.0;
    for (const auto& m : init_imu_accum_) {
        mean_acc += m.accel_mps2;
        mean_gyro += m.gyro_rads;
        max_gyro = std::max(max_gyro, m.gyro_rads.norm());
    }
    const double inv_n = 1.0 / static_cast<double>(init_imu_accum_.size());
    mean_acc *= inv_n;
    mean_gyro *= inv_n;

    const double acc_norm = mean_acc.norm();
    if (acc_norm > 5.0 && acc_norm < 15.0) {
        const math::Vec3d z_body_up = mean_acc / acc_norm;
        const math::Vec3d x_seed = (std::abs(z_body_up.x) < 0.9)
            ? math::Vec3d(1.0, 0.0, 0.0)
            : math::Vec3d(0.0, 1.0, 0.0);
        const math::Vec3d y_body = z_body_up.cross(x_seed).normalized();
        const math::Vec3d x_body = y_body.cross(z_body_up).normalized();

        math::Mat3d R_wb;
        R_wb(0, 0) = x_body.x;    R_wb(0, 1) = x_body.y;    R_wb(0, 2) = x_body.z;
        R_wb(1, 0) = y_body.x;    R_wb(1, 1) = y_body.y;    R_wb(1, 2) = y_body.z;
        R_wb(2, 0) = z_body_up.x; R_wb(2, 1) = z_body_up.y; R_wb(2, 2) = z_body_up.z;

        state_.T_world_body = math::SE3d(R_wb, math::Vec3d(0.0, 0.0, 0.0));
        state_.gravity_mps2 = math::Vec3d(0.0, 0.0, -kGravityMag);

        if (max_gyro < 0.05) {
            preintegrator_.setAccelScale(kGravityMag / acc_norm);
            state_.bias_gyro_rads = mean_gyro;
            state_.bias_accel_mps2 = {0.0, 0.0, 0.0};
        } else {
            preintegrator_.setAccelScale(1.0);
            state_.bias_gyro_rads = {0.0, 0.0, 0.0};
            state_.bias_accel_mps2 = {0.0, 0.0, 0.0};
            for (int i = 0; i < 3; ++i) {
                P_(i + 9, i + 9)   = 1e-3;
                P_(i + 12, i + 12) = 1e-2;
            }
            P_(15, 15) = 1e-2;
            P_(16, 16) = 1e-2;
        }
    }

    state_.velocity_mps = {0.0, 0.0, 0.0};
    state_.timestamp_sec = init_imu_accum_.back().timestamp_sec;
    last_imu_ = init_imu_accum_.back();
    has_last_imu_ = true;
    state_.status = core::TrackingStatus::TRACKING_LIO;
    return true;
}

void AvSlamEngine::propagatePriorCovariance(const core::SlamState& state_prev,
                                            const PreintegratedImuFactor& preint) {
    const double dt = preint.dt_total;
    if (dt <= 1e-7) return;

    const math::Mat3d& R_i = state_prev.T_world_body.so3.R;
    const math::Mat3d dR_T = preint.delta_R.R.transpose();
    const math::Mat3d F_pr = (R_i * math::SO3d::hat(preint.delta_p)) * (-1.0);
    const math::Mat3d F_vr = (R_i * math::SO3d::hat(preint.delta_v)) * (-1.0);
    const math::Mat3d R_Jpb = R_i * preint.J_p_bg;
    const math::Mat3d R_Jpa = R_i * preint.J_p_ba;
    const math::Mat3d R_Jvb = R_i * preint.J_v_bg;
    const math::Mat3d R_Jva = R_i * preint.J_v_ba;
    const GravityBasis gb = computeGravityTangentBasis(state_prev.gravity_mps2);

    // Single sweep-level 17x17 state transition matrix F
    Mat17 F = Mat17::Identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            F(r + 0, c + 0)   = dR_T(r, c);
            F(r + 0, c + 9)   = preint.J_R_bg(r, c);
            F(r + 3, c + 0)   = F_pr(r, c);
            F(r + 3, c + 9)   = R_Jpb(r, c);
            F(r + 3, c + 12)  = R_Jpa(r, c);
            F(r + 6, c + 0)   = F_vr(r, c);
            F(r + 6, c + 9)   = R_Jvb(r, c);
            F(r + 6, c + 12)  = R_Jva(r, c);
        }
        F(r + 3, r + 6) = dt;
        F(r + 3, 15) = 0.5 * dt * dt * gb.b1[r];
        F(r + 3, 16) = 0.5 * dt * dt * gb.b2[r];
        F(r + 6, 15) = dt * gb.b1[r];
        F(r + 6, 16) = dt * gb.b2[r];
    }

    // Sparse-skipping FP = F * P_
    Mat17 FP = Mat17::Zero();
    for (int i = 0; i < kStateDim; ++i) {
        for (int k = 0; k < kStateDim; ++k) {
            const double fik = F(i, k);
            if (fik == 0.0) continue;
            for (int j = 0; j < kStateDim; ++j) {
                FP(i, j) += fik * P_(k, j);
            }
        }
    }

    // P_new = FP * F^T (upper triangle then mirror)
    Mat17 P_new = Mat17::Zero();
    for (int i = 0; i < kStateDim; ++i) {
        for (int j = i; j < kStateDim; ++j) {
            double s = 0.0;
            for (int k = 0; k < kStateDim; ++k) {
                const double fjk = F(j, k);
                if (fjk != 0.0) s += FP(i, k) * fjk;
            }
            P_new(i, j) = P_new(j, i) = s;
        }
    }

    // Inject rotated 9x9 preintegration noise covariance G * Sigma_delta * G^T
    auto blk9 = [&](int br, int bc) -> math::Mat3d {
        math::Mat3d B;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                B(r, c) = preint.covariance(br * 3 + r, bc * 3 + c);
        return B;
    };
    const math::Mat3d R_iT = R_i.transpose();
    const math::Mat3d Q_rr = blk9(0, 0);
    const math::Mat3d Q_rv = blk9(0, 1) * R_iT;
    const math::Mat3d Q_rp = blk9(0, 2) * R_iT;
    const math::Mat3d Q_vv = R_i * blk9(1, 1) * R_iT;
    const math::Mat3d Q_vp = R_i * blk9(1, 2) * R_iT;
    const math::Mat3d Q_pp = R_i * blk9(2, 2) * R_iT;

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            P_new(r + 0, c + 0) += Q_rr(r, c);
            P_new(r + 0, c + 6) += Q_rv(r, c);
            P_new(c + 6, r + 0) += Q_rv(r, c);
            P_new(r + 0, c + 3) += Q_rp(r, c);
            P_new(c + 3, r + 0) += Q_rp(r, c);
            P_new(r + 6, c + 6) += Q_vv(r, c);
            P_new(r + 6, c + 3) += Q_vp(r, c);
            P_new(c + 3, r + 6) += Q_vp(r, c);
            P_new(r + 3, c + 3) += Q_pp(r, c);
        }
    }

    // Bias random walk & mild S^2 gravity process noise
    const double var_bg = config_.sensor_rig.noise.bias_gyro_rw_sigma * config_.sensor_rig.noise.bias_gyro_rw_sigma * dt;
    const double var_ba = config_.sensor_rig.noise.bias_accel_rw_sigma * config_.sensor_rig.noise.bias_accel_rw_sigma * dt;
    for (int r = 0; r < 3; ++r) {
        P_new(r + 9, r + 9)   += var_bg;
        P_new(r + 12, r + 12) += var_ba;
    }
    P_new(15, 15) += 1e-8 * dt;
    P_new(16, 16) += 1e-8 * dt;

    // Symmetrize
    for (int i = 0; i < kStateDim; ++i) {
        for (int j = i + 1; j < kStateDim; ++j) {
            const double avg = 0.5 * (P_new(i, j) + P_new(j, i));
            P_new(i, j) = P_new(j, i) = avg;
        }
    }
    P_ = P_new;
}

AvSlamEngine::ImagePyramid AvSlamEngine::buildGrayPyramid(const core::RgbFrame& rgb_frame, int max_levels) {
    ImagePyramid pyr;
    const int w0 = rgb_frame.intrinsics.width;
    const int h0 = rgb_frame.intrinsics.height;
    if (w0 < 16 || h0 < 16 || rgb_frame.gray_bytes.size() < static_cast<size_t>(w0 * h0)) {
        return pyr;
    }
    pyr.base_intrinsics = rgb_frame.intrinsics;
    pyr.exposure_sec = rgb_frame.exposure_duration_sec;

    const int n_lvls = std::clamp(max_levels, 1, 3);
    pyr.levels[0].width = w0;
    pyr.levels[0].height = h0;
    pyr.levels[0].scale = 1.0;
    pyr.levels[0].pixels.resize(static_cast<size_t>(w0 * h0));
    for (size_t i = 0; i < static_cast<size_t>(w0 * h0); ++i) {
        pyr.levels[0].pixels[i] = static_cast<float>(rgb_frame.gray_bytes[i]);
    }
    pyr.num_levels = 1;

    for (int l = 1; l < n_lvls; ++l) {
        const auto& prev = pyr.levels[static_cast<size_t>(l - 1)];
        const int wl = prev.width / 2;
        const int hl = prev.height / 2;
        if (wl < 16 || hl < 16) break;
        auto& cur = pyr.levels[static_cast<size_t>(l)];
        cur.width = wl;
        cur.height = hl;
        cur.scale = prev.scale * 0.5;
        cur.pixels.resize(static_cast<size_t>(wl * hl));
        for (int y = 0; y < hl; ++y) {
            const int py = y * 2;
            const size_t row0 = static_cast<size_t>(py * prev.width);
            const size_t row1 = static_cast<size_t>((py + 1) * prev.width);
            for (int x = 0; x < wl; ++x) {
                const int px = x * 2;
                cur.pixels[static_cast<size_t>(y * wl + x)] =
                    0.25f * (prev.pixels[row0 + px] + prev.pixels[row0 + px + 1] +
                             prev.pixels[row1 + px] + prev.pixels[row1 + px + 1]);
            }
        }
        pyr.num_levels = l + 1;
    }
    return pyr;
}

float AvSlamEngine::samplePyramidBilinear(const GrayLevel& lvl, double u, double v) {
    if (lvl.pixels.empty() || u < 0.0 || v < 0.0 ||
        u >= static_cast<double>(lvl.width - 1) || v >= static_cast<double>(lvl.height - 1)) {
        return 0.0f;
    }
    const int x0 = static_cast<int>(std::floor(u));
    const int y0 = static_cast<int>(std::floor(v));
    const double ax = u - x0;
    const double ay = v - y0;
    const size_t row0 = static_cast<size_t>(y0 * lvl.width + x0);
    const size_t row1 = static_cast<size_t>((y0 + 1) * lvl.width + x0);
    return static_cast<float>(
        (1.0 - ax) * (1.0 - ay) * lvl.pixels[row0] +
        ax * (1.0 - ay) * lvl.pixels[row0 + 1] +
        (1.0 - ax) * ay * lvl.pixels[row1] +
        ax * ay * lvl.pixels[row1 + 1]);
}

void AvSlamEngine::solveJointMapOptimization(const std::vector<math::Vec3d>& feats_body,
                                             const PreintegratedImuFactor& preint,
                                             const ImagePyramid* visual_pyramid,
                                             const std::vector<VisualPatch>* visual_patches,
                                             EstimatorMetrics& io_metrics) {
    io_metrics.evaluated_features_count = feats_body.size();
    stats_.planar_constraints_last_frame = 0;
    stats_.linear_constraints_last_frame = 0;
    stats_.anisotropic_degeneracy_active = false;
    stats_.zupt_active_last_frame = preint.is_stationary_zupt;

    const bool has_visual_factors = (visual_pyramid != nullptr && visual_pyramid->num_levels > 0 &&
                                     visual_patches != nullptr && visual_patches->size() >= 4);
    if (surfel_map_.numValidPlanarSurfels() < 5 && !has_visual_factors) {
        return;
    }

    const core::SlamState x_prop = state_;

    // Compute exact prior information matrix Lambda_prior = P^-1 ONCE via Cholesky LL^T
    Mat17 L_P;
    Mat17 Lambda_prior;
    if (!choleskyDecompose(P_, L_P) || !choleskyInverseFromL(L_P, Lambda_prior)) {
        return;
    }

    const math::SE3d T_lidar_body = config_.sensor_rig.T_imu_lidar.inverse();
    const math::SE3d T_cam_body = config_.sensor_rig.camera_extrinsics.T_cam_lidar * T_lidar_body;
    const double vis_inv_var = 1.0 / std::max(16.0, config_.visual_photometric_sigma * config_.visual_photometric_sigma);
    const double vis_huber2 = config_.visual_huber_delta * config_.visual_huber_delta;

    Mat17 L_final;
    bool updated = false;

    for (int iter = 0; iter < config_.max_map_iterations; ++iter) {
        const bool is_coarse_stage = (iter < 2);
        const double corr_thresh = is_coarse_stage ? config_.initial_corr_dist_m : config_.refined_corr_dist_m;
        const double gnc_c = is_coarse_stage ? config_.gnc_coarse_scale_m : config_.gnc_fine_scale_m;
        const double gnc_c2 = gnc_c * gnc_c;

        double HTH6[6][6]{};
        double HTr6[6]{};
        size_t valid_corrs = 0;
        size_t planar_corrs = 0;
        size_t linear_corrs = 0;
        double sum_sq_err = 0.0;

        const math::Mat3d R_wb_T = state_.T_world_body.so3.R.transpose();

        for (const auto& p_body : feats_body) {
            const math::Vec3d p_world = state_.T_world_body.transform(p_body);
            SurfelConstraint sc;
            if (!surfel_map_.queryGeometricConstraint(p_body, p_world, corr_thresh, false, sc)) {
                continue;
            }

            const double d2 = sc.signed_distance * sc.signed_distance;
            const double gnc_w = gnc_c2 / (gnc_c2 + d2);
            const double inv_var = 1.0 / std::max(0.001, sc.measurement_variance + 0.0007);
            const double w = gnc_w * sc.quality_weight * inv_var;

            const math::Vec3d R_n = R_wb_T * sc.residual_normal;
            const math::Vec3d J_rot = p_body.cross(R_n);
            const math::Vec3d& J_pos = sc.residual_normal;

            const double J[6] = {J_rot.x, J_rot.y, J_rot.z, J_pos.x, J_pos.y, J_pos.z};
            const double z = -sc.signed_distance;

            for (int r = 0; r < 6; ++r) {
                const double wJ = w * J[r];
                HTr6[r] += wJ * z;
                for (int c = r; c < 6; ++c) {
                    HTH6[r][c] += wJ * J[c];
                }
            }
            sum_sq_err += d2;
            valid_corrs++;
            if (sc.type == SurfelGeometryType::PLANAR) planar_corrs++;
            else linear_corrs++;
        }

        // Tightly-Coupled Coarse-to-Fine 3-Level Pyramid Direct 8x8 Photometric Visual Factors
        // Fused directly into the exact same 17-DOF normal matrix so texture stabilizes degenerate LiDAR corridors!
        size_t valid_vis_patches = 0;
        double sum_vis_err2 = 0.0;
        size_t vis_samples = 0;
        if (has_visual_factors) {
            // Coarse-to-fine pyramid schedule: iter 0 -> finest available coarse level (L2), iter 1 -> L1, iter 2+ -> L0
            const int target_lvl = std::clamp((iter == 0) ? 2 : (iter == 1 ? 1 : 0), 0, visual_pyramid->num_levels - 1);
            const auto& lvl = visual_pyramid->levels[static_cast<size_t>(target_lvl)];
            const double sc = lvl.scale;
            const math::SE3d T_cam_world = T_cam_body * state_.T_world_body.inverse();
            const double fx_l = visual_pyramid->base_intrinsics.fx * sc;
            const double fy_l = visual_pyramid->base_intrinsics.fy * sc;

            // Sample 5x5 core of each 8x8 visual patch (stride 2 on fine level = 9 taps/patch for ultra-fast mobile execution)
            static constexpr int kTapOffsets[9][2] = {
                {-2, -2}, {0, -2}, {2, -2},
                {-2,  0}, {0,  0}, {2,  0},
                {-2,  2}, {0,  2}, {2,  2}
            };

            for (const auto& patch : *visual_patches) {
                const math::Vec3d p_cam = T_cam_world.transform(patch.center_world);
                if (p_cam.z <= 0.25 || p_cam.z > 25.0) continue;
                const math::Vec2d uv0 = visual_pyramid->base_intrinsics.project(p_cam, true);
                const double u_l = uv0.x * sc;
                const double v_l = uv0.y * sc;
                if (u_l < 4.0 || v_l < 4.0 || u_l >= lvl.width - 5.0 || v_l >= lvl.height - 5.0) continue;

                // Online exposure-ratio compensation
                const double exp_ratio = (patch.ref_exposure_sec > 1e-5 && visual_pyramid->exposure_sec > 1e-5)
                    ? std::clamp(patch.ref_exposure_sec / visual_pyramid->exposure_sec, 0.5, 2.0)
                    : 1.0;

                const double inv_z = 1.0 / p_cam.z;
                const double inv_z2 = inv_z * inv_z;
                const math::Vec3d p_body = state_.T_world_body.inverse().transform(patch.center_world);
                bool patch_ok = false;

                for (int t = 0; t < 9; ++t) {
                    const int ox = kTapOffsets[t][0];
                    const int oy = kTapOffsets[t][1];
                    const double uu = u_l + static_cast<double>(ox) * sc;
                    const double vv = v_l + static_cast<double>(oy) * sc;
                    const float I_curr = static_cast<float>(samplePyramidBilinear(lvl, uu, vv) * exp_ratio);
                    const float I_ref = patch.patch_intensities[static_cast<size_t>((oy + 3) * 8 + (ox + 3))];
                    const double res = static_cast<double>(I_curr - I_ref);
                    if (std::abs(res) > 75.0) continue;

                    const double gx = 0.5 * (samplePyramidBilinear(lvl, uu + 1.0, vv) - samplePyramidBilinear(lvl, uu - 1.0, vv));
                    const double gy = 0.5 * (samplePyramidBilinear(lvl, uu, vv + 1.0) - samplePyramidBilinear(lvl, uu, vv - 1.0));
                    if (gx * gx + gy * gy < 16.0) continue;

                    const math::Vec3d J_pcam(
                        gx * fx_l * inv_z,
                        gy * fy_l * inv_z,
                        -(gx * fx_l * p_cam.x + gy * fy_l * p_cam.y) * inv_z2
                    );
                    const math::Vec3d J_pbody = -(T_cam_body.so3.R.transpose() * J_pcam);
                    const math::Vec3d J_pos = state_.T_world_body.so3.R * J_pbody;
                    const math::Vec3d J_rot = p_body.cross(J_pbody);

                    // State ordering matches [rot(0..2), pos(3..5)]
                    const double J[6] = {J_rot.x, J_rot.y, J_rot.z, J_pos.x, J_pos.y, J_pos.z};
                    const double rob_w = vis_huber2 / (vis_huber2 + res * res);
                    const double w = (rob_w * vis_inv_var) / 9.0;

                    for (int r = 0; r < 6; ++r) {
                        const double wJ = w * J[r];
                        HTr6[r] -= wJ * res;
                        for (int c = r; c < 6; ++c) {
                            HTH6[r][c] += wJ * J[c];
                        }
                    }
                    sum_vis_err2 += res * res;
                    vis_samples++;
                    patch_ok = true;
                }
                if (patch_ok) valid_vis_patches++;
            }
            if (valid_vis_patches >= 4 && vis_samples > 0) {
                io_metrics.visual_patch_correspondences = valid_vis_patches;
                io_metrics.photometric_rmse = std::sqrt(sum_vis_err2 / static_cast<double>(vis_samples));
                io_metrics.visual_update_applied = true;
            }
        }

        // Symmetrize HTH6
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < r; ++c) HTH6[r][c] = HTH6[c][r];
        }

        io_metrics.geometric_correspondences = valid_corrs;
        stats_.planar_constraints_last_frame = planar_corrs;
        stats_.linear_constraints_last_frame = linear_corrs;
        if (valid_corrs < 12 && valid_vis_patches < 4) break;
        if (valid_corrs > 0) {
            io_metrics.geometric_rmse_m = std::sqrt(sum_sq_err / static_cast<double>(valid_corrs));
        }

        // Anisotropic Subspace-Projected Degeneracy Analysis on translational Hessian H_tt
        math::Mat3d H_trans;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                H_trans(r, c) = HTH6[r + 3][c + 3];

        const math::SymEigen3d eig_trans = math::SymEigen3d::compute(H_trans, 20);
        io_metrics.min_observability_eigenvalue = eig_trans.eigenvalues[0];
        io_metrics.is_degenerate = (eig_trans.eigenvalues[0] < config_.degeneracy_eigenvalue_thresh);

        // Compute manifold box-minus difference vec = x_prop [-] state_curr
        const GravityBasis gb_curr = computeGravityTangentBasis(state_.gravity_mps2);
        const math::Vec3d drot_prop = (state_.T_world_body.so3.inverse() * x_prop.T_world_body.so3).log();
        const math::Vec3d dpos_prop = x_prop.T_world_body.t - state_.T_world_body.t;
        const math::Vec3d dvel_prop = x_prop.velocity_mps - state_.velocity_mps;
        const math::Vec3d dbg_prop  = x_prop.bias_gyro_rads - state_.bias_gyro_rads;
        const math::Vec3d dba_prop  = x_prop.bias_accel_mps2 - state_.bias_accel_mps2;
        const math::Vec2d dg_prop   = gravityBoxMinus(x_prop.gravity_mps2, state_.gravity_mps2, gb_curr);

        double vec[kStateDim]{};
        for (int i = 0; i < 3; ++i) {
            vec[i + 0]  = drot_prop[i];
            vec[i + 3]  = dpos_prop[i];
            vec[i + 6]  = dvel_prop[i];
            vec[i + 9]  = dbg_prop[i];
            vec[i + 12] = dba_prop[i];
        }
        vec[15] = dg_prop.x;
        vec[16] = dg_prop.y;

        // Form full 17x17 MAP normal equations: (Lambda_prior + H_geo + H_vis + H_degen + H_zupt) dx = rhs
        Mat17 A = Lambda_prior;
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < 6; ++c) {
                A(r, c) += HTH6[r][c];
            }
        }

        double rhs[kStateDim]{};
        for (int r = 0; r < kStateDim; ++r) {
            double s = (r < 6) ? HTr6[r] : 0.0;
            for (int c = 0; c < kStateDim; ++c) {
                s += Lambda_prior(r, c) * vec[c];
            }
            rhs[r] = s;
        }

        // Anisotropic degeneracy mitigation: stiffen IMU prior ONLY along under-constrained eigenvectors
        if (io_metrics.is_degenerate) {
            stats_.anisotropic_degeneracy_active = true;
            for (int axis = 0; axis < 3; ++axis) {
                if (eig_trans.eigenvalues[axis] < config_.degeneracy_eigenvalue_thresh) {
                    const double deficit = config_.degeneracy_eigenvalue_thresh - std::max(0.0, eig_trans.eigenvalues[axis]);
                    const double stiff = config_.degeneracy_prior_stiffness * (deficit / config_.degeneracy_eigenvalue_thresh);
                    const math::Vec3d vk(eig_trans.eigenvectors(0, axis),
                                         eig_trans.eigenvectors(1, axis),
                                         eig_trans.eigenvectors(2, axis));
                    const double proj_err = vk.dot(dpos_prop);
                    for (int r = 0; r < 3; ++r) {
                        rhs[r + 3] += stiff * vk[r] * proj_err;
                        for (int c = 0; c < 3; ++c) {
                            A(r + 3, c + 3) += stiff * vk[r] * vk[c];
                        }
                    }
                }
            }
        }

        // Soft ZUPT velocity factor when stationary
        if (preint.is_stationary_zupt) {
            const double w_zupt = 60.0 * preint.zupt_confidence;
            for (int i = 0; i < 3; ++i) {
                A(i + 6, i + 6) += w_zupt;
                rhs[i + 6] += w_zupt * (0.0 - state_.velocity_mps[i]);
            }
        }

        for (int i = 0; i < kStateDim; ++i) A(i, i) += 1e-8;

        Mat17 L_A;
        if (!choleskyDecompose(A, L_A)) break;

        double dx[kStateDim]{};
        choleskySolve(L_A, rhs, dx);

        math::Vec3d dtheta(dx[0], dx[1], dx[2]);
        math::Vec3d dt(dx[3], dx[4], dx[5]);
        if (dtheta.norm() > 0.35) dtheta *= (0.35 / dtheta.norm());
        if (dt.norm() > 0.50) dt *= (0.50 / dt.norm());

        state_.T_world_body.so3 = state_.T_world_body.so3 * math::SO3d::exp(dtheta);
        state_.T_world_body.t  += dt;
        state_.velocity_mps    += math::Vec3d(dx[6], dx[7], dx[8]);
        state_.bias_gyro_rads  += math::Vec3d(dx[9], dx[10], dx[11]);
        state_.bias_accel_mps2 += math::Vec3d(dx[12], dx[13], dx[14]);
        state_.gravity_mps2     = gravityBoxPlus(state_.gravity_mps2, gb_curr, dx[15], dx[16]);

        for (int i = 0; i < 3; ++i) {
            state_.bias_gyro_rads[i]  = std::clamp(state_.bias_gyro_rads[i], -0.25, 0.25);
            state_.bias_accel_mps2[i] = std::clamp(state_.bias_accel_mps2[i], -1.5, 1.5);
        }

        L_final = L_A;
        updated = true;

        if (!is_coarse_stage && dtheta.norm() * math::kRadToDeg < 0.008 && dt.norm() < 0.0004) {
            break;
        }
    }

    if (updated) {
        Mat17 P_post;
        if (choleskyInverseFromL(L_final, P_post)) {
            P_ = P_post;
        }
    } else {
        state_.velocity_mps *= 0.5;
    }
}

float AvSlamEngine::sampleBilinearGray(const core::RgbFrame& img, double u, double v) {
    if (img.gray_bytes.empty() || u < 0.0 || v < 0.0 ||
        u >= static_cast<double>(img.intrinsics.width - 1) ||
        v >= static_cast<double>(img.intrinsics.height - 1)) {
        return 0.0f;
    }
    const int x0 = static_cast<int>(std::floor(u));
    const int y0 = static_cast<int>(std::floor(v));
    const double ax = u - x0;
    const double ay = v - y0;
    const int w = img.intrinsics.width;

    const float i00 = static_cast<float>(img.gray_bytes[static_cast<size_t>(y0 * w + x0)]);
    const float i10 = static_cast<float>(img.gray_bytes[static_cast<size_t>(y0 * w + x0 + 1)]);
    const float i01 = static_cast<float>(img.gray_bytes[static_cast<size_t>((y0 + 1) * w + x0)]);
    const float i11 = static_cast<float>(img.gray_bytes[static_cast<size_t>((y0 + 1) * w + x0 + 1)]);

    return static_cast<float>((1.0 - ax) * (1.0 - ay) * i00 +
                              ax * (1.0 - ay) * i10 +
                              (1.0 - ax) * ay * i01 +
                              ax * ay * i11);
}

void AvSlamEngine::sampleBilinearRgb(const core::RgbFrame& img, double u, double v,
                                     uint8_t& r, uint8_t& g, uint8_t& b) {
    const int w = img.intrinsics.width;
    const int h = img.intrinsics.height;
    if (w < 2 || h < 2 || img.rgb_bytes.size() < static_cast<size_t>(w * h * 3)) {
        const uint8_t gray = static_cast<uint8_t>(std::clamp(sampleBilinearGray(img, u, v), 0.0f, 255.0f));
        r = gray; g = gray; b = gray;
        return;
    }
    const double uc = std::clamp(u, 0.0, static_cast<double>(w - 1) - 1e-4);
    const double vc = std::clamp(v, 0.0, static_cast<double>(h - 1) - 1e-4);
    const int x0 = static_cast<int>(std::floor(uc));
    const int y0 = static_cast<int>(std::floor(vc));
    const int x1 = std::min(x0 + 1, w - 1);
    const int y1 = std::min(y0 + 1, h - 1);
    const double ax = uc - x0;
    const double ay = vc - y0;
    const double w00 = (1.0 - ax) * (1.0 - ay);
    const double w10 = ax * (1.0 - ay);
    const double w01 = (1.0 - ax) * ay;
    const double w11 = ax * ay;

    const size_t i00 = static_cast<size_t>((y0 * w + x0) * 3);
    const size_t i10 = static_cast<size_t>((y0 * w + x1) * 3);
    const size_t i01 = static_cast<size_t>((y1 * w + x0) * 3);
    const size_t i11 = static_cast<size_t>((y1 * w + x1) * 3);

    auto interp = [&](size_t c) -> uint8_t {
        const double val = w00 * img.rgb_bytes[i00 + c] + w10 * img.rgb_bytes[i10 + c] +
                           w01 * img.rgb_bytes[i01 + c] + w11 * img.rgb_bytes[i11 + c];
        return static_cast<uint8_t>(std::clamp(std::round(val), 0.0, 255.0));
    };
    r = interp(0);
    g = interp(1);
    b = interp(2);
}

math::SE3d AvSlamEngine::interpolateBodyPoseAtTime(double target_time_sec,
                                                   const std::vector<MotionKnot>& sweep_knots) const {
    if (sweep_knots.size() >= 2 &&
        target_time_sec >= sweep_knots.front().timestamp_sec &&
        target_time_sec <= sweep_knots.back().timestamp_sec) {
        auto it = std::lower_bound(
            sweep_knots.begin(), sweep_knots.end(), target_time_sec,
            [](const MotionKnot& k, double t) { return k.timestamp_sec < t; });
        if (it == sweep_knots.begin()) return it->T_world_body;
        if (it == sweep_knots.end()) return sweep_knots.back().T_world_body;
        const auto& prev = *(it - 1);
        const double sdt = it->timestamp_sec - prev.timestamp_sec;
        const double alpha = (sdt > 1e-7) ? ((target_time_sec - prev.timestamp_sec) / sdt) : 1.0;
        return math::SE3d::interpolate(prev.T_world_body, it->T_world_body, alpha);
    }

    if (recent_pose_history_.size() >= 2) {
        if (target_time_sec <= recent_pose_history_.front().first) return recent_pose_history_.front().second;
        if (target_time_sec >= recent_pose_history_.back().first) return recent_pose_history_.back().second;
        auto it = std::lower_bound(
            recent_pose_history_.begin(), recent_pose_history_.end(), target_time_sec,
            [](const std::pair<double, math::SE3d>& p, double t) { return p.first < t; });
        if (it == recent_pose_history_.begin()) return it->second;
        if (it == recent_pose_history_.end()) return recent_pose_history_.back().second;
        const auto& prev = *(it - 1);
        const double sdt = it->first - prev.first;
        const double alpha = (sdt > 1e-7) ? ((target_time_sec - prev.first) / sdt) : 1.0;
        return math::SE3d::interpolate(prev.second, it->second, alpha);
    }
    return state_.T_world_body;
}

void AvSlamEngine::colorizeAndExtractVisualPatches(const core::RgbFrame& rgb_frame,
                                                   const std::vector<MotionKnot>& corrected_knots,
                                                   core::LidarFrame& io_deskewed_body,
                                                   core::LidarFrame& io_world_frame) {
    const size_t n_pts = std::min(io_deskewed_body.points.size(), io_world_frame.points.size());
    auto applyIntensityFallback = [&](size_t idx) {
        auto& pb = io_deskewed_body.points[idx];
        auto& pw = io_world_frame.points[idx];
        if (!pb.colorized) {
            const uint8_t shade = static_cast<uint8_t>(std::clamp(55.0f + pb.intensity * 0.55f, 45.0f, 195.0f));
            pb.r = shade; pb.g = shade; pb.b = shade;
            pw.r = shade; pw.g = shade; pw.b = shade;
        }
    };

    const auto& cam_ext = config_.sensor_rig.camera_extrinsics;
    const int img_w = rgb_frame.intrinsics.width;
    const int img_h = rgb_frame.intrinsics.height;
    if (!config_.enable_rgb_colorization || !cam_ext.verified ||
        img_w < 16 || img_h < 16 ||
        (rgb_frame.gray_bytes.empty() && rgb_frame.rgb_bytes.empty())) {
        for (size_t i = 0; i < n_pts; ++i) applyIntensityFallback(i);
        return;
    }

    const math::SE3d T_world_body_rgb = interpolateBodyPoseAtTime(rgb_frame.timestamp_sec, corrected_knots);
    const math::SE3d T_lidar_body = config_.sensor_rig.T_imu_lidar.inverse();
    const math::SE3d T_cam_body = cam_ext.T_cam_lidar * T_lidar_body;
    const math::SE3d T_cam_world_rgb = T_cam_body * T_world_body_rgb.inverse();

    const int zb_w = (img_w + 1) / 2;
    const int zb_h = (img_h + 1) / 2;
    std::vector<float> z_buffer(static_cast<size_t>(zb_w * zb_h), std::numeric_limits<float>::infinity());

    struct ProjInfo {
        float u{-1.0f};
        float v{-1.0f};
        float z{-1.0f};
        bool in_fov{false};
    };
    std::vector<ProjInfo> proj(n_pts);

    for (size_t i = 0; i < n_pts; ++i) {
        const auto& pt_world = io_world_frame.points[i];
        const math::Vec3d p_cam = T_cam_world_rgb.transform(math::Vec3d(pt_world.x, pt_world.y, pt_world.z));
        if (p_cam.z <= 0.20 || p_cam.z > 35.0) continue;
        if (!rgb_frame.intrinsics.isInFieldOfView(p_cam, 6.0)) continue;

        const math::Vec2d uv = rgb_frame.intrinsics.project(p_cam, true);
        proj[i].u = static_cast<float>(uv.x);
        proj[i].v = static_cast<float>(uv.y);
        proj[i].z = static_cast<float>(p_cam.z);
        proj[i].in_fov = true;

        const int zx = std::clamp(static_cast<int>(uv.x) / 2, 0, zb_w - 1);
        const int zy = std::clamp(static_cast<int>(uv.y) / 2, 0, zb_h - 1);
        float& cell = z_buffer[static_cast<size_t>(zy * zb_w + zx)];
        if (proj[i].z < cell) cell = proj[i].z;
    }

    constexpr float kOcclusionTolMeters = 0.08f;
    size_t patches_added = 0;

    for (size_t i = 0; i < n_pts; ++i) {
        if (!proj[i].in_fov) {
            applyIntensityFallback(i);
            continue;
        }
        const int zx = std::clamp(static_cast<int>(proj[i].u) / 2, 0, zb_w - 1);
        const int zy = std::clamp(static_cast<int>(proj[i].v) / 2, 0, zb_h - 1);
        if (proj[i].z > z_buffer[static_cast<size_t>(zy * zb_w + zx)] + kOcclusionTolMeters) {
            applyIntensityFallback(i);
            continue;
        }

        uint8_t r = 200, g = 200, b = 200;
        sampleBilinearRgb(rgb_frame, proj[i].u, proj[i].v, r, g, b);
        auto& pt_body = io_deskewed_body.points[i];
        auto& pt_world = io_world_frame.points[i];
        pt_body.r = r; pt_body.g = g; pt_body.b = b; pt_body.colorized = true;
        pt_world.r = r; pt_world.g = g; pt_world.b = b; pt_world.colorized = true;

        if (config_.enable_visual_fusion && !rgb_frame.gray_bytes.empty() && patches_added < 24 && (i % 8 == 0)) {
            const double gx = sampleBilinearGray(rgb_frame, proj[i].u + 1.0, proj[i].v) -
                              sampleBilinearGray(rgb_frame, proj[i].u - 1.0, proj[i].v);
            const double gy = sampleBilinearGray(rgb_frame, proj[i].u, proj[i].v + 1.0) -
                              sampleBilinearGray(rgb_frame, proj[i].u, proj[i].v - 1.0);
            if (gx * gx + gy * gy > 100.0) {
                VisualPatch patch;
                patch.center_world = math::Vec3d(pt_world.x, pt_world.y, pt_world.z);
                patch.ref_T_cam_world = T_cam_world_rgb;
                patch.ref_intrinsics = rgb_frame.intrinsics;
                patch.ref_exposure_sec = rgb_frame.exposure_duration_sec;
                patch.last_updated_frame = rgb_frame.frame_id;
                patch.valid = true;
                for (int py = 0; py < 8; ++py) {
                    for (int px = 0; px < 8; ++px) {
                        patch.patch_intensities[static_cast<size_t>(py * 8 + px)] =
                            sampleBilinearGray(rgb_frame, proj[i].u + (px - 3), proj[i].v + (py - 3));
                    }
                }
                surfel_map_.updateVisualPatch(patch.center_world, patch);
                patches_added++;
            }
        }
    }
}

bool AvSlamEngine::processBundle(const sync::SyncBundle& bundle,
                                 core::LidarFrame& out_deskewed_body_frame,
                                 core::LidarFrame& out_world_frame,
                                 EstimatorMetrics& out_metrics) {
    const int splits = std::clamp(config_.sub_sweep_splits, 1, 4);
    const double sweep_dur = bundle.lidar_frame.timestamp_end_sec - bundle.lidar_frame.timestamp_start_sec;
    if (splits <= 1 || sweep_dur < 0.025 || bundle.lidar_frame.points.size() < 400 ||
        state_.status == core::TrackingStatus::UNINITIALIZED ||
        state_.status == core::TrackingStatus::INITIALIZING_IMU) {
        return processSingleSweep(bundle, out_deskewed_body_frame, out_world_frame, out_metrics);
    }

    // Point-LIO-inspired chronological Sub-Sweep Micro-Update mode (20-40 Hz) for extreme handheld dynamics
    const double t0 = bundle.lidar_frame.timestamp_start_sec;
    const double sub_dt = sweep_dur / static_cast<double>(splits);

    out_deskewed_body_frame = core::LidarFrame{};
    out_deskewed_body_frame.frame_id = bundle.lidar_frame.frame_id;
    out_deskewed_body_frame.timestamp_start_sec = t0;
    out_deskewed_body_frame.timestamp_end_sec = bundle.lidar_frame.timestamp_end_sec;
    out_world_frame = out_deskewed_body_frame;

    EstimatorMetrics agg_metrics{};
    bool any_ok = false;
    std::vector<core::LidarPointXYZIRT> all_world;
    all_world.reserve(bundle.lidar_frame.points.size());

    for (int s = 0; s < splits; ++s) {
        const double sbeg = t0 + static_cast<double>(s) * sub_dt;
        const double send = (s + 1 == splits) ? bundle.lidar_frame.timestamp_end_sec : (t0 + static_cast<double>(s + 1) * sub_dt);

        sync::SyncBundle sub_b;
        sub_b.lidar_frame.frame_id = bundle.lidar_frame.frame_id * static_cast<uint64_t>(splits) + static_cast<uint64_t>(s);
        sub_b.lidar_frame.timestamp_start_sec = sbeg;
        sub_b.lidar_frame.timestamp_end_sec = send;
        sub_b.estimated_cam_to_lidar_offset_sec = bundle.estimated_cam_to_lidar_offset_sec;
        for (const auto& pt : bundle.lidar_frame.points) {
            if ((s == 0 && pt.timestamp_sec <= send) ||
                (s + 1 == splits && pt.timestamp_sec > sbeg) ||
                (pt.timestamp_sec > sbeg && pt.timestamp_sec <= send)) {
                sub_b.lidar_frame.points.push_back(pt);
            }
        }
        if (sub_b.lidar_frame.points.empty()) continue;

        for (const auto& imu : bundle.imu_measurements) {
            if (imu.timestamp_sec >= sbeg - 0.01 && imu.timestamp_sec <= send + 0.005) {
                sub_b.imu_measurements.push_back(imu);
            }
        }
        sub_b.has_valid_imu_coverage = (sub_b.imu_measurements.size() >= 2);
        if (s + 1 == splits) sub_b.rgb_frame = bundle.rgb_frame;

        core::LidarFrame sub_body, sub_world;
        EstimatorMetrics sub_m;
        if (processSingleSweep(sub_b, sub_body, sub_world, sub_m)) {
            any_ok = true;
            agg_metrics.processing_time_ms += sub_m.processing_time_ms;
            agg_metrics.geometric_correspondences = sub_m.geometric_correspondences;
            agg_metrics.evaluated_features_count = sub_m.evaluated_features_count;
            agg_metrics.geometric_rmse_m = sub_m.geometric_rmse_m;
            agg_metrics.min_observability_eigenvalue = sub_m.min_observability_eigenvalue;
            agg_metrics.is_degenerate = agg_metrics.is_degenerate || sub_m.is_degenerate;
            agg_metrics.visual_update_applied = agg_metrics.visual_update_applied || sub_m.visual_update_applied;
            agg_metrics.keyframe_emitted = agg_metrics.keyframe_emitted || sub_m.keyframe_emitted;
            all_world.insert(all_world.end(), sub_world.points.begin(), sub_world.points.end());
        }
    }

    if (!any_ok) return false;
    // Express merged world cloud consistently in final sweep-end body frame
    const math::SE3d T_body_world_end = state_.T_world_body.inverse();
    out_world_frame.points = all_world;
    out_deskewed_body_frame.points = all_world;
    for (auto& pt : out_deskewed_body_frame.points) {
        const math::Vec3d pb = T_body_world_end.transform(math::Vec3d(pt.x, pt.y, pt.z));
        pt.x = pb.x; pt.y = pb.y; pt.z = pb.z;
    }
    agg_metrics.deskewed_points_count = out_deskewed_body_frame.points.size();
    agg_metrics.frames_processed = trajectory_.size();
    out_metrics = agg_metrics;
    return true;
}

bool AvSlamEngine::processSingleSweep(const sync::SyncBundle& bundle,
                                      core::LidarFrame& out_deskewed_body_frame,
                                      core::LidarFrame& out_world_frame,
                                      EstimatorMetrics& out_metrics) {
    const auto t_frame_start = std::chrono::steady_clock::now();
    out_metrics = EstimatorMetrics{};

    if (bundle.lidar_frame.points.empty()) {
        return false;
    }

    if (state_.status == core::TrackingStatus::UNINITIALIZED ||
        state_.status == core::TrackingStatus::INITIALIZING_IMU) {
        if (!initializeImuGravity(bundle.imu_measurements)) {
            if (!bundle.imu_measurements.empty()) return false;
            state_.timestamp_sec = bundle.lidar_frame.timestamp_end_sec;
            state_.status = core::TrackingStatus::TRACKING_LIO;
        }
    }

    // 1. On-Manifold IMU Preintegration + Single Sweep-Level Covariance Propagation + Twist Deskewing
    const auto t_pre_0 = std::chrono::steady_clock::now();
    const core::SlamState state_prev = state_;
    std::vector<MotionKnot> motion_knots;
    core::SlamState predicted_state;
    const PreintegratedImuFactor preint = preintegrator_.preintegrateInterval(
        state_prev,
        bundle.imu_measurements,
        has_last_imu_ ? &last_imu_ : nullptr,
        bundle.lidar_frame.timestamp_end_sec,
        predicted_state,
        motion_knots);

    if (!bundle.imu_measurements.empty()) {
        last_imu_ = bundle.imu_measurements.back();
        has_last_imu_ = true;
    }

    state_ = predicted_state;
    propagatePriorCovariance(state_prev, preint);
    const math::SE3d T_wb_prop_end = state_.T_world_body;

    core::LidarFrame raw_deskewed_body;
    AvImuPreintegrator::deskewScanToSweepEnd(
        bundle.lidar_frame,
        config_.sensor_rig.T_imu_lidar,
        state_.T_world_body,
        motion_knots,
        raw_deskewed_body);
    const auto t_pre_1 = std::chrono::steady_clock::now();

    // 2. Proprietary Real-Time Point Cloud Conditioning Pipeline (Range/Mask -> Flying-Pixel -> Adaptive Centroid Grid)
    const auto t_filt_0 = std::chrono::steady_clock::now();
    out_deskewed_body_frame = raw_deskewed_body;
    std::vector<math::Vec3d> feats_body;
    filters::FilterPipelineStats fstats;
    filters::PointCloudFilters::conditionFrontEndScan(
        raw_deskewed_body.points,
        config_.front_end_filters,
        out_deskewed_body_frame.points,
        feats_body,
        &fstats);
    stats_.total_flying_pixels_removed += fstats.rejected_flying_pixels;
    out_metrics.deskewed_points_count = out_deskewed_body_frame.points.size();
    const auto t_filt_1 = std::chrono::steady_clock::now();

    // 3. 17-DOF Tightly-Coupled Factor-Manifold Cholesky MAP Update
    //    (GNC Cauchy + Anisotropic Degeneracy Mitigation + optional 3-Level Pyramid Direct Visual Factor)
    const auto t_solve_0 = std::chrono::steady_clock::now();
    ImagePyramid vis_pyr;
    std::vector<VisualPatch> vis_patches;
    const ImagePyramid* pyr_ptr = nullptr;
    const std::vector<VisualPatch>* patches_ptr = nullptr;
    const auto& cam_ext = config_.sensor_rig.camera_extrinsics;
    if (config_.enable_visual_fusion && bundle.rgb_frame.has_value() &&
        cam_ext.verified && cam_ext.confidence_score >= 0.65 &&
        !bundle.rgb_frame->gray_bytes.empty()) {
        vis_patches = surfel_map_.getActiveVisualPatches(state_.T_world_body.t, 12.0);
        if (vis_patches.size() >= 4) {
            vis_pyr = buildGrayPyramid(bundle.rgb_frame.value(), config_.visual_pyramid_levels);
            if (vis_pyr.num_levels > 0) {
                pyr_ptr = &vis_pyr;
                patches_ptr = &vis_patches;
            }
        }
    }
    solveJointMapOptimization(feats_body, preint, pyr_ptr, patches_ptr, out_metrics);
    const auto t_solve_1 = std::chrono::steady_clock::now();

    // Rigidly align sweep motion knots to optimized sweep-end pose for sub-ms SLERP RGB colorization
    const math::SE3d T_corr = state_.T_world_body * T_wb_prop_end.inverse();
    for (auto& knot : motion_knots) {
        knot.T_world_body = T_corr * knot.T_world_body;
    }
    recent_pose_history_.emplace_back(state_.timestamp_sec, state_.T_world_body);
    while (recent_pose_history_.size() > 50) {
        recent_pose_history_.pop_front();
    }

    // 5. Transform clean deskewed BODY points into WORLD frame
    out_world_frame = out_deskewed_body_frame;
    for (auto& pt : out_world_frame.points) {
        const math::Vec3d pw = state_.T_world_body.transform(math::Vec3d(pt.x, pt.y, pt.z));
        pt.x = pw.x;
        pt.y = pw.y;
        pt.z = pw.z;
    }

    // 6. Depth-buffered SLERP bilinear RGB colorization
    if (bundle.rgb_frame.has_value()) {
        colorizeAndExtractVisualPatches(bundle.rgb_frame.value(), motion_knots, out_deskewed_body_frame, out_world_frame);
    } else {
        for (size_t i = 0; i < out_deskewed_body_frame.points.size(); ++i) {
            auto& pb = out_deskewed_body_frame.points[i];
            auto& pw = out_world_frame.points[i];
            if (!pb.colorized) {
                const uint8_t shade = static_cast<uint8_t>(std::clamp(55.0f + pb.intensity * 0.55f, 45.0f, 195.0f));
                pb.r = shade; pb.g = shade; pb.b = shade;
                pw.r = shade; pw.g = shade; pw.b = shade;
            }
        }
    }

    // 7. Update Dual-Resolution Sufficient-Statistics Surfel Map & Carve Live Free Space
    const auto t_map_0 = std::chrono::steady_clock::now();
    std::vector<math::Vec3d> world_feats;
    world_feats.reserve(feats_body.size());
    for (const auto& pb : feats_body) {
        world_feats.push_back(state_.T_world_body.transform(pb));
    }
    surfel_map_.carveFreeSpaceRays(state_.T_world_body.t, world_feats, 5);
    surfel_map_.insertWorldPoints(world_feats, bundle.lidar_frame.frame_id, 1.0, false);
    if ((bundle.lidar_frame.frame_id % 5) == 0) {
        surfel_map_.pruneHorizon(state_.T_world_body.t, config_.map_horizon_radius_m, bundle.lidar_frame.frame_id);
    }
    const auto t_map_1 = std::chrono::steady_clock::now();

    // Optional real-time removal of points lying inside dynamic/transient surfels
    if (config_.filter_dynamic_points_on_output) {
        std::vector<core::LidarPointXYZIRT> static_body;
        std::vector<core::LidarPointXYZIRT> static_world;
        static_body.reserve(out_deskewed_body_frame.points.size());
        static_world.reserve(out_world_frame.points.size());
        for (size_t i = 0; i < out_world_frame.points.size(); ++i) {
            const auto& pw = out_world_frame.points[i];
            if (!surfel_map_.isPointDynamic(math::Vec3d(pw.x, pw.y, pw.z))) {
                static_body.push_back(out_deskewed_body_frame.points[i]);
                static_world.push_back(pw);
            }
        }
        out_deskewed_body_frame.points.swap(static_body);
        out_world_frame.points.swap(static_world);
    }

    // 8. Keyframe emission criteria
    if (!has_first_keyframe_) {
        last_keyframe_pose_ = state_.T_world_body;
        has_first_keyframe_ = true;
        out_metrics.keyframe_emitted = true;
    } else {
        const math::SE3d dT = last_keyframe_pose_.inverse() * state_.T_world_body;
        const double trans_dist = dT.t.norm();
        const double rot_deg = dT.so3.log().norm() * math::kRadToDeg;
        if (trans_dist >= config_.keyframe_translation_thresh_m ||
            rot_deg >= config_.keyframe_rotation_thresh_deg) {
            last_keyframe_pose_ = state_.T_world_body;
            out_metrics.keyframe_emitted = true;
        }
    }

    state_.position_uncertainty_m = std::sqrt(std::max(0.0, P_(3, 3) + P_(4, 4) + P_(5, 5)));
    state_.orientation_uncertainty_deg = std::sqrt(std::max(0.0, P_(0, 0) + P_(1, 1) + P_(2, 2))) * math::kRadToDeg;
    if (out_metrics.is_degenerate) {
        state_.status = core::TrackingStatus::DEGENERATE_GEOMETRY;
    } else if (out_metrics.visual_update_applied) {
        state_.status = core::TrackingStatus::TRACKING_LIVO;
    } else {
        state_.status = core::TrackingStatus::TRACKING_LIO;
    }

    trajectory_.push_back(state_);
    out_metrics.frames_processed = trajectory_.size();

    const auto t_frame_end = std::chrono::steady_clock::now();
    const double frame_ms = std::chrono::duration<double, std::milli>(t_frame_end - t_frame_start).count();
    const double pre_ms   = std::chrono::duration<double, std::milli>(t_pre_1 - t_pre_0).count();
    const double filt_ms  = std::chrono::duration<double, std::milli>(t_filt_1 - t_filt_0).count();
    const double solve_ms = std::chrono::duration<double, std::milli>(t_solve_1 - t_solve_0).count();
    const double map_ms   = std::chrono::duration<double, std::milli>(t_map_1 - t_map_0).count();
    out_metrics.processing_time_ms = frame_ms;

    // Running averages for live profiler telemetry
    const double n_prev = static_cast<double>(stats_.total_frames);
    stats_.total_frames++;
    const double inv_n = 1.0 / static_cast<double>(stats_.total_frames);
    stats_.avg_frame_ms        = (stats_.avg_frame_ms * n_prev + frame_ms) * inv_n;
    stats_.avg_preint_deskew_ms= (stats_.avg_preint_deskew_ms * n_prev + pre_ms) * inv_n;
    stats_.avg_filter_ms       = (stats_.avg_filter_ms * n_prev + filt_ms) * inv_n;
    stats_.avg_map_solve_ms    = (stats_.avg_map_solve_ms * n_prev + solve_ms) * inv_n;
    stats_.avg_surfel_update_ms= (stats_.avg_surfel_update_ms * n_prev + map_ms) * inv_n;
    stats_.fine_surfel_cells   = surfel_map_.numFineCells();
    stats_.coarse_surfel_cells = surfel_map_.numCoarseCells();
    stats_.flagged_dynamic_cells = surfel_map_.numFlaggedDynamicCells();

    return true;
}

}  // namespace slam
}  // namespace avscan
