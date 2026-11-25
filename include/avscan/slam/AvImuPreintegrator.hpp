#pragma once

#include "avscan/core/Types.hpp"
#include "avscan/sensors/SensorRigProfile.hpp"
#include <array>
#include <vector>

namespace avscan {
namespace slam {

// Lightweight 9x9 symmetric matrix for on-manifold IMU preintegration covariance [dphi, dv, dp]
struct Mat9d {
    double m[9][9]{};
    static Mat9d Zero() { return Mat9d{}; }
    double& operator()(int r, int c) { return m[r][c]; }
    double operator()(int r, int c) const { return m[r][c]; }
};

// Micro-pose knot recorded during IMU preintegration for continuous-time twist deskewing
struct MotionKnot {
    double timestamp_sec{0.0};
    math::SE3d T_world_body{math::SE3d::Identity()};
    math::Vec3d velocity_world{0.0, 0.0, 0.0};
    math::Vec3d omega_body{0.0, 0.0, 0.0};
    math::Vec3d accel_world{0.0, 0.0, 0.0};
};

// Compact preintegrated inertial factor between two consecutive LiDAR sweeps (i -> j).
// Stores relative SO(3) x R^6 increments, exact first-order bias Jacobians, and 9x9 noise covariance.
struct PreintegratedImuFactor {
    double dt_total{0.0};
    size_t sample_count{0};

    // Preintegrated increments in start-frame body coordinates
    math::SO3d delta_R{};
    math::Vec3d delta_v{0.0, 0.0, 0.0};
    math::Vec3d delta_p{0.0, 0.0, 0.0};

    // Linearization biases used during preintegration
    math::Vec3d linearized_bg{0.0, 0.0, 0.0};
    math::Vec3d linearized_ba{0.0, 0.0, 0.0};

    // Exact analytical first-order bias Jacobians for zero-cost re-linearization
    math::Mat3d J_R_bg{math::Mat3d::Zero()};
    math::Mat3d J_v_bg{math::Mat3d::Zero()};
    math::Mat3d J_v_ba{math::Mat3d::Zero()};
    math::Mat3d J_p_bg{math::Mat3d::Zero()};
    math::Mat3d J_p_ba{math::Mat3d::Zero()};

    // 9x9 preintegrated measurement covariance ordered as [dphi(0..2), dv(3..5), dp(6..8)]
    Mat9d covariance{Mat9d::Zero()};

    // Zero-Velocity Update (ZUPT) stillness detection
    bool is_stationary_zupt{false};
    double zupt_confidence{0.0};

    // Corrected increments under a bias delta (dbt = b_new - linearized_b)
    math::SO3d correctedDeltaR(const math::Vec3d& dbg) const {
        return delta_R * math::SO3d::exp(J_R_bg * dbg);
    }
    math::Vec3d correctedDeltaV(const math::Vec3d& dbg, const math::Vec3d& dba) const {
        return delta_v + J_v_bg * dbg + J_v_ba * dba;
    }
    math::Vec3d correctedDeltaP(const math::Vec3d& dbg, const math::Vec3d& dba) const {
        return delta_p + J_p_bg * dbg + J_p_ba * dba;
    }
};

// On-Manifold Lie-Group IMU Preintegrator & Continuous-Time Twist Deskewer for AV-SLAM.
// Replaces expensive per-sample 18x18 dense Kalman covariance multiplications with O(1)
// block-sparse 3x3 preintegration and exact bias Jacobians.
class AvImuPreintegrator {
public:
    static constexpr double kGravityMag = 9.80665;

    AvImuPreintegrator() = default;

    void setNoiseModel(const sensors::SensorNoiseModel& noise) { noise_ = noise; }
    void setAccelScale(double scale) { acc_scale_ = scale; }
    double accelScale() const { return acc_scale_; }

    // Preintegrates all IMU samples across [t_start, t_end] into a single compact factor and
    // records continuous-time world motion knots starting from state_start.
    PreintegratedImuFactor preintegrateInterval(
        const core::SlamState& state_start,
        const std::vector<core::ImuMeasurement>& imu_samples,
        const core::ImuMeasurement* prev_tail_imu,
        double t_end_sec,
        core::SlamState& out_predicted_state,
        std::vector<MotionKnot>& out_motion_knots) const;

    // Piecewise-linear Lie-group twist motion compensation into sweep-end IMU BODY frame.
    // Precomputes per-segment SO(3)/R^3 deltas once per IMU interval for ultra-fast O(1)/point execution.
    static void deskewScanToSweepEnd(
        const core::LidarFrame& raw_lidar,
        const math::SE3d& T_imu_lidar,
        const math::SE3d& T_world_body_end,
        const std::vector<MotionKnot>& motion_knots,
        core::LidarFrame& out_deskewed_body);

private:
    sensors::SensorNoiseModel noise_{};
    double acc_scale_{1.0};
};

}  // namespace slam
}  // namespace avscan
