#include "avscan/slam/AvImuPreintegrator.hpp"
#include <algorithm>
#include <cmath>

namespace avscan {
namespace slam {

namespace {

inline math::Mat3d getBlock3(const Mat9d& M, int br, int bc) {
    math::Mat3d B;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            B(r, c) = M(br * 3 + r, bc * 3 + c);
    return B;
}

inline void setBlock3(Mat9d& M, int br, int bc, const math::Mat3d& B) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            M(br * 3 + r, bc * 3 + c) = B(r, c);
}

inline void addBlock3(Mat9d& M, int br, int bc, const math::Mat3d& B) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            M(br * 3 + r, bc * 3 + c) += B(r, c);
}

}  // namespace

PreintegratedImuFactor AvImuPreintegrator::preintegrateInterval(
    const core::SlamState& state_start,
    const std::vector<core::ImuMeasurement>& imu_samples,
    const core::ImuMeasurement* prev_tail_imu,
    double t_end_sec,
    core::SlamState& out_predicted_state,
    std::vector<MotionKnot>& out_motion_knots) const {
    PreintegratedImuFactor fac;
    fac.linearized_bg = state_start.bias_gyro_rads;
    fac.linearized_ba = state_start.bias_accel_mps2;
    out_motion_knots.clear();

    // Assemble strictly increasing IMU sequence bridged seamlessly from previous sweep tail
    std::vector<core::ImuMeasurement> seq;
    seq.reserve(imu_samples.size() + 1);
    if (prev_tail_imu != nullptr) {
        seq.push_back(*prev_tail_imu);
    }
    for (const auto& m : imu_samples) {
        if (seq.empty() || m.timestamp_sec > seq.back().timestamp_sec) {
            seq.push_back(m);
        }
    }

    const double prop_beg = (state_start.timestamp_sec > 0.0)
        ? state_start.timestamp_sec
        : (seq.empty() ? t_end_sec : seq.front().timestamp_sec);
    const double prop_end = std::max(prop_beg, t_end_sec);

    // Conservative dual-threshold Zero-Velocity Update (ZUPT) stillness check
    if (imu_samples.size() >= 6) {
        double max_gyro = 0.0;
        double max_acc_dev = 0.0;
        for (const auto& imu : imu_samples) {
            const math::Vec3d w = imu.gyro_rads - state_start.bias_gyro_rads;
            const math::Vec3d ab = imu.accel_mps2 * acc_scale_ - state_start.bias_accel_mps2;
            const math::Vec3d aw = state_start.T_world_body.so3 * ab + state_start.gravity_mps2;
            max_gyro = std::max(max_gyro, w.norm());
            max_acc_dev = std::max(max_acc_dev, aw.norm());
        }
        if (max_gyro < 0.032 && max_acc_dev < 0.32 && state_start.velocity_mps.norm() < 0.18) {
            fac.is_stationary_zupt = true;
            fac.zupt_confidence = std::clamp(1.0 - (max_gyro / 0.032), 0.25, 1.0);
        }
    }

    out_predicted_state = state_start;
    if (fac.is_stationary_zupt) {
        out_predicted_state.velocity_mps = {0.0, 0.0, 0.0};
    }

    MotionKnot knot0;
    knot0.timestamp_sec = prop_beg;
    knot0.T_world_body = out_predicted_state.T_world_body;
    knot0.velocity_world = out_predicted_state.velocity_mps;
    out_motion_knots.push_back(knot0);

    const double var_g = noise_.gyro_noise_sigma * noise_.gyro_noise_sigma;
    const double var_a = noise_.accel_noise_sigma * noise_.accel_noise_sigma;

    math::Vec3d last_omega(0.0, 0.0, 0.0);
    math::Vec3d last_acc_body(0.0, 0.0, 0.0);
    math::Vec3d last_acc_world(0.0, 0.0, 0.0);

    auto integrateStep = [&](double dt, const math::Vec3d& omega, const math::Vec3d& acc_body) {
        const double dt2 = dt * dt;
        const math::Mat3d R_k = fac.delta_R.R;
        const math::Mat3d a_hat = math::SO3d::hat(acc_body);
        const math::Vec3d w_dt = omega * dt;
        const math::SO3d dR_step = math::SO3d::exp(w_dt);
        const math::Mat3d E = dR_step.R.transpose();  // exp(-omega * dt)
        const math::Mat3d Jr = math::SO3d::rightJacobian(w_dt);

        // 1. Block-sparse 3x3 covariance propagation: Sigma <- A * Sigma * A^T + Q
        // A = [ E, 0, 0 ; F_vr, I, 0 ; F_pr, I*dt, I ]
        const math::Mat3d F_vr = (R_k * a_hat) * (-dt);
        const math::Mat3d F_pr = (R_k * a_hat) * (-0.5 * dt2);

        const math::Mat3d S00 = getBlock3(fac.covariance, 0, 0);
        const math::Mat3d S01 = getBlock3(fac.covariance, 0, 1);
        const math::Mat3d S02 = getBlock3(fac.covariance, 0, 2);
        const math::Mat3d S11 = getBlock3(fac.covariance, 1, 1);
        const math::Mat3d S12 = getBlock3(fac.covariance, 1, 2);
        const math::Mat3d S22 = getBlock3(fac.covariance, 2, 2);
        const math::Mat3d S10 = S01.transpose();
        const math::Mat3d S20 = S02.transpose();
        const math::Mat3d S21 = S12.transpose();

        // T = A * S (row blocks)
        const math::Mat3d T00 = E * S00;
        const math::Mat3d T01 = E * S01;
        const math::Mat3d T02 = E * S02;

        const math::Mat3d T10 = F_vr * S00 + S10;
        const math::Mat3d T11 = F_vr * S01 + S11;
        const math::Mat3d T12 = F_vr * S02 + S12;

        const math::Mat3d T20 = F_pr * S00 + S10 * dt + S20;
        const math::Mat3d T21 = F_pr * S01 + S11 * dt + S21;
        const math::Mat3d T22 = F_pr * S02 + S12 * dt + S22;

        // New Sigma = T * A^T + Q (upper triangular blocks then mirror)
        const math::Mat3d Et = E.transpose();
        const math::Mat3d F_vrt = F_vr.transpose();
        const math::Mat3d F_prt = F_pr.transpose();

        math::Mat3d N00 = T00 * Et;
        math::Mat3d N01 = T00 * F_vrt + T01;
        math::Mat3d N02 = T00 * F_prt + T01 * dt + T02;
        math::Mat3d N11 = T10 * F_vrt + T11;
        math::Mat3d N12 = T10 * F_prt + T11 * dt + T12;
        math::Mat3d N22 = T20 * F_prt + T21 * dt + T22;

        // Continuous-to-discrete noise injection Q = B * diag(var_g/dt, var_a/dt) * B^T * dt^2
        // Note: discrete white noise variance over dt is sigma^2 * dt for integrated increment
        const math::Mat3d Qg = (Jr * Jr.transpose()) * (var_g * dt);
        const math::Mat3d I3 = math::Mat3d::Identity();
        N00 = N00 + Qg;
        N11 = N11 + I3 * (var_a * dt);
        N12 = N12 + I3 * (var_a * 0.5 * dt2);
        N22 = N22 + I3 * (var_a * 0.25 * dt2 * dt + 1e-8 * dt);

        setBlock3(fac.covariance, 0, 0, N00);
        setBlock3(fac.covariance, 0, 1, N01);
        setBlock3(fac.covariance, 0, 2, N02);
        setBlock3(fac.covariance, 1, 0, N01.transpose());
        setBlock3(fac.covariance, 1, 1, N11);
        setBlock3(fac.covariance, 1, 2, N12);
        setBlock3(fac.covariance, 2, 0, N02.transpose());
        setBlock3(fac.covariance, 2, 1, N12.transpose());
        setBlock3(fac.covariance, 2, 2, N22);

        // 2. Analytical bias Jacobians
        const math::Mat3d R_ahat_JR = (R_k * a_hat) * fac.J_R_bg;
        fac.J_p_bg = fac.J_p_bg + fac.J_v_bg * dt - R_ahat_JR * (0.5 * dt2);
        fac.J_p_ba = fac.J_p_ba + fac.J_v_ba * dt - R_k * (0.5 * dt2);
        fac.J_v_bg = fac.J_v_bg - R_ahat_JR * dt;
        fac.J_v_ba = fac.J_v_ba - R_k * dt;
        fac.J_R_bg = E * fac.J_R_bg - Jr * dt;

        // 3. Preintegrated nominal increments
        const math::Vec3d dv_step = (R_k * acc_body) * dt;
        fac.delta_p += fac.delta_v * dt + dv_step * (0.5 * dt);
        fac.delta_v += dv_step;
        fac.delta_R = fac.delta_R * dR_step;
        fac.dt_total += dt;
        fac.sample_count++;
    };

    for (size_t i = 0; i + 1 < seq.size(); ++i) {
        const auto& head = seq[i];
        const auto& tail = seq[i + 1];
        if (tail.timestamp_sec <= prop_beg) continue;
        if (head.timestamp_sec >= prop_end) break;

        const double t0 = std::max(head.timestamp_sec, prop_beg);
        const double t1 = std::min(tail.timestamp_sec, prop_end);
        const double dt = std::clamp(t1 - t0, 1e-5, 0.05);

        const math::Vec3d omega = (head.gyro_rads + tail.gyro_rads) * 0.5 - state_start.bias_gyro_rads;
        const math::Vec3d acc_body = (head.accel_mps2 + tail.accel_mps2) * (0.5 * acc_scale_) - state_start.bias_accel_mps2;

        integrateStep(dt, omega, acc_body);

        if (!fac.is_stationary_zupt) {
            out_predicted_state.T_world_body.so3 = out_predicted_state.T_world_body.so3 * math::SO3d::exp(omega * dt);
            const math::Vec3d acc_world = out_predicted_state.T_world_body.so3 * acc_body + out_predicted_state.gravity_mps2;
            out_predicted_state.T_world_body.t += out_predicted_state.velocity_mps * dt + acc_world * (0.5 * dt * dt);
            out_predicted_state.velocity_mps += acc_world * dt;
            const double v_norm = out_predicted_state.velocity_mps.norm();
            if (v_norm > 6.5) {
                out_predicted_state.velocity_mps *= (6.5 / v_norm);
            }
            last_omega = omega;
            last_acc_body = acc_body;
            last_acc_world = acc_world;
        }
        out_predicted_state.timestamp_sec = t1;

        MotionKnot knot;
        knot.timestamp_sec = t1;
        knot.T_world_body = out_predicted_state.T_world_body;
        knot.velocity_world = out_predicted_state.velocity_mps;
        knot.omega_body = fac.is_stationary_zupt ? math::Vec3d(0.0, 0.0, 0.0) : omega;
        knot.accel_world = fac.is_stationary_zupt ? math::Vec3d(0.0, 0.0, 0.0) : last_acc_world;
        out_motion_knots.push_back(knot);
    }

    // Bridge final fractional gap cleanly to exact sweep-end timestamp
    if (prop_end > out_predicted_state.timestamp_sec + 1e-6) {
        const double dt = std::clamp(prop_end - out_predicted_state.timestamp_sec, 1e-5, 0.05);
        integrateStep(dt, last_omega, last_acc_body);
        if (!fac.is_stationary_zupt) {
            out_predicted_state.T_world_body.so3 = out_predicted_state.T_world_body.so3 * math::SO3d::exp(last_omega * dt);
            out_predicted_state.T_world_body.t += out_predicted_state.velocity_mps * dt + last_acc_world * (0.5 * dt * dt);
            out_predicted_state.velocity_mps += last_acc_world * dt;
        }
        out_predicted_state.timestamp_sec = prop_end;

        MotionKnot end_knot;
        end_knot.timestamp_sec = prop_end;
        end_knot.T_world_body = out_predicted_state.T_world_body;
        end_knot.velocity_world = out_predicted_state.velocity_mps;
        end_knot.omega_body = last_omega;
        end_knot.accel_world = last_acc_world;
        out_motion_knots.push_back(end_knot);
    }

    // Symmetrize 9x9 preintegrated covariance
    for (int r = 0; r < 9; ++r) {
        for (int c = r + 1; c < 9; ++c) {
            const double avg = 0.5 * (fac.covariance(r, c) + fac.covariance(c, r));
            fac.covariance(r, c) = fac.covariance(c, r) = avg;
        }
    }

    return fac;
}

void AvImuPreintegrator::deskewScanToSweepEnd(
    const core::LidarFrame& raw_lidar,
    const math::SE3d& T_imu_lidar,
    const math::SE3d& T_world_body_end,
    const std::vector<MotionKnot>& motion_knots,
    core::LidarFrame& out_deskewed_body) {
    out_deskewed_body = raw_lidar;
    const math::SE3d T_end_inv = T_world_body_end.inverse();
    const size_t n_knots = motion_knots.size();

    struct TwistSegment {
        double t0{0.0};
        double inv_dt{0.0};
        math::Mat3d R_end_k0{};
        math::Vec3d t_end_k0{};
        math::Vec3d d_omega{};
        math::Vec3d d_trans{};
    };

    std::vector<TwistSegment> segs;
    if (n_knots >= 2) {
        segs.resize(n_knots - 1);
        for (size_t s = 0; s + 1 < n_knots; ++s) {
            const auto& k0 = motion_knots[s];
            const auto& k1 = motion_knots[s + 1];
            const double sdt = k1.timestamp_sec - k0.timestamp_sec;
            segs[s].t0 = k0.timestamp_sec;
            segs[s].inv_dt = (sdt > 1e-7) ? (1.0 / sdt) : 0.0;
            const math::SE3d T_end_k0 = T_end_inv * k0.T_world_body;
            const math::SE3d T_end_k1 = T_end_inv * k1.T_world_body;
            segs[s].R_end_k0 = T_end_k0.so3.R;
            segs[s].t_end_k0 = T_end_k0.t;
            segs[s].d_omega = (T_end_k0.so3.inverse() * T_end_k1.so3).log();
            segs[s].d_trans = T_end_k1.t - T_end_k0.t;
        }
    }

    size_t hint = 0;
    const double t_first = motion_knots.empty() ? 0.0 : motion_knots.front().timestamp_sec;

    for (auto& pt : out_deskewed_body.points) {
        const math::Vec3d p_imu = T_imu_lidar.transform(math::Vec3d(pt.x, pt.y, pt.z));
        if (!segs.empty() && pt.timestamp_sec >= t_first) {
            if (pt.timestamp_sec < motion_knots[hint].timestamp_sec) {
                hint = 0;
            }
            while (hint + 1 < segs.size() && pt.timestamp_sec > motion_knots[hint + 1].timestamp_sec) {
                hint++;
            }
            const auto& sg = segs[hint];
            const double alpha = std::clamp((pt.timestamp_sec - sg.t0) * sg.inv_dt, 0.0, 1.0);
            const math::Vec3d w = sg.d_omega * alpha;
            const math::Vec3d w_x_p = w.cross(p_imu);
            const math::Vec3d p_rot = p_imu + w_x_p + w.cross(w_x_p) * 0.5;
            const math::Vec3d p_end = (sg.R_end_k0 * p_rot) + sg.t_end_k0 + sg.d_trans * alpha;
            pt.x = p_end.x;
            pt.y = p_end.y;
            pt.z = p_end.z;
        } else {
            pt.x = p_imu.x;
            pt.y = p_imu.y;
            pt.z = p_imu.z;
        }
    }
}

}  // namespace slam
}  // namespace avscan
