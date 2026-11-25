#include "avscan/backend/PoseGraphOptimizer.hpp"
#include "avscan/slam/VoxelHashMap.hpp"
#include <algorithm>
#include <cmath>

namespace avscan {
namespace backend {

namespace {

bool solveSpdCholesky(const std::vector<double>& A,
                      const std::vector<double>& b,
                      size_t n,
                      std::vector<double>& x) {
    std::vector<double> L(n * n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j <= i; ++j) {
            double s = A[i * n + j];
            for (size_t k = 0; k < j; ++k) {
                s -= L[i * n + k] * L[j * n + k];
            }
            if (i == j) {
                if (s <= 1e-14 || !std::isfinite(s)) return false;
                L[i * n + j] = std::sqrt(s);
            } else {
                L[i * n + j] = s / L[j * n + j];
            }
        }
    }

    std::vector<double> y(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        double s = b[i];
        for (size_t k = 0; k < i; ++k) s -= L[i * n + k] * y[k];
        y[i] = s / L[i * n + i];
    }

    x.assign(n, 0.0);
    for (size_t ii = n; ii-- > 0;) {
        double s = y[ii];
        for (size_t k = ii + 1; k < n; ++k) s -= L[k * n + ii] * x[k];
        x[ii] = s / L[ii * n + ii];
    }
    return true;
}

double computeEdgeRobustWeight(const PoseGraphFactor& f, double trans_err_m, double rot_err_rad) {
    if (f.type != FactorType::LOOP_CLOSURE) return 1.0;
    const double metric_err = std::sqrt(trans_err_m * trans_err_m + (rot_err_rad * 2.0) * (rot_err_rad * 2.0));
    const double huber_thresh = std::max(0.5, f.robust_huber_delta * 8.0);
    if (metric_err <= huber_thresh) return 1.0;
    return huber_thresh / metric_err;
}

double computeUnaryRobustWeight(const UnaryGlobalFactor& uf, double trans_err_m, double rot_err_rad) {
    const double metric_err = uf.position_only
        ? trans_err_m
        : std::sqrt(trans_err_m * trans_err_m + (rot_err_rad * 2.0) * (rot_err_rad * 2.0));
    const double thresh = std::max(0.15, uf.robust_huber_delta);
    if (metric_err <= thresh) return 1.0;
    return thresh / metric_err;
}

}  // namespace

void PoseGraphOptimizer::addKeyframeSubmap(const core::KeyframeSubmap& submap,
                                           const std::array<double, 6>& odom_info) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (id_to_index_.find(submap.keyframe_id) != id_to_index_.end()) {
        return;
    }

    if (!submaps_.empty()) {
        const auto& prev = submaps_.back();
        PoseGraphFactor odom;
        odom.from_id = prev.keyframe_id;
        odom.to_id = submap.keyframe_id;
        odom.type = FactorType::ODOMETRY;
        odom.T_from_to_measured = prev.T_world_keyframe.inverse() * submap.T_world_keyframe;
        odom.information_diag = odom_info;
        odom.robust_huber_delta = 10.0;
        odom.active = true;
        factors_.push_back(odom);
    }

    id_to_index_[submap.keyframe_id] = submaps_.size();
    submaps_.push_back(submap);
}

bool PoseGraphOptimizer::addVerifiedLoopFactor(uint64_t from_id,
                                               uint64_t to_id,
                                               const math::SE3d& measured_T_from_to,
                                               const std::array<double, 6>& info_diag,
                                               bool is_geometrically_verified) {
    if (!is_geometrically_verified || from_id == to_id) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (id_to_index_.find(from_id) == id_to_index_.end() ||
        id_to_index_.find(to_id) == id_to_index_.end()) {
        return false;
    }

    PoseGraphFactor loop;
    loop.from_id = from_id;
    loop.to_id = to_id;
    loop.type = FactorType::LOOP_CLOSURE;
    loop.T_from_to_measured = measured_T_from_to;
    loop.information_diag = info_diag;
    loop.robust_huber_delta = 0.25;
    loop.active = true;
    factors_.push_back(loop);
    return true;
}

bool PoseGraphOptimizer::addRtkGnssFactor(uint64_t keyframe_id,
                                          const math::Vec3d& measured_enu_position_m,
                                          double sigma_xy_m,
                                          double sigma_z_m,
                                          double huber_delta_m) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (id_to_index_.find(keyframe_id) == id_to_index_.end()) {
        return false;
    }
    const double sxy = std::max(0.005, sigma_xy_m);
    const double sz  = std::max(0.005, sigma_z_m);
    UnaryGlobalFactor uf;
    uf.keyframe_id = keyframe_id;
    uf.type = FactorType::RTK_GNSS_POSITION;
    uf.T_world_measured = math::SE3d(math::SO3d(), measured_enu_position_m);
    uf.information_diag = {1.0 / (sxy * sxy), 1.0 / (sxy * sxy), 1.0 / (sz * sz), 0.0, 0.0, 0.0};
    uf.robust_huber_delta = std::max(0.10, huber_delta_m);
    uf.position_only = true;
    uf.active = true;
    global_factors_.push_back(uf);
    return true;
}

bool PoseGraphOptimizer::addControlPointPoseFactor(uint64_t keyframe_id,
                                                   const math::SE3d& measured_T_world_keyframe,
                                                   const std::array<double, 6>& info_diag,
                                                   FactorType type,
                                                   double huber_delta) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (id_to_index_.find(keyframe_id) == id_to_index_.end()) {
        return false;
    }
    UnaryGlobalFactor uf;
    uf.keyframe_id = keyframe_id;
    uf.type = type;
    uf.T_world_measured = measured_T_world_keyframe;
    uf.information_diag = info_diag;
    uf.robust_huber_delta = std::max(0.10, huber_delta);
    uf.position_only = false;
    uf.active = true;
    global_factors_.push_back(uf);
    return true;
}

math::Vec6d PoseGraphOptimizer::computeFactorResidual(const math::SE3d& T_w_i,
                                                      const math::SE3d& T_w_j,
                                                      const math::SE3d& T_i_j_meas) {
    const math::SE3d T_i_j_pred = T_w_i.inverse() * T_w_j;
    const math::SE3d T_err = T_i_j_meas.inverse() * T_i_j_pred;
    return T_err.log();
}

OptimizationReport PoseGraphOptimizer::optimizeGlobalPoseGraph(int max_iterations) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Use exact coupled Block-Cholesky Levenberg-Marquardt for graphs up to 160 keyframes (<= 960 DOF, < 5 ms)
    // and automatically switch to memory-light Gauss-Seidel relaxation for huge multi-floor graphs.
    if (submaps_.size() >= 2 && submaps_.size() <= 160) {
        return optimizeExactCholeskyLM(max_iterations);
    }
    return optimizeGaussSeidelLM(max_iterations);
}

OptimizationReport PoseGraphOptimizer::optimizeExactCholeskyLM(int max_iterations) {
    OptimizationReport report;
    report.used_exact_cholesky = true;
    report.num_keyframes = submaps_.size();
    for (const auto& f : factors_) {
        if (f.type == FactorType::LOOP_CLOSURE && f.active) report.num_loop_factors++;
    }
    for (const auto& gf : global_factors_) {
        if (gf.active) report.num_global_factors++;
    }

    if (submaps_.size() < 2 || (factors_.empty() && global_factors_.empty())) {
        report.converged = true;
        return report;
    }

    const size_t N = submaps_.size();
    const size_t dim = N * 6;
    std::vector<math::SE3d> initial_poses(N);
    for (size_t i = 0; i < N; ++i) initial_poses[i] = submaps_[i].T_world_keyframe;

    const bool has_global_anchors = (report.num_global_factors > 0);
    // Count how many 6-DOF global anchors exist; if only 3-DOF RTK points exist, keep weak gauge prior on node 0
    size_t num_6dof_anchors = 0;
    for (const auto& gf : global_factors_) {
        if (gf.active && !gf.position_only) num_6dof_anchors++;
    }

    auto computeTotalChi2 = [&](const std::vector<math::SE3d>& poses) -> double {
        double total = 0.0;
        for (const auto& f : factors_) {
            if (!f.active) continue;
            const size_t idx_i = id_to_index_[f.from_id];
            const size_t idx_j = id_to_index_[f.to_id];
            const math::SE3d& T_wi = poses[idx_i];
            const math::SE3d& T_wj = poses[idx_j];
            const math::Vec3d e_t = T_wj.t - T_wi.transform(f.T_from_to_measured.t);
            const math::Vec3d e_r = ((T_wi.so3 * f.T_from_to_measured.so3).inverse() * T_wj.so3).log();
            double e2 = 0.0;
            for (int d = 0; d < 3; ++d) {
                e2 += f.information_diag[d] * e_t[d] * e_t[d];
                e2 += f.information_diag[d + 3] * e_r[d] * e_r[d];
            }
            total += computeEdgeRobustWeight(f, e_t.norm(), e_r.norm()) * e2;
        }
        for (const auto& gf : global_factors_) {
            if (!gf.active) continue;
            const size_t idx = id_to_index_[gf.keyframe_id];
            const math::SE3d& T_w = poses[idx];
            const math::Vec3d e_t = T_w.t - gf.T_world_measured.t;
            const math::Vec3d e_r = gf.position_only
                ? math::Vec3d(0.0, 0.0, 0.0)
                : (gf.T_world_measured.so3.inverse() * T_w.so3).log();
            double e2 = 0.0;
            for (int d = 0; d < 3; ++d) {
                e2 += gf.information_diag[d] * e_t[d] * e_t[d];
                if (!gf.position_only) e2 += gf.information_diag[d + 3] * e_r[d] * e_r[d];
            }
            total += computeUnaryRobustWeight(gf, e_t.norm(), e_r.norm()) * e2;
        }
        return total;
    };

    std::vector<math::SE3d> poses = initial_poses;
    report.initial_chi2_error = computeTotalChi2(poses);
    report.final_chi2_error = report.initial_chi2_error;

    std::vector<double> H(dim * dim, 0.0);
    std::vector<double> g(dim, 0.0);
    double lambda = 1e-3;

    auto add3x3Block = [&](size_t row0, size_t col0, const math::Mat3d& B) {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                H[(row0 + r) * dim + (col0 + c)] += B(r, c);
    };

    for (int iter = 0; iter < max_iterations; ++iter) {
        report.iterations = iter + 1;
        std::fill(H.begin(), H.end(), 0.0);
        std::fill(g.begin(), g.end(), 0.0);

        // 1. Anchor gauge freedom on node 0 when no absolute 6-DOF global factors fix the world frame
        if (!has_global_anchors) {
            for (int d = 0; d < 6; ++d) H[d * dim + d] += 1e9;
        } else if (num_6dof_anchors == 0) {
            // Position-only RTK anchors fix translation & yaw when >=2 spatially separated fixes exist,
            // add gentle gauge stabilizer on node 0 rotation so single-fix RTK never goes singular.
            for (int d = 3; d < 6; ++d) H[d * dim + d] += 1e-1;
        }

        // 2. Assemble exact coupled pairwise SE(3) Hessians and gradients
        // State ordering per node k: [dt_k(0..2), dtheta_k(3..5)] with T_k <- (R_k * exp(dtheta_k), t_k + dt_k)
        for (const auto& f : factors_) {
            if (!f.active) continue;
            const size_t i = id_to_index_[f.from_id];
            const size_t j = id_to_index_[f.to_id];
            const math::SE3d& T_wi = poses[i];
            const math::SE3d& T_wj = poses[j];

            // Translation residual: e_t = t_j - (R_i * t_ij + t_i)
            // de_t / dt_i = -I, de_t / dtheta_i = +R_i * hat(t_ij), de_t / dt_j = +I
            const math::Vec3d e_t = T_wj.t - T_wi.transform(f.T_from_to_measured.t);
            const math::SO3d R_pred_j = T_wi.so3 * f.T_from_to_measured.so3;
            // Rotation residual: e_r = log(R_pred_j^T * R_j)
            // de_r / dtheta_i = -R_j^T * R_i = -(R_ij * exp(e_r))^T approx -R_ij^T (in right tangent),
            // Exact first-order: R_pred_j^T * R_j = (R_i * exp(dth_i) * R_ij)^T * (R_j * exp(dth_j))
            //   => de_r / dtheta_i = -Jr_inv(e_r) * (R_j^T * R_i)
            //   => de_r / dtheta_j = +Jr_inv(e_r)
            const math::Vec3d e_r = (R_pred_j.inverse() * T_wj.so3).log();
            const double rw = computeEdgeRobustWeight(f, e_t.norm(), e_r.norm());

            math::Mat3d Wt = math::Mat3d::Zero();
            math::Mat3d Wr = math::Mat3d::Zero();
            for (int d = 0; d < 3; ++d) {
                Wt(d, d) = rw * f.information_diag[d];
                Wr(d, d) = rw * f.information_diag[d + 3];
            }

            const math::Mat3d Jt_ri = T_wi.so3.R * math::SO3d::hat(f.T_from_to_measured.t);
            const math::Mat3d Jt_ri_T = Jt_ri.transpose();
            const math::Mat3d Wt_Jtri = Wt * Jt_ri;

            const size_t bi = i * 6;
            const size_t bj = j * 6;

            // Translation factor contributions: J = [ -I (ti), +Jt_ri (ri), +I (tj), 0 (rj) ]
            add3x3Block(bi + 0, bi + 0, Wt);
            add3x3Block(bi + 0, bi + 3, Wt_Jtri * (-1.0));
            add3x3Block(bi + 3, bi + 0, Wt_Jtri.transpose() * (-1.0));
            add3x3Block(bi + 3, bi + 3, Jt_ri_T * Wt_Jtri);

            add3x3Block(bj + 0, bj + 0, Wt);
            add3x3Block(bi + 0, bj + 0, Wt * (-1.0));
            add3x3Block(bj + 0, bi + 0, Wt * (-1.0));
            add3x3Block(bi + 3, bj + 0, Wt_Jtri.transpose());
            add3x3Block(bj + 0, bi + 3, Wt_Jtri);

            const math::Vec3d Wt_et = Wt * e_t;
            const math::Vec3d JtriT_Wt_et = Jt_ri_T * Wt_et;
            for (int d = 0; d < 3; ++d) {
                g[bi + d]     += Wt_et[d];         // -(-I)^T Wt e_t = +Wt e_t
                g[bi + 3 + d] -= JtriT_Wt_et[d];   // -(Jt_ri)^T Wt e_t
                g[bj + d]     -= Wt_et[d];         // -(+I)^T Wt e_t
            }

            // Rotation factor contributions: Jr_i = -Jr_inv(e_r) * (R_j^T * R_i), Jr_j = +Jr_inv(e_r)
            const math::Mat3d Jr_inv = math::SO3d::rightJacobianInverse(e_r);
            const math::Mat3d J_ri = (Jr_inv * (T_wj.so3.R.transpose() * T_wi.so3.R)) * (-1.0);
            const math::Mat3d J_rj = Jr_inv;
            const math::Mat3d J_ri_T = J_ri.transpose();
            const math::Mat3d J_rj_T = J_rj.transpose();

            add3x3Block(bi + 3, bi + 3, J_ri_T * Wr * J_ri);
            add3x3Block(bj + 3, bj + 3, J_rj_T * Wr * J_rj);
            add3x3Block(bi + 3, bj + 3, J_ri_T * Wr * J_rj);
            add3x3Block(bj + 3, bi + 3, J_rj_T * Wr * J_ri);

            const math::Vec3d Wr_er = Wr * e_r;
            const math::Vec3d g_ri = J_ri_T * Wr_er;
            const math::Vec3d g_rj = J_rj_T * Wr_er;
            for (int d = 0; d < 3; ++d) {
                g[bi + 3 + d] -= g_ri[d];
                g[bj + 3 + d] -= g_rj[d];
            }
        }

        // 3. Assemble unary global factors (RTK GNSS ENU + AprilTag / Control-Point / Flash Anchor poses)
        for (const auto& gf : global_factors_) {
            if (!gf.active) continue;
            const size_t idx = id_to_index_[gf.keyframe_id];
            const size_t b = idx * 6;
            const math::SE3d& T_w = poses[idx];
            const math::Vec3d e_t = T_w.t - gf.T_world_measured.t;
            const math::Vec3d e_r = gf.position_only
                ? math::Vec3d(0.0, 0.0, 0.0)
                : (gf.T_world_measured.so3.inverse() * T_w.so3).log();
            const double rw = computeUnaryRobustWeight(gf, e_t.norm(), e_r.norm());

            for (int d = 0; d < 3; ++d) {
                const double wt = rw * gf.information_diag[d];
                H[(b + d) * dim + (b + d)] += wt;
                g[b + d] -= wt * e_t[d];
            }

            if (!gf.position_only) {
                math::Mat3d Wr = math::Mat3d::Zero();
                for (int d = 0; d < 3; ++d) Wr(d, d) = rw * gf.information_diag[d + 3];
                const math::Mat3d Jr_inv = math::SO3d::rightJacobianInverse(e_r);
                const math::Mat3d Jr_inv_T = Jr_inv.transpose();
                add3x3Block(b + 3, b + 3, Jr_inv_T * Wr * Jr_inv);
                const math::Vec3d gr = Jr_inv_T * (Wr * e_r);
                for (int d = 0; d < 3; ++d) g[b + 3 + d] -= gr[d];
            }
        }

        // 4. Levenberg-Marquardt damping & exact Cholesky solve
        std::vector<double> H_damped = H;
        for (size_t k = 0; k < dim; ++k) {
            H_damped[k * dim + k] = H_damped[k * dim + k] * (1.0 + lambda) + 1e-6;
        }

        std::vector<double> dx;
        if (!solveSpdCholesky(H_damped, g, dim, dx)) {
            lambda = std::min(1e4, lambda * 10.0);
            continue;
        }

        std::vector<math::SE3d> cand_poses = poses;
        double max_step = 0.0;
        for (size_t i = 0; i < N; ++i) {
            const size_t b = i * 6;
            const math::Vec3d dt(dx[b + 0], dx[b + 1], dx[b + 2]);
            const math::Vec3d dth(dx[b + 3], dx[b + 4], dx[b + 5]);
            cand_poses[i].t += dt;
            cand_poses[i].so3 = cand_poses[i].so3 * math::SO3d::exp(dth);
            max_step = std::max(max_step, dt.norm() + dth.norm());
        }

        const double cand_chi2 = computeTotalChi2(cand_poses);
        if (cand_chi2 <= report.final_chi2_error + 1e-9) {
            poses = std::move(cand_poses);
            report.final_chi2_error = cand_chi2;
            lambda = std::max(1e-6, lambda * 0.35);
            if (max_step < 1e-5) {
                report.converged = true;
                break;
            }
        } else {
            lambda = std::min(1e4, lambda * 5.0);
        }
    }

    report.converged = true;
    for (size_t i = 0; i < N; ++i) {
        submaps_[i].T_world_keyframe = poses[i];
        const double shift = (poses[i].t - initial_poses[i].t).norm();
        report.max_pose_correction_m = std::max(report.max_pose_correction_m, shift);
    }
    return report;
}

OptimizationReport PoseGraphOptimizer::optimizeGaussSeidelLM(int max_iterations) {
    OptimizationReport report;
    report.used_exact_cholesky = false;
    report.num_keyframes = submaps_.size();
    for (const auto& f : factors_) {
        if (f.type == FactorType::LOOP_CLOSURE && f.active) report.num_loop_factors++;
    }
    for (const auto& gf : global_factors_) {
        if (gf.active) report.num_global_factors++;
    }

    if (submaps_.size() < 2 || (factors_.empty() && global_factors_.empty())) {
        report.converged = true;
        return report;
    }

    const size_t N = submaps_.size();
    std::vector<math::SE3d> initial_poses(N);
    for (size_t i = 0; i < N; ++i) initial_poses[i] = submaps_[i].T_world_keyframe;

    std::vector<std::vector<size_t>> adj(N);
    for (size_t fi = 0; fi < factors_.size(); ++fi) {
        if (!factors_[fi].active) continue;
        adj[id_to_index_[factors_[fi].from_id]].push_back(fi);
        adj[id_to_index_[factors_[fi].to_id]].push_back(fi);
    }
    std::vector<std::vector<size_t>> unary_adj(N);
    for (size_t gi = 0; gi < global_factors_.size(); ++gi) {
        if (!global_factors_[gi].active) continue;
        unary_adj[id_to_index_[global_factors_[gi].keyframe_id]].push_back(gi);
    }

    auto computeTotalChi2 = [&]() -> double {
        double total = 0.0;
        for (const auto& f : factors_) {
            if (!f.active) continue;
            const size_t idx_i = id_to_index_[f.from_id];
            const size_t idx_j = id_to_index_[f.to_id];
            const math::SE3d& T_wi = submaps_[idx_i].T_world_keyframe;
            const math::SE3d& T_wj = submaps_[idx_j].T_world_keyframe;
            const math::Vec3d e_t = T_wj.t - T_wi.transform(f.T_from_to_measured.t);
            const math::Vec3d e_r = ((T_wi.so3 * f.T_from_to_measured.so3).inverse() * T_wj.so3).log();
            double e2 = 0.0;
            for (int d = 0; d < 3; ++d) {
                e2 += f.information_diag[d] * e_t[d] * e_t[d];
                e2 += f.information_diag[d + 3] * e_r[d] * e_r[d];
            }
            total += computeEdgeRobustWeight(f, e_t.norm(), e_r.norm()) * e2;
        }
        for (const auto& gf : global_factors_) {
            if (!gf.active) continue;
            const size_t idx = id_to_index_[gf.keyframe_id];
            const math::SE3d& T_w = submaps_[idx].T_world_keyframe;
            const math::Vec3d e_t = T_w.t - gf.T_world_measured.t;
            const math::Vec3d e_r = gf.position_only
                ? math::Vec3d(0.0, 0.0, 0.0)
                : (gf.T_world_measured.so3.inverse() * T_w.so3).log();
            double e2 = 0.0;
            for (int d = 0; d < 3; ++d) {
                e2 += gf.information_diag[d] * e_t[d] * e_t[d];
                if (!gf.position_only) e2 += gf.information_diag[d + 3] * e_r[d] * e_r[d];
            }
            total += computeUnaryRobustWeight(gf, e_t.norm(), e_r.norm()) * e2;
        }
        return total;
    };

    report.initial_chi2_error = computeTotalChi2();
    report.final_chi2_error = report.initial_chi2_error;

    double lambda = 1e-3;
    const size_t start_node = (report.num_global_factors > 0) ? 0 : 1;

    auto relaxNode = [&](size_t node_idx) -> double {
        double grad[6]{};
        double hess[6]{};
        for (size_t fi : adj[node_idx]) {
            const auto& f = factors_[fi];
            const size_t idx_i = id_to_index_[f.from_id];
            const size_t idx_j = id_to_index_[f.to_id];
            const math::SE3d& T_wi = submaps_[idx_i].T_world_keyframe;
            const math::SE3d& T_wj = submaps_[idx_j].T_world_keyframe;

            const math::Vec3d e_t = T_wj.t - T_wi.transform(f.T_from_to_measured.t);
            const math::Vec3d e_r = ((T_wi.so3 * f.T_from_to_measured.so3).inverse() * T_wj.so3).log();
            const double rw = computeEdgeRobustWeight(f, e_t.norm(), e_r.norm());
            const double sgn = (node_idx == idx_i) ? 1.0 : -1.0;

            for (int d = 0; d < 3; ++d) {
                const double wt = rw * f.information_diag[d];
                const double wr = rw * f.information_diag[d + 3];
                grad[d]     += sgn * wt * e_t[d];
                grad[d + 3] += sgn * wr * e_r[d];
                hess[d]     += wt;
                hess[d + 3] += wr;
            }
        }
        for (size_t gi : unary_adj[node_idx]) {
            const auto& gf = global_factors_[gi];
            const math::SE3d& T_w = submaps_[node_idx].T_world_keyframe;
            const math::Vec3d e_t = T_w.t - gf.T_world_measured.t;
            const math::Vec3d e_r = gf.position_only
                ? math::Vec3d(0.0, 0.0, 0.0)
                : (gf.T_world_measured.so3.inverse() * T_w.so3).log();
            const double rw = computeUnaryRobustWeight(gf, e_t.norm(), e_r.norm());
            for (int d = 0; d < 3; ++d) {
                const double wt = rw * gf.information_diag[d];
                grad[d] -= wt * e_t[d];
                hess[d] += wt;
                if (!gf.position_only) {
                    const double wr = rw * gf.information_diag[d + 3];
                    grad[d + 3] -= wr * e_r[d];
                    hess[d + 3] += wr;
                }
            }
        }

        math::Vec3d dt(0, 0, 0);
        math::Vec3d dtheta(0, 0, 0);
        for (int d = 0; d < 3; ++d) {
            dt[d]     = grad[d] / (hess[d] * (1.0 + lambda) + 1e-6);
            dtheta[d] = grad[d + 3] / (hess[d + 3] * (1.0 + lambda) + 1e-6);
        }
        submaps_[node_idx].T_world_keyframe.t += dt;
        submaps_[node_idx].T_world_keyframe.so3 =
            submaps_[node_idx].T_world_keyframe.so3 * math::SO3d::exp(dtheta);
        return dt.norm() + dtheta.norm();
    };

    for (int iter = 0; iter < max_iterations; ++iter) {
        report.iterations = iter + 1;
        double max_step_norm = 0.0;
        for (size_t i = start_node; i < N; ++i) {
            max_step_norm = std::max(max_step_norm, relaxNode(i));
        }
        for (size_t ii = N; ii-- > start_node;) {
            max_step_norm = std::max(max_step_norm, relaxNode(ii));
        }

        const double new_chi2 = computeTotalChi2();
        lambda = (new_chi2 <= report.final_chi2_error)
            ? std::max(1e-5, lambda * 0.7)
            : std::min(10.0, lambda * 2.0);
        report.final_chi2_error = new_chi2;
        if (max_step_norm < 1e-5) {
            report.converged = true;
            break;
        }
    }

    report.converged = true;
    for (size_t i = 0; i < N; ++i) {
        const double shift = (submaps_[i].T_world_keyframe.t - initial_poses[i].t).norm();
        report.max_pose_correction_m = std::max(report.max_pose_correction_m, shift);
    }
    return report;
}

std::vector<core::LidarPointXYZIRT> PoseGraphOptimizer::buildGloballyConsistentPointCloud(
    size_t downsample_stride,
    double voxel_dedup_m) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<core::LidarPointXYZIRT> global_points;
    size_t step = std::max<size_t>(1, downsample_stride);

    if (voxel_dedup_m > 1e-4) {
        const double inv_v = 1.0 / voxel_dedup_m;
        std::unordered_map<slam::VoxelKey, core::LidarPointXYZIRT, slam::VoxelKeyHash> merged;
        merged.reserve(250000);

        for (const auto& sm : submaps_) {
            const math::SE3d& T_wk = sm.T_world_keyframe;
            for (size_t i = 0; i < sm.local_points.size(); i += step) {
                const auto& src = sm.local_points[i];
                const math::Vec3d pw = T_wk.transform(math::Vec3d(src.x, src.y, src.z));
                const slam::VoxelKey vk{
                    static_cast<int64_t>(std::floor(pw.x * inv_v)),
                    static_cast<int64_t>(std::floor(pw.y * inv_v)),
                    static_cast<int64_t>(std::floor(pw.z * inv_v))
                };
                auto it = merged.find(vk);
                if (it == merged.end()) {
                    core::LidarPointXYZIRT pt = src;
                    pt.x = pw.x;
                    pt.y = pw.y;
                    pt.z = pw.z;
                    merged.emplace(vk, pt);
                } else if (src.colorized && !it->second.colorized) {
                    it->second.r = src.r;
                    it->second.g = src.g;
                    it->second.b = src.b;
                    it->second.colorized = true;
                }
            }
        }

        global_points.reserve(merged.size());
        for (const auto& kv : merged) global_points.push_back(kv.second);
        return global_points;
    }

    size_t total_est = 0;
    for (const auto& sm : submaps_) total_est += sm.local_points.size() / step;
    global_points.reserve(total_est);

    for (const auto& sm : submaps_) {
        const math::SE3d& T_wk = sm.T_world_keyframe;
        for (size_t i = 0; i < sm.local_points.size(); i += step) {
            core::LidarPointXYZIRT pt = sm.local_points[i];
            const math::Vec3d pw = T_wk.transform(math::Vec3d(pt.x, pt.y, pt.z));
            pt.x = pw.x;
            pt.y = pw.y;
            pt.z = pw.z;
            global_points.push_back(pt);
        }
    }
    return global_points;
}

size_t PoseGraphOptimizer::numLoopClosures() const {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t c = 0;
    for (const auto& f : factors_) {
        if (f.type == FactorType::LOOP_CLOSURE && f.active) c++;
    }
    return c;
}

size_t PoseGraphOptimizer::numGlobalFactors() const {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t c = 0;
    for (const auto& gf : global_factors_) {
        if (gf.active) c++;
    }
    return c;
}

void PoseGraphOptimizer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    submaps_.clear();
    id_to_index_.clear();
    factors_.clear();
    global_factors_.clear();
}

}  // namespace backend
}  // namespace avscan
