#pragma once

#include "avscan/core/Types.hpp"
#include <array>
#include <cstdint>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace avscan {
namespace backend {

enum class FactorType : uint8_t {
    ODOMETRY = 0,
    LOOP_CLOSURE = 1,
    RTK_GNSS_POSITION = 2,   // 3-DOF global ENU position constraint (e.g., Emlid Reach RX / viDoc RTK)
    CONTROL_POINT_POSE = 3,  // 6-DOF absolute pose constraint (AprilTag / surveyed GCP / Flash Anchor)
    FLASH_ANCHOR = 4         // High-rigidity single-shot stationary Flash Scan anchor
};

struct PoseGraphFactor {
    uint64_t from_id{0};
    uint64_t to_id{0};
    FactorType type{FactorType::ODOMETRY};
    math::SE3d T_from_to_measured{math::SE3d::Identity()};
    std::array<double, 6> information_diag{100.0, 100.0, 100.0, 400.0, 400.0, 400.0};
    double robust_huber_delta{0.25};
    bool active{true};
};

// Unary global constraint anchored on a single keyframe (RTK GNSS ENU or AprilTag / Surveyed GCP pose)
struct UnaryGlobalFactor {
    uint64_t keyframe_id{0};
    FactorType type{FactorType::RTK_GNSS_POSITION};
    math::SE3d T_world_measured{math::SE3d::Identity()};
    std::array<double, 6> information_diag{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // [tx, ty, tz, rx, ry, rz]
    double robust_huber_delta{0.50};
    bool position_only{true};
    bool active{true};
};

struct OptimizationReport {
    bool converged{false};
    bool used_exact_cholesky{false};
    int iterations{0};
    double initial_chi2_error{0.0};
    double final_chi2_error{0.0};
    double max_pose_correction_m{0.0};
    size_t num_keyframes{0};
    size_t num_loop_factors{0};
    size_t num_global_factors{0};
};

class PoseGraphOptimizer {
public:
    PoseGraphOptimizer() = default;

    // Adds a new keyframe submap (and sequential odometry constraint from previous keyframe if any)
    void addKeyframeSubmap(const core::KeyframeSubmap& submap,
                           const std::array<double, 6>& odom_info = {200.0, 200.0, 200.0, 800.0, 800.0, 800.0});

    // Adds a geometrically verified loop closure constraint (never accepts unverified constraints)
    bool addVerifiedLoopFactor(uint64_t from_id,
                               uint64_t to_id,
                               const math::SE3d& measured_T_from_to,
                               const std::array<double, 6>& info_diag,
                               bool is_geometrically_verified);

    // Adds a 3-DOF RTK GNSS ENU position factor (horizontal sigma_xy_m, vertical sigma_z_m)
    bool addRtkGnssFactor(uint64_t keyframe_id,
                          const math::Vec3d& measured_enu_position_m,
                          double sigma_xy_m = 0.02,
                          double sigma_z_m = 0.04,
                          double huber_delta_m = 0.50);

    // Adds a 6-DOF absolute pose control-point factor (AprilTag / Surveyed Benchmark / Flash Anchor)
    bool addControlPointPoseFactor(uint64_t keyframe_id,
                                   const math::SE3d& measured_T_world_keyframe,
                                   const std::array<double, 6>& info_diag = {2500.0, 2500.0, 2500.0, 4000.0, 4000.0, 4000.0},
                                   FactorType type = FactorType::CONTROL_POINT_POSE,
                                   double huber_delta = 0.50);

    // Executes Exact Coupled Cholesky Levenberg-Marquardt on SE(3) (for N <= 250 keyframes)
    // with Huber robust kernels, automatically falling back to Gauss-Seidel for huge graphs.
    OptimizationReport optimizeGlobalPoseGraph(int max_iterations = 25);

    // Reconstructs the globally consistent point cloud from all corrected keyframe submaps.
    std::vector<core::LidarPointXYZIRT> buildGloballyConsistentPointCloud(
        size_t downsample_stride = 1,
        double voxel_dedup_m = 0.0) const;

    const std::vector<core::KeyframeSubmap>& getSubmaps() const { return submaps_; }
    std::vector<core::KeyframeSubmap>& getMutableSubmaps() { return submaps_; }
    const std::vector<PoseGraphFactor>& getFactors() const { return factors_; }
    const std::vector<UnaryGlobalFactor>& getGlobalFactors() const { return global_factors_; }
    size_t numLoopClosures() const;
    size_t numGlobalFactors() const;
    void clear();

private:
    static math::Vec6d computeFactorResidual(const math::SE3d& T_w_i,
                                             const math::SE3d& T_w_j,
                                             const math::SE3d& T_i_j_meas);

    OptimizationReport optimizeExactCholeskyLM(int max_iterations);
    OptimizationReport optimizeGaussSeidelLM(int max_iterations);

    mutable std::mutex mutex_;
    std::vector<core::KeyframeSubmap> submaps_;
    std::unordered_map<uint64_t, size_t> id_to_index_;
    std::vector<PoseGraphFactor> factors_;
    std::vector<UnaryGlobalFactor> global_factors_;
};

}  // namespace backend
}  // namespace avscan
