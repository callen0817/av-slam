#pragma once

#include "avscan/filters/PointCloudFilters.hpp"
#include "avscan/sensors/SensorRigProfile.hpp"
#include "avscan/slam/AvImuPreintegrator.hpp"
#include "avscan/slam/AvSurfelMap.hpp"
#include "avscan/slam/ISlamEstimator.hpp"
#include <array>
#include <deque>
#include <string>
#include <vector>

namespace avscan {
namespace slam {

struct AvSlamConfig {
    // Hardware-agnostic sensor rig & noise profile
    sensors::GenericSensorRigConfig sensor_rig{sensors::GenericSensorRigConfig::presetRoboSenseAiry()};

    // Dual-resolution sufficient-statistics surfel map configuration
    AvSurfelMapConfig map_config{};

    // Real-time front-end point cloud conditioning filters
    filters::FrontEndFilterConfig front_end_filters{};

    // Factor-manifold MAP solver parameters
    int max_map_iterations{7};
    int sub_sweep_splits{1};                  // 1 = 10 Hz full sweep; 2..4 = 20..40 Hz Point-LIO-style micro-updates
    double initial_corr_dist_m{0.45};         // Wide capture basin on GNC iter 0..1
    double refined_corr_dist_m{0.22};         // Crisp convergence basin on GNC iter 2+
    double gnc_coarse_scale_m{0.12};          // Cauchy robust scale during coarse iterations
    double gnc_fine_scale_m{0.055};           // Cauchy robust scale during refinement iterations

    // Tightly-coupled 3-level direct photometric visual fusion parameters (fused inside 17-DOF Cholesky solve)
    int visual_pyramid_levels{3};
    double visual_photometric_sigma{12.0};    // Pixel intensity noise sigma (8-bit gray)
    double visual_huber_delta{25.0};          // Robust photometric threshold

    // Anisotropic subspace-projected degeneracy mitigation
    double degeneracy_eigenvalue_thresh{8.0};
    double degeneracy_prior_stiffness{18.0};  // Directional IMU prior stiffening along weak eigenvectors

    // Keyframe & visual settings
    double keyframe_translation_thresh_m{0.50};
    double keyframe_rotation_thresh_deg{22.0};
    double map_horizon_radius_m{45.0};
    bool enable_visual_fusion{false};
    bool enable_rgb_colorization{true};
    bool filter_dynamic_points_on_output{false};
};

// Detailed runtime telemetry specific to AV-SLAM (for diagnostics, benchmarking & funding demos)
struct AvSlamRuntimeStats {
    uint64_t total_frames{0};
    double avg_frame_ms{0.0};
    double avg_preint_deskew_ms{0.0};
    double avg_filter_ms{0.0};
    double avg_map_solve_ms{0.0};
    double avg_surfel_update_ms{0.0};
    size_t total_flying_pixels_removed{0};
    size_t planar_constraints_last_frame{0};
    size_t linear_constraints_last_frame{0};
    size_t fine_surfel_cells{0};
    size_t coarse_surfel_cells{0};
    size_t flagged_dynamic_cells{0};
    bool anisotropic_degeneracy_active{false};
    bool zupt_active_last_frame{false};
};

// ============================================================================
// AV-SLAM: Artificial Vision Proprietary Unified Factor-Manifold SLAM Engine
// ============================================================================
// 100% first-party, zero-GPL, zero-external-dependency C++17 SLAM core designed
// to power AV Scan iOS (iPhone/iPad), AV scanAR (Jetson), and AV Scan (Desktop)
// across any external 3D LiDAR + IMU (+ optional internal or external RGB camera).
//
// Core Proprietary Pillars:
//  1. On-Manifold Lie-Group IMU Preintegration (9x9 block-sparse covariance +
//     exact analytical bias Jacobians) with piecewise-linear twist deskewing.
//  2. Minimal 17-DOF Hybrid Manifold MAP Solver (SO(3) x R^12 x S^2 gravity)
//     solved via Cholesky LL^T (zero dense matrix inversions during iterations)
//     with Graduated Non-Convexity (GNC) Cauchy weighting.
//  3. Anisotropic Subspace-Projected Degeneracy Mitigation (directional IMU prior
//     stiffening strictly along weak geometric eigenvectors).
//  4. Dual-Resolution O(1) Sufficient-Statistics Surfel Map (Fine + Coarse tiers,
//     Westin Planar + Linear-Edge constraints, live free-space ray carving).
//  5. Integrated Real-Time Front-End Point Cloud Filter Pipeline (Range/Mask,
//     Ring Flying-Pixel Veil Rejection, Adaptive Range-Stratified Centroid Grid)
//     + Hybrid Terrestrial Flash-Station Anchor Injection.
class AvSlamEngine final : public ISlamEstimator {
public:
    explicit AvSlamEngine(const AvSlamConfig& config = AvSlamConfig{});
    ~AvSlamEngine() override = default;

    // Hardware-agnostic configuration API (works for Airy, Mid-360, Ouster, Hesai, ARKit, Generic)
    void setSensorRigConfig(const sensors::GenericSensorRigConfig& rig);
    const sensors::GenericSensorRigConfig& sensorRigConfig() const { return config_.sensor_rig; }
    void setFrontEndFilterConfig(const filters::FrontEndFilterConfig& filter_cfg);

    // ISlamEstimator interface (drop-in compatibility with AVScanCoreBridge & all AV apps)
    void setFactoryCalibration(const core::AiryFactoryCalibration& calib) override;
    void setExtrinsicCalibration(const core::ExtrinsicCalibrationProfile& profile) override;
    void setVisualFusionEnabled(bool enabled) override { config_.enable_visual_fusion = enabled; }

    bool processBundle(const sync::SyncBundle& bundle,
                       core::LidarFrame& out_deskewed_body_frame,
                       core::LidarFrame& out_world_frame,
                       EstimatorMetrics& out_metrics) override;

    core::SlamState getCurrentState() const override { return state_; }
    void resetPose(const math::SE3d& new_T_world_body) override;
    void reset() override;

    std::string getLicenseIdentifier() const override {
        return "AV-SLAM Proprietary Engine v1.0 (Artificial Vision Inc. — 100% First-Party C++17, Zero GPL)";
    }

    // Hybrid Mobile + Terrestrial Flash-Station anchor injection
    void injectTerrestrialAnchor(const math::SE3d& T_world_anchor,
                                 const std::vector<core::LidarPointXYZIRT>& anchor_local_points);

    const AvSurfelMap& getSurfelMap() const { return surfel_map_; }
    const std::vector<core::SlamState>& getTrajectory() const { return trajectory_; }
    AvSlamRuntimeStats getRuntimeStats() const { return stats_; }

private:
    static constexpr int kStateDim = 17;  // [rot(0..2), pos(3..5), vel(6..8), bg(9..11), ba(12..14), grav_S2(15..16)]

    struct Mat17 {
        double m[kStateDim][kStateDim]{};
        static Mat17 Zero() { return Mat17{}; }
        static Mat17 Identity() {
            Mat17 r;
            for (int i = 0; i < kStateDim; ++i) r.m[i][i] = 1.0;
            return r;
        }
        double& operator()(int r, int c) { return m[r][c]; }
        double operator()(int r, int c) const { return m[r][c]; }
    };

    struct GravityBasis {
        math::Vec3d b1{1.0, 0.0, 0.0};
        math::Vec3d b2{0.0, 1.0, 0.0};
    };

    // Compact 3-level grayscale pyramid for coarse-to-fine tightly-coupled 8x8 direct photometric fusion
    struct GrayLevel {
        int width{0};
        int height{0};
        double scale{1.0};
        std::vector<float> pixels;
    };
    struct ImagePyramid {
        std::array<GrayLevel, 3> levels{};
        int num_levels{0};
        core::CameraIntrinsics base_intrinsics{};
        double exposure_sec{0.008};
    };

    bool initializeImuGravity(const std::vector<core::ImuMeasurement>& imus);

    // Single per-sweep 17x17 covariance propagation from preintegrated factor
    void propagatePriorCovariance(const core::SlamState& state_prev,
                                  const PreintegratedImuFactor& preint);

    // 17-DOF Factor-Manifold Cholesky MAP optimization with GNC + anisotropic degeneracy mitigation
    // + optional tightly-coupled 3-level pyramid direct visual photometric factors
    void solveJointMapOptimization(const std::vector<math::Vec3d>& feats_body,
                                   const PreintegratedImuFactor& preint,
                                   const ImagePyramid* visual_pyramid,
                                   const std::vector<VisualPatch>* visual_patches,
                                   EstimatorMetrics& io_metrics);

    static ImagePyramid buildGrayPyramid(const core::RgbFrame& rgb_frame, int max_levels = 3);
    static float samplePyramidBilinear(const GrayLevel& lvl, double u, double v);

    // Internal single-bundle (or single sub-sweep) processing step
    bool processSingleSweep(const sync::SyncBundle& bundle,
                            core::LidarFrame& out_deskewed_body_frame,
                            core::LidarFrame& out_world_frame,
                            EstimatorMetrics& out_metrics);

    // SLERP-interpolated depth-buffered bilinear RGB colorization + visual patch extraction
    void colorizeAndExtractVisualPatches(const core::RgbFrame& rgb_frame,
                                         const std::vector<MotionKnot>& corrected_knots,
                                         core::LidarFrame& io_deskewed_body,
                                         core::LidarFrame& io_world_frame);

    math::SE3d interpolateBodyPoseAtTime(double target_time_sec,
                                         const std::vector<MotionKnot>& sweep_knots) const;

    // Exact S^2 gravity manifold helpers (preserves ||g|| == 9.80665 m/s^2 by construction)
    static GravityBasis computeGravityTangentBasis(const math::Vec3d& g);
    static math::Vec3d gravityBoxPlus(const math::Vec3d& g, const GravityBasis& basis, double dg0, double dg1);
    static math::Vec2d gravityBoxMinus(const math::Vec3d& g_target, const math::Vec3d& g_base, const GravityBasis& basis);

    // Fast Cholesky LL^T linear algebra on symmetric positive-definite 17x17 matrices
    static bool choleskyDecompose(const Mat17& A, Mat17& out_L);
    static void choleskySolve(const Mat17& L, const double b[kStateDim], double x[kStateDim]);
    static bool choleskyInverseFromL(const Mat17& L, Mat17& out_inv);
    static bool solve6x6CholeskyOrGauss(double A[6][6], const double b[6], double x[6]);

    static float sampleBilinearGray(const core::RgbFrame& img, double u, double v);
    static void sampleBilinearRgb(const core::RgbFrame& img, double u, double v,
                                  uint8_t& r, uint8_t& g, uint8_t& b);

    AvSlamConfig config_;
    AvImuPreintegrator preintegrator_;
    AvSurfelMap surfel_map_;

    core::SlamState state_{};
    Mat17 P_{};  // 17x17 manifold state covariance

    std::vector<core::ImuMeasurement> init_imu_accum_;
    core::ImuMeasurement last_imu_{};
    bool has_last_imu_{false};

    std::vector<core::SlamState> trajectory_;
    std::deque<std::pair<double, math::SE3d>> recent_pose_history_;
    math::SE3d last_keyframe_pose_{math::SE3d::Identity()};
    bool has_first_keyframe_{false};
    AvSlamRuntimeStats stats_{};
};

}  // namespace slam
}  // namespace avscan
