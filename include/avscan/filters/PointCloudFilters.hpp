#pragma once

#include "avscan/core/Types.hpp"
#include "avscan/sensors/SensorRigProfile.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace avscan {
namespace filters {

// Configuration for AV-SLAM real-time front-end scan conditioning filters
struct FrontEndFilterConfig {
    bool enable_range_and_mask_filter{true};
    double min_range_m{0.40};
    double max_range_m{60.0};
    bool enable_self_mask_box{false};
    math::Vec3d self_mask_min_m{-0.25, -0.25, -0.25};
    math::Vec3d self_mask_max_m{0.25, 0.25, 0.25};

    // Mixed-pixel / flying-pixel veil filter along beam rings
    bool enable_flying_pixel_filter{true};
    double flying_pixel_jump_ratio{0.18};     // Relative range jump threshold vs neighbor rays
    double flying_pixel_min_jump_m{0.35};     // Absolute range jump floor (m)

    // Adaptive range-stratified centroid voxel downsampling for odometry features
    bool enable_adaptive_voxel_downsample{true};
    double near_voxel_m{0.08};                // Voxel size for range <= near_range_cutoff_m
    double mid_voxel_m{0.12};                 // Voxel size for near..far
    double far_voxel_m{0.20};                 // Voxel size for range > far_range_cutoff_m
    double near_range_cutoff_m{6.0};
    double far_range_cutoff_m{18.0};
    size_t max_output_features{3600};         // Soft cap to keep mobile CPU frame budget bounded
};

// Configuration for AV-SLAM post-registration / export map refinement filters
struct MapRefinementFilterConfig {
    // Sub-voxel centroid + confidence-weighted RGB fusion
    double fusion_voxel_m{0.015};             // 1.5 cm final unified map resolution

    // Fast spatial-hash Radius Outlier Removal (removes airborne dust / isolated glint specks)
    bool enable_radius_outlier_removal{true};
    double ror_cell_size_m{0.08};
    uint32_t ror_min_neighbors{3};

    // Fast spatial-hash Statistical Outlier Removal (SOR without KD-tree overhead)
    bool enable_statistical_outlier_removal{true};
    double sor_cell_size_m{0.12};
    double sor_std_multiplier{2.2};

    // Edge-preserving surfel-guided surface denoising (tightens planar walls/floors/ceilings)
    bool enable_surface_denoising{true};
    double denoise_voxel_m{0.20};
    double denoise_planarity_min{0.78};       // Only snap high-planarity surfaces (preserve corners/edges)
    double denoise_max_thickness_var{0.003};  // Max lambda_0 (m^2)
    double denoise_projection_alpha{0.55};    // Blend factor [0..1] toward local plane
};

struct FilterPipelineStats {
    size_t input_points{0};
    size_t rejected_range_or_nan{0};
    size_t rejected_self_mask{0};
    size_t rejected_flying_pixels{0};
    size_t rejected_ror{0};
    size_t rejected_sor{0};
    size_t denoised_planar_points{0};
    size_t output_points{0};
};

// Proprietary AV-SLAM zero-dependency C++17 Point Cloud Conditioning & Refinement Suite.
// Designed for O(N) spatial-hash execution on Apple Silicon (iOS), NVIDIA Jetson, and Desktop.
class PointCloudFilters {
public:
    // Stage 1A: Sanitize NaN/Inf, enforce spherical range envelope and optional rig self-exclusion box
    static std::vector<core::LidarPointXYZIRT> filterRangeAndSelfMask(
        const std::vector<core::LidarPointXYZIRT>& input,
        const FrontEndFilterConfig& cfg,
        FilterPipelineStats* out_stats = nullptr);

    // Stage 1B: Ring-ordered mixed-pixel ("flying pixel" / edge-veil) rejection
    // Detects range-discontinuity ghosts along consecutive laser firings on the same channel.
    static std::vector<core::LidarPointXYZIRT> filterFlyingPixels(
        const std::vector<core::LidarPointXYZIRT>& input,
        double jump_ratio = 0.18,
        double min_jump_m = 0.35,
        size_t* out_rejected_count = nullptr);

    // Stage 1C: Range-stratified adaptive voxel centroid downsampling (returns body-frame centroids)
    static std::vector<math::Vec3d> adaptiveRangeVoxelDownsample(
        const std::vector<core::LidarPointXYZIRT>& input,
        const FrontEndFilterConfig& cfg);

    // Combined real-time front-end conditioning pass (range + self-mask + flying-pixel + adaptive features)
    static void conditionFrontEndScan(
        const std::vector<core::LidarPointXYZIRT>& raw_body_points,
        const FrontEndFilterConfig& cfg,
        std::vector<core::LidarPointXYZIRT>& out_clean_points,
        std::vector<math::Vec3d>& out_odometry_features,
        FilterPipelineStats* out_stats = nullptr);

    // Stage 2A: Sub-voxel centroid averaging + confidence-weighted RGB fusion across observations
    static std::vector<core::LidarPointXYZIRT> fuseVoxelCentroidsAndRgb(
        const std::vector<core::LidarPointXYZIRT>& world_points,
        double voxel_size_m);

    // Stage 2B: Fast O(N) spatial-hash Radius Outlier Removal (27-cell neighborhood density gate)
    static std::vector<core::LidarPointXYZIRT> radiusOutlierRemoval(
        const std::vector<core::LidarPointXYZIRT>& input,
        double cell_size_m,
        uint32_t min_neighbors,
        size_t* out_removed_count = nullptr);

    // Stage 2C: Fast O(N) spatial-hash Statistical Outlier Removal (local cell centroid distance Z-score)
    static std::vector<core::LidarPointXYZIRT> statisticalOutlierRemoval(
        const std::vector<core::LidarPointXYZIRT>& input,
        double cell_size_m,
        double std_multiplier,
        size_t* out_removed_count = nullptr);

    // Stage 2D: Edge-preserving surfel-guided planar projection denoising
    static std::vector<core::LidarPointXYZIRT> surfelGuidedSurfaceDenoise(
        const std::vector<core::LidarPointXYZIRT>& input,
        double voxel_size_m,
        double min_planarity,
        double max_thickness_var,
        double projection_alpha,
        size_t* out_snapped_count = nullptr);

    // Complete post-registration map refinement pipeline (Fusion -> ROR -> SOR -> Surfel Denoise)
    static std::vector<core::LidarPointXYZIRT> runFullMapRefinement(
        const std::vector<core::LidarPointXYZIRT>& raw_world_points,
        const MapRefinementFilterConfig& cfg,
        FilterPipelineStats* out_stats = nullptr);
};

}  // namespace filters
}  // namespace avscan
