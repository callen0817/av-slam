#pragma once

#include "avscan/core/Types.hpp"
#include "avscan/slam/VoxelHashMap.hpp"
#include <array>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace avscan {
namespace slam {

// Westin eigenvalue surface classification for each O(1) sufficient-statistics surfel cell
enum class SurfelGeometryType : uint8_t {
    UNRESOLVED = 0,
    PLANAR = 1,       // Crisp architectural wall / floor / ceiling / desk plane
    LINEAR = 2,       // Sharp edge / pillar / doorframe / railing line
    SCATTER = 3       // Foliage / clutter / isotropic scatter (excluded from pose registration)
};

// Compact O(1) sufficient-statistics surfel node (no per-cell raw point array or O(N) rebuilds).
struct SurfelCell {
    // Incremental sufficient statistics: sum of points and symmetric upper-triangle outer products
    double sx{0.0}, sy{0.0}, sz{0.0};
    double sxx{0.0}, sxy{0.0}, sxz{0.0}, syy{0.0}, syz{0.0}, szz{0.0};
    double weight_sum{0.0};

    // Cached spectral geometry (recomputed lazily when dirty)
    math::Vec3d mean{0.0, 0.0, 0.0};
    math::Vec3d normal{0.0, 0.0, 1.0};        // Smallest eigenvector v0 (plane normal)
    math::Vec3d principal_axis{1.0, 0.0, 0.0}; // Largest eigenvector v2 (line direction)
    double lambda0{1.0};
    double lambda1{1.0};
    double lambda2{1.0};
    double planarity{0.0};
    double linearity{0.0};
    SurfelGeometryType geom_type{SurfelGeometryType::UNRESOLVED};

    // Live visibility & dynamic ghost statistics
    uint64_t last_access_frame{0};
    uint32_t hit_count{0};
    uint32_t free_space_passes{0};
    bool is_anchor{false};                    // Locked terrestrial flash-station anchor surfel
    bool dirty{false};

    // Optional attached 8x8 photometric patch for tightly-coupled visual updates
    VisualPatch visual_patch{};

    // Static persistence confidence in (0, 1]; drops rapidly for moving objects seen through later
    double staticConfidence() const {
        if (is_anchor) return 1.0;
        const double h = static_cast<double>(hit_count) + 3.0;
        const double f = static_cast<double>(free_space_passes);
        return h / (h + f);
    }

    bool isLikelyDynamic() const {
        return !is_anchor && (free_space_passes >= 4) && (free_space_passes > hit_count * 2);
    }
};

// Geometric constraint returned by AvSurfelMap (supports both Point-to-Plane and Point-to-Line)
struct SurfelConstraint {
    SurfelGeometryType type{SurfelGeometryType::UNRESOLVED};
    math::Vec3d point_body{0.0, 0.0, 0.0};
    math::Vec3d point_world{0.0, 0.0, 0.0};
    math::Vec3d surfel_mean{0.0, 0.0, 0.0};
    math::Vec3d residual_normal{0.0, 0.0, 1.0}; // Unit Jacobian normal (plane normal OR perp-to-edge unit vector)
    double signed_distance{0.0};
    double measurement_variance{0.001};
    double quality_weight{1.0};                // Combines planarity/linearity, static confidence, fine/coarse tier
    bool is_fine_tier{true};
    bool valid{false};
};

struct AvSurfelMapConfig {
    double fine_voxel_m{0.38};                // High-detail surfel grid (walls, doors, furniture)
    double coarse_voxel_m{0.76};              // Wide-basin fallback surfel grid (fast motion & open areas)
    double max_weight_per_cell{64.0};         // Exponential saturation cap keeping surfels crisp
    size_t max_fine_cells{70000};
    size_t max_coarse_cells{35000};
    double range_noise_sigma_m{0.015};
    double max_plane_thickness_var_m2{0.0045};
    bool enable_linear_edge_constraints{true};
    bool enable_live_free_space_carving{true};
};

// Proprietary Dual-Resolution Sufficient-Statistics Surfel Map for AV-SLAM.
// Distinct from FAST-LIVO2's ikd-Tree and VoxelHashMap:
//  - Zero per-cell point list allocations (O(1) sufficient-statistics update)
//  - Dual-resolution fine + coarse surfel hierarchy
//  - Westin planar + linear-edge geometric constraints
//  - Active ray-visibility free-space carving for live dynamic ghost suppression
//  - First-class terrestrial anchor injection for hybrid mobile+terrestrial mapping
class AvSurfelMap {
public:
    explicit AvSurfelMap(const AvSurfelMapConfig& cfg = AvSurfelMapConfig{});

    void setConfig(const AvSurfelMapConfig& cfg);
    const AvSurfelMapConfig& config() const { return cfg_; }

    // Inserts registered world points in O(N) into both fine & coarse sufficient-statistics layers
    void insertWorldPoints(const std::vector<math::Vec3d>& world_points,
                           uint64_t frame_id,
                           double point_weight = 1.0,
                           bool mark_as_anchor = false);

    // Live free-space visibility carving along rays from sensor_origin_world to observed hits
    void carveFreeSpaceRays(const math::Vec3d& sensor_origin_world,
                            const std::vector<math::Vec3d>& world_points,
                            size_t ray_stride = 4);

    // Queries best geometric surfel constraint (Fine tier first -> Coarse fallback; Planar preferred -> Linear edge)
    bool queryGeometricConstraint(const math::Vec3d& p_body,
                                  const math::Vec3d& p_world,
                                  double max_dist_m,
                                  bool prefer_fine_only,
                                  SurfelConstraint& out_constraint) const;

    // Attaches or updates an 8x8 photometric patch in the fine surfel tier
    void updateVisualPatch(const math::Vec3d& p_world, const VisualPatch& patch);
    std::vector<VisualPatch> getActiveVisualPatches(const math::Vec3d& sensor_pos_world, double max_radius_m) const;

    // Evicts distant or stale surfels outside active horizon to strictly respect mobile RAM limits
    size_t pruneHorizon(const math::Vec3d& center_world, double max_radius_m, uint64_t current_frame_id);

    size_t numFineCells() const { return fine_grid_.size(); }
    size_t numCoarseCells() const { return coarse_grid_.size(); }
    size_t numValidPlanarSurfels() const;
    size_t numValidLinearSurfels() const;
    size_t numFlaggedDynamicCells() const;

    // Checks whether a world point falls in a surfel flagged as dynamic/transient
    bool isPointDynamic(const math::Vec3d& p_world) const;

    void clear();

private:
    using GridMap = std::unordered_map<VoxelKey, SurfelCell, VoxelKeyHash>;

    static VoxelKey toKey(const math::Vec3d& p, double inv_voxel);
    static void addPointToCell(SurfelCell& cell, const math::Vec3d& p, double weight, double max_w,
                               uint64_t frame_id, bool mark_as_anchor);
    void recomputeSurfelGeometry(SurfelCell& cell) const;

    bool queryLayer(const GridMap& grid,
                    double voxel_size_m,
                    double inv_voxel,
                    bool is_fine_tier,
                    const math::Vec3d& p_body,
                    const math::Vec3d& p_world,
                    double max_dist_m,
                    SurfelConstraint& out_constraint) const;

    AvSurfelMapConfig cfg_;
    double inv_fine_{1.0 / 0.32};
    double inv_coarse_{1.0 / 0.68};
    GridMap fine_grid_;
    GridMap coarse_grid_;
};

}  // namespace slam
}  // namespace avscan
