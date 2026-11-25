#include "avscan/slam/AvSurfelMap.hpp"
#include <algorithm>
#include <cmath>

namespace avscan {
namespace slam {

AvSurfelMap::AvSurfelMap(const AvSurfelMapConfig& cfg) {
    setConfig(cfg);
}

void AvSurfelMap::setConfig(const AvSurfelMapConfig& cfg) {
    cfg_ = cfg;
    cfg_.fine_voxel_m = std::max(0.08, cfg_.fine_voxel_m);
    cfg_.coarse_voxel_m = std::max(cfg_.fine_voxel_m * 1.5, cfg_.coarse_voxel_m);
    inv_fine_ = 1.0 / cfg_.fine_voxel_m;
    inv_coarse_ = 1.0 / cfg_.coarse_voxel_m;
}

VoxelKey AvSurfelMap::toKey(const math::Vec3d& p, double inv_voxel) {
    return {
        static_cast<int64_t>(std::floor(p.x * inv_voxel)),
        static_cast<int64_t>(std::floor(p.y * inv_voxel)),
        static_cast<int64_t>(std::floor(p.z * inv_voxel))
    };
}

void AvSurfelMap::addPointToCell(SurfelCell& cell,
                                 const math::Vec3d& p,
                                 double weight,
                                 double max_w,
                                 uint64_t frame_id,
                                 bool mark_as_anchor) {
    cell.last_access_frame = frame_id;
    cell.hit_count++;
    if (mark_as_anchor) {
        cell.is_anchor = true;
    }

    // Exponential moving cap preserves crisp surfels when revisiting areas without freezing forever
    if (cell.weight_sum >= max_w && !cell.is_anchor) {
        const double decay = (max_w - weight) / max_w;
        if (decay > 0.5) {
            cell.sx *= decay;  cell.sy *= decay;  cell.sz *= decay;
            cell.sxx *= decay; cell.sxy *= decay; cell.sxz *= decay;
            cell.syy *= decay; cell.syz *= decay; cell.szz *= decay;
            cell.weight_sum *= decay;
        } else {
            return;
        }
    }

    cell.sx  += weight * p.x;
    cell.sy  += weight * p.y;
    cell.sz  += weight * p.z;
    cell.sxx += weight * p.x * p.x;
    cell.sxy += weight * p.x * p.y;
    cell.sxz += weight * p.x * p.z;
    cell.syy += weight * p.y * p.y;
    cell.syz += weight * p.y * p.z;
    cell.szz += weight * p.z * p.z;
    cell.weight_sum += weight;
    cell.dirty = true;
}

void AvSurfelMap::recomputeSurfelGeometry(SurfelCell& cell) const {
    cell.dirty = false;
    if (cell.weight_sum < 1e-6) {
        cell.geom_type = SurfelGeometryType::UNRESOLVED;
        return;
    }

    const double inv_w = 1.0 / cell.weight_sum;
    cell.mean = math::Vec3d(cell.sx * inv_w, cell.sy * inv_w, cell.sz * inv_w);

    if (cell.hit_count < 5) {
        cell.geom_type = SurfelGeometryType::UNRESOLVED;
        return;
    }

    math::Mat3d cov;
    cov(0, 0) = std::max(0.0, cell.sxx * inv_w - cell.mean.x * cell.mean.x);
    cov(0, 1) = cov(1, 0) = cell.sxy * inv_w - cell.mean.x * cell.mean.y;
    cov(0, 2) = cov(2, 0) = cell.sxz * inv_w - cell.mean.x * cell.mean.z;
    cov(1, 1) = std::max(0.0, cell.syy * inv_w - cell.mean.y * cell.mean.y);
    cov(1, 2) = cov(2, 1) = cell.syz * inv_w - cell.mean.y * cell.mean.z;
    cov(2, 2) = std::max(0.0, cell.szz * inv_w - cell.mean.z * cell.mean.z);

    const math::SymEigen3d eig = math::SymEigen3d::compute(cov, 20);
    const double l0 = std::max(0.0, eig.eigenvalues[0]);
    const double l1 = std::max(0.0, eig.eigenvalues[1]);
    const double l2 = std::max(1e-9, eig.eigenvalues[2]);

    cell.lambda0 = l0;
    cell.lambda1 = l1;
    cell.lambda2 = l2;
    cell.normal = math::Vec3d(eig.eigenvectors(0, 0), eig.eigenvectors(1, 0), eig.eigenvectors(2, 0)).normalized();
    cell.principal_axis = math::Vec3d(eig.eigenvectors(0, 2), eig.eigenvectors(1, 2), eig.eigenvectors(2, 2)).normalized();

    // Westin shape descriptors
    cell.planarity = std::clamp((l1 - l0) / l2, 0.0, 1.0);
    cell.linearity = std::clamp((l2 - l1) / l2, 0.0, 1.0);

    if (l1 > 1.2e-4 && l0 < 0.10 * l1 && l0 < cfg_.max_plane_thickness_var_m2) {
        cell.geom_type = SurfelGeometryType::PLANAR;
    } else if (cfg_.enable_linear_edge_constraints && l2 > 4e-4 && cell.linearity > 0.72 && l1 < 0.008) {
        cell.geom_type = SurfelGeometryType::LINEAR;
    } else {
        cell.geom_type = SurfelGeometryType::SCATTER;
    }
}

void AvSurfelMap::insertWorldPoints(const std::vector<math::Vec3d>& world_points,
                                    uint64_t frame_id,
                                    double point_weight,
                                    bool mark_as_anchor) {
    if (world_points.empty()) return;

    std::vector<VoxelKey> dirty_fine;
    std::vector<VoxelKey> dirty_coarse;
    dirty_fine.reserve(std::min<size_t>(world_points.size(), 4096));
    dirty_coarse.reserve(std::min<size_t>(world_points.size(), 2048));

    for (const auto& pw : world_points) {
        const VoxelKey kf = toKey(pw, inv_fine_);
        auto& cf = fine_grid_[kf];
        const bool was_dirty_f = cf.dirty;
        addPointToCell(cf, pw, point_weight, cfg_.max_weight_per_cell, frame_id, mark_as_anchor);
        if (!was_dirty_f && cf.dirty) dirty_fine.push_back(kf);

        const VoxelKey kc = toKey(pw, inv_coarse_);
        auto& cc = coarse_grid_[kc];
        const bool was_dirty_c = cc.dirty;
        addPointToCell(cc, pw, point_weight, cfg_.max_weight_per_cell * 1.5, frame_id, mark_as_anchor);
        if (!was_dirty_c && cc.dirty) dirty_coarse.push_back(kc);
    }

    for (const auto& k : dirty_fine) {
        auto it = fine_grid_.find(k);
        if (it != fine_grid_.end() && it->second.dirty) {
            recomputeSurfelGeometry(it->second);
        }
    }
    for (const auto& k : dirty_coarse) {
        auto it = coarse_grid_.find(k);
        if (it != coarse_grid_.end() && it->second.dirty) {
            recomputeSurfelGeometry(it->second);
        }
    }
}

void AvSurfelMap::carveFreeSpaceRays(const math::Vec3d& sensor_origin_world,
                                     const std::vector<math::Vec3d>& world_points,
                                     size_t ray_stride) {
    if (!cfg_.enable_live_free_space_carving || fine_grid_.empty() || world_points.empty()) {
        return;
    }

    const size_t step = std::max<size_t>(1, ray_stride);
    const double min_ray_len = cfg_.fine_voxel_m * 4.0;
    const double safe_margin_m = cfg_.fine_voxel_m * 1.6;

    for (size_t i = 0; i < world_points.size(); i += step) {
        const math::Vec3d ray = world_points[i] - sensor_origin_world;
        const double len = ray.norm();
        if (len < min_ray_len) continue;

        const VoxelKey hit_k = toKey(world_points[i], inv_fine_);
        // Sample two interior visibility checkpoints along the clear line-of-sight well short of the hit surface
        const double fracs[2] = {0.42, 0.72};
        for (double frac : fracs) {
            const double dist_along = len * frac;
            if ((len - dist_along) < safe_margin_m) continue;
            const math::Vec3d sample_p = sensor_origin_world + ray * frac;
            const VoxelKey sk = toKey(sample_p, inv_fine_);
            if (sk == hit_k) continue;
            auto it = fine_grid_.find(sk);
            if (it != fine_grid_.end() && !it->second.is_anchor) {
                // Only count free-space pass if the ray passes close to the cell's occupied centroid
                // (avoids false carving of grazing walls inside a coarse corner voxel)
                if ((it->second.mean - sample_p).squaredNorm() < (cfg_.fine_voxel_m * 0.45) * (cfg_.fine_voxel_m * 0.45)) {
                    it->second.free_space_passes++;
                }
            }
        }
    }
}

bool AvSurfelMap::queryLayer(const GridMap& grid,
                             double voxel_size_m,
                             double inv_voxel,
                             bool is_fine_tier,
                             const math::Vec3d& p_body,
                             const math::Vec3d& p_world,
                             double max_dist_m,
                             SurfelConstraint& out_constraint) const {
    const VoxelKey base_k = toKey(p_world, inv_voxel);
    static constexpr int64_t kOff7[7][3] = {
        {0, 0, 0},
        {1, 0, 0}, {-1, 0, 0},
        {0, 1, 0}, {0, -1, 0},
        {0, 0, 1}, {0, 0, -1}
    };

    const double max_center_dist2 = (voxel_size_m * 1.70) * (voxel_size_m * 1.70);
    double best_score = max_dist_m;
    const SurfelCell* best_cell = nullptr;
    math::Vec3d best_normal(0.0, 0.0, 1.0);
    double best_signed_dist = 0.0;
    SurfelGeometryType best_type = SurfelGeometryType::UNRESOLVED;

    for (int i = 0; i < 7; ++i) {
        const VoxelKey nk{base_k.x + kOff7[i][0], base_k.y + kOff7[i][1], base_k.z + kOff7[i][2]};
        auto it = grid.find(nk);
        if (it == grid.end()) continue;
        const SurfelCell& cell = it->second;
        if (cell.geom_type != SurfelGeometryType::PLANAR && cell.geom_type != SurfelGeometryType::LINEAR) {
            continue;
        }
        if (cell.isLikelyDynamic()) continue;

        const math::Vec3d diff = p_world - cell.mean;
        if (diff.squaredNorm() > max_center_dist2) continue;

        if (cell.geom_type == SurfelGeometryType::PLANAR) {
            const double signed_d = cell.normal.dot(diff);
            const double abs_d = std::abs(signed_d);
            if (abs_d < best_score) {
                best_score = abs_d;
                best_signed_dist = signed_d;
                best_normal = cell.normal;
                best_cell = &cell;
                best_type = SurfelGeometryType::PLANAR;
                // Early exit on ultra-crisp center-voxel planar match (huge speedup!)
                if (i == 0 && abs_d < 0.03 && cell.planarity > 0.70) break;
            }
        } else if (cell.geom_type == SurfelGeometryType::LINEAR && best_type != SurfelGeometryType::PLANAR) {
            // Point-to-line perpendicular vector: d_perp = diff - (diff . axis) * axis
            const math::Vec3d d_perp = diff - cell.principal_axis * diff.dot(cell.principal_axis);
            const double dist = d_perp.norm();
            if (dist > 1e-5 && dist < best_score * 0.85) {
                best_score = dist;
                best_signed_dist = dist;
                best_normal = d_perp / dist;
                best_cell = &cell;
                best_type = SurfelGeometryType::LINEAR;
            }
        }
    }

    if (!best_cell) return false;

    out_constraint.valid = true;
    out_constraint.type = best_type;
    out_constraint.is_fine_tier = is_fine_tier;
    out_constraint.point_body = p_body;
    out_constraint.point_world = p_world;
    out_constraint.surfel_mean = best_cell->mean;
    out_constraint.residual_normal = best_normal;
    out_constraint.signed_distance = best_signed_dist;

    const double range_sq = p_body.squaredNorm();
    const double base_var = cfg_.range_noise_sigma_m * cfg_.range_noise_sigma_m;
    const double geom_var = (best_type == SurfelGeometryType::PLANAR) ? best_cell->lambda0 : best_cell->lambda1;
    out_constraint.measurement_variance = std::max(1.5e-4, base_var + geom_var + 8e-6 * range_sq);

    double q_weight = best_cell->staticConfidence();
    if (best_type == SurfelGeometryType::PLANAR) {
        q_weight *= (0.65 + 0.35 * best_cell->planarity);
    } else {
        q_weight *= 0.45 * best_cell->linearity;  // Linear edges assist without dominating planar walls
    }
    if (!is_fine_tier) q_weight *= 0.70;
    if (best_cell->is_anchor) q_weight *= 1.80;   // Terrestrial flash stations act as high-rigidity anchors
    out_constraint.quality_weight = q_weight;
    return true;
}

bool AvSurfelMap::queryGeometricConstraint(const math::Vec3d& p_body,
                                           const math::Vec3d& p_world,
                                           double max_dist_m,
                                           bool prefer_fine_only,
                                           SurfelConstraint& out_constraint) const {
    out_constraint.valid = false;
    if (queryLayer(fine_grid_, cfg_.fine_voxel_m, inv_fine_, true, p_body, p_world, max_dist_m, out_constraint)) {
        return true;
    }
    if (!prefer_fine_only) {
        return queryLayer(coarse_grid_, cfg_.coarse_voxel_m, inv_coarse_, false, p_body, p_world, max_dist_m, out_constraint);
    }
    return false;
}

void AvSurfelMap::updateVisualPatch(const math::Vec3d& p_world, const VisualPatch& patch) {
    auto it = fine_grid_.find(toKey(p_world, inv_fine_));
    if (it != fine_grid_.end()) {
        it->second.visual_patch = patch;
        it->second.visual_patch.valid = true;
    }
}

std::vector<VisualPatch> AvSurfelMap::getActiveVisualPatches(const math::Vec3d& sensor_pos_world,
                                                             double max_radius_m) const {
    std::vector<VisualPatch> patches;
    const double r2 = max_radius_m * max_radius_m;
    for (const auto& kv : fine_grid_) {
        if (kv.second.visual_patch.valid && !kv.second.isLikelyDynamic()) {
            if ((kv.second.visual_patch.center_world - sensor_pos_world).squaredNorm() <= r2) {
                patches.push_back(kv.second.visual_patch);
            }
        }
    }
    return patches;
}

size_t AvSurfelMap::pruneHorizon(const math::Vec3d& center_world, double max_radius_m, uint64_t current_frame_id) {
    const double r2 = max_radius_m * max_radius_m;
    size_t removed = 0;

    auto pruneGrid = [&](GridMap& grid, size_t max_cells) {
        const bool over_budget = grid.size() > max_cells;
        for (auto it = grid.begin(); it != grid.end(); ) {
            if (it->second.is_anchor) {
                ++it;
                continue;
            }
            const bool too_far = (it->second.hit_count > 0) &&
                                 ((it->second.mean - center_world).squaredNorm() > r2);
            const bool too_old = over_budget && (current_frame_id > it->second.last_access_frame + 120);
            if (too_far || too_old) {
                it = grid.erase(it);
                removed++;
            } else {
                ++it;
            }
        }
    };

    pruneGrid(fine_grid_, cfg_.max_fine_cells);
    pruneGrid(coarse_grid_, cfg_.max_coarse_cells);
    return removed;
}

size_t AvSurfelMap::numValidPlanarSurfels() const {
    size_t count = 0;
    for (const auto& kv : fine_grid_) {
        if (kv.second.geom_type == SurfelGeometryType::PLANAR) count++;
    }
    for (const auto& kv : coarse_grid_) {
        if (kv.second.geom_type == SurfelGeometryType::PLANAR) count++;
    }
    return count;
}

size_t AvSurfelMap::numValidLinearSurfels() const {
    size_t count = 0;
    for (const auto& kv : fine_grid_) {
        if (kv.second.geom_type == SurfelGeometryType::LINEAR) count++;
    }
    return count;
}

size_t AvSurfelMap::numFlaggedDynamicCells() const {
    size_t count = 0;
    for (const auto& kv : fine_grid_) {
        if (kv.second.isLikelyDynamic()) count++;
    }
    return count;
}

bool AvSurfelMap::isPointDynamic(const math::Vec3d& p_world) const {
    auto it = fine_grid_.find(toKey(p_world, inv_fine_));
    return (it != fine_grid_.end()) && it->second.isLikelyDynamic();
}

void AvSurfelMap::clear() {
    fine_grid_.clear();
    coarse_grid_.clear();
}

}  // namespace slam
}  // namespace avscan
