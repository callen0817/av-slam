#include "avscan/filters/PointCloudFilters.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_map>

namespace avscan {
namespace filters {

namespace {

struct GridKey {
    int64_t x{0}, y{0}, z{0};
    bool operator==(const GridKey& o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct GridKeyHash {
    size_t operator()(const GridKey& k) const {
        uint64_t h1 = static_cast<uint64_t>(k.x) * 73856093ULL;
        uint64_t h2 = static_cast<uint64_t>(k.y) * 19349663ULL;
        uint64_t h3 = static_cast<uint64_t>(k.z) * 83492791ULL;
        return static_cast<size_t>(h1 ^ h2 ^ h3);
    }
};

inline GridKey makeKey(double x, double y, double z, double inv_cell) {
    return {
        static_cast<int64_t>(std::floor(x * inv_cell)),
        static_cast<int64_t>(std::floor(y * inv_cell)),
        static_cast<int64_t>(std::floor(z * inv_cell))
    };
}

// Fast 64-bit packed spatial key (20 bits signed per axis + 2 bits tier) — zero struct hashing overhead
inline uint64_t packTierVoxelKey(int64_t ix, int64_t iy, int64_t iz, uint64_t tier) {
    constexpr int64_t kBias = (1LL << 19);
    const uint64_t ux = static_cast<uint64_t>(ix + kBias) & 0xFFFFFULL;
    const uint64_t uy = static_cast<uint64_t>(iy + kBias) & 0xFFFFFULL;
    const uint64_t uz = static_cast<uint64_t>(iz + kBias) & 0xFFFFFULL;
    return (tier << 60) | (ux << 40) | (uy << 20) | uz;
}

struct FastU64Hash {
    size_t operator()(uint64_t x) const noexcept {
        x ^= x >> 33;
        x *= 0xff51afd7ed558ccdULL;
        x ^= x >> 33;
        return static_cast<size_t>(x);
    }
};

}  // namespace

std::vector<core::LidarPointXYZIRT> PointCloudFilters::filterRangeAndSelfMask(
    const std::vector<core::LidarPointXYZIRT>& input,
    const FrontEndFilterConfig& cfg,
    FilterPipelineStats* out_stats) {
    std::vector<core::LidarPointXYZIRT> out;
    out.reserve(input.size());

    const double r_min2 = cfg.min_range_m * cfg.min_range_m;
    const double r_max2 = cfg.max_range_m * cfg.max_range_m;

    size_t rej_range = 0;
    size_t rej_mask = 0;

    for (const auto& pt : input) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            rej_range++;
            continue;
        }
        const double r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (r2 < r_min2 || r2 > r_max2) {
            rej_range++;
            continue;
        }
        if (cfg.enable_self_mask_box) {
            if (pt.x >= cfg.self_mask_min_m.x && pt.x <= cfg.self_mask_max_m.x &&
                pt.y >= cfg.self_mask_min_m.y && pt.y <= cfg.self_mask_max_m.y &&
                pt.z >= cfg.self_mask_min_m.z && pt.z <= cfg.self_mask_max_m.z) {
                rej_mask++;
                continue;
            }
        }
        out.push_back(pt);
    }

    if (out_stats) {
        out_stats->input_points = input.size();
        out_stats->rejected_range_or_nan += rej_range;
        out_stats->rejected_self_mask += rej_mask;
        out_stats->output_points = out.size();
    }
    return out;
}

std::vector<core::LidarPointXYZIRT> PointCloudFilters::filterFlyingPixels(
    const std::vector<core::LidarPointXYZIRT>& input,
    double jump_ratio,
    double min_jump_m,
    size_t* out_rejected_count) {
    if (input.size() < 32) {
        if (out_rejected_count) *out_rejected_count = 0;
        return input;
    }

    // Zero-allocation O(N) single-pass 3-point sliding window per laser ring (up to 256 rings)
    struct RingWindow {
        int prev_out_idx{-1};
        double r_prev2{-1.0};
        double r_prev1{-1.0};
    };
    std::array<RingWindow, 256> rings{};

    // Quick probe to verify ring indices are populated (skip for ringless depth/Mid-360 scans)
    bool has_rings = false;
    const size_t probe_n = std::min<size_t>(input.size(), 128);
    for (size_t i = 0; i < probe_n; ++i) {
        if (input[i].ring > 0 && input[i].ring < 256) {
            has_rings = true;
            break;
        }
    }
    if (!has_rings) {
        for (size_t i = probe_n; i < input.size(); i += 16) {
            if (input[i].ring > 0 && input[i].ring < 256) {
                has_rings = true;
                break;
            }
        }
    }
    if (!has_rings) {
        if (out_rejected_count) *out_rejected_count = 0;
        return input;
    }

    std::vector<uint8_t> keep(input.size(), 1);
    size_t rejected = 0;

    for (size_t i = 0; i < input.size(); ++i) {
        const auto& pt = input[i];
        if (pt.ring >= 256) continue;
        const double r = (pt.range_m > 0.0f)
            ? static_cast<double>(pt.range_m)
            : std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);

        auto& rw = rings[pt.ring];
        if (rw.prev_out_idx >= 0 && rw.r_prev2 >= 0.0) {
            const double thresh = std::max(min_jump_m, rw.r_prev1 * jump_ratio);
            if (std::abs(rw.r_prev1 - rw.r_prev2) > thresh &&
                std::abs(rw.r_prev1 - r) > thresh) {
                keep[static_cast<size_t>(rw.prev_out_idx)] = 0;
                rejected++;
            }
        }
        rw.r_prev2 = rw.r_prev1;
        rw.r_prev1 = r;
        rw.prev_out_idx = static_cast<int>(i);
    }

    std::vector<core::LidarPointXYZIRT> out;
    out.reserve(input.size() - rejected);
    for (size_t i = 0; i < input.size(); ++i) {
        if (keep[i]) out.push_back(input[i]);
    }
    if (out_rejected_count) *out_rejected_count = rejected;
    return out;
}

std::vector<math::Vec3d> PointCloudFilters::adaptiveRangeVoxelDownsample(
    const std::vector<core::LidarPointXYZIRT>& input,
    const FrontEndFilterConfig& cfg) {
    std::vector<math::Vec3d> features;
    if (input.empty()) return features;

    if (!cfg.enable_adaptive_voxel_downsample || input.size() <= 300) {
        features.reserve(input.size());
        for (const auto& pt : input) {
            features.emplace_back(pt.x, pt.y, pt.z);
        }
        return features;
    }

    const double inv_near = 1.0 / std::max(0.02, cfg.near_voxel_m);
    const double inv_mid  = 1.0 / std::max(0.02, cfg.mid_voxel_m);
    const double inv_far  = 1.0 / std::max(0.02, cfg.far_voxel_m);
    const double near_r2  = cfg.near_range_cutoff_m * cfg.near_range_cutoff_m;
    const double far_r2   = cfg.far_range_cutoff_m * cfg.far_range_cutoff_m;

    struct CentroidAcc {
        double sx{0.0}, sy{0.0}, sz{0.0};
        uint32_t n{0};
    };

    // Fast open-addressing flat hash table (16K slots power-of-two, zero heap node allocations!)
    constexpr size_t kTableSize = 16384;
    constexpr size_t kTableMask = kTableSize - 1;
    struct Slot {
        uint64_t key{0};
        CentroidAcc acc{};
    };
    std::vector<Slot> table(kTableSize);
    std::vector<size_t> used_slots;
    used_slots.reserve(4096);

    for (const auto& pt : input) {
        const double r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        uint64_t tier = 1;
        double inv_v = inv_mid;
        if (r2 <= near_r2) {
            tier = 0;
            inv_v = inv_near;
        } else if (r2 > far_r2) {
            tier = 2;
            inv_v = inv_far;
        }

        const uint64_t key = packTierVoxelKey(
            static_cast<int64_t>(std::floor(pt.x * inv_v)),
            static_cast<int64_t>(std::floor(pt.y * inv_v)),
            static_cast<int64_t>(std::floor(pt.z * inv_v)),
            tier) + 1ULL;  // +1 ensures key 0 is always empty sentinel

        size_t slot = FastU64Hash{}(key) & kTableMask;
        for (int probe = 0; probe < 24; ++probe) {
            auto& s = table[slot];
            if (s.key == 0) {
                s.key = key;
                s.acc = {pt.x, pt.y, pt.z, 1u};
                used_slots.push_back(slot);
                break;
            }
            if (s.key == key) {
                s.acc.sx += pt.x;
                s.acc.sy += pt.y;
                s.acc.sz += pt.z;
                s.acc.n++;
                break;
            }
            slot = (slot + 1) & kTableMask;
        }
    }

    const size_t n_used = used_slots.size();
    const size_t stride = (cfg.max_output_features > 0 && n_used > cfg.max_output_features)
        ? ((n_used + cfg.max_output_features - 1) / cfg.max_output_features)
        : 1;

    features.reserve(n_used / stride + 1);
    for (size_t i = 0; i < n_used; i += stride) {
        const auto& acc = table[used_slots[i]].acc;
        const double inv_c = 1.0 / static_cast<double>(acc.n);
        features.emplace_back(acc.sx * inv_c, acc.sy * inv_c, acc.sz * inv_c);
    }
    return features;
}

void PointCloudFilters::conditionFrontEndScan(
    const std::vector<core::LidarPointXYZIRT>& raw_body_points,
    const FrontEndFilterConfig& cfg,
    std::vector<core::LidarPointXYZIRT>& out_clean_points,
    std::vector<math::Vec3d>& out_odometry_features,
    FilterPipelineStats* out_stats) {
    FilterPipelineStats stats;
    stats.input_points = raw_body_points.size();
    out_clean_points.clear();
    out_clean_points.reserve(raw_body_points.size());

    const double r_min2 = cfg.min_range_m * cfg.min_range_m;
    const double r_max2 = cfg.max_range_m * cfg.max_range_m;
    const bool do_range = cfg.enable_range_and_mask_filter;
    const bool do_mask  = do_range && cfg.enable_self_mask_box;

    // Fused Pass 1: Range/NaN/Self-Mask + Ring Flying-Pixel Veil Detection (zero intermediate copies!)
    struct RingWindow {
        int prev_out_idx{-1};
        double r_prev2{-1.0};
        double r_prev1{-1.0};
    };
    std::array<RingWindow, 256> rings{};
    std::vector<uint8_t> drop_flags;
    bool do_fp = cfg.enable_flying_pixel_filter && raw_body_points.size() >= 32;
    if (do_fp) {
        bool has_rings = false;
        const size_t probe_n = std::min<size_t>(raw_body_points.size(), 128);
        for (size_t i = 0; i < probe_n; ++i) {
            if (raw_body_points[i].ring > 0 && raw_body_points[i].ring < 256) { has_rings = true; break; }
        }
        do_fp = has_rings;
        if (do_fp) drop_flags.assign(raw_body_points.size(), 0);
    }

    for (const auto& pt : raw_body_points) {
        if (do_range) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                stats.rejected_range_or_nan++;
                continue;
            }
            const double r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
            if (r2 < r_min2 || r2 > r_max2) {
                stats.rejected_range_or_nan++;
                continue;
            }
            if (do_mask &&
                pt.x >= cfg.self_mask_min_m.x && pt.x <= cfg.self_mask_max_m.x &&
                pt.y >= cfg.self_mask_min_m.y && pt.y <= cfg.self_mask_max_m.y &&
                pt.z >= cfg.self_mask_min_m.z && pt.z <= cfg.self_mask_max_m.z) {
                stats.rejected_self_mask++;
                continue;
            }
        }

        const size_t out_idx = out_clean_points.size();
        out_clean_points.push_back(pt);

        if (do_fp && pt.ring < 256) {
            const double r = (pt.range_m > 0.0f)
                ? static_cast<double>(pt.range_m)
                : std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            auto& rw = rings[pt.ring];
            if (rw.prev_out_idx >= 0 && rw.r_prev2 >= 0.0) {
                const double thresh = std::max(cfg.flying_pixel_min_jump_m, rw.r_prev1 * cfg.flying_pixel_jump_ratio);
                if (std::abs(rw.r_prev1 - rw.r_prev2) > thresh &&
                    std::abs(rw.r_prev1 - r) > thresh) {
                    drop_flags[static_cast<size_t>(rw.prev_out_idx)] = 1;
                    stats.rejected_flying_pixels++;
                }
            }
            rw.r_prev2 = rw.r_prev1;
            rw.r_prev1 = r;
            rw.prev_out_idx = static_cast<int>(out_idx);
        }
    }

    // In-place compaction if any flying pixels were flagged (zero extra allocation)
    if (do_fp && stats.rejected_flying_pixels > 0) {
        size_t w = 0;
        for (size_t i = 0; i < out_clean_points.size(); ++i) {
            if (!drop_flags[i]) {
                if (w != i) out_clean_points[w] = out_clean_points[i];
                w++;
            }
        }
        out_clean_points.resize(w);
    }

    out_odometry_features = adaptiveRangeVoxelDownsample(out_clean_points, cfg);
    stats.output_points = out_clean_points.size();
    if (out_stats) *out_stats = stats;
}

std::vector<core::LidarPointXYZIRT> PointCloudFilters::fuseVoxelCentroidsAndRgb(
    const std::vector<core::LidarPointXYZIRT>& world_points,
    double voxel_size_m) {
    if (world_points.empty() || voxel_size_m <= 1e-4) {
        return world_points;
    }

    const double inv_v = 1.0 / voxel_size_m;
    struct VoxelAccum {
        double sx{0.0}, sy{0.0}, sz{0.0};
        double sint{0.0};
        double sr{0.0}, sg{0.0}, sb{0.0};
        double rgb_weight{0.0};
        uint32_t count{0};
        uint32_t rgb_count{0};
        core::LidarPointXYZIRT proto{};
    };

    std::unordered_map<GridKey, VoxelAccum, GridKeyHash> grid;
    grid.reserve(world_points.size() / 2 + 64);

    for (const auto& pt : world_points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
        const GridKey k = makeKey(pt.x, pt.y, pt.z, inv_v);
        auto& acc = grid[k];
        if (acc.count == 0) {
            acc.proto = pt;
        }
        acc.sx += pt.x;
        acc.sy += pt.y;
        acc.sz += pt.z;
        acc.sint += static_cast<double>(pt.intensity);
        acc.count++;

        if (pt.colorized) {
            const double w = 1.0 / std::max(0.5, static_cast<double>(pt.range_m > 0.1f ? pt.range_m : 2.0f));
            acc.sr += w * pt.r;
            acc.sg += w * pt.g;
            acc.sb += w * pt.b;
            acc.rgb_weight += w;
            acc.rgb_count++;
        }
    }

    std::vector<core::LidarPointXYZIRT> out;
    out.reserve(grid.size());
    for (const auto& kv : grid) {
        const auto& acc = kv.second;
        core::LidarPointXYZIRT p = acc.proto;
        const double inv_n = 1.0 / static_cast<double>(acc.count);
        p.x = acc.sx * inv_n;
        p.y = acc.sy * inv_n;
        p.z = acc.sz * inv_n;
        p.intensity = static_cast<float>(acc.sint * inv_n);
        if (acc.rgb_count > 0 && acc.rgb_weight > 1e-6) {
            p.r = static_cast<uint8_t>(std::clamp(std::round(acc.sr / acc.rgb_weight), 0.0, 255.0));
            p.g = static_cast<uint8_t>(std::clamp(std::round(acc.sg / acc.rgb_weight), 0.0, 255.0));
            p.b = static_cast<uint8_t>(std::clamp(std::round(acc.sb / acc.rgb_weight), 0.0, 255.0));
            p.colorized = true;
        }
        out.push_back(p);
    }
    return out;
}

std::vector<core::LidarPointXYZIRT> PointCloudFilters::radiusOutlierRemoval(
    const std::vector<core::LidarPointXYZIRT>& input,
    double cell_size_m,
    uint32_t min_neighbors,
    size_t* out_removed_count) {
    if (input.size() < 16 || min_neighbors == 0 || cell_size_m <= 1e-4) {
        if (out_removed_count) *out_removed_count = 0;
        return input;
    }

    const double inv_c = 1.0 / cell_size_m;
    std::unordered_map<GridKey, uint32_t, GridKeyHash> counts;
    counts.reserve(input.size() / 2 + 64);

    for (const auto& pt : input) {
        counts[makeKey(pt.x, pt.y, pt.z, inv_c)]++;
    }

    std::vector<core::LidarPointXYZIRT> out;
    out.reserve(input.size());
    size_t removed = 0;

    for (const auto& pt : input) {
        const GridKey bk = makeKey(pt.x, pt.y, pt.z, inv_c);
        uint32_t total = 0;
        auto it_center = counts.find(bk);
        if (it_center != counts.end()) {
            total += it_center->second;
        }
        if (total < min_neighbors + 1) {
            for (int dx = -1; dx <= 1 && total < min_neighbors + 1; ++dx) {
                for (int dy = -1; dy <= 1 && total < min_neighbors + 1; ++dy) {
                    for (int dz = -1; dz <= 1 && total < min_neighbors + 1; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) continue;
                        auto it = counts.find({bk.x + dx, bk.y + dy, bk.z + dz});
                        if (it != counts.end()) total += it->second;
                    }
                }
            }
        }
        if (total >= min_neighbors + 1) {
            out.push_back(pt);
        } else {
            removed++;
        }
    }

    if (out_removed_count) *out_removed_count = removed;
    return out;
}

std::vector<core::LidarPointXYZIRT> PointCloudFilters::statisticalOutlierRemoval(
    const std::vector<core::LidarPointXYZIRT>& input,
    double cell_size_m,
    double std_multiplier,
    size_t* out_removed_count) {
    if (input.size() < 32 || cell_size_m <= 1e-4) {
        if (out_removed_count) *out_removed_count = 0;
        return input;
    }

    const double inv_c = 1.0 / cell_size_m;
    struct CellStat {
        math::Vec3d sum{0.0, 0.0, 0.0};
        uint32_t count{0};
    };
    std::unordered_map<GridKey, CellStat, GridKeyHash> cells;
    cells.reserve(input.size() / 4 + 64);

    for (const auto& pt : input) {
        auto& c = cells[makeKey(pt.x, pt.y, pt.z, inv_c)];
        c.sum.x += pt.x;
        c.sum.y += pt.y;
        c.sum.z += pt.z;
        c.count++;
    }

    std::vector<double> dists(input.size(), 0.0);
    double sum_d = 0.0;
    double sum_d2 = 0.0;

    static constexpr int64_t kOff7[7][3] = {
        {0, 0, 0}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
    };

    for (size_t i = 0; i < input.size(); ++i) {
        const auto& pt = input[i];
        const GridKey bk = makeKey(pt.x, pt.y, pt.z, inv_c);
        math::Vec3d nb_sum(0.0, 0.0, 0.0);
        uint32_t nb_cnt = 0;
        for (int n = 0; n < 7; ++n) {
            auto it = cells.find({bk.x + kOff7[n][0], bk.y + kOff7[n][1], bk.z + kOff7[n][2]});
            if (it != cells.end()) {
                nb_sum += it->second.sum;
                nb_cnt += it->second.count;
            }
        }
        double d = 0.0;
        if (nb_cnt > 1) {
            const math::Vec3d mean = (nb_sum - math::Vec3d(pt.x, pt.y, pt.z)) / static_cast<double>(nb_cnt - 1);
            d = (math::Vec3d(pt.x, pt.y, pt.z) - mean).norm();
        } else {
            d = cell_size_m * 2.0;
        }
        dists[i] = d;
        sum_d += d;
        sum_d2 += d * d;
    }

    const double inv_n = 1.0 / static_cast<double>(input.size());
    const double mean_d = sum_d * inv_n;
    const double var_d = std::max(0.0, sum_d2 * inv_n - mean_d * mean_d);
    const double thresh = mean_d + std_multiplier * std::sqrt(var_d);

    std::vector<core::LidarPointXYZIRT> out;
    out.reserve(input.size());
    size_t removed = 0;
    for (size_t i = 0; i < input.size(); ++i) {
        if (dists[i] <= thresh) {
            out.push_back(input[i]);
        } else {
            removed++;
        }
    }

    if (out_removed_count) *out_removed_count = removed;
    return out;
}

std::vector<core::LidarPointXYZIRT> PointCloudFilters::surfelGuidedSurfaceDenoise(
    const std::vector<core::LidarPointXYZIRT>& input,
    double voxel_size_m,
    double min_planarity,
    double max_thickness_var,
    double projection_alpha,
    size_t* out_snapped_count) {
    if (input.size() < 32 || voxel_size_m <= 1e-4 || projection_alpha <= 1e-4) {
        if (out_snapped_count) *out_snapped_count = 0;
        return input;
    }

    const double inv_v = 1.0 / voxel_size_m;
    struct SurfStat {
        double sx{0}, sy{0}, sz{0};
        double sxx{0}, sxy{0}, sxz{0}, syy{0}, syz{0}, szz{0};
        uint32_t n{0};
        math::Vec3d mean{};
        math::Vec3d normal{};
        bool planar{false};
    };

    std::unordered_map<GridKey, SurfStat, GridKeyHash> grid;
    grid.reserve(input.size() / 6 + 64);

    for (const auto& pt : input) {
        auto& s = grid[makeKey(pt.x, pt.y, pt.z, inv_v)];
        s.sx += pt.x; s.sy += pt.y; s.sz += pt.z;
        s.sxx += pt.x * pt.x; s.sxy += pt.x * pt.y; s.sxz += pt.x * pt.z;
        s.syy += pt.y * pt.y; s.syz += pt.y * pt.z; s.szz += pt.z * pt.z;
        s.n++;
    }

    for (auto& kv : grid) {
        auto& s = kv.second;
        if (s.n < 6) continue;
        const double inv_n = 1.0 / static_cast<double>(s.n);
        s.mean = math::Vec3d(s.sx * inv_n, s.sy * inv_n, s.sz * inv_n);
        math::Mat3d cov;
        cov(0, 0) = s.sxx * inv_n - s.mean.x * s.mean.x;
        cov(0, 1) = cov(1, 0) = s.sxy * inv_n - s.mean.x * s.mean.y;
        cov(0, 2) = cov(2, 0) = s.sxz * inv_n - s.mean.x * s.mean.z;
        cov(1, 1) = s.syy * inv_n - s.mean.y * s.mean.y;
        cov(1, 2) = cov(2, 1) = s.syz * inv_n - s.mean.y * s.mean.z;
        cov(2, 2) = s.szz * inv_n - s.mean.z * s.mean.z;

        const auto eig = math::SymEigen3d::compute(cov);
        const double l0 = std::max(0.0, eig.eigenvalues[0]);
        const double l1 = std::max(0.0, eig.eigenvalues[1]);
        const double l2 = std::max(1e-9, eig.eigenvalues[2]);
        const double planarity = (l1 - l0) / l2;
        if (l1 > 1e-4 && planarity >= min_planarity && l0 <= max_thickness_var) {
            s.normal = math::Vec3d(eig.eigenvectors(0, 0), eig.eigenvectors(1, 0), eig.eigenvectors(2, 0)).normalized();
            s.planar = true;
        }
    }

    const double alpha = std::clamp(projection_alpha, 0.0, 1.0);
    std::vector<core::LidarPointXYZIRT> out = input;
    size_t snapped = 0;

    for (auto& pt : out) {
        auto it = grid.find(makeKey(pt.x, pt.y, pt.z, inv_v));
        if (it == grid.end() || !it->second.planar) continue;
        const auto& s = it->second;
        const math::Vec3d p(pt.x, pt.y, pt.z);
        const double signed_d = s.normal.dot(p - s.mean);
        if (std::abs(signed_d) <= 0.06) {
            const math::Vec3d p_clean = p - s.normal * (alpha * signed_d);
            pt.x = p_clean.x;
            pt.y = p_clean.y;
            pt.z = p_clean.z;
            snapped++;
        }
    }

    if (out_snapped_count) *out_snapped_count = snapped;
    return out;
}

std::vector<core::LidarPointXYZIRT> PointCloudFilters::runFullMapRefinement(
    const std::vector<core::LidarPointXYZIRT>& raw_world_points,
    const MapRefinementFilterConfig& cfg,
    FilterPipelineStats* out_stats) {
    FilterPipelineStats stats;
    stats.input_points = raw_world_points.size();

    std::vector<core::LidarPointXYZIRT> cloud =
        (cfg.fusion_voxel_m > 1e-4) ? fuseVoxelCentroidsAndRgb(raw_world_points, cfg.fusion_voxel_m)
                                    : raw_world_points;

    if (cfg.enable_radius_outlier_removal) {
        cloud = radiusOutlierRemoval(cloud, cfg.ror_cell_size_m, cfg.ror_min_neighbors, &stats.rejected_ror);
    }
    if (cfg.enable_statistical_outlier_removal) {
        cloud = statisticalOutlierRemoval(cloud, cfg.sor_cell_size_m, cfg.sor_std_multiplier, &stats.rejected_sor);
    }
    if (cfg.enable_surface_denoising) {
        cloud = surfelGuidedSurfaceDenoise(cloud,
                                           cfg.denoise_voxel_m,
                                           cfg.denoise_planarity_min,
                                           cfg.denoise_max_thickness_var,
                                           cfg.denoise_projection_alpha,
                                           &stats.denoised_planar_points);
    }

    stats.output_points = cloud.size();
    if (out_stats) *out_stats = stats;
    return cloud;
}

}  // namespace filters
}  // namespace avscan
