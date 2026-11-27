#pragma once

#include "avscan/core/Types.hpp"
#include "avscan/sensors/GnssRtkReceiver.hpp"
#include "avscan/sensors/IDepthProvider.hpp"
#include "avscan/slam/AvSlamEngine.hpp"
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace avscan {
namespace slam {

// Bitmask of active sensor channels automatically detected per frame
enum SensorChannelMask : uint32_t {
    SENSOR_NONE         = 0,
    SENSOR_LIDAR        = 1u << 0,
    SENSOR_DEPTH_CAMERA = 1u << 1,
    SENSOR_RGB_CAMERA   = 1u << 2,
    SENSOR_IMU          = 1u << 3,
    SENSOR_GNSS_RTK     = 1u << 4
};

// Universal sensor-agnostic input bundle: populate whichever sensors are connected.
struct UniversalSensorBundle {
    uint64_t sequence_id{0};
    double timestamp_sec{0.0};
    std::optional<core::LidarFrame> lidar_frame;
    std::optional<sensors::DepthFrame> depth_frame;
    std::optional<core::RgbFrame> rgb_frame;
    std::vector<core::ImuMeasurement> imu_measurements;
    std::optional<sensors::GnssRtkMeasurement> gnss_fix;

    uint32_t activeChannelMask() const {
        uint32_t m = SENSOR_NONE;
        if (lidar_frame.has_value() && !lidar_frame->points.empty()) m |= SENSOR_LIDAR;
        if (depth_frame.has_value() && depth_frame->valid())         m |= SENSOR_DEPTH_CAMERA;
        if (rgb_frame.has_value() && !rgb_frame->gray_bytes.empty()) m |= SENSOR_RGB_CAMERA;
        if (!imu_measurements.empty())                               m |= SENSOR_IMU;
        if (gnss_fix.has_value() && gnss_fix->valid)                 m |= SENSOR_GNSS_RTK;
        return m;
    }
};

struct AgnosticSlamStatus {
    uint32_t active_channels{SENSOR_NONE};
    std::string mode_summary{"IDLE"};
    bool gnss_rtk_locked{false};
    sensors::GnssFixQuality latest_gnss_quality{sensors::GnssFixQuality::INVALID};
    size_t frames_processed{0};
    size_t depth_frames_fused{0};
    size_t monocular_frames_tracked{0};
    size_t gnss_factors_applied{0};
    double last_gnss_residual_m{0.0};
};

// Single unified sensor-agnostic AV-SLAM engine:
// Replaces separate LIO, LIVO, Depth-SLAM, and Monocular SLAM pipelines with ONE
// proprietary estimator that adapts automatically to whatever sensors are connected.
class SensorAgnosticSlamEngine {
public:
    explicit SensorAgnosticSlamEngine(const AvSlamConfig& config = AvSlamConfig{});

    void setSensorRigConfig(const sensors::GenericSensorRigConfig& rig);
    void setExtrinsicCalibration(const core::ExtrinsicCalibrationProfile& profile);
    void setDepthProvider(std::shared_ptr<sensors::IDepthProvider> provider);
    void reset();

    // Process ANY combination of connected sensors through the unified AV-SLAM pipeline
    bool processUniversalBundle(const UniversalSensorBundle& bundle,
                                core::LidarFrame& out_body_frame,
                                core::LidarFrame& out_world_frame,
                                EstimatorMetrics& out_metrics);

    core::SlamState getCurrentState() const;
    std::vector<core::SlamState> getTrajectory() const;
    AgnosticSlamStatus getAgnosticStatus() const { return agnostic_status_; }
    AvSlamEngine& coreEngine() { return core_engine_; }
    const AvSlamEngine& coreEngine() const { return core_engine_; }

    static std::string describeChannelMask(uint32_t mask);

private:
    bool runMonocularDirectStep(const core::RgbFrame& rgb,
                                const std::vector<core::ImuMeasurement>& imus,
                                core::LidarFrame& out_body_frame,
                                core::LidarFrame& out_world_frame,
                                EstimatorMetrics& out_metrics);

    void fuseGnssRtkMeasurement(const sensors::GnssRtkMeasurement& fix);

    AvSlamEngine core_engine_;
    std::shared_ptr<sensors::IDepthProvider> depth_provider_;
    core::ExtrinsicCalibrationProfile cam_extrinsics_{};
    AgnosticSlamStatus agnostic_status_{};

    // Monocular-only & optical-flow helper state
    bool has_prev_mono_frame_{false};
    core::RgbFrame prev_mono_frame_{};
    core::SlamState mono_state_{};
    math::SE3d last_mono_kf_pose_{math::SE3d::Identity()};
    bool has_mono_kf_{false};
    std::vector<core::SlamState> mono_trajectory_;
    bool using_mono_fallback_{false};

    // Optical-flow rotation prior for IMU-less Depth-only mode
    bool has_prev_depth_rgb_{false};
    core::RgbFrame prev_depth_rgb_{};
    double last_depth_rgb_ts_{0.0};

    // Geodetic ENU offset alignment
    bool has_gnss_anchor_{false};
    math::Vec3d gnss_world_offset_m_{0.0, 0.0, 0.0};
};

}  // namespace slam
}  // namespace avscan
