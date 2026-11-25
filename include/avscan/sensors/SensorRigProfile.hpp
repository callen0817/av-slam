#pragma once

#include "avscan/core/Types.hpp"
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace avscan {
namespace sensors {

// Hardware-agnostic LiDAR / depth sensor family identifier for AV-SLAM
enum class LidarHardwareType : uint8_t {
    GENERIC_XYZIRT = 0,
    ROBOSENSE_AIRY = 1,
    LIVOX_MID360 = 2,
    OUSTER_OS0_OS1 = 3,
    HESAI_XT_JT = 4,
    APPLE_ARKIT_DTOF = 5
};

// Per-sensor stochastic noise and geometric operating envelope
struct SensorNoiseModel {
    double min_valid_range_m{0.35};
    double max_valid_range_m{60.0};
    double range_noise_sigma_m{0.015};        // 1-sigma ranging precision (m)
    double angular_noise_sigma_deg{0.05};     // 1-sigma beam angular jitter (deg)
    double nominal_scan_rate_hz{10.0};        // Sweep rate (Hz)
    double nominal_imu_rate_hz{200.0};        // IMU ODR (Hz)

    // Continuous-time IMU noise densities
    double gyro_noise_sigma{0.08};            // rad / s / sqrt(Hz)
    double accel_noise_sigma{0.28};           // m / s^2 / sqrt(Hz)
    double bias_gyro_rw_sigma{0.008};         // rad / s^2 / sqrt(Hz)
    double bias_accel_rw_sigma{0.008};        // m / s^3 / sqrt(Hz)

    // Self-occlusion / operator rig exclusion box in LiDAR sensor frame (disabled if max <= min)
    bool enable_self_mask_box{false};
    math::Vec3d self_mask_min_m{-0.25, -0.25, -0.25};
    math::Vec3d self_mask_max_m{0.25, 0.25, 0.25};
};

// Complete hardware-agnostic sensor rig specification consumed by AV-SLAM across
// AV Scan iOS (iPhone/iPad), AV scanAR (Jetson), and AV Scan (Desktop).
struct GenericSensorRigConfig {
    LidarHardwareType hardware_type{LidarHardwareType::GENERIC_XYZIRT};
    std::string rig_id{"generic_external_lidar"};
    std::string display_name{"Generic External 3D LiDAR + IMU"};

    // Rigid transform mapping points from LiDAR sensor frame into IMU body frame: p_imu = T_imu_lidar * p_lidar
    math::SE3d T_imu_lidar{math::SE3d::Identity()};

    // Optional camera-to-LiDAR extrinsic & temporal calibration
    core::ExtrinsicCalibrationProfile camera_extrinsics{};
    bool has_integrated_imu{true};
    bool has_per_point_timestamps{true};
    bool is_ring_ordered{true};

    SensorNoiseModel noise{};

    // Seamless adapter from existing RoboSense Airy DIFOP factory calibration
    static GenericSensorRigConfig fromAiryFactoryCalibration(const core::AiryFactoryCalibration& airy) {
        GenericSensorRigConfig cfg = presetRoboSenseAiry();
        cfg.T_imu_lidar = airy.T_imu_lidar;
        if (!airy.mac_address.empty()) {
            cfg.rig_id = "robosense_airy_" + airy.mac_address;
        }
        return cfg;
    }

    static GenericSensorRigConfig presetRoboSenseAiry() {
        GenericSensorRigConfig cfg;
        cfg.hardware_type = LidarHardwareType::ROBOSENSE_AIRY;
        cfg.rig_id = "robosense_airy_96";
        cfg.display_name = "RoboSense Airy (96-Beam Hemispherical + 200 Hz IMU)";
        cfg.has_integrated_imu = true;
        cfg.has_per_point_timestamps = true;
        cfg.is_ring_ordered = true;
        // Official RoboSense Airy nominal T_imu_from_lidar (matches AiryDecoder factory default before first DIFOP arrives)
        math::Quatd q_lidar_imu(0.711454, -0.702718, 0.001879, 0.004094);
        q_lidar_imu.normalize();
        cfg.T_imu_lidar = math::SE3d(q_lidar_imu, math::Vec3d(0.00425, 0.00418, -0.00446)).inverse();
        cfg.noise.min_valid_range_m = 0.55;
        cfg.noise.max_valid_range_m = 60.0;
        cfg.noise.range_noise_sigma_m = 0.015;
        cfg.noise.nominal_scan_rate_hz = 10.0;
        cfg.noise.nominal_imu_rate_hz = 200.0;
        cfg.noise.gyro_noise_sigma = 0.08;
        cfg.noise.accel_noise_sigma = 0.28;
        cfg.noise.bias_gyro_rw_sigma = 0.008;
        cfg.noise.bias_accel_rw_sigma = 0.008;
        return cfg;
    }

    static GenericSensorRigConfig presetLivoxMid360() {
        GenericSensorRigConfig cfg;
        cfg.hardware_type = LidarHardwareType::LIVOX_MID360;
        cfg.rig_id = "livox_mid360";
        cfg.display_name = "Livox Mid-360 (Non-Repetitive + 200 Hz IMU)";
        cfg.has_integrated_imu = true;
        cfg.has_per_point_timestamps = true;
        cfg.is_ring_ordered = false;
        // Official Livox Mid-360 nominal IMU offset (mm -> m)
        cfg.T_imu_lidar = math::SE3d(math::SO3d(), math::Vec3d(-0.011, -0.02329, 0.04412));
        cfg.noise.min_valid_range_m = 0.40;
        cfg.noise.max_valid_range_m = 70.0;
        cfg.noise.range_noise_sigma_m = 0.020;
        cfg.noise.nominal_scan_rate_hz = 10.0;
        cfg.noise.nominal_imu_rate_hz = 200.0;
        cfg.noise.gyro_noise_sigma = 0.06;
        cfg.noise.accel_noise_sigma = 0.25;
        return cfg;
    }

    static GenericSensorRigConfig presetOusterOs0() {
        GenericSensorRigConfig cfg;
        cfg.hardware_type = LidarHardwareType::OUSTER_OS0_OS1;
        cfg.rig_id = "ouster_os0_128";
        cfg.display_name = "Ouster OS0 / OS1 (Multi-Beam + 100 Hz IMU)";
        cfg.has_integrated_imu = true;
        cfg.has_per_point_timestamps = true;
        cfg.is_ring_ordered = true;
        cfg.noise.min_valid_range_m = 0.50;
        cfg.noise.max_valid_range_m = 90.0;
        cfg.noise.range_noise_sigma_m = 0.018;
        cfg.noise.nominal_scan_rate_hz = 10.0;
        cfg.noise.nominal_imu_rate_hz = 100.0;
        return cfg;
    }

    static GenericSensorRigConfig presetHesaiXt32() {
        GenericSensorRigConfig cfg;
        cfg.hardware_type = LidarHardwareType::HESAI_XT_JT;
        cfg.rig_id = "hesai_xt32";
        cfg.display_name = "Hesai XT / JT Series External LiDAR";
        cfg.has_integrated_imu = true;
        cfg.has_per_point_timestamps = true;
        cfg.is_ring_ordered = true;
        cfg.noise.min_valid_range_m = 0.40;
        cfg.noise.max_valid_range_m = 100.0;
        cfg.noise.range_noise_sigma_m = 0.010;
        cfg.noise.nominal_scan_rate_hz = 10.0;
        cfg.noise.nominal_imu_rate_hz = 200.0;
        return cfg;
    }

    static GenericSensorRigConfig presetAppleArkitDepth() {
        GenericSensorRigConfig cfg;
        cfg.hardware_type = LidarHardwareType::APPLE_ARKIT_DTOF;
        cfg.rig_id = "apple_arkit_dtof";
        cfg.display_name = "Apple Built-In dToF LiDAR + CoreMotion IMU";
        cfg.has_integrated_imu = true;
        cfg.has_per_point_timestamps = false;
        cfg.is_ring_ordered = false;
        cfg.noise.min_valid_range_m = 0.25;
        cfg.noise.max_valid_range_m = 5.0;
        cfg.noise.range_noise_sigma_m = 0.025;
        cfg.noise.nominal_scan_rate_hz = 30.0;
        cfg.noise.nominal_imu_rate_hz = 100.0;
        return cfg;
    }
};

// Abstract sensor stream adapter interface so AV-SLAM ingests any external or native
// LiDAR + IMU hardware (RoboSense, Livox, Ouster, Hesai, ARKit, MCAP/AVRAW replay) uniformly.
class ISensorStreamAdapter {
public:
    using FrameCallback = std::function<void(const core::LidarFrame&)>;
    using ImuCallback = std::function<void(const core::ImuMeasurement&)>;
    using RigConfigCallback = std::function<void(const GenericSensorRigConfig&)>;

    virtual ~ISensorStreamAdapter() = default;

    virtual std::string adapterName() const = 0;
    virtual GenericSensorRigConfig rigConfig() const = 0;
    virtual void setCallbacks(FrameCallback on_frame, ImuCallback on_imu, RigConfigCallback on_rig_update) = 0;
    virtual bool start() = 0;
    virtual void stop() = 0;
    virtual bool isRunning() const = 0;
};

}  // namespace sensors
}  // namespace avscan
