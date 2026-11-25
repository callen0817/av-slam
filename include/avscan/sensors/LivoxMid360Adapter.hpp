#pragma once

#include "avscan/sensors/SensorRigProfile.hpp"
#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

namespace avscan {
namespace sensors {

// Clean-room implementation of the public Livox Mid-360 SDK2 UDP protocol (zero third-party SDK dependencies).
// Supports:
//  - Point Cloud UDP (default port 56301): data_type 1 (Cartesian 32-bit mm, 96 pts/pkt),
//    data_type 2 (Cartesian 16-bit 10mm, 96 pts/pkt), data_type 3 (Spherical, 96 pts/pkt)
//  - IMU UDP (default port 56401): data_type 0 (200 Hz 6-axis gyro rad/s + accel g -> m/s^2)
struct LivoxMid360NetworkConfig {
    std::string sensor_ip{"192.168.1.100"};
    std::string host_ip{"192.168.1.50"};
    uint16_t point_cloud_port{56301};
    uint16_t imu_port{56401};
    double frame_period_sec{0.10};  // 10 Hz scan accumulation window
};

struct LivoxPacketStats {
    uint64_t cloud_packets_valid{0};
    uint64_t cloud_packets_malformed{0};
    uint64_t imu_packets_valid{0};
    uint64_t imu_packets_malformed{0};
    uint64_t frames_emitted{0};
};

class LivoxMid360Decoder {
public:
    static constexpr size_t kHeaderSize = 36;
    static constexpr uint8_t kDataTypeImu = 0;
    static constexpr uint8_t kDataTypeCartesianHigh = 1; // 14 bytes/pt: int32 x,y,z (mm), uint8 refl, uint8 tag
    static constexpr uint8_t kDataTypeCartesianLow = 2;  // 8 bytes/pt: int16 x,y,z (10mm), uint8 refl, uint8 tag
    static constexpr uint8_t kDataTypeSpherical = 3;     // 10 bytes/pt: uint32 depth (mm), uint16 theta/phi (0.01 deg), uint8 refl, uint8 tag

    explicit LivoxMid360Decoder(double frame_period_sec = 0.10);

    bool decodePointCloudPacket(const uint8_t* data, size_t len,
                                bool& out_frame_completed,
                                core::LidarFrame& out_frame);

    bool decodeImuPacket(const uint8_t* data, size_t len,
                         core::ImuMeasurement& out_imu);

    const LivoxPacketStats& stats() const { return stats_; }
    void reset();

    // Deterministic packet builders for protocol verification & synthetic testing
    static std::vector<uint8_t> buildCartesianHighPacket(
        uint64_t timestamp_ns,
        uint32_t time_interval_tenth_us,
        const std::vector<core::LidarPointXYZIRT>& points);

    static std::vector<uint8_t> buildImuPacket(
        uint64_t timestamp_ns,
        const core::ImuMeasurement& imu);

private:
    double frame_period_sec_{0.10};
    uint64_t next_frame_id_{1};
    core::LidarFrame accum_frame_{};
    LivoxPacketStats stats_{};
};

// Native BSD/WinSock UDP stream adapter plugging Livox Mid-360 directly into AV-SLAM on iOS, Jetson, or Desktop
class LivoxMid360Adapter final : public ISensorStreamAdapter {
public:
    explicit LivoxMid360Adapter(const LivoxMid360NetworkConfig& cfg = LivoxMid360NetworkConfig{});
    ~LivoxMid360Adapter() override;

    std::string adapterName() const override { return "Livox Mid-360 Native UDP Adapter"; }
    GenericSensorRigConfig rigConfig() const override { return GenericSensorRigConfig::presetLivoxMid360(); }

    void setCallbacks(FrameCallback on_frame, ImuCallback on_imu, RigConfigCallback on_rig_update) override;
    bool start() override;
    void stop() override;
    bool isRunning() const override { return running_.load(); }

    const LivoxPacketStats& stats() const { return decoder_.stats(); }

private:
    void receiveLoop();

    LivoxMid360NetworkConfig cfg_;
    LivoxMid360Decoder decoder_;
    FrameCallback on_frame_;
    ImuCallback on_imu_;
    RigConfigCallback on_rig_update_;
    std::atomic<bool> running_{false};
    std::thread rx_thread_;
};

}  // namespace sensors
}  // namespace avscan
