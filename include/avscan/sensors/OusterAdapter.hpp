#pragma once

#include "avscan/sensors/SensorRigProfile.hpp"
#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

namespace avscan {
namespace sensors {

// Clean-room implementation of the public Ouster OS0 / OS1 / OS2 Legacy & RNG19 UDP protocol
// (UDP port 7502 LiDAR columns, UDP port 7503 100 Hz IMU packets). Zero third-party SDK dependencies.
struct OusterSensorConfig {
    std::string sensor_ip{"192.168.1.201"};
    uint16_t lidar_port{7502};
    uint16_t imu_port{7503};
    uint16_t pixels_per_column{64};     // 32, 64, or 128 channels
    uint16_t columns_per_packet{16};    // Standard Ouster 16 measurement blocks/packet
    uint16_t columns_per_frame{1024};   // 512, 1024, or 2048 horizontal resolution
    double vfov_deg{90.0};              // 90 deg for OS0, 45 deg for OS1
    double lidar_origin_to_beam_m{0.015806};
};

struct OusterPacketStats {
    uint64_t lidar_packets_valid{0};
    uint64_t lidar_packets_malformed{0};
    uint64_t imu_packets_valid{0};
    uint64_t imu_packets_malformed{0};
    uint64_t frames_emitted{0};
};

class OusterDecoder {
public:
    static constexpr size_t kColumnHeaderBytes = 16;  // ts(8) + m_id(2) + frame_id(2) + enc_count(4)
    static constexpr size_t kChannelBytes = 12;       // range_mm(4, lower 20b) + refl(2) + sig(2) + nir(2) + pad(2)
    static constexpr size_t kColumnFooterBytes = 4;   // packet_status (0xFFFFFFFF valid)
    static constexpr size_t kImuPacketBytes = 48;     // Ouster 48-byte IMU UDP packet

    explicit OusterDecoder(const OusterSensorConfig& cfg = OusterSensorConfig{});

    bool decodeLidarPacket(const uint8_t* data, size_t len,
                           bool& out_frame_completed,
                           core::LidarFrame& out_frame);

    bool decodeImuPacket(const uint8_t* data, size_t len,
                         core::ImuMeasurement& out_imu);

    const OusterPacketStats& stats() const { return stats_; }
    void reset();

    // Deterministic packet builders for unit testing & replay verification
    std::vector<uint8_t> buildSyntheticLidarPacket(
        uint16_t frame_id,
        uint16_t start_m_id,
        uint64_t base_ts_ns,
        double nominal_range_m) const;

    static std::vector<uint8_t> buildSyntheticImuPacket(
        uint64_t timestamp_ns,
        const core::ImuMeasurement& imu);

private:
    void rebuildBeamTables();

    OusterSensorConfig cfg_;
    std::vector<double> sin_alt_;
    std::vector<double> cos_alt_;
    std::vector<double> alt_deg_;
    int32_t active_frame_id_{-1};
    uint64_t next_out_frame_id_{1};
    core::LidarFrame accum_frame_{};
    OusterPacketStats stats_{};
};

class OusterAdapter final : public ISensorStreamAdapter {
public:
    explicit OusterAdapter(const OusterSensorConfig& cfg = OusterSensorConfig{});
    ~OusterAdapter() override;

    std::string adapterName() const override { return "Ouster OS0/OS1 Native UDP Adapter"; }
    GenericSensorRigConfig rigConfig() const override { return GenericSensorRigConfig::presetOusterOs0(); }

    void setCallbacks(FrameCallback on_frame, ImuCallback on_imu, RigConfigCallback on_rig_update) override;
    bool start() override;
    void stop() override;
    bool isRunning() const override { return running_.load(); }

    const OusterPacketStats& stats() const { return decoder_.stats(); }

private:
    void receiveLoop();

    OusterSensorConfig cfg_;
    OusterDecoder decoder_;
    FrameCallback on_frame_;
    ImuCallback on_imu_;
    RigConfigCallback on_rig_update_;
    std::atomic<bool> running_{false};
    std::thread rx_thread_;
};

}  // namespace sensors
}  // namespace avscan
