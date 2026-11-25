#include "avscan/sensors/LivoxMid360Adapter.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace avscan {
namespace sensors {

namespace {

constexpr double kStdGravity = 9.80665;

inline uint16_t readU16LE(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

inline int16_t readI16LE(const uint8_t* p) {
    return static_cast<int16_t>(readU16LE(p));
}

inline uint32_t readU32LE(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

inline int32_t readI32LE(const uint8_t* p) {
    return static_cast<int32_t>(readU32LE(p));
}

inline uint64_t readU64LE(const uint8_t* p) {
    return static_cast<uint64_t>(readU32LE(p)) |
           (static_cast<uint64_t>(readU32LE(p + 4)) << 32);
}

inline float readF32LE(const uint8_t* p) {
    uint32_t u = readU32LE(p);
    float f = 0.0f;
    std::memcpy(&f, &u, sizeof(float));
    return f;
}

inline void writeU16LE(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

inline void writeU32LE(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

inline void writeU64LE(uint8_t* p, uint64_t v) {
    writeU32LE(p, static_cast<uint32_t>(v & 0xFFFFFFFFULL));
    writeU32LE(p + 4, static_cast<uint32_t>((v >> 32) & 0xFFFFFFFFULL));
}

inline void writeF32LE(uint8_t* p, float f) {
    uint32_t u = 0;
    std::memcpy(&u, &f, sizeof(float));
    writeU32LE(p, u);
}

}  // namespace

LivoxMid360Decoder::LivoxMid360Decoder(double frame_period_sec)
    : frame_period_sec_(std::clamp(frame_period_sec, 0.02, 0.50)) {}

void LivoxMid360Decoder::reset() {
    accum_frame_ = core::LidarFrame{};
    next_frame_id_ = 1;
    stats_ = LivoxPacketStats{};
}

bool LivoxMid360Decoder::decodePointCloudPacket(const uint8_t* data, size_t len,
                                                bool& out_frame_completed,
                                                core::LidarFrame& out_frame) {
    out_frame_completed = false;
    if (!data || len < kHeaderSize) {
        stats_.cloud_packets_malformed++;
        return false;
    }

    // Livox SDK2 header layout (36 bytes):
    // [0] version (0), [1..2] length, [3..4] time_interval (0.1 us), [5..6] dot_num,
    // [7..8] udp_cnt, [9] frame_cnt, [10] data_type, [11] time_type, [12..23] rsvd,
    // [24..27] crc32, [28..35] timestamp (uint64 ns)
    const uint16_t time_interval_tenth_us = readU16LE(data + 3);
    const uint16_t dot_num = readU16LE(data + 5);
    const uint8_t data_type = data[10];
    const uint64_t ts_ns = readU64LE(data + 28);
    const double pkt_t0_sec = static_cast<double>(ts_ns) * 1e-9;
    const double pkt_span_sec = static_cast<double>(time_interval_tenth_us) * 1e-7;

    if (dot_num == 0 || (data_type != kDataTypeCartesianHigh &&
                         data_type != kDataTypeCartesianLow &&
                         data_type != kDataTypeSpherical)) {
        stats_.cloud_packets_malformed++;
        return false;
    }

    const size_t pt_stride = (data_type == kDataTypeCartesianHigh) ? 14 :
                             (data_type == kDataTypeCartesianLow)  ? 8  : 10;
    if (len < kHeaderSize + static_cast<size_t>(dot_num) * pt_stride) {
        stats_.cloud_packets_malformed++;
        return false;
    }

    if (accum_frame_.points.empty()) {
        accum_frame_.frame_id = next_frame_id_;
        accum_frame_.timestamp_start_sec = pkt_t0_sec;
    }

    const double dt_per_pt = (dot_num > 1) ? (pkt_span_sec / static_cast<double>(dot_num - 1)) : 0.0;
    const uint8_t* payload = data + kHeaderSize;

    for (uint16_t i = 0; i < dot_num; ++i) {
        const uint8_t* p = payload + static_cast<size_t>(i) * pt_stride;
        double x = 0.0, y = 0.0, z = 0.0;
        uint8_t refl = 0;
        uint8_t tag = 0;

        if (data_type == kDataTypeCartesianHigh) {
            x = static_cast<double>(readI32LE(p + 0)) * 0.001;
            y = static_cast<double>(readI32LE(p + 4)) * 0.001;
            z = static_cast<double>(readI32LE(p + 8)) * 0.001;
            refl = p[12];
            tag = p[13];
        } else if (data_type == kDataTypeCartesianLow) {
            x = static_cast<double>(readI16LE(p + 0)) * 0.01;
            y = static_cast<double>(readI16LE(p + 2)) * 0.01;
            z = static_cast<double>(readI16LE(p + 4)) * 0.01;
            refl = p[6];
            tag = p[7];
        } else {
            const double depth_m = static_cast<double>(readU32LE(p + 0)) * 0.001;
            const double theta_rad = static_cast<double>(readU16LE(p + 4)) * 0.01 * math::kDegToRad;
            const double phi_rad   = static_cast<double>(readU16LE(p + 6)) * 0.01 * math::kDegToRad;
            refl = p[8];
            tag = p[9];
            x = depth_m * std::sin(theta_rad) * std::cos(phi_rad);
            y = depth_m * std::sin(theta_rad) * std::sin(phi_rad);
            z = depth_m * std::cos(theta_rad);
        }

        // Reject Livox SDK2 low-confidence noise bits (bits 2..3 spatial noise / bits 0..1 intensity noise)
        if ((tag & 0x0Cu) != 0u) continue;
        const double r = std::sqrt(x * x + y * y + z * z);
        if (r < 0.35 || r > 70.0) continue;

        core::LidarPointXYZIRT pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        pt.intensity = static_cast<float>(refl);
        pt.ring = 0;  // Non-repetitive scan pattern
        pt.timestamp_sec = pkt_t0_sec + static_cast<double>(i) * dt_per_pt;
        pt.range_m = static_cast<float>(r);
        pt.status_flags = tag;
        accum_frame_.points.push_back(pt);
    }

    accum_frame_.timestamp_end_sec = pkt_t0_sec + pkt_span_sec;
    stats_.cloud_packets_valid++;

    if ((accum_frame_.timestamp_end_sec - accum_frame_.timestamp_start_sec) >= frame_period_sec_ &&
        !accum_frame_.points.empty()) {
        out_frame = std::move(accum_frame_);
        out_frame_completed = true;
        stats_.frames_emitted++;
        next_frame_id_++;
        accum_frame_ = core::LidarFrame{};
    }
    return true;
}

bool LivoxMid360Decoder::decodeImuPacket(const uint8_t* data, size_t len,
                                         core::ImuMeasurement& out_imu) {
    // 36-byte header + 24 bytes (6 float32: gyro_x,y,z rad/s, acc_x,y,z in g)
    if (!data || len < kHeaderSize + 24 || data[10] != kDataTypeImu) {
        stats_.imu_packets_malformed++;
        return false;
    }

    const uint64_t ts_ns = readU64LE(data + 28);
    const uint8_t* p = data + kHeaderSize;
    out_imu = core::ImuMeasurement{};
    out_imu.timestamp_sec = static_cast<double>(ts_ns) * 1e-9;
    out_imu.gyro_rads.x = static_cast<double>(readF32LE(p + 0));
    out_imu.gyro_rads.y = static_cast<double>(readF32LE(p + 4));
    out_imu.gyro_rads.z = static_cast<double>(readF32LE(p + 8));
    // Livox Mid-360 reports accelerometer in units of g -> convert to SI m/s^2
    out_imu.accel_mps2.x = static_cast<double>(readF32LE(p + 12)) * kStdGravity;
    out_imu.accel_mps2.y = static_cast<double>(readF32LE(p + 16)) * kStdGravity;
    out_imu.accel_mps2.z = static_cast<double>(readF32LE(p + 20)) * kStdGravity;

    if (!std::isfinite(out_imu.gyro_rads.x) || !std::isfinite(out_imu.accel_mps2.x)) {
        stats_.imu_packets_malformed++;
        return false;
    }
    stats_.imu_packets_valid++;
    return true;
}

std::vector<uint8_t> LivoxMid360Decoder::buildCartesianHighPacket(
    uint64_t timestamp_ns,
    uint32_t time_interval_tenth_us,
    const std::vector<core::LidarPointXYZIRT>& points) {
    const uint16_t n = static_cast<uint16_t>(std::min<size_t>(points.size(), 96));
    const size_t total = kHeaderSize + static_cast<size_t>(n) * 14;
    std::vector<uint8_t> pkt(total, 0);
    pkt[0] = 0;
    writeU16LE(pkt.data() + 1, static_cast<uint16_t>(total));
    writeU16LE(pkt.data() + 3, static_cast<uint16_t>(time_interval_tenth_us));
    writeU16LE(pkt.data() + 5, n);
    pkt[10] = kDataTypeCartesianHigh;
    writeU64LE(pkt.data() + 28, timestamp_ns);

    for (uint16_t i = 0; i < n; ++i) {
        uint8_t* p = pkt.data() + kHeaderSize + static_cast<size_t>(i) * 14;
        writeU32LE(p + 0, static_cast<uint32_t>(static_cast<int32_t>(std::round(points[i].x * 1000.0))));
        writeU32LE(p + 4, static_cast<uint32_t>(static_cast<int32_t>(std::round(points[i].y * 1000.0))));
        writeU32LE(p + 8, static_cast<uint32_t>(static_cast<int32_t>(std::round(points[i].z * 1000.0))));
        p[12] = static_cast<uint8_t>(std::clamp(points[i].intensity, 0.0f, 255.0f));
        p[13] = 0;
    }
    return pkt;
}

std::vector<uint8_t> LivoxMid360Decoder::buildImuPacket(uint64_t timestamp_ns,
                                                        const core::ImuMeasurement& imu) {
    std::vector<uint8_t> pkt(kHeaderSize + 24, 0);
    pkt[0] = 0;
    writeU16LE(pkt.data() + 1, static_cast<uint16_t>(pkt.size()));
    pkt[10] = kDataTypeImu;
    writeU64LE(pkt.data() + 28, timestamp_ns);
    uint8_t* p = pkt.data() + kHeaderSize;
    writeF32LE(p + 0, static_cast<float>(imu.gyro_rads.x));
    writeF32LE(p + 4, static_cast<float>(imu.gyro_rads.y));
    writeF32LE(p + 8, static_cast<float>(imu.gyro_rads.z));
    writeF32LE(p + 12, static_cast<float>(imu.accel_mps2.x / kStdGravity));
    writeF32LE(p + 16, static_cast<float>(imu.accel_mps2.y / kStdGravity));
    writeF32LE(p + 20, static_cast<float>(imu.accel_mps2.z / kStdGravity));
    return pkt;
}

LivoxMid360Adapter::LivoxMid360Adapter(const LivoxMid360NetworkConfig& cfg)
    : cfg_(cfg), decoder_(cfg.frame_period_sec) {}

LivoxMid360Adapter::~LivoxMid360Adapter() {
    stop();
}

void LivoxMid360Adapter::setCallbacks(FrameCallback on_frame, ImuCallback on_imu, RigConfigCallback on_rig_update) {
    on_frame_ = std::move(on_frame);
    on_imu_ = std::move(on_imu);
    on_rig_update_ = std::move(on_rig_update);
}

bool LivoxMid360Adapter::start() {
    if (running_.exchange(true)) return true;
    if (on_rig_update_) {
        on_rig_update_(rigConfig());
    }
    rx_thread_ = std::thread([this]() { receiveLoop(); });
    return true;
}

void LivoxMid360Adapter::stop() {
    if (running_.exchange(false)) {
        if (rx_thread_.joinable()) rx_thread_.join();
    }
}

void LivoxMid360Adapter::receiveLoop() {
#if defined(_WIN32)
    WSADATA wsa{};
    WSAStartup(MAKEWORD(2, 2), &wsa);
    auto closeSock = [](SOCKET s) { if (s != INVALID_SOCKET) closesocket(s); };
    using SockType = SOCKET;
    const SockType kInvalidSock = INVALID_SOCKET;
#else
    auto closeSock = [](int s) { if (s >= 0) close(s); };
    using SockType = int;
    const SockType kInvalidSock = -1;
#endif

    auto bindUdp = [&](uint16_t port) -> SockType {
        SockType s = socket(AF_INET, SOCK_DGRAM, 0);
        if (s == kInvalidSock) return kInvalidSock;
        int reuse = 1;
        setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&reuse), sizeof(reuse));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);
        if (bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            closeSock(s);
            return kInvalidSock;
        }
        return s;
    };

    SockType s_cloud = bindUdp(cfg_.point_cloud_port);
    SockType s_imu = bindUdp(cfg_.imu_port);
    std::vector<uint8_t> buf(2048);

    while (running_.load()) {
        fd_set rfds;
        FD_ZERO(&rfds);
        int max_fd = 0;
        if (s_cloud != kInvalidSock) { FD_SET(s_cloud, &rfds); max_fd = std::max(max_fd, static_cast<int>(s_cloud)); }
        if (s_imu   != kInvalidSock) { FD_SET(s_imu, &rfds);   max_fd = std::max(max_fd, static_cast<int>(s_imu)); }
        timeval tv{0, 20000};
        const int ready = select(max_fd + 1, &rfds, nullptr, nullptr, &tv);
        if (ready <= 0) continue;

        if (s_cloud != kInvalidSock && FD_ISSET(s_cloud, &rfds)) {
            const int n = recv(s_cloud, reinterpret_cast<char*>(buf.data()), static_cast<int>(buf.size()), 0);
            if (n > 0) {
                bool completed = false;
                core::LidarFrame frame;
                if (decoder_.decodePointCloudPacket(buf.data(), static_cast<size_t>(n), completed, frame) &&
                    completed && on_frame_) {
                    on_frame_(frame);
                }
            }
        }
        if (s_imu != kInvalidSock && FD_ISSET(s_imu, &rfds)) {
            const int n = recv(s_imu, reinterpret_cast<char*>(buf.data()), static_cast<int>(buf.size()), 0);
            if (n > 0) {
                core::ImuMeasurement imu;
                if (decoder_.decodeImuPacket(buf.data(), static_cast<size_t>(n), imu) && on_imu_) {
                    on_imu_(imu);
                }
            }
        }
    }

    closeSock(s_cloud);
    closeSock(s_imu);
}

}  // namespace sensors
}  // namespace avscan
