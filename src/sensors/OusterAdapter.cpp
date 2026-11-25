#include "avscan/sensors/OusterAdapter.hpp"
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

inline uint16_t rdU16LE(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

inline uint32_t rdU32LE(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

inline uint64_t rdU64LE(const uint8_t* p) {
    return static_cast<uint64_t>(rdU32LE(p)) |
           (static_cast<uint64_t>(rdU32LE(p + 4)) << 32);
}

inline float rdF32LE(const uint8_t* p) {
    uint32_t u = rdU32LE(p);
    float f = 0.0f;
    std::memcpy(&f, &u, sizeof(float));
    return f;
}

inline void wrU16LE(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

inline void wrU32LE(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

inline void wrU64LE(uint8_t* p, uint64_t v) {
    wrU32LE(p, static_cast<uint32_t>(v & 0xFFFFFFFFULL));
    wrU32LE(p + 4, static_cast<uint32_t>((v >> 32) & 0xFFFFFFFFULL));
}

inline void wrF32LE(uint8_t* p, float f) {
    uint32_t u = 0;
    std::memcpy(&u, &f, sizeof(float));
    wrU32LE(p, u);
}

}  // namespace

OusterDecoder::OusterDecoder(const OusterSensorConfig& cfg) : cfg_(cfg) {
    rebuildBeamTables();
}

void OusterDecoder::rebuildBeamTables() {
    const size_t n = std::max<size_t>(1, cfg_.pixels_per_column);
    sin_alt_.resize(n);
    cos_alt_.resize(n);
    alt_deg_.resize(n);
    const double half_fov = 0.5 * cfg_.vfov_deg;
    for (size_t ch = 0; ch < n; ++ch) {
        const double frac = (n > 1) ? (static_cast<double>(ch) / static_cast<double>(n - 1)) : 0.5;
        const double deg = half_fov - frac * cfg_.vfov_deg;  // Top-to-bottom channel order
        alt_deg_[ch] = deg;
        sin_alt_[ch] = std::sin(deg * math::kDegToRad);
        cos_alt_[ch] = std::cos(deg * math::kDegToRad);
    }
}

void OusterDecoder::reset() {
    active_frame_id_ = -1;
    next_out_frame_id_ = 1;
    accum_frame_ = core::LidarFrame{};
    stats_ = OusterPacketStats{};
}

bool OusterDecoder::decodeLidarPacket(const uint8_t* data, size_t len,
                                      bool& out_frame_completed,
                                      core::LidarFrame& out_frame) {
    out_frame_completed = false;
    const size_t col_bytes = kColumnHeaderBytes + static_cast<size_t>(cfg_.pixels_per_column) * kChannelBytes + kColumnFooterBytes;
    const size_t expected_bytes = col_bytes * static_cast<size_t>(cfg_.columns_per_packet);
    if (!data || len < expected_bytes) {
        stats_.lidar_packets_malformed++;
        return false;
    }

    const double inv_cols = 1.0 / static_cast<double>(std::max<uint16_t>(1, cfg_.columns_per_frame));

    for (uint16_t col = 0; col < cfg_.columns_per_packet; ++col) {
        const uint8_t* cptr = data + static_cast<size_t>(col) * col_bytes;
        const uint64_t ts_ns = rdU64LE(cptr + 0);
        const uint16_t m_id = rdU16LE(cptr + 8);
        const uint16_t frame_id = rdU16LE(cptr + 10);
        const uint32_t status = rdU32LE(cptr + col_bytes - 4);
        if (status != 0xFFFFFFFFu) continue;

        const double col_t_sec = static_cast<double>(ts_ns) * 1e-9;

        if (active_frame_id_ >= 0 && static_cast<int32_t>(frame_id) != active_frame_id_) {
            if (!accum_frame_.points.empty()) {
                out_frame = std::move(accum_frame_);
                out_frame_completed = true;
                stats_.frames_emitted++;
                next_out_frame_id_++;
                accum_frame_ = core::LidarFrame{};
            }
        }
        active_frame_id_ = static_cast<int32_t>(frame_id);

        if (accum_frame_.points.empty()) {
            accum_frame_.frame_id = next_out_frame_id_;
            accum_frame_.timestamp_start_sec = col_t_sec;
        }

        const double az_rad = -2.0 * math::kPi * (static_cast<double>(m_id) * inv_cols);
        const double cos_az = std::cos(az_rad);
        const double sin_az = std::sin(az_rad);
        const uint8_t* ch_base = cptr + kColumnHeaderBytes;

        for (uint16_t ch = 0; ch < cfg_.pixels_per_column; ++ch) {
            const uint8_t* p = ch_base + static_cast<size_t>(ch) * kChannelBytes;
            const uint32_t range_mm = rdU32LE(p + 0) & 0x000FFFFFu;
            if (range_mm == 0u) continue;
            const double range_m = static_cast<double>(range_mm) * 0.001;
            if (range_m < 0.45 || range_m > 90.0) continue;

            const uint16_t refl = rdU16LE(p + 4);
            const double xy = range_m * cos_alt_[ch];
            core::LidarPointXYZIRT pt;
            pt.x = xy * cos_az + cfg_.lidar_origin_to_beam_m * cos_az;
            pt.y = xy * sin_az + cfg_.lidar_origin_to_beam_m * sin_az;
            pt.z = range_m * sin_alt_[ch];
            pt.intensity = static_cast<float>(std::min<uint16_t>(255u, refl));
            pt.ring = ch;
            pt.timestamp_sec = col_t_sec;
            pt.range_m = static_cast<float>(range_m);
            pt.elevation_deg = static_cast<float>(alt_deg_[ch]);
            accum_frame_.points.push_back(pt);
        }
        accum_frame_.timestamp_end_sec = col_t_sec;
    }

    stats_.lidar_packets_valid++;
    return true;
}

bool OusterDecoder::decodeImuPacket(const uint8_t* data, size_t len,
                                    core::ImuMeasurement& out_imu) {
    // Ouster 48-byte IMU UDP layout:
    // [0..7] sys_ts_ns, [8..15] acc_ts_ns, [16..23] gyro_ts_ns,
    // [24..35] acc_x,y,z (float32 in g), [36..47] gyro_x,y,z (float32 in deg/s)
    if (!data || len < kImuPacketBytes) {
        stats_.imu_packets_malformed++;
        return false;
    }
    const uint64_t ts_ns = rdU64LE(data + 16);
    out_imu = core::ImuMeasurement{};
    out_imu.timestamp_sec = static_cast<double>(ts_ns) * 1e-9;
    out_imu.accel_mps2.x = static_cast<double>(rdF32LE(data + 24)) * kStdGravity;
    out_imu.accel_mps2.y = static_cast<double>(rdF32LE(data + 28)) * kStdGravity;
    out_imu.accel_mps2.z = static_cast<double>(rdF32LE(data + 32)) * kStdGravity;
    out_imu.gyro_rads.x = static_cast<double>(rdF32LE(data + 36)) * math::kDegToRad;
    out_imu.gyro_rads.y = static_cast<double>(rdF32LE(data + 40)) * math::kDegToRad;
    out_imu.gyro_rads.z = static_cast<double>(rdF32LE(data + 44)) * math::kDegToRad;

    if (!std::isfinite(out_imu.accel_mps2.x) || !std::isfinite(out_imu.gyro_rads.x)) {
        stats_.imu_packets_malformed++;
        return false;
    }
    stats_.imu_packets_valid++;
    return true;
}

std::vector<uint8_t> OusterDecoder::buildSyntheticLidarPacket(
    uint16_t frame_id,
    uint16_t start_m_id,
    uint64_t base_ts_ns,
    double nominal_range_m) const {
    const size_t col_bytes = kColumnHeaderBytes + static_cast<size_t>(cfg_.pixels_per_column) * kChannelBytes + kColumnFooterBytes;
    std::vector<uint8_t> pkt(col_bytes * cfg_.columns_per_packet, 0);
    const uint32_t range_mm = static_cast<uint32_t>(std::round(nominal_range_m * 1000.0)) & 0x000FFFFFu;

    for (uint16_t col = 0; col < cfg_.columns_per_packet; ++col) {
        uint8_t* cptr = pkt.data() + static_cast<size_t>(col) * col_bytes;
        wrU64LE(cptr + 0, base_ts_ns + static_cast<uint64_t>(col) * 97656ULL);
        wrU16LE(cptr + 8, static_cast<uint16_t>((start_m_id + col) % cfg_.columns_per_frame));
        wrU16LE(cptr + 10, frame_id);
        wrU32LE(cptr + 12, static_cast<uint32_t>(start_m_id + col) * 88u);

        uint8_t* ch_base = cptr + kColumnHeaderBytes;
        for (uint16_t ch = 0; ch < cfg_.pixels_per_column; ++ch) {
            uint8_t* p = ch_base + static_cast<size_t>(ch) * kChannelBytes;
            wrU32LE(p + 0, range_mm);
            wrU16LE(p + 4, 150u);
        }
        wrU32LE(cptr + col_bytes - 4, 0xFFFFFFFFu);
    }
    return pkt;
}

std::vector<uint8_t> OusterDecoder::buildSyntheticImuPacket(uint64_t timestamp_ns,
                                                            const core::ImuMeasurement& imu) {
    std::vector<uint8_t> pkt(kImuPacketBytes, 0);
    wrU64LE(pkt.data() + 0, timestamp_ns);
    wrU64LE(pkt.data() + 8, timestamp_ns);
    wrU64LE(pkt.data() + 16, timestamp_ns);
    wrF32LE(pkt.data() + 24, static_cast<float>(imu.accel_mps2.x / kStdGravity));
    wrF32LE(pkt.data() + 28, static_cast<float>(imu.accel_mps2.y / kStdGravity));
    wrF32LE(pkt.data() + 32, static_cast<float>(imu.accel_mps2.z / kStdGravity));
    wrF32LE(pkt.data() + 36, static_cast<float>(imu.gyro_rads.x * math::kRadToDeg));
    wrF32LE(pkt.data() + 40, static_cast<float>(imu.gyro_rads.y * math::kRadToDeg));
    wrF32LE(pkt.data() + 44, static_cast<float>(imu.gyro_rads.z * math::kRadToDeg));
    return pkt;
}

OusterAdapter::OusterAdapter(const OusterSensorConfig& cfg) : cfg_(cfg), decoder_(cfg) {}
OusterAdapter::~OusterAdapter() { stop(); }

void OusterAdapter::setCallbacks(FrameCallback on_frame, ImuCallback on_imu, RigConfigCallback on_rig_update) {
    on_frame_ = std::move(on_frame);
    on_imu_ = std::move(on_imu);
    on_rig_update_ = std::move(on_rig_update);
}

bool OusterAdapter::start() {
    if (running_.exchange(true)) return true;
    if (on_rig_update_) on_rig_update_(rigConfig());
    rx_thread_ = std::thread([this]() { receiveLoop(); });
    return true;
}

void OusterAdapter::stop() {
    if (running_.exchange(false)) {
        if (rx_thread_.joinable()) rx_thread_.join();
    }
}

void OusterAdapter::receiveLoop() {
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

    SockType s_lidar = bindUdp(cfg_.lidar_port);
    SockType s_imu = bindUdp(cfg_.imu_port);
    std::vector<uint8_t> buf(32768);

    while (running_.load()) {
        fd_set rfds;
        FD_ZERO(&rfds);
        int max_fd = 0;
        if (s_lidar != kInvalidSock) { FD_SET(s_lidar, &rfds); max_fd = std::max(max_fd, static_cast<int>(s_lidar)); }
        if (s_imu   != kInvalidSock) { FD_SET(s_imu, &rfds);   max_fd = std::max(max_fd, static_cast<int>(s_imu)); }
        timeval tv{0, 20000};
        if (select(max_fd + 1, &rfds, nullptr, nullptr, &tv) <= 0) continue;

        if (s_lidar != kInvalidSock && FD_ISSET(s_lidar, &rfds)) {
            const int n = recv(s_lidar, reinterpret_cast<char*>(buf.data()), static_cast<int>(buf.size()), 0);
            if (n > 0) {
                bool completed = false;
                core::LidarFrame frame;
                if (decoder_.decodeLidarPacket(buf.data(), static_cast<size_t>(n), completed, frame) &&
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

    closeSock(s_lidar);
    closeSock(s_imu);
}

}  // namespace sensors
}  // namespace avscan
