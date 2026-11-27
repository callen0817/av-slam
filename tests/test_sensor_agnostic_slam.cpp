#include "avscan/sensors/GnssRtkReceiver.hpp"
#include "avscan/sensors/UniversalRigPresets.hpp"
#include "avscan/slam/SensorAgnosticSlamEngine.hpp"
#include <cmath>
#include <cstdio>

#define CHECK(cond) do { if (!(cond)) { std::fprintf(stderr, "CHECK FAILED at line %d: %s\n", __LINE__, #cond); return 1; } } while (0)

using namespace avscan;

static core::LidarFrame makeRoomSweep(uint64_t fid, double t0) {
    core::LidarFrame f;
    f.frame_id = fid;
    f.timestamp_start_sec = t0;
    f.timestamp_end_sec = t0 + 0.10;
    for (int u = -8; u <= 8; ++u) {
        for (int v = -8; v <= 8; ++v) {
            core::LidarPointXYZIRT p1;
            p1.x = 2.0 + u * 0.1; p1.y = v * 0.1; p1.z = -1.0; p1.timestamp_sec = t0 + 0.02; p1.range_m = 2.5f;
            f.points.push_back(p1);
            core::LidarPointXYZIRT p2;
            p2.x = 4.0; p2.y = u * 0.1; p2.z = v * 0.1; p2.timestamp_sec = t0 + 0.05; p2.range_m = 4.0f;
            f.points.push_back(p2);
            core::LidarPointXYZIRT p3;
            p3.x = 2.0 + u * 0.1; p3.y = 3.0; p3.z = v * 0.1; p3.timestamp_sec = t0 + 0.08; p3.range_m = 3.5f;
            f.points.push_back(p3);
        }
    }
    return f;
}

int main() {
    auto catalog = sensors::UniversalRigCatalog::allProfiles();
    CHECK(catalog.size() >= 8);

    // 1. Test NMEA-0183 RTK-Fixed GGA parsing & WGS-84 -> Local ENU projection
    sensors::GnssRtkProcessor gnss;
    sensors::GnssRtkMeasurement fix0, fix1;
    CHECK(gnss.parseNmeaGga("$GNGGA,123519,4045.0000,N,07359.0000,W,4,18,0.7,12.5,M,0.0,M,1.0,0000*00", 1.0, fix0));
    CHECK(fix0.fix_quality == sensors::GnssFixQuality::RTK_FIXED);
    CHECK(fix0.horizontal_sigma_m < 0.05);
    CHECK(fix0.enu_position_m.norm() < 1e-6);

    CHECK(gnss.parseNmeaGga("$GNGGA,123520,4045.0010,N,07359.0000,W,4,18,0.7,12.7,M,0.0,M,1.0,0000*00", 2.0, fix1));
    CHECK(fix1.enu_position_m.y > 1.5 && fix1.enu_position_m.y < 2.2); // ~0.001 arcmin North ~= 1.85 m
    CHECK(std::abs(fix1.enu_position_m.z - 0.2) < 1e-3);

    // 2. Mode A: LiDAR + IMU + RTK-GPS in unified SensorAgnosticSlamEngine
    {
        slam::SensorAgnosticSlamEngine engine;
        for (int i = 0; i < 4; ++i) {
            slam::UniversalSensorBundle b;
            b.sequence_id = static_cast<uint64_t>(i + 1);
            b.timestamp_sec = i * 0.1;
            b.lidar_frame = makeRoomSweep(b.sequence_id, b.timestamp_sec);
            for (int k = 0; k < 20; ++k) {
                core::ImuMeasurement m;
                m.timestamp_sec = b.timestamp_sec + k * 0.005;
                m.accel_mps2 = {0.0, 0.0, 9.80665};
                b.imu_measurements.push_back(m);
            }
            if (i == 3) b.gnss_fix = fix0;
            core::LidarFrame body, world;
            slam::EstimatorMetrics met;
            CHECK(engine.processUniversalBundle(b, body, world, met));
        }
        CHECK(engine.getAgnosticStatus().gnss_factors_applied == 1);
        CHECK(engine.getAgnosticStatus().gnss_rtk_locked);
    }

    // 3. Mode B: Depth-Camera-Only (e.g., Orbbec Gemini 336L Standalone, no LiDAR)
    {
        slam::SensorAgnosticSlamEngine engine;
        sensors::DepthFrame df;
        df.width = 80; df.height = 60;
        df.timestamp_sec = 0.1;
        df.units = sensors::DepthUnits::METERS;
        df.is_metric_anchored = true;
        df.intrinsics = {80, 60, 60.0, 60.0, 40.0, 30.0, {0,0,0,0,0}};
        df.depth_data.assign(80 * 60, 2.5f);
        df.confidence_data.assign(80 * 60, 0.95f);

        slam::UniversalSensorBundle b;
        b.sequence_id = 1;
        b.timestamp_sec = 0.1;
        b.depth_frame = df;
        core::LidarFrame body, world;
        slam::EstimatorMetrics met;
        CHECK(engine.processUniversalBundle(b, body, world, met));
        CHECK(engine.getAgnosticStatus().depth_frames_fused == 1);
        CHECK(!world.points.empty());
    }

    // 4. Mode C: Monocular-Camera-Only (no LiDAR, no Depth)
    {
        slam::SensorAgnosticSlamEngine engine;
        for (int i = 0; i < 3; ++i) {
            core::RgbFrame rgb;
            rgb.frame_id = static_cast<uint64_t>(i + 1);
            rgb.timestamp_sec = i * 0.05;
            rgb.intrinsics = {64, 48, 50.0, 50.0, 32.0, 24.0, {0,0,0,0,0}};
            rgb.gray_bytes.resize(64 * 48, 0);
            rgb.rgb_bytes.resize(64 * 48 * 3, 0);
            for (int y = 0; y < 48; ++y) {
                for (int x = 0; x < 64; ++x) {
                    uint8_t val = (((x + i) / 8 + y / 8) % 2 == 0) ? 200 : 40;
                    rgb.gray_bytes[static_cast<size_t>(y * 64 + x)] = val;
                    rgb.rgb_bytes[static_cast<size_t>((y * 64 + x) * 3 + 0)] = val;
                    rgb.rgb_bytes[static_cast<size_t>((y * 64 + x) * 3 + 1)] = val;
                    rgb.rgb_bytes[static_cast<size_t>((y * 64 + x) * 3 + 2)] = val;
                }
            }
            slam::UniversalSensorBundle b;
            b.sequence_id = rgb.frame_id;
            b.timestamp_sec = rgb.timestamp_sec;
            b.rgb_frame = rgb;
            core::LidarFrame body, world;
            slam::EstimatorMetrics met;
            CHECK(engine.processUniversalBundle(b, body, world, met));
        }
        CHECK(engine.getAgnosticStatus().monocular_frames_tracked == 3);
    }

    std::printf("[PASS] Unified Sensor-Agnostic AV-SLAM Test (LiDAR+IMU+RTK, Depth-Only, Monocular-Only, %zu profiles)\n",
                catalog.size());
    return 0;
}
