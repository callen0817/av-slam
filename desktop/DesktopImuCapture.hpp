#pragma once

#include "avscan/core/Types.hpp"
#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace avscan {
namespace desktop {

struct DetectedImuInfo {
    std::string id;          // e.g. "host_sensor_imu", "orbbec_336l_imu"
    std::string display_name;
    int priority{0};         // higher = preferred when auto-selecting
    bool streaming_supported{true};
};

// Zero-SDK cross-platform Desktop IMU Capture:
// - Windows: native Windows Sensor API (ISensorManager: Accelerometer3D + Gyrometer3D)
//   for Surface / laptop / HID Sensor Collection V2 IMUs, plus Orbbec 336L HID IMU detection.
// - Linux/Jetson: native /sys/bus/iio/devices accelerometer + gyroscope polling.
class DesktopImuCapture {
public:
    using ImuCallback = std::function<void(const core::ImuMeasurement&)>;

    DesktopImuCapture() = default;
    ~DesktopImuCapture();

    void setImuCallback(ImuCallback cb);
    std::vector<DetectedImuInfo> enumerateAvailableImus() const;
    bool start(const std::string& preferred_imu_id = "");
    void stop();
    bool isRunning() const { return running_.load(); }
    std::string statusSummary() const;
    uint64_t samplesDelivered() const { return samples_delivered_.load(); }

private:
    void captureLoop(std::string preferred_imu_id);

    mutable std::mutex mutex_;
    ImuCallback on_imu_;
    std::string status_summary_{"IDLE"};
    std::atomic<bool> running_{false};
    std::atomic<uint64_t> samples_delivered_{0};
    std::thread worker_;
};

}  // namespace desktop
}  // namespace avscan
