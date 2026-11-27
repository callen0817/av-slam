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

struct InertialSensePortInfo {
    std::string port_name;      // e.g. "COM5" on Windows or "/dev/ttyACM0" on Linux
    std::string friendly_name;  // e.g. "Inertial Sense uINS-3 / IMX-5 (COM5)"
    std::string vid_pid;        // e.g. "0403:6001" or "0483:5740"
};

// Native cross-platform Inertial Sense uINS-3 RUG-3 & IMX-5 Dual RTK Serial Driver.
// Speaks official Inertial Sense ASCII/NMEA protocol ($STPB, $ASCE, $PIMU, $PPIMU, $GNGGA/$GPGGA)
// at 921600 / 115200 baud over Windows COMx and Linux/Jetson /dev/ttyACM* & /dev/ttyUSB*.
// Zero ROS2 or external runtime dependencies; works out of the box on Windows & Linux/Jetson.
class DesktopInertialSenseCapture {
public:
    using ImuCallback = std::function<void(const core::ImuMeasurement&)>;
    using NmeaCallback = std::function<void(const std::string&)>;

    DesktopInertialSenseCapture() = default;
    ~DesktopInertialSenseCapture();

    void setCallbacks(ImuCallback on_imu, NmeaCallback on_nmea);
    std::vector<InertialSensePortInfo> enumerateCandidatePorts() const;

    bool start(const std::string& preferred_port = "");
    void stop();
    bool isRunning() const { return running_.load(); }
    bool isStreamingLive() const { return imu_samples_delivered_.load() > 0; }
    std::string statusSummary() const;
    uint64_t imuSamplesDelivered() const { return imu_samples_delivered_.load(); }
    uint64_t nmeaSentencesDelivered() const { return nmea_sentences_delivered_.load(); }

    static std::string buildNmeaCommand(const std::string& body_without_dollar_or_star);

private:
    void captureLoop(std::string preferred_port);
    void processAsciiLine(const std::string& line);

    mutable std::mutex mutex_;
    ImuCallback on_imu_;
    NmeaCallback on_nmea_;
    std::string status_summary_{"IDLE"};
    std::atomic<bool> running_{false};
    std::atomic<uint64_t> imu_samples_delivered_{0};
    std::atomic<uint64_t> nmea_sentences_delivered_{0};
    std::thread worker_;
};

}  // namespace desktop
}  // namespace avscan
