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

// Zero-external-SDK cross-platform UVC RGB camera capture (Windows Media Foundation / Linux V4L2).
// Automatically prefers Orbbec Gemini 336L / ScanAR USB-C cameras when connected.
class DesktopCameraCapture {
public:
    using FrameCallback = std::function<void(const core::RgbFrame&)>;

    DesktopCameraCapture() = default;
    ~DesktopCameraCapture();

    void setPreferredCameraHint(const std::string& name_hint,
                                const core::CameraIntrinsics& fallback_intrinsics);
    void setFrameCallback(FrameCallback cb);

    std::vector<std::string> enumerateCameraNames() const;

    bool start(int target_fps = 30);
    void stop();

    bool isRunning() const { return running_.load(); }
    uint64_t framesDelivered() const { return frames_delivered_.load(); }
    double measuredFps() const { return measured_fps_.load(); }
    std::string activeCameraName() const;
    std::string statusSummary() const;

private:
    void captureLoop();

    std::atomic<bool> running_{false};
    std::atomic<uint64_t> frames_delivered_{0};
    std::atomic<double> measured_fps_{0.0};
    std::thread worker_;

    mutable std::mutex mutex_;
    std::string preferred_hint_{"Gemini"};
    core::CameraIntrinsics fallback_intrinsics_{};
    std::string active_camera_name_{"None"};
    std::string status_summary_{"IDLE"};
    FrameCallback on_frame_;
    int target_fps_{30};
};

}  // namespace desktop
}  // namespace avscan
