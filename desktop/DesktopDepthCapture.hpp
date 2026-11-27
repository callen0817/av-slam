#pragma once

#include "avscan/sensors/IDepthProvider.hpp"
#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace avscan {
namespace desktop {

// Zero-SDK cross-platform UVC 16-bit Depth stream capture (Windows Media Foundation / Linux V4L2).
// Targets Orbbec Gemini 336L Depth endpoint (16-bit millimeters) and emits standardized sensors::DepthFrame.
class DesktopDepthCapture {
public:
    using DepthCallback = std::function<void(const sensors::DepthFrame&)>;

    DesktopDepthCapture() = default;
    ~DesktopDepthCapture();

    void setDepthCallback(DepthCallback cb);
    bool start();
    void stop();

    bool isRunning() const { return running_.load(); }
    uint64_t framesDelivered() const { return frames_delivered_.load(); }
    std::string statusSummary() const;

private:
    void captureLoop();

    std::atomic<bool> running_{false};
    std::atomic<uint64_t> frames_delivered_{0};
    std::thread worker_;
    mutable std::mutex mutex_;
    DepthCallback on_depth_;
    std::string status_summary_{"IDLE"};
};

}  // namespace desktop
}  // namespace avscan
