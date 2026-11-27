#pragma once

#include "avscan/sensors/HardwareRegistry.hpp"
#include "avscan/sensors/IAvSensorPlugin.hpp"
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace avscan {
namespace desktop {

// Live Hardware Auto-Detector, Online Lookup & User-Confirmed SDK Provisioner.
// Scans USB VID:PID devices, Ethernet LiDARs, and local plugin DLLs/SOs.
// Works 100% offline with plug-and-play fallback; when online, asks user before
// performing capability lookups or downloading vendor SDKs.
class DesktopHardwareManager {
public:
    DesktopHardwareManager();
    ~DesktopHardwareManager();

    // Re-scan connected USB cameras/sensors, Ethernet LiDARs, and installed SDKs
    void refreshConnectedHardware(bool airy_online, const std::string& airy_ip);

    std::vector<sensors::DetectedHardwareDevice> detectedDevices() const;
    bool isOnline() const { return is_online_.load(); }
    bool hasPendingOptimizationPrompt() const;
    bool isProvisioningBusy() const { return provisioning_busy_.load(); }
    std::string provisioningStatus() const;

    // User-confirmed online actions (never invoked without user clicking confirm!)
    void lookupUnknownDeviceOnlineAsync(const std::string& vid_pid, const std::string& friendly_name);
    void installSdkForDeviceAsync(const std::string& device_id);

    std::string appDirectory() const { return app_dir_; }
    std::string pluginsDirectory() const { return plugins_dir_; }

private:
    static bool probeInternetConnectivity();
    std::vector<sensors::HardwareDeviceFingerprint> enumerateUsbVideoAndSensorDevices() const;
    void loadInstalledDynamicPlugins();

    sensors::HardwareRegistry registry_;
    std::string app_dir_;
    std::string plugins_dir_;
    std::string cache_path_;

    mutable std::mutex mutex_;
    std::vector<sensors::DetectedHardwareDevice> detected_devices_;
    std::vector<std::string> loaded_plugin_names_;
    std::string provisioning_status_{"Idle"};

    std::atomic<bool> is_online_{false};
    std::atomic<bool> provisioning_busy_{false};
    std::thread worker_;
};

}  // namespace desktop
}  // namespace avscan
