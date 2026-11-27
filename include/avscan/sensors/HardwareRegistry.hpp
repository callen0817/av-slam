#pragma once

#include "avscan/slam/SensorAgnosticSlamEngine.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace avscan {
namespace sensors {

enum class HardwareBusType {
    USB = 0,
    ETHERNET = 1,
    SERIAL = 2,
    HOST_INTERNAL = 3
};

enum class HardwareOptimizationState {
    OPTIMIZED_NATIVE = 0,          // Full native or installed SDK support active
    PLUG_AND_PLAY_BASIC = 1,       // Working via generic UVC/OS driver; SDK upgrade available
    UNKNOWN_NEEDS_LOOKUP = 2,      // Unrecognized VID:PID or Ethernet sensor; online lookup available
    OFFLINE_BASIC_FALLBACK = 3     // Plug-and-play active, offline so cannot check SDK/lookup
};

struct HardwareDeviceFingerprint {
    HardwareBusType bus{HardwareBusType::USB};
    std::string vid_pid;        // Uppercase "XXXX:YYYY" for USB, or "ETH:IP" / "COMx"
    std::string friendly_name;
    std::string endpoint_hint;  // e.g. IP address, COM port, or USB interface
};

struct HardwareRegistryEntry {
    std::string device_id;
    std::string display_name;
    std::string vendor;
    std::vector<std::string> vid_pid_prefixes;  // Exact "2BC5:0807" or vendor prefix "2BC5:"
    std::string ethernet_hint;                  // e.g. "192.168.1.200" or "MSOP:6699"
    uint32_t capabilities{slam::SENSOR_NONE};
    std::string recommended_profile_id;
    std::string sdk_name;                       // e.g. "OrbbecSDK v2", "librealsense2", "Built-in Native"
    std::string sdk_license;                    // e.g. "Apache-2.0", "MIT", "Proprietary Native"
    std::string sdk_download_url_win64;
    std::string sdk_download_url_linux_x64;
    std::string sdk_download_url_linux_arm64;
    std::string runtime_marker_file;            // e.g. "OrbbecSDK.dll" / "libOrbbecSDK.so"
    std::string optimization_summary;
    bool built_in_native{false};
};

struct DetectedHardwareDevice {
    HardwareDeviceFingerprint fingerprint;
    bool matched_registry{false};
    HardwareRegistryEntry registry_entry;
    HardwareOptimizationState state{HardwareOptimizationState::UNKNOWN_NEEDS_LOOKUP};
    bool sdk_installed{false};
    std::string status_message;
};

// Cross-platform Hardware Capability Registry (reusable on Windows, Linux/Jetson, and iOS).
// Ships with a built-in offline catalog and can merge/cache online registry JSON updates.
class HardwareRegistry {
public:
    HardwareRegistry();

    const std::vector<HardwareRegistryEntry>& entries() const { return entries_; }

    // Match a live hardware fingerprint against the catalog + local installed SDK directory
    DetectedHardwareDevice classifyDevice(const HardwareDeviceFingerprint& fp,
                                          const std::string& app_dir,
                                          const std::string& plugins_dir,
                                          bool is_online) const;

    // Persist/load user-discovered or online-fetched hardware entries
    bool loadCacheFile(const std::string& cache_path);
    bool saveCacheFile(const std::string& cache_path) const;
    void upsertEntry(const HardwareRegistryEntry& entry);

    static std::string busToString(HardwareBusType b);
    static std::string stateToString(HardwareOptimizationState s);

private:
    void populateBuiltInCatalog();
    bool isRuntimeInstalled(const HardwareRegistryEntry& e,
                            const std::string& app_dir,
                            const std::string& plugins_dir) const;

    std::vector<HardwareRegistryEntry> entries_;
};

}  // namespace sensors
}  // namespace avscan
