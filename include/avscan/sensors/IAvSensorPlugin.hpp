#pragma once

#include <cstdint>

#ifdef _WIN32
#define AVSLAM_PLUGIN_EXPORT extern "C" __declspec(dllexport)
#else
#define AVSLAM_PLUGIN_EXPORT extern "C" __attribute__((visibility("default")))
#endif

namespace avscan {
namespace sensors {

constexpr uint32_t kAvSlamPluginAbiVersion = 1;

// C-compatible descriptor exported by dynamically loaded vendor sensor plugins
// (e.g. avslam_plugin_realsense.dll/.so, avslam_plugin_livox.dll/.so, avslam_plugin_ouster.dll/.so)
struct AvSensorPluginDescriptor {
    uint32_t abi_version;
    const char* plugin_id;
    const char* display_name;
    const char* vendor;
    const char* license;
    uint32_t supported_channel_mask;  // slam::SensorChannelMask bitmask
    const char* supported_vid_pids;   // comma-separated e.g. "8086:0B3A,8086:0B5C"
};

using AvGetPluginDescriptorFn = const AvSensorPluginDescriptor* (*)();

}  // namespace sensors
}  // namespace avscan
