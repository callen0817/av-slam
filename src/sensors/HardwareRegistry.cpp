#include "avscan/sensors/HardwareRegistry.hpp"
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace avscan {
namespace sensors {

namespace {

std::string upperStr(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return s;
}

}  // namespace

HardwareRegistry::HardwareRegistry() {
    populateBuiltInCatalog();
}

void HardwareRegistry::populateBuiltInCatalog() {
    entries_.clear();

    // 1. RoboSense Airy (Native built-in C++17 driver)
    {
        HardwareRegistryEntry e;
        e.device_id = "robosense_airy";
        e.display_name = "RoboSense Airy (96-Beam LiDAR + 200Hz IMU)";
        e.vendor = "RoboSense";
        e.ethernet_hint = "192.168.1.200";
        e.capabilities = slam::SENSOR_LIDAR | slam::SENSOR_IMU;
        e.recommended_profile_id = "airy_standalone";
        e.sdk_name = "AV-SLAM Native Zero-SDK UDP";
        e.sdk_license = "Proprietary Native";
        e.optimization_summary = "Full 96-beam MSOP/DIFOP + 200Hz IMU built in";
        e.built_in_native = true;
        entries_.push_back(e);
    }

    // 2. Orbbec Gemini 336L / 335L / Femto Bolt (OrbbecSDK v2 Apache-2.0)
    {
        HardwareRegistryEntry e;
        e.device_id = "orbbec_gemini_336l";
        e.display_name = "Orbbec Gemini 336L / 330-Series (RGB-D + 200Hz IMU)";
        e.vendor = "Orbbec";
        e.vid_pid_prefixes = {"2BC5:0807", "2BC5:0800", "2BC5:0805", "2BC5:"};
        e.capabilities = slam::SENSOR_DEPTH_CAMERA | slam::SENSOR_RGB_CAMERA | slam::SENSOR_IMU;
        e.recommended_profile_id = "orbbec_gemini_336l_standalone";
        e.sdk_name = "OrbbecSDK v2.10.6";
        e.sdk_license = "Apache-2.0";
        e.sdk_download_url_win64 =
            "https://github.com/orbbec/OrbbecSDK_v2/releases/download/v2.10.6/OrbbecSDK_v2.10.6_202609291804_575347c_win_x64.zip";
        e.sdk_download_url_linux_x64 =
            "https://github.com/orbbec/OrbbecSDK_v2/releases/download/v2.10.6/OrbbecSDK_v2.10.6_202609291004_575347c_linux_x86_64.tar.gz";
        e.sdk_download_url_linux_arm64 =
            "https://github.com/orbbec/OrbbecSDK_v2/releases/download/v2.10.6/OrbbecSDK_v2.10.6_202609291004_575347c_linux_arm64.tar.gz";
#ifdef _WIN32
        e.runtime_marker_file = "OrbbecSDK.dll";
#else
        e.runtime_marker_file = "libOrbbecSDK.so";
#endif
        e.optimization_summary = "Enables 200Hz hardware IMU + calibrated RGB-D alignment";
        e.built_in_native = false;
        entries_.push_back(e);
    }

    // 3. Intel RealSense D435i / D455 / L515
    {
        HardwareRegistryEntry e;
        e.device_id = "intel_realsense_d4xx";
        e.display_name = "Intel RealSense D435i / D455 / L515 (RGB-D + IMU)";
        e.vendor = "Intel";
        e.vid_pid_prefixes = {"8086:0B3A", "8086:0B5C", "8086:0B64", "8086:0B07", "8086:0AD3"};
        e.capabilities = slam::SENSOR_DEPTH_CAMERA | slam::SENSOR_RGB_CAMERA | slam::SENSOR_IMU;
        e.recommended_profile_id = "orbbec_gemini_336l_standalone";
        e.sdk_name = "Intel librealsense2";
        e.sdk_license = "Apache-2.0";
        e.sdk_download_url_win64 = "https://github.com/IntelRealSense/librealsense/releases/latest";
        e.sdk_download_url_linux_x64 = "https://github.com/IntelRealSense/librealsense/releases/latest";
        e.sdk_download_url_linux_arm64 = "https://github.com/IntelRealSense/librealsense/releases/latest";
#ifdef _WIN32
        e.runtime_marker_file = "realsense2.dll";
#else
        e.runtime_marker_file = "librealsense2.so";
#endif
        e.optimization_summary = "Enables RealSense hardware IMU + emitter control + D2C align";
        e.built_in_native = false;
        entries_.push_back(e);
    }

    // 4. Stereolabs ZED 2 / 2i / Mini / X
    {
        HardwareRegistryEntry e;
        e.device_id = "stereolabs_zed";
        e.display_name = "Stereolabs ZED Stereo Camera (RGB-D + IMU)";
        e.vendor = "Stereolabs";
        e.vid_pid_prefixes = {"2B03:F880", "2B03:F780", "2B03:F680", "2B03:"};
        e.capabilities = slam::SENSOR_DEPTH_CAMERA | slam::SENSOR_RGB_CAMERA | slam::SENSOR_IMU;
        e.recommended_profile_id = "monocular_camera_standalone";
        e.sdk_name = "Stereolabs ZED SDK";
        e.sdk_license = "Vendor SDK";
        e.sdk_download_url_win64 = "https://www.stereolabs.com/developers/release";
        e.sdk_download_url_linux_x64 = "https://www.stereolabs.com/developers/release";
        e.sdk_download_url_linux_arm64 = "https://www.stereolabs.com/developers/release";
#ifdef _WIN32
        e.runtime_marker_file = "sl_zed64.dll";
#else
        e.runtime_marker_file = "libsl_zed.so";
#endif
        e.optimization_summary = "Enables neural stereo depth + calibrated ZED IMU";
        e.built_in_native = false;
        entries_.push_back(e);
    }

    // 5. Livox Mid-360 / HAP / Avia (Ethernet)
    {
        HardwareRegistryEntry e;
        e.device_id = "livox_mid360";
        e.display_name = "Livox Mid-360 / Avia (LiDAR + IMU)";
        e.vendor = "Livox";
        e.ethernet_hint = "192.168.1.100";
        e.capabilities = slam::SENSOR_LIDAR | slam::SENSOR_IMU;
        e.recommended_profile_id = "livox_mid360_avscan";
        e.sdk_name = "Livox-SDK2";
        e.sdk_license = "MIT";
        e.sdk_download_url_win64 = "https://github.com/Livox-SDK/Livox-SDK2/releases/latest";
        e.sdk_download_url_linux_x64 = "https://github.com/Livox-SDK/Livox-SDK2/releases/latest";
        e.sdk_download_url_linux_arm64 = "https://github.com/Livox-SDK/Livox-SDK2/releases/latest";
#ifdef _WIN32
        e.runtime_marker_file = "livox_lidar_sdk_shared.dll";
#else
        e.runtime_marker_file = "liblivox_lidar_sdk_shared.so";
#endif
        e.optimization_summary = "Enables Livox Mid-360 UDP pointcloud + 200Hz IMU";
        e.built_in_native = false;
        entries_.push_back(e);
    }

    // 6. Ouster OS0 / OS1 / OS2 (Ethernet)
    {
        HardwareRegistryEntry e;
        e.device_id = "ouster_os";
        e.display_name = "Ouster OS0 / OS1 / OS2 (Spinning LiDAR + IMU)";
        e.vendor = "Ouster";
        e.ethernet_hint = "UDP:7502";
        e.capabilities = slam::SENSOR_LIDAR | slam::SENSOR_IMU;
        e.recommended_profile_id = "ouster_os0_os1_avscan";
        e.sdk_name = "Ouster SDK";
        e.sdk_license = "BSD-3-Clause";
        e.sdk_download_url_win64 = "https://github.com/ouster-lidar/ouster-sdk/releases/latest";
        e.sdk_download_url_linux_x64 = "https://github.com/ouster-lidar/ouster-sdk/releases/latest";
        e.sdk_download_url_linux_arm64 = "https://github.com/ouster-lidar/ouster-sdk/releases/latest";
#ifdef _WIN32
        e.runtime_marker_file = "ouster_client.dll";
#else
        e.runtime_marker_file = "libouster_client.so";
#endif
        e.optimization_summary = "Enables Ouster UDP LiDAR + 100Hz IMU decoding";
        e.built_in_native = false;
        entries_.push_back(e);
    }

    // 7. u-blox ZED-F9P / Emlid Reach RTK-GNSS
    {
        HardwareRegistryEntry e;
        e.device_id = "ublox_rtk_gnss";
        e.display_name = "u-blox ZED-F9P / RTK-GNSS Receiver";
        e.vendor = "u-blox";
        e.vid_pid_prefixes = {"1546:01A9", "1546:01A8", "1546:"};
        e.capabilities = slam::SENSOR_GNSS_RTK;
        e.recommended_profile_id = "outdoor_rtk_multisensor";
        e.sdk_name = "AV-SLAM Native NMEA-0183 / UBX";
        e.sdk_license = "Proprietary Native";
        e.optimization_summary = "Native cm-grade RTK-GNSS geodetic fusion built in";
        e.built_in_native = true;
        entries_.push_back(e);
    }

    // 8. Microsoft Surface / Generic UVC Monocular Camera
    {
        HardwareRegistryEntry e;
        e.device_id = "surface_uvc_camera";
        e.display_name = "Microsoft Surface / Standard UVC Camera";
        e.vendor = "Microsoft";
        e.vid_pid_prefixes = {"045E:0C85", "045E:0991", "046D:"};
        e.capabilities = slam::SENSOR_RGB_CAMERA;
        e.recommended_profile_id = "monocular_camera_standalone";
        e.sdk_name = "AV-SLAM Native UVC + LK Mono-VIO";
        e.sdk_license = "Proprietary Native";
        e.optimization_summary = "Plug-and-play UVC capture + sparse Lucas-Kanade visual odometry";
        e.built_in_native = true;
        entries_.push_back(e);
    }
}

bool HardwareRegistry::isRuntimeInstalled(const HardwareRegistryEntry& e,
                                          const std::string& app_dir,
                                          const std::string& plugins_dir) const {
    if (e.built_in_native) return true;
    if (e.runtime_marker_file.empty()) return false;
    std::error_code ec;
    if (!app_dir.empty() && fs::exists(fs::path(app_dir) / e.runtime_marker_file, ec)) return true;
    if (!plugins_dir.empty() && fs::exists(fs::path(plugins_dir) / e.runtime_marker_file, ec)) return true;
    if (!plugins_dir.empty() && fs::exists(fs::path(plugins_dir) / e.device_id / e.runtime_marker_file, ec)) return true;
    return false;
}

DetectedHardwareDevice HardwareRegistry::classifyDevice(const HardwareDeviceFingerprint& fp,
                                                        const std::string& app_dir,
                                                        const std::string& plugins_dir,
                                                        bool is_online) const {
    DetectedHardwareDevice dev;
    dev.fingerprint = fp;
    const std::string vp = upperStr(fp.vid_pid);

    // First try exact VID:PID match, then vendor prefix match, then Ethernet hint
    const HardwareRegistryEntry* best = nullptr;
    int best_score = 0;

    for (const auto& entry : entries_) {
        if (fp.bus == HardwareBusType::ETHERNET) {
            if (!entry.ethernet_hint.empty() &&
                (fp.endpoint_hint.find(entry.ethernet_hint) != std::string::npos ||
                 upperStr(fp.friendly_name).find(upperStr(entry.vendor)) != std::string::npos)) {
                best = &entry;
                best_score = 100;
                break;
            }
        } else {
            for (const auto& pref : entry.vid_pid_prefixes) {
                std::string up = upperStr(pref);
                if (vp == up && 100 > best_score) {
                    best = &entry;
                    best_score = 100;
                } else if (!up.empty() && up.back() == ':' && vp.rfind(up, 0) == 0 && 60 > best_score) {
                    best = &entry;
                    best_score = 60;
                }
            }
        }
    }

    if (best) {
        dev.matched_registry = true;
        dev.registry_entry = *best;
        dev.sdk_installed = isRuntimeInstalled(*best, app_dir, plugins_dir);
        if (dev.sdk_installed) {
            dev.state = HardwareOptimizationState::OPTIMIZED_NATIVE;
            dev.status_message = "OPTIMIZED (" + best->sdk_name + ")";
        } else if (is_online) {
            dev.state = HardwareOptimizationState::PLUG_AND_PLAY_BASIC;
            dev.status_message = "PLUG-AND-PLAY — " + best->sdk_name + " available online (" + best->optimization_summary + ")";
        } else {
            dev.state = HardwareOptimizationState::OFFLINE_BASIC_FALLBACK;
            dev.status_message = "OFFLINE PLUG-AND-PLAY — connect to internet to install " + best->sdk_name;
        }
    } else {
        dev.matched_registry = false;
        dev.sdk_installed = false;
        if (is_online) {
            dev.state = HardwareOptimizationState::UNKNOWN_NEEDS_LOOKUP;
            dev.status_message = "UNRECOGNIZED DEVICE (" + fp.vid_pid + ") — online capability lookup available";
        } else {
            dev.state = HardwareOptimizationState::OFFLINE_BASIC_FALLBACK;
            dev.status_message = "OFFLINE PLUG-AND-PLAY (" + fp.vid_pid + ") — connect online to identify & optimize";
        }
    }
    return dev;
}

void HardwareRegistry::upsertEntry(const HardwareRegistryEntry& entry) {
    for (auto& existing : entries_) {
        if (existing.device_id == entry.device_id) {
            existing = entry;
            return;
        }
    }
    entries_.push_back(entry);
}

bool HardwareRegistry::loadCacheFile(const std::string& cache_path) {
    std::ifstream in(cache_path);
    if (!in.is_open()) return false;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        std::string id, name, vendor, vidpid, caps_s, sdk;
        if (std::getline(ss, id, '|') && std::getline(ss, name, '|') &&
            std::getline(ss, vendor, '|') && std::getline(ss, vidpid, '|') &&
            std::getline(ss, caps_s, '|') && std::getline(ss, sdk, '|')) {
            HardwareRegistryEntry e;
            e.device_id = id;
            e.display_name = name;
            e.vendor = vendor;
            e.vid_pid_prefixes = {vidpid};
            e.capabilities = static_cast<uint32_t>(std::stoul(caps_s));
            e.sdk_name = sdk;
            e.built_in_native = (sdk.find("Native") != std::string::npos);
            upsertEntry(e);
        }
    }
    return true;
}

bool HardwareRegistry::saveCacheFile(const std::string& cache_path) const {
    std::ofstream out(cache_path, std::ios::trunc);
    if (!out.is_open()) return false;
    out << "# AV-SLAM Local Hardware Registry Cache\n";
    for (const auto& e : entries_) {
        std::string vp = e.vid_pid_prefixes.empty() ? e.ethernet_hint : e.vid_pid_prefixes.front();
        out << e.device_id << "|" << e.display_name << "|" << e.vendor << "|"
            << vp << "|" << e.capabilities << "|" << e.sdk_name << "\n";
    }
    return out.good();
}

std::string HardwareRegistry::busToString(HardwareBusType b) {
    switch (b) {
        case HardwareBusType::USB:           return "USB";
        case HardwareBusType::ETHERNET:      return "Ethernet";
        case HardwareBusType::SERIAL:        return "Serial";
        case HardwareBusType::HOST_INTERNAL: return "Host";
    }
    return "Unknown";
}

std::string HardwareRegistry::stateToString(HardwareOptimizationState s) {
    switch (s) {
        case HardwareOptimizationState::OPTIMIZED_NATIVE:       return "OPTIMIZED";
        case HardwareOptimizationState::PLUG_AND_PLAY_BASIC:    return "UPGRADE AVAILABLE";
        case HardwareOptimizationState::UNKNOWN_NEEDS_LOOKUP:   return "LOOKUP AVAILABLE";
        case HardwareOptimizationState::OFFLINE_BASIC_FALLBACK: return "OFFLINE BASIC";
    }
    return "UNKNOWN";
}

}  // namespace sensors
}  // namespace avscan
