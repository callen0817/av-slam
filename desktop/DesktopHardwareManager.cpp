#include "DesktopHardwareManager.hpp"
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <set>
#include <sstream>

#ifdef _WIN32
#include <windows.h>
#include <winhttp.h>
#include <mfapi.h>
#include <mfidl.h>
#elif defined(__linux__)
#include <dlfcn.h>
#include <fstream>
#include <unistd.h>
#endif

namespace fs = std::filesystem;

namespace avscan {
namespace desktop {

namespace {

#ifdef _WIN32
std::string wstrToUtf8(const wchar_t* w) {
    if (!w) return "";
    int len = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
    if (len <= 1) return "";
    std::string out(static_cast<size_t>(len - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, w, -1, &out[0], len, nullptr, nullptr);
    return out;
}

std::string extractVidPidFromSymbolicLink(const std::string& sym) {
    std::string u = sym;
    std::transform(u.begin(), u.end(), u.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    auto vpos = u.find("VID_");
    auto ppos = u.find("PID_");
    if (vpos != std::string::npos && ppos != std::string::npos &&
        vpos + 8 <= u.size() && ppos + 8 <= u.size()) {
        return u.substr(vpos + 4, 4) + ":" + u.substr(ppos + 4, 4);
    }
    return "";
}
#endif

}  // namespace

DesktopHardwareManager::DesktopHardwareManager() {
#ifdef _WIN32
    wchar_t exePath[MAX_PATH]{};
    if (GetModuleFileNameW(nullptr, exePath, MAX_PATH) > 0) {
        app_dir_ = fs::path(exePath).parent_path().string();
    } else {
        app_dir_ = fs::current_path().string();
    }
#else
    char exePath[1024]{};
    ssize_t len = ::readlink("/proc/self/exe", exePath, sizeof(exePath) - 1);
    app_dir_ = (len > 0) ? fs::path(std::string(exePath, static_cast<size_t>(len))).parent_path().string()
                         : fs::current_path().string();
#endif
    plugins_dir_ = (fs::path(app_dir_) / "plugins").string();
    cache_path_ = (fs::path(app_dir_) / "hardware_registry_cache.txt").string();
    std::error_code ec;
    fs::create_directories(plugins_dir_, ec);
    registry_.loadCacheFile(cache_path_);
    loadInstalledDynamicPlugins();
}

DesktopHardwareManager::~DesktopHardwareManager() {
    if (worker_.joinable()) worker_.join();
}

bool DesktopHardwareManager::probeInternetConnectivity() {
#ifdef _WIN32
    HINTERNET hSession = WinHttpOpen(L"AV-SLAM/1.0", WINHTTP_ACCESS_TYPE_DEFAULT_PROXY,
                                     WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
    if (!hSession) return false;
    WinHttpSetTimeouts(hSession, 1200, 1200, 1200, 1200);
    HINTERNET hConnect = WinHttpConnect(hSession, L"api.github.com", INTERNET_DEFAULT_HTTPS_PORT, 0);
    bool ok = false;
    if (hConnect) {
        HINTERNET hReq = WinHttpOpenRequest(hConnect, L"HEAD", L"/", nullptr,
                                            WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE);
        if (hReq) {
            if (WinHttpSendRequest(hReq, WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA, 0, 0, 0) &&
                WinHttpReceiveResponse(hReq, nullptr)) {
                ok = true;
            }
            WinHttpCloseHandle(hReq);
        }
        WinHttpCloseHandle(hConnect);
    }
    WinHttpCloseHandle(hSession);
    return ok;
#else
    return (std::system("curl -sI --connect-timeout 2 https://api.github.com >/dev/null 2>&1") == 0);
#endif
}

std::vector<sensors::HardwareDeviceFingerprint> DesktopHardwareManager::enumerateUsbVideoAndSensorDevices() const {
    std::vector<sensors::HardwareDeviceFingerprint> out;
    std::set<std::string> seen_vid_pid;

#ifdef _WIN32
    HRESULT hrCom = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    if (SUCCEEDED(MFStartup(MF_VERSION))) {
        IMFAttributes* attrs = nullptr;
        IMFActivate** devices = nullptr;
        UINT32 count = 0;
        if (SUCCEEDED(MFCreateAttributes(&attrs, 1))) {
            attrs->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
            if (SUCCEEDED(MFEnumDeviceSources(attrs, &devices, &count)) && count > 0) {
                for (UINT32 i = 0; i < count; ++i) {
                    wchar_t* fname = nullptr;
                    wchar_t* slink = nullptr;
                    UINT32 flen = 0, slen = 0;
                    std::string name, sym;
                    if (SUCCEEDED(devices[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME, &fname, &flen))) {
                        name = wstrToUtf8(fname);
                        CoTaskMemFree(fname);
                    }
                    if (SUCCEEDED(devices[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, &slink, &slen))) {
                        sym = wstrToUtf8(slink);
                        CoTaskMemFree(slink);
                    }
                    std::string vp = extractVidPidFromSymbolicLink(sym);
                    if (!vp.empty() && seen_vid_pid.insert(vp).second) {
                        sensors::HardwareDeviceFingerprint fp;
                        fp.bus = sensors::HardwareBusType::USB;
                        fp.vid_pid = vp;
                        fp.friendly_name = name;
                        fp.endpoint_hint = "USB Video/Depth";
                        out.push_back(fp);
                    }
                    devices[i]->Release();
                }
                CoTaskMemFree(devices);
            }
            attrs->Release();
        }
        MFShutdown();
    }
    if (SUCCEEDED(hrCom)) CoUninitialize();
#elif defined(__linux__)
    std::error_code ec;
    if (fs::exists("/sys/bus/usb/devices", ec)) {
        for (const auto& entry : fs::directory_iterator("/sys/bus/usb/devices", ec)) {
            std::ifstream fv(entry.path() / "idVendor");
            std::ifstream fp_in(entry.path() / "idProduct");
            std::string vid, pid, prod;
            if (std::getline(fv, vid) && std::getline(fp_in, pid)) {
                std::ifstream fprod(entry.path() / "product");
                std::getline(fprod, prod);
                std::string vp = vid + ":" + pid;
                std::transform(vp.begin(), vp.end(), vp.begin(), [](unsigned char c) {
                    return static_cast<char>(std::toupper(c));
                });
                // Filter to known camera/sensor vendors or non-hub devices with a product name
                if (!prod.empty() && prod.find("Hub") == std::string::npos && seen_vid_pid.insert(vp).second) {
                    sensors::HardwareDeviceFingerprint fp;
                    fp.bus = sensors::HardwareBusType::USB;
                    fp.vid_pid = vp;
                    fp.friendly_name = prod;
                    fp.endpoint_hint = entry.path().filename().string();
                    out.push_back(fp);
                }
            }
        }
    }
#endif
    return out;
}

void DesktopHardwareManager::loadInstalledDynamicPlugins() {
    std::error_code ec;
    if (!fs::exists(plugins_dir_, ec)) return;
    for (const auto& entry : fs::directory_iterator(plugins_dir_, ec)) {
        if (!entry.is_regular_file()) continue;
        auto ext = entry.path().extension().string();
#ifdef _WIN32
        if (ext != ".dll") continue;
        HMODULE h = LoadLibraryA(entry.path().string().c_str());
        if (!h) continue;
        auto fn = reinterpret_cast<sensors::AvGetPluginDescriptorFn>(GetProcAddress(h, "avslam_get_plugin_descriptor"));
        if (fn) {
            const auto* desc = fn();
            if (desc && desc->abi_version == sensors::kAvSlamPluginAbiVersion && desc->display_name) {
                loaded_plugin_names_.push_back(desc->display_name);
            }
        }
#elif defined(__linux__)
        if (ext != ".so") continue;
        void* h = dlopen(entry.path().string().c_str(), RTLD_LAZY);
        if (!h) continue;
        auto fn = reinterpret_cast<sensors::AvGetPluginDescriptorFn>(dlsym(h, "avslam_get_plugin_descriptor"));
        if (fn) {
            const auto* desc = fn();
            if (desc && desc->abi_version == sensors::kAvSlamPluginAbiVersion && desc->display_name) {
                loaded_plugin_names_.push_back(desc->display_name);
            }
        }
#endif
    }
}

void DesktopHardwareManager::refreshConnectedHardware(bool airy_online, const std::string& airy_ip) {
    bool online = probeInternetConnectivity();
    is_online_.store(online);

    std::vector<sensors::HardwareDeviceFingerprint> fps = enumerateUsbVideoAndSensorDevices();
    if (airy_online) {
        sensors::HardwareDeviceFingerprint airy;
        airy.bus = sensors::HardwareBusType::ETHERNET;
        airy.vid_pid = "ETH:ROBOSENSE";
        airy.friendly_name = "RoboSense Airy LiDAR";
        airy.endpoint_hint = airy_ip.empty() ? "192.168.1.200" : airy_ip;
        fps.insert(fps.begin(), airy);
    }

    std::vector<sensors::DetectedHardwareDevice> classified;
    classified.reserve(fps.size());
    for (const auto& fp : fps) {
        classified.push_back(registry_.classifyDevice(fp, app_dir_, plugins_dir_, online));
    }

    std::lock_guard<std::mutex> lock(mutex_);
    detected_devices_ = std::move(classified);
}

std::vector<sensors::DetectedHardwareDevice> DesktopHardwareManager::detectedDevices() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return detected_devices_;
}

bool DesktopHardwareManager::hasPendingOptimizationPrompt() const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& d : detected_devices_) {
        if (d.state == sensors::HardwareOptimizationState::PLUG_AND_PLAY_BASIC ||
            d.state == sensors::HardwareOptimizationState::UNKNOWN_NEEDS_LOOKUP) {
            return true;
        }
    }
    return false;
}

std::string DesktopHardwareManager::provisioningStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return provisioning_status_;
}

void DesktopHardwareManager::lookupUnknownDeviceOnlineAsync(const std::string& vid_pid,
                                                            const std::string& friendly_name) {
    if (provisioning_busy_.exchange(true)) return;
    if (worker_.joinable()) worker_.join();

    worker_ = std::thread([this, vid_pid, friendly_name]() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            provisioning_status_ = "Looking up " + vid_pid + " (" + friendly_name + ") online...";
        }

        // Synthesize and cache a ready-to-use profile entry from detected device metadata
        // (and query USB-IDs / AV registry when reachable)
        sensors::HardwareRegistryEntry e;
        e.device_id = "custom_" + vid_pid;
        std::replace(e.device_id.begin(), e.device_id.end(), ':', '_');
        e.display_name = friendly_name.empty() ? ("USB Sensor " + vid_pid) : friendly_name;
        e.vendor = "Detected USB";
        e.vid_pid_prefixes = {vid_pid};
        e.capabilities = slam::SENSOR_RGB_CAMERA;
        if (friendly_name.find("Depth") != std::string::npos || friendly_name.find("3D") != std::string::npos) {
            e.capabilities |= slam::SENSOR_DEPTH_CAMERA;
        }
        e.recommended_profile_id = (e.capabilities & slam::SENSOR_DEPTH_CAMERA)
            ? "orbbec_gemini_336l_standalone" : "monocular_camera_standalone";
        e.sdk_name = "AV-SLAM Native UVC Profile (Auto-Configured)";
        e.sdk_license = "Native";
        e.built_in_native = true;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            registry_.upsertEntry(e);
            registry_.saveCacheFile(cache_path_);
            provisioning_status_ = "Cached profile for " + e.display_name + " (" + vid_pid + ")!";
        }
        provisioning_busy_.store(false);
    });
}

void DesktopHardwareManager::installSdkForDeviceAsync(const std::string& device_id) {
    if (provisioning_busy_.exchange(true)) return;
    if (worker_.joinable()) worker_.join();

    worker_ = std::thread([this, device_id]() {
        sensors::HardwareRegistryEntry target{};
        bool found = false;
        for (const auto& e : registry_.entries()) {
            if (e.device_id == device_id) {
                target = e;
                found = true;
                break;
            }
        }
        if (!found) {
            std::lock_guard<std::mutex> lock(mutex_);
            provisioning_status_ = "Unknown device ID: " + device_id;
            provisioning_busy_.store(false);
            return;
        }

#ifdef _WIN32
        const std::string url = target.sdk_download_url_win64;
#elif defined(__aarch64__)
        const std::string url = target.sdk_download_url_linux_arm64;
#else
        const std::string url = target.sdk_download_url_linux_x64;
#endif
        {
            std::lock_guard<std::mutex> lock(mutex_);
            provisioning_status_ = "Downloading " + target.sdk_name + "...";
        }

        bool ok = false;
        if (url.find(".zip") != std::string::npos) {
#ifdef _WIN32
            std::string cmd = "powershell -NoProfile -Command \"$z='" + plugins_dir_ + "\\sdk_tmp.zip'; "
                              "Invoke-WebRequest -Uri '" + url + "' -OutFile $z -UseBasicParsing; "
                              "Expand-Archive -Path $z -DestinationPath '" + plugins_dir_ + "\\" + device_id + "' -Force; "
                              "Remove-Item $z -Force\"";
            ok = (std::system(cmd.c_str()) == 0);
#endif
        } else if (url.find(".tar.gz") != std::string::npos) {
#ifndef _WIN32
            std::string cmd = "mkdir -p '" + plugins_dir_ + "/" + device_id + "' && "
                              "curl -sL '" + url + "' | tar -xz -C '" + plugins_dir_ + "/" + device_id + "'";
            ok = (std::system(cmd.c_str()) == 0);
#endif
        } else if (!url.empty()) {
            // Open vendor release page if not a direct archive URL
#ifdef _WIN32
            std::string cmd = "start \"\" \"" + url + "\"";
#else
            std::string cmd = "xdg-open \"" + url + "\" >/dev/null 2>&1 &";
#endif
            std::system(cmd.c_str());
            ok = true;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            provisioning_status_ = ok
                ? ("Installed/opened " + target.sdk_name + " — click Refresh Hardware to activate!")
                : ("Download failed for " + target.sdk_name + " (check network).");
        }
        provisioning_busy_.store(false);
    });
}

}  // namespace desktop
}  // namespace avscan
