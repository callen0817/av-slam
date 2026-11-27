#include "DesktopSessionController.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace avscan {
namespace desktop {

namespace {

constexpr double kDesktopLiveVoxelM = 0.03;

inline slam::VoxelKey desktopVoxelKey(double x, double y, double z) {
    return {
        static_cast<int64_t>(std::floor(x / kDesktopLiveVoxelM)),
        static_cast<int64_t>(std::floor(y / kDesktopLiveVoxelM)),
        static_cast<int64_t>(std::floor(z / kDesktopLiveVoxelM))
    };
}

void mergeIntoLiveCloud(
    const std::vector<core::LidarPointXYZIRT>& pts,
    size_t stride,
    std::vector<core::LidarPointXYZIRT>& cloud,
    std::unordered_map<slam::VoxelKey, size_t, slam::VoxelKeyHash>& vidx) {
    const size_t step = std::max<size_t>(1, stride);
    for (size_t i = 0; i < pts.size(); i += step) {
        const auto& p = pts[i];
        auto key = desktopVoxelKey(p.x, p.y, p.z);
        auto it = vidx.find(key);
        if (it == vidx.end()) {
            vidx.emplace(key, cloud.size());
            cloud.push_back(p);
        } else if (p.colorized && !cloud[it->second].colorized) {
            cloud[it->second].r = p.r;
            cloud[it->second].g = p.g;
            cloud[it->second].b = p.b;
            cloud[it->second].colorized = true;
        }
    }
}

cslam::PeerPlatform detectDesktopPlatform() {
#ifdef _WIN32
    return cslam::PeerPlatform::WINDOWS;
#elif defined(__APPLE__)
    return cslam::PeerPlatform::MACOS;
#else
    std::ifstream f("/proc/device-tree/model");
    std::string model;
    if (std::getline(f, model)) {
        if (model.find("Jetson") != std::string::npos || model.find("Orin") != std::string::npos) {
            return cslam::PeerPlatform::JETSON;
        }
    }
    return cslam::PeerPlatform::LINUX;
#endif
}

std::string timestampStamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm_val{};
#ifdef _WIN32
    localtime_s(&tm_val, &tt);
#else
    localtime_r(&tt, &tm_val);
#endif
    char buf[32]{};
    std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d",
                  tm_val.tm_year + 1900, tm_val.tm_mon + 1, tm_val.tm_mday,
                  tm_val.tm_hour, tm_val.tm_min, tm_val.tm_sec);
    return buf;
}

}  // namespace

double DesktopSessionController::steadyNowSec() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

std::string DesktopSessionController::sanitizeId(const std::string& raw, const std::string& fallback) {
    std::string out;
    for (char c : raw) {
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-') {
            out.push_back(c);
        } else if (c == ' ' || c == '.') {
            out.push_back('_');
        }
    }
    return out.empty() ? fallback : out;
}

bool DesktopSessionController::scanDirHasExistingData(const std::string& dir) {
    if (dir.empty()) return false;
    std::error_code ec;
    fs::path p(dir);
    if (!fs::exists(p, ec)) return false;
    if (fs::exists(p / "session.wal", ec) && fs::file_size(p / "session.wal", ec) > 8) return true;
    if (fs::exists(p / "pose_graph.factors", ec)) return true;
    if (fs::exists(p / "raw_capture.avraw", ec) && fs::file_size(p / "raw_capture.avraw", ec) > 0) return true;
    if (fs::exists(p / "chunks", ec)) {
        for (const auto& entry : fs::directory_iterator(p / "chunks", ec)) {
            if (entry.is_regular_file()) return true;
        }
    }
    for (const auto& entry : fs::directory_iterator(p, ec)) {
        if (!entry.is_regular_file()) continue;
        auto ext = entry.path().extension().string();
        if (ext == ".ply" || ext == ".pcd" || ext == ".e57") return true;
    }
    return false;
}

std::string DesktopSessionController::autoScanLabelForActiveProfile() const {
    const bool en_lidar = (sensor_toggles_.lidar_mode != SensorSelectionMode::DISABLED_NONE);
    const bool en_rgb   = (sensor_toggles_.rgb_mode != SensorSelectionMode::DISABLED_NONE);
    const bool en_depth = (sensor_toggles_.depth_mode != SensorSelectionMode::DISABLED_NONE);

    std::string cam_tag = "Cam";
    if (sensor_toggles_.rgb_mode == SensorSelectionMode::ENABLED_SPECIFIC) {
        const auto& n = sensor_toggles_.rgb_device_name;
        if (n.find("Surface") != std::string::npos || n.find("Webcam") != std::string::npos ||
            (n.find("Gemini") == std::string::npos && n.find("Orbbec") == std::string::npos)) {
            cam_tag = "Webcam";
        } else {
            cam_tag = "336L";
        }
    } else if (en_depth) {
        cam_tag = "336L";
    } else {
        cam_tag = "Webcam";
    }

    if (en_lidar && en_depth) return "Scan_Airy_336L";
    if (en_lidar && en_rgb)   return "Scan_Airy_" + cam_tag;
    if (en_lidar)             return "Scan_Airy_Only";
    if (en_depth)             return "Scan_336L_Only";
    if (en_rgb)               return "Scan_" + cam_tag + "_Only";
    return "Scan_Agnostic";
}

DesktopSessionController::DesktopSessionController() {
    backend::GicpVerificationConfig gicpCfg;
    gicpCfg.voxel_resolution_m = 0.40;
    gicpCfg.max_correspondence_dist_m = 0.30;
    gicpCfg.max_iterations = 20;
    gicpCfg.min_inlier_overlap_ratio = 0.50;
    gicpCfg.max_fitness_rmse_m = 0.07;
    gicpCfg.max_source_points = 4000;
    gicpCfg.allow_centroid_fallback = false;
    gicp_verifier_ = backend::GicpVerifier(gicpCfg);

    discovery_ = std::make_unique<sensors::AiryDiscovery>(net_config_);
    udp_receiver_ = std::make_unique<sensors::AiryUdpReceiver>(net_config_);

    const auto plat = detectDesktopPlatform();
    const std::string plat_name = cslam::CollaborativeSlamEngine::platformToString(plat);
    cslam_engine_ = std::make_unique<cslam::CollaborativeSlamEngine>(
        "desktop_" + timestampStamp(),
        "AV-SLAM " + plat_name,
        plat,
        cslam::PeerApp::AV_SLAM);
    cslam_transport_ = std::make_unique<cslam::CollaborativeLanTransport>();

    setupCallbacks();

    // Default to ScanAR Orbbec Gemini 336L + Airy profile on Desktop if available
    const auto& profiles = core::DeviceProfileRegistry::instance().allProfiles();
    bool set_scanar = false;
    for (const auto& p : profiles) {
        if (p.profile_id.find("scanar") != std::string::npos || p.uses_external_usb_camera) {
            setActiveDeviceProfile(p.profile_id);
            set_scanar = true;
            break;
        }
    }
    if (!set_scanar && !profiles.empty()) {
        setActiveDeviceProfile(profiles.front().profile_id);
    }

    auto existing = listProjects();
    if (!existing.empty()) {
        openProject(existing.front().id);
    } else {
        createProject("ScanAR_Project_01", "Default Site", "");
    }
    refreshDetectedHardware();
}

void DesktopSessionController::refreshDetectedHardware() {
    bool airy_ok = false;
    if (discovery_) {
        auto diag = discovery_->runDiagnostics(120, 120);
        airy_ok = diag.adapter_detected || diag.difop_received || diag.http_reachable;
    }
    hw_manager_.refreshConnectedHardware(airy_ok, net_config_.sensor_ip);
}

DesktopSessionController::~DesktopSessionController() {
    setCollaborativeSlamEnabled(false);
    stopScanning();
}

void DesktopSessionController::setupCallbacks() {
    udp_receiver_->setCallbacks(
        [this](const core::LidarFrame& frame) {
            double now = steadyNowSec();
            last_lidar_arrival_sec_ = now;
            if (!is_terrestrial_active_.load()) {
                synchronizer_.pushLidarFrame(frame, now);
            } else if (station_manager_) {
                station_manager_->pushLidarFrame(frame);
            }
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!is_scanning_.load() && !is_terrestrial_active_.load()) {
                live_preview_cloud_ = frame.points;
            }
        },
        [this](const core::ImuMeasurement& imu) {
            if (sensor_toggles_.imu_mode == SensorSelectionMode::DISABLED_NONE) return;
            if (sensor_toggles_.imu_mode == SensorSelectionMode::ENABLED_SPECIFIC &&
                sensor_toggles_.imu_source_id != "airy_imu") {
                return;
            }
            // In AUTO mode, yield to higher-priority Inertial Sense uINS-3/IMX-5 (Prio 120) if live!
            if (sensor_toggles_.imu_mode == SensorSelectionMode::AUTO &&
                inertial_sense_.isStreamingLive()) {
                return;
            }
            double now = steadyNowSec();
            if (!is_terrestrial_active_.load()) {
                synchronizer_.pushImu(imu, now);
            } else if (station_manager_) {
                station_manager_->pushImuMeasurement(imu);
            }
        },
        [this](const core::AiryFactoryCalibration& calib) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            const double dRotDeg =
                (factory_calib_.T_imu_lidar.so3.inverse() * calib.T_imu_lidar.so3).log().norm() * math::kRadToDeg;
            factory_calib_ = calib;
            agnostic_engine_.coreEngine().setFactoryCalibration(calib);
            if (dRotDeg > 5.0 && is_scanning_.load() && pose_graph_.getSubmaps().size() <= 5) {
                pose_graph_.clear();
                sc_db_.clear();
                cached_global_cloud_.clear();
                live_voxel_index_.clear();
                keyframe_traveled_dist_.clear();
                next_keyframe_id_ = 1;
                total_traveled_dist_m_ = 0.0;
                unclosed_dist_m_ = 0.0;
                has_prev_odom_pos_ = false;
            }
        });

    camera_.setFrameCallback([this](const core::RgbFrame& rgb) {
        if (!is_terrestrial_active_.load()) {
            synchronizer_.pushRgbFrame(rgb);
            std::lock_guard<std::mutex> lk(mono_rgb_mutex_);
            standalone_rgb_queue_.push_back(rgb);
            while (standalone_rgb_queue_.size() > 4) standalone_rgb_queue_.pop_front();
        }
    });

    depth_capture_.setDepthCallback([this](const sensors::DepthFrame& df) {
        if (!is_terrestrial_active_.load()) {
            std::lock_guard<std::mutex> lk(mono_rgb_mutex_);
            standalone_depth_queue_.push_back(df);
            while (standalone_depth_queue_.size() > 4) standalone_depth_queue_.pop_front();
        }
    });

    host_imu_.setImuCallback([this](const core::ImuMeasurement& imu) {
        if (sensor_toggles_.imu_mode == SensorSelectionMode::DISABLED_NONE) return;
        if (sensor_toggles_.imu_mode == SensorSelectionMode::ENABLED_SPECIFIC &&
            (sensor_toggles_.imu_source_id == "airy_imu" || sensor_toggles_.imu_source_id == "inertial_sense_imu")) {
            return;
        }
        // In AUTO mode, yield to higher-priority Inertial Sense (Prio 120) if live!
        if (sensor_toggles_.imu_mode == SensorSelectionMode::AUTO &&
            inertial_sense_.isStreamingLive()) {
            return;
        }
        if (!is_terrestrial_active_.load()) {
            std::lock_guard<std::mutex> lk(mono_rgb_mutex_);
            standalone_imu_queue_.push_back(imu);
            while (standalone_imu_queue_.size() > 120) standalone_imu_queue_.pop_front();
            // If user explicitly chose Orbbec/Host IMU while LiDAR is running, also feed synchronizer
            if (sensor_toggles_.imu_mode == SensorSelectionMode::ENABLED_SPECIFIC &&
                sensor_toggles_.imu_source_id != "airy_imu") {
                synchronizer_.pushImu(imu, steadyNowSec());
            }
        }
    });

    // Priority-1 (120) Inertial Sense uINS-3 RUG-3 / IMX-5 Dual RTK IMU + NMEA RTK callback
    inertial_sense_.setCallbacks(
        [this](const core::ImuMeasurement& imu) {
            if (sensor_toggles_.imu_mode == SensorSelectionMode::DISABLED_NONE) return;
            if (sensor_toggles_.imu_mode == SensorSelectionMode::ENABLED_SPECIFIC &&
                sensor_toggles_.imu_source_id != "inertial_sense_imu") {
                return;
            }
            double now = steadyNowSec();
            if (!is_terrestrial_active_.load()) {
                synchronizer_.pushImu(imu, now);
                std::lock_guard<std::mutex> lk(mono_rgb_mutex_);
                standalone_imu_queue_.push_back(imu);
                while (standalone_imu_queue_.size() > 160) standalone_imu_queue_.pop_front();
            } else if (station_manager_) {
                station_manager_->pushImuMeasurement(imu);
            }
        },
        [this](const std::string& nmea_gga) {
            if (sensor_toggles_.gnss_mode != SensorSelectionMode::DISABLED_NONE) {
                injectNmeaGnssSentence(nmea_gga);
            }
        });
}

std::string DesktopSessionController::projectsRootDir() const {
    fs::path root = fs::current_path() / "AV_SLAM_Projects";
    std::error_code ec;
    fs::create_directories(root, ec);
    return root.string();
}

std::vector<DesktopProjectSummary> DesktopSessionController::listProjects() const {
    std::vector<DesktopProjectSummary> out;
    fs::path root(projectsRootDir());
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(root, ec)) {
        if (!entry.is_directory()) continue;
        DesktopProjectSummary s;
        s.id = entry.path().filename().string();
        s.name = s.id;
        s.path = entry.path().string();
        fs::path scans_dir = entry.path() / "scans";
        if (fs::exists(scans_dir, ec)) {
            for (const auto& se : fs::directory_iterator(scans_dir, ec)) {
                if (se.is_directory()) s.scan_count++;
            }
        }
        out.push_back(s);
    }
    return out;
}

bool DesktopSessionController::createProject(const std::string& name, const std::string& site, const std::string& notes) {
    std::string display = name.empty() ? "Untitled_Project" : name;
    std::string pid = sanitizeId(display, "PROJECT") + "_" + timestampStamp();
    fs::path pdir = fs::path(projectsRootDir()) / pid;
    std::error_code ec;
    fs::create_directories(pdir / "scans", ec);
    std::ofstream mf((pdir / "project_manifest.json").string());
    mf << "{\n  \"id\": \"" << pid << "\",\n  \"name\": \"" << display
       << "\",\n  \"site\": \"" << site << "\",\n  \"notes\": \"" << notes << "\"\n}\n";
    active_project_name_ = display;
    return openProject(pid);
}

bool DesktopSessionController::openProject(const std::string& project_id) {
    fs::path pdir = fs::path(projectsRootDir()) / project_id;
    if (!fs::exists(pdir)) return false;
    active_project_id_ = project_id;
    if (active_project_name_.empty()) active_project_name_ = project_id;
    auto scans = listScansInActiveProject();
    if (!scans.empty()) {
        return selectScan(scans.front().id);
    }
    return createAndSelectScan("Scan 01", "Level 01");
}

std::vector<DesktopScanSummary> DesktopSessionController::listScansInActiveProject() const {
    std::vector<DesktopScanSummary> out;
    if (active_project_id_.empty()) return out;
    fs::path sdir = fs::path(projectsRootDir()) / active_project_id_ / "scans";
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(sdir, ec)) {
        if (!entry.is_directory()) continue;
        DesktopScanSummary s;
        s.id = entry.path().filename().string();
        s.name = s.id;
        s.floor_level = "Level 01";
        s.mode = "Mobile SLAM";
        s.path = entry.path().string();
        out.push_back(s);
    }
    return out;
}

bool DesktopSessionController::createAndSelectScan(const std::string& scan_name, const std::string& floor_level) {
    if (active_project_id_.empty()) createProject("ScanAR_Project", "", "");
    std::string disp = scan_name.empty() ? "Scan" : scan_name;
    std::string base_sid = sanitizeId(disp, "SCAN") + "_" + timestampStamp();
    std::string sid = base_sid;
    fs::path scans_root = fs::path(projectsRootDir()) / active_project_id_ / "scans";
    fs::path sdir = scans_root / sid;
    std::error_code ec;
    int suffix = 2;
    while (fs::exists(sdir, ec)) {
        sid = base_sid + "_" + std::to_string(suffix++);
        sdir = scans_root / sid;
    }
    fs::create_directories(sdir, ec);
    std::ofstream mf((sdir / "scan_manifest.json").string());
    mf << "{\n  \"id\": \"" << sid << "\",\n  \"name\": \"" << disp
       << "\",\n  \"profile\": \"" << active_universal_profile_id_
       << "\",\n  \"floorLevel\": \"" << (floor_level.empty() ? "Level 01" : floor_level) << "\"\n}\n";
    active_scan_name_ = disp;
    active_scan_floor_level_ = floor_level.empty() ? "Level 01" : floor_level;
    return selectScan(sid);
}

bool DesktopSessionController::selectScan(const std::string& scan_id) {
    if (is_scanning_.load()) stopScanning();
    fs::path sdir = fs::path(projectsRootDir()) / active_project_id_ / "scans" / scan_id;
    if (!fs::exists(sdir)) return false;
    active_scan_id_ = scan_id;
    active_scan_name_ = scan_id;
    active_scan_dir_ = sdir.string();
    current_scan_used_ = scanDirHasExistingData(active_scan_dir_);

    std::lock_guard<std::mutex> lock(state_mutex_);
    agnostic_engine_.reset();
    gnss_processor_.reset();
    synchronizer_.reset();
    pose_graph_.clear();
    sc_db_.clear();
    cached_global_cloud_.clear();
    live_voxel_index_.clear();
    next_keyframe_id_ = 1;
    total_traveled_dist_m_ = 0.0;
    unclosed_dist_m_ = 0.0;
    has_prev_odom_pos_ = false;
    return true;
}

bool DesktopSessionController::setActiveDeviceProfile(const std::string& profile_id) {
    if (!core::DeviceProfileRegistry::instance().setActiveProfile(profile_id)) return false;
    const auto active = core::DeviceProfileRegistry::instance().activeProfile();
    camera_.setPreferredCameraHint(active.preferred_usb_camera_name.empty() ? "Gemini" : active.preferred_usb_camera_name,
                                   active.default_intrinsics);
    auto ext = core::DeviceProfileRegistry::instance().activeExtrinsics();
    std::lock_guard<std::mutex> lock(state_mutex_);
    calib_profile_ = ext;
    agnostic_engine_.setExtrinsicCalibration(ext);
    synchronizer_.setCalibratedCameraTimeOffset(ext.camera_time_offset_sec);
    return true;
}

void DesktopSessionController::setUniversalSensorProfile(const std::string& universal_profile_id) {
    for (const auto& up : sensors::UniversalRigCatalog::allProfiles()) {
        if (up.profile_id == universal_profile_id) {
            active_universal_profile_id_ = up.profile_id;
            active_universal_profile_name_ = up.display_name;
            // Sync per-sensor toggles to the chosen profile (user can still override any sensor individually!)
            sensor_toggles_.lidar_mode = (up.expected_channels & slam::SENSOR_LIDAR)
                ? SensorSelectionMode::AUTO : SensorSelectionMode::DISABLED_NONE;
            sensor_toggles_.depth_mode = (up.expected_channels & slam::SENSOR_DEPTH_CAMERA)
                ? SensorSelectionMode::AUTO : SensorSelectionMode::DISABLED_NONE;
            if (up.expected_channels & slam::SENSOR_RGB_CAMERA) {
                if (sensor_toggles_.rgb_mode == SensorSelectionMode::DISABLED_NONE) {
                    sensor_toggles_.rgb_mode = SensorSelectionMode::AUTO;
                }
            } else {
                sensor_toggles_.rgb_mode = SensorSelectionMode::DISABLED_NONE;
            }
            sensor_toggles_.imu_mode = SensorSelectionMode::AUTO;
            sensor_toggles_.gnss_mode = up.supports_rtk_gps
                ? SensorSelectionMode::AUTO : SensorSelectionMode::DISABLED_NONE;
            std::lock_guard<std::mutex> lock(state_mutex_);
            agnostic_engine_.setSensorRigConfig(up.rig_config);
            return;
        }
    }
}

void DesktopSessionController::selectCameraOverride(const std::string& exact_name) {
    if (exact_name.empty()) {
        setRgbSelection(SensorSelectionMode::AUTO, "");
    } else {
        setRgbSelection(SensorSelectionMode::ENABLED_SPECIFIC, exact_name);
    }
}

void DesktopSessionController::setRgbSelection(SensorSelectionMode mode, const std::string& exact_name) {
    sensor_toggles_.rgb_mode = mode;
    sensor_toggles_.rgb_device_name = (mode == SensorSelectionMode::ENABLED_SPECIFIC) ? exact_name : "";
    camera_.setExactCameraNameOverride(sensor_toggles_.rgb_device_name);
}

void DesktopSessionController::setImuSelection(SensorSelectionMode mode, const std::string& imu_id) {
    sensor_toggles_.imu_mode = mode;
    sensor_toggles_.imu_source_id = (mode == SensorSelectionMode::ENABLED_SPECIFIC) ? imu_id : "";
}

std::vector<DetectedImuInfo> DesktopSessionController::availableImuSources() const {
    std::vector<DetectedImuInfo> out;
    auto is_ports = inertial_sense_.enumerateCandidatePorts();
    if (!is_ports.empty()) {
        out.push_back({"inertial_sense_imu",
                       "Inertial Sense uINS-3 / IMX-5 Dual RTK IMU (" + is_ports.front().port_name + ")",
                       120, true});
    } else {
        out.push_back({"inertial_sense_imu",
                       "Inertial Sense uINS-3 / IMX-5 Dual RTK IMU (Auto Serial)",
                       120, true});
    }
    out.push_back({"airy_imu", "RoboSense Airy 200Hz IMU (Ethernet)", 100, true});
    auto host_imus = host_imu_.enumerateAvailableImus();
    out.insert(out.end(), host_imus.begin(), host_imus.end());
    std::sort(out.begin(), out.end(), [](const DetectedImuInfo& a, const DetectedImuInfo& b) {
        return a.priority > b.priority;
    });
    return out;
}

std::string DesktopSessionController::primaryGeometryStatusSummary() const {
    switch (sensor_toggles_.primary_geometry) {
        case PrimaryGeometrySensorMode::FORCE_LIDAR:     return "MANUAL -> LiDAR";
        case PrimaryGeometrySensorMode::FORCE_DEPTH:     return "MANUAL -> Depth Camera";
        case PrimaryGeometrySensorMode::FORCE_MONOCULAR: return "MANUAL -> Monocular RGB";
        case PrimaryGeometrySensorMode::AUTO_BEST:
        default: {
            if (sensor_toggles_.lidar_mode != SensorSelectionMode::DISABLED_NONE &&
                (steadyNowSec() - last_lidar_arrival_sec_) <= 0.5) {
                return "AUTO -> LiDAR (LiDAR > Depth > Mono)";
            }
            if (sensor_toggles_.depth_mode != SensorSelectionMode::DISABLED_NONE && depth_capture_.isRunning()) {
                return "AUTO -> Depth (LiDAR > Depth > Mono)";
            }
            if (sensor_toggles_.rgb_mode != SensorSelectionMode::DISABLED_NONE && camera_.isRunning()) {
                return "AUTO -> Monocular (LiDAR > Depth > Mono)";
            }
            return "AUTO (LiDAR > Depth > Mono)";
        }
    }
}

std::string DesktopSessionController::imuStatusSummary() const {
    if (sensor_toggles_.imu_mode == SensorSelectionMode::DISABLED_NONE) return "OFF (None)";
    if (sensor_toggles_.imu_mode == SensorSelectionMode::ENABLED_SPECIFIC) {
        if (sensor_toggles_.imu_source_id == "inertial_sense_imu") {
            return inertial_sense_.isStreamingLive()
                ? ("MANUAL -> " + inertial_sense_.statusSummary())
                : "MANUAL -> Waiting for uINS-3 / IMX-5 Serial";
        }
        if (sensor_toggles_.imu_source_id == "airy_imu") {
            return (udp_receiver_ && udp_receiver_->getPacketStats().imu_packets_valid > 0)
                ? "MANUAL -> Airy 200Hz IMU" : "MANUAL -> Waiting for Airy IMU";
        }
        return host_imu_.isRunning() ? ("MANUAL -> " + host_imu_.statusSummary()) : ("MANUAL -> " + sensor_toggles_.imu_source_id);
    }
    // AUTO (Best) priority readout: Inertial Sense (120) > Airy (100) > Orbbec 336L (85) > Host (65)
    if (inertial_sense_.isStreamingLive()) {
        return "AUTO -> Inertial Sense uINS-3/IMX-5 (Prio 120)";
    }
    if (udp_receiver_ && udp_receiver_->getPacketStats().imu_packets_valid > 0 &&
        sensor_toggles_.lidar_mode != SensorSelectionMode::DISABLED_NONE) {
        return "AUTO -> Airy 200Hz IMU (Prio 100)";
    }
    if (host_imu_.isRunning() && host_imu_.samplesDelivered() > 0) {
        return "AUTO -> " + host_imu_.statusSummary() + " (Prio 85)";
    }
    return "AUTO (uINS-3/IMX-5 > Airy > 336L > Host)";
}

slam::AgnosticSlamStatus DesktopSessionController::currentAgnosticStatus() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return agnostic_engine_.getAgnosticStatus();
}

bool DesktopSessionController::injectNmeaGnssSentence(const std::string& nmea_gga) {
    if (sensor_toggles_.gnss_mode == SensorSelectionMode::DISABLED_NONE) return false;
    sensors::GnssRtkMeasurement fix;
    if (!gnss_processor_.parseNmeaGga(nmea_gga, steadyNowSec(), fix)) return false;
    std::lock_guard<std::mutex> lock(state_mutex_);
    pending_gnss_fix_ = fix;
    return true;
}

std::string DesktopSessionController::activeDeviceProfileDisplayName() const {
    return core::DeviceProfileRegistry::instance().activeProfile().display_name;
}

std::string DesktopSessionController::activeDeviceProfileId() const {
    return core::DeviceProfileRegistry::instance().activeProfile().profile_id;
}

sensors::AiryNetworkConfig DesktopSessionController::currentNetworkConfig() const { return net_config_; }
void DesktopSessionController::updateNetworkConfig(const sensors::AiryNetworkConfig& cfg) {
    net_config_ = cfg;
    discovery_->setConfig(cfg);
}
sensors::AiryDiscoveryReport DesktopSessionController::runAiryDiagnostics() { return discovery_->runDiagnostics(200, 200); }
sensors::AiryPacketStats DesktopSessionController::currentPacketStats() const { return udp_receiver_->getPacketStats(); }
sync::SyncDiagnostics DesktopSessionController::currentSyncDiagnostics() const { return synchronizer_.getDiagnostics(); }

bool DesktopSessionController::startScanning() {
    if (is_scanning_.load()) return true;

    // Always allocate a fresh unique scan folder if the current folder already has data
    // so consecutive scans (Airy, 336L, Webcam, Airy+Webcam, Airy+336L) never overwrite each other.
    if (active_scan_dir_.empty() || current_scan_used_ || scanDirHasExistingData(active_scan_dir_)) {
        createAndSelectScan(autoScanLabelForActiveProfile(), active_scan_floor_level_);
    }
    project_store_.initializeProject(active_scan_dir_, active_project_id_, active_scan_name_);
    current_scan_used_ = true;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        agnostic_engine_.reset();
        gnss_processor_.reset();
        synchronizer_.reset();
        pose_graph_.clear();
        sc_db_.clear();
        cached_global_cloud_.clear();
        live_preview_cloud_.clear();
        live_voxel_index_.clear();
        keyframe_traveled_dist_.clear();
        next_keyframe_id_ = 1;
        total_traveled_dist_m_ = 0.0;
        unclosed_dist_m_ = 0.0;
        last_loop_kf_id_ = 0;
        has_prev_odom_pos_ = false;
        universal_seq_id_ = 0;
        last_lidar_arrival_sec_ = 0.0;
        if (factory_calib_.valid) {
            agnostic_engine_.coreEngine().setFactoryCalibration(factory_calib_);
        }
        agnostic_engine_.setExtrinsicCalibration(calib_profile_);
        agnostic_engine_.coreEngine().setVisualFusionEnabled(false);
        synchronizer_.setCalibratedCameraTimeOffset(calib_profile_.camera_time_offset_sec);
    }
    {
        std::lock_guard<std::mutex> lk(mono_rgb_mutex_);
        standalone_rgb_queue_.clear();
        standalone_depth_queue_.clear();
        standalone_imu_queue_.clear();
    }

    const bool use_lidar = (sensor_toggles_.lidar_mode != SensorSelectionMode::DISABLED_NONE) &&
                           (sensor_toggles_.primary_geometry != PrimaryGeometrySensorMode::FORCE_DEPTH) &&
                           (sensor_toggles_.primary_geometry != PrimaryGeometrySensorMode::FORCE_MONOCULAR);
    const bool use_rgb   = (sensor_toggles_.rgb_mode != SensorSelectionMode::DISABLED_NONE);
    const bool use_depth = (sensor_toggles_.depth_mode != SensorSelectionMode::DISABLED_NONE) &&
                           (sensor_toggles_.primary_geometry != PrimaryGeometrySensorMode::FORCE_MONOCULAR);
    const bool use_inertial_sense = (sensor_toggles_.imu_mode == SensorSelectionMode::AUTO) ||
                                    (sensor_toggles_.imu_mode == SensorSelectionMode::ENABLED_SPECIFIC &&
                                     sensor_toggles_.imu_source_id == "inertial_sense_imu") ||
                                    (sensor_toggles_.gnss_mode != SensorSelectionMode::DISABLED_NONE);
    const bool use_host_imu = (sensor_toggles_.imu_mode == SensorSelectionMode::AUTO) ||
                              (sensor_toggles_.imu_mode == SensorSelectionMode::ENABLED_SPECIFIC &&
                               sensor_toggles_.imu_source_id != "airy_imu" &&
                               sensor_toggles_.imu_source_id != "inertial_sense_imu");

    if (use_lidar) {
        fs::path raw_path = fs::path(active_scan_dir_) / (active_scan_id_ + "_raw.avraw");
        udp_receiver_->startRawRecording(raw_path.string());
        udp_receiver_->start();
    }
    if (use_rgb) {
        camera_.start(30);
    }
    if (use_depth) {
        depth_capture_.start();
    }
    if (use_inertial_sense) {
        inertial_sense_.start();
    }
    if (use_host_imu) {
        host_imu_.start(sensor_toggles_.imu_source_id);
    }

    is_scanning_.store(true);
    worker_running_.store(true);
    slam_thread_ = std::thread(&DesktopSessionController::slamWorkerLoop, this);
    return true;
}

void DesktopSessionController::stopScanning() {
    is_scanning_.store(false);
    if (worker_running_.exchange(false)) {
        if (slam_thread_.joinable()) slam_thread_.join();
    }
    camera_.stop();
    depth_capture_.stop();
    inertial_sense_.stop();
    host_imu_.stop();
    udp_receiver_->stopRawRecording();
    udp_receiver_->stop();
}

void DesktopSessionController::slamWorkerLoop() {
    while (worker_running_.load()) {
        if (is_slam_paused_for_station_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        sync::SyncBundle bundle;
        slam::UniversalSensorBundle u_bundle;
        bool have_bundle = false;

        if (synchronizer_.popSyncBundle(bundle)) {
            u_bundle.sequence_id = ++universal_seq_id_;
            u_bundle.timestamp_sec = bundle.lidar_frame.timestamp_end_sec;
            u_bundle.lidar_frame = bundle.lidar_frame;
            u_bundle.imu_measurements = bundle.imu_measurements;
            u_bundle.rgb_frame = bundle.rgb_frame;
            {
                std::lock_guard<std::mutex> lk(mono_rgb_mutex_);
                if (!standalone_depth_queue_.empty()) {
                    u_bundle.depth_frame = standalone_depth_queue_.back();
                    standalone_depth_queue_.clear();
                }
            }
            have_bundle = true;
        } else if ((steadyNowSec() - last_lidar_arrival_sec_) > 0.25) {
            // Sensor-agnostic fallback: run Depth-only (336L) or Monocular-only (Surface webcam / USB cam),
            // automatically attaching Host/PC IMU samples when IMU is enabled!
            std::lock_guard<std::mutex> lk(mono_rgb_mutex_);
            if (!standalone_depth_queue_.empty()) {
                u_bundle.sequence_id = ++universal_seq_id_;
                u_bundle.depth_frame = standalone_depth_queue_.front();
                u_bundle.timestamp_sec = u_bundle.depth_frame->timestamp_sec;
                standalone_depth_queue_.pop_front();
                if (!standalone_rgb_queue_.empty()) {
                    u_bundle.rgb_frame = standalone_rgb_queue_.back();
                    standalone_rgb_queue_.clear();
                }
                if (sensor_toggles_.imu_mode != SensorSelectionMode::DISABLED_NONE && !standalone_imu_queue_.empty()) {
                    u_bundle.imu_measurements.assign(standalone_imu_queue_.begin(), standalone_imu_queue_.end());
                    standalone_imu_queue_.clear();
                }
                have_bundle = true;
            } else if (!standalone_rgb_queue_.empty()) {
                u_bundle.sequence_id = ++universal_seq_id_;
                u_bundle.rgb_frame = standalone_rgb_queue_.front();
                u_bundle.timestamp_sec = u_bundle.rgb_frame->timestamp_sec;
                standalone_rgb_queue_.pop_front();
                if (sensor_toggles_.imu_mode != SensorSelectionMode::DISABLED_NONE && !standalone_imu_queue_.empty()) {
                    u_bundle.imu_measurements.assign(standalone_imu_queue_.begin(), standalone_imu_queue_.end());
                    standalone_imu_queue_.clear();
                }
                have_bundle = true;
            }
        }

        if (!have_bundle) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        core::LidarFrame deskewed_body;
        core::LidarFrame world_frame;
        slam::EstimatorMetrics metrics;

        std::lock_guard<std::mutex> lock(state_mutex_);
        if (pending_gnss_fix_.has_value()) {
            u_bundle.gnss_fix = pending_gnss_fix_;
            pending_gnss_fix_.reset();
        }
        if (!agnostic_engine_.processUniversalBundle(u_bundle, deskewed_body, world_frame, metrics)) continue;

        auto state = agnostic_engine_.getCurrentState();
        if (has_prev_odom_pos_) {
            double step_m = (state.T_world_body.t - prev_odom_pos_).norm();
            if (step_m < 2.0) {
                total_traveled_dist_m_ += step_m;
                unclosed_dist_m_ += step_m;
            }
        }
        prev_odom_pos_ = state.T_world_body.t;
        has_prev_odom_pos_ = true;

        latest_quality_ = quality_monitor_.evaluateFrameQuality(state, metrics, unclosed_dist_m_);
        mergeIntoLiveCloud(world_frame.points, 2, cached_global_cloud_, live_voxel_index_);

        if (metrics.keyframe_emitted) {
            core::KeyframeSubmap submap;
            submap.keyframe_id = next_keyframe_id_++;
            submap.timestamp_sec = u_bundle.timestamp_sec;
            submap.T_world_keyframe = state.T_world_body;
            submap.local_points = deskewed_body.points;

            project_store_.appendKeyframeToWal(submap);
            pose_graph_.addKeyframeSubmap(submap);
            keyframe_traveled_dist_[submap.keyframe_id] = total_traveled_dist_m_;

            auto sc_desc = backend::ScanContextDatabase::computeDescriptor(
                submap.keyframe_id, submap.timestamp_sec, submap.local_points, u_bundle.rgb_frame);
            auto loop_cand = sc_db_.detectLoopClosure(sc_desc);
            sc_db_.addKeyframeDescriptor(sc_desc);

            if (loop_cand.valid && (submap.keyframe_id >= last_loop_kf_id_ + 20)) {
                for (auto& target_sm : pose_graph_.getMutableSubmaps()) {
                    if (target_sm.keyframe_id == loop_cand.match_keyframe_id) {
                        uint64_t kf_gap = (submap.keyframe_id > target_sm.keyframe_id) ? (submap.keyframe_id - target_sm.keyframe_id) : 0;
                        double dist_gap = total_traveled_dist_m_ - keyframe_traveled_dist_[target_sm.keyframe_id];
                        double sp_dist = (submap.T_world_keyframe.t - target_sm.T_world_keyframe.t).norm();
                        if (kf_gap >= 30 && dist_gap >= 8.0 && sp_dist < 4.5) {
                            if (target_sm.local_points.empty()) project_store_.loadSubmapFromDisk(target_sm);
                            math::SE3d init_guess = target_sm.T_world_keyframe.inverse() * submap.T_world_keyframe;
                            auto ver = gicp_verifier_.verifyLoopConstraint(submap.local_points, target_sm.local_points, init_guess);
                            if (ver.verified) {
                                const math::SE3d delta = init_guess.inverse() * ver.T_target_source;
                                const double delta_rot_deg = delta.so3.log().norm() * math::kRadToDeg;
                                const double delta_trans_m = delta.t.norm();
                                const double delta_z_m = std::abs(ver.T_target_source.t.z - init_guess.t.z);
                                // Strict loop sanity gate: reject floor-to-ceiling or false corridor jumps
                                if (delta_rot_deg < 15.0 && delta_trans_m < 1.5 && delta_z_m < 0.60 &&
                                    ver.fitness_rmse_m <= 0.06 && ver.inlier_overlap_ratio >= 0.55) {
                                    pose_graph_.addVerifiedLoopFactor(target_sm.keyframe_id, submap.keyframe_id,
                                                                      ver.T_target_source, ver.information_diagonal, true);
                                    pose_graph_.optimizeGlobalPoseGraph(8);
                                    last_loop_kf_id_ = submap.keyframe_id;
                                    unclosed_dist_m_ = 0.0;
                                    cached_global_cloud_ = pose_graph_.buildGloballyConsistentPointCloud(2, kDesktopLiveVoxelM);
                                    live_voxel_index_.clear();
                                    for (size_t i = 0; i < cached_global_cloud_.size(); ++i) {
                                        const auto& p = cached_global_cloud_[i];
                                        live_voxel_index_[desktopVoxelKey(p.x, p.y, p.z)] = i;
                                    }
                                }
                            }
                        }
                        break;
                    }
                }
            }

            latest_floor_plan_ = quality::QualityMonitor::generateLiveFloorPlan(
                cached_global_cloud_, pose_graph_.getSubmaps(), pose_graph_.getFactors(), 0.10);

            if (is_cslam_enabled_.load() && cslam_engine_) {
                cslam_engine_->addLocalKeyframe(submap, sc_desc);
                auto pkt = cslam_engine_->serializeKeyframeSync(submap, sc_desc, 1200);
                if (cslam_transport_ && cslam_transport_->isRunning()) {
                    cslam_transport_->broadcastTcpPacket(pkt);
                }
            }
        }
    }
}

bool DesktopSessionController::saveAndRefineSession() {
    stopScanning();
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (auto& sm : pose_graph_.getMutableSubmaps()) {
        if (sm.local_points.empty()) project_store_.loadSubmapFromDisk(sm);
    }
    if (!pose_graph_.getSubmaps().empty()) {
        pose_graph_.optimizeGlobalPoseGraph(40);
        processing::CleanupConfig cfg;
        std::vector<core::LidarPointXYZIRT> cleaned;
        last_cleanup_stats_ = processing::PointCloudCleanup::runPostScanPipeline(
            pose_graph_.getSubmaps(), cleaned, cfg);
        if (last_cleanup_stats_.floor_leveled) {
            const math::SE3d& T_lvl = last_cleanup_stats_.T_level_world;
            for (auto& sm : pose_graph_.getMutableSubmaps()) {
                sm.T_world_keyframe = T_lvl * sm.T_world_keyframe;
            }
        }
        if (!cleaned.empty()) {
            cached_global_cloud_ = std::move(cleaned);
        }
        latest_floor_plan_ = quality::QualityMonitor::generateLiveFloorPlan(
            cached_global_cloud_, pose_graph_.getSubmaps(), pose_graph_.getFactors(), 0.08);
    }
    std::vector<storage::PoiPhotoRecord> empty_pois;
    bool ok = project_store_.saveProjectSession(calib_profile_, pose_graph_.getSubmaps(), pose_graph_.getFactors(), empty_pois);
    if (!cached_global_cloud_.empty() && !active_scan_dir_.empty()) {
        std::string base = active_scan_id_.empty() ? ("scan_" + timestampStamp()) : active_scan_id_;
        exporters::PointCloudExporter::exportToPly(
            (fs::path(active_scan_dir_) / (base + ".ply")).string(), cached_global_cloud_, true);
    }
    current_scan_used_ = true;
    return ok;
}

bool DesktopSessionController::startTerrestrialStation(double step_deg, bool is_flash_during_slam) {
    if (is_flash_during_slam && is_scanning_.load()) {
        is_slam_paused_for_station_.store(true);
        std::lock_guard<std::mutex> lock(state_mutex_);
        flash_station_anchor_pose_ = agnostic_engine_.getCurrentState().T_world_body;
        acquisition_mode_ = DesktopAcquisitionMode::HYBRID_FLASH_STATION;
    } else {
        if (is_scanning_.load()) stopScanning();
        is_slam_paused_for_station_.store(false);
        acquisition_mode_ = DesktopAcquisitionMode::TERRESTRIAL_STATION;
    }

    terrestrial::TerrestrialStationConfig cfg;
    cfg.project_id = active_project_id_;
    cfg.station_id = (is_flash_during_slam ? "flash_st_" : "st_") + timestampStamp();
    cfg.station_name = cfg.station_id;
    cfg.nominal_step_deg = (step_deg > 5.0) ? step_deg : 60.0;
    cfg.total_expected_stops = static_cast<size_t>(std::round(360.0 / cfg.nominal_step_deg));
    cfg.accumulation_sec_per_stop = 5.0;

    auto mount_profile = core::DeviceProfileRegistry::instance().activeProfile();
    station_manager_ = std::make_unique<terrestrial::TerrestrialStationManager>(cfg);
    station_manager_->startNewStation(active_scan_dir_, cfg, mount_profile);
    udp_receiver_->start();
    is_terrestrial_active_.store(true);
    return true;
}

bool DesktopSessionController::advanceToNextTerrestrialStop() {
    return station_manager_ ? station_manager_->advanceToNextStop() : false;
}

bool DesktopSessionController::repeatCurrentTerrestrialStop() {
    return station_manager_ ? station_manager_->repeatCurrentStop() : false;
}

bool DesktopSessionController::finalizeAndSaveTerrestrialStation() {
    if (!station_manager_) return false;
    std::string err;
    bool ok = station_manager_->finalizeAndSaveStation(err);
    if (ok) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        auto stat_cloud = station_manager_->buildStationPointCloud();
        if (is_slam_paused_for_station_.load()) {
            core::KeyframeSubmap anchor;
            anchor.keyframe_id = next_keyframe_id_++;
            anchor.timestamp_sec = steadyNowSec();
            anchor.T_world_keyframe = flash_station_anchor_pose_;
            anchor.local_points = stat_cloud;
            pose_graph_.addKeyframeSubmap(anchor);
            cached_global_cloud_ = pose_graph_.buildGloballyConsistentPointCloud(2, kDesktopLiveVoxelM);
            is_slam_paused_for_station_.store(false);
            acquisition_mode_ = DesktopAcquisitionMode::MOBILE_SLAM;
        } else {
            cached_global_cloud_ = stat_cloud;
        }
        is_terrestrial_active_.store(false);
    }
    return ok;
}

uint32_t DesktopSessionController::currentTerrestrialStopIndex() const {
    return station_manager_ ? station_manager_->getCurrentStopIndex() : 0;
}

double DesktopSessionController::terrestrialAccumulationProgress() const {
    return station_manager_ ? station_manager_->getCurrentAccumulationProgress() : 0.0;
}

terrestrial::TerrestrialCaptureState DesktopSessionController::currentTerrestrialCaptureState() const {
    return station_manager_ ? station_manager_->getCaptureState() : terrestrial::TerrestrialCaptureState::IDLE;
}

bool DesktopSessionController::exportPly(const std::string& path) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return exporters::PointCloudExporter::exportToPly(path, cached_global_cloud_, true);
}

bool DesktopSessionController::exportPcd(const std::string& path) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return exporters::PointCloudExporter::exportToPcd(path, cached_global_cloud_, true);
}

bool DesktopSessionController::exportE57(const std::string& path) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return exporters::PointCloudExporter::exportToE57(path, cached_global_cloud_);
}

bool DesktopSessionController::exportSpatialBundle(const std::string& output_dir) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    export_bundle::AvShowcaseProjectConfig cfg;
    cfg.project_id = active_project_id_;
    cfg.project_name = active_project_name_;
    cfg.device_profile_id = activeDeviceProfileId();

    std::vector<export_bundle::PanoStationRecord> stations;
    const auto& submaps = pose_graph_.getSubmaps();
    for (size_t i = 0; i < submaps.size(); ++i) {
        export_bundle::PanoStationRecord st;
        st.station_id = "st_" + std::to_string(i);
        st.station_index = i;
        st.keyframe_id = submaps[i].keyframe_id;
        st.timestamp_sec = submaps[i].timestamp_sec;
        st.T_world_station = submaps[i].T_world_keyframe;
        st.point_count = submaps[i].local_points.size();
        stations.push_back(st);
    }
    auto traj = agnostic_engine_.getTrajectory();
    return export_bundle::AvSpatialBundleExporter::exportWalkthroughBundle(
        output_dir, cfg, cached_global_cloud_, traj, latest_floor_plan_, stations);
}

bool DesktopSessionController::setCollaborativeSlamEnabled(bool enabled) {
    if (enabled == is_cslam_enabled_.load()) return true;
    if (enabled) {
        cslam_transport_->setPacketCallback([this](const uint8_t* data, size_t len, const std::string& ip) {
            if (cslam_engine_) cslam_engine_->ingestRemotePacket(data, len, ip, steadyNowSec());
        });
        cslam_transport_->start();
        is_cslam_enabled_.store(true);
        cslam_beacon_running_.store(true);
        cslam_beacon_thread_ = std::thread(&DesktopSessionController::cslamBeaconLoop, this);
    } else {
        is_cslam_enabled_.store(false);
        if (cslam_beacon_running_.exchange(false)) {
            if (cslam_beacon_thread_.joinable()) cslam_beacon_thread_.join();
        }
        cslam_transport_->stop();
    }
    return true;
}

void DesktopSessionController::cslamBeaconLoop() {
    while (cslam_beacon_running_.load()) {
        math::SE3d pose;
        uint32_t kfs = 0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            pose = agnostic_engine_.getCurrentState().T_world_body;
            kfs = static_cast<uint32_t>(pose_graph_.getSubmaps().size());
        }
        auto beacon = cslam_engine_->serializeBeacon(pose, kfs);
        if (cslam_transport_ && cslam_transport_->isRunning()) {
            cslam_transport_->broadcastUdpPacket(beacon);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void DesktopSessionController::connectCollaborativePeerByIp(const std::string& ip) {
    if (ip.empty()) return;
    if (!is_cslam_enabled_.load()) setCollaborativeSlamEnabled(true);
    cslam_transport_->addDirectPeerIp(ip);
}

void DesktopSessionController::syncSharedOriginWithPeers() {
    if (!cslam_engine_) return;
    cslam_engine_->syncSharedOriginWithAllPeers();
    auto pkt = cslam_engine_->serializeSharedOriginSync();
    if (cslam_transport_ && cslam_transport_->isRunning()) {
        cslam_transport_->broadcastUdpPacket(pkt);
        cslam_transport_->broadcastTcpPacket(pkt);
    }
}

std::vector<cslam::PeerInfo> DesktopSessionController::connectedPeers() const {
    return cslam_engine_ ? cslam_engine_->getPeers(steadyNowSec(), 20.0) : std::vector<cslam::PeerInfo>{};
}

cslam::CslamStats DesktopSessionController::cslamStats() const {
    return cslam_engine_ ? cslam_engine_->getStats() : cslam::CslamStats{};
}

std::vector<core::LidarPointXYZIRT> DesktopSessionController::latestGlobalPointCloud() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (is_terrestrial_active_.load() && station_manager_) {
        return station_manager_->buildStationPointCloud();
    }
    auto cloud = !cached_global_cloud_.empty() ? cached_global_cloud_ : live_preview_cloud_;
    if (is_cslam_enabled_.load() && cslam_engine_) {
        auto peer_pts = cslam_engine_->buildAlignedPeerPointCloud(100000);
        cloud.insert(cloud.end(), peer_pts.begin(), peer_pts.end());
    }
    return cloud;
}

std::vector<core::SlamState> DesktopSessionController::currentTrajectory() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto traj = agnostic_engine_.getTrajectory();
    if (is_cslam_enabled_.load() && cslam_engine_) {
        auto pt = cslam_engine_->getAlignedPeerTrajectories();
        traj.insert(traj.end(), pt.begin(), pt.end());
    }
    return traj;
}

quality::ScanQualityReport DesktopSessionController::latestQualityReport() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_quality_;
}

quality::FloorPlanGrid2D DesktopSessionController::latestFloorPlan() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_floor_plan_;
}

slam::AvSlamRuntimeStats DesktopSessionController::currentAvSlamStats() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return agnostic_engine_.coreEngine().getRuntimeStats();
}

core::ExtrinsicCalibrationProfile DesktopSessionController::currentCalibrationProfile() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return calib_profile_;
}

}  // namespace desktop
}  // namespace avscan
