#pragma once

#include "DesktopCameraCapture.hpp"
#include "avscan/backend/GicpVerifier.hpp"
#include "avscan/backend/PoseGraphOptimizer.hpp"
#include "avscan/backend/ScanContext.hpp"
#include "avscan/calibration/OnlineAutoCalibrator.hpp"
#include "avscan/core/DeviceMountProfile.hpp"
#include "avscan/cslam/CollaborativeSlamEngine.hpp"
#include "avscan/cslam/CollaborativeTransport.hpp"
#include "avscan/export/AvSpatialBundleExporter.hpp"
#include "avscan/export/PointCloudExporter.hpp"
#include "avscan/processing/PointCloudCleanup.hpp"
#include "avscan/quality/QualityMonitor.hpp"
#include "avscan/sensors/AiryDiscovery.hpp"
#include "avscan/sensors/AiryUdpReceiver.hpp"
#include "avscan/sensors/GnssRtkReceiver.hpp"
#include "avscan/sensors/IDepthProvider.hpp"
#include "avscan/sensors/UniversalRigPresets.hpp"
#include "avscan/slam/AvSlamEngine.hpp"
#include "avscan/slam/SensorAgnosticSlamEngine.hpp"
#include "avscan/storage/ProjectStore.hpp"
#include "avscan/sync/TimeSynchronizer.hpp"
#include "avscan/terrestrial/TerrestrialStationManager.hpp"
#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace avscan {
namespace desktop {

enum class DesktopAcquisitionMode {
    MOBILE_SLAM = 0,
    TERRESTRIAL_STATION = 1,
    HYBRID_FLASH_STATION = 2
};

struct DesktopProjectSummary {
    std::string id;
    std::string name;
    std::string site;
    std::string path;
    size_t scan_count{0};
};

struct DesktopScanSummary {
    std::string id;
    std::string name;
    std::string floor_level;
    std::string mode;
    std::string path;
};

// Cross-platform Desktop Session Controller (Windows & Linux/Jetson) wrapping avslam_core.
// Provides 100% feature parity with iOS AVScanCoreBridge + ScanAR (Airy + Orbbec 336L) support.
class DesktopSessionController {
public:
    DesktopSessionController();
    ~DesktopSessionController();

    // Project & Scan Hierarchy
    std::string projectsRootDir() const;
    std::vector<DesktopProjectSummary> listProjects() const;
    bool createProject(const std::string& name, const std::string& site, const std::string& notes);
    bool openProject(const std::string& project_id);
    std::vector<DesktopScanSummary> listScansInActiveProject() const;
    bool createAndSelectScan(const std::string& scan_name, const std::string& floor_level);
    bool selectScan(const std::string& scan_id);

    std::string activeProjectId() const { return active_project_id_; }
    std::string activeProjectName() const { return active_project_name_; }
    std::string activeScanName() const { return active_scan_name_; }
    std::string activeScanFloorLevel() const { return active_scan_floor_level_; }
    std::string activeScanDir() const { return active_scan_dir_; }

    // Rig & Hardware Profiles (Universal Sensor-Agnostic Catalog)
    bool setActiveDeviceProfile(const std::string& profile_id);
    std::string activeDeviceProfileDisplayName() const;
    std::string activeDeviceProfileId() const;
    void setUniversalSensorProfile(const std::string& universal_profile_id);
    std::string activeUniversalProfileId() const { return active_universal_profile_id_; }
    std::string activeUniversalProfileName() const { return active_universal_profile_name_; }
    slam::AgnosticSlamStatus currentAgnosticStatus() const;
    bool injectNmeaGnssSentence(const std::string& nmea_gga);

    // Airy Ethernet Diagnostics
    sensors::AiryNetworkConfig currentNetworkConfig() const;
    void updateNetworkConfig(const sensors::AiryNetworkConfig& cfg);
    sensors::AiryDiscoveryReport runAiryDiagnostics();
    sensors::AiryPacketStats currentPacketStats() const;
    sync::SyncDiagnostics currentSyncDiagnostics() const;

    // Mobile SLAM Lifecycle
    bool startScanning();
    void stopScanning();
    bool saveAndRefineSession();
    bool isScanning() const { return is_scanning_.load(); }
    void setAcquisitionMode(DesktopAcquisitionMode m) { acquisition_mode_ = m; }
    DesktopAcquisitionMode acquisitionMode() const { return acquisition_mode_; }

    // Terrestrial & Hybrid Flash Station Mode
    bool startTerrestrialStation(double step_deg, bool is_flash_during_slam);
    bool advanceToNextTerrestrialStop();
    bool repeatCurrentTerrestrialStop();
    bool finalizeAndSaveTerrestrialStation();
    bool isTerrestrialActive() const { return is_terrestrial_active_.load(); }
    bool isSlamPausedForStation() const { return is_slam_paused_for_station_.load(); }
    uint32_t currentTerrestrialStopIndex() const;
    double terrestrialAccumulationProgress() const;
    terrestrial::TerrestrialCaptureState currentTerrestrialCaptureState() const;

    // Exports
    bool exportPly(const std::string& path);
    bool exportPcd(const std::string& path);
    bool exportE57(const std::string& path);
    bool exportSpatialBundle(const std::string& output_dir);

    // Collaborative SLAM (C-SLAM) — Interoperates with iOS AV SLAM, AV Scan iOS, Jetson & Windows
    bool setCollaborativeSlamEnabled(bool enabled);
    bool isCslamEnabled() const { return is_cslam_enabled_.load(); }
    void connectCollaborativePeerByIp(const std::string& ip);
    void syncSharedOriginWithPeers();
    std::vector<cslam::PeerInfo> connectedPeers() const;
    cslam::CslamStats cslamStats() const;

    // Live UI Telemetry & Render Data
    std::vector<core::LidarPointXYZIRT> latestGlobalPointCloud() const;
    std::vector<core::SlamState> currentTrajectory() const;
    quality::ScanQualityReport latestQualityReport() const;
    quality::FloorPlanGrid2D latestFloorPlan() const;
    slam::AvSlamRuntimeStats currentAvSlamStats() const;
    core::ExtrinsicCalibrationProfile currentCalibrationProfile() const;
    size_t loopClosureCount() const { return pose_graph_.numLoopClosures(); }
    std::string cameraStatusSummary() const { return camera_.statusSummary(); }
    double cameraMeasuredFps() const { return camera_.measuredFps(); }

private:
    static double steadyNowSec();
    static std::string sanitizeId(const std::string& raw, const std::string& fallback);
    void setupCallbacks();
    void slamWorkerLoop();
    void cslamBeaconLoop();

    sensors::AiryNetworkConfig net_config_{};
    std::unique_ptr<sensors::AiryDiscovery> discovery_;
    std::unique_ptr<sensors::AiryUdpReceiver> udp_receiver_;
    DesktopCameraCapture camera_;

    sync::TimeSynchronizer synchronizer_;
    slam::SensorAgnosticSlamEngine agnostic_engine_;
    sensors::GnssRtkProcessor gnss_processor_;
    backend::ScanContextDatabase sc_db_;
    backend::GicpVerifier gicp_verifier_;
    backend::PoseGraphOptimizer pose_graph_;
    quality::QualityMonitor quality_monitor_;
    storage::ProjectStore project_store_;
    calibration::OnlineAutoCalibrator auto_calibrator_;

    std::unique_ptr<terrestrial::TerrestrialStationManager> station_manager_;
    std::unique_ptr<cslam::CollaborativeSlamEngine> cslam_engine_;
    std::unique_ptr<cslam::CollaborativeLanTransport> cslam_transport_;

    mutable std::mutex state_mutex_;
    core::AiryFactoryCalibration factory_calib_{};
    core::ExtrinsicCalibrationProfile calib_profile_{};
    quality::ScanQualityReport latest_quality_{};
    quality::FloorPlanGrid2D latest_floor_plan_{};
    processing::CleanupStats last_cleanup_stats_{};
    std::vector<core::LidarPointXYZIRT> cached_global_cloud_;
    std::vector<core::LidarPointXYZIRT> live_preview_cloud_;
    std::unordered_map<slam::VoxelKey, size_t, slam::VoxelKeyHash> live_voxel_index_;

    std::string active_project_id_;
    std::string active_project_name_;
    std::string active_scan_id_;
    std::string active_scan_name_;
    std::string active_scan_floor_level_{"Level 01"};
    std::string active_scan_dir_;
    std::string active_universal_profile_id_{"scanar_v4_airy_336l"};
    std::string active_universal_profile_name_{"ScanAR V4 (RoboSense Airy + Orbbec Gemini 336L)"};

    std::mutex mono_rgb_mutex_;
    std::deque<core::RgbFrame> standalone_rgb_queue_;
    std::optional<sensors::GnssRtkMeasurement> pending_gnss_fix_;
    double last_lidar_arrival_sec_{0.0};
    uint64_t universal_seq_id_{0};

    DesktopAcquisitionMode acquisition_mode_{DesktopAcquisitionMode::MOBILE_SLAM};
    std::atomic<bool> is_scanning_{false};
    std::atomic<bool> worker_running_{false};
    std::atomic<bool> is_terrestrial_active_{false};
    std::atomic<bool> is_slam_paused_for_station_{false};
    std::atomic<bool> is_cslam_enabled_{false};
    std::atomic<bool> cslam_beacon_running_{false};
    math::SE3d flash_station_anchor_pose_{math::SE3d::Identity()};

    std::thread slam_thread_;
    std::thread cslam_beacon_thread_;
    uint64_t next_keyframe_id_{1};
    double unclosed_dist_m_{0.0};
    double total_traveled_dist_m_{0.0};
    std::unordered_map<uint64_t, double> keyframe_traveled_dist_;
    uint64_t last_loop_kf_id_{0};
    math::Vec3d prev_odom_pos_{0.0, 0.0, 0.0};
    bool has_prev_odom_pos_{false};
};

}  // namespace desktop
}  // namespace avscan
