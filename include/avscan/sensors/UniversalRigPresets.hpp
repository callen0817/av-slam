#pragma once

#include "avscan/sensors/SensorRigProfile.hpp"
#include "avscan/slam/SensorAgnosticSlamEngine.hpp"
#include <string>
#include <vector>

namespace avscan {
namespace sensors {

struct UniversalHardwareProfile {
    std::string profile_id;
    std::string display_name;
    std::string description;
    uint32_t expected_channels{slam::SENSOR_NONE};
    GenericSensorRigConfig rig_config{};
    bool supports_rtk_gps{true};
};

// Unified hardware preset catalog consolidating AV Scan (Jetson), AV Scanar, and AV iOS rigs
class UniversalRigCatalog {
public:
    static std::vector<UniversalHardwareProfile> allProfiles() {
        std::vector<UniversalHardwareProfile> list;

        // 1. Proprietary ScanAR V4 (Airy + Orbbec Gemini 336L)
        {
            UniversalHardwareProfile p;
            p.profile_id = "scanar_v4_airy_336l";
            p.display_name = "ScanAR V4 (RoboSense Airy + Orbbec Gemini 336L)";
            p.description = "Proprietary ScanAR rig: 96-beam Airy LiDAR + 200Hz IMU + Gemini 336L RGB-D";
            p.expected_channels = slam::SENSOR_LIDAR | slam::SENSOR_DEPTH_CAMERA | slam::SENSOR_RGB_CAMERA | slam::SENSOR_IMU;
            p.rig_config = GenericSensorRigConfig::presetRoboSenseAiry();
            list.push_back(p);
        }
        // 2. Proprietary iOS Mounted Airy (iPad / iPhone USB-C)
        {
            UniversalHardwareProfile p;
            p.profile_id = "ios_mounted_airy";
            p.display_name = "iOS Mounted Airy (iPad Air/Pro/iPhone + USB-C Ethernet)";
            p.description = "Proprietary iOS rig: RoboSense Airy + iOS wide camera + optional Core ML depth";
            p.expected_channels = slam::SENSOR_LIDAR | slam::SENSOR_RGB_CAMERA | slam::SENSOR_IMU;
            p.rig_config = GenericSensorRigConfig::presetRoboSenseAiry();
            list.push_back(p);
        }
        // 3. RoboSense Airy + Monocular / Webcam (LiDAR + RGB + IMU, no Depth camera)
        {
            UniversalHardwareProfile p;
            p.profile_id = "airy_monocular_cam";
            p.display_name = "RoboSense Airy + Monocular / Webcam (LiDAR + RGB + IMU)";
            p.description = "Airy LiDAR + IMU colorized by onboard webcam or USB monocular RGB camera";
            p.expected_channels = slam::SENSOR_LIDAR | slam::SENSOR_RGB_CAMERA | slam::SENSOR_IMU;
            p.rig_config = GenericSensorRigConfig::presetRoboSenseAiry();
            list.push_back(p);
        }
        // 4. RoboSense Airy Standalone (LiDAR + IMU)
        {
            UniversalHardwareProfile p;
            p.profile_id = "airy_standalone";
            p.display_name = "RoboSense Airy Standalone (LiDAR + IMU)";
            p.description = "LiDAR-inertial SLAM without camera";
            p.expected_channels = slam::SENSOR_LIDAR | slam::SENSOR_IMU;
            p.rig_config = GenericSensorRigConfig::presetRoboSenseAiry();
            list.push_back(p);
        }
        // 4. Livox Mid-360 (from AV Scan)
        {
            UniversalHardwareProfile p;
            p.profile_id = "livox_mid360_avscan";
            p.display_name = "Livox Mid-360 (LiDAR + IMU + Optional Camera)";
            p.description = "Non-repetitive hemispherical LiDAR supported in AV Scan";
            p.expected_channels = slam::SENSOR_LIDAR | slam::SENSOR_IMU;
            p.rig_config = GenericSensorRigConfig::presetLivoxMid360();
            list.push_back(p);
        }
        // 5. Ouster OS0 / OS1 (from AV Scan)
        {
            UniversalHardwareProfile p;
            p.profile_id = "ouster_os0_os1_avscan";
            p.display_name = "Ouster OS0 / OS1 (Multi-Beam LiDAR + IMU)";
            p.description = "High-channel spinning LiDAR supported in AV Scan";
            p.expected_channels = slam::SENSOR_LIDAR | slam::SENSOR_IMU;
            p.rig_config = GenericSensorRigConfig::presetOusterOs0();
            list.push_back(p);
        }
        // 6. Orbbec Gemini 336L Standalone (Stereo Depth + RGB + IMU)
        {
            UniversalHardwareProfile p;
            p.profile_id = "orbbec_gemini_336l_standalone";
            p.display_name = "Orbbec Gemini 336L Standalone (RGB-D + IMU)";
            p.description = "Camera-only stereo depth SLAM with no external LiDAR required";
            p.expected_channels = slam::SENSOR_DEPTH_CAMERA | slam::SENSOR_RGB_CAMERA | slam::SENSOR_IMU;
            p.rig_config = GenericSensorRigConfig::presetAppleArkitDepth();
            p.rig_config.rig_id = "orbbec_gemini_336l";
            p.rig_config.display_name = "Orbbec Gemini 336L Stereo RGB-D";
            list.push_back(p);
        }
        // 7. Monocular Camera Standalone (replaces LingBot in AV Scan)
        {
            UniversalHardwareProfile p;
            p.profile_id = "monocular_camera_standalone";
            p.display_name = "Monocular Camera Standalone (UVC / iOS + CoreML/ONNX Depth)";
            p.description = "Single RGB camera SLAM (replaces standalone monocular pipelines)";
            p.expected_channels = slam::SENSOR_RGB_CAMERA;
            p.rig_config = GenericSensorRigConfig::presetAppleArkitDepth();
            p.rig_config.rig_id = "monocular_rgb";
            p.rig_config.has_integrated_imu = false;
            list.push_back(p);
        }
        // 8. RTK-GNSS + Multi-Sensor Outdoor Rig
        {
            UniversalHardwareProfile p;
            p.profile_id = "outdoor_rtk_multisensor";
            p.display_name = "Outdoor Geodetic Rig (LiDAR/Camera + IMU + RTK-GPS)";
            p.description = "Cm-grade georeferenced outdoor SLAM with NMEA-0183 / UBX RTK fusion";
            p.expected_channels = slam::SENSOR_LIDAR | slam::SENSOR_RGB_CAMERA | slam::SENSOR_IMU | slam::SENSOR_GNSS_RTK;
            p.rig_config = GenericSensorRigConfig::presetRoboSenseAiry();
            list.push_back(p);
        }
        return list;
    }
};

}  // namespace sensors
}  // namespace avscan
