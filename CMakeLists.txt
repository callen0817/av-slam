cmake_minimum_required(VERSION 3.20)
project(AVSlam VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(AVSLAM_BUILD_TESTS "Build automated unit, integration, and regression tests" ON)
option(AVSLAM_BUILD_IOS_APP "Build native iOS/iPadOS AV SLAM application target" OFF)
option(AVSLAM_BUILD_DESKTOP_APP "Build native Windows/Linux/Jetson AV SLAM Desktop GUI application" OFF)
option(AVSLAM_BUILD_AB_BENCHMARK "Build host-only A/B benchmark comparing AV-SLAM against legacy FastLivo2Engine" ON)

if(CMAKE_SYSTEM_NAME STREQUAL "iOS")
    set(AVSLAM_BUILD_IOS_APP ON)
    enable_language(OBJCXX)
endif()

# Core portable 100% First-Party Proprietary C++17 AV-SLAM Library (Zero GPL, Zero Third-Party SLAM)
# Shared cross-platform core for AV Scan iOS, AV scanAR (Jetson), and AV Scan (Desktop).
set(AVSLAM_CORE_SOURCES
    src/sensors/AiryDiscovery.cpp
    src/sensors/AiryDecoder.cpp
    src/sensors/AiryUdpReceiver.cpp
    src/sensors/DepthProviderRegistry.cpp
    src/sensors/GnssRtkReceiver.cpp
    src/sensors/LivoxMid360Adapter.cpp
    src/sensors/OusterAdapter.cpp
    src/cslam/CollaborativeSlamEngine.cpp
    src/cslam/CollaborativeTransport.cpp
    src/sync/TimeSynchronizer.cpp
    src/filters/PointCloudFilters.cpp
    src/processing/PointCloudCleanup.cpp
    src/slam/VoxelHashMap.cpp
    src/slam/AvImuPreintegrator.cpp
    src/slam/AvSurfelMap.cpp
    src/slam/AvSlamEngine.cpp
    src/slam/SensorAgnosticSlamEngine.cpp
    src/calibration/TargetlessCalibrator.cpp
    src/calibration/OnlineAutoCalibrator.cpp
    src/backend/ScanContext.cpp
    src/backend/GicpVerifier.cpp
    src/backend/PoseGraphOptimizer.cpp
    src/quality/QualityMonitor.cpp
    src/storage/ProjectStore.cpp
    src/export/PointCloudExporter.cpp
    src/export/AvSpatialBundleExporter.cpp
    src/core/DeviceMountProfile.cpp
    src/system/ResourceController.cpp
    src/terrestrial/TerrestrialStationManager.cpp
    src/sensors/HardwareRegistry.cpp
)

add_library(avslam_core STATIC ${AVSLAM_CORE_SOURCES})

target_include_directories(avslam_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(avslam_core PUBLIC AVSLAM_PROPRIETARY_ENGINE=1)

if(WIN32)
    target_compile_definitions(avslam_core PUBLIC _CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
    target_link_libraries(avslam_core PUBLIC ws2_32 iphlpapi)
elseif(UNIX AND NOT APPLE)
    target_link_libraries(avslam_core PUBLIC pthread m)
endif()

# Native iOS / iPadOS Application Target (Installs side-by-side with AV Scan on same iPhone/iPad!)
if(AVSLAM_BUILD_IOS_APP)
    set(AVSLAM_IOS_SOURCES
        ios/AVScan/main.mm
        ios/AVScan/AppDelegate.mm
        ios/AVScan/Bridge/AVScanCoreBridge.mm
        ios/AVScan/Sensors/CameraCaptureManager.mm
        ios/AVScan/Sensors/MultipeerCollabTransport.mm
        ios/AVScan/Rendering/MetalPointCloudRenderer.mm
        ios/AVScan/UI/MainScannerViewController.mm
        ios/AVScan/UI/DiagnosticsViewController.mm
        ios/AVScan/UI/CalibrationViewController.mm
        ios/AVScan/UI/FloorPlanView.mm
        ios/AVScan/UI/AppleGlassTheme.mm
        ios/AVScan/UI/AppleGlassSplashScreenView.mm
    )

    add_executable(AVSlam MACOSX_BUNDLE ${AVSLAM_IOS_SOURCES})

    target_include_directories(AVSlam PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/ios/AVScan
    )

    target_compile_options(AVSlam PRIVATE -fobjc-arc)

    target_link_libraries(AVSlam PRIVATE
        avslam_core
        "-framework UIKit"
        "-framework Foundation"
        "-framework Metal"
        "-framework MetalKit"
        "-framework AVFoundation"
        "-framework ARKit"
        "-framework CoreMotion"
        "-framework CoreMedia"
        "-framework CoreVideo"
        "-framework CoreGraphics"
        "-framework QuartzCore"
        "-framework Network"
        "-framework MultipeerConnectivity"
        "-framework SystemConfiguration"
        "-framework UniformTypeIdentifiers"
        "-lsqlite3"
    )

    set_target_properties(AVSlam PROPERTIES
        MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/ios/AVScan/Info.plist"
        XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER "com.artificialvision.avslam"
        XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2"
        XCODE_ATTRIBUTE_IPHONEOS_DEPLOYMENT_TARGET "16.0"
    )
endif()

# Automated Test Suite & Side-by-Side A/B Benchmark Targets
if(AVSLAM_BUILD_TESTS AND NOT CMAKE_SYSTEM_NAME STREQUAL "iOS")
    enable_testing()

    add_executable(avslam_tests
        tests/test_main.cpp
        tests/test_m1_airy_discovery.cpp
        tests/test_m2_airy_decoder.cpp
        tests/test_av_slam.cpp
        tests/test_m5_m6_sync_fusion.cpp
        tests/test_m7_calibration.cpp
        tests/test_m8_loop_closure_pgo.cpp
        tests/test_m9_quality_reloc.cpp
        tests/test_m10_storage_export.cpp
        tests/test_m11_hardening.cpp
    )
    target_link_libraries(avslam_tests PRIVATE avslam_core)
    add_test(NAME AVSlamFullTestSuite COMMAND avslam_tests)

    add_executable(avslam_autocalib_test tests/test_online_auto_calibrator.cpp)
    target_link_libraries(avslam_autocalib_test PRIVATE avslam_core)
    add_test(NAME OnlineAutoCalibratorTest COMMAND avslam_autocalib_test)

    add_executable(avslam_terrestrial_test tests/test_terrestrial_mode.cpp)
    target_link_libraries(avslam_terrestrial_test PRIVATE avslam_core)
    add_test(NAME TerrestrialModeTest COMMAND avslam_terrestrial_test)

    add_executable(avslam_cslam_depth_test tests/test_cslam_and_depth.cpp)
    target_link_libraries(avslam_cslam_depth_test PRIVATE avslam_core)
    add_test(NAME CslamAndDepthTest COMMAND avslam_cslam_depth_test)

    add_executable(avslam_agnostic_test tests/test_sensor_agnostic_slam.cpp)
    target_link_libraries(avslam_agnostic_test PRIVATE avslam_core)
    add_test(NAME SensorAgnosticSlamTest COMMAND avslam_agnostic_test)

    add_executable(avslam_live_regression_test tests/test_live_scan_regression.cpp)
    target_link_libraries(avslam_live_regression_test PRIVATE avslam_core)
    target_compile_definitions(avslam_live_regression_test PRIVATE AVSCAN_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME LiveScanRegressionTest COMMAND avslam_live_regression_test)

    # Host-only A/B Benchmark comparing proprietary AV-SLAM vs legacy FastLivo2Engine on real captures.
    # FastLivo2Engine.cpp is NEVER linked into avslam_core or the shipping AVSlam iOS app.
    if(AVSLAM_BUILD_AB_BENCHMARK)
        add_executable(avslam_ab_benchmark
            tests/test_av_slam_benchmark.cpp
            src/slam/FastLivo2Engine.cpp
        )
        target_link_libraries(avslam_ab_benchmark PRIVATE avslam_core)
        target_compile_definitions(avslam_ab_benchmark PRIVATE AVSCAN_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
        add_test(NAME AVSlamVsFastLivo2Benchmark COMMAND avslam_ab_benchmark)
    endif()
endif()

# Native Windows & Linux/Jetson Desktop GUI Application (GLFW + OpenGL + Dear ImGui)
# Replaces standalone av-scanar with the 100% proprietary avslam_core engine!
if(AVSLAM_BUILD_DESKTOP_APP AND NOT CMAKE_SYSTEM_NAME STREQUAL "iOS")
    include(FetchContent)
    find_package(OpenGL REQUIRED)

    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG 3.4
        GIT_SHALLOW TRUE
    )
    FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.91.5
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(glfw imgui)

    add_executable(AVSlamDesktop
        desktop/AVSlamDesktopApp.cpp
        desktop/DesktopSessionController.cpp
        desktop/DesktopCameraCapture.cpp
        desktop/DesktopDepthCapture.cpp
        desktop/DesktopImuCapture.cpp
        desktop/DesktopHardwareManager.cpp
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.cpp
    )

    target_include_directories(AVSlamDesktop PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/desktop
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
    )

    target_link_libraries(AVSlamDesktop PRIVATE
        avslam_core
        glfw
        OpenGL::GL
    )

    # Auto-detect OrbbecSDK v2 (Apache-2.0) for native 200Hz Gemini 336L IMU on Windows & Linux/Jetson
    set(AVSLAM_ORBBEC_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build/_deps/orbbec_sdk" CACHE PATH "Path to OrbbecSDK v2")
    if(EXISTS "${AVSLAM_ORBBEC_SDK_DIR}/include/libobsensor/ObSensor.hpp")
        message(STATUS "AV-SLAM: Enabling native OrbbecSDK v2 IMU support from ${AVSLAM_ORBBEC_SDK_DIR}")
        target_compile_definitions(AVSlamDesktop PRIVATE AVSLAM_HAS_ORBBEC_SDK=1)
        target_include_directories(AVSlamDesktop PRIVATE "${AVSLAM_ORBBEC_SDK_DIR}/include")
        if(WIN32 AND EXISTS "${AVSLAM_ORBBEC_SDK_DIR}/lib/OrbbecSDK.lib")
            target_link_libraries(AVSlamDesktop PRIVATE "${AVSLAM_ORBBEC_SDK_DIR}/lib/OrbbecSDK.lib")
        elseif(UNIX AND NOT APPLE)
            find_library(ORBBEC_SDK_LIB NAMES OrbbecSDK PATHS "${AVSLAM_ORBBEC_SDK_DIR}/lib" NO_DEFAULT_PATH)
            if(ORBBEC_SDK_LIB)
                target_link_libraries(AVSlamDesktop PRIVATE ${ORBBEC_SDK_LIB})
                set_target_properties(AVSlamDesktop PROPERTIES BUILD_RPATH "${AVSLAM_ORBBEC_SDK_DIR}/lib")
            endif()
        endif()
    endif()

    if(WIN32)
        target_link_libraries(AVSlamDesktop PRIVATE mf mfplat mfreadwrite mfuuid ole32 sensorsapi portabledeviceguids winhttp)
    elseif(UNIX AND NOT APPLE)
        target_link_libraries(AVSlamDesktop PRIVATE pthread dl)
    endif()
endif()
