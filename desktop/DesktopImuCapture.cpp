#include "DesktopImuCapture.hpp"
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>

#ifdef AVSLAM_HAS_ORBBEC_SDK
#include <libobsensor/ObSensor.hpp>
#endif

#ifdef _WIN32
#include <windows.h>
#include <sensorsapi.h>
#include <sensors.h>
#include <propvarutil.h>
#elif defined(__linux__)
#include <filesystem>
#endif

namespace avscan {
namespace desktop {

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kGravityMps2 = 9.80665;

double nowSteadySec() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

#ifdef _WIN32
bool readSensorFloat(ISensorDataReport* report, REFPROPERTYKEY key, double& out_val) {
    PROPVARIANT pv;
    PropVariantInit(&pv);
    HRESULT hr = report->GetSensorValue(key, &pv);
    if (FAILED(hr)) {
        PropVariantClear(&pv);
        return false;
    }
    bool ok = false;
    if (pv.vt == VT_R4) {
        out_val = static_cast<double>(pv.fltVal);
        ok = true;
    } else if (pv.vt == VT_R8) {
        out_val = pv.dblVal;
        ok = true;
    }
    PropVariantClear(&pv);
    return ok;
}
#endif

}  // namespace

DesktopImuCapture::~DesktopImuCapture() {
    stop();
}

void DesktopImuCapture::setImuCallback(ImuCallback cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    on_imu_ = std::move(cb);
}

std::string DesktopImuCapture::statusSummary() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_summary_;
}

std::vector<DetectedImuInfo> DesktopImuCapture::enumerateAvailableImus() const {
    std::vector<DetectedImuInfo> list;

#ifdef AVSLAM_HAS_ORBBEC_SDK
    try {
        ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_OFF);
        ob::Context ctx;
        auto devList = ctx.queryDeviceList();
        if (devList && devList->getCount() > 0) {
            for (uint32_t i = 0; i < devList->getCount(); ++i) {
                std::string name = devList->getName(i) ? devList->getName(i) : "Orbbec Camera";
                DetectedImuInfo info;
                info.id = "orbbec_336l_imu";
                info.display_name = name + " 200Hz IMU (USB)";
                info.priority = 85;
                info.streaming_supported = true;
                list.push_back(info);
                break;
            }
        }
    } catch (...) {
        // Orbbec device not connected or busy
    }
#endif

#ifdef _WIN32
    HRESULT hrCom = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    ISensorManager* mgr = nullptr;
    if (SUCCEEDED(CoCreateInstance(CLSID_SensorManager, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&mgr))) && mgr) {
        ISensorCollection* accels = nullptr;
        ISensorCollection* gyros = nullptr;
        ULONG n_acc = 0, n_gyr = 0;
        if (SUCCEEDED(mgr->GetSensorsByType(SENSOR_TYPE_ACCELEROMETER_3D, &accels)) && accels) {
            accels->GetCount(&n_acc);
            accels->Release();
        }
        if (SUCCEEDED(mgr->GetSensorsByType(SENSOR_TYPE_GYROMETER_3D, &gyros)) && gyros) {
            gyros->GetCount(&n_gyr);
            gyros->Release();
        }
        if (n_acc > 0 || n_gyr > 0) {
            DetectedImuInfo info;
            info.id = "host_sensor_imu";
            info.display_name = "Host / PC Built-in IMU (Accel+Gyro)";
            info.priority = 65;
            info.streaming_supported = true;
            list.push_back(info);
        }
        mgr->Release();
    }
    if (SUCCEEDED(hrCom)) CoUninitialize();
#elif defined(__linux__)
    namespace fs = std::filesystem;
    std::error_code ec;
    if (fs::exists("/sys/bus/iio/devices", ec)) {
        for (const auto& entry : fs::directory_iterator("/sys/bus/iio/devices", ec)) {
            if (fs::exists(entry.path() / "in_accel_x_raw", ec) ||
                fs::exists(entry.path() / "in_anglvel_x_raw", ec)) {
                DetectedImuInfo info;
                info.id = "linux_iio_imu";
                info.display_name = "Linux IIO IMU (" + entry.path().filename().string() + ")";
                info.priority = 65;
                info.streaming_supported = true;
                list.push_back(info);
                break;
            }
        }
    }
#endif
    return list;
}

bool DesktopImuCapture::start(const std::string& preferred_imu_id) {
    if (running_.load()) return true;
    samples_delivered_.store(0);
    running_.store(true);
    worker_ = std::thread(&DesktopImuCapture::captureLoop, this, preferred_imu_id);
    return true;
}

void DesktopImuCapture::stop() {
    if (!running_.exchange(false)) return;
    if (worker_.joinable()) worker_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    status_summary_ = "IDLE";
}

void DesktopImuCapture::captureLoop(std::string preferred_imu_id) {
#ifdef AVSLAM_HAS_ORBBEC_SDK
    // Priority 1 (non-Airy): Orbbec Gemini 336L / Orbbec USB 200Hz hardware IMU
    const bool try_orbbec = preferred_imu_id.empty() || (preferred_imu_id == "orbbec_336l_imu");
    if (try_orbbec) {
        try {
            ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_OFF);
            ob::Context ctx;
            auto devList = ctx.queryDeviceList();
            if (devList && devList->getCount() > 0) {
                auto device = devList->getDevice(0);
                std::string dev_name = device->getDeviceInfo()->getName();
                auto accelSensor = device->getSensor(OB_SENSOR_ACCEL);
                auto gyroSensor = device->getSensor(OB_SENSOR_GYRO);
                if (accelSensor && gyroSensor) {
                    ob::Pipeline pipe(device);
                    auto config = std::make_shared<ob::Config>();
                    config->enableAccelStream();
                    config->enableGyroStream();
                    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
                    pipe.start(config);

                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        status_summary_ = "ACTIVE: " + dev_name + " 200Hz IMU";
                    }

                    bool gravity_sign_checked = false;
                    double z_sign = 1.0;

                    while (running_.load()) {
                        auto frameSet = pipe.waitForFrameset(100);
                        if (!frameSet) continue;
                        auto accelRaw = frameSet->getFrame(OB_FRAME_ACCEL);
                        auto gyroRaw = frameSet->getFrame(OB_FRAME_GYRO);
                        if (!accelRaw || !gyroRaw) continue;

                        auto accelFrame = accelRaw->as<ob::AccelFrame>();
                        auto gyroFrame = gyroRaw->as<ob::GyroFrame>();
                        if (!accelFrame || !gyroFrame) continue;

                        auto av = accelFrame->getValue();
                        auto gv = gyroFrame->getValue();

                        // Convert Orbbec optical IMU frame (+X right, +Y down, +Z fwd)
                        // -> AV-SLAM FLU body frame (+X fwd, +Y left, +Z up).
                        // Auto-verify gravity Z sign on first valid stationary sample so
                        // upright gravity ALWAYS points +Z (+9.81 m/s^2) and never causes 2g free-fall!
                        double ax = av.z;
                        double ay = -av.x;
                        double az = -av.y;
                        if (!gravity_sign_checked && std::abs(az) > 4.0) {
                            z_sign = (az < 0.0) ? -1.0 : 1.0;
                            gravity_sign_checked = true;
                        }
                        core::ImuMeasurement meas;
                        meas.timestamp_sec = nowSteadySec();
                        meas.accel_mps2 = math::Vec3d(ax, ay * z_sign, az * z_sign);
                        meas.gyro_rads  = math::Vec3d(gv.z, -gv.x * z_sign, -gv.y * z_sign);
                        samples_delivered_.fetch_add(1);

                        ImuCallback cb;
                        {
                            std::lock_guard<std::mutex> lock(mutex_);
                            cb = on_imu_;
                        }
                        if (cb) cb(meas);
                    }

                    pipe.stop();
                    return;
                }
            }
        } catch (...) {
            // Fall through to Host PC IMU if Orbbec device is absent or unavailable
        }
    }
#endif

#ifdef _WIN32
    HRESULT hrCom = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    ISensorManager* mgr = nullptr;
    ISensor* accel_sensor = nullptr;
    ISensor* gyro_sensor = nullptr;

    if (SUCCEEDED(CoCreateInstance(CLSID_SensorManager, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&mgr))) && mgr) {
        ISensorCollection* col = nullptr;
        ULONG count = 0;
        if (SUCCEEDED(mgr->GetSensorsByType(SENSOR_TYPE_ACCELEROMETER_3D, &col)) && col) {
            if (SUCCEEDED(col->GetCount(&count)) && count > 0) {
                col->GetAt(0, &accel_sensor);
            }
            col->Release();
        }
        col = nullptr;
        count = 0;
        if (SUCCEEDED(mgr->GetSensorsByType(SENSOR_TYPE_GYROMETER_3D, &col)) && col) {
            if (SUCCEEDED(col->GetCount(&count)) && count > 0) {
                col->GetAt(0, &gyro_sensor);
            }
            col->Release();
        }
    }

    if (!accel_sensor && !gyro_sensor) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            status_summary_ = "NO_USB_OR_HOST_IMU";
        }
        if (mgr) mgr->Release();
        if (SUCCEEDED(hrCom)) CoUninitialize();
        running_.store(false);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (accel_sensor && gyro_sensor) {
            status_summary_ = "ACTIVE: Host 6-DOF IMU (WinSensor)";
        } else if (gyro_sensor) {
            status_summary_ = "ACTIVE: Host Gyro IMU (WinSensor)";
        } else {
            status_summary_ = "ACTIVE: Host Accel IMU (WinSensor)";
        }
    }

    while (running_.load()) {
        core::ImuMeasurement meas;
        meas.timestamp_sec = nowSteadySec();
        bool got_any = false;

        if (accel_sensor) {
            ISensorDataReport* rep = nullptr;
            if (SUCCEEDED(accel_sensor->GetData(&rep)) && rep) {
                double ax = 0.0, ay = 0.0, az = 1.0;
                if (readSensorFloat(rep, SENSOR_DATA_TYPE_ACCELERATION_X_G, ax) &&
                    readSensorFloat(rep, SENSOR_DATA_TYPE_ACCELERATION_Y_G, ay) &&
                    readSensorFloat(rep, SENSOR_DATA_TYPE_ACCELERATION_Z_G, az)) {
                    meas.accel_mps2 = math::Vec3d(ax * kGravityMps2, ay * kGravityMps2, az * kGravityMps2);
                    got_any = true;
                }
                rep->Release();
            }
        }
        if (gyro_sensor) {
            ISensorDataReport* rep = nullptr;
            if (SUCCEEDED(gyro_sensor->GetData(&rep)) && rep) {
                double gx = 0.0, gy = 0.0, gz = 0.0;
                if (readSensorFloat(rep, SENSOR_DATA_TYPE_ANGULAR_VELOCITY_X_DEGREES_PER_SECOND, gx) &&
                    readSensorFloat(rep, SENSOR_DATA_TYPE_ANGULAR_VELOCITY_Y_DEGREES_PER_SECOND, gy) &&
                    readSensorFloat(rep, SENSOR_DATA_TYPE_ANGULAR_VELOCITY_Z_DEGREES_PER_SECOND, gz)) {
                    meas.gyro_rads = math::Vec3d(gx * kDegToRad, gy * kDegToRad, gz * kDegToRad);
                    got_any = true;
                }
                rep->Release();
            }
        }

        if (got_any) {
            samples_delivered_.fetch_add(1);
            ImuCallback cb;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                cb = on_imu_;
            }
            if (cb) cb(meas);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (accel_sensor) accel_sensor->Release();
    if (gyro_sensor) gyro_sensor->Release();
    if (mgr) mgr->Release();
    if (SUCCEEDED(hrCom)) CoUninitialize();
#else
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_summary_ = "NO_USB_OR_HOST_IMU";
    }
    running_.store(false);
#endif
}

}  // namespace desktop
}  // namespace avscan
