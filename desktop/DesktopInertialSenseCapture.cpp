#include "DesktopInertialSenseCapture.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <sstream>

#ifdef _WIN32
#include <windows.h>
#include <setupapi.h>
#include <devguid.h>
#elif defined(__linux__)
#include <fcntl.h>
#include <filesystem>
#include <termios.h>
#include <unistd.h>
#endif

namespace avscan {
namespace desktop {

namespace {

double nowSteadySec() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

std::vector<std::string> splitCsv(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        out.push_back(item);
    }
    return out;
}

bool parseDoubleSafe(const std::string& s, double& out) {
    if (s.empty()) return false;
    try {
        out = std::stod(s);
        return std::isfinite(out);
    } catch (...) {
        return false;
    }
}

}  // namespace

std::string DesktopInertialSenseCapture::buildNmeaCommand(const std::string& body) {
    uint8_t csum = 0;
    for (char c : body) csum ^= static_cast<uint8_t>(c);
    char tail[16]{};
    std::snprintf(tail, sizeof(tail), "*%02X\r\n", static_cast<unsigned>(csum));
    return "$" + body + tail;
}

DesktopInertialSenseCapture::~DesktopInertialSenseCapture() {
    stop();
}

void DesktopInertialSenseCapture::setCallbacks(ImuCallback on_imu, NmeaCallback on_nmea) {
    std::lock_guard<std::mutex> lock(mutex_);
    on_imu_ = std::move(on_imu);
    on_nmea_ = std::move(on_nmea);
}

std::string DesktopInertialSenseCapture::statusSummary() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_summary_;
}

std::vector<InertialSensePortInfo> DesktopInertialSenseCapture::enumerateCandidatePorts() const {
    std::vector<InertialSensePortInfo> ports;
#ifdef _WIN32
    HDEVINFO hDevInfo = SetupDiGetClassDevsA(&GUID_DEVCLASS_PORTS, nullptr, nullptr, DIGCF_PRESENT);
    if (hDevInfo != INVALID_HANDLE_VALUE) {
        SP_DEVINFO_DATA devData{};
        devData.cbSize = sizeof(devData);
        for (DWORD i = 0; SetupDiEnumDeviceInfo(hDevInfo, i, &devData); ++i) {
            char friendly[256]{};
            char hwid[256]{};
            SetupDiGetDeviceRegistryPropertyA(hDevInfo, &devData, SPDRP_FRIENDLYNAME, nullptr,
                                              reinterpret_cast<PBYTE>(friendly), sizeof(friendly) - 1, nullptr);
            SetupDiGetDeviceRegistryPropertyA(hDevInfo, &devData, SPDRP_HARDWAREID, nullptr,
                                              reinterpret_cast<PBYTE>(hwid), sizeof(hwid) - 1, nullptr);
            std::string fn(friendly);
            std::string hw(hwid);
            std::string hwu = hw;
            std::transform(hwu.begin(), hwu.end(), hwu.begin(), [](unsigned char c) {
                return static_cast<char>(std::toupper(c));
            });

            // Extract COMx from friendly name "(COM5)"
            auto lp = fn.rfind("(COM");
            auto rp = fn.rfind(')');
            if (lp == std::string::npos || rp == std::string::npos || rp <= lp + 1) continue;
            std::string com = fn.substr(lp + 1, rp - lp - 1);

            std::string vp;
            auto vpos = hwu.find("VID_");
            auto ppos = hwu.find("PID_");
            if (vpos != std::string::npos && ppos != std::string::npos &&
                vpos + 8 <= hwu.size() && ppos + 8 <= hwu.size()) {
                vp = hwu.substr(vpos + 4, 4) + ":" + hwu.substr(ppos + 4, 4);
            }

            // Inertial Sense uINS-3 / IMX-5 use FTDI (0403), STM32 CDC (0483), or Silicon Labs (10C4)
            bool likely_is = (fn.find("Inertial") != std::string::npos ||
                              fn.find("uINS") != std::string::npos ||
                              fn.find("IMX") != std::string::npos ||
                              vp.rfind("0403:", 0) == 0 ||
                              vp.rfind("0483:", 0) == 0 ||
                              vp.rfind("10C4:", 0) == 0 ||
                              fn.find("USB Serial") != std::string::npos);
            if (likely_is) {
                InertialSensePortInfo info;
                info.port_name = com;
                info.friendly_name = "Inertial Sense uINS-3 / IMX-5 (" + com + ")";
                info.vid_pid = vp.empty() ? ("SER:" + com) : vp;
                ports.push_back(info);
            }
        }
        SetupDiDestroyDeviceInfoList(hDevInfo);
    }
#elif defined(__linux__)
    namespace fs = std::filesystem;
    std::error_code ec;
    if (fs::exists("/dev", ec)) {
        for (const auto& entry : fs::directory_iterator("/dev", ec)) {
            std::string n = entry.path().filename().string();
            if (n.rfind("ttyACM", 0) == 0 || n.rfind("ttyUSB", 0) == 0) {
                InertialSensePortInfo info;
                info.port_name = entry.path().string();
                info.friendly_name = "Inertial Sense uINS-3 / IMX-5 (" + n + ")";
                info.vid_pid = "SER:" + n;
                ports.push_back(info);
            }
        }
    }
#endif
    return ports;
}

bool DesktopInertialSenseCapture::start(const std::string& preferred_port) {
    if (running_.load()) return true;
    imu_samples_delivered_.store(0);
    nmea_sentences_delivered_.store(0);
    running_.store(true);
    worker_ = std::thread(&DesktopInertialSenseCapture::captureLoop, this, preferred_port);
    return true;
}

void DesktopInertialSenseCapture::stop() {
    if (!running_.exchange(false)) return;
    if (worker_.joinable()) worker_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    status_summary_ = "IDLE";
}

void DesktopInertialSenseCapture::processAsciiLine(const std::string& raw_line) {
    if (raw_line.size() < 6 || raw_line[0] != '$') return;
    std::string line = raw_line;
    auto star = line.find('*');
    if (star != std::string::npos) line = line.substr(0, star);

    // 1. Standard NMEA RTK-GPS ($GNGGA / $GPGGA) -> forward directly to GnssRtkProcessor
    if (line.rfind("$GNGGA,", 0) == 0 || line.rfind("$GPGGA,", 0) == 0) {
        nmea_sentences_delivered_.fetch_add(1);
        NmeaCallback cb;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cb = on_nmea_;
        }
        if (cb) cb(raw_line);
        return;
    }

    // 2. Inertial Sense Processed IMU ($PIMU,time,gx,gy,gz,ax,ay,az,...)
    //    or Pre-integrated IMU ($PPIMU,time,dThetaX,dThetaY,dThetaZ,dVelX,dVelY,dVelZ,dt)
    if (line.rfind("$PIMU,", 0) == 0 || line.rfind("$PPIMU,", 0) == 0) {
        const bool is_preint = (line.rfind("$PPIMU,", 0) == 0);
        auto fields = splitCsv(line);
        if (fields.size() < 8) return;

        double gx = 0.0, gy = 0.0, gz = 0.0;
        double ax = 0.0, ay = 0.0, az = 0.0;
        if (!parseDoubleSafe(fields[2], gx) || !parseDoubleSafe(fields[3], gy) || !parseDoubleSafe(fields[4], gz) ||
            !parseDoubleSafe(fields[5], ax) || !parseDoubleSafe(fields[6], ay) || !parseDoubleSafe(fields[7], az)) {
            return;
        }
        if (is_preint && fields.size() >= 9) {
            double dt = 0.005;
            if (parseDoubleSafe(fields[8], dt) && dt > 1e-5) {
                gx /= dt; gy /= dt; gz /= dt;
                ax /= dt; ay /= dt; az /= dt;
            }
        }

        // Convert Inertial Sense NED body frame (+X fwd, +Y right, +Z down)
        // -> AV-SLAM FLU body frame (+X fwd, +Y left, +Z up)
        core::ImuMeasurement meas;
        meas.timestamp_sec = nowSteadySec();
        meas.gyro_rads  = math::Vec3d(gx, -gy, -gz);
        meas.accel_mps2 = math::Vec3d(ax, -ay, -az);
        imu_samples_delivered_.fetch_add(1);

        ImuCallback cb;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cb = on_imu_;
        }
        if (cb) cb(meas);
    }
}

void DesktopInertialSenseCapture::captureLoop(std::string preferred_port) {
    std::string port_to_open = preferred_port;
    if (port_to_open.empty()) {
        auto cands = enumerateCandidatePorts();
        if (!cands.empty()) port_to_open = cands.front().port_name;
    }
    if (port_to_open.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_summary_ = "NOT_CONNECTED";
        running_.store(false);
        return;
    }

    // Official Inertial Sense ASCII setup: stop stale broadcasts + enable $PIMU (ID 0) & $GGA (ID 6)
    const std::string cmd_stop = buildNmeaCommand("STPB");
    const std::string cmd_asce = buildNmeaCommand("ASCE,0,0,1,6,1");

#ifdef _WIN32
    std::string win_path = (port_to_open.rfind("\\\\.\\", 0) == 0) ? port_to_open : ("\\\\.\\" + port_to_open);
    HANDLE hCom = CreateFileA(win_path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (hCom == INVALID_HANDLE_VALUE) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_summary_ = "PORT_BUSY (" + port_to_open + ")";
        running_.store(false);
        return;
    }

    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(hCom, &dcb);
    dcb.BaudRate = 921600;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    SetCommState(hCom, &dcb);

    COMMTIMEOUTS to{};
    to.ReadIntervalTimeout = 10;
    to.ReadTotalTimeoutConstant = 40;
    to.ReadTotalTimeoutMultiplier = 0;
    SetCommTimeouts(hCom, &to);

    DWORD written = 0;
    WriteFile(hCom, cmd_stop.c_str(), static_cast<DWORD>(cmd_stop.size()), &written, nullptr);
    Sleep(20);
    WriteFile(hCom, cmd_asce.c_str(), static_cast<DWORD>(cmd_asce.size()), &written, nullptr);

    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_summary_ = "ACTIVE: Inertial Sense uINS-3/IMX-5 (" + port_to_open + " @921600)";
    }

    std::string accum;
    char buf[512]{};
    while (running_.load()) {
        DWORD nread = 0;
        if (ReadFile(hCom, buf, sizeof(buf) - 1, &nread, nullptr) && nread > 0) {
            accum.append(buf, nread);
            size_t pos = 0;
            while ((pos = accum.find('\n')) != std::string::npos) {
                std::string line = accum.substr(0, pos);
                accum.erase(0, pos + 1);
                if (!line.empty() && line.back() == '\r') line.pop_back();
                processAsciiLine(line);
            }
            if (accum.size() > 4096) accum.clear();
        }
    }
    CloseHandle(hCom);
#elif defined(__linux__)
    int fd = ::open(port_to_open.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_summary_ = "PORT_OPEN_FAILED (" + port_to_open + ")";
        running_.store(false);
        return;
    }
    termios tty{};
    if (::tcgetattr(fd, &tty) == 0) {
        ::cfmakeraw(&tty);
        ::cfsetispeed(&tty, B921600);
        ::cfsetospeed(&tty, B921600);
        ::tcsetattr(fd, TCSANOW, &tty);
    }
    (void)::write(fd, cmd_stop.c_str(), cmd_stop.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    (void)::write(fd, cmd_asce.c_str(), cmd_asce.size());

    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_summary_ = "ACTIVE: Inertial Sense uINS-3/IMX-5 (" + port_to_open + " @921600)";
    }

    std::string accum;
    char buf[512]{};
    while (running_.load()) {
        ssize_t n = ::read(fd, buf, sizeof(buf) - 1);
        if (n > 0) {
            accum.append(buf, static_cast<size_t>(n));
            size_t pos = 0;
            while ((pos = accum.find('\n')) != std::string::npos) {
                std::string line = accum.substr(0, pos);
                accum.erase(0, pos + 1);
                if (!line.empty() && line.back() == '\r') line.pop_back();
                processAsciiLine(line);
            }
            if (accum.size() > 4096) accum.clear();
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    ::close(fd);
#endif
}

}  // namespace desktop
}  // namespace avscan
