#include "DesktopCameraCapture.hpp"
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstring>

#ifdef _WIN32
#include <windows.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <mferror.h>
#elif defined(__linux__)
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace avscan {
namespace desktop {

namespace {

std::string toLowerCopy(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return out;
}

double steadyNowSec() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

#ifdef _WIN32
std::string wideToUtf8(const wchar_t* wstr) {
    if (!wstr) return "";
    int len = WideCharToMultiByte(CP_UTF8, 0, wstr, -1, nullptr, 0, nullptr, nullptr);
    if (len <= 1) return "";
    std::string out(static_cast<size_t>(len - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, wstr, -1, &out[0], len, nullptr, nullptr);
    return out;
}
#endif

}  // namespace

DesktopCameraCapture::~DesktopCameraCapture() {
    stop();
}

void DesktopCameraCapture::setPreferredCameraHint(const std::string& name_hint,
                                                  const core::CameraIntrinsics& fallback_intrinsics) {
    std::lock_guard<std::mutex> lock(mutex_);
    preferred_hint_ = name_hint;
    fallback_intrinsics_ = fallback_intrinsics;
}

void DesktopCameraCapture::setFrameCallback(FrameCallback cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    on_frame_ = std::move(cb);
}

std::string DesktopCameraCapture::activeCameraName() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_camera_name_;
}

std::string DesktopCameraCapture::statusSummary() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_summary_;
}

std::vector<std::string> DesktopCameraCapture::enumerateCameraNames() const {
    std::vector<std::string> names;
#ifdef _WIN32
    HRESULT hrCom = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    MFStartup(MF_VERSION);
    IMFAttributes* attrs = nullptr;
    if (SUCCEEDED(MFCreateAttributes(&attrs, 1))) {
        attrs->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        IMFActivate** devices = nullptr;
        UINT32 count = 0;
        if (SUCCEEDED(MFEnumDeviceSources(attrs, &devices, &count))) {
            for (UINT32 i = 0; i < count; ++i) {
                wchar_t* fname = nullptr;
                UINT32 flen = 0;
                if (SUCCEEDED(devices[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME, &fname, &flen))) {
                    names.push_back(wideToUtf8(fname));
                    CoTaskMemFree(fname);
                }
                devices[i]->Release();
            }
            CoTaskMemFree(devices);
        }
        attrs->Release();
    }
    MFShutdown();
    if (SUCCEEDED(hrCom)) CoUninitialize();
#elif defined(__linux__)
    for (int i = 0; i < 12; ++i) {
        std::string dev = "/dev/video" + std::to_string(i);
        int fd = ::open(dev.c_str(), O_RDWR | O_NONBLOCK);
        if (fd < 0) continue;
        v4l2_capability cap{};
        if (::ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0 && (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
            names.push_back(std::string(reinterpret_cast<const char*>(cap.card)) + " (" + dev + ")");
        }
        ::close(fd);
    }
#endif
    return names;
}

bool DesktopCameraCapture::start(int target_fps) {
    if (running_.load()) return true;
    target_fps_ = std::clamp(target_fps, 5, 60);
    frames_delivered_.store(0);
    measured_fps_.store(0.0);
    running_.store(true);
    worker_ = std::thread(&DesktopCameraCapture::captureLoop, this);
    return true;
}

void DesktopCameraCapture::stop() {
    if (!running_.exchange(false)) return;
    if (worker_.joinable()) {
        worker_.join();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    status_summary_ = "IDLE";
}

void DesktopCameraCapture::captureLoop() {
#ifdef _WIN32
    HRESULT hrCom = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    if (FAILED(MFStartup(MF_VERSION))) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_summary_ = "MF_INIT_FAILED";
        running_.store(false);
        if (SUCCEEDED(hrCom)) CoUninitialize();
        return;
    }

    IMFAttributes* attrs = nullptr;
    IMFActivate** devices = nullptr;
    UINT32 count = 0;
    IMFMediaSource* source = nullptr;
    IMFSourceReader* reader = nullptr;

    std::string hint;
    core::CameraIntrinsics fallback_intr;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        hint = toLowerCopy(preferred_hint_);
        fallback_intr = fallback_intrinsics_;
    }

    if (SUCCEEDED(MFCreateAttributes(&attrs, 1))) {
        attrs->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        if (SUCCEEDED(MFEnumDeviceSources(attrs, &devices, &count)) && count > 0) {
            UINT32 chosen = 0;
            std::string chosen_name;
            for (UINT32 i = 0; i < count; ++i) {
                wchar_t* fname = nullptr;
                UINT32 flen = 0;
                std::string n;
                if (SUCCEEDED(devices[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME, &fname, &flen))) {
                    n = wideToUtf8(fname);
                    CoTaskMemFree(fname);
                }
                std::string nl = toLowerCopy(n);
                if (i == 0) chosen_name = n;
                if ((!hint.empty() && nl.find(hint) != std::string::npos) ||
                    nl.find("orbbec") != std::string::npos ||
                    nl.find("gemini") != std::string::npos ||
                    nl.find("336") != std::string::npos) {
                    chosen = i;
                    chosen_name = n;
                    break;
                }
            }
            if (SUCCEEDED(devices[chosen]->ActivateObject(IID_PPV_ARGS(&source)))) {
                IMFAttributes* rAttrs = nullptr;
                MFCreateAttributes(&rAttrs, 1);
                if (rAttrs) rAttrs->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING, TRUE);
                if (SUCCEEDED(MFCreateSourceReaderFromMediaSource(source, rAttrs, &reader))) {
                    IMFMediaType* outType = nullptr;
                    if (SUCCEEDED(MFCreateMediaType(&outType))) {
                        outType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
                        outType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_RGB32);
                        reader->SetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, nullptr, outType);
                        outType->Release();
                    }
                    std::lock_guard<std::mutex> lock(mutex_);
                    active_camera_name_ = chosen_name;
                    status_summary_ = "LIVE: " + chosen_name;
                }
                if (rAttrs) rAttrs->Release();
            }
            for (UINT32 i = 0; i < count; ++i) devices[i]->Release();
            CoTaskMemFree(devices);
        }
        attrs->Release();
    }

    if (!reader) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_summary_ = "NO_CAMERA (LiDAR-only mode active)";
        if (source) source->Release();
        MFShutdown();
        if (SUCCEEDED(hrCom)) CoUninitialize();
        running_.store(false);
        return;
    }

    UINT32 frameW = 640, frameH = 480;
    IMFMediaType* curType = nullptr;
    if (SUCCEEDED(reader->GetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, &curType))) {
        MFGetAttributeSize(curType, MF_MT_FRAME_SIZE, &frameW, &frameH);
        curType->Release();
    }

    double fps_window_start = steadyNowSec();
    uint64_t fps_window_frames = 0;
    uint64_t seq = 0;

    while (running_.load()) {
        DWORD streamIdx = 0, flags = 0;
        LONGLONG llTs = 0;
        IMFSample* sample = nullptr;
        HRESULT hr = reader->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, &streamIdx, &flags, &llTs, &sample);
        if (FAILED(hr) || !sample) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        IMFMediaBuffer* buf = nullptr;
        if (SUCCEEDED(sample->ConvertToContiguousBuffer(&buf))) {
            BYTE* raw = nullptr;
            DWORD maxLen = 0, curLen = 0;
            if (SUCCEEDED(buf->Lock(&raw, &maxLen, &curLen)) && raw && curLen >= frameW * frameH * 4) {
                int scale = (frameW >= 1280) ? 4 : ((frameW >= 640) ? 2 : 1);
                int dstW = static_cast<int>(frameW) / scale;
                int dstH = static_cast<int>(frameH) / scale;

                core::RgbFrame rf;
                rf.frame_id = ++seq;
                rf.timestamp_sec = steadyNowSec();
                rf.intrinsics.width = dstW;
                rf.intrinsics.height = dstH;
                double fx_src = (fallback_intr.fx > 10.0 && fallback_intr.width > 0)
                    ? fallback_intr.fx * (static_cast<double>(frameW) / fallback_intr.width)
                    : static_cast<double>(frameW) * 0.78;
                rf.intrinsics.fx = fx_src / scale;
                rf.intrinsics.fy = rf.intrinsics.fx;
                rf.intrinsics.cx = dstW * 0.5;
                rf.intrinsics.cy = dstH * 0.5;
                rf.rgb_bytes.resize(static_cast<size_t>(dstW * dstH * 3));
                rf.gray_bytes.resize(static_cast<size_t>(dstW * dstH));

                const size_t stride = static_cast<size_t>(frameW) * 4;
                for (int y = 0; y < dstH; ++y) {
                    const BYTE* row = raw + static_cast<size_t>(y * scale) * stride;
                    for (int x = 0; x < dstW; ++x) {
                        const BYTE* px = row + static_cast<size_t>(x * scale) * 4;
                        uint8_t b = px[0], g = px[1], r = px[2];
                        size_t idx = static_cast<size_t>(y * dstW + x);
                        rf.rgb_bytes[idx * 3 + 0] = r;
                        rf.rgb_bytes[idx * 3 + 1] = g;
                        rf.rgb_bytes[idx * 3 + 2] = b;
                        rf.gray_bytes[idx] = static_cast<uint8_t>((77 * r + 150 * g + 29 * b) >> 8);
                    }
                }
                buf->Unlock();

                frames_delivered_.fetch_add(1);
                fps_window_frames++;
                double now = steadyNowSec();
                if (now - fps_window_start >= 1.0) {
                    measured_fps_.store(static_cast<double>(fps_window_frames) / (now - fps_window_start));
                    fps_window_start = now;
                    fps_window_frames = 0;
                }

                FrameCallback cb;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    cb = on_frame_;
                }
                if (cb) cb(rf);
            }
            buf->Release();
        }
        sample->Release();
    }

    reader->Release();
    if (source) source->Release();
    MFShutdown();
    if (SUCCEEDED(hrCom)) CoUninitialize();

#elif defined(__linux__)
    // Native Linux / Jetson V4L2 YUYV capture (Orbbec Gemini 336L RGB endpoint or any UVC camera)
    std::string chosen_dev;
    std::string chosen_name;
    std::string hint;
    core::CameraIntrinsics fallback_intr;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        hint = toLowerCopy(preferred_hint_);
        fallback_intr = fallback_intrinsics_;
    }

    for (int i = 0; i < 12; ++i) {
        std::string dev = "/dev/video" + std::to_string(i);
        int fd = ::open(dev.c_str(), O_RDWR | O_NONBLOCK);
        if (fd < 0) continue;
        v4l2_capability cap{};
        if (::ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0 && (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
            std::string card(reinterpret_cast<const char*>(cap.card));
            std::string cl = toLowerCopy(card);
            if (chosen_dev.empty()) {
                chosen_dev = dev;
                chosen_name = card;
            }
            if ((!hint.empty() && cl.find(hint) != std::string::npos) ||
                cl.find("orbbec") != std::string::npos ||
                cl.find("gemini") != std::string::npos ||
                cl.find("336") != std::string::npos) {
                chosen_dev = dev;
                chosen_name = card;
                ::close(fd);
                break;
            }
        }
        ::close(fd);
    }

    if (chosen_dev.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_summary_ = "NO_CAMERA (LiDAR-only mode active)";
        running_.store(false);
        return;
    }

    int fd = ::open(chosen_dev.c_str(), O_RDWR);
    if (fd < 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_summary_ = "CAMERA_OPEN_FAILED";
        running_.store(false);
        return;
    }

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = 640;
    fmt.fmt.pix.height = 480;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    ::ioctl(fd, VIDIOC_S_FMT, &fmt);
    int frameW = static_cast<int>(fmt.fmt.pix.width);
    int frameH = static_cast<int>(fmt.fmt.pix.height);

    v4l2_requestbuffers req{};
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (::ioctl(fd, VIDIOC_REQBUFS, &req) < 0 || req.count == 0) {
        ::close(fd);
        running_.store(false);
        return;
    }

    struct MmapBuf { void* ptr{nullptr}; size_t len{0}; };
    std::vector<MmapBuf> bufs(req.count);
    for (unsigned i = 0; i < req.count; ++i) {
        v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;
        ::ioctl(fd, VIDIOC_QUERYBUF, &b);
        bufs[i].len = b.length;
        bufs[i].ptr = ::mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        ::ioctl(fd, VIDIOC_QBUF, &b);
    }

    v4l2_buf_type btype = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ::ioctl(fd, VIDIOC_STREAMON, &btype);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_camera_name_ = chosen_name;
        status_summary_ = "LIVE (V4L2): " + chosen_name;
    }

    uint64_t seq = 0;
    double fps_t0 = steadyNowSec();
    uint64_t fps_cnt = 0;
    while (running_.load()) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        timeval tv{0, 100000};
        if (::select(fd + 1, &fds, nullptr, nullptr, &tv) <= 0) continue;

        v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        if (::ioctl(fd, VIDIOC_DQBUF, &b) < 0) continue;

        const uint8_t* yuyv = static_cast<const uint8_t*>(bufs[b.index].ptr);
        int dstW = frameW / 2;
        int dstH = frameH / 2;
        core::RgbFrame rf;
        rf.frame_id = ++seq;
        rf.timestamp_sec = steadyNowSec();
        rf.intrinsics.width = dstW;
        rf.intrinsics.height = dstH;
        rf.intrinsics.fx = dstW * 0.78;
        rf.intrinsics.fy = rf.intrinsics.fx;
        rf.intrinsics.cx = dstW * 0.5;
        rf.intrinsics.cy = dstH * 0.5;
        rf.rgb_bytes.resize(static_cast<size_t>(dstW * dstH * 3));
        rf.gray_bytes.resize(static_cast<size_t>(dstW * dstH));

        for (int y = 0; y < dstH; ++y) {
            const uint8_t* row = yuyv + static_cast<size_t>(y * 2) * static_cast<size_t>(frameW * 2);
            for (int x = 0; x < dstW; ++x) {
                const uint8_t* p = row + static_cast<size_t>(x * 2) * 2;
                int y0 = p[0], u = p[1] - 128, v = p[3] - 128;
                int r = std::clamp(static_cast<int>(y0 + 1.402 * v), 0, 255);
                int g = std::clamp(static_cast<int>(y0 - 0.344136 * u - 0.714136 * v), 0, 255);
                int bl = std::clamp(static_cast<int>(y0 + 1.772 * u), 0, 255);
                size_t idx = static_cast<size_t>(y * dstW + x);
                rf.gray_bytes[idx] = static_cast<uint8_t>(y0);
                rf.rgb_bytes[idx * 3 + 0] = static_cast<uint8_t>(r);
                rf.rgb_bytes[idx * 3 + 1] = static_cast<uint8_t>(g);
                rf.rgb_bytes[idx * 3 + 2] = static_cast<uint8_t>(bl);
            }
        }
        ::ioctl(fd, VIDIOC_QBUF, &b);

        frames_delivered_.fetch_add(1);
        fps_cnt++;
        double now = steadyNowSec();
        if (now - fps_t0 >= 1.0) {
            measured_fps_.store(static_cast<double>(fps_cnt) / (now - fps_t0));
            fps_t0 = now;
            fps_cnt = 0;
        }
        FrameCallback cb;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cb = on_frame_;
        }
        if (cb) cb(rf);
    }

    ::ioctl(fd, VIDIOC_STREAMOFF, &btype);
    for (auto& mb : bufs) {
        if (mb.ptr) ::munmap(mb.ptr, mb.len);
    }
    ::close(fd);
#endif
}

}  // namespace desktop
}  // namespace avscan
