#include "DesktopDepthCapture.hpp"
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

std::string lowerStr(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return out;
}

double nowSteadySec() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

#ifdef _WIN32
std::string wstrToUtf8(const wchar_t* w) {
    if (!w) return "";
    int len = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
    if (len <= 1) return "";
    std::string out(static_cast<size_t>(len - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, w, -1, &out[0], len, nullptr, nullptr);
    return out;
}
#endif

}  // namespace

DesktopDepthCapture::~DesktopDepthCapture() {
    stop();
}

void DesktopDepthCapture::setDepthCallback(DepthCallback cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    on_depth_ = std::move(cb);
}

std::string DesktopDepthCapture::statusSummary() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_summary_;
}

bool DesktopDepthCapture::start() {
    if (running_.load()) return true;
    frames_delivered_.store(0);
    running_.store(true);
    worker_ = std::thread(&DesktopDepthCapture::captureLoop, this);
    return true;
}

void DesktopDepthCapture::stop() {
    if (!running_.exchange(false)) return;
    if (worker_.joinable()) worker_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    status_summary_ = "IDLE";
}

void DesktopDepthCapture::captureLoop() {
#ifdef _WIN32
    HRESULT hrCom = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    if (FAILED(MFStartup(MF_VERSION))) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_summary_ = "NO_DEPTH_DEVICE";
        running_.store(false);
        if (SUCCEEDED(hrCom)) CoUninitialize();
        return;
    }

    IMFAttributes* attrs = nullptr;
    IMFActivate** devices = nullptr;
    UINT32 count = 0;
    IMFMediaSource* source = nullptr;
    IMFSourceReader* reader = nullptr;
    std::string chosen_name;

    if (SUCCEEDED(MFCreateAttributes(&attrs, 1))) {
        attrs->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        if (SUCCEEDED(MFEnumDeviceSources(attrs, &devices, &count)) && count > 0) {
            int chosen = -1;
            int best_score = 0;
            for (UINT32 i = 0; i < count; ++i) {
                wchar_t* fname = nullptr;
                UINT32 flen = 0;
                std::string n;
                if (SUCCEEDED(devices[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME, &fname, &flen))) {
                    n = wstrToUtf8(fname);
                    CoTaskMemFree(fname);
                }
                std::string nl = lowerStr(n);
                int sc = 0;
                if (nl.find("depth") != std::string::npos) sc += 120;
                if (nl.find("orbbec") != std::string::npos || nl.find("gemini") != std::string::npos || nl.find("336") != std::string::npos) sc += 80;
                if (nl.find("rgb") != std::string::npos || nl.find("color") != std::string::npos || nl.find("surface") != std::string::npos) sc -= 200;
                if (sc > best_score) {
                    best_score = sc;
                    chosen = static_cast<int>(i);
                    chosen_name = n;
                }
            }
            if (chosen >= 0 && SUCCEEDED(devices[chosen]->ActivateObject(IID_PPV_ARGS(&source)))) {
                MFCreateSourceReaderFromMediaSource(source, nullptr, &reader);
            }
            for (UINT32 i = 0; i < count; ++i) devices[i]->Release();
            CoTaskMemFree(devices);
        }
        attrs->Release();
    }

    if (!reader) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_summary_ = "NO_DEPTH_CAM";
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
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_summary_ = "DEPTH [" + std::to_string(frameW) + "x" + std::to_string(frameH) + "]: " + chosen_name;
    }

    uint64_t seq = 0;
    while (running_.load()) {
        DWORD streamIdx = 0, flags = 0;
        LONGLONG llTs = 0;
        IMFSample* sample = nullptr;
        if (FAILED(reader->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, &streamIdx, &flags, &llTs, &sample)) || !sample) {
            std::this_thread::sleep_for(std::chrono::milliseconds(8));
            continue;
        }
        IMFMediaBuffer* buf = nullptr;
        if (SUCCEEDED(sample->ConvertToContiguousBuffer(&buf))) {
            BYTE* raw = nullptr;
            DWORD maxLen = 0, curLen = 0;
            if (SUCCEEDED(buf->Lock(&raw, &maxLen, &curLen)) && raw && curLen >= frameW * frameH * 2) {
                int scale = (frameW >= 640) ? 2 : 1;
                int dstW = static_cast<int>(frameW) / scale;
                int dstH = static_cast<int>(frameH) / scale;

                sensors::DepthFrame df;
                df.frame_id = ++seq;
                df.timestamp_sec = nowSteadySec();
                df.width = dstW;
                df.height = dstH;
                df.provider_type = sensors::DepthProviderType::STEREO_CAMERA;
                df.measurement_class = sensors::DepthMeasurementClass::MEASURED;
                df.units = sensors::DepthUnits::METERS;
                df.is_metric_anchored = true;
                df.intrinsics.width = dstW;
                df.intrinsics.height = dstH;
                df.intrinsics.fx = dstW * 0.78;
                df.intrinsics.fy = df.intrinsics.fx;
                df.intrinsics.cx = dstW * 0.5;
                df.intrinsics.cy = dstH * 0.5;
                df.depth_data.resize(static_cast<size_t>(dstW * dstH), 0.0f);
                df.confidence_data.resize(static_cast<size_t>(dstW * dstH), 0.0f);

                const uint16_t* depth16 = reinterpret_cast<const uint16_t*>(raw);
                for (int y = 0; y < dstH; ++y) {
                    const uint16_t* row = depth16 + static_cast<size_t>(y * scale) * frameW;
                    for (int x = 0; x < dstW; ++x) {
                        uint16_t mm = row[x * scale];
                        size_t idx = static_cast<size_t>(y * dstW + x);
                        if (mm >= 200 && mm <= 10000) {
                            df.depth_data[idx] = static_cast<float>(mm) * 0.001f;
                            df.confidence_data[idx] = 0.95f;
                        }
                    }
                }
                buf->Unlock();
                frames_delivered_.fetch_add(1);

                DepthCallback cb;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    cb = on_depth_;
                }
                if (cb) cb(df);
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
    std::string chosen_dev, chosen_name;
    for (int i = 0; i < 12; ++i) {
        std::string dev = "/dev/video" + std::to_string(i);
        int fd = ::open(dev.c_str(), O_RDWR | O_NONBLOCK);
        if (fd < 0) continue;
        v4l2_capability cap{};
        if (::ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0 && (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
            std::string card(reinterpret_cast<const char*>(cap.card));
            std::string cl = lowerStr(card);
            if (cl.find("depth") != std::string::npos ||
                ((cl.find("orbbec") != std::string::npos || cl.find("gemini") != std::string::npos) &&
                 cl.find("rgb") == std::string::npos && cl.find("color") == std::string::npos)) {
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
        status_summary_ = "NO_DEPTH_CAM";
        running_.store(false);
        return;
    }
    int fd = ::open(chosen_dev.c_str(), O_RDWR);
    if (fd < 0) { running_.store(false); return; }
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = 640;
    fmt.fmt.pix.height = 480;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    ::ioctl(fd, VIDIOC_S_FMT, &fmt);
    int frameW = static_cast<int>(fmt.fmt.pix.width);
    int frameH = static_cast<int>(fmt.fmt.pix.height);

    v4l2_requestbuffers req{};
    req.count = 4; req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; req.memory = V4L2_MEMORY_MMAP;
    if (::ioctl(fd, VIDIOC_REQBUFS, &req) < 0 || req.count == 0) { ::close(fd); running_.store(false); return; }
    struct MmapBuf { void* ptr{nullptr}; size_t len{0}; };
    std::vector<MmapBuf> bufs(req.count);
    for (unsigned i = 0; i < req.count; ++i) {
        v4l2_buffer b{}; b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP; b.index = i;
        ::ioctl(fd, VIDIOC_QUERYBUF, &b);
        bufs[i].len = b.length;
        bufs[i].ptr = ::mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        ::ioctl(fd, VIDIOC_QBUF, &b);
    }
    v4l2_buf_type bt = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ::ioctl(fd, VIDIOC_STREAMON, &bt);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_summary_ = "DEPTH (V4L2): " + chosen_name;
    }
    uint64_t seq = 0;
    while (running_.load()) {
        fd_set fds; FD_ZERO(&fds); FD_SET(fd, &fds); timeval tv{0, 100000};
        if (::select(fd + 1, &fds, nullptr, nullptr, &tv) <= 0) continue;
        v4l2_buffer b{}; b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP;
        if (::ioctl(fd, VIDIOC_DQBUF, &b) < 0) continue;
        const uint16_t* d16 = static_cast<const uint16_t*>(bufs[b.index].ptr);
        int dstW = frameW / 2, dstH = frameH / 2;
        sensors::DepthFrame df;
        df.frame_id = ++seq; df.timestamp_sec = nowSteadySec();
        df.width = dstW; df.height = dstH;
        df.provider_type = sensors::DepthProviderType::STEREO_CAMERA;
        df.measurement_class = sensors::DepthMeasurementClass::MEASURED;
        df.units = sensors::DepthUnits::METERS;
        df.is_metric_anchored = true;
        df.intrinsics = {dstW, dstH, dstW * 0.78, dstW * 0.78, dstW * 0.5, dstH * 0.5, {0,0,0,0,0}};
        df.depth_data.resize(static_cast<size_t>(dstW * dstH), 0.0f);
        df.confidence_data.resize(static_cast<size_t>(dstW * dstH), 0.0f);
        for (int y = 0; y < dstH; ++y) {
            for (int x = 0; x < dstW; ++x) {
                uint16_t mm = d16[(y * 2) * frameW + (x * 2)];
                size_t idx = static_cast<size_t>(y * dstW + x);
                if (mm >= 200 && mm <= 10000) {
                    df.depth_data[idx] = static_cast<float>(mm) * 0.001f;
                    df.confidence_data[idx] = 0.95f;
                }
            }
        }
        ::ioctl(fd, VIDIOC_QBUF, &b);
        frames_delivered_.fetch_add(1);
        DepthCallback cb;
        { std::lock_guard<std::mutex> lock(mutex_); cb = on_depth_; }
        if (cb) cb(df);
    }
    ::ioctl(fd, VIDIOC_STREAMOFF, &bt);
    for (auto& mb : bufs) if (mb.ptr) ::munmap(mb.ptr, mb.len);
    ::close(fd);
#endif
}

}  // namespace desktop
}  // namespace avscan
