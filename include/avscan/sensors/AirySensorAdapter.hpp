#pragma once

#include "avscan/sensors/AiryUdpReceiver.hpp"
#include "avscan/sensors/SensorRigProfile.hpp"
#include <memory>

namespace avscan {
namespace sensors {

// Zero-overhead ISensorStreamAdapter wrapper around AiryUdpReceiver so RoboSense Airy,
// Livox Mid-360, and Ouster OS0/OS1 plug interchangeably into AV-SLAM.
class AirySensorAdapter final : public ISensorStreamAdapter {
public:
    explicit AirySensorAdapter(const AiryNetworkConfig& cfg = AiryNetworkConfig{})
        : receiver_(std::make_unique<AiryUdpReceiver>(cfg)),
          rig_(GenericSensorRigConfig::presetRoboSenseAiry()) {}

    std::string adapterName() const override { return "RoboSense Airy Native UDP Adapter"; }
    GenericSensorRigConfig rigConfig() const override { return rig_; }

    void setCallbacks(FrameCallback on_frame, ImuCallback on_imu, RigConfigCallback on_rig_update) override {
        receiver_->setCallbacks(
            std::move(on_frame),
            std::move(on_imu),
            [this, cb = std::move(on_rig_update)](const core::AiryFactoryCalibration& cal) {
                rig_ = GenericSensorRigConfig::fromAiryFactoryCalibration(cal);
                if (cb) cb(rig_);
            });
    }

    bool start() override { return receiver_->start(); }
    void stop() override { receiver_->stop(); }
    bool isRunning() const override { return receiver_->isRunning(); }

    AiryUdpReceiver& rawReceiver() { return *receiver_; }

private:
    std::unique_ptr<AiryUdpReceiver> receiver_;
    GenericSensorRigConfig rig_;
};

}  // namespace sensors
}  // namespace avscan
