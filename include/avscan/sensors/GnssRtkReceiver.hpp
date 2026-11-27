#pragma once

#include "avscan/math/SophusLie.hpp"
#include <cstdint>
#include <string>

namespace avscan {
namespace sensors {

enum class GnssFixQuality : uint8_t {
    INVALID = 0,
    GPS_SINGLE = 1,    // ~2.5 m sigma
    DGPS = 2,          // ~1.0 m sigma
    RTK_FIXED = 4,     // ~0.02 m sigma (cm-grade)
    RTK_FLOAT = 5      // ~0.30 m sigma
};

struct GnssRtkMeasurement {
    double timestamp_sec{0.0};
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_m{0.0};
    GnssFixQuality fix_quality{GnssFixQuality::INVALID};
    int satellites_used{0};
    double hdop{99.0};
    // Local ENU coordinates relative to first valid fix origin
    math::Vec3d enu_position_m{0.0, 0.0, 0.0};
    double horizontal_sigma_m{10.0};
    double vertical_sigma_m{20.0};
    bool valid{false};
};

// NMEA-0183 ($GPGGA / $GNGGA) parser & WGS-84 LLA -> Local ENU tangent plane projector
class GnssRtkProcessor {
public:
    GnssRtkProcessor() = default;

    void reset();
    bool hasOrigin() const { return has_origin_; }

    // Parse an NMEA GGA sentence (e.g. from u-blox ZED-F9P, Emlid Reach, Bluetooth RTK)
    bool parseNmeaGga(const std::string& sentence, double timestamp_sec, GnssRtkMeasurement& out_fix);

    // Project a raw WGS-84 fix into the local ENU frame (sets origin on first valid fix)
    bool projectToLocalEnu(GnssRtkMeasurement& io_fix);

    static void sigmasForFixQuality(GnssFixQuality q, double hdop,
                                    double& out_horiz_sigma_m, double& out_vert_sigma_m);

private:
    bool has_origin_{false};
    double origin_lat_rad_{0.0};
    double origin_lon_rad_{0.0};
    double origin_alt_m_{0.0};
};

}  // namespace sensors
}  // namespace avscan
