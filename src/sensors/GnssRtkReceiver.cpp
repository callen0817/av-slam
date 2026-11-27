#include "avscan/sensors/GnssRtkReceiver.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

namespace avscan {
namespace sensors {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84E2 = 6.69437999014e-3;

double nmeaCoordToDeg(const std::string& raw, const std::string& hemi) {
    if (raw.empty()) return 0.0;
    double val = std::stod(raw);
    int deg = static_cast<int>(std::floor(val / 100.0));
    double mins = val - deg * 100.0;
    double out = static_cast<double>(deg) + mins / 60.0;
    if (hemi == "S" || hemi == "W") out = -out;
    return out;
}

}  // namespace

void GnssRtkProcessor::reset() {
    has_origin_ = false;
    origin_lat_rad_ = 0.0;
    origin_lon_rad_ = 0.0;
    origin_alt_m_ = 0.0;
}

void GnssRtkProcessor::sigmasForFixQuality(GnssFixQuality q, double hdop,
                                           double& out_horiz_sigma_m, double& out_vert_sigma_m) {
    double hd = std::clamp(hdop, 0.5, 10.0);
    switch (q) {
        case GnssFixQuality::RTK_FIXED:
            out_horiz_sigma_m = 0.02 * hd;
            out_vert_sigma_m  = 0.04 * hd;
            break;
        case GnssFixQuality::RTK_FLOAT:
            out_horiz_sigma_m = 0.30 * hd;
            out_vert_sigma_m  = 0.50 * hd;
            break;
        case GnssFixQuality::DGPS:
            out_horiz_sigma_m = 1.00 * hd;
            out_vert_sigma_m  = 1.80 * hd;
            break;
        case GnssFixQuality::GPS_SINGLE:
            out_horiz_sigma_m = 2.50 * hd;
            out_vert_sigma_m  = 4.50 * hd;
            break;
        default:
            out_horiz_sigma_m = 50.0;
            out_vert_sigma_m  = 100.0;
            break;
    }
}

bool GnssRtkProcessor::projectToLocalEnu(GnssRtkMeasurement& io_fix) {
    if (io_fix.fix_quality == GnssFixQuality::INVALID) {
        io_fix.valid = false;
        return false;
    }

    const double lat_rad = io_fix.latitude_deg * kDegToRad;
    const double lon_rad = io_fix.longitude_deg * kDegToRad;
    if (!has_origin_) {
        origin_lat_rad_ = lat_rad;
        origin_lon_rad_ = lon_rad;
        origin_alt_m_ = io_fix.altitude_m;
        has_origin_ = true;
    }

    const double sin_lat = std::sin(origin_lat_rad_);
    const double cos_lat = std::cos(origin_lat_rad_);
    const double N = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
    const double M = kWgs84A * (1.0 - kWgs84E2) / std::pow(1.0 - kWgs84E2 * sin_lat * sin_lat, 1.5);

    const double dlat = lat_rad - origin_lat_rad_;
    const double dlon = lon_rad - origin_lon_rad_;

    io_fix.enu_position_m.x = (N + origin_alt_m_) * cos_lat * dlon;  // East
    io_fix.enu_position_m.y = (M + origin_alt_m_) * dlat;            // North
    io_fix.enu_position_m.z = io_fix.altitude_m - origin_alt_m_;     // Up
    sigmasForFixQuality(io_fix.fix_quality, io_fix.hdop,
                        io_fix.horizontal_sigma_m, io_fix.vertical_sigma_m);
    io_fix.valid = true;
    return true;
}

bool GnssRtkProcessor::parseNmeaGga(const std::string& sentence,
                                    double timestamp_sec,
                                    GnssRtkMeasurement& out_fix) {
    out_fix = GnssRtkMeasurement{};
    if (sentence.size() < 15) return false;
    if (sentence.find("GGA") == std::string::npos) return false;

    std::string body = sentence;
    auto star = body.find('*');
    if (star != std::string::npos) body = body.substr(0, star);

    std::vector<std::string> fields;
    std::stringstream ss(body);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        fields.push_back(tok);
    }
    if (fields.size() < 10) return false;
    if (fields[2].empty() || fields[4].empty() || fields[6].empty()) return false;

    int qual_int = std::stoi(fields[6]);
    if (qual_int <= 0) return false;

    out_fix.timestamp_sec = timestamp_sec;
    out_fix.latitude_deg = nmeaCoordToDeg(fields[2], fields[3]);
    out_fix.longitude_deg = nmeaCoordToDeg(fields[4], fields[5]);
    out_fix.fix_quality = static_cast<GnssFixQuality>(qual_int);
    out_fix.satellites_used = fields[7].empty() ? 0 : std::stoi(fields[7]);
    out_fix.hdop = fields[8].empty() ? 1.5 : std::stod(fields[8]);
    out_fix.altitude_m = fields[9].empty() ? 0.0 : std::stod(fields[9]);
    return projectToLocalEnu(out_fix);
}

}  // namespace sensors
}  // namespace avscan
