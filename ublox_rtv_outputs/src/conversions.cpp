// Copyright 2026 Australian Robotics Supplies & Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ublox_rtv_outputs/conversions.hpp"

#include <cmath>
#include <ctime>
#include <limits>

#include "ublox_ubx_msgs/msg/carr_soln.hpp"
#include "ublox_ubx_msgs/msg/gps_fix.hpp"

namespace ublox_rtv_outputs
{
using gps_msgs::msg::GPSFix;
using gps_msgs::msg::GPSStatus;
using rtv_gnss_utils::msg::RtkStatus;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;
using ublox_ubx_msgs::msg::UBXNavPVT;

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

struct Position
{
  double lat{0}, lon{0}, alt{0}, h_acc{0}, v_acc{0};
  std::array<double, 9> cov{};
  uint8_t cov_type{NavSatFix::COVARIANCE_TYPE_UNKNOWN};
};

Position position(const Epoch & e)
{
  Position p;
  const UBXNavPVT & pvt = *e.pvt;
  const auto & hp = e.hpposllh;
  if (hp && !hp->invalid_lon && !hp->invalid_lat && !hp->invalid_height) {
    // standard part (1e-7 deg, mm) + high-precision part (1e-9 deg, 0.1 mm); accuracy in 0.1 mm
    p.lat = hp->lat * 1e-7 + hp->lat_hp * 1e-9;
    p.lon = hp->lon * 1e-7 + hp->lon_hp * 1e-9;
    p.alt = hp->height * 1e-3 + hp->height_hp * 1e-4;
    p.h_acc = hp->h_acc * 1e-4;
    p.v_acc = hp->v_acc * 1e-4;
  } else {
    p.lat = pvt.lat * 1e-7;
    p.lon = pvt.lon * 1e-7;
    p.alt = pvt.height * 1e-3;
    p.h_acc = pvt.h_acc * 1e-3;
    p.v_acc = pvt.v_acc * 1e-3;
  }
  if (e.cov && e.cov->pos_cor_valid) {
    const auto & c = *e.cov;
    p.cov = ned_to_enu(
      c.pos_cov_nn, c.pos_cov_ne, c.pos_cov_nd, c.pos_cov_ee, c.pos_cov_ed, c.pos_cov_dd);
    p.cov_type = NavSatFix::COVARIANCE_TYPE_KNOWN;
  } else {
    // hAcc is the receiver's horizontal accuracy estimate: used for both E and N (conservative).
    p.cov = {p.h_acc * p.h_acc, 0, 0, 0, p.h_acc * p.h_acc, 0, 0, 0, p.v_acc * p.v_acc};
    p.cov_type = NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
  }
  return p;
}

int8_t navsat_status(FixQuality q)
{
  switch (q) {
    case FixQuality::kNoFix: return NavSatStatus::STATUS_NO_FIX;
    case FixQuality::kSingle: return NavSatStatus::STATUS_FIX;
    default: return NavSatStatus::STATUS_GBAS_FIX;  // DGNSS and RTK: ground-based corrections
  }
}

uint8_t rtk_fix(FixQuality q)
{
  switch (q) {
    case FixQuality::kNoFix: return RtkStatus::FIX_NONE;
    case FixQuality::kSingle: return RtkStatus::FIX_SINGLE;
    case FixQuality::kDgnss: return RtkStatus::FIX_DGNSS;
    case FixQuality::kRtkFloat: return RtkStatus::FIX_RTK_FLOAT;
    case FixQuality::kRtkFixed: return RtkStatus::FIX_RTK_FIXED;
  }
  return RtkStatus::FIX_NONE;
}

int16_t gpsfix_status(FixQuality q)
{
  switch (q) {
    case FixQuality::kNoFix: return GPSStatus::STATUS_NO_FIX;
    case FixQuality::kSingle: return GPSStatus::STATUS_FIX;
    case FixQuality::kDgnss: return GPSStatus::STATUS_DGPS_FIX;
    case FixQuality::kRtkFloat: return GPSStatus::STATUS_RTK_FLOAT;
    case FixQuality::kRtkFixed: return GPSStatus::STATUS_RTK_FIX;
  }
  return GPSStatus::STATUS_NO_FIX;
}
}  // namespace

FixQuality fix_quality(const UBXNavPVT & pvt)
{
  using ublox_ubx_msgs::msg::CarrSoln;
  using ublox_ubx_msgs::msg::GpsFix;
  // Dead reckoning only and time only give no usable GNSS position.
  const uint8_t t = pvt.gps_fix.fix_type;
  if (!pvt.gnss_fix_ok || t < GpsFix::GPS_FIX_2D || t > GpsFix::GPS_PLUS_DEAD_RECKONING) {
    return FixQuality::kNoFix;
  }
  if (pvt.carr_soln.status == CarrSoln::CARRIER_SOLUTION_PHASE_WITH_FIXED_AMBIGUITIES) {
    return FixQuality::kRtkFixed;
  }
  if (pvt.carr_soln.status == CarrSoln::CARRIER_SOLUTION_PHASE_WITH_FLOATING_AMBIGUITIES) {
    return FixQuality::kRtkFloat;
  }
  return pvt.diff_soln ? FixQuality::kDgnss : FixQuality::kSingle;
}

const char * to_string(FixQuality q)
{
  switch (q) {
    case FixQuality::kNoFix: return "NO FIX";
    case FixQuality::kSingle: return "SINGLE";
    case FixQuality::kDgnss: return "DGNSS";
    case FixQuality::kRtkFloat: return "RTK FLOAT";
    case FixQuality::kRtkFixed: return "RTK FIXED";
  }
  return "?";
}

double correction_age_upper_s(uint8_t bucket)
{
  static const double kUpper[] = {0, 1, 2, 5, 10, 15, 20, 30, 45, 60, 90, 120};
  if (bucket < sizeof(kUpper) / sizeof(kUpper[0])) {
    return kUpper[bucket];
  }
  return std::numeric_limits<double>::infinity();
}

std::optional<int64_t> utc_ns(const UBXNavPVT & pvt)
{
  if (!pvt.valid_date || !pvt.valid_time || !pvt.fully_resolved ||
    (pvt.confirmed_avail && (!pvt.confirmed_date || !pvt.confirmed_time)))
  {
    return std::nullopt;
  }
  std::tm t{};
  t.tm_year = pvt.year - 1900;
  t.tm_mon = pvt.month - 1;
  t.tm_mday = pvt.day;
  t.tm_hour = pvt.hour;
  t.tm_min = pvt.min;
  t.tm_sec = pvt.sec;
  const time_t s = ::timegm(&t);
  if (s == static_cast<time_t>(-1)) {
    return std::nullopt;
  }
  return static_cast<int64_t>(s) * 1000000000LL + pvt.nano;
}

std::array<double, 9> ned_to_enu(double nn, double ne, double nd, double ee, double ed, double dd)
{
  // x = E, y = N, z = U = -D
  return {
    ee, ne, -ed,
    ne, nn, -nd,
    -ed, -nd, dd,
  };
}

builtin_interfaces::msg::Time to_time_msg(int64_t ns)
{
  builtin_interfaces::msg::Time t;
  int64_t s = ns / 1000000000LL;
  int64_t r = ns % 1000000000LL;
  if (r < 0) {
    r += 1000000000LL;
    --s;
  }
  t.sec = static_cast<int32_t>(s);
  t.nanosec = static_cast<uint32_t>(r);
  return t;
}

uint16_t service_mask(const ublox_ubx_msgs::msg::UBXNavSat & sat)
{
  uint16_t m = 0;
  for (const auto & s : sat.sv_info) {
    if (!s.flags.sv_used) {
      continue;
    }
    switch (s.gnss_id) {
      case 0: m |= NavSatStatus::SERVICE_GPS; break;
      case 2: m |= NavSatStatus::SERVICE_GALILEO; break;
      case 3: m |= NavSatStatus::SERVICE_COMPASS; break;
      case 6: m |= NavSatStatus::SERVICE_GLONASS; break;
      default: break;  // SBAS, QZSS: no bit
    }
  }
  return m;
}

int32_t satellite_number(uint8_t gnss_id, uint8_t sv_id)
{
  if (gnss_id == 1) {
    return sv_id;  // SBAS PRN 120..158: gnssId * 100 would collide with Galileo 2xx
  }
  return static_cast<int32_t>(gnss_id) * 100 + sv_id;
}

std::optional<NavSatFix> to_navsatfix(const Epoch & e, const OutputContext & ctx)
{
  if (!e.pvt) {
    return std::nullopt;
  }
  const Position p = position(e);
  NavSatFix m;
  m.header.stamp = ctx.stamp;
  m.header.frame_id = ctx.frame_id;
  m.status.status = navsat_status(fix_quality(*e.pvt));
  m.status.service = ctx.sat ? service_mask(*ctx.sat) : 0;
  m.latitude = p.lat;
  m.longitude = p.lon;
  m.altitude = p.alt;  // ellipsoid height, as NavSatFix defines
  m.position_covariance = p.cov;
  m.position_covariance_type = p.cov_type;
  return m;
}

std::optional<GPSFix> to_gpsfix(const Epoch & e, const OutputContext & ctx)
{
  if (!e.pvt) {
    return std::nullopt;
  }
  const UBXNavPVT & pvt = *e.pvt;
  const Position p = position(e);
  GPSFix m;
  m.header.stamp = ctx.stamp;
  m.header.frame_id = ctx.frame_id;
  m.status.header = m.header;
  m.status.status = gpsfix_status(fix_quality(pvt));
  m.status.satellites_used = pvt.num_sv;
  m.status.position_source = GPSStatus::SOURCE_GPS;
  m.status.motion_source = GPSStatus::SOURCE_DOPPLER;
  m.status.orientation_source = GPSStatus::SOURCE_NONE;  // single antenna: no attitude
  if (ctx.sat) {
    m.status.satellites_visible = static_cast<uint16_t>(ctx.sat->sv_info.size());
    for (const auto & s : ctx.sat->sv_info) {
      const int32_t n = satellite_number(s.gnss_id, s.sv_id);
      if (s.flags.sv_used) {
        m.status.satellite_used_prn.push_back(n);
      }
      m.status.satellite_visible_prn.push_back(n);
      m.status.satellite_visible_z.push_back(s.elev);
      m.status.satellite_visible_azimuth.push_back(s.azim);
      m.status.satellite_visible_snr.push_back(s.cno);
    }
  }
  m.latitude = p.lat;
  m.longitude = p.lon;
  m.altitude = p.alt;
  m.track = pvt.head_mot * 1e-5;  // course over ground; meaningful only while moving
  m.speed = pvt.g_speed * 1e-3;
  m.climb = -pvt.vel_d * 1e-3;
  if (const auto utc = utc_ns(pvt)) {
    m.time = static_cast<double>(*utc) * 1e-9;
  }
  if (e.dop) {
    m.gdop = e.dop->g_dop * 0.01;
    m.pdop = e.dop->p_dop * 0.01;
    m.hdop = e.dop->h_dop * 0.01;
    m.vdop = e.dop->v_dop * 0.01;
    m.tdop = e.dop->t_dop * 0.01;
  } else {
    m.pdop = pvt.p_dop * 0.01;
  }
  // u-blox accuracy estimates (about 1 sigma), not the 95 % gpsd uses for err_*.
  const double s_acc = pvt.s_acc * 1e-3;
  m.err_horz = p.h_acc;
  m.err_vert = p.v_acc;
  m.err = std::hypot(p.h_acc, p.v_acc);
  m.err_speed = s_acc;
  m.err_track = pvt.head_acc * 1e-5;
  m.err_climb = e.cov && e.cov->vel_cor_valid ? std::sqrt(e.cov->vel_cov_dd) : s_acc;
  m.err_time = pvt.t_acc * 1e-9;
  m.position_covariance = p.cov;
  m.position_covariance_type = p.cov_type;
  return m;
}

std::optional<geometry_msgs::msg::TwistWithCovarianceStamped> to_twist(
  const Epoch & e, const OutputContext & ctx)
{
  if (!e.pvt) {
    return std::nullopt;
  }
  const UBXNavPVT & pvt = *e.pvt;
  geometry_msgs::msg::TwistWithCovarianceStamped m;
  m.header.stamp = ctx.stamp;
  m.header.frame_id = ctx.velocity_frame_id;
  // Doppler velocity over ground, ENU axes
  m.twist.twist.linear.x = pvt.vel_e * 1e-3;
  m.twist.twist.linear.y = pvt.vel_n * 1e-3;
  m.twist.twist.linear.z = -pvt.vel_d * 1e-3;
  auto & c = m.twist.covariance;  // 6x6 row-major
  if (e.cov && e.cov->vel_cor_valid) {
    const auto & v = *e.cov;
    const auto enu = ned_to_enu(
      v.vel_cov_nn, v.vel_cov_ne, v.vel_cov_nd, v.vel_cov_ee, v.vel_cov_ed, v.vel_cov_dd);
    for (size_t r = 0; r < 3; ++r) {
      for (size_t k = 0; k < 3; ++k) {
        c[r * 6 + k] = enu[r * 3 + k];
      }
    }
  } else {
    const double s_acc = pvt.s_acc * 1e-3;
    c[0] = c[7] = c[14] = s_acc * s_acc;
  }
  c[21] = c[28] = c[35] = -1.0;  // angular velocity: not measured
  return m;
}

std::optional<RtkStatus> to_rtk_status(
  const Epoch & e, const OutputContext & ctx, const CorrectionInfo & corrections)
{
  if (!e.pvt) {
    return std::nullopt;
  }
  const UBXNavPVT & pvt = *e.pvt;
  const Position p = position(e);
  const FixQuality q = fix_quality(pvt);
  RtkStatus m;
  m.header.stamp = ctx.stamp;
  m.header.frame_id = ctx.frame_id;
  m.fix = rtk_fix(q);
  m.fix_text = to_string(q);
  m.time_of_week_ms = pvt.itow;
  m.satellites_used = pvt.num_sv;
  m.horizontal_accuracy = p.h_acc;
  m.vertical_accuracy = p.v_acc;
  m.position_covariance = p.cov;
  m.position_covariance_type = p.cov_type;
  m.pdop = static_cast<float>(e.dop ? e.dop->p_dop * 0.01 : pvt.p_dop * 0.01);
  m.hdop = static_cast<float>(e.dop ? e.dop->h_dop * 0.01 : kNaN);
  m.vdop = static_cast<float>(e.dop ? e.dop->v_dop * 0.01 : kNaN);
  m.correction_age_s = pvt.last_correction_age == 0 ?
    kNaN : correction_age_upper_s(pvt.last_correction_age);
  m.reference_station = corrections.reference_station;
  m.rtcm_messages_used = corrections.used;
  m.rtcm_messages_not_used = corrections.not_used;
  m.rtcm_crc_failures = corrections.crc_failed;
  m.seconds_since_last_rtcm = corrections.seconds_since_last_rtcm;
  return m;
}

std::optional<sensor_msgs::msg::TimeReference> to_time_reference(
  const Epoch & e, const OutputContext & ctx)
{
  if (!e.pvt) {
    return std::nullopt;
  }
  const auto utc = utc_ns(*e.pvt);
  if (!utc) {
    return std::nullopt;
  }
  sensor_msgs::msg::TimeReference m;
  m.header.stamp = ctx.stamp;
  m.header.frame_id = ctx.frame_id;
  m.time_ref = to_time_msg(*utc);
  m.source = ctx.time_source;
  return m;
}

}  // namespace ublox_rtv_outputs
