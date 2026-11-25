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

// One navigation epoch of ublox_dgnss messages -> standard ROS messages and RtkStatus.
// Position from NAV-HPPOSLLH (falls back to NAV-PVT), covariance from NAV-COV rotated
// NED -> ENU (falls back to hAcc/vAcc), velocity from NAV-PVT.

#ifndef UBLOX_RTV_OUTPUTS__CONVERSIONS_HPP_
#define UBLOX_RTV_OUTPUTS__CONVERSIONS_HPP_

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "rtv_gnss_utils/msg/rtk_status.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_cov.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_dop.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_hp_pos_llh.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_pvt.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_sat.hpp"

namespace ublox_rtv_outputs
{

/// The NAV messages of one navigation epoch (same iTOW).
struct Epoch
{
  uint32_t itow{0};
  builtin_interfaces::msg::Time rx_stamp;  // header.stamp of its first message (host time)
  bool closed_by_eoe{false};
  std::optional<ublox_ubx_msgs::msg::UBXNavPVT> pvt;
  std::optional<ublox_ubx_msgs::msg::UBXNavHPPosLLH> hpposllh;
  std::optional<ublox_ubx_msgs::msg::UBXNavCov> cov;
  std::optional<ublox_ubx_msgs::msg::UBXNavDOP> dop;
};

enum class FixQuality { kNoFix, kSingle, kDgnss, kRtkFloat, kRtkFixed };
FixQuality fix_quality(const ublox_ubx_msgs::msg::UBXNavPVT & pvt);
const char * to_string(FixQuality q);

/// Upper bound of the NAV-PVT lastCorrectionAge bucket in seconds; 0 = no corrections,
/// infinity for the last bucket (>= 120 s).
double correction_age_upper_s(uint8_t bucket);

/// UTC of the epoch as nanoseconds since the Unix epoch. nullopt unless date and time are valid,
/// fully resolved and, if the receiver reports it, confirmed.
std::optional<int64_t> utc_ns(const ublox_ubx_msgs::msg::UBXNavPVT & pvt);

/// Row-major 3x3 ENU covariance from the NED upper triangle (x = E, y = N, z = -D).
std::array<double, 9> ned_to_enu(double nn, double ne, double nd, double ee, double ed, double dd);

struct OutputContext
{
  builtin_interfaces::msg::Time stamp;
  std::string frame_id;           // antenna frame, for position messages
  std::string velocity_frame_id;  // frame whose axes are ENU, for the twist
  std::string time_source;        // TimeReference.source
  const ublox_ubx_msgs::msg::UBXNavSat * sat{nullptr};  // latest NAV-SAT (1 Hz), may be null
};

/// Correction counters the node keeps from RXM-RTCM.
struct CorrectionInfo
{
  int32_t reference_station{-1};
  uint32_t used{0}, not_used{0}, crc_failed{0};
  double seconds_since_last_rtcm{std::numeric_limits<double>::quiet_NaN()};
};

/// NavSatStatus.service bits of the constellations used in the solution.
uint16_t service_mask(const ublox_ubx_msgs::msg::UBXNavSat & sat);

/// Satellite number in GPSFix: gnssId * 100 + svId (GPS 1..32, Galileo 201.., BeiDou 301..,
/// QZSS 501.., GLONASS 601..); SBAS keeps its own PRN (120..158).
int32_t satellite_number(uint8_t gnss_id, uint8_t sv_id);

/// All return nullopt if the epoch has no NAV-PVT.
std::optional<sensor_msgs::msg::NavSatFix> to_navsatfix(
  const Epoch & e, const OutputContext & ctx);
std::optional<gps_msgs::msg::GPSFix> to_gpsfix(const Epoch & e, const OutputContext & ctx);
std::optional<geometry_msgs::msg::TwistWithCovarianceStamped> to_twist(
  const Epoch & e, const OutputContext & ctx);
std::optional<rtv_gnss_utils::msg::RtkStatus> to_rtk_status(
  const Epoch & e, const OutputContext & ctx, const CorrectionInfo & corrections);
/// Also nullopt while the receiver does not know UTC.
std::optional<sensor_msgs::msg::TimeReference> to_time_reference(
  const Epoch & e, const OutputContext & ctx);

builtin_interfaces::msg::Time to_time_msg(int64_t ns);

}  // namespace ublox_rtv_outputs

#endif  // UBLOX_RTV_OUTPUTS__CONVERSIONS_HPP_
