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

#include <gtest/gtest.h>

#include <cmath>

#include "ublox_rtv_outputs/conversions.hpp"
#include "ublox_ubx_msgs/msg/carr_soln.hpp"
#include "ublox_ubx_msgs/msg/gps_fix.hpp"

using ublox_rtv_outputs::Epoch;
using ublox_rtv_outputs::FixQuality;
using ublox_rtv_outputs::OutputContext;
using ublox_ubx_msgs::msg::CarrSoln;
using ublox_ubx_msgs::msg::GpsFix;
using ublox_ubx_msgs::msg::UBXNavPVT;

namespace
{
UBXNavPVT pvt_3d()
{
  UBXNavPVT p;
  p.itow = 1000;
  p.gps_fix.fix_type = GpsFix::GPS_FIX_3D;
  p.gnss_fix_ok = true;
  p.num_sv = 20;
  p.lat = 481234567;   // 48.1234567 deg
  p.lon = 171234567;
  p.height = 150000;   // 150 m
  p.h_acc = 20;        // 20 mm
  p.v_acc = 30;
  p.vel_n = 1000;      // 1 m/s north
  p.vel_e = 2000;      // 2 m/s east
  p.vel_d = -500;      // 0.5 m/s up
  p.s_acc = 100;       // 0.1 m/s
  p.p_dop = 150;
  return p;
}
}  // namespace

TEST(Conversions, FixQuality)
{
  UBXNavPVT p = pvt_3d();
  EXPECT_EQ(ublox_rtv_outputs::fix_quality(p), FixQuality::kSingle);
  p.diff_soln = true;
  EXPECT_EQ(ublox_rtv_outputs::fix_quality(p), FixQuality::kDgnss);
  p.carr_soln.status = CarrSoln::CARRIER_SOLUTION_PHASE_WITH_FLOATING_AMBIGUITIES;
  EXPECT_EQ(ublox_rtv_outputs::fix_quality(p), FixQuality::kRtkFloat);
  p.carr_soln.status = CarrSoln::CARRIER_SOLUTION_PHASE_WITH_FIXED_AMBIGUITIES;
  EXPECT_EQ(ublox_rtv_outputs::fix_quality(p), FixQuality::kRtkFixed);
  p.gnss_fix_ok = false;
  EXPECT_EQ(ublox_rtv_outputs::fix_quality(p), FixQuality::kNoFix);
  p.gnss_fix_ok = true;
  p.gps_fix.fix_type = GpsFix::GPS_TIME_ONLY;
  EXPECT_EQ(ublox_rtv_outputs::fix_quality(p), FixQuality::kNoFix);
}

TEST(Conversions, NavSatFixPrefersHighPrecisionAndCov)
{
  Epoch e;
  e.pvt = pvt_3d();
  e.pvt->carr_soln.status = CarrSoln::CARRIER_SOLUTION_PHASE_WITH_FIXED_AMBIGUITIES;
  OutputContext ctx;
  ctx.frame_id = "gnss";

  auto m = ublox_rtv_outputs::to_navsatfix(e, ctx);
  ASSERT_TRUE(m);
  EXPECT_EQ(m->header.frame_id, "gnss");
  EXPECT_EQ(m->status.status, sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX);
  EXPECT_NEAR(m->latitude, 48.1234567, 1e-9);
  EXPECT_NEAR(m->altitude, 150.0, 1e-9);
  // no NAV-COV: hAcc / vAcc
  EXPECT_EQ(
    m->position_covariance_type, sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED);
  EXPECT_NEAR(m->position_covariance[0], 0.02 * 0.02, 1e-12);
  EXPECT_NEAR(m->position_covariance[8], 0.03 * 0.03, 1e-12);

  ublox_ubx_msgs::msg::UBXNavHPPosLLH hp;
  hp.lat = 481234567;
  hp.lat_hp = 89;      // +0.89e-7 deg
  hp.lon = 171234567;
  hp.height = 150000;
  hp.height_hp = 5;    // +0.5 mm
  hp.h_acc = 141;      // 14.1 mm
  hp.v_acc = 200;
  e.hpposllh = hp;
  ublox_ubx_msgs::msg::UBXNavCov cov;
  cov.pos_cor_valid = true;
  cov.pos_cov_nn = 4.0f;
  cov.pos_cov_ee = 1.0f;
  cov.pos_cov_dd = 9.0f;
  cov.pos_cov_ne = 0.5f;
  cov.pos_cov_ed = 0.25f;
  e.cov = cov;

  m = ublox_rtv_outputs::to_navsatfix(e, ctx);
  EXPECT_NEAR(m->latitude, 48.123456789, 1e-12);
  EXPECT_NEAR(m->altitude, 150.0005, 1e-9);
  EXPECT_EQ(m->position_covariance_type, sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN);
  // ENU: [ee ne -ed; ne nn -nd; -ed -nd dd]
  EXPECT_DOUBLE_EQ(m->position_covariance[0], 1.0);
  EXPECT_DOUBLE_EQ(m->position_covariance[1], 0.5);
  EXPECT_DOUBLE_EQ(m->position_covariance[2], -0.25);
  EXPECT_DOUBLE_EQ(m->position_covariance[4], 4.0);
  EXPECT_DOUBLE_EQ(m->position_covariance[8], 9.0);

  auto r = ublox_rtv_outputs::to_rtk_status(e, ctx, {});
  ASSERT_TRUE(r);
  EXPECT_EQ(r->fix, rtv_gnss_utils::msg::RtkStatus::FIX_RTK_FIXED);
  EXPECT_EQ(r->fix_text, "RTK FIXED");
  EXPECT_NEAR(r->horizontal_accuracy, 0.0141, 1e-12);
  EXPECT_TRUE(std::isnan(r->correction_age_s));
}

TEST(Conversions, TwistIsEnuWithoutAngular)
{
  Epoch e;
  e.pvt = pvt_3d();
  OutputContext ctx;
  ctx.velocity_frame_id = "enu";
  auto t = ublox_rtv_outputs::to_twist(e, ctx);
  ASSERT_TRUE(t);
  EXPECT_EQ(t->header.frame_id, "enu");
  EXPECT_DOUBLE_EQ(t->twist.twist.linear.x, 2.0);
  EXPECT_DOUBLE_EQ(t->twist.twist.linear.y, 1.0);
  EXPECT_DOUBLE_EQ(t->twist.twist.linear.z, 0.5);
  EXPECT_NEAR(t->twist.covariance[0], 0.01, 1e-12);
  EXPECT_NEAR(t->twist.covariance[14], 0.01, 1e-12);
  EXPECT_EQ(t->twist.covariance[21], -1.0);
  EXPECT_EQ(t->twist.covariance[35], -1.0);
}

TEST(Conversions, CorrectionAgeAndUtc)
{
  UBXNavPVT p = pvt_3d();
  p.last_correction_age = 3;  // 2-5 s
  Epoch e;
  e.pvt = p;
  const OutputContext ctx;
  auto r = ublox_rtv_outputs::to_rtk_status(e, ctx, {});
  EXPECT_DOUBLE_EQ(r->correction_age_s, 5.0);
  EXPECT_TRUE(std::isinf(ublox_rtv_outputs::correction_age_upper_s(12)));

  EXPECT_FALSE(ublox_rtv_outputs::utc_ns(p));
  p.year = 2026;
  p.month = 1;
  p.day = 2;
  p.hour = 3;
  p.min = 4;
  p.sec = 5;
  p.nano = -100;
  p.valid_date = p.valid_time = p.fully_resolved = true;
  const auto utc = ublox_rtv_outputs::utc_ns(p);
  ASSERT_TRUE(utc);
  EXPECT_EQ(*utc, 1767323045LL * 1000000000LL - 100);
  p.confirmed_avail = true;  // available but not confirmed
  EXPECT_FALSE(ublox_rtv_outputs::utc_ns(p));
}

TEST(Conversions, SatelliteNumbers)
{
  EXPECT_EQ(ublox_rtv_outputs::satellite_number(0, 5), 5);
  EXPECT_EQ(ublox_rtv_outputs::satellite_number(2, 11), 211);
  EXPECT_EQ(ublox_rtv_outputs::satellite_number(1, 123), 123);
  EXPECT_EQ(ublox_rtv_outputs::satellite_number(6, 3), 603);
}
