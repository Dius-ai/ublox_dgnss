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

#include "ublox_rtv_outputs/epoch_assembler.hpp"

using ublox_rtv_outputs::EpochAssembler;
using ublox_ubx_msgs::msg::UBXNavCov;
using ublox_ubx_msgs::msg::UBXNavPVT;

namespace
{
constexpr int64_t kMs = 1000000;

UBXNavPVT pvt(uint32_t itow)
{
  UBXNavPVT m;
  m.itow = itow;
  m.header.stamp.sec = static_cast<int32_t>(itow / 1000);
  return m;
}

UBXNavCov cov(uint32_t itow)
{
  UBXNavCov m;
  m.itow = itow;
  return m;
}

builtin_interfaces::msg::Time stamp() {return builtin_interfaces::msg::Time{};}
}  // namespace

TEST(EpochAssembler, EoeClosesEpoch)
{
  EpochAssembler a;
  EXPECT_TRUE(a.add(pvt(200), 0).empty());
  EXPECT_TRUE(a.add(cov(200), 0).empty());
  const auto out = a.add_eoe(200, stamp(), 1 * kMs);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].itow, 200u);
  EXPECT_TRUE(out[0].closed_by_eoe);
  EXPECT_TRUE(out[0].pvt && out[0].cov);
  EXPECT_EQ(out[0].rx_stamp.sec, 0);  // stamp of the first message
}

TEST(EpochAssembler, EoeBeforeExpectedMessageWaits)
{
  EpochAssembler a;
  a.add(pvt(200), 0);
  a.add(cov(200), 0);
  ASSERT_EQ(a.add_eoe(200, stamp(), 0).size(), 1u);
  // next epoch: EOE overtakes NAV-COV on another topic
  a.add(pvt(400), 0);
  EXPECT_TRUE(a.add_eoe(400, stamp(), 1 * kMs).empty());
  const auto out = a.add(cov(400), 2 * kMs);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_TRUE(out[0].cov);
}

TEST(EpochAssembler, MissingMessageTimesOut)
{
  EpochAssembler a(100 * kMs);
  a.add(pvt(200), 0);
  a.add(cov(200), 0);
  a.add_eoe(200, stamp(), 0);
  a.add(pvt(400), 0);
  EXPECT_TRUE(a.add_eoe(400, stamp(), 10 * kMs).empty());  // waits for NAV-COV
  EXPECT_TRUE(a.flush(50 * kMs).empty());
  const auto out = a.flush(110 * kMs);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FALSE(out[0].cov);
  // NAV-COV no longer expected: the next epoch closes on its EOE
  a.add(pvt(600), 120 * kMs);
  EXPECT_EQ(a.add_eoe(600, stamp(), 120 * kMs).size(), 1u);
  // a late NAV-COV of a published epoch is counted and ignored
  EXPECT_TRUE(a.add(cov(400), 130 * kMs).empty());
  EXPECT_EQ(a.stats().late_messages, 1u);
}

TEST(EpochAssembler, LostEoeClosedByNextEpoch)
{
  EpochAssembler a;
  a.add(pvt(200), 0);
  const auto out = a.add(pvt(400), 0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].itow, 200u);
  EXPECT_FALSE(out[0].closed_by_eoe);
  EXPECT_EQ(a.stats().epochs_without_eoe, 1u);
}

TEST(EpochAssembler, EpochWithoutPvtDropped)
{
  EpochAssembler a;
  EXPECT_TRUE(a.add(cov(200), 0).empty());
  EXPECT_TRUE(a.add_eoe(200, stamp(), 0).empty());
  EXPECT_TRUE(a.flush(200 * kMs).empty());
  EXPECT_EQ(a.stats().epochs_without_pvt, 1u);
  EXPECT_EQ(a.stats().epochs, 0u);
}

TEST(EpochAssembler, WeekRollover)
{
  EpochAssembler a;
  const uint32_t last = 604800000u - 200u;
  a.add(pvt(last), 0);
  ASSERT_EQ(a.add_eoe(last, stamp(), 0).size(), 1u);
  a.add(pvt(0), 0);
  const auto out = a.add_eoe(0, stamp(), 0);
  ASSERT_EQ(out.size(), 1u);  // not taken for a late message of an old epoch
  EXPECT_EQ(out[0].itow, 0u);
  EXPECT_EQ(a.stats().late_messages, 0u);
}

TEST(EpochAssembler, TimeJumpBackRestarts)
{
  EpochAssembler a;
  // pre-fix receiver time 2 days ahead of the real GNSS time
  const uint32_t bogus = 400000000u;
  a.add(pvt(bogus), 0);
  ASSERT_EQ(a.add_eoe(bogus, stamp(), 0).size(), 1u);
  a.add(pvt(bogus + 200), 0);
  // time fix: iTOW drops back 2 days, epochs continue from there
  const uint32_t real = bogus - 172800000u;
  EXPECT_TRUE(a.add(pvt(real), 0).empty());
  const auto out = a.add_eoe(real, stamp(), 0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].itow, real);
  a.add(pvt(real + 200), 0);
  EXPECT_EQ(a.add_eoe(real + 200, stamp(), 0).size(), 1u);
  EXPECT_EQ(a.stats().late_messages, 0u);
  EXPECT_EQ(a.stats().time_jumps, 1u);
}
