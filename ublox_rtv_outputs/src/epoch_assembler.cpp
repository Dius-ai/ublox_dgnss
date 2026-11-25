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

#include "ublox_rtv_outputs/epoch_assembler.hpp"

#include <utility>

namespace ublox_rtv_outputs
{

std::vector<Epoch> EpochAssembler::add(const ublox_ubx_msgs::msg::UBXNavPVT & m, int64_t now_ns)
{
  Pending * p = entry(m.itow, m.header.stamp);
  if (!p) {
    return {};
  }
  p->epoch.pvt = m;
  return collect(now_ns);
}

std::vector<Epoch> EpochAssembler::add(
  const ublox_ubx_msgs::msg::UBXNavHPPosLLH & m, int64_t now_ns)
{
  Pending * p = entry(m.itow, m.header.stamp);
  if (!p) {
    return {};
  }
  p->epoch.hpposllh = m;
  return collect(now_ns);
}

std::vector<Epoch> EpochAssembler::add(const ublox_ubx_msgs::msg::UBXNavCov & m, int64_t now_ns)
{
  Pending * p = entry(m.itow, m.header.stamp);
  if (!p) {
    return {};
  }
  p->epoch.cov = m;
  return collect(now_ns);
}

std::vector<Epoch> EpochAssembler::add(const ublox_ubx_msgs::msg::UBXNavDOP & m, int64_t now_ns)
{
  Pending * p = entry(m.itow, m.header.stamp);
  if (!p) {
    return {};
  }
  p->epoch.dop = m;
  return collect(now_ns);
}

std::vector<Epoch> EpochAssembler::add_eoe(
  uint32_t itow, const builtin_interfaces::msg::Time & stamp, int64_t now_ns)
{
  Pending * p = entry(itow, stamp);
  if (!p) {
    return {};
  }
  p->epoch.closed_by_eoe = true;
  p->eoe_ns = now_ns;
  return collect(now_ns);
}

std::vector<Epoch> EpochAssembler::flush(int64_t now_ns)
{
  return collect(now_ns);
}

int64_t EpochAssembler::unwrap(uint32_t itow)
{
  constexpr int64_t kWeekMs = 604800000;
  if (!have_ref_) {
    have_ref_ = true;
    ref_ = itow;
    return itow;
  }
  // same week as the newest iTOW seen, or the neighbouring one across the week rollover
  int64_t u = ref_ - (ref_ % kWeekMs) + itow;
  if (u - ref_ > kWeekMs / 2) {
    u -= kWeekMs;
  } else if (ref_ - u > kWeekMs / 2) {
    u += kWeekMs;
  }
  if (u > ref_) {
    ref_ = u;
  }
  return u;
}

EpochAssembler::Pending * EpochAssembler::entry(
  uint32_t itow, const builtin_interfaces::msg::Time & stamp)
{
  /* TODO(review): Original: a backward time jump made every message "late" until time caught up
  const int64_t key = unwrap(itow);
  */
  // TODO(review): New implementation restarting the assembler on a backward time jump
  int64_t key = unwrap(itow);
  if (ref_ - key > kTimeJumpMs) {
    ++stats_.time_jumps;
    pending_.clear();
    have_ref_ = false;
    have_last_ = false;
    expect_hp_ = expect_cov_ = expect_dop_ = false;
    key = unwrap(itow);
  }
  if (have_last_ && key <= last_key_) {
    ++stats_.late_messages;
    return nullptr;
  }
  auto it = pending_.find(key);
  if (it == pending_.end()) {
    it = pending_.try_emplace(key).first;
    it->second.epoch.itow = itow;
    it->second.epoch.rx_stamp = stamp;
  }
  return &it->second;
}

bool EpochAssembler::complete(const Epoch & e) const
{
  return e.pvt && (!expect_hp_ || e.hpposllh) && (!expect_cov_ || e.cov) &&
         (!expect_dop_ || e.dop);
}

std::vector<Epoch> EpochAssembler::collect(int64_t now_ns)
{
  std::vector<Epoch> out;
  while (!pending_.empty()) {
    auto it = pending_.begin();
    const Pending & p = it->second;
    const bool eoe = p.epoch.closed_by_eoe;
    const bool next_started = pending_.size() > 1;
    const bool timed_out = eoe && now_ns - p.eoe_ns >= timeout_ns_;
    if (!(eoe && complete(p.epoch)) && !next_started && !timed_out) {
      break;
    }
    have_last_ = true;
    last_key_ = it->first;
    Epoch e = std::move(it->second.epoch);
    pending_.erase(it);
    if (!e.pvt) {
      ++stats_.epochs_without_pvt;
      continue;
    }
    expect_hp_ = e.hpposllh.has_value();
    expect_cov_ = e.cov.has_value();
    expect_dop_ = e.dop.has_value();
    ++stats_.epochs;
    stats_.epochs_without_eoe += e.closed_by_eoe ? 0 : 1;
    out.push_back(std::move(e));
  }
  return out;
}

}  // namespace ublox_rtv_outputs
