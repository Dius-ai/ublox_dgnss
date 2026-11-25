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

// Groups the NAV messages ublox_dgnss publishes on separate topics into navigation epochs (same
// iTOW). Topics do not keep the receiver's message order, so NAV-EOE closes an epoch only once
// the message types of the previous epoch have arrived too; a message that never comes is
// waited for until the next epoch starts or `timeout_ns` after the EOE (see flush()).

#ifndef UBLOX_RTV_OUTPUTS__EPOCH_ASSEMBLER_HPP_
#define UBLOX_RTV_OUTPUTS__EPOCH_ASSEMBLER_HPP_

#include <cstdint>
#include <map>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "ublox_rtv_outputs/conversions.hpp"

namespace ublox_rtv_outputs
{

struct AssemblerStats
{
  uint64_t epochs{0};              // returned, with NAV-PVT
  uint64_t epochs_without_eoe{0};  // returned before their NAV-EOE arrived
  uint64_t epochs_without_pvt{0};  // dropped: no NAV-PVT
  uint64_t late_messages{0};       // arrived after their epoch was returned
  uint64_t time_jumps{0};          // iTOW went back more than kTimeJumpMs: assembler restarted
};

class EpochAssembler
{
public:
  explicit EpochAssembler(int64_t timeout_ns = 100000000)
  : timeout_ns_(timeout_ns) {}

  // Each returns the epochs this message finished, oldest first. `now_ns`: a monotonic clock.
  std::vector<Epoch> add(const ublox_ubx_msgs::msg::UBXNavPVT & m, int64_t now_ns);
  std::vector<Epoch> add(const ublox_ubx_msgs::msg::UBXNavHPPosLLH & m, int64_t now_ns);
  std::vector<Epoch> add(const ublox_ubx_msgs::msg::UBXNavCov & m, int64_t now_ns);
  std::vector<Epoch> add(const ublox_ubx_msgs::msg::UBXNavDOP & m, int64_t now_ns);
  std::vector<Epoch> add_eoe(
    uint32_t itow, const builtin_interfaces::msg::Time & stamp, int64_t now_ns);

  /// Epochs whose NAV-EOE arrived more than the timeout ago; call periodically.
  std::vector<Epoch> flush(int64_t now_ns);

  const AssemblerStats & stats() const {return stats_;}

  // TODO(review): A message this far behind the newest iTOW is not a late one (those lag by an
  // epoch or two) but a receiver time jump (time fix after a pre-fix guess, cold reset, bag
  // replayed in a loop): start over instead of dropping everything until time catches up.
  static constexpr int64_t kTimeJumpMs = 30000;

private:
  struct Pending
  {
    Epoch epoch;
    int64_t eoe_ns{0};
  };

  /// iTOW continued across week rollovers, so epochs keep their order.
  int64_t unwrap(uint32_t itow);
  /// Pending epoch for `itow`, or nullptr if that epoch was already returned.
  Pending * entry(uint32_t itow, const builtin_interfaces::msg::Time & stamp);
  std::vector<Epoch> collect(int64_t now_ns);
  bool complete(const Epoch & e) const;

  int64_t timeout_ns_;
  std::map<int64_t, Pending> pending_;  // by unwrapped iTOW
  bool have_ref_{false};
  int64_t ref_{0};  // newest unwrapped iTOW seen
  bool have_last_{false};
  int64_t last_key_{0};  // newest returned or dropped epoch
  // message types of the last returned epoch, expected in the next one
  bool expect_hp_{false}, expect_cov_{false}, expect_dop_{false};
  AssemblerStats stats_;
};

}  // namespace ublox_rtv_outputs

#endif  // UBLOX_RTV_OUTPUTS__EPOCH_ASSEMBLER_HPP_
