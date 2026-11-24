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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_PL_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_PL_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::pl
{
// UBX-NAV-PL (0x01 0x62), 52 bytes: protection levels of the navigation solution.
class NavPLPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_PL;
  static const size_t PAYLOAD_SIZE = 52;

  u1_t version;           // message version
  u1_t tmirCoeff;         // target misleading information risk = tmirCoeff * 10^tmirExp [%/epoch]
  i1_t tmirExp;
  u1_t plPosValid;        // 1 = position protection level valid
  u1_t plPosFrame;        // 0 invalid, 1 NED, 2 longitudinal-lateral-vertical, 3 HorizSemiMajor-
                          // HorizSemiMinor-Vertical
  u1_t plVelValid;        // 1 = velocity protection level valid
  u1_t plVelFrame;        // same values as plPosFrame
  u1_t plTimeValid;       // 1 = time protection level valid
  // 4 bytes reserved
  u4_t iTOW;              // ms - GPS time of week of the navigation epoch
  u4_t plPos1;            // mm - position PL, 1st axis of plPosFrame
  u4_t plPos2;            // mm - position PL, 2nd axis
  u4_t plPos3;            // mm - position PL, 3rd axis
  u4_t plVel1;            // mm/s - velocity PL, 1st axis of plVelFrame
  u4_t plVel2;            // mm/s - velocity PL, 2nd axis
  u4_t plVel3;            // mm/s - velocity PL, 3rd axis
  u2_t plPosHorizOrient;  // 1e-2 deg - orientation of the horizontal position frame
  u2_t plVelHorizOrient;  // 1e-2 deg - orientation of the horizontal velocity frame
  u4_t plTime;            // ns - time protection level
  // 4 bytes reserved

public:
  NavPLPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  NavPLPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    // A short (malformed) payload reads as zeros, i.e. all protection levels invalid.
    if (payload_.size() < PAYLOAD_SIZE) {
      payload_.resize(PAYLOAD_SIZE, 0);
    }
    version = buf_offset<u1_t>(&payload_, 0);
    tmirCoeff = buf_offset<u1_t>(&payload_, 1);
    tmirExp = buf_offset<i1_t>(&payload_, 2);
    plPosValid = buf_offset<u1_t>(&payload_, 3);
    plPosFrame = buf_offset<u1_t>(&payload_, 4);
    plVelValid = buf_offset<u1_t>(&payload_, 5);
    plVelFrame = buf_offset<u1_t>(&payload_, 6);
    plTimeValid = buf_offset<u1_t>(&payload_, 7);
    iTOW = buf_offset<u4_t>(&payload_, 12);
    plPos1 = buf_offset<u4_t>(&payload_, 16);
    plPos2 = buf_offset<u4_t>(&payload_, 20);
    plPos3 = buf_offset<u4_t>(&payload_, 24);
    plVel1 = buf_offset<u4_t>(&payload_, 28);
    plVel2 = buf_offset<u4_t>(&payload_, 32);
    plVel3 = buf_offset<u4_t>(&payload_, 36);
    plPosHorizOrient = buf_offset<u2_t>(&payload_, 40);
    plVelHorizOrient = buf_offset<u2_t>(&payload_, 42);
    plTime = buf_offset<u4_t>(&payload_, 44);
  }
  std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();
    return std::make_tuple(payload_.data(), payload_.size());
  }
  std::string to_string()
  {
    std::ostringstream oss;
    oss << "version: " << +version;
    oss << " tmir: " << +tmirCoeff << "e" << +tmirExp;
    oss << " iTOW: " << iTOW;
    oss << " plPosValid: " << +plPosValid;
    oss << " plPosFrame: " << +plPosFrame;
    oss << " plPos: [" << plPos1 << ", " << plPos2 << ", " << plPos3 << "] mm";
    oss << " plPosHorizOrient: " << plPosHorizOrient * 1e-2 << " deg";
    oss << " plVelValid: " << +plVelValid;
    oss << " plVelFrame: " << +plVelFrame;
    oss << " plVel: [" << plVel1 << ", " << plVel2 << ", " << plVel3 << "] mm/s";
    oss << " plVelHorizOrient: " << plVelHorizOrient * 1e-2 << " deg";
    oss << " plTimeValid: " << +plTimeValid;
    oss << " plTime: " << plTime << " ns";
    return oss.str();
  }
};
}  // namespace ubx::nav::pl
#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_PL_HPP_
