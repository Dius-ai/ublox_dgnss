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

#include "ublox_dgnss_node/stream_parser.hpp"

#include <algorithm>
#include <string>
#include <utility>

namespace ublox_dgnss
{

namespace
{
constexpr uint8_t UBX_SYNC_1 = 0xB5;
constexpr uint8_t UBX_SYNC_2 = 0x62;
constexpr uint8_t RTCM3_PREAMBLE = 0xD3;

// Messages whose payload can exceed MAX_UNRULED_UBX_PAYLOAD, or whose length is fully determined
// by a count field: payload length == base + step * payload[count_offset]
// (u-blox F9 HPG 1.32 / X20 HPG 2.00 interface descriptions).
struct CountedRule
{
  uint8_t msg_class;
  uint8_t msg_id;
  uint16_t base;
  uint16_t step;
  uint8_t count_offset;
};

constexpr CountedRule COUNTED_RULES[] = {
  {0x01, 0x35, 8, 12, 5},    // NAV-SAT numSvs
  {0x01, 0x43, 8, 16, 5},    // NAV-SIG numSigs
  {0x01, 0x34, 8, 6, 5},     // NAV-ORB numSv
  {0x02, 0x15, 16, 32, 11},  // RXM-RAWX numMeas
  {0x02, 0x13, 8, 4, 4},     // RXM-SFRBX numWords
  {0x02, 0x14, 44, 24, 34},  // RXM-MEASX numSV
  {0x0A, 0x36, 8, 40, 1},    // MON-COMMS nPorts
};

// Classes a u-blox F9/X20 can output; any other class after "B5 62" is not a header.
bool known_class(uint8_t c)
{
  switch (c) {
    case 0x01:  // NAV
    case 0x02:  // RXM
    case 0x04:  // INF
    case 0x05:  // ACK
    case 0x06:  // CFG
    case 0x09:  // UPD
    case 0x0A:  // MON
    case 0x0D:  // TIM
    case 0x10:  // ESF
    case 0x13:  // MGA
    case 0x21:  // LOG
    case 0x27:  // SEC
    case 0x28:  // HNR
    case 0x29:  // NAV2
      return true;
    default:
      return false;
  }
}

int hex_value(char c)
{
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  return -1;
}
}  // namespace

bool nmea_checksum_ok(const std::string & s)
{
  if (s.size() < 4 || s[0] != '$') {
    return false;
  }
  const auto star = s.rfind('*');
  if (star == std::string::npos || star + 3 != s.size()) {
    return false;
  }
  uint8_t sum = 0;
  for (size_t i = 1; i < star; ++i) {
    sum = static_cast<uint8_t>(sum ^ static_cast<uint8_t>(s[i]));
  }
  const int hi = hex_value(s[star + 1]);
  const int lo = hex_value(s[star + 2]);
  return hi >= 0 && lo >= 0 && sum == static_cast<uint8_t>(hi * 16 + lo);
}

std::string nmea_sentence_type(const std::string & s)
{
  if (s.size() < 2 || s[0] != '$') {
    return "";
  }
  const auto comma = s.find(',');
  const size_t address_len = comma == std::string::npos ? std::string::npos : comma - 1;
  const std::string address = s.substr(1, address_len);
  if (!address.empty() && address[0] == 'P') {
    return address;  // proprietary, e.g. "PUBX"
  }
  return address.size() > 2 ? address.substr(2) : address;
}

uint32_t crc24q(const uint8_t * data, size_t len)
{
  uint32_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint32_t>(data[i]) << 16;
    for (int b = 0; b < 8; ++b) {
      crc <<= 1;
      if (crc & 0x1000000) {
        crc ^= 0x1864CFB;
      }
    }
  }
  return crc & 0xFFFFFF;
}

StreamParser::StreamParser(Callback callback)
: callback_(std::move(callback))
{
}

void StreamParser::reset()
{
  buf_.clear();
}

StreamParser::Result StreamParser::try_ubx(size_t pos, size_t & consumed)
{
  const size_t avail = buf_.size() - pos;
  if (avail < 2) {
    return Result::INCOMPLETE;
  }
  if (buf_[pos + 1] != UBX_SYNC_2) {
    return Result::BAD;
  }
  if (avail < 6) {
    return Result::INCOMPLETE;
  }
  // Check the header before waiting for the payload: during resynchronisation "B5 62" can occur
  // inside data, and waiting for a random length would hold back every real frame behind it.
  const uint8_t msg_class = buf_[pos + 2];
  const uint8_t msg_id = buf_[pos + 3];
  const size_t len = static_cast<size_t>(buf_[pos + 4]) | (static_cast<size_t>(buf_[pos + 5]) << 8);
  bool plausible = known_class(msg_class) && len <= MAX_UNRULED_UBX_PAYLOAD;
  for (const auto & r : COUNTED_RULES) {
    if (r.msg_class != msg_class || r.msg_id != msg_id) {
      continue;
    }
    if (len <= r.count_offset) {
      plausible = false;
      break;
    }
    if (avail < 6 + r.count_offset + 1u) {
      return Result::INCOMPLETE;
    }
    const size_t count = buf_[pos + 6 + r.count_offset];
    plausible = len == r.base + r.step * count;
    break;
  }
  if (!plausible) {
    ++stats_.ubx_header_errors;
    return Result::BAD;
  }
  if (avail < len + 8) {
    return Result::INCOMPLETE;
  }
  uint8_t a = 0;
  uint8_t b = 0;
  for (size_t i = pos + 2; i < pos + 6 + len; ++i) {
    a = static_cast<uint8_t>(a + buf_[i]);
    b = static_cast<uint8_t>(b + a);
  }
  if (a != buf_[pos + 6 + len] || b != buf_[pos + 7 + len]) {
    ++stats_.ubx_checksum_errors;
    return Result::BAD;
  }
  StreamFrame f;
  f.kind = StreamFrame::Kind::UBX;
  f.bytes.assign(
    buf_.begin() + static_cast<std::ptrdiff_t>(pos),
    buf_.begin() + static_cast<std::ptrdiff_t>(pos + len + 8));
  ++stats_.ubx_frames;
  consumed = len + 8;
  callback_(f);
  return Result::FRAME;
}

StreamParser::Result StreamParser::try_nmea(size_t pos, size_t & consumed)
{
  const size_t limit = std::min(buf_.size(), pos + MAX_NMEA + 2);  // + CR LF
  for (size_t i = pos + 1; i < limit; ++i) {
    const uint8_t c = buf_[i];
    if (c == '\n') {
      size_t end = i;
      if (end > pos && buf_[end - 1] == '\r') {
        --end;
      }
      std::string s(
        buf_.begin() + static_cast<std::ptrdiff_t>(pos),
        buf_.begin() + static_cast<std::ptrdiff_t>(end));
      if (!nmea_checksum_ok(s)) {
        ++stats_.nmea_errors;
        return Result::BAD;
      }
      StreamFrame f;
      f.kind = StreamFrame::Kind::NMEA;
      f.bytes.assign(s.begin(), s.end());
      ++stats_.nmea_sentences;
      consumed = i + 1 - pos;
      callback_(f);
      return Result::FRAME;
    }
    const bool printable = c >= 0x20 && c <= 0x7E;
    if (!printable && c != '\r') {
      ++stats_.nmea_errors;  // broken sentence (e.g. a UBX frame inside it)
      return Result::BAD;
    }
  }
  if (limit == buf_.size() && buf_.size() - pos < MAX_NMEA + 2) {
    return Result::INCOMPLETE;
  }
  ++stats_.nmea_errors;  // too long without a line end
  return Result::BAD;
}

StreamParser::Result StreamParser::try_rtcm3(size_t pos, size_t & consumed)
{
  const size_t avail = buf_.size() - pos;
  if (avail < 3) {
    return Result::INCOMPLETE;
  }
  if ((buf_[pos + 1] & 0xFC) != 0) {  // 6 reserved bits are zero
    return Result::BAD;
  }
  const size_t len = (static_cast<size_t>(buf_[pos + 1] & 0x03) << 8) | buf_[pos + 2];
  if (avail < len + 6) {
    return Result::INCOMPLETE;
  }
  const uint32_t crc = (static_cast<uint32_t>(buf_[pos + 3 + len]) << 16) |
    (static_cast<uint32_t>(buf_[pos + 4 + len]) << 8) | buf_[pos + 5 + len];
  if (crc24q(&buf_[pos], len + 3) != crc) {
    ++stats_.rtcm3_crc_errors;
    return Result::BAD;
  }
  StreamFrame f;
  f.kind = StreamFrame::Kind::RTCM3;
  f.bytes.assign(
    buf_.begin() + static_cast<std::ptrdiff_t>(pos),
    buf_.begin() + static_cast<std::ptrdiff_t>(pos + len + 6));
  ++stats_.rtcm3_frames;
  consumed = len + 6;
  callback_(f);
  return Result::FRAME;
}

void StreamParser::feed(const uint8_t * data, size_t len)
{
  buf_.insert(buf_.end(), data, data + len);
  size_t pos = 0;
  while (pos < buf_.size()) {
    const uint8_t b = buf_[pos];
    Result r = Result::BAD;
    size_t consumed = 0;
    if (b == UBX_SYNC_1) {
      r = try_ubx(pos, consumed);
    } else if (b == '$') {
      r = try_nmea(pos, consumed);
    } else if (b == RTCM3_PREAMBLE) {
      r = try_rtcm3(pos, consumed);
    } else {
      ++stats_.garbage_bytes;
      ++pos;
      continue;
    }
    if (r == Result::INCOMPLETE) {
      break;
    }
    if (r == Result::FRAME) {
      pos += consumed;
    } else {
      ++stats_.garbage_bytes;  // resynchronise one byte later
      ++pos;
    }
  }
  buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(pos));
}

}  // namespace ublox_dgnss
