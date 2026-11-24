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

// Splits the receiver's USB byte stream into UBX frames, NMEA sentences and RTCM 3 frames,
// whatever the USB transfer boundaries are (a transfer may hold several messages or part of one).

#ifndef UBLOX_DGNSS_NODE__STREAM_PARSER_HPP_
#define UBLOX_DGNSS_NODE__STREAM_PARSER_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace ublox_dgnss
{

struct StreamFrame
{
  enum class Kind { UBX, NMEA, RTCM3 };
  Kind kind{Kind::UBX};
  // UBX and RTCM 3: the whole frame (sync/preamble to checksum/CRC).
  // NMEA: the sentence "$...*HH" without CR LF.
  std::vector<uint8_t> bytes;
};

struct StreamParserStats
{
  uint64_t ubx_frames{0};
  uint64_t nmea_sentences{0};
  uint64_t rtcm3_frames{0};
  uint64_t ubx_checksum_errors{0};
  uint64_t ubx_header_errors{0};  // sync pair followed by a class or length no real message has
  uint64_t nmea_errors{0};
  uint64_t rtcm3_crc_errors{0};
  uint64_t garbage_bytes{0};
};

class StreamParser
{
public:
  using Callback = std::function<void (const StreamFrame &)>;

  // Longest UBX payload accepted for a message without a length rule: a false "B5 62" inside
  // data then holds real frames back by at most this many bytes.
  static constexpr size_t MAX_UNRULED_UBX_PAYLOAD = 1024;
  static constexpr size_t MAX_NMEA = 100;  // NMEA 0183 allows 82 characters; u-blox HP a bit more

  explicit StreamParser(Callback callback);
  // Bytes may arrive in any chunking; complete frames go to the callback in stream order.
  void feed(const uint8_t * data, size_t len);
  // Drops a partial frame, e.g. after the device was unplugged.
  void reset();
  const StreamParserStats & stats() const {return stats_;}

private:
  enum class Result { FRAME, INCOMPLETE, BAD };
  Result try_ubx(size_t pos, size_t & consumed);
  Result try_nmea(size_t pos, size_t & consumed);
  Result try_rtcm3(size_t pos, size_t & consumed);

  Callback callback_;
  std::vector<uint8_t> buf_;
  StreamParserStats stats_;
};

// "$...*HH" with a correct XOR checksum.
bool nmea_checksum_ok(const std::string & sentence);
// "$GNGGA,..." -> "GGA" (two-letter talker ID removed).
std::string nmea_sentence_type(const std::string & sentence);
// CRC-24Q as used by RTCM 3.
uint32_t crc24q(const uint8_t * data, size_t len);

}  // namespace ublox_dgnss

#endif  // UBLOX_DGNSS_NODE__STREAM_PARSER_HPP_
