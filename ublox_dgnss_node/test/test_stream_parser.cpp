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

#include <algorithm>
#include <string>
#include <vector>

#include "ublox_dgnss_node/stream_parser.hpp"

using ublox_dgnss::StreamFrame;
using ublox_dgnss::StreamParser;

namespace
{
// Example sentence from the NMEA 0183 reference, checksum 47.
const char GGA[] = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";

std::vector<uint8_t> bytes(const std::string & s)
{
  return {s.begin(), s.end()};
}

std::vector<uint8_t> ubx(uint8_t cls, uint8_t id, const std::vector<uint8_t> & payload)
{
  std::vector<uint8_t> out{0xB5, 0x62, cls, id,
    static_cast<uint8_t>(payload.size() & 0xFF), static_cast<uint8_t>(payload.size() >> 8)};
  out.insert(out.end(), payload.begin(), payload.end());
  uint8_t a = 0;
  uint8_t b = 0;
  for (size_t i = 2; i < out.size(); ++i) {
    a = static_cast<uint8_t>(a + out[i]);
    b = static_cast<uint8_t>(b + a);
  }
  out.push_back(a);
  out.push_back(b);
  return out;
}

std::vector<uint8_t> rtcm3(const std::vector<uint8_t> & payload)
{
  std::vector<uint8_t> out{0xD3, static_cast<uint8_t>(payload.size() >> 8),
    static_cast<uint8_t>(payload.size() & 0xFF)};
  out.insert(out.end(), payload.begin(), payload.end());
  const uint32_t crc = ublox_dgnss::crc24q(out.data(), out.size());
  out.push_back(static_cast<uint8_t>(crc >> 16));
  out.push_back(static_cast<uint8_t>(crc >> 8));
  out.push_back(static_cast<uint8_t>(crc));
  return out;
}

void append(std::vector<uint8_t> & to, const std::vector<uint8_t> & what)
{
  to.insert(to.end(), what.begin(), what.end());
}

struct Collector
{
  std::vector<StreamFrame> frames;
  StreamParser parser{[this](const StreamFrame & f) {frames.push_back(f);}};
  void feed(const std::vector<uint8_t> & b) {parser.feed(b.data(), b.size());}
};
}  // namespace

TEST(Nmea, ChecksumAndType)
{
  EXPECT_TRUE(ublox_dgnss::nmea_checksum_ok(GGA));
  EXPECT_FALSE(
    ublox_dgnss::nmea_checksum_ok(
      "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48"));
  EXPECT_FALSE(ublox_dgnss::nmea_checksum_ok("$GPGGA,123519"));
  EXPECT_EQ(ublox_dgnss::nmea_sentence_type(GGA), "GGA");
  EXPECT_EQ(ublox_dgnss::nmea_sentence_type("$PUBX,00,1*00"), "PUBX");
}

TEST(Rtcm3, Crc24qKnownValue)
{
  // CRC-24Q of "123456789" is 0xCDE703.
  const auto s = bytes("123456789");
  EXPECT_EQ(ublox_dgnss::crc24q(s.data(), s.size()), 0xCDE703u);
}

// Several messages in one USB transfer: the per-transfer handling would see only the first.
TEST(Parser, SeveralMessagesInOneChunk)
{
  std::vector<uint8_t> chunk = ubx(0x01, 0x07, std::vector<uint8_t>(92, 0x11));  // NAV-PVT
  append(chunk, bytes(std::string(GGA) + "\r\n"));
  append(chunk, rtcm3(std::vector<uint8_t>(300, 0x22)));  // longer than 255: length MSB set
  append(chunk, ubx(0x01, 0x61, {1, 2, 3, 4}));  // NAV-EOE
  Collector c;
  c.feed(chunk);
  ASSERT_EQ(c.frames.size(), 4u);
  EXPECT_EQ(c.frames[0].kind, StreamFrame::Kind::UBX);
  EXPECT_EQ(c.frames[0].bytes, ubx(0x01, 0x07, std::vector<uint8_t>(92, 0x11)));
  EXPECT_EQ(c.frames[1].kind, StreamFrame::Kind::NMEA);
  EXPECT_EQ(std::string(c.frames[1].bytes.begin(), c.frames[1].bytes.end()), GGA);
  EXPECT_EQ(c.frames[2].kind, StreamFrame::Kind::RTCM3);
  EXPECT_EQ(c.frames[2].bytes.size(), 306u);
  EXPECT_EQ(c.frames[3].bytes, ubx(0x01, 0x61, {1, 2, 3, 4}));
  EXPECT_EQ(c.parser.stats().garbage_bytes, 0u);
}

// One message split across USB transfers (64-byte packets).
TEST(Parser, MessageSplitAcrossChunks)
{
  std::vector<uint8_t> stream = ubx(0x01, 0x07, std::vector<uint8_t>(92, 0x42));
  append(stream, bytes(std::string(GGA) + "\r\n"));
  append(stream, rtcm3(std::vector<uint8_t>(20, 0x33)));
  Collector c;
  for (size_t i = 0; i < stream.size(); i += 7) {
    const size_t end = std::min(i + 7, stream.size());
    c.feed(std::vector<uint8_t>(stream.begin() + i, stream.begin() + end));
  }
  ASSERT_EQ(c.frames.size(), 3u);
  EXPECT_EQ(c.frames[0].bytes.size(), 100u);
  EXPECT_EQ(c.frames[1].kind, StreamFrame::Kind::NMEA);
  EXPECT_EQ(c.frames[2].kind, StreamFrame::Kind::RTCM3);
  EXPECT_EQ(c.parser.stats().garbage_bytes, 0u);
}

TEST(Parser, GarbageAndCorruptionAreSkipped)
{
  std::vector<uint8_t> stream{0x00, 0x13, 0xB5};  // garbage, including a lone sync byte
  append(stream, ubx(0x05, 0x01, {0x06, 0x8A}));  // ACK-ACK
  auto bad = ubx(0x01, 0x07, std::vector<uint8_t>(92, 0x11));
  bad[20] ^= 0xFF;  // checksum fails
  append(stream, bad);
  auto bad_rtcm = rtcm3(std::vector<uint8_t>(10, 0x01));
  bad_rtcm[5] ^= 0xFF;  // CRC fails
  append(stream, bad_rtcm);
  append(stream, ubx(0x01, 0x61, {1, 2, 3, 4}));
  Collector c;
  c.feed(stream);
  ASSERT_EQ(c.frames.size(), 2u);
  EXPECT_EQ(c.frames[0].bytes[3], 0x01);  // ACK-ACK
  EXPECT_EQ(c.frames[1].bytes[3], 0x61);  // NAV-EOE
  EXPECT_EQ(c.parser.stats().ubx_checksum_errors, 1u);
  EXPECT_EQ(c.parser.stats().rtcm3_crc_errors, 1u);
  EXPECT_GT(c.parser.stats().garbage_bytes, 0u);
}

TEST(Parser, NmeaInterruptedByUbxIsDropped)
{
  auto stream = bytes("$GNGGA,1234");
  append(stream, ubx(0x01, 0x61, {1, 2, 3, 4}));
  Collector c;
  c.feed(stream);
  ASSERT_EQ(c.frames.size(), 1u);
  EXPECT_EQ(c.frames[0].bytes[3], 0x61);
  EXPECT_EQ(c.parser.stats().nmea_errors, 1u);
}

// While resynchronising, "B5 62" inside data followed by a random length must not make the
// parser wait for that many bytes: each real frame must come out as soon as its bytes are in.
TEST(Parser, FalseSyncDoesNotHoldBackFrames)
{
  const std::vector<std::vector<uint8_t>> false_headers{
    {0xB5, 0x62, 0x01, 0x07, 0x00, 0x1F},  // NAV-PVT of 7936 bytes
    {0xB5, 0x62, 0x77, 0x01, 0x00, 0x04},  // no such class
    {0xB5, 0x62, 0x01, 0x35, 0xF8, 0x00, 0, 0, 0, 0, 0, 3},  // NAV-SAT 248 bytes but numSvs 3
  };
  const std::vector<std::vector<uint8_t>> epoch{
    ubx(0x01, 0x07, std::vector<uint8_t>(92, 0x01)),
    ubx(0x01, 0x14, std::vector<uint8_t>(36, 0x02)),
    bytes(std::string(GGA) + "\r\n"),
    ubx(0x01, 0x61, {1, 2, 3, 4}),
  };
  for (const auto & header : false_headers) {
    Collector c;
    c.feed(header);
    for (size_t i = 0; i < epoch.size(); ++i) {
      c.feed(epoch[i]);
      ASSERT_EQ(c.frames.size(), i + 1) << "header class 0x" << std::hex << int{header[2]};
    }
    EXPECT_EQ(c.parser.stats().ubx_header_errors, 1u);
    EXPECT_EQ(c.parser.stats().ubx_checksum_errors, 0u);
  }
}

// Counted messages may exceed the cap for messages without a rule.
TEST(Parser, LargeCountedMessagesAreAccepted)
{
  std::vector<uint8_t> sig(8 + 16 * 100, 0);
  sig[5] = 100;  // NAV-SIG numSigs: 1608-byte payload
  std::vector<uint8_t> measx(44 + 24 * 50, 0);
  measx[34] = 50;  // RXM-MEASX numSV: 1244-byte payload
  std::vector<uint8_t> stream = ubx(0x01, 0x43, sig);
  append(stream, ubx(0x02, 0x14, measx));
  append(stream, ubx(0x01, 0x3C, std::vector<uint8_t>(64, 0)));  // NAV-RELPOSNED, no rule
  Collector c;
  c.feed(stream);
  ASSERT_EQ(c.frames.size(), 3u);
  EXPECT_EQ(c.frames[0].bytes.size(), sig.size() + 8);
  EXPECT_EQ(c.frames[1].bytes.size(), measx.size() + 8);
  EXPECT_EQ(c.parser.stats().ubx_header_errors, 0u);
}

TEST(Parser, ResetDropsPartialFrame)
{
  const auto pvt = ubx(0x01, 0x07, std::vector<uint8_t>(92, 0x11));
  Collector c;
  c.feed(std::vector<uint8_t>(pvt.begin(), pvt.begin() + 40));
  c.parser.reset();
  c.feed(ubx(0x01, 0x61, {1, 2, 3, 4}));
  ASSERT_EQ(c.frames.size(), 1u);
  EXPECT_EQ(c.frames[0].bytes[3], 0x61);
}
