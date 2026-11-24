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

// PtyLink: the test plays the NTRIP client, opening the symlink as its serial device.

#include <gtest/gtest.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "ublox_dgnss_node/pty_link.hpp"

using ublox_dgnss::PtyLink;

namespace
{
std::string temp_dir()
{
  std::string t = "/tmp/pty_link_testXXXXXX";
  return ::mkdtemp(t.data());
}

std::vector<uint8_t> read_some(int fd)
{
  std::vector<uint8_t> out(256);
  for (int i = 0; i < 100; ++i) {
    const ssize_t n = ::read(fd, out.data(), out.size());
    if (n > 0) {
      out.resize(static_cast<size_t>(n));
      return out;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return {};
}

std::string gga(int i)
{
  return "$GNGGA,1312" + std::to_string(10 + i) + ".00,4808.44972,N*00\r\n";
}

size_t read_driver_side(PtyLink & pty, std::vector<uint8_t> & got)
{
  got.resize(64);
  size_t n = 0;
  for (int i = 0; i < 100 && n == 0; ++i) {
    n = pty.read(got.data(), got.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  got.resize(n);
  return n;
}
}  // namespace

TEST(PtyLink, ClientTalksToDriverThroughSymlink)
{
  const std::string dir = temp_dir();
  const std::string link = dir + "/ublox";
  {
    PtyLink pty;
    pty.open(link);
    char target[128] = {};
    ASSERT_GT(::readlink(link.c_str(), target, sizeof(target) - 1), 0);
    EXPECT_EQ(std::string(target), pty.slave_path());

    const int client = ::open(link.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    ASSERT_GE(client, 0);

    // RTCM: client -> driver, binary bytes unchanged (raw mode: no CR/LF translation, no echo)
    const std::vector<uint8_t> rtcm{0xD3, 0x00, 0x13, 0x3E, 0xD0, 0x0D, 0x0A, 0x00, 0x03, 0xFF};
    ASSERT_EQ(::write(client, rtcm.data(), rtcm.size()), static_cast<ssize_t>(rtcm.size()));
    std::vector<uint8_t> got;
    read_driver_side(pty, got);
    EXPECT_EQ(got, rtcm);

    // GGA: driver -> client
    const std::string gga =
      "$GNGGA,131205.20,4808.44972,N,01707.55822,E,1,12,0.63,180.8,M,41.4,M,,*7A\r\n";
    EXPECT_TRUE(pty.write(reinterpret_cast<const uint8_t *>(gga.data()), gga.size()));
    const auto line = read_some(client);
    EXPECT_EQ(std::string(line.begin(), line.end()), gga);

    // nothing echoed back to the driver
    got.resize(64);
    EXPECT_EQ(pty.read(got.data(), got.size()), 0u);

    // the client closes and reopens (reconnect): the link keeps working
    ::close(client);
    EXPECT_EQ(pty.read(got.data(), got.size()), 0u);
    const int client2 = ::open(link.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    ASSERT_GE(client2, 0);
    ASSERT_EQ(::write(client2, rtcm.data(), 3), 3);
    EXPECT_EQ(read_driver_side(pty, got), 3u);
    ::close(client2);
  }
  struct stat st {};
  EXPECT_NE(::lstat(link.c_str(), &st), 0) << "symlink removed on close";
  ::rmdir(dir.c_str());
}

// GGA written while no client reads must not pile up: a client starting later would first get
// stale positions (and, with a full queue, a sentence cut in half).
TEST(PtyLink, UnreadMessagesAreReplacedByTheNewest)
{
  const std::string dir = temp_dir();
  const std::string link = dir + "/ublox";
  PtyLink pty;
  pty.open(link);
  size_t discarded = 0;
  for (int i = 0; i < 80; ++i) {
    const std::string s = gga(i);
    ASSERT_TRUE(pty.write(reinterpret_cast<const uint8_t *>(s.data()), s.size(), &discarded)) << i;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  const std::string last = gga(80);
  ASSERT_TRUE(pty.write(reinterpret_cast<const uint8_t *>(last.data()), last.size(), &discarded));
  EXPECT_EQ(discarded, gga(79).size());

  const int client = ::open(link.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  ASSERT_GE(client, 0);
  const auto got = read_some(client);
  EXPECT_EQ(std::string(got.begin(), got.end()), last);
  ::close(client);
  pty.close();
  ::rmdir(dir.c_str());
}

TEST(PtyLink, MessageThatDoesNotFitLeavesNoFragment)
{
  const std::string dir = temp_dir();
  const std::string link = dir + "/ublox";
  PtyLink pty;
  pty.open(link);
  const int client = ::open(link.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  ASSERT_GE(client, 0);
  const std::vector<uint8_t> huge(1 << 20, 'x');  // far more than the pty buffers
  EXPECT_FALSE(pty.write(huge.data(), huge.size()));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  std::vector<uint8_t> buf(256);
  EXPECT_LT(::read(client, buf.data(), buf.size()), 1) << "a fragment reached the client";
  ::close(client);
  pty.close();
  ::rmdir(dir.c_str());
}

TEST(PtyLink, ReplacesStaleSymlinkButNotAFile)
{
  const std::string dir = temp_dir();
  const std::string link = dir + "/ublox";
  ASSERT_EQ(::symlink("/dev/pts/9999", link.c_str()), 0);  // left over by a crashed driver
  {
    PtyLink pty;
    EXPECT_NO_THROW(pty.open(link));
  }
  std::ofstream(link) << "not a symlink";
  {
    PtyLink pty;
    EXPECT_THROW(pty.open(link), std::runtime_error);
  }
  ::unlink(link.c_str());
  ::rmdir(dir.c_str());
}
