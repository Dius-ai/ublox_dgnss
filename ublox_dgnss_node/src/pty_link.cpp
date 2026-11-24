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

#include "ublox_dgnss_node/pty_link.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <system_error>

namespace ublox_dgnss
{
namespace
{
[[noreturn]] void throw_errno(const std::string & what)
{
  throw std::system_error(errno, std::generic_category(), what);
}
}  // namespace

PtyLink::~PtyLink()
{
  close();
}

void PtyLink::open(const std::string & link_path)
{
  close();
  const int m = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (m < 0) {
    throw_errno("posix_openpt");
  }
  master_ = m;
  try {
    if (::grantpt(m) != 0 || ::unlockpt(m) != 0) {
      throw_errno("grantpt/unlockpt");
    }
    char name[128];
    if (::ptsname_r(m, name, sizeof(name)) != 0) {
      throw_errno("ptsname_r");
    }
    slave_path_ = name;
    slave_ = ::open(name, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (slave_ < 0) {
      throw_errno("open " + slave_path_);
    }
    // Raw: no echo (GGA would come straight back as "RTCM"), no CR/LF translation of binary RTCM.
    termios tio{};
    if (::tcgetattr(slave_, &tio) != 0) {
      throw_errno("tcgetattr " + slave_path_);
    }
    ::cfmakeraw(&tio);
    if (::tcsetattr(slave_, TCSANOW, &tio) != 0) {
      throw_errno("tcsetattr " + slave_path_);
    }

    struct stat st {};
    if (::lstat(link_path.c_str(), &st) == 0) {
      if (!S_ISLNK(st.st_mode)) {
        throw std::runtime_error(link_path + " exists and is not a symlink");
      }
      ::unlink(link_path.c_str());
    }
    if (::symlink(slave_path_.c_str(), link_path.c_str()) != 0) {
      throw_errno("symlink " + link_path + " -> " + slave_path_);
    }
    link_path_ = link_path;
  } catch (...) {
    close();
    throw;
  }
}

void PtyLink::close()
{
  if (!link_path_.empty()) {
    char target[128];
    const ssize_t n = ::readlink(link_path_.c_str(), target, sizeof(target) - 1);
    if (n > 0 && std::string(target, static_cast<size_t>(n)) == slave_path_) {
      ::unlink(link_path_.c_str());  // only if a newer driver has not taken it over
    }
    link_path_.clear();
  }
  if (slave_ >= 0) {
    ::close(slave_);
    slave_ = -1;
  }
  if (master_ >= 0) {
    ::close(master_);
    master_ = -1;
  }
  slave_path_.clear();
}

size_t PtyLink::read(uint8_t * buf, size_t len)
{
  if (master_ < 0) {
    return 0;
  }
  const ssize_t n = ::read(master_, buf, len);
  if (n < 0) {
    if (errno == EAGAIN || errno == EINTR || errno == EIO) {  // EIO: no slave open (not with ours)
      return 0;
    }
    throw_errno("read " + slave_path_ + " (master)");
  }
  return static_cast<size_t>(n);
}

bool PtyLink::write(const uint8_t * data, size_t len, size_t * discarded)
{
  if (discarded) {
    *discarded = 0;
  }
  if (master_ < 0) {
    return false;
  }
  // The driver keeps the slave open, so what the client did not read (or everything, while no
  // client runs) stays in the slave's input queue: drop it rather than hand the client a backlog.
  // Flushed unconditionally: the count may miss bytes the kernel has not moved to the queue yet.
  int unread = 0;
  if (discarded && ::ioctl(slave_, FIONREAD, &unread) == 0 && unread > 0) {
    *discarded = static_cast<size_t>(unread);
  }
  ::tcflush(slave_, TCIFLUSH);
  const ssize_t n = ::write(master_, data, len);
  if (n < 0) {
    if (errno == EAGAIN || errno == EINTR || errno == EIO) {
      return false;
    }
    throw_errno("write " + slave_path_ + " (master)");
  }
  if (static_cast<size_t>(n) != len) {
    ::tcflush(slave_, TCIFLUSH);  // no partial message left behind
    return false;
  }
  return true;
}

}  // namespace ublox_dgnss
