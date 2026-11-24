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

// Pseudo-terminal that stands in for the receiver's serial port, so an NTRIP client that talks
// to a serial device (e.g. rtv_ntrip_bridge for VRS casters) can run beside this driver, which
// owns the USB device: the client opens the slave side (through a stable symlink) as if it were
// the receiver; the driver writes the receiver's GGA to the master and forwards whatever the
// client writes (RTCM 3) to the receiver.

#ifndef UBLOX_DGNSS_NODE__PTY_LINK_HPP_
#define UBLOX_DGNSS_NODE__PTY_LINK_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace ublox_dgnss
{

class PtyLink
{
public:
  PtyLink() = default;
  ~PtyLink();
  PtyLink(const PtyLink &) = delete;
  PtyLink & operator=(const PtyLink &) = delete;

  // Creates the pty (raw, no echo) and the symlink `link_path` -> /dev/pts/N. An existing symlink
  // at `link_path` is replaced (left over by a crashed driver); any other file there is an error.
  // Throws std::system_error / std::runtime_error.
  void open(const std::string & link_path);
  // Closes the pty and removes the symlink if it is still ours.
  void close();
  bool is_open() const {return master_ >= 0;}

  // Non-blocking: what the client wrote (RTCM), 0 if nothing.
  size_t read(uint8_t * buf, size_t len);
  // Non-blocking, latest wins: whatever the client has not read yet is discarded first (an old
  // GGA is worthless once a newer one exists), so the client reads the newest message, whole.
  // Returns false if `data` did not fit; then none of it stays queued (no fragment for the
  // client to glue to the next message). `discarded` gets the number of unread bytes thrown away.
  bool write(const uint8_t * data, size_t len, size_t * discarded = nullptr);

  const std::string & slave_path() const {return slave_path_;}
  const std::string & link_path() const {return link_path_;}

private:
  int master_{-1};
  int slave_{-1};  // kept open: without any slave open the master reports hangup in a loop
  std::string slave_path_;
  std::string link_path_;
};

}  // namespace ublox_dgnss

#endif  // UBLOX_DGNSS_NODE__PTY_LINK_HPP_
