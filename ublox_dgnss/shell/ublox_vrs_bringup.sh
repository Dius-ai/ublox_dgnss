#!/usr/bin/env bash
# ZED-F9P with SKPOS VRS corrections: ublox_dgnss (owns the USB device, serves a pty link) +
# rtv_gnss_ntrip_client (rtv_ntrip_bridge on that pty, ntrip_client_node reporting on it).
#
#   ./ublox_vrs_bringup.sh                 # Ctrl+C stops everything
#
# Environment overrides (defaults in brackets):
#   PTY_LINK      pty symlink for the bridge            [$XDG_RUNTIME_DIR/ublox_dgnss]
#   SKPOS_ENV     file exporting SKPOS_USER, SKPOS_PASS [~/.skpos_env]
#   BRIDGE_CONFIG bridge TOML template                  [<this dir>/ntrip_skpos_vrs.toml]
#   ROS_WS        workspace with install/               [~/ros2_ws]
#   FAKE_POSITION LAT,LON,HEIGHT sent instead of the receiver's GGA (garage tests)  [unset]
#   LOG_LEVEL     ROS log level                         [info]
#   DRIVER_ARGS   extra ublox_rover_vrs.launch.py args, e.g. "rate_meas_ms:=100 one_hz_divider:=10"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PTY_LINK="${PTY_LINK:-${XDG_RUNTIME_DIR:-/tmp}/ublox_dgnss}"
SKPOS_ENV="${SKPOS_ENV:-${HOME}/.skpos_env}"
BRIDGE_CONFIG="${BRIDGE_CONFIG:-${script_dir}/ntrip_skpos_vrs.toml}"
ROS_WS="${ROS_WS:-${HOME}/ros2_ws}"
LOG_LEVEL="${LOG_LEVEL:-info}"
read -r -a driver_args <<< "${DRIVER_ARGS:-}"

die() { echo "ublox_vrs_bringup: $*" >&2; exit 1; }

[[ -f "${BRIDGE_CONFIG}" ]] || die "bridge config ${BRIDGE_CONFIG} not found"
[[ -r "${SKPOS_ENV}" ]] || die "credentials file ${SKPOS_ENV} not readable"

# ROS setup scripts do not work with set -u.
source /opt/ros/jazzy/setup.bash || die "ROS 2 Jazzy not found"
source "${ROS_WS}/install/setup.bash" || die "workspace ${ROS_WS}/install not built"
source "${SKPOS_ENV}"
[[ -n "${SKPOS_USER:-}" && -n "${SKPOS_PASS:-}" ]] || die "${SKPOS_ENV} must export SKPOS_USER and SKPOS_PASS"
export SKPOS_USER SKPOS_PASS

# The bridge has to open the pty the driver serves: write the TOML with serial.device = PTY_LINK.
bridge_config="$(dirname "${PTY_LINK}")/ntrip_skpos_vrs.toml"
sed -E "s|^device[[:space:]]*=.*|device = \"${PTY_LINK}\"|" "${BRIDGE_CONFIG}" > "${bridge_config}" ||
  die "cannot write ${bridge_config}"

ntrip_args=("bridge_config:=${bridge_config}" "log_level:=${LOG_LEVEL}")
if [[ -n "${FAKE_POSITION:-}" ]]; then
  ntrip_args+=("use_fake_rtk_position:=true" "fake_position:=${FAKE_POSITION}")
fi

# Job control: without it, bash starts background jobs with SIGINT ignored and ros2 launch could
# not be stopped. With it, each launch gets its own process group, and stop() sends that group
# SIGINT, as Ctrl+C would. Their stdin is /dev/null: a background job touching the terminal is
# stopped by the kernel.
set -m
pids=()
stop() {
  trap - INT TERM
  echo "ublox_vrs_bringup: stopping"
  for pid in "${pids[@]}"; do kill -INT -- "-${pid}" 2>/dev/null; kill -CONT -- "-${pid}" 2>/dev/null; done
  # with job control, wait also returns when a job is merely stopped
  for pid in "${pids[@]}"; do while kill -0 "${pid}" 2>/dev/null; do wait "${pid}" 2>/dev/null; done; done
  rm -f "${bridge_config}"
  exit 0
}
trap stop INT TERM

# A symlink left by a driver that was killed (SIGKILL, crash) may point at a /dev/pts/N that now
# belongs to another terminal, and the bridge would open that. Remove it unless a running process
# of ours still holds the pty (the driver keeps the slave side open); then any link that appears
# is the new one.
pty_held() {
  local target fd
  target="$(readlink "${PTY_LINK}")" || return 1
  for fd in /proc/[0-9]*/fd/*; do
    [[ "$(readlink "${fd}" 2>/dev/null)" == "${target}" ]] && return 0
  done
  return 1
}
if [[ -L "${PTY_LINK}" ]]; then
  pty_held && die "${PTY_LINK} is served by a running driver; stop it first"
  rm -f "${PTY_LINK}" || die "cannot remove stale ${PTY_LINK}"
fi

echo "ublox_vrs_bringup: pty ${PTY_LINK}, bridge config ${bridge_config}"
ros2 launch ublox_dgnss ublox_rover_vrs.launch.py \
  pty_link:="${PTY_LINK}" log_level:="${LOG_LEVEL}" "${driver_args[@]}" < /dev/null &
pids+=($!)
# The bridge reopens its serial device by itself, but starting after the pty exists avoids
# a first "cannot open" error in its log.
for _ in $(seq 50); do [[ -L "${PTY_LINK}" ]] && break; sleep 0.1; done
ros2 launch rtv_gnss_ntrip_client ntrip_client.launch.xml "${ntrip_args[@]}" < /dev/null &
pids+=($!)

# Either launch ending (e.g. a launch error) stops the other one too.
wait -n "${pids[@]}"
stop
