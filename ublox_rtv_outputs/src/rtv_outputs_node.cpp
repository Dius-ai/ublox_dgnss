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

// rtv_outputs: subscribes to the ublox_dgnss NAV / RXM topics, groups them into navigation epochs
// and publishes the outputs of the rtv_gnss ZED-F9P driver: ~/navsatfix, ~/gpsfix, ~/twist,
// ~/time_reference, ~/rtk_status, and /diagnostics.
//
// All callbacks are in the node's default (mutually exclusive) callback group: no locking.

#include <chrono>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "ublox_rtv_outputs/conversions.hpp"
#include "ublox_rtv_outputs/epoch_assembler.hpp"
#include "ublox_ubx_msgs/msg/spoof_det.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_eoe.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_status.hpp"
#include "ublox_ubx_msgs/msg/ubx_rxm_rtcm.hpp"

namespace ublox_rtv_outputs
{
using Clock = std::chrono::steady_clock;
using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
using rtv_gnss_utils::msg::RtkStatus;
namespace ubx = ublox_ubx_msgs::msg;

namespace
{
int64_t steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    Clock::now().time_since_epoch()).count();
}

double seconds_since(const std::optional<Clock::time_point> & t)
{
  return t ? std::chrono::duration<double>(Clock::now() - *t).count() : -1.0;
}

std::string fmt(double v, int precision = 3)
{
  char b[32];
  std::snprintf(b, sizeof(b), "%.*f", precision, v);
  return b;
}
}  // namespace

class RtvOutputsNode : public rclcpp::Node
{
public:
  explicit RtvOutputsNode(const rclcpp::NodeOptions & options)
  : Node("rtv_outputs", options), diag_(this)
  {
    frame_id_ = declare_parameter("frame_id", "");
    velocity_frame_id_ = declare_parameter("velocity_frame_id", "enu");
    use_gnss_time_ = declare_parameter("use_gnss_time", false);
    time_source_ = declare_parameter("time_reference_source", "ublox");
    max_correction_age_s_ = declare_parameter("max_correction_age_s", 10.0);
    max_epoch_gap_s_ = declare_parameter("max_epoch_gap_s", 2.0);
    const double timeout_s = declare_parameter("epoch_timeout_s", 0.1);
    assembler_ = EpochAssembler(static_cast<int64_t>(timeout_s * 1e9));

    const auto qos = rclcpp::SensorDataQoS().keep_last(10);
    if (declare_parameter("publish.navsatfix", true)) {
      navsatfix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("~/navsatfix", qos);
    }
    if (declare_parameter("publish.gpsfix", true)) {
      gpsfix_pub_ = create_publisher<gps_msgs::msg::GPSFix>("~/gpsfix", qos);
    }
    if (declare_parameter("publish.twist", true)) {
      twist_pub_ =
        create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("~/twist", qos);
    }
    if (declare_parameter("publish.time_reference", true)) {
      time_ref_pub_ = create_publisher<sensor_msgs::msg::TimeReference>("~/time_reference", qos);
    }
    if (declare_parameter("publish.rtk_status", true)) {
      rtk_status_pub_ = create_publisher<RtkStatus>("~/rtk_status", qos);
    }

    // ublox_dgnss publishes reliable; keep a few epochs in case this node falls behind briefly
    const auto in_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    pvt_sub_ = create_subscription<ubx::UBXNavPVT>(
      "ubx_nav_pvt", in_qos, [this](const ubx::UBXNavPVT & m) {
        publish(assembler_.add(m, steady_ns()));
      });
    hp_sub_ = create_subscription<ubx::UBXNavHPPosLLH>(
      "ubx_nav_hp_pos_llh", in_qos, [this](const ubx::UBXNavHPPosLLH & m) {
        publish(assembler_.add(m, steady_ns()));
      });
    cov_sub_ = create_subscription<ubx::UBXNavCov>(
      "ubx_nav_cov", in_qos, [this](const ubx::UBXNavCov & m) {
        publish(assembler_.add(m, steady_ns()));
      });
    dop_sub_ = create_subscription<ubx::UBXNavDOP>(
      "ubx_nav_dop", in_qos, [this](const ubx::UBXNavDOP & m) {
        publish(assembler_.add(m, steady_ns()));
      });
    eoe_sub_ = create_subscription<ubx::UBXNavEOE>(
      "ubx_nav_eoe", in_qos, [this](const ubx::UBXNavEOE & m) {
        publish(assembler_.add_eoe(m.itow, m.header.stamp, steady_ns()));
      });
    sat_sub_ = create_subscription<ubx::UBXNavSat>(
      "ubx_nav_sat", in_qos, [this](const ubx::UBXNavSat & m) {last_sat_ = m;});
    status_sub_ = create_subscription<ubx::UBXNavStatus>(
      "ubx_nav_status", in_qos, [this](const ubx::UBXNavStatus & m) {nav_status_ = m;});
    rtcm_sub_ = create_subscription<ubx::UBXRxmRTCM>(
      "ubx_rxm_rtcm", in_qos, [this](const ubx::UBXRxmRTCM & m) {on_rxm_rtcm(m);});
    flush_timer_ = create_wall_timer(
      std::chrono::milliseconds(20), [this] {publish(assembler_.flush(steady_ns()));});

    diag_.setHardwareID("u-blox (ublox_dgnss)");
    diag_.add("navigation", this, &RtvOutputsNode::diag_navigation);
    diag_.add("position", this, &RtvOutputsNode::diag_position);
    diag_.add("corrections", this, &RtvOutputsNode::diag_corrections);

    RCLCPP_INFO(
      get_logger(), "epochs from %s + %s, %s, %s; closed by %s", pvt_sub_->get_topic_name(),
      hp_sub_->get_topic_name(), cov_sub_->get_topic_name(), dop_sub_->get_topic_name(),
      eoe_sub_->get_topic_name());
  }

private:
  void on_rxm_rtcm(const ubx::UBXRxmRTCM & m)
  {
    if (m.crc_failed) {
      ++rtcm_crc_failed_;
    } else if (m.msg_used == 2) {
      ++rtcm_used_;
    } else if (m.msg_used == 1) {
      ++rtcm_not_used_;
    }
    if (m.ref_station != 0xFFFF) {  // 0xFFFF: message without a station (e.g. 1021)
      ref_station_ = m.ref_station;
    }
    last_rtcm_type_ = m.msg_type;
    last_rtcm_ = Clock::now();
  }

  void publish(const std::vector<Epoch> & epochs)
  {
    for (const auto & e : epochs) {
      publish_epoch(e);
    }
  }

  void publish_epoch(const Epoch & e)
  {
    OutputContext ctx;
    ctx.frame_id = frame_id_.empty() ? e.pvt->header.frame_id : frame_id_;
    ctx.velocity_frame_id = velocity_frame_id_;
    ctx.time_source = time_source_;
    ctx.sat = last_sat_ ? &*last_sat_ : nullptr;
    const auto utc = utc_ns(*e.pvt);
    ctx.stamp = use_gnss_time_ && utc ? to_time_msg(*utc) : e.rx_stamp;

    if (navsatfix_pub_) {
      navsatfix_pub_->publish(*to_navsatfix(e, ctx));
    }
    if (gpsfix_pub_) {
      gpsfix_pub_->publish(*to_gpsfix(e, ctx));
    }
    if (twist_pub_) {
      twist_pub_->publish(*to_twist(e, ctx));
    }
    if (rtk_status_pub_) {
      CorrectionInfo c;
      c.reference_station = ref_station_;
      c.used = rtcm_used_;
      c.not_used = rtcm_not_used_;
      c.crc_failed = rtcm_crc_failed_;
      if (last_rtcm_) {
        c.seconds_since_last_rtcm = seconds_since(last_rtcm_);
      }
      rtk_status_pub_->publish(*to_rtk_status(e, ctx, c));
    }
    if (time_ref_pub_) {
      OutputContext rx = ctx;
      rx.stamp = e.rx_stamp;  // host time vs GNSS time, whatever stamp the rest uses
      if (auto m = to_time_reference(e, rx)) {
        time_ref_pub_->publish(*m);
      }
    }

    const FixQuality q = fix_quality(*e.pvt);
    if (q != last_quality_) {
      RCLCPP_INFO(
        get_logger(), "fix: %s -> %s (%u satellites, hAcc %.3f m)", to_string(last_quality_),
        to_string(q), e.pvt->num_sv, e.pvt->h_acc * 1e-3);
      last_quality_ = q;
    }
    last_epoch_ = Clock::now();
    last_pvt_ = e.pvt;
  }

  void diag_navigation(diagnostic_updater::DiagnosticStatusWrapper & d)
  {
    const AssemblerStats & s = assembler_.stats();
    const auto t = Clock::now();
    double rate = 0.0;
    if (rate_t0_) {
      const double dt = std::chrono::duration<double>(t - *rate_t0_).count();
      rate = dt > 0 ? static_cast<double>(s.epochs - rate_epochs0_) / dt : 0.0;
    }
    rate_t0_ = t;
    rate_epochs0_ = s.epochs;

    d.add("epoch rate (Hz)", fmt(rate, 2));
    d.add("epochs", s.epochs);
    d.add("epochs closed without NAV-EOE", s.epochs_without_eoe);
    d.add("epochs dropped without NAV-PVT", s.epochs_without_pvt);
    d.add("messages after their epoch was published", s.late_messages);
    d.add("receiver time jumps back", s.time_jumps);  // TODO(review): new diagnostic
    const double age = seconds_since(last_epoch_);
    if (age < 0) {
      d.summary(DiagStatus::ERROR, "no navigation epochs yet from " + std::string(
          pvt_sub_->get_topic_name()));
    } else if (age > max_epoch_gap_s_) {
      d.summary(DiagStatus::ERROR, "no navigation epochs for " + fmt(age, 1) + " s");
    } else {
      d.summary(DiagStatus::OK, "OK");
    }
  }

  void diag_position(diagnostic_updater::DiagnosticStatusWrapper & d)
  {
    if (!last_pvt_) {
      d.summary(DiagStatus::WARN, "no navigation solution yet");
      return;
    }
    const ubx::UBXNavPVT & p = *last_pvt_;
    const FixQuality q = fix_quality(p);
    d.add("fix", to_string(q));
    d.add("fix type", static_cast<int>(p.gps_fix.fix_type));
    d.add("satellites used", static_cast<int>(p.num_sv));
    d.add("hAcc (m)", fmt(p.h_acc * 1e-3));
    d.add("vAcc (m)", fmt(p.v_acc * 1e-3));
    d.add("pDOP", fmt(p.p_dop * 0.01, 2));
    const double age = correction_age_upper_s(p.last_correction_age);
    d.add(
      "correction age (s)", p.last_correction_age == 0 ? std::string("none") : "< " + fmt(age, 0));
    if (nav_status_) {
      d.add("time to first fix (s)", fmt(nav_status_->ttff * 1e-3, 1));
    }
    const bool spoofing = nav_status_ &&
      nav_status_->spoof_det.state >= ubx::SpoofDet::SPOOF_DET_SPOOFING;
    if (spoofing) {
      d.summary(DiagStatus::WARN, "spoofing indicated");
    } else if (q == FixQuality::kRtkFixed) {
      d.summary(DiagStatus::OK, "RTK FIXED");
    } else {
      d.summary(DiagStatus::WARN, std::string(to_string(q)) + " (not RTK fixed)");
    }
  }

  void diag_corrections(diagnostic_updater::DiagnosticStatusWrapper & d)
  {
    const double age = seconds_since(last_rtcm_);
    d.add("last RTCM at the receiver (s ago)", age < 0 ? std::string("never") : fmt(age, 1));
    d.add("RTCM messages used", rtcm_used_);
    d.add("RTCM messages not used", rtcm_not_used_);
    d.add("RTCM CRC failures", rtcm_crc_failed_);
    d.add("last RTCM message type", static_cast<int>(last_rtcm_type_));
    d.add(
      "reference station", ref_station_ < 0 ? std::string("unknown") : std::to_string(
        ref_station_));
    if (age < 0) {
      d.summary(DiagStatus::WARN, "no corrections at the receiver yet (UBX-RXM-RTCM)");
    } else if (age > max_correction_age_s_) {
      d.summary(DiagStatus::WARN, "no corrections for " + fmt(age, 0) + " s");
    } else if (rtcm_used_ == 0) {
      d.summary(DiagStatus::WARN, "corrections arrive but the receiver uses none");
    } else {
      d.summary(DiagStatus::OK, "corrections used");
    }
  }

  // parameters
  std::string frame_id_, velocity_frame_id_, time_source_;
  bool use_gnss_time_{false};
  double max_correction_age_s_{10.0};
  double max_epoch_gap_s_{2.0};

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_pub_;
  rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gpsfix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr time_ref_pub_;
  rclcpp::Publisher<RtkStatus>::SharedPtr rtk_status_pub_;
  rclcpp::Subscription<ubx::UBXNavPVT>::SharedPtr pvt_sub_;
  rclcpp::Subscription<ubx::UBXNavHPPosLLH>::SharedPtr hp_sub_;
  rclcpp::Subscription<ubx::UBXNavCov>::SharedPtr cov_sub_;
  rclcpp::Subscription<ubx::UBXNavDOP>::SharedPtr dop_sub_;
  rclcpp::Subscription<ubx::UBXNavEOE>::SharedPtr eoe_sub_;
  rclcpp::Subscription<ubx::UBXNavSat>::SharedPtr sat_sub_;
  rclcpp::Subscription<ubx::UBXNavStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<ubx::UBXRxmRTCM>::SharedPtr rtcm_sub_;
  rclcpp::TimerBase::SharedPtr flush_timer_;
  diagnostic_updater::Updater diag_;

  EpochAssembler assembler_;
  std::optional<ubx::UBXNavSat> last_sat_;
  std::optional<ubx::UBXNavStatus> nav_status_;
  std::optional<ubx::UBXNavPVT> last_pvt_;
  std::optional<Clock::time_point> last_epoch_;
  FixQuality last_quality_{FixQuality::kNoFix};
  // corrections, from UBX-RXM-RTCM
  uint32_t rtcm_used_{0}, rtcm_not_used_{0}, rtcm_crc_failed_{0};
  int32_t ref_station_{-1};
  uint16_t last_rtcm_type_{0};
  std::optional<Clock::time_point> last_rtcm_;
  // diagnostics rate measurement
  std::optional<Clock::time_point> rate_t0_;
  uint64_t rate_epochs0_{0};
};

}  // namespace ublox_rtv_outputs

RCLCPP_COMPONENTS_REGISTER_NODE(ublox_rtv_outputs::RtvOutputsNode)
