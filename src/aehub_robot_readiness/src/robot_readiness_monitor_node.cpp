// Copyright 2026 Boris
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

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <rclcpp/rclcpp.hpp>

#include "aehub_robot_readiness/robot_readiness_gate.hpp"

namespace aehub::robot
{
namespace
{

diagnostic_msgs::msg::DiagnosticStatus toDiag(const aehub::nav2::ReadinessReport & report)
{
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "aehub_robot_readiness";
  st.hardware_id = "";

  const bool ok = report.isReady();
  st.level =
    ok ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  st.message = ok ? "ready" : ("not_ready: " + report.summary);

  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "summary";
  kv.value = report.summary;
  st.values.push_back(kv);

  diagnostic_msgs::msg::KeyValue kv2;
  kv2.key = "overall_level";
  kv2.value = ok ? "READY" : "NOT_READY";
  st.values.push_back(kv2);

  return st;
}

}  // namespace

// Thin ROS adapter: periodically evaluates RobotReadinessGate and publishes health.
// SRS invariant: the gate is observational only; this node publishes telemetry only.
class RobotReadinessMonitorNode final : public rclcpp::Node
{
public:
  RobotReadinessMonitorNode()
  : rclcpp::Node("robot_readiness_monitor")
  {
    publish_period_s_ = declare_parameter<double>("publish_period_s", 0.5);
    health_topic_ = declare_parameter<std::string>("health_topic", "health/robot_readiness");
    log_on_change_ = declare_parameter<bool>("log_on_change", true);
  }

  void initialize()
  {
    if (initialized_) {
      return;
    }
    initialized_ = true;

    // IMPORTANT: shared_from_this() is not valid inside the constructor. This must be called
    // only after the node is owned by a std::shared_ptr.
    gate_ = std::make_unique<RobotReadinessGate>(shared_from_this());

    health_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      health_topic_, rclcpp::QoS(1).transient_local());

    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(publish_period_s_));
    timer_ = create_wall_timer(period_ns, [this]() {this->tick();});

    RCLCPP_INFO(get_logger(), "RobotReadinessMonitor started (health_topic=%s)",
      health_topic_.c_str());
  }

private:
  void tick()
  {
    if (!health_pub_ || !gate_) {
      return;
    }

    const auto report = gate_->check();

    if (log_on_change_) {
      const bool ready = report.isReady();
      if (!last_ready_.has_value() || *last_ready_ != ready) {
        last_ready_ = ready;
        RCLCPP_INFO(get_logger(), "RobotReadiness changed: %s (%s)",
          ready ? "READY" : "NOT_READY",
          report.summary.c_str());
      }
    }

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();
    arr.status.push_back(toDiag(report));
    health_pub_->publish(arr);
  }

  double publish_period_s_{0.5};
  std::string health_topic_;
  bool log_on_change_{true};

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<RobotReadinessGate> gate_;

  std::optional<bool> last_ready_;
  bool initialized_{false};
};

}  // namespace aehub::robot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aehub::robot::RobotReadinessMonitorNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
