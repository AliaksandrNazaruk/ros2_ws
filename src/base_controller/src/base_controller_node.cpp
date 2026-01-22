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
#include <algorithm>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <nlohmann/json.hpp>

#include "aehub_msgs/msg/robot_status.hpp"

#include "base_controller/async_driver_client.hpp"
#include "base_controller/symovo_json_extract.hpp"
#include "base_controller/symovo_safety_mapping.hpp"

using namespace std::chrono_literals;

/**
 * @brief Base Controller Node - Thin Adapter for AE.HUB MVP
 * 
 * AE.HUB MVP requirement:
 * - Nav2 is the SINGLE source of motion commands
 * - base_controller = thin adapter only
 * - NO arbitration, NO decision making, NO navigation logic
 * 
 * Responsibilities:
 * - HTTP adapter (Symovo API)
 * - odom publisher
 * - tf publisher (odom -> base_link)
 * - cmd_vel subscriber (from Nav2 only)
 */
class BaseControllerNode : public rclcpp::Node
{
public:
  BaseControllerNode()
  : Node("base_controller")
  {
    // Check for duplicate instances BEFORE initialization - CRITICAL: prevent duplicate nodes
    const char* lock_file_path = "/tmp/base_controller.lock";
    lock_fd_ = open(lock_file_path, O_CREAT | O_WRONLY | O_TRUNC, 0644);
    
    if (lock_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), 
        "CRITICAL ERROR: Failed to open lock file: %s", 
        strerror(errno));
      // Continue anyway, but log the error
    } else {
      // Try to acquire non-blocking exclusive lock
      struct flock lock;
      lock.l_type = F_WRLCK;
      lock.l_whence = SEEK_SET;
      lock.l_start = 0;
      lock.l_len = 0;
      
      if (fcntl(lock_fd_, F_SETLK, &lock) < 0) {
        // Lock is held by another process - duplicate detected!
        close(lock_fd_);
        lock_fd_ = -1;
        
        RCLCPP_ERROR(get_logger(),
          "CRITICAL ERROR: base_controller is already running!\n"
          "Another instance is holding the lock file.\n"
          "This will cause topic conflicts and unpredictable behavior.\n"
          "Only one instance should be running.\n"
          "Please stop the existing instance before starting a new one.\n"
          "Lock file: %s", lock_file_path);
        
        // Exit with error code
        std::exit(1);
      }
      
      // Lock acquired successfully - write PID to file
      char pid_str[32];
      snprintf(pid_str, sizeof(pid_str), "%d\n", getpid());
      ssize_t written = write(lock_fd_, pid_str, strlen(pid_str));
      if (written < 0) {
        RCLCPP_WARN(get_logger(), "Failed to write PID to lock file: %s", strerror(errno));
      }
      fsync(lock_fd_);
      
      RCLCPP_INFO(get_logger(), 
        "Lock file acquired successfully (PID: %d)", getpid());
    }
    // Parameters
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 20.0);
    cmd_vel_timeout_ms_ = this->declare_parameter<int>("cmd_vel_timeout_ms", 200);
    max_linear_mps_ = this->declare_parameter<double>("max_linear_mps", 0.6);
    max_angular_rps_ = this->declare_parameter<double>("max_angular_rps", 1.2);
    driver_endpoint_ = this->declare_parameter<std::string>(
        "driver_endpoint", "https://192.168.1.100");
    amr_id_ = this->declare_parameter<int>("amr_id", 15);
    move_duration_s_ = this->declare_parameter<double>("move_duration_s", 0.05);
    tls_verify_ = this->declare_parameter<bool>("tls_verify", true);

    // Frames (odom + TF)
    frame_prefix_ = this->declare_parameter<std::string>("frame_prefix", "");
    odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_link");
    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);

    // Robot status mirror
    robot_status_topic_ = this->declare_parameter<std::string>("robot_status_topic", "robot/status");
    robot_status_publish_hz_ = this->declare_parameter<double>("robot_status_publish_hz", 5.0);
    symovo_raw_json_log_enabled_ = this->declare_parameter<bool>("symovo_raw_json_log_enabled", true);
    symovo_raw_json_log_period_s_ = this->declare_parameter<double>("symovo_raw_json_log_period_s", 5.0);

    // Symovo safety field mapping (configurable; JSON-pointer or dot-path)
    // Examples:
    // - JSON pointer: /safety/motors_enabled
    // - Dot path: safety.motors_enabled
    symovo_motors_enabled_path_ =
      this->declare_parameter<std::string>("symovo_motors_enabled_path", "");
    symovo_estop_active_path_ =
      this->declare_parameter<std::string>("symovo_estop_active_path", "");
    symovo_motors_enabled_invert_ =
      this->declare_parameter<bool>("symovo_motors_enabled_invert", false);
    symovo_estop_active_invert_ =
      this->declare_parameter<bool>("symovo_estop_active_invert", false);
    symovo_status_json_required_ =
      this->declare_parameter<bool>("symovo_status_json_required", false);

    // Log TLS configuration
    if (tls_verify_) {
      RCLCPP_INFO(get_logger(), "✅ Secure TLS mode enabled (tls_verify=true)");
    } else {
      RCLCPP_WARN(get_logger(),
        "⚠️  INSECURE TLS MODE: tls_verify=false. "
        "This is unsafe for production! Set tls_verify:=true for secure connections.");
    }

    RCLCPP_INFO(get_logger(),
      "BaseController (AE.HUB MVP mode): %.1f Hz, timeout=%d ms",
      control_rate_hz_, cmd_vel_timeout_ms_);
    RCLCPP_INFO(get_logger(),
      "Nav2 is the SINGLE source of motion commands");

    // Create AsyncDriverClient for Symovo API
    // AE.HUB MVP: Thin adapter - only HTTP communication
    driver_client_ = std::make_unique<base_controller::AsyncDriverClient>(driver_endpoint_, amr_id_, tls_verify_);

    // Subscribe to cmd_vel from capability/Nav2 (relative for /robot/<id>/... namespace)
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      rclcpp::QoS(10),
      std::bind(&BaseControllerNode::cmdVelCallback, this, std::placeholders::_1));

    // Publisher for odometry
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Publisher for typed robot status mirror (readiness/health consumers)
    robot_status_pub_ = this->create_publisher<aehub_msgs::msg::RobotStatus>(
      robot_status_topic_, rclcpp::QoS(1).transient_local());

    // TF broadcaster for odom -> base_link
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // Control loop timer
    control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz_),
      std::bind(&BaseControllerNode::controlLoop, this));

    if (robot_status_publish_hz_ > 0.0) {
      robot_status_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / robot_status_publish_hz_),
        std::bind(&BaseControllerNode::publishRobotStatus, this));
    }
  }

  // Destructor to release lock (must be public for std::make_shared)
  ~BaseControllerNode()
  {
    if (lock_fd_ >= 0) {
      struct flock lock;
      lock.l_type = F_UNLCK;
      lock.l_whence = SEEK_SET;
      lock.l_start = 0;
      lock.l_len = 0;
      fcntl(lock_fd_, F_SETLK, &lock);
      close(lock_fd_);
      RCLCPP_INFO(get_logger(), "Lock file released");
    }
  }

private:
  std::string resolveFrame(const std::string & frame) const
  {
    if (frame_prefix_.empty()) {
      return frame;
    }
    if (frame.empty()) {
      return frame;
    }
    if (frame_prefix_.back() == '/') {
      return frame_prefix_ + frame;
    }
    return frame_prefix_ + "/" + frame;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // AE.HUB MVP: Nav2 is the ONLY source
    // No arbitration, no priority checking
    last_cmd_vel_ = *msg;
    last_cmd_time_ = this->now();
  }

  void controlLoop()
  {
    auto now = this->now();
    double age_ms = (now - last_cmd_time_).seconds() * 1000.0;

    geometry_msgs::msg::Twist cmd;

    // Watchdog: stop if no commands received
    if (age_ms > cmd_vel_timeout_ms_) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    } else {
      // Clamp velocities to safety limits
      cmd.linear.x = std::clamp(
        last_cmd_vel_.linear.x,
        -max_linear_mps_,
        max_linear_mps_);
      cmd.angular.z = std::clamp(
        last_cmd_vel_.angular.z,
        -max_angular_rps_,
        max_angular_rps_);
    }

    // Send command to Symovo API via AsyncDriverClient
    driver_client_->submit(cmd);

    // Publish odometry and TF from Symovo API
    publishOdometry();
  }

  void publishRobotStatus()
  {
    if (!robot_status_pub_) {
      return;
    }

    aehub_msgs::msg::RobotStatus st;
    st.stamp = this->now();
    st.source = "symovo";
    st.driver_connected = driver_client_ && driver_client_->isHealthy();

    const auto pose = driver_client_ ? driver_client_->getPose() : std::nullopt;
    const auto vel = driver_client_ ? driver_client_->getVelocity() : std::nullopt;
    st.pose_valid = pose.has_value();
    st.velocity_valid = vel.has_value();

    // Safety fields are mapped from Symovo JSON (configurable). Defaults to invalid.
    st.motors_enabled_valid = false;
    st.motors_enabled = false;
    st.estop_active_valid = false;
    st.estop_active = false;

    if (driver_client_) {
      base_controller::SafetyMappingConfig cfg;
      cfg.motors_enabled_path = symovo_motors_enabled_path_;
      cfg.estop_active_path = symovo_estop_active_path_;
      cfg.motors_enabled_invert = symovo_motors_enabled_invert_;
      cfg.estop_active_invert = symovo_estop_active_invert_;
      cfg.status_json_required = symovo_status_json_required_;

      const auto raw = driver_client_->getLastRawAgvJson();
      const auto r = base_controller::mapSafetyFromRawSymovoJson(raw, cfg);

      st.motors_enabled_valid = r.motors_enabled_valid;
      st.motors_enabled = r.motors_enabled;
      st.estop_active_valid = r.estop_active_valid;
      st.estop_active = r.estop_active;

      if (!r.ok) {
        // Per policy: if JSON mapping is required but unavailable, mark as disconnected.
        st.driver_connected = false;
        st.details = "safety_map_error:" + r.error;
      }
    } else if (symovo_status_json_required_) {
      // Per policy: if JSON mapping is required but unavailable, mark as disconnected.
      st.driver_connected = false;
      st.details = "safety_map_error:driver_client_null";
    }

    // Debug: periodically log raw JSON to discover field names for safety mapping.
    if (symovo_raw_json_log_enabled_ && driver_client_) {
      const auto now_steady = std::chrono::steady_clock::now();
      if (now_steady - last_raw_json_log_time_ >= std::chrono::duration<double>(symovo_raw_json_log_period_s_)) {
        last_raw_json_log_time_ = now_steady;
        const auto raw = driver_client_->getLastRawAgvJson();
        if (!raw.empty()) {
          RCLCPP_INFO(get_logger(), "Symovo raw AMR JSON (amr_id=%d): %s", amr_id_, raw.c_str());
          if (st.details.empty()) {
            st.details = "raw_json_logged";
          } else {
            st.details += ";raw_json_logged";
          }
        } else {
          if (st.details.empty()) {
            st.details = "raw_json_not_available_yet";
          } else {
            st.details += ";raw_json_not_available_yet";
          }
        }
      }
    }

    robot_status_pub_->publish(st);
  }

  void publishOdometry()
  {
    // Get pose and velocity from AsyncDriverClient
    auto pose = driver_client_->getPose();
    auto velocity = driver_client_->getVelocity();

    if (!pose.has_value() || !velocity.has_value()) {
      // No data available yet, skip publication
      return;
    }

    auto now = this->now();

    // Create Odometry message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = resolveFrame(odom_frame_id_);
    odom_msg.child_frame_id = resolveFrame(base_frame_id_);

    // Set pose
    odom_msg.pose.pose.position.x = pose->x;
    odom_msg.pose.pose.position.y = pose->y;
    odom_msg.pose.pose.position.z = 0.0;

    // Convert theta to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, pose->theta);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    // Set velocity
    odom_msg.twist.twist.linear.x = velocity->x;
    odom_msg.twist.twist.linear.y = velocity->y;
    odom_msg.twist.twist.angular.z = velocity->theta;

    // Publish odometry
    odom_pub_->publish(odom_msg);

    // Publish TF: odom -> base_link
    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = now;
      transform.header.frame_id = odom_msg.header.frame_id;
      transform.child_frame_id = odom_msg.child_frame_id;

      transform.transform.translation.x = pose->x;
      transform.transform.translation.y = pose->y;
      transform.transform.translation.z = 0.0;

      transform.transform.rotation = odom_msg.pose.pose.orientation;

      tf_broadcaster_->sendTransform(transform);
    }
  }

private:
  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<aehub_msgs::msg::RobotStatus>::SharedPtr robot_status_pub_;
  
  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr robot_status_timer_;

  // Command state
  geometry_msgs::msg::Twist last_cmd_vel_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  // Parameters
  double control_rate_hz_;
  int cmd_vel_timeout_ms_;
  double max_linear_mps_;
  double max_angular_rps_;
  std::string driver_endpoint_;
  int amr_id_;
  double move_duration_s_;
  bool tls_verify_;
  std::string frame_prefix_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  bool publish_tf_{true};
  std::string robot_status_topic_;
  double robot_status_publish_hz_;
  bool symovo_raw_json_log_enabled_;
  double symovo_raw_json_log_period_s_;
  std::chrono::steady_clock::time_point last_raw_json_log_time_{std::chrono::steady_clock::now()};

  // Symovo mapping configuration (safety state)
  std::string symovo_motors_enabled_path_;
  std::string symovo_estop_active_path_;
  bool symovo_motors_enabled_invert_{false};
  bool symovo_estop_active_invert_{false};
  bool symovo_status_json_required_{false};

  // Driver client
  std::unique_ptr<base_controller::AsyncDriverClient> driver_client_;
  
  // Lock file descriptor for duplicate prevention
  int lock_fd_ = -1;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseControllerNode>());
  rclcpp::shutdown();
  return 0;
}

