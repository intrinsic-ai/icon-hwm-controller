// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "fanuc_msgs/msg/robot_status.hpp"
#include "fanuc_msgs/srv/read_error.hpp"
#include "fanuc_msgs/srv/reset.hpp"
#include "fanuc_msgs/srv/switch_control_state.hpp"
#include "icon/utils/attributes.h"
#include "icon/utils/mutex.h"
#include "icon/utils/status.h"
#include "icon_hwm_controller_msgs/msg/operational_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace fanuc_ros2_icon_hwm
{

// Evaluates the `OperationalStatus` for ICON from the robot state published by
// the FANUC ROS 2 driver. The expected states are the following:
// - Normal operation:
//     in_error=false
//     tp_enabled=false
//     e_stopped=false
//     motion_possible=true
//     contact_stop_mode=0
// - E-stopped (In error):
//     in_error=true
//     tp_enabled=false
//     e_stopped=true
//     motion_possible=false
//     contact_stop_mode=0
// - E-stop released (Unacknowledged / Alarms active):
//     in_error=true
//     tp_enabled=false
//     e_stopped=false
//     motion_possible=false
//     contact_stop_mode=0
// - Teach pendant enabled (Manual control):
//     tp_enabled=true
// - Contact stop active:
//     contact_stop_mode=2
icon_hwm_controller_msgs::msg::OperationalStatus ToOperationalStatus(
  const std::optional<fanuc_msgs::msg::RobotStatus> & robot_status);

// Monitors FANUC driver topics, publishes `OperationalStatus` and handles
// `clear_faults` requests.
class FanucOperationalStateNode : public rclcpp::Node
{
public:
  explicit FanucOperationalStateNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~FanucOperationalStateNode() = default;

  // Used to retrieve the current operational status in tests.
  icon_hwm_controller_msgs::msg::OperationalStatus GetOperationalStatus() const;

  // Publishes the current operational status.
  void PublishOperationalStatus();

  // Clear the faults on the controller and regain motion control.
  intrinsic::Status ClearFaults();

private:
  void ClearFaultsCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void RobotStatusCallback(const fanuc_msgs::msg::RobotStatus::SharedPtr msg);

  mutable intrinsic::Mutex state_mutex_;
  icon_hwm_controller_msgs::msg::OperationalStatus operational_status_
  INTR_GUARDED_BY(state_mutex_);

  // Callback groups for concurrency.
  rclcpp::CallbackGroup::SharedPtr topic_cb_group_;
  rclcpp::CallbackGroup::SharedPtr clear_faults_service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  rclcpp::Subscription<fanuc_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Publisher<icon_hwm_controller_msgs::msg::OperationalStatus>::SharedPtr
    operational_status_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_faults_srv_;
  rclcpp::Client<fanuc_msgs::srv::Reset>::SharedPtr reset_client_;
  rclcpp::Client<fanuc_msgs::srv::SwitchControlState>::SharedPtr switch_control_state_client_;
  rclcpp::Client<fanuc_msgs::srv::ReadError>::SharedPtr read_error_client_;

  std::chrono::seconds service_timeout_;
};

}  // namespace fanuc_ros2_icon_hwm
