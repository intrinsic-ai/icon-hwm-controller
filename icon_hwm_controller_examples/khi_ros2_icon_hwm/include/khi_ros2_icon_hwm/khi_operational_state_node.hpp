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

#include "icon/utils/attributes.h"
#include "icon/utils/mutex.h"
#include "icon/utils/status.h"
#include "icon_hwm_controller_msgs/msg/operational_status.hpp"
#include "khi_msgs/msg/error_info.hpp"
#include "khi_msgs/srv/reset_error.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace khi_ros2_icon_hwm
{

// Evaluates the `OperationalStatus` for ICON from the error info published by
// the Kawasaki KHI ROS 2 driver.
icon_hwm_controller_msgs::msg::OperationalStatus ToOperationalStatus(
  const std::optional<khi_msgs::msg::ErrorInfo> & error_info);

// Monitors KHI driver topics, publishes `OperationalStatus` and handles
// `clear_faults` requests by resetting KHI controller errors.
class KhiOperationalStateNode : public rclcpp::Node
{
public:
  explicit KhiOperationalStateNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~KhiOperationalStateNode() = default;

  // Used to retrieve the current operational status in tests.
  icon_hwm_controller_msgs::msg::OperationalStatus GetOperationalStatus() const;

  // Publishes the current operational status.
  void PublishOperationalStatus();

  // Clear the faults on the controller.
  intrinsic::Status ClearFaults();

private:
  void ClearFaultsCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void ErrorInfoCallback(const khi_msgs::msg::ErrorInfo::SharedPtr msg);

  mutable intrinsic::Mutex state_mutex_;
  icon_hwm_controller_msgs::msg::OperationalStatus operational_status_
  INTR_GUARDED_BY(state_mutex_);

  // Callback groups for concurrency.
  rclcpp::CallbackGroup::SharedPtr topic_cb_group_;
  rclcpp::CallbackGroup::SharedPtr clear_faults_service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  rclcpp::Subscription<khi_msgs::msg::ErrorInfo>::SharedPtr error_info_sub_;
  rclcpp::Publisher<icon_hwm_controller_msgs::msg::OperationalStatus>::SharedPtr
    operational_status_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_faults_srv_;
  rclcpp::Client<khi_msgs::srv::ResetError>::SharedPtr reset_error_client_;

  std::chrono::seconds service_timeout_;
};

}  // namespace khi_ros2_icon_hwm
