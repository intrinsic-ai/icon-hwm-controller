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

#include "fanuc_ros2_icon_hwm/fanuc_operational_state_node.hpp"

#include <chrono>
#include <format>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "icon/utils/mutex.h"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace fanuc_ros2_icon_hwm
{

icon_hwm_controller_msgs::msg::OperationalStatus ToOperationalStatus(
  const std::optional<fanuc_msgs::msg::RobotStatus> & robot_status)
{
  icon_hwm_controller_msgs::msg::OperationalStatus status;

  if (!robot_status.has_value()) {
    status.state = icon_hwm_controller_msgs::msg::OperationalStatus::UNKNOWN;
    status.message = "Waiting for FANUC robot status messages";
    return status;
  }

  const bool in_error = robot_status->in_error;
  const bool e_stopped = robot_status->e_stopped;
  const bool motion_possible = robot_status->motion_possible;
  const bool tp_enabled = robot_status->tp_enabled;
  const uint8_t contact_stop_mode = robot_status->contact_stop_mode;

  // E-stop has highest priority. The fault has to be fixed manually before
  // the faults can be cleared.
  if (e_stopped) {
    status.state = icon_hwm_controller_msgs::msg::OperationalStatus::FAULTED;
    status.message = "FANUC robot emergency stop is active. Release E-stop and call clear_faults.";
    return status;
  }

  // The robot is in a collaborative stop. This is only possible for
  // collaborative robots like the CRX when running in collaborative mode.
  if (contact_stop_mode != fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_NONE) {
    status.state = icon_hwm_controller_msgs::msg::OperationalStatus::FAULTED;
    if (contact_stop_mode == fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_SAFE) {
      status.message = "FANUC robot contact stop active. Call clear_faults to recover.";
    } else {
      status.message = std::format(
        "FANUC robot contact stop mode {} active. Call clear_faults to recover.",
        contact_stop_mode);
    }
    return status;
  }

  // The robot's teach pendant is enabled: The robot can't be controlled
  // externally.
  if (tp_enabled) {
    status.state = icon_hwm_controller_msgs::msg::OperationalStatus::DISABLED;
    status.message = "FANUC teach pendant is enabled (manual control mode).";
    return status;
  }

  // The robot is in error but a regular clear faults might fix it.
  if (in_error) {
    status.state = icon_hwm_controller_msgs::msg::OperationalStatus::FAULTED;
    status.message = "FANUC robot is in error/alarm state. Call clear_faults to recover.";
    return status;
  }

  // Motion is not possible due to an unknown reason.
  if (!motion_possible) {
    status.state = icon_hwm_controller_msgs::msg::OperationalStatus::FAULTED;
    status.message =
      "FANUC robot motion_possible is false. Call clear_faults to restore ROS 2 motion control.";
    return status;
  }

  // Normal operation.
  status.state = icon_hwm_controller_msgs::msg::OperationalStatus::ENABLED;
  status.message = "";
  return status;
}

FanucOperationalStateNode::FanucOperationalStateNode(const rclcpp::NodeOptions & options)
: Node("fanuc_operational_state_node", options)
{
  rcl_interfaces::msg::ParameterDescriptor publish_rate_desc;
  publish_rate_desc.description = "Publish rate of the operational status in Hz.";
  rcl_interfaces::msg::FloatingPointRange rate_range;
  rate_range.from_value = 0.1;
  rate_range.to_value = 1000.0;
  publish_rate_desc.floating_point_range.push_back(rate_range);
  const double publish_rate_hz =
    declare_parameter<double>("publish_rate_hz", 10.0, publish_rate_desc);

  rcl_interfaces::msg::ParameterDescriptor timeout_desc;
  timeout_desc.description = "Service timeout in seconds.";
  rcl_interfaces::msg::IntegerRange timeout_range;
  timeout_range.from_value = 1;
  timeout_range.to_value = 300;
  timeout_desc.integer_range.push_back(timeout_range);
  const int64_t service_timeout_sec =
    declare_parameter<int64_t>("service_timeout_sec", 5, timeout_desc);
  service_timeout_ = std::chrono::seconds(service_timeout_sec);

  {
    intrinsic::MutexLock lock(state_mutex_);
    operational_status_.state = icon_hwm_controller_msgs::msg::OperationalStatus::UNKNOWN;
    operational_status_.message = "Waiting for FANUC robot status messages";
  }

  topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  clear_faults_service_cb_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = topic_cb_group_;

  robot_status_sub_ = create_subscription<fanuc_msgs::msg::RobotStatus>(
    "/fanuc_gpio_controller/robot_status", rclcpp::SystemDefaultsQoS(),
    std::function<void(const fanuc_msgs::msg::RobotStatus::SharedPtr)>(
      std::bind_front(&FanucOperationalStateNode::RobotStatusCallback, this)),
    sub_options);

  operational_status_pub_ = create_publisher<icon_hwm_controller_msgs::msg::OperationalStatus>(
    "operational_status", rclcpp::SystemDefaultsQoS());

  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_hz));
  publish_timer_ = create_wall_timer(
    timer_period,
    std::function<void()>(
      std::bind_front(&FanucOperationalStateNode::PublishOperationalStatus, this)),
    topic_cb_group_);

  clear_faults_srv_ = create_service<std_srvs::srv::Trigger>(
    "clear_faults",
    std::function<void(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response>)>(
      std::bind_front(&FanucOperationalStateNode::ClearFaultsCallback, this)),
    rclcpp::SystemDefaultsQoS(),
    clear_faults_service_cb_group_);

  reset_client_ = create_client<fanuc_msgs::srv::Reset>(
    "/fanuc_gpio_controller/reset", rclcpp::SystemDefaultsQoS(), client_cb_group_);

  switch_control_state_client_ = create_client<fanuc_msgs::srv::SwitchControlState>(
    "/fanuc_gpio_controller/switch_control_state", rclcpp::SystemDefaultsQoS(), client_cb_group_);

  read_error_client_ = create_client<fanuc_msgs::srv::ReadError>(
    "/fanuc_gpio_controller/read_error", rclcpp::SystemDefaultsQoS(), client_cb_group_);

  RCLCPP_INFO(get_logger(), "fanuc_operational_state_node initialized.");
}

icon_hwm_controller_msgs::msg::OperationalStatus FanucOperationalStateNode::GetOperationalStatus()
const
{
  intrinsic::MutexLock lock(state_mutex_);
  return operational_status_;
}

void FanucOperationalStateNode::RobotStatusCallback(
  const fanuc_msgs::msg::RobotStatus::SharedPtr msg)
{
  icon_hwm_controller_msgs::msg::OperationalStatus status_to_publish;
  {
    intrinsic::MutexLock lock(state_mutex_);
    operational_status_ = ToOperationalStatus(*msg);
    status_to_publish = operational_status_;
  }
  operational_status_pub_->publish(status_to_publish);
}

void FanucOperationalStateNode::PublishOperationalStatus()
{
  icon_hwm_controller_msgs::msg::OperationalStatus status_to_publish;
  {
    intrinsic::MutexLock lock(state_mutex_);
    status_to_publish = operational_status_;
  }
  operational_status_pub_->publish(status_to_publish);
}

// Executes the official FANUC alarm recovery sequence:
// Reference: https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/fanuc_driver/motion_control_authority.html#alarm-recovery
intrinsic::Status FanucOperationalStateNode::ClearFaults()
{
  RCLCPP_INFO(get_logger(), "Starting FANUC alarm recovery sequence...");

  // First reset active alarms on the robot controller
  if (!reset_client_->wait_for_service(std::chrono::seconds(2))) {
    const std::string err_msg =
      std::format("Service '{}' is not available", reset_client_->get_service_name());
    RCLCPP_WARN(get_logger(), "%s", err_msg.c_str());
    return {intrinsic::StatusCode::kUnavailable, err_msg};
  }

  auto reset_req = std::make_shared<fanuc_msgs::srv::Reset::Request>();
  auto reset_future = reset_client_->async_send_request(reset_req);
  if (reset_future.wait_for(service_timeout_) != std::future_status::ready) {
    const std::string err_msg =
      std::format("Timeout waiting for service '{}'", reset_client_->get_service_name());
    RCLCPP_WARN(get_logger(), "%s", err_msg.c_str());
    return {intrinsic::StatusCode::kDeadlineExceeded, err_msg};
  }

  const auto reset_res = reset_future.get();
  if (reset_res->result != 0) {
    const std::string err_msg =
      std::format("Reset service failed with error code: {}", reset_res->result);
    RCLCPP_WARN(get_logger(), "%s", err_msg.c_str());
    return {intrinsic::StatusCode::kInternal, err_msg};
  }

  RCLCPP_INFO(get_logger(),
    "Alarms reset successfully. Switching motion control back to ROS 2...");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Switch motion control authority back to ROS 2 driver.
  if (!switch_control_state_client_->wait_for_service(std::chrono::seconds(2))) {
    const std::string err_msg = std::format(
      "Service '{}' is not available", switch_control_state_client_->get_service_name());
    RCLCPP_WARN(get_logger(), "%s", err_msg.c_str());
    return {intrinsic::StatusCode::kUnavailable, err_msg};
  }

  auto switch_req = std::make_shared<fanuc_msgs::srv::SwitchControlState::Request>();
  // A status 1 means that the ROS 2 Driver has motion control.
  switch_req->status = 1;

  auto switch_future = switch_control_state_client_->async_send_request(switch_req);
  if (switch_future.wait_for(service_timeout_) != std::future_status::ready) {
    const std::string err_msg = std::format(
      "Timeout waiting for service '{}'", switch_control_state_client_->get_service_name());
    RCLCPP_WARN(get_logger(), "%s", err_msg.c_str());
    return {intrinsic::StatusCode::kDeadlineExceeded, err_msg};
  }

  const auto switch_res = switch_future.get();
  if (switch_res->result != 0) {
    const std::string err_msg = std::format(
      "Switching control state failed with error code: {}", switch_res->result);
    RCLCPP_WARN(get_logger(), "%s", err_msg.c_str());
    return {intrinsic::StatusCode::kInternal, err_msg};
  }

  RCLCPP_INFO(get_logger(), "FANUC alarm recovery succeeded: motion control restored to ROS 2.");

  return intrinsic::OkStatus();
}

void FanucOperationalStateNode::ClearFaultsCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*unused*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received clear_faults service request");

  const auto status = ClearFaults();
  response->success = status.ok();
  response->message = status.ok() ? "Successfully reset alarms and restored ROS 2 motion control." :
    status.message;
}

}  // namespace fanuc_ros2_icon_hwm
