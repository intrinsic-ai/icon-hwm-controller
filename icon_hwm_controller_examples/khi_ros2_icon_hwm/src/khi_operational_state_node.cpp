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

#include "khi_ros2_icon_hwm/khi_operational_state_node.hpp"

#include <chrono>
#include <format>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "icon/utils/mutex.h"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace khi_ros2_icon_hwm
{

icon_hwm_controller_msgs::msg::OperationalStatus ToOperationalStatus(
  const std::optional<khi_msgs::msg::ErrorInfo> & error_info)
{
  icon_hwm_controller_msgs::msg::OperationalStatus status;

  if (!error_info.has_value() || error_info->error_codes.empty()) {
    status.state = icon_hwm_controller_msgs::msg::OperationalStatus::ENABLED;
    status.message = "";
    return status;
  }

  status.state = icon_hwm_controller_msgs::msg::OperationalStatus::FAULTED;
  std::string error_details;
  for (const auto & error_msg : error_info->error_msgs) {
    if (!error_details.empty()) {
      error_details += "; ";
    }
    error_details += error_msg;
  }
  if (error_details.empty()) {
    for (const auto error_code : error_info->error_codes) {
      if (!error_details.empty()) {
        error_details += ", ";
      }
      error_details += std::to_string(error_code);
    }
  }

  status.message =
    std::format("KHI robot error active: {}. Call clear_faults to recover.", error_details);
  return status;
}

KhiOperationalStateNode::KhiOperationalStateNode(const rclcpp::NodeOptions & options)
: Node("khi_operational_state_node", options)
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

  rcl_interfaces::msg::ParameterDescriptor controller_no_desc;
  controller_no_desc.description =
    "Controller number if connecting multiple controllers from a single PC.";
  rcl_interfaces::msg::IntegerRange controller_no_range;
  controller_no_range.from_value = 0;
  controller_no_range.to_value = 15;
  controller_no_desc.integer_range.push_back(controller_no_range);
  const int64_t controller_no =
    declare_parameter<int64_t>("controller_no", 0, controller_no_desc);

  const std::string reset_error_service = declare_parameter<std::string>(
    "reset_error_service",
    std::format("/khi_controller{}/khi_service/reset_error", controller_no));
  const std::string error_info_topic = declare_parameter<std::string>(
    "error_info_topic",
    std::format("/khi_controller{}/khi_publisher/error_info", controller_no));

  {
    intrinsic::MutexLock lock(state_mutex_);
    operational_status_.state = icon_hwm_controller_msgs::msg::OperationalStatus::ENABLED;
    operational_status_.message = "";
  }

  topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  clear_faults_service_cb_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = topic_cb_group_;

  error_info_sub_ = create_subscription<khi_msgs::msg::ErrorInfo>(
    error_info_topic, rclcpp::SystemDefaultsQoS(),
    std::function<void(const khi_msgs::msg::ErrorInfo::SharedPtr)>(
      std::bind_front(&KhiOperationalStateNode::ErrorInfoCallback, this)),
    sub_options);

  operational_status_pub_ = create_publisher<icon_hwm_controller_msgs::msg::OperationalStatus>(
    "operational_status", rclcpp::SystemDefaultsQoS());

  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_hz));
  publish_timer_ = create_wall_timer(
    timer_period,
    std::function<void()>(
      std::bind_front(&KhiOperationalStateNode::PublishOperationalStatus, this)),
    topic_cb_group_);

  clear_faults_srv_ = create_service<std_srvs::srv::Trigger>(
    "clear_faults",
    std::function<void(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response>)>(
      std::bind_front(&KhiOperationalStateNode::ClearFaultsCallback, this)),
    rclcpp::SystemDefaultsQoS(),
    clear_faults_service_cb_group_);

  reset_error_client_ = create_client<khi_msgs::srv::ResetError>(
    reset_error_service, rclcpp::SystemDefaultsQoS(), client_cb_group_);

  RCLCPP_INFO(
    get_logger(),
    "khi_operational_state_node initialized with reset_service='%s'.",
    reset_error_service.c_str());
}

icon_hwm_controller_msgs::msg::OperationalStatus KhiOperationalStateNode::GetOperationalStatus()
const
{
  intrinsic::MutexLock lock(state_mutex_);
  return operational_status_;
}

void KhiOperationalStateNode::ErrorInfoCallback(
  const khi_msgs::msg::ErrorInfo::SharedPtr msg)
{
  icon_hwm_controller_msgs::msg::OperationalStatus status_to_publish;
  {
    intrinsic::MutexLock lock(state_mutex_);
    operational_status_ = ToOperationalStatus(*msg);
    status_to_publish = operational_status_;
  }
  operational_status_pub_->publish(status_to_publish);
}

void KhiOperationalStateNode::PublishOperationalStatus()
{
  icon_hwm_controller_msgs::msg::OperationalStatus status_to_publish;
  {
    intrinsic::MutexLock lock(state_mutex_);
    status_to_publish = operational_status_;
  }
  operational_status_pub_->publish(status_to_publish);
}

intrinsic::Status KhiOperationalStateNode::ClearFaults()
{
  RCLCPP_INFO(get_logger(), "Starting KHI alarm recovery sequence...");

  // Reset active error on KHI robot controller
  if (!reset_error_client_->wait_for_service(std::chrono::seconds(2))) {
    const std::string err_msg =
      std::format("Service '{}' is not available", reset_error_client_->get_service_name());
    RCLCPP_WARN(get_logger(), "%s", err_msg.c_str());
    return {intrinsic::StatusCode::kUnavailable, err_msg};
  }

  const auto reset_req = std::make_shared<khi_msgs::srv::ResetError::Request>();
  auto reset_future = reset_error_client_->async_send_request(reset_req);
  if (reset_future.wait_for(service_timeout_) != std::future_status::ready) {
    const std::string err_msg =
      std::format("Timeout waiting for service '{}'", reset_error_client_->get_service_name());
    RCLCPP_WARN(get_logger(), "%s", err_msg.c_str());
    return {intrinsic::StatusCode::kDeadlineExceeded, err_msg};
  }

  const auto reset_res = reset_future.get();
  if (!reset_res->success) {
    std::string details;
    for (const auto & err : reset_res->error_msgs) {
      if (!details.empty()) {
        details += "; ";
      }
      details += err;
    }
    const std::string err_msg =
      std::format("KHI reset error service failed: {}",
        details.empty() ? "unknown error" : details);
    RCLCPP_WARN(get_logger(), "%s", err_msg.c_str());
    return {intrinsic::StatusCode::kInternal, err_msg};
  }

  RCLCPP_INFO(get_logger(), "KHI alarms reset successfully.");

  {
    intrinsic::MutexLock lock(state_mutex_);
    operational_status_.state = icon_hwm_controller_msgs::msg::OperationalStatus::ENABLED;
    operational_status_.message = "";
  }
  PublishOperationalStatus();

  return intrinsic::OkStatus();
}

void KhiOperationalStateNode::ClearFaultsCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*unused*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received clear_faults service request");

  const auto status = ClearFaults();
  response->success = status.ok();
  response->message = status.ok() ?
    "Successfully reset KHI error." :
    status.message;
}

}  // namespace khi_ros2_icon_hwm
