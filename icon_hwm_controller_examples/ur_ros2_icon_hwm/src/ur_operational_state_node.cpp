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

#include "ur_ros2_icon_hwm/ur_operational_state_node.hpp"

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

namespace ur_ros2_icon_hwm
{

icon_hwm_controller_msgs::msg::OperationalStatus ToOperationalStatus(
  const RobotStatus & robot_status)
{
  icon_hwm_controller_msgs::msg::OperationalStatus status;

  if (!robot_status.safety_mode.has_value() ||
    !robot_status.robot_mode.has_value() ||
    !robot_status.program_running.has_value())
  {
    status.state = icon_hwm_controller_msgs::msg::OperationalStatus::UNKNOWN;
    status.message = "Waiting for robot status messages";
    return status;
  }

  const uint8_t safety_mode = *robot_status.safety_mode;
  const int8_t robot_mode = *robot_status.robot_mode;
  const bool program_running = *robot_status.program_running;

  // Safety mode takes precedence. If safety is not `NORMAL` or `REDUCED`, the
  // robot is `FAULTED`.
  // For more information see https://www.universal-robots.com/manuals/EN/HTML/SW5_19/Content/prod-usr-man/software/PolyScope/content/safety_g5/Safety_modes_g5_en.htm,
  // https://www.universal-robots.com/manuals/EN/HTML/SW5_25/Content/prod-dashboard/Dashboard_table.htm,
  // as well as https://docs.universal-robots.com/tutorials/controlling-robot-externally/stop-recovery.html
  if (safety_mode != ur_dashboard_msgs::msg::SafetyMode::NORMAL &&
    safety_mode != ur_dashboard_msgs::msg::SafetyMode::REDUCED)
  {
    status.state = icon_hwm_controller_msgs::msg::OperationalStatus::FAULTED;
    switch (safety_mode) {
      case ur_dashboard_msgs::msg::SafetyMode::PROTECTIVE_STOP:
        status.message = "Robot in protective stop. Call clear_faults to unlock.";
        break;
      case ur_dashboard_msgs::msg::SafetyMode::RECOVERY:
        status.message = "Robot in safety recovery mode.";
        break;
      case ur_dashboard_msgs::msg::SafetyMode::SAFEGUARD_STOP:
        status.message = "Safeguard stop active.";
        break;
      case ur_dashboard_msgs::msg::SafetyMode::SYSTEM_EMERGENCY_STOP:
        status.message = "System emergency stop active.";
        break;
      case ur_dashboard_msgs::msg::SafetyMode::ROBOT_EMERGENCY_STOP:
        status.message = "Robot emergency stop active.";
        break;
      case ur_dashboard_msgs::msg::SafetyMode::VIOLATION:
        status.message = "Safety violation detected. Safety restart required.";
        break;
      case ur_dashboard_msgs::msg::SafetyMode::FAULT:
        status.message = "Safety fault detected. Safety restart required.";
        break;
      case ur_dashboard_msgs::msg::SafetyMode::VALIDATE_JOINT_ID:
        status.message = "Robot validating joint ID.";
        break;
      case ur_dashboard_msgs::msg::SafetyMode::UNDEFINED_SAFETY_MODE:
        status.message = "Undefined safety mode.";
        break;
      case ur_dashboard_msgs::msg::SafetyMode::AUTOMATIC_MODE_SAFEGUARD_STOP:
        status.message = "Automatic mode safeguard stop active.";
        break;
      case ur_dashboard_msgs::msg::SafetyMode::SYSTEM_THREE_POSITION_ENABLING_STOP:
        status.message = "System three-position enabling stop active.";
        break;
      default:
        status.message = std::format("Unknown safety mode: {}", safety_mode);
        break;
    }
    return status;
  }

  // Safety is `NORMAL` or `REDUCED`: Check robot mode.
  // For more information see https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html#robot-mode
  switch (robot_mode) {
    case ur_dashboard_msgs::msg::RobotMode::NO_CONTROLLER:
      status.state = icon_hwm_controller_msgs::msg::OperationalStatus::FAULTED;
      status.message = "No controller connected to robot.";
      return status;
    case ur_dashboard_msgs::msg::RobotMode::DISCONNECTED:
      status.state = icon_hwm_controller_msgs::msg::OperationalStatus::FAULTED;
      status.message = "Robot is disconnected.";
      return status;
    case ur_dashboard_msgs::msg::RobotMode::CONFIRM_SAFETY:
      status.state = icon_hwm_controller_msgs::msg::OperationalStatus::FAULTED;
      status.message = "Robot requires safety confirmation.";
      return status;
    case ur_dashboard_msgs::msg::RobotMode::BOOTING:
    case ur_dashboard_msgs::msg::RobotMode::POWER_OFF:
    case ur_dashboard_msgs::msg::RobotMode::POWER_ON:
    case ur_dashboard_msgs::msg::RobotMode::IDLE:
    case ur_dashboard_msgs::msg::RobotMode::BACKDRIVE:
    case ur_dashboard_msgs::msg::RobotMode::UPDATING_FIRMWARE:
      status.state = icon_hwm_controller_msgs::msg::OperationalStatus::DISABLED;
      status.message = "";
      return status;
    case ur_dashboard_msgs::msg::RobotMode::RUNNING:
      if (program_running) {
        status.state = icon_hwm_controller_msgs::msg::OperationalStatus::ENABLED;
        status.message = "";
      } else {
        status.state = icon_hwm_controller_msgs::msg::OperationalStatus::DISABLED;
        status.message = "";
      }
      return status;
    default:
      status.state = icon_hwm_controller_msgs::msg::OperationalStatus::FAULTED;
      status.message = std::format("Unknown robot mode: {}", robot_mode);
      return status;
  }
}

UrOperationalStateNode::UrOperationalStateNode(const rclcpp::NodeOptions & options)
: Node("ur_operational_state_node", options)
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

  rcl_interfaces::msg::ParameterDescriptor action_timeout_desc;
  action_timeout_desc.description = "Action timeout in seconds.";
  rcl_interfaces::msg::IntegerRange action_timeout_range;
  action_timeout_range.from_value = 1;
  action_timeout_range.to_value = 300;
  action_timeout_desc.integer_range.push_back(action_timeout_range);
  const int64_t action_timeout_sec =
    declare_parameter<int64_t>("action_timeout_sec", 15, action_timeout_desc);
  action_timeout_ = std::chrono::seconds(action_timeout_sec);

  {
    intrinsic::MutexLock lock(state_mutex_);
    operational_status_.state = icon_hwm_controller_msgs::msg::OperationalStatus::UNKNOWN;
    operational_status_.message = "Waiting for robot status messages";
  }

  // Create callback groups for concurrent execution.
  // We don't want to concurrently read from the status topics, or publish to the topic
  // that combines the status values.
  topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // The ClearFaults service server gets its own group.
  clear_faults_service_cb_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // Service and action clients can receive responses in parallel.
  client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = topic_cb_group_;

  robot_mode_sub_ = create_subscription<ur_dashboard_msgs::msg::RobotMode>(
    "io_and_status_controller/robot_mode", rclcpp::SystemDefaultsQoS(),
    std::function<void(const ur_dashboard_msgs::msg::RobotMode::SharedPtr)>(
      std::bind_front(&UrOperationalStateNode::RobotModeCallback, this)),
    sub_options);
  safety_mode_sub_ = create_subscription<ur_dashboard_msgs::msg::SafetyMode>(
    "io_and_status_controller/safety_mode", rclcpp::SystemDefaultsQoS(),
    std::function<void(const ur_dashboard_msgs::msg::SafetyMode::SharedPtr)>(
      std::bind_front(&UrOperationalStateNode::SafetyModeCallback, this)),
    sub_options);
  program_running_sub_ = create_subscription<std_msgs::msg::Bool>(
    "io_and_status_controller/robot_program_running", rclcpp::SystemDefaultsQoS(),
    std::function<void(const std_msgs::msg::Bool::SharedPtr)>(
      std::bind_front(&UrOperationalStateNode::ProgramRunningCallback, this)),
    sub_options);

  operational_status_pub_ = create_publisher<icon_hwm_controller_msgs::msg::OperationalStatus>(
    "operational_status", rclcpp::SystemDefaultsQoS());

  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_hz));
  publish_timer_ = create_wall_timer(
    timer_period,
    std::function<void()>(
      std::bind_front(&UrOperationalStateNode::PublishOperationalStatus, this)),
    topic_cb_group_);

  clear_faults_srv_ = create_service<std_srvs::srv::Trigger>(
    "clear_faults",
    std::function<void(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response>)>(
      std::bind_front(&UrOperationalStateNode::ClearFaultsCallback, this)),
    rclcpp::SystemDefaultsQoS(),
    clear_faults_service_cb_group_);

  set_mode_action_client_ = rclcpp_action::create_client<SetModeAction>(
    this, "ur_robot_state_helper/set_mode", client_cb_group_);

  resend_robot_program_client_ = create_client<std_srvs::srv::Trigger>(
    "io_and_status_controller/resend_robot_program", rclcpp::SystemDefaultsQoS(),
    client_cb_group_);
  unlock_protective_stop_client_ = create_client<std_srvs::srv::Trigger>(
    "dashboard_client/unlock_protective_stop", rclcpp::SystemDefaultsQoS(), client_cb_group_);
  restart_safety_client_ = create_client<std_srvs::srv::Trigger>(
    "dashboard_client/restart_safety", rclcpp::SystemDefaultsQoS(), client_cb_group_);
  close_safety_popup_client_ = create_client<std_srvs::srv::Trigger>(
    "dashboard_client/close_safety_popup", rclcpp::SystemDefaultsQoS(), client_cb_group_);
  close_popup_client_ = create_client<std_srvs::srv::Trigger>(
    "dashboard_client/close_popup", rclcpp::SystemDefaultsQoS(), client_cb_group_);
  brake_release_client_ = create_client<std_srvs::srv::Trigger>(
    "dashboard_client/brake_release", rclcpp::SystemDefaultsQoS(), client_cb_group_);
  power_on_client_ = create_client<std_srvs::srv::Trigger>(
    "dashboard_client/power_on", rclcpp::SystemDefaultsQoS(), client_cb_group_);
  connect_client_ = create_client<std_srvs::srv::Trigger>(
    "dashboard_client/connect", rclcpp::SystemDefaultsQoS(), client_cb_group_);

  RCLCPP_INFO(get_logger(), "ur_operational_state_node initialized.");
}

icon_hwm_controller_msgs::msg::OperationalStatus UrOperationalStateNode::GetOperationalStatus()
const
{
  intrinsic::MutexLock lock(state_mutex_);
  return operational_status_;
}

void UrOperationalStateNode::RobotModeCallback(
  const ur_dashboard_msgs::msg::RobotMode::SharedPtr msg)
{
  icon_hwm_controller_msgs::msg::OperationalStatus status_to_publish;
  {
    intrinsic::MutexLock lock(state_mutex_);
    robot_status_.robot_mode = msg->mode;
    operational_status_ = ToOperationalStatus(robot_status_);
    status_to_publish = operational_status_;
  }
  operational_status_pub_->publish(status_to_publish);
}

void UrOperationalStateNode::SafetyModeCallback(
  const ur_dashboard_msgs::msg::SafetyMode::SharedPtr msg)
{
  icon_hwm_controller_msgs::msg::OperationalStatus status_to_publish;
  {
    intrinsic::MutexLock lock(state_mutex_);
    robot_status_.safety_mode = msg->mode;
    operational_status_ = ToOperationalStatus(robot_status_);
    status_to_publish = operational_status_;
  }
  operational_status_pub_->publish(status_to_publish);
}

void UrOperationalStateNode::ProgramRunningCallback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  icon_hwm_controller_msgs::msg::OperationalStatus status_to_publish;
  {
    intrinsic::MutexLock lock(state_mutex_);
    robot_status_.program_running = msg->data;
    operational_status_ = ToOperationalStatus(robot_status_);
    status_to_publish = operational_status_;
  }
  operational_status_pub_->publish(status_to_publish);
}

void UrOperationalStateNode::PublishOperationalStatus()
{
  icon_hwm_controller_msgs::msg::OperationalStatus status_to_publish;
  {
    intrinsic::MutexLock lock(state_mutex_);
    status_to_publish = operational_status_;
  }
  operational_status_pub_->publish(status_to_publish);
}

intrinsic::Status UrOperationalStateNode::CallTriggerService(
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
  const std::chrono::seconds timeout)
{
  if (!client) {
    return {intrinsic::StatusCode::kInvalidArgument, "Service client is null"};
  }
  if (!client->service_is_ready()) {
    if (!client->wait_for_service(std::chrono::milliseconds(500))) {
      const std::string err_msg =
        std::format("Service '{}' is not available", client->get_service_name());
      RCLCPP_DEBUG(get_logger(), "%s", err_msg.c_str());
      return {intrinsic::StatusCode::kUnavailable, err_msg};
    }
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    const std::string err_msg = std::format(
      "Service '{}' timed out after {} s", client->get_service_name(), timeout.count());
    RCLCPP_WARN(get_logger(), "%s", err_msg.c_str());
    return {intrinsic::StatusCode::kDeadlineExceeded, err_msg};
  }
  const auto response = future.get();
  RCLCPP_INFO(
    get_logger(), "Service '%s' responded: success=%s, message='%s'",
    client->get_service_name(), response->success ? "true" : "false",
    response->message.c_str());
  if (!response->success) {
    return {intrinsic::StatusCode::kInternal, response->message};
  }
  return intrinsic::OkStatus();
}

intrinsic::Status UrOperationalStateNode::CallSetModeAction(
  const int8_t target_robot_mode, const bool stop_program,
  const bool play_program, const std::chrono::seconds timeout)
{
  if (!set_mode_action_client_) {
    return {intrinsic::StatusCode::kFailedPrecondition, "SetMode action client is not initialized"};
  }

  if (!set_mode_action_client_->action_server_is_ready()) {
    if (!set_mode_action_client_->wait_for_action_server(std::chrono::milliseconds(1000))) {
      return {intrinsic::StatusCode::kUnavailable, "SetMode action server is not available"};
    }
  }

  const auto goal = SetModeAction::Goal()
    .set__target_robot_mode(target_robot_mode)
    .set__stop_program(stop_program)
    .set__play_program(play_program);

  RCLCPP_INFO(
    get_logger(),
    "Sending SetMode goal: target_robot_mode=%d, stop_program=%s, play_program=%s",
    target_robot_mode, stop_program ? "true" : "false",
    play_program ? "true" : "false");

  auto send_goal_options = rclcpp_action::Client<SetModeAction>::SendGoalOptions();
  auto goal_handle_future = set_mode_action_client_->async_send_goal(goal, send_goal_options);

  if (goal_handle_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    return {intrinsic::StatusCode::kDeadlineExceeded,
      "Timeout sending goal to SetMode action server"};
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    return {intrinsic::StatusCode::kInternal, "SetMode goal was rejected by action server"};
  }

  auto result_future = set_mode_action_client_->async_get_result(goal_handle);
  if (result_future.wait_for(timeout) != std::future_status::ready) {
    return {intrinsic::StatusCode::kDeadlineExceeded, "Timeout waiting for SetMode action result"};
  }

  const auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
    !wrapped_result.result || !wrapped_result.result->success)
  {
    const std::string err_msg = (wrapped_result.result && !wrapped_result.result->message.empty()) ?
      wrapped_result.result->message : "SetMode action did not succeed";
    RCLCPP_WARN(
      get_logger(), "SetMode action failed with code %d: '%s'",
      static_cast<int>(wrapped_result.code), err_msg.c_str());
    return {intrinsic::StatusCode::kInternal, err_msg};
  }
  RCLCPP_INFO(
    get_logger(), "SetMode action succeeded: '%s'",
    wrapped_result.result->message.c_str());
  return intrinsic::OkStatus();
}

intrinsic::Status UrOperationalStateNode::ClearFaults()
{
  RobotStatus current_status;
  {
    intrinsic::MutexLock lock(state_mutex_);
    current_status = robot_status_;
  }
  const uint8_t current_safety = current_status.safety_mode.value_or(
    ur_dashboard_msgs::msg::SafetyMode::UNDEFINED_SAFETY_MODE);
  const int8_t current_robot = current_status.robot_mode.value_or(
    ur_dashboard_msgs::msg::RobotMode::DISCONNECTED);

  RCLCPP_INFO(
    get_logger(),
    "Executing ClearFaults sequence (current safety_mode=%u, robot_mode=%d)",
    current_safety, current_robot);

  // Attempts to connect to dashboard client if disconnected.
  if (current_robot == ur_dashboard_msgs::msg::RobotMode::DISCONNECTED ||
    current_robot == ur_dashboard_msgs::msg::RobotMode::NO_CONTROLLER)
  {
    RCLCPP_INFO(get_logger(), "Connecting to dashboard server...");
    CallTriggerService(connect_client_, service_timeout_);
  }

  // Handles safety faults and unlocks stops.
  switch (current_safety) {
    case ur_dashboard_msgs::msg::SafetyMode::PROTECTIVE_STOP: {
        RCLCPP_INFO(get_logger(), "Unlocking protective stop...");
        CallTriggerService(unlock_protective_stop_client_, service_timeout_);
        break;
      }
    case ur_dashboard_msgs::msg::SafetyMode::VIOLATION:
    case ur_dashboard_msgs::msg::SafetyMode::FAULT:
    case ur_dashboard_msgs::msg::SafetyMode::SAFEGUARD_STOP:
    case ur_dashboard_msgs::msg::SafetyMode::AUTOMATIC_MODE_SAFEGUARD_STOP:
    case ur_dashboard_msgs::msg::SafetyMode::SYSTEM_EMERGENCY_STOP:
    case ur_dashboard_msgs::msg::SafetyMode::ROBOT_EMERGENCY_STOP: {
        RCLCPP_INFO(get_logger(), "Restarting safety system...");
        CallTriggerService(close_safety_popup_client_, service_timeout_);
        CallTriggerService(restart_safety_client_, service_timeout_);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        break;
      }
    default: {
        RCLCPP_INFO(get_logger(), "Robot safety mode is okay.");
        break;
      }
  }
  // Dismiss any lingering dialogs.
  CallTriggerService(close_safety_popup_client_, service_timeout_);
  CallTriggerService(close_popup_client_, service_timeout_);

  // Set robot mode to RUNNING (including a stop/start of the robot program).
  const auto set_mode_status = CallSetModeAction(
    ur_dashboard_msgs::msg::RobotMode::RUNNING,
    /*stop_program=*/ true,
    /*play_program=*/ true,
    action_timeout_);

  if (set_mode_status.ok()) {
    RCLCPP_INFO(get_logger(), "Faults cleared successfully via robot_state_helper SetMode action.");
    PublishOperationalStatus();
    return intrinsic::OkStatus();
  }

  RCLCPP_WARN(
    get_logger(),
    "SetMode action failed: '%s'. Attempting to resend robot program...",
    set_mode_status.message.c_str());

  // We are running in headless mode, so use
  // `/io_and_status_controller/resend_robot_program` to restart the external
  // control script.
  CallTriggerService(brake_release_client_, service_timeout_);
  const auto resend_status =
    CallTriggerService(resend_robot_program_client_, service_timeout_);

  if (resend_status.ok()) {
    RCLCPP_INFO(get_logger(), "Faults cleared via resend_robot_program.");
    PublishOperationalStatus();
    return intrinsic::OkStatus();
  }

  PublishOperationalStatus();
  std::string failure_message =
    "Neither SetMode nor resend_robot_program were able to clear the fault.";
  if (!set_mode_status.message.empty()) {
    failure_message += " SetMode error: '" + set_mode_status.message + "'.";
  }
  if (!resend_status.message.empty()) {
    failure_message += " resend_robot_program error: '" + resend_status.message + "'.";
  }
  return {intrinsic::StatusCode::kInternal, failure_message};
}

void UrOperationalStateNode::ClearFaultsCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*unused*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received clear_faults service request");

  const auto status = ClearFaults();
  response->success = status.ok();
  response->message = status.ok() ? "Successfully cleared faults and recovered robot." :
    status.message;
}

}  // namespace ur_ros2_icon_hwm
