#include "ur_operational_state_node/ur_operational_state_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>

namespace ur_operational_state_node
{

icon_hwm_controller_msgs::msg::OperationalStatus EvaluateOperationalStatus(
  const RobotStateInputs & inputs)
{
  icon_hwm_controller_msgs::msg::OperationalStatus status;

  if (!inputs.safety_mode.has_value() || !inputs.robot_mode.has_value() ||
    !inputs.program_running.has_value())
  {
    status.state = icon_hwm_controller_msgs::msg::OperationalStatus::UNKNOWN;
    status.message = "Waiting for robot status messages";
    return status;
  }

  const uint8_t safety_mode = *inputs.safety_mode;
  const int8_t robot_mode = *inputs.robot_mode;
  const bool program_running = *inputs.program_running;

  // Safety mode takes precedence. If safety is not NORMAL or REDUCED, the robot is FAULTED.
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
        status.message = "Unknown safety mode: " + std::to_string(safety_mode);
        break;
    }
    return status;
  }

  // Safety is NORMAL or REDUCED. Checks robot mode.
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
      status.message = "Unknown robot mode: " + std::to_string(robot_mode);
      return status;
  }
}

UrOperationalStateNode::UrOperationalStateNode(const rclcpp::NodeOptions & options)
: Node("ur_operational_state_node", options)
{
  const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 10.0);
  const int64_t service_timeout_sec = declare_parameter<int64_t>("service_timeout_sec", 5);
  const int64_t action_timeout_sec = declare_parameter<int64_t>("action_timeout_sec", 15);

  service_timeout_ = std::chrono::seconds(service_timeout_sec);
  action_timeout_ = std::chrono::seconds(action_timeout_sec);

  {
    // Initialize status to unknown.
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_status_.state = icon_hwm_controller_msgs::msg::OperationalStatus::UNKNOWN;
    latest_status_.message = "Waiting for robot status messages";
  }
  // Create callback groups for concurrent execution.
  // We don't want to concurrently read from the status topics, or publish to the topic
  // that combines the status values.
  topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // The ClearFaults service server gets its own group.
  clear_faults_service_cb_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // Service and action clients can receive responsese in parallel.
  client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = topic_cb_group_;

  // Subscribe to UR driver status topics.
  robot_mode_sub_ = create_subscription<ur_dashboard_msgs::msg::RobotMode>(
    "io_and_status_controller/robot_mode", rclcpp::SystemDefaultsQoS(),
    std::function<void(const ur_dashboard_msgs::msg::RobotMode::SharedPtr)>(
      std::bind_front(&UrOperationalStateNode::OnRobotMode, this)),
    sub_options);
  safety_mode_sub_ = create_subscription<ur_dashboard_msgs::msg::SafetyMode>(
    "io_and_status_controller/safety_mode", rclcpp::SystemDefaultsQoS(),
    std::function<void(const ur_dashboard_msgs::msg::SafetyMode::SharedPtr)>(
      std::bind_front(&UrOperationalStateNode::OnSafetyMode, this)),
    sub_options);
  program_running_sub_ = create_subscription<std_msgs::msg::Bool>(
    "io_and_status_controller/robot_program_running", rclcpp::SystemDefaultsQoS(),
    std::function<void(const std_msgs::msg::Bool::SharedPtr)>(
      std::bind_front(&UrOperationalStateNode::OnProgramRunning, this)),
    sub_options);

  operational_status_pub_ = create_publisher<icon_hwm_controller_msgs::msg::OperationalStatus>(
    "operational_status", rclcpp::SystemDefaultsQoS());

  // Creates periodic timer for regular status publication. See header for explanation.
  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / (publish_rate_hz > 0.0 ? publish_rate_hz : 10.0)));
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
      std::bind_front(&UrOperationalStateNode::HandleClearFaults, this)),
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

icon_hwm_controller_msgs::msg::OperationalStatus UrOperationalStateNode::GetLatestStatus() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_status_;
}

RobotStateInputs UrOperationalStateNode::CurrentInputs() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return inputs_;
}

void UrOperationalStateNode::OnRobotMode(
  const ur_dashboard_msgs::msg::RobotMode::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  inputs_.robot_mode = msg->mode;
  latest_status_ = EvaluateOperationalStatus(inputs_);
  operational_status_pub_->publish(latest_status_);
}

void UrOperationalStateNode::OnSafetyMode(
  const ur_dashboard_msgs::msg::SafetyMode::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  inputs_.safety_mode = msg->mode;
  latest_status_ = EvaluateOperationalStatus(inputs_);
  operational_status_pub_->publish(latest_status_);
}

void UrOperationalStateNode::OnProgramRunning(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  inputs_.program_running = msg->data;
  latest_status_ = EvaluateOperationalStatus(inputs_);
  operational_status_pub_->publish(latest_status_);
}

void UrOperationalStateNode::PublishOperationalStatus()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_status_ = EvaluateOperationalStatus(inputs_);
  operational_status_pub_->publish(latest_status_);
}

SuccessAndMessage UrOperationalStateNode::CallTriggerService(
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
  std::chrono::seconds timeout)
{
  SuccessAndMessage result;
  if (!client) {
    result.success = false;
    result.message = "Service client is null";
    return result;
  }
  if (!client->service_is_ready()) {
    if (!client->wait_for_service(std::chrono::milliseconds(500))) {
      RCLCPP_DEBUG(get_logger(), "Service '%s' is not available", client->get_service_name());
      result.success = false;
      result.message = std::string("Service '") + client->get_service_name() + "' is not available";
      return result;
    }
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_WARN(
      get_logger(), "Service '%s' timed out after %ld s",
      client->get_service_name(), timeout.count());
    result.success = false;
    result.message = std::string("Service '") + client->get_service_name() + "' timed out";
    return result;
  }
  auto response = future.get();
  result.success = response->success;
  result.message = response->message;
  RCLCPP_INFO(
    get_logger(), "Service '%s' responded: success=%s, message='%s'",
    client->get_service_name(), result.success ? "true" : "false",
    result.message.c_str());
  return result;
}

SuccessAndMessage UrOperationalStateNode::CallSetModeAction(
  int8_t target_robot_mode, bool stop_program, bool play_program,
  std::chrono::seconds timeout)
{
  SuccessAndMessage result;
  if (!set_mode_action_client_) {
    result.success = false;
    result.message = "SetMode action client is not initialized";
    return result;
  }

  if (!set_mode_action_client_->action_server_is_ready()) {
    if (!set_mode_action_client_->wait_for_action_server(std::chrono::milliseconds(1000))) {
      result.success = false;
      result.message = "SetMode action server is not available";
      return result;
    }
  }

  SetModeAction::Goal goal;
  goal.target_robot_mode = target_robot_mode;
  goal.stop_program = stop_program;
  goal.play_program = play_program;

  RCLCPP_INFO(
    get_logger(),
    "Sending SetMode goal: target_robot_mode=%d, stop_program=%s, play_program=%s",
    target_robot_mode, stop_program ? "true" : "false",
    play_program ? "true" : "false");

  auto send_goal_options = rclcpp_action::Client<SetModeAction>::SendGoalOptions();
  auto goal_handle_future = set_mode_action_client_->async_send_goal(goal, send_goal_options);

  if (goal_handle_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    result.success = false;
    result.message = "Timeout sending goal to SetMode action server";
    return result;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    result.success = false;
    result.message = "SetMode goal was rejected by action server";
    return result;
  }

  auto result_future = set_mode_action_client_->async_get_result(goal_handle);
  if (result_future.wait_for(timeout) != std::future_status::ready) {
    result.success = false;
    result.message = "Timeout waiting for SetMode action result";
    return result;
  }

  auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
    !wrapped_result.result || !wrapped_result.result->success)
  {
    result.success = false;
    result.message = (wrapped_result.result && !wrapped_result.result->message.empty()) ?
      wrapped_result.result->message : "SetMode action did not succeed";
    RCLCPP_WARN(
      get_logger(), "SetMode action failed with code %d: '%s'",
      static_cast<int>(wrapped_result.code), result.message.c_str());
    return result;
  }
  result.success = true;
  result.message = wrapped_result.result->message;
  RCLCPP_INFO(
    get_logger(), "SetMode action succeeded: '%s'",
    result.message.c_str());
  return result;
}

SuccessAndMessage UrOperationalStateNode::ClearFaults()
{
  RobotStateInputs current_inputs = CurrentInputs();
  const uint8_t current_safety = current_inputs.safety_mode.value_or(
    ur_dashboard_msgs::msg::SafetyMode::UNDEFINED_SAFETY_MODE);
  const int8_t current_robot = current_inputs.robot_mode.value_or(
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
  switch(current_safety) {
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
  SuccessAndMessage set_mode_result = CallSetModeAction(
    ur_dashboard_msgs::msg::RobotMode::RUNNING,
    /*stop_program=*/ true,
    /*play_program=*/ true,
    action_timeout_);

  if (set_mode_result.success) {
    RCLCPP_INFO(get_logger(), "Faults cleared successfully via robot_state_helper SetMode action.");
    PublishOperationalStatus();
    return set_mode_result;
  }

  RCLCPP_WARN(
    get_logger(),
    "SetMode action failed: '%s'. Attempting to resend robot program...",
    set_mode_result.message.c_str());

  // We are running in headless mode, so use
  // /io_and_status_controller/resend_robot_program
  // to restart the external control script.
  CallTriggerService(brake_release_client_, service_timeout_);
  SuccessAndMessage resend_result =
    CallTriggerService(resend_robot_program_client_, service_timeout_);

  if (resend_result.success) {
    RCLCPP_INFO(get_logger(), "Faults cleared via resend_robot_program.");
    PublishOperationalStatus();
    return resend_result;
  }

  PublishOperationalStatus();
  SuccessAndMessage failure_message{
    .success = false,
    .message = "Neither SetMode nor resend_robot_program were able to clear the fault.",
  };

  if (!set_mode_result.message.empty()) {
    failure_message.message += " Setmode error: " + set_mode_result.message + "'.";
  }
  if (!resend_result.message.empty()) {
    failure_message.message += " resend_robot_program error: " + resend_result.message + "'.";
  }
  return failure_message;
}

void UrOperationalStateNode::HandleClearFaults(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*unused*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received clear_faults service request");

  auto error = ClearFaults();
  if (error.success) {
    response->success = true;
    response->message = "Successfully cleared faults and recovered robot.";
  } else {
    response->success = error.success;
    response->message = error.message;
  }
}

}  // namespace ur_operational_state_node
