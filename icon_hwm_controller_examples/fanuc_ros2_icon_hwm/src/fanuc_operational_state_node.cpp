#include "fanuc_operational_state_node/fanuc_operational_state_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>

namespace fanuc_operational_state_node
{

// Evaluates OperationalStatus based on FANUC driver status:
// Reference: https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/fanuc_driver/motion_control_authority.html#alarm-recovery
icon_hwm_controller_msgs::msg::OperationalStatus EvaluateOperationalStatus(
  const RobotStateInputs & inputs)
{
  icon_hwm_controller_msgs::msg::OperationalStatus status;

  if (!inputs.in_error.has_value() || !inputs.e_stopped.has_value() ||
      !inputs.motion_possible.has_value())
  {
    status.state = icon_hwm_controller_msgs::msg::OperationalStatus::UNKNOWN;
    status.message = "Waiting for FANUC robot status messages";
    return status;
  }

  const bool in_error = *inputs.in_error;
  const bool e_stopped = *inputs.e_stopped;
  const bool motion_possible = *inputs.motion_possible;
  const bool tp_enabled = inputs.tp_enabled.value_or(false);
  const uint8_t contact_stop_mode = inputs.contact_stop_mode.value_or(
    fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_NONE);

  // 1. E-Stop takes highest priority: e_stopped = true -> FAULTED
  if (e_stopped) {
    status.state = icon_hwm_controller_msgs::msg::OperationalStatus::FAULTED;
    status.message = "FANUC robot emergency stop is active. Release E-stop and call clear_faults.";
    return status;
  }

  // 2. Active controller alarm / error state: in_error = true -> FAULTED
  if (in_error) {
    status.state = icon_hwm_controller_msgs::msg::OperationalStatus::FAULTED;
    status.message = "FANUC robot is in error/alarm state. Call clear_faults to recover.";
    return status;
  }

  // 3. Contact stop check: contact_stop_mode != 0 -> FAULTED
  if (contact_stop_mode != fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_NONE) {
    status.state = icon_hwm_controller_msgs::msg::OperationalStatus::FAULTED;
    if (contact_stop_mode == fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_SAFE) {
      status.message = "FANUC robot contact stop active. Call clear_faults to recover.";
    } else {
      status.message = "FANUC robot contact stop mode " + std::to_string(contact_stop_mode) +
                       " active. Call clear_faults to recover.";
    }
    return status;
  }

  // 4. Teach pendant enabled: tp_enabled = true -> DISABLED (manual control mode)
  if (tp_enabled) {
    status.state = icon_hwm_controller_msgs::msg::OperationalStatus::DISABLED;
    status.message = "FANUC teach pendant is enabled (manual control mode).";
    return status;
  }

  // 5. Motion control lost: motion_possible = false -> FAULTED
  if (!motion_possible) {
    status.state = icon_hwm_controller_msgs::msg::OperationalStatus::FAULTED;
    status.message = "FANUC robot motion_possible is false. Call clear_faults to restore ROS 2 motion control.";
    return status;
  }

  // 6. Normal healthy operation: ENABLED
  status.state = icon_hwm_controller_msgs::msg::OperationalStatus::ENABLED;
  status.message = "";
  return status;
}

FanucOperationalStateNode::FanucOperationalStateNode(const rclcpp::NodeOptions & options)
: Node("fanuc_operational_state_node", options)
{
  const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 10.0);
  const int64_t service_timeout_sec = declare_parameter<int64_t>("service_timeout_sec", 5);

  service_timeout_ = std::chrono::seconds(service_timeout_sec);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_status_.state = icon_hwm_controller_msgs::msg::OperationalStatus::UNKNOWN;
    latest_status_.message = "Waiting for FANUC robot status messages";
  }

  // Create callback groups for concurrency
  topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  clear_faults_service_cb_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = topic_cb_group_;

  // Subscribe to FANUC driver status topic: /fanuc_gpio_controller/robot_status
  robot_status_sub_ = create_subscription<fanuc_msgs::msg::RobotStatus>(
    "/fanuc_gpio_controller/robot_status", rclcpp::SystemDefaultsQoS(),
    std::function<void(const fanuc_msgs::msg::RobotStatus::SharedPtr)>(
      std::bind_front(&FanucOperationalStateNode::OnRobotStatus, this)),
    sub_options);

  // Publish OperationalStatus topic for icon_hwm_controller
  operational_status_pub_ = create_publisher<icon_hwm_controller_msgs::msg::OperationalStatus>(
    "operational_status", rclcpp::SystemDefaultsQoS());

  // Periodic publish timer so subscribers always receive latest status
  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / (publish_rate_hz > 0.0 ? publish_rate_hz : 10.0)));
  publish_timer_ = create_wall_timer(
    timer_period,
    std::function<void()>(
      std::bind_front(&FanucOperationalStateNode::PublishOperationalStatus, this)),
    topic_cb_group_);

  // Expose clear_faults Trigger service for icon_hwm_controller
  clear_faults_srv_ = create_service<std_srvs::srv::Trigger>(
    "clear_faults",
    std::function<void(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response>)>(
      std::bind_front(&FanucOperationalStateNode::HandleClearFaults, this)),
    rclcpp::SystemDefaultsQoS(),
    clear_faults_service_cb_group_);

  // Clients for FANUC controller services
  reset_client_ = create_client<fanuc_msgs::srv::Reset>(
    "/fanuc_gpio_controller/reset", rclcpp::SystemDefaultsQoS(), client_cb_group_);

  switch_control_state_client_ = create_client<fanuc_msgs::srv::SwitchControlState>(
    "/fanuc_gpio_controller/switch_control_state", rclcpp::SystemDefaultsQoS(), client_cb_group_);

  read_error_client_ = create_client<fanuc_msgs::srv::ReadError>(
    "/fanuc_gpio_controller/read_error", rclcpp::SystemDefaultsQoS(), client_cb_group_);

  RCLCPP_INFO(get_logger(), "fanuc_operational_state_node initialized.");
}

icon_hwm_controller_msgs::msg::OperationalStatus FanucOperationalStateNode::GetLatestStatus() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_status_;
}

RobotStateInputs FanucOperationalStateNode::CurrentInputs() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return inputs_;
}

void FanucOperationalStateNode::OnRobotStatus(
  const fanuc_msgs::msg::RobotStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  inputs_.in_error = msg->in_error;
  inputs_.tp_enabled = msg->tp_enabled;
  inputs_.e_stopped = msg->e_stopped;
  inputs_.motion_possible = msg->motion_possible;
  inputs_.contact_stop_mode = msg->contact_stop_mode;

  latest_status_ = EvaluateOperationalStatus(inputs_);
  operational_status_pub_->publish(latest_status_);
}

void FanucOperationalStateNode::PublishOperationalStatus()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_status_ = EvaluateOperationalStatus(inputs_);
  operational_status_pub_->publish(latest_status_);
}

// Executes the official FANUC alarm recovery sequence:
// Reference: https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/fanuc_driver/motion_control_authority.html#alarm-recovery
SuccessAndMessage FanucOperationalStateNode::ClearFaults()
{
  RCLCPP_INFO(get_logger(), "Starting FANUC alarm recovery sequence...");

  // Step 1: Reset active alarms on the robot controller:
  //   ros2 service call /fanuc_gpio_controller/reset fanuc_msgs/srv/Reset
  if (!reset_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(get_logger(), "Service '/fanuc_gpio_controller/reset' is not available");
    return {false, "Service '/fanuc_gpio_controller/reset' is not available"};
  }

  auto reset_req = std::make_shared<fanuc_msgs::srv::Reset::Request>();
  auto reset_future = reset_client_->async_send_request(reset_req);
  if (reset_future.wait_for(service_timeout_) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "Service '/fanuc_gpio_controller/reset' timed out");
    return {false, "Timeout waiting for '/fanuc_gpio_controller/reset'"};
  }

  auto reset_res = reset_future.get();
  if (reset_res->result != 0) {
    std::string err_msg = "reset service failed with error code: " + std::to_string(reset_res->result);
    RCLCPP_WARN(get_logger(), "%s", err_msg.c_str());
    return {false, err_msg};
  }

  RCLCPP_INFO(get_logger(), "Alarms reset successfully. Switching motion control back to ROS 2 (status: 1)...");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Step 2: Switch motion control authority back to ROS 2 driver:
  //   ros2 service call /fanuc_gpio_controller/switch_control_state fanuc_msgs/srv/SwitchControlState "status: 1"
  if (!switch_control_state_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(get_logger(), "Service '/fanuc_gpio_controller/switch_control_state' is not available");
    return {false, "Service '/fanuc_gpio_controller/switch_control_state' is not available"};
  }

  auto switch_req = std::make_shared<fanuc_msgs::srv::SwitchControlState::Request>();
  switch_req->status = 1;  // status: 1 = ROS 2 Driver has motion control

  auto switch_future = switch_control_state_client_->async_send_request(switch_req);
  if (switch_future.wait_for(service_timeout_) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "Service '/fanuc_gpio_controller/switch_control_state' timed out");
    return {false, "Timeout waiting for '/fanuc_gpio_controller/switch_control_state'"};
  }

  auto switch_res = switch_future.get();
  if (switch_res->result != 0) {
    std::string err_msg = "switch_control_state failed with error code: " + std::to_string(switch_res->result);
    RCLCPP_WARN(get_logger(), "%s", err_msg.c_str());
    return {false, err_msg};
  }

  RCLCPP_INFO(get_logger(), "FANUC alarm recovery succeeded: motion control restored to ROS 2.");

  // Update internal status immediately to reflect cleared state
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    inputs_.in_error = false;
    inputs_.e_stopped = false;
    inputs_.motion_possible = true;
    inputs_.contact_stop_mode = fanuc_msgs::msg::RobotStatus::CONTACT_STOP_MODE_NONE;
    latest_status_ = EvaluateOperationalStatus(inputs_);
  }
  PublishOperationalStatus();

  return {true, "Successfully reset alarms and restored ROS 2 motion control."};
}

void FanucOperationalStateNode::HandleClearFaults(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*unused*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received clear_faults service request");

  auto res = ClearFaults();
  response->success = res.success;
  response->message = res.message;
}

}  // namespace fanuc_operational_state_node
