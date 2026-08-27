#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "fanuc_msgs/msg/robot_status.hpp"
#include "fanuc_msgs/srv/read_error.hpp"
#include "fanuc_msgs/srv/reset.hpp"
#include "fanuc_msgs/srv/switch_control_state.hpp"
#include "icon_hwm_controller_msgs/msg/operational_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace fanuc_operational_state_node
{

// Holds raw telemetry inputs from the FANUC driver.
// Reference: https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/fanuc_driver/motion_control_authority.html#alarm-recovery
struct RobotStateInputs
{
  std::optional<bool> in_error;
  std::optional<bool> tp_enabled;
  std::optional<bool> e_stopped;
  std::optional<bool> motion_possible;
  std::optional<uint8_t> contact_stop_mode;
};

struct SuccessAndMessage
{
  bool success = false;
  std::string message;
};

// Evaluates the OperationalStatus from raw robot state inputs based on FANUC ROS 2 driver states:
// Documentation reference:
//   https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/fanuc_driver/motion_control_authority.html#alarm-recovery
//
// States:
// - Normal Operation:
//     in_error=false, tp_enabled=false, e_stopped=false, motion_possible=true, contact_stop_mode=0 -> ENABLED
// - E-Stopped:
//     in_error=true, tp_enabled=false, e_stopped=true, motion_possible=false, contact_stop_mode=0 -> FAULTED
// - E-Stop Released (Unacknowledged / Alarms active):
//     in_error=true, tp_enabled=false, e_stopped=false, motion_possible=false, contact_stop_mode=0 -> FAULTED
// - Teach Pendant Enabled (Manual control):
//     tp_enabled=true -> DISABLED
// - Contact Stop Active:
//     contact_stop_mode != CONTACT_STOP_MODE_NONE -> FAULTED
// - No Inputs Received:
//     missing required fields -> UNKNOWN
icon_hwm_controller_msgs::msg::OperationalStatus EvaluateOperationalStatus(
  const RobotStateInputs & inputs);

// Monitors FANUC driver topics, publishes OperationalStatus, and handles clear_faults requests.
//
// Alarm Recovery:
// When alarms occur on the robot controller, motion_possible becomes false. The ROS driver
// loses motion control, and the robot controller gains motion control.
// To restart ROS 2 motion control:
// 1. Call /fanuc_gpio_controller/reset (fanuc_msgs/srv/Reset) to reset alarms on the controller.
// 2. Call /fanuc_gpio_controller/switch_control_state (fanuc_msgs/srv/SwitchControlState) with status: 1
//    to switch motion authority back to the ROS 2 driver.
//
// For details, see the official FANUC driver documentation:
// https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/fanuc_driver/motion_control_authority.html#alarm-recovery
class FanucOperationalStateNode : public rclcpp::Node
{
public:
  explicit FanucOperationalStateNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~FanucOperationalStateNode() = default;

  icon_hwm_controller_msgs::msg::OperationalStatus GetLatestStatus() const;

  RobotStateInputs CurrentInputs() const;

  // Evaluates and publishes the current operational status.
  void PublishOperationalStatus();

  // Performs the official FANUC alarm recovery sequence:
  // 1. Calls /fanuc_gpio_controller/reset to reset active alarms on the controller.
  // 2. Calls /fanuc_gpio_controller/switch_control_state with status: 1 to restore ROS 2 motion control.
  // Reference: https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/fanuc_driver/motion_control_authority.html#alarm-recovery
  SuccessAndMessage ClearFaults();

  // Service callback for std_srvs/srv/Trigger 'clear_faults'.
  void HandleClearFaults(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

private:
  // Subscription callback
  void OnRobotStatus(const fanuc_msgs::msg::RobotStatus::SharedPtr msg);

  // State synchronization
  mutable std::mutex state_mutex_;
  RobotStateInputs inputs_;
  icon_hwm_controller_msgs::msg::OperationalStatus latest_status_;

  // Callback groups for concurrency
  rclcpp::CallbackGroup::SharedPtr topic_cb_group_;
  rclcpp::CallbackGroup::SharedPtr clear_faults_service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  // Subscribers and Publishers
  rclcpp::Subscription<fanuc_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Publisher<icon_hwm_controller_msgs::msg::OperationalStatus>::SharedPtr
    operational_status_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Service server for ICON HWM controller
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_faults_srv_;

  // Service clients for FANUC GPIO controller
  rclcpp::Client<fanuc_msgs::srv::Reset>::SharedPtr reset_client_;
  rclcpp::Client<fanuc_msgs::srv::SwitchControlState>::SharedPtr switch_control_state_client_;
  rclcpp::Client<fanuc_msgs::srv::ReadError>::SharedPtr read_error_client_;

  std::chrono::seconds service_timeout_{5};
};

}  // namespace fanuc_operational_state_node
