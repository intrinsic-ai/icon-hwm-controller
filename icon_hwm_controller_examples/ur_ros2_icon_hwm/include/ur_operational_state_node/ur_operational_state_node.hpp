#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "icon_hwm_controller_msgs/msg/operational_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ur_dashboard_msgs/action/set_mode.hpp"
#include "ur_dashboard_msgs/msg/robot_mode.hpp"
#include "ur_dashboard_msgs/msg/safety_mode.hpp"

namespace ur_operational_state_node
{

// Holds raw telemetry inputs from the UR driver.
struct RobotStateInputs
{
  std::optional<int8_t> robot_mode;
  std::optional<uint8_t> safety_mode;
  std::optional<bool> program_running;
};

struct SuccessAndMessage
{
  bool success = false;
  std::string message;
};

// Evaluates the OperationalStatus from the raw robot state inputs.
//
// Returns UNKNOWN if any input is missing.
// Returns FAULTED if safety_mode is not NORMAL or REDUCED.
// Returns FAULTED if robot_mode is NO_CONTROLLER, DISCONNECTED, or CONFIRM_SAFETY.
// Returns ENABLED if robot_mode is RUNNING and program_running is true.
// Returns DISABLED if robot_mode is RUNNING and program_running is false.
// Returns DISABLED if robot_mode is BOOTING, POWER_OFF, POWER_ON, IDLE, BACKDRIVE, or UPDATING_FIRMWARE.
// Returns FAULTED for any unrecognized robot mode.
icon_hwm_controller_msgs::msg::OperationalStatus EvaluateOperationalStatus(
  const RobotStateInputs & inputs);

// Monitors UR driver topics, publishes OperationalStatus, and handles clear_faults requests.
class UrOperationalStateNode : public rclcpp::Node {
public:
  using SetModeAction = ur_dashboard_msgs::action::SetMode;
  using SetModeGoalHandle = rclcpp_action::ClientGoalHandle<SetModeAction>;

  explicit UrOperationalStateNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~UrOperationalStateNode() = default;

  icon_hwm_controller_msgs::msg::OperationalStatus GetLatestStatus() const;

  RobotStateInputs CurrentInputs() const;

  // Evaluates and publishes the current operational status.
  void PublishOperationalStatus();

  // Communicates with robot_state_helper and dashboard_client to clear transient faults.
  SuccessAndMessage ClearFaults();

  // Service callback for std_srvs/srv/Trigger 'clear_faults'.
  void HandleClearFaults(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

private:
  // Subscription callbacks
  void OnRobotMode(const ur_dashboard_msgs::msg::RobotMode::SharedPtr msg);
  void OnSafetyMode(const ur_dashboard_msgs::msg::SafetyMode::SharedPtr msg);
  void OnProgramRunning(const std_msgs::msg::Bool::SharedPtr msg);

  // Calls a Trigger service and returns the response success and message.
  SuccessAndMessage CallTriggerService(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    std::chrono::seconds timeout);

  // Calls the SetMode action on robot_state_helper and returns the result.
  SuccessAndMessage CallSetModeAction(
    int8_t target_robot_mode, bool stop_program,
    bool play_program, std::chrono::seconds timeout);

  // State synchronization
  mutable std::mutex state_mutex_;
  RobotStateInputs inputs_;
  icon_hwm_controller_msgs::msg::OperationalStatus latest_status_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr topic_cb_group_;
  rclcpp::CallbackGroup::SharedPtr clear_faults_service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  rclcpp::Subscription<ur_dashboard_msgs::msg::RobotMode>::SharedPtr
    robot_mode_sub_;
  rclcpp::Subscription<ur_dashboard_msgs::msg::SafetyMode>::SharedPtr
    safety_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr program_running_sub_;

  rclcpp::Publisher<icon_hwm_controller_msgs::msg::OperationalStatus>::SharedPtr
    operational_status_pub_;
  // This timer triggers a publisher callback, so that new clients get the latest
  // state even if the state does not change. We could also use QoS parameters to
  // implement "latching", but that requires buy-in from both the publisher and
  // subscriber node(s) and because of that, is easy to get wrong.
  rclcpp::TimerBase::SharedPtr publish_timer_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_faults_srv_;

  rclcpp_action::Client<SetModeAction>::SharedPtr set_mode_action_client_;

  // Service clients for UR driver controllers and dashboard_client
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
    resend_robot_program_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
    unlock_protective_stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr restart_safety_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr close_safety_popup_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr close_popup_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr brake_release_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_on_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr connect_client_;

  std::chrono::seconds service_timeout_{5};
  std::chrono::seconds action_timeout_{15};
};

}  // namespace ur_operational_state_node
