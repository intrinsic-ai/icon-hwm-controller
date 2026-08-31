#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "icon/utils/attributes.h"
#include "icon/utils/mutex.h"
#include "icon/utils/status.h"
#include "icon_hwm_controller_msgs/msg/operational_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ur_dashboard_msgs/action/set_mode.hpp"
#include "ur_dashboard_msgs/msg/robot_mode.hpp"
#include "ur_dashboard_msgs/msg/safety_mode.hpp"

namespace ur_ros2_icon_hwm
{

// Holds the current robot safety status published by the UR ROS 2  driver.
struct RobotStatus
{
  // Individual optionals since the this information is obtained from three
  // distinct topics.
  std::optional<int8_t> robot_mode;
  std::optional<uint8_t> safety_mode;
  std::optional<bool> program_running;
};

// Evaluates the `OperationalStatus` for ICON from the robot state published by
// the Universal Robots ROS 2 driver.
icon_hwm_controller_msgs::msg::OperationalStatus ToOperationalStatus(
  const RobotStatus & robot_status);

// Monitors UR driver topics, publishes `OperationalStatus` and handles
// `clear_faults` requests.
class UrOperationalStateNode : public rclcpp::Node
{
public:
  using SetModeAction = ur_dashboard_msgs::action::SetMode;
  using SetModeGoalHandle = rclcpp_action::ClientGoalHandle<SetModeAction>;

  explicit UrOperationalStateNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~UrOperationalStateNode() = default;

  // Used to retrieve the current operational status in tests.
  icon_hwm_controller_msgs::msg::OperationalStatus GetOperationalStatus() const;

  // Publishes the current operational status.
  void PublishOperationalStatus();

  // Communicates with robot_state_helper and dashboard_client to clear transient faults.
  intrinsic::Status ClearFaults();

private:
  void ClearFaultsCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void RobotModeCallback(const ur_dashboard_msgs::msg::RobotMode::SharedPtr msg);
  void SafetyModeCallback(const ur_dashboard_msgs::msg::SafetyMode::SharedPtr msg);
  void ProgramRunningCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // Calls a Trigger service and returns the status.
  intrinsic::Status CallTriggerService(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    std::chrono::seconds timeout);

  // Calls the SetMode action on robot_state_helper and returns the result status.
  intrinsic::Status CallSetModeAction(
    int8_t target_robot_mode, bool stop_program,
    bool play_program, std::chrono::seconds timeout);

  mutable intrinsic::Mutex state_mutex_;
  RobotStatus robot_status_ INTR_GUARDED_BY(state_mutex_);
  icon_hwm_controller_msgs::msg::OperationalStatus operational_status_
  INTR_GUARDED_BY(state_mutex_);

  // Callback groups for concurrency.
  rclcpp::CallbackGroup::SharedPtr topic_cb_group_;
  rclcpp::CallbackGroup::SharedPtr clear_faults_service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  rclcpp::Subscription<ur_dashboard_msgs::msg::RobotMode>::SharedPtr robot_mode_sub_;
  rclcpp::Subscription<ur_dashboard_msgs::msg::SafetyMode>::SharedPtr safety_mode_sub_;
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

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resend_robot_program_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr unlock_protective_stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr restart_safety_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr close_safety_popup_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr close_popup_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr brake_release_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_on_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr connect_client_;

  std::chrono::seconds service_timeout_;
  std::chrono::seconds action_timeout_;
};

}  // namespace ur_ros2_icon_hwm
