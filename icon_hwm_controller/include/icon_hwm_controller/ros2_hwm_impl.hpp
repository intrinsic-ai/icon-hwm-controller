#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "tl/expected.hpp"

#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "icon_hwm_controller_msgs/msg/operational_status.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "icon/hal/hardware_interface_handle.h"
#include "icon/hal/hardware_module_init_context.h"
#include "icon/hal/hardware_module_interface.h"
#include "icon/hal/realtime_clock.h"
#include "icon/utils/log.h"
#include "icon/utils/status.h"

#include "flatbuffer_definitions/icon/hal/interfaces/joint_command.fbs.h"
#include "flatbuffer_definitions/icon/hal/interfaces/joint_state.fbs.h"

namespace icon_hwm_controller
{
class Ros2HwmImpl final : public intrinsic::icon::HardwareModuleInterface {
public:
  struct Params
  {
    std::string hardware_component_name;
    size_t num_dofs;
    // Interfaces must be in joint order, and each vector must have either `num_dofs` elements,
    // or (only for velocity interfaces) zero elements.
    //
    // Note that these are pointers, and ownership *stays* with the IconHwmController class!
    std::vector<const hardware_interface::LoanedStateInterface *> position_state_interface_pointers;
    std::vector<const hardware_interface::LoanedStateInterface *> velocity_state_interface_pointers;
    std::vector<hardware_interface::LoanedCommandInterface *> position_command_interface_pointers;
    std::vector<hardware_interface::LoanedCommandInterface *> velocity_command_interface_pointers;
    // If these two are empty, the HWM will only report critical faults, and respond to ClearFaults()
    // calls by trying to re-start its associated HardwareComponent.
    // This is likely not always enough.
    std::string operational_status_topic = "";
    std::string clear_faults_service = "";
    std::vector<std::string> controllers_to_activate;
    std::vector<std::string> controllers_to_deactivate;
    intrinsic::RealtimeClock * clock = nullptr;
    const intrinsic::log::Logger * logger = nullptr;
  };

  // Validates `params` and creates a Ros2HwmImpl instance.
  //
  // That instance saves a copy of `params`, but NOT `interface_registry`.
  // We use `interface_registry` to advertise hardware interfaces, based on `params`, in the Create() function.
  static tl::expected<std::unique_ptr<Ros2HwmImpl>, intrinsic::Status> Create(
    Params params,
    rclcpp_lifecycle::LifecycleNode & node);

  intrinsic::Status Init(intrinsic::icon::HardwareModuleInitContext & context) override;
  intrinsic::Status Prepare() override;

  intrinsic::RealtimeStatus Activate() override;

  intrinsic::RealtimeStatus Deactivate() override;

  intrinsic::Status EnableMotion() override;

  intrinsic::Status DisableMotion() override;

  intrinsic::Status ClearFaults() override;

  intrinsic::Status Shutdown() override;

  intrinsic::RealtimeStatus ReadStatus() override;

  intrinsic::RealtimeStatus ApplyCommand() override;

private:
  Params params_;

  intrinsic::icon::MutableStrictHardwareInterfaceHandle<intrinsic_fbs::JointPositionState>
  joint_position_state_;
  intrinsic::icon::MutableStrictHardwareInterfaceHandle<intrinsic_fbs::JointVelocityState>
  joint_velocity_state_;
  intrinsic::icon::StrictHardwareInterfaceHandle<intrinsic_fbs::JointPositionCommand>
  joint_position_command_;

  // Topic subscription for OperationalStatus
  rclcpp::Subscription<icon_hwm_controller_msgs::msg::OperationalStatus>::SharedPtr
    operational_status_subscription_;
  // RealtimeThreadSafeBox for the latest OperationalStatus value, so the realtime functions can safely read it.
  realtime_tools::RealtimeThreadSafeBox<icon_hwm_controller_msgs::msg::OperationalStatus>
  latest_operational_status_;
  // Service Clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
    clear_faults_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr
    set_hw_state_client_;

  void OnOperationalStatus(const icon_hwm_controller_msgs::msg::OperationalStatus::SharedPtr msg);
  intrinsic::Status CallSwitchController(
    const std::vector<std::string> & activate,
    const std::vector<std::string> & deactivate);
  // Sets the state for the given ROS2 HardwareComponent to `state`.
  intrinsic::Status CallSetHwState(const std::string & name, uint8_t state);
};
}  // namespace icon_hwm_controller
