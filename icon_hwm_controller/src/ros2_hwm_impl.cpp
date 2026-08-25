#include "icon_hwm_controller/ros2_hwm_impl.hpp"

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "icon_hwm_controller_msgs/msg/operational_status.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "icon/hal/hardware_interface_handle.h"
#include "icon/hal/hardware_interface_traits.h"
#include "icon/hal/hardware_module_init_context.h"
#include "icon/hal/hardware_module_interface.h"
#include "icon/hal/interfaces/joint_command_utils.h"
#include "icon/hal/interfaces/joint_limits_utils.h"
#include "icon/hal/interfaces/joint_state_utils.h"
#include "icon/hal/realtime_clock.h"
#include "icon/utils/log.h"
#include "icon/utils/status.h"
#include "icon/utils/status_and_expected_macros.h"

#include "flatbuffer_definitions/icon/hal/interfaces/joint_command.fbs.h"
#include "flatbuffer_definitions/icon/hal/interfaces/joint_state.fbs.h"
#include "flatbuffer_definitions/icon/hal/interfaces/joint_limits.fbs.h"

using intrinsic::FormatRealtimeStatus;
using intrinsic::FormatStatus;
using intrinsic::OkStatus;
using intrinsic::RealtimeStatus;
using intrinsic::RtOkStatus;
using intrinsic::StatusCode;
using intrinsic::Status;
using intrinsic::ToStatus;

namespace intrinsic::icon::hardware_interface_traits
{

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::JointPositionCommand,
                                 intrinsic_fbs::BuildJointPositionCommand,
                                 "intrinsic_fbs.JointPositionCommand")

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::JointCommandedPosition,
                                     intrinsic_fbs::BuildJointCommandedPosition,
                                     "intrinsic_fbs.JointCommandedPosition")

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::JointPositionState,
                                     intrinsic_fbs::BuildJointPositionState,
                                     "intrinsic_fbs.JointPositionState")

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::JointVelocityState,
                                     intrinsic_fbs::BuildJointVelocityState,
                                     "intrinsic_fbs.JointVelocityState")

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::JointLimits,
                                     intrinsic_fbs::BuildJointLimits,
                                     "intrinsic_fbs.JointLimits")

#if 0
INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::HardwareModuleState,
                                     intrinsic_fbs::BuildHardwareModuleState,
                                     "intrinsic_fbs.HardwareModuleState")

INTRINSIC_ADD_HARDWARE_INTERFACE(::intrinsic_fbs::PayloadCommand,
                                         intrinsic_fbs::BuildPayloadCommand,
                                         "intrinsic_fbs.PayloadCommand")

INTRINSIC_ADD_HARDWARE_INTERFACE(::intrinsic_fbs::PayloadState,
                                         intrinsic_fbs::BuildPayloadState,
                                         "intrinsic_fbs.PayloadState")
#endif
}  // namespace intrinsic::icon::hardware_interface_traits


namespace icon_hwm_controller
{
tl::expected<std::unique_ptr<Ros2HwmImpl>, intrinsic::Status> Ros2HwmImpl::Create(
  Params params,
  rclcpp_lifecycle::LifecycleNode & node)
{
  if (params.state_interfaces == nullptr) {
    return tl::unexpected(FormatStatus(
        StatusCode::kInvalidArgument,
        "state_interfaces pointer cannot be null"));
  }
  if (params.command_interfaces == nullptr) {
    return tl::unexpected(FormatStatus(
        StatusCode::kInvalidArgument,
        "command_interfaces pointer cannot be null"));
  }
  if (params.state_stride == 0) {
    return tl::unexpected(FormatStatus(
        StatusCode::kInvalidArgument,
        "state_stride cannot be 0"));
  }
  if (params.command_stride == 0) {
    return tl::unexpected(FormatStatus(
        StatusCode::kInvalidArgument,
        "command_stride cannot be 0"));
  }

  auto impl = std::make_unique<Ros2HwmImpl>();


  // Set up ROS2 comms
  // Set the initial value for the thread safe box either way, so we can use UNKNOWN to indicate that there is no subscription.
  {
    icon_hwm_controller_msgs::msg::OperationalStatus unknown_status;
    unknown_status.state = icon_hwm_controller_msgs::msg::OperationalStatus::UNKNOWN;
    impl->latest_operational_status_.set(std::move(unknown_status));
  }
  if(!params.operational_status_topic.empty()) {
    impl->operational_status_subscription_ = node.create_subscription<icon_hwm_controller_msgs::msg::OperationalStatus>(
        params.operational_status_topic,
        10,
      [impl_ptr =
      impl.get()](const icon_hwm_controller_msgs::msg::OperationalStatus::SharedPtr msg){
        impl_ptr->OnOperationalStatus(msg);
        });
  }
  if (!params.clear_faults_service.empty()) {
    impl->clear_faults_client_ = node.create_client<std_srvs::srv::Trigger>(
        params.clear_faults_service);
  }
  impl->switch_controller_client_ = node.create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");
  impl->set_hw_state_client_ = node.create_client<controller_manager_msgs::srv::SetHardwareComponentState>(
      "/controller_manager/set_hardware_component_state");

  impl->params_ = std::move(params);
  return impl;
}

Status Ros2HwmImpl::Init(intrinsic::icon::HardwareModuleInitContext & context)
{
  // Register ICON HardwareInterfaces
  // Advertise JointPositionState
  // Build a default flatbuffer for JointPositionState with the correct number of DOFs
  INTR_ASSIGN_OR_RETURN_STATUS(
      joint_position_state_,
      context.interface_registry.AdvertiseMutableStrictInterface<intrinsic_fbs::JointPositionState>(
          "joint_position_state", context.logger,
          params_.num_dofs));
  // Advertise JointVelocityState
  INTR_ASSIGN_OR_RETURN_STATUS(
      joint_velocity_state_,
      context.interface_registry.AdvertiseMutableStrictInterface<intrinsic_fbs::JointVelocityState>(
          "joint_velocity_state", context.logger,
          params_.num_dofs));

  // Advertise JointPositionCommand
  INTR_ASSIGN_OR_RETURN_STATUS(
      joint_position_command_,
      context.interface_registry.AdvertiseStrictInterface<intrinsic_fbs::JointPositionCommand>(
          "joint_position_command", context.logger,
          params_.num_dofs));
  return OkStatus();
}

Status Ros2HwmImpl::Prepare()
{
  if (params_.clock != nullptr) {
    INTR_RETURN_STATUS_IF_ERROR(ToStatus(params_.clock->Reset(std::chrono::seconds(20))));
  }

  if (!params_.hardware_component_name.empty()) {
    Status res = CallSetHwState(params_.hardware_component_name, 3); // 3 = ACTIVE
    if (!res.ok()) {
      return
        FormatStatus(
              res.code,
              "Failed to activate hardware component'{}': {}",
              params_.hardware_component_name,
              res.message);
    }
  }
  return CallSwitchController(
      params_.controllers_to_activate,
      params_.controllers_to_deactivate);
}

RealtimeStatus Ros2HwmImpl::Activate()
{
  // If `Prepare()` succeeded, `Activate()` is a no-op.
  return RtOkStatus();
}

RealtimeStatus Ros2HwmImpl::Deactivate()
{
  return RtOkStatus();
}

Status Ros2HwmImpl::EnableMotion()
{
  // If operational status is not happy, refuse to enable (this kicks the HWM into kFaulted)
  auto current_status = latest_operational_status_.get();
  if (current_status.state == icon_hwm_controller_msgs::msg::OperationalStatus::FAULTED) {
    return FormatStatus(StatusCode::kFailedPrecondition, "Cannot enable motion while faulted ({})",
        current_status.message);
  }
  // Otherwise, do nothing
  return OkStatus();
}

Status Ros2HwmImpl::DisableMotion()
{
  // If operational status is not happy, refuse to disable (this kicks the HWM into kFaulted)
  auto current_status = latest_operational_status_.get();
  if (current_status.state == icon_hwm_controller_msgs::msg::OperationalStatus::FAULTED) {
    return FormatStatus(StatusCode::kFailedPrecondition, "Cannot disable motion while faulted ({})",
        current_status.message);
  }
  return OkStatus();
}

Status Ros2HwmImpl::ClearFaults()
{
  // Call ClearFaults service, if present
  if (clear_faults_client_ != nullptr) {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto response = clear_faults_client_->async_send_request(request).get();
    if (!response->success) {
      return FormatStatus(StatusCode::kInternal,
                          "ClearFaults service call failed with message: {}",
                          response->message);
    }
  }


  // Try re-activating the HardwareComponent,
  // and then activating the controller again.
  if (!params_.hardware_component_name.empty()) {
    Status res = CallSetHwState(params_.hardware_component_name, 3); // 3 = ACTIVE
    if (!res.ok()) {
      return
        FormatStatus(
              res.code,
              "ClearFault: Failed to activate hardware component'{}': {}",
              params_.hardware_component_name,
              res.message);
    }
  }
  return CallSwitchController(
      params_.controllers_to_activate,
      params_.controllers_to_deactivate);
}

Status Ros2HwmImpl::Shutdown()
{
  if (params_.clock != nullptr) {
    INTR_RETURN_STATUS_IF_ERROR(ToStatus(params_.clock->Reset(std::chrono::seconds(20))));
  }

  // Deactivate the controllers we activated, and wait until that's done.
  INTR_RETURN_STATUS_IF_ERROR(
      CallSwitchController(
          params_.controllers_to_deactivate,
          params_.controllers_to_activate));

  if (!params_.hardware_component_name.empty()) {
    Status res = CallSetHwState(params_.hardware_component_name, 2); // 2 = INACTIVE
    if (!res.ok()) {
      return FormatStatus(
          res.code,
          "Failed to deactivate hardware component'{}': {}",
          params_.hardware_component_name,
          res.message);
    }
  }
  return OkStatus();
}

RealtimeStatus Ros2HwmImpl::ReadStatus()
{
  if (params_.state_interfaces == nullptr || params_.state_interfaces->empty()) {
    return FormatRealtimeStatus(StatusCode::kUnavailable,
                                "State interfaces not available");
  }
  auto now = intrinsic::Now();
  auto * mutable_pos_state = joint_position_state_.MutableValue();
  auto * pos_vec = mutable_pos_state->mutable_position();
  for (size_t i = 0; i < params_.num_dofs; ++i) {
    auto position_from_ros =
      (*params_.state_interfaces)[i * params_.state_stride].get_optional<double>();
    if (position_from_ros == std::nullopt) {
      return FormatRealtimeStatus(StatusCode::kInternal,
                                  "Failed to read position for joint {}", i);
    }
    pos_vec->Mutate(i, *position_from_ros);
  }
  if (params_.has_velocity_state) {
    auto * mutable_vel_state = joint_velocity_state_.MutableValue();
    auto * vel_vec = mutable_vel_state->mutable_velocity();
    for (size_t i = 0; i < params_.num_dofs; ++i) {
      auto velocity_from_ros =
        (*params_.state_interfaces)[1 + (i * params_.state_stride)].get_optional<double>();
      if (velocity_from_ros == std::nullopt) {
        return FormatRealtimeStatus(StatusCode::kInternal,
                                    "Failed to read velocity for joint {}", i);
      }
      vel_vec->Mutate(i, *velocity_from_ros);
    }
  }
  joint_position_state_.UpdatedAt(now, params_.logger);
  joint_velocity_state_.UpdatedAt(now, params_.logger);
  return RtOkStatus();
}

RealtimeStatus Ros2HwmImpl::ApplyCommand()
{
  {
    // Check OperationalStatus topic and return error if appropriate.
    RealtimeStatus operational_state_status = RtOkStatus();
    // We read with `bool try_get(fn)` here, not `std::optional<T> try_get()`, because the latter makes a copy of the data.
    // OperationalStatus has a string member, and copying that could get arbitrarily expensive.
    auto check_current_status =
      [&operational_state_status](const icon_hwm_controller_msgs::msg::OperationalStatus &
      current_status) {
        switch (current_status.state) {
          case icon_hwm_controller_msgs::msg::OperationalStatus::DISABLED:
            operational_state_status = FormatRealtimeStatus(
              StatusCode::kFailedPrecondition, "Cannot ApplyCommand while disabled");
            break;
          case icon_hwm_controller_msgs::msg::OperationalStatus::FAULTED:
            operational_state_status = FormatRealtimeStatus(
              StatusCode::kFailedPrecondition, "Cannot ApplyCommand while faulted ({})",
            current_status.message);
            break;
          default:
            operational_state_status = RtOkStatus();
        }
      };
    if (latest_operational_status_.try_get(check_current_status)) {
      INTR_RETURN_STATUS_IF_ERROR(operational_state_status);
    }
  }

  // Read from ICON interface
  auto command_from_icon = joint_position_command_.Value();
  if (!command_from_icon.has_value()) {
    // Command not updated this cycle?
    return command_from_icon.error();
  }

  const auto * command_message = command_from_icon.value();
  const auto * pos_vec = command_message->position();
  const auto * vel_vec = command_message->velocity_feedforward();

  if (pos_vec->size() != params_.num_dofs) {
    return FormatRealtimeStatus(
        StatusCode::kInternal,
        "Position command vector size mismatch (expected {}, got{})",
        params_.num_dofs,
        pos_vec->size());
  }
  if (vel_vec->size() != params_.num_dofs) {
    return FormatRealtimeStatus(
        StatusCode::kInternal,
        "Velocity command vector size mismatch (expected {}, got{})",
        params_.num_dofs,
        vel_vec->size());
  }
  if (params_.command_interfaces == nullptr || params_.command_interfaces->empty()) {
    return FormatRealtimeStatus(StatusCode::kUnavailable,
                                "Command interfaces not available");
  }
  for (size_t i = 0; i < pos_vec->size(); ++i) {
    if (!(*params_.command_interfaces)[i * params_.command_stride].set_value<double>(
            pos_vec->Get(i)))
    {
      return FormatRealtimeStatus(
          StatusCode::kInternal,
          "Failed to set position command for joint {}", i);
    }
    if (params_.has_velocity_command) {
      if (!(*params_.command_interfaces)[1 + (i * params_.command_stride)].set_value<double>(
              vel_vec->Get(i)))
      {
        return FormatRealtimeStatus(
            StatusCode::kInternal,
            "Failed to set velocity command for joint {}", i);
      }
    }
  }
  return RtOkStatus();
}

void Ros2HwmImpl::OnOperationalStatus(
  const icon_hwm_controller_msgs::msg::OperationalStatus::SharedPtr msg)
{
  latest_operational_status_.set(*msg);
}

Status Ros2HwmImpl::CallSwitchController(
  const std::vector<std::string> & activate,
  const std::vector<std::string> & deactivate)
{
  if (!switch_controller_client_->wait_for_service(std::chrono::seconds(1))) {
    return {StatusCode::kDeadlineExceeded,
      "SwitchController service did not become available within 1 second"};
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->activate_controllers = activate;
  request->deactivate_controllers = deactivate;
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

  auto result_future = switch_controller_client_->async_send_request(request);

  if (result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    return {StatusCode::kDeadlineExceeded, "SwitchController timeout"};
  }

  auto response = result_future.get();
  if (!response->ok) {
    return {StatusCode::kInternal, "SwitchController failed"};
  }
  return OkStatus();
}

// Sets the state for the given ROS2 HardwareComponent to `state`.
Status Ros2HwmImpl::CallSetHwState(const std::string & name, uint8_t state)
{
  if (!set_hw_state_client_->wait_for_service(std::chrono::seconds(1))) {
    return {StatusCode::kUnavailable, "SetHardwareComponentState service not available"};
  }

  auto request =
    std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
  request->name = name;
  request->target_state.id = state;

  auto result_future = set_hw_state_client_->async_send_request(request);
  if (result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    return {StatusCode::kDeadlineExceeded, "SetHardwareComponentState timeout"};
  }

  auto response = result_future.get();
  if (!response->ok) {
    return {StatusCode::kInternal, "SetHardwareComponentState failed"};
  }
  return OkStatus();
}

}  // namespace icon_hwm_controller
