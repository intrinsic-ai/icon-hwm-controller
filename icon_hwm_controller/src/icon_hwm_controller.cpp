#include "icon_hwm_controller/icon_hwm_controller.hpp"

#include <algorithm>
#include <cmath>
#include <thread>
#include <limits>
#include <utility>

#include "rcutils/logging.h"

#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "icon_hwm_controller_msgs/msg/operational_status.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "icon_hwm_controller/ros2_hwm_impl.hpp"
#include "icon/utils/log.h"
#include "icon/utils/status_and_expected_macros.h"
#include "icon/utils/strerror.h"
#include "icon/utils/time.h"
#include "icon/hal/interfaces/joint_command_utils.h"
#include "icon/hal/interfaces/joint_state_utils.h"
#include "icon/hal/interfaces/joint_limits_utils.h"
#include "icon/hal/interfaces/hardware_module_state_utils.h"
#include "icon/hal/hardware_interface_traits.h"
#include "icon/hal/hardware_module_runtime.h"
#include "icon/hal/hardware_interface_registry.h"
#include "icon/hal/icon_state_register.h"
#include "icon/interprocess/shared_memory_manager/domain_socket_server.h"
#include "icon/interprocess/shared_memory_manager/shared_memory_manager.h"
#include "icon/hal/hardware_module_util.h"

#include "flatbuffer_definitions/icon/hal/interfaces/joint_command.fbs.h"
#include "flatbuffer_definitions/icon/hal/interfaces/joint_state.fbs.h"
#include "flatbuffer_definitions/icon/hal/interfaces/joint_limits.fbs.h"
#include "flatbuffer_definitions/icon/hal/interfaces/hardware_module_state.fbs.h"
#include "flatbuffer_definitions/icon/hal/interfaces/icon_state.fbs.h"
#include "flatbuffer_definitions/icon/interprocess/shared_memory_manager/segment_info.fbs.h"

#include "realtime_tools/realtime_helpers.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "tl/expected.hpp"

namespace icon_hwm_controller
{

using intrinsic::FormatRealtimeStatus;
using intrinsic::FormatStatus;
using intrinsic::OkStatus;
using intrinsic::RealtimeStatus;
using intrinsic::RtOkStatus;
using intrinsic::StatusCode;
using intrinsic::Status;
using intrinsic::ToStatus;
using intrinsic::ToString;


namespace
{
RCUTILS_LOG_SEVERITY ConvertSeverity(intrinsic::log::Logger::Severity severity)
{
  switch(severity) {
    case intrinsic::log::Logger::Severity::kDebug:
      return RCUTILS_LOG_SEVERITY_DEBUG;
    case intrinsic::log::Logger::Severity::kInfo:
      return RCUTILS_LOG_SEVERITY_INFO;
    case intrinsic::log::Logger::Severity::kWarning:
      return RCUTILS_LOG_SEVERITY_WARN;
    case intrinsic::log::Logger::Severity::kError:
      return RCUTILS_LOG_SEVERITY_ERROR;
    case intrinsic::log::Logger::Severity::kFatal:
      return RCUTILS_LOG_SEVERITY_FATAL;
    default:
      return RCUTILS_LOG_SEVERITY_UNSET;
  }
}

// Converts `from_val` to the selected type `To`. If `from_val` is outside the range
// of `To`, clamp its value.
//
// C++26 will have https://cppreference.com/cpp/numeric/saturating_cast to replace this.
//
// Examples:
// // Returns std::numeric_limits<int>::max()
// clamp_cast<int>(std::numeric_limits<unsigned int>::max());
// // Returns 0
// clamp_cast<size_t>(-42);
template<std::integral To, std::integral From> 
constexpr To clamp_cast (From from_val) noexcept {
  if (std::cmp_greater(from_val, std::numeric_limits<To>::max())) {
    return std::numeric_limits<To>::max();
  }
  if (std::cmp_less(from_val, std::numeric_limits<To>::min())) {
    return std::numeric_limits<To>::min();
  }
  return static_cast<To>(from_val);
}
}

IconHwmController::IconHwmController()
    :logger_(intrinsic::log::Logger(intrinsic::log::Logger::Severity::kInfo,
                                    [this](const intrinsic::log::Logger::LogEntry & entry){
                                      auto rutils_severity = ConvertSeverity(entry.severity);
                                      const auto name = get_node()->get_logger().get_name();
                                      if (!rcutils_logging_logger_is_enabled_for(name, rutils_severity)) {
                                        return;
                                      }
                                      rcutils_log_location_t rcutils_logging_location = {
                                        .function_name = entry.loc.function_name(),
                                        .file_name = entry.loc.file_name(),
                                        .line_number = entry.loc.line()
                                      };
                                      rcutils_log(&rcutils_logging_location,
                                                  rutils_severity,
                                                  name,
                                                  "%.*s",
                                                  clamp_cast<int>(entry.msg.size()),
                                                  entry.msg.data());
                                    }))
{
}

controller_interface::InterfaceConfiguration IconHwmController::command_interface_configuration()
    const
{
  controller_interface::InterfaceConfiguration config;
  // By specifying INDIVIDUAL here, we ensure that the interfaces
  // appear in the same order we request them in.
  //
  // See https://github.com/ros-controls/ros2_control/blob/7c5e76766307705b3ef0c28247a17c91814fb311/controller_interface/include/controller_interface/controller_interface_base.hpp#L373-L400
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & dof_name : params_.dof_names) {
    for (const auto & interface_type : params_.command_interfaces) {
      config.names.push_back(dof_name + "/" + interface_type);
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration IconHwmController::state_interface_configuration()
    const
{
  controller_interface::InterfaceConfiguration config;
  // By specifying INDIVIDUAL here, we ensure that the interfaces
  // appear in the same order we request them in.
  //
  // See https://github.com/ros-controls/ros2_control/blob/7c5e76766307705b3ef0c28247a17c91814fb311/controller_interface/include/controller_interface/controller_interface_base.hpp#L373-L400
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & dof_name : params_.dof_names) {
    for (const auto & interface_type : params_.reference_and_state_interfaces) {
      config.names.push_back(dof_name + "/" + interface_type);
    }
  }
  return config;
}

controller_interface::CallbackReturn IconHwmController::on_init()
{
  try {
    param_listener_ = std::make_unique<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s",
                 e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  // add self to `controllers_to_activate` (only if it's not already present)
  if (std::find(params_.controllers_to_activate.begin(),
                params_.controllers_to_activate.end(),
                get_node()->get_name())
      == params_.controllers_to_activate.end()) {
    params_.controllers_to_activate.push_back(get_node()->get_name());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IconHwmController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  if (std::find(params_.controllers_to_activate.begin(),
                params_.controllers_to_activate.end(),
                get_node()->get_name())
      == params_.controllers_to_activate.end()) {
    params_.controllers_to_activate.push_back(get_node()->get_name());
  }

  if (params_.name.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'name' (ICON module name) is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Extract pointers to the state and command interfaces (if our parameters do not have velocity interfaces, leave the vector empty).
  std::vector<const hardware_interface::LoanedStateInterface*> position_state_interface_pointers;
  std::vector<const hardware_interface::LoanedStateInterface*> velocity_state_interface_pointers;
  std::vector<hardware_interface::LoanedCommandInterface*> position_command_interface_pointers;
  std::vector<hardware_interface::LoanedCommandInterface*> velocity_command_interface_pointers;

  if (params_.command_interfaces.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'command_interfaces' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (params_.reference_and_state_interfaces.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'reference_and_state_interfaces' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (params_.dof_names.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'dof_names' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  {
    size_t state_stride = params_.reference_and_state_interfaces.size();
    size_t command_stride = params_.command_interfaces.size();
    for (size_t i = 0; i < params_.dof_names.size(); ++i) {
      position_state_interface_pointers.push_back(
        &(state_interfaces_[i * state_stride]));
      if (state_stride > 1) {
        velocity_state_interface_pointers.push_back(
          &(state_interfaces_[1 + (i * state_stride)]));
      }
      position_command_interface_pointers.push_back(
        &(command_interfaces_[i * command_stride]));
      if (command_stride > 1) {
        velocity_command_interface_pointers.push_back(
          &(command_interfaces_[1 + (i * command_stride)]));
      }
    }
  }

  // Create state publisher
  get_node()->create_publisher<icon_hwm_controller_msgs::msg::HardwareModuleState>("icon_hardware_module_state", 10);

  // Create Shared Memory Manager
  std::string shm_namespace = params_.shm_namespace;
  auto shared_memory_manager = intrinsic::icon::SharedMemoryManager::Create(shm_namespace,
                                                                            params_.name, &logger_);
  if (!shared_memory_manager.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create SharedMemoryManager: %s",
                 shared_memory_manager.error().message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  auto shm_manager = std::move(shared_memory_manager.value());
  // Must be clock driver. If not, return an error.
  if (!params_.drives_realtime_clock) {
    RCLCPP_ERROR(get_node()->get_logger(), "This controller must be a clock driver.");
    return controller_interface::CallbackReturn::ERROR;
  }
  auto clock_res = intrinsic::RealtimeClock::Create(*shm_manager, &logger_);
  if (!clock_res.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create RealtimeClock.");
    return controller_interface::CallbackReturn::ERROR;
  }
  clock_ = std::move(clock_res.value());
  // Create Ros2HwmImpl
  intrinsic::icon::HardwareInterfaceRegistry interface_registry(*shm_manager);
  auto create_impl_result = Ros2HwmImpl::Create(
    Ros2HwmImpl::Params{
    .hardware_component_name=params_.hardware_component_name,
    .num_dofs=params_.dof_names.size(),
    .position_state_interface_pointers=std::move(position_state_interface_pointers),
    .velocity_state_interface_pointers=std::move(velocity_state_interface_pointers),
    .position_command_interface_pointers=std::move(position_command_interface_pointers),
    .velocity_command_interface_pointers=std::move(velocity_command_interface_pointers),
    .operational_status_topic=params_.operational_status_topic,
    .clear_faults_service=params_.clear_faults_trigger_service,
    .controllers_to_activate=params_.controllers_to_activate,
    .controllers_to_deactivate=params_.controllers_to_deactivate,
    .clock=clock_.get(),
    .logger=&logger_,
    },
    *get_node(),
    interface_registry
  ); 
  if (!create_impl_result.has_value()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to create HWM: %s",
      ToString(create_impl_result.error()).c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  auto impl = std::move(create_impl_result.value());

  auto create_hwm_runtime_result = intrinsic::icon::HardwareModuleRuntime::Create(
    /*name=*/params_.name,
    /*control_period=*/std::chrono::nanoseconds(1000),
    /*shared_memory_manager=*/std::move(shm_manager),
    /*hardware_module=*/std::move(impl),
    /*logger=*/&logger_,
    /*exit_code_promise=*/{}
  );
  if (!create_hwm_runtime_result.has_value()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to create HWM runtime: %s",
      ToString(create_hwm_runtime_result.error()).c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  hwm_runtime_ = std::move(create_hwm_runtime_result.value());
  bool has_realtime_kernel = realtime_tools::has_realtime_kernel();
  const auto affinity_as_int = std::vector<int>{params_.cpu_affinity.begin(), params_.cpu_affinity.end()};

  auto run_result = hwm_runtime_->Run(
    /*is_realtime=*/has_realtime_kernel,
    /*cpu_affinity=*/affinity_as_int);
  
  if (!run_result.ok()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to start HWM runtime: %s",
      ToString(run_result).c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  publish_hwm_state_timer_ = get_node()->create_wall_timer(
    std::chrono::seconds(1),
    [this](){PublishCurrentHwmState();});
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IconHwmController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IconHwmController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IconHwmController::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop the state publisher timer
  publish_hwm_state_timer_->cancel();
  publish_hwm_state_timer_ = {};
  // Stop the runtime, if present.
  if(hwm_runtime_ != nullptr) {
    auto stop_result = hwm_runtime_->Stop();

    if (!stop_result.ok()) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to stop HWM runtime: %s",
        ToString(stop_result).c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    hwm_runtime_.reset();
  }
  clock_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type IconHwmController::update(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & period)
{
  if (!clock_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Clock driver not initialized.");
    return controller_interface::return_type::ERROR;
  }
  auto now_shm = intrinsic::Now();
  auto deadline = now_shm + period.to_chrono<std::chrono::nanoseconds>();
  auto tick_result = clock_->TickBlockingWithDeadline(now_shm, deadline);
  if (!tick_result.ok()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                  "Failed to tick the ICON clock in update(). "
                  "Resetting clock, will retry next cycle.");
    if (auto reset_status = clock_->Reset(std::chrono::milliseconds(10));
        !reset_status.ok()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                    "Failed to reset the ICON clock in update().");
      return controller_interface::return_type::OK;
    }
  }

  return controller_interface::return_type::OK;
}

void IconHwmController::PublishCurrentHwmState() {
  if(hwm_runtime_ == nullptr) {
    return;
  }
  icon_hwm_controller_msgs::msg::HardwareModuleState state_msg;
  tl::expected<intrinsic_fbs::HardwareModuleState, Status> hwm_state = hwm_runtime_->GetHardwareModuleState();
  if (!hwm_state.has_value()) {
    return;
  }
  state_msg.code = static_cast<uint8_t>(hwm_state.value().code());
  std::memcpy(
    state_msg.message.data(), hwm_state.value().message()->data(), 
    std::min<size_t>(state_msg.message.size(), hwm_state.value().message()->size()));
  hwm_state_publisher_->publish(state_msg);
}

} // namespace icon_hwm_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    icon_hwm_controller::IconHwmController, controller_interface::ControllerInterface)
