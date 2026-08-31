#include "icon_hwm_controller/icon_hwm_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ratio>
#include <thread>
#include <utility>

#include "rcutils/logging.h"

#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "icon_hwm_controller_msgs/msg/operational_status.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "icon_hwm_controller/ros2_hwm_impl.hpp"
#include "icon/hal/hardware_interface_traits.h"
#include "icon/hal/hardware_module_runtime.h"
#include "icon/hal/hardware_module_util.h"
#include "icon/hal/icon_state_register.h"
#include "icon/interprocess/shared_memory_manager/domain_socket_server.h"
#include "icon/interprocess/shared_memory_manager/shared_memory_manager.h"
#include "icon/hal/interfaces/joint_command_utils.h"
#include "icon/hal/interfaces/joint_state_utils.h"
#include "icon/hal/interfaces/joint_limits_utils.h"
#include "icon/hal/interfaces/hardware_module_state_utils.h"
#include "icon/utils/log.h"
#include "icon/utils/mutex.h"
#include "icon/utils/status_and_expected_macros.h"
#include "icon/utils/strerror.h"
#include "icon/utils/time.h"

#include "flatbuffer_definitions/icon/hal/interfaces/hardware_module_state.fbs.h"
#include "flatbuffer_definitions/icon/hal/interfaces/icon_state.fbs.h"
#include "flatbuffer_definitions/icon/hal/interfaces/joint_command.fbs.h"
#include "flatbuffer_definitions/icon/hal/interfaces/joint_state.fbs.h"
#include "flatbuffer_definitions/icon/hal/interfaces/joint_limits.fbs.h"
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
constexpr To clamp_cast(From from_val) noexcept
{
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
                get_node()->get_name()) ==
    params_.controllers_to_activate.end())
  {
    params_.controllers_to_activate.push_back(get_node()->get_name());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IconHwmController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Clear init_error_state_
  {
    intrinsic::MutexLock l(init_error_state_mutex_);
    init_error_state_ = std::nullopt;
  }
  // Create state publisher
  hwm_state_publisher_ = get_node()->create_publisher<icon_hwm_controller_msgs::msg::HardwareModuleState>(
    "icon_hardware_module_state", 10);

  publish_hwm_state_timer_ = get_node()->create_wall_timer(
    std::chrono::seconds(1),
    [this](){PublishCurrentHwmState();});

  auto log_and_save_init_error = [this](std::string error_string){
      RCLCPP_ERROR(get_node()->get_logger(), "%s", error_string.c_str());
      intrinsic::MutexLock l(init_error_state_mutex_);
      init_error_state_.emplace();
      init_error_state_->code = icon_hwm_controller_msgs::msg::HardwareModuleState::INIT_FAILED;
      std::strncpy(
        // `message` is an array of *unsigned* chars...
        reinterpret_cast<char *>(init_error_state_->message.data()),
        error_string.c_str(),
        init_error_state_->message.size());
      // Zero-terminate in any case. If `error_string` was shorter than `message`,
      // then `strncpy()` has already filled the remainder of `message` with zeroes.
      // But if `error_string` was bigger than `message`, we need to manually set the
      // last element of `message` to zero.
      init_error_state_->message.at(init_error_state_->message.size() - 1) = 0;
    };
  if (params_.name.empty()) {
    log_and_save_init_error("Parameter 'name' (ICON module name) is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.command_interfaces.empty()) {
    log_and_save_init_error("Parameter 'command_interfaces' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (params_.reference_and_state_interfaces.empty()) {
    log_and_save_init_error("Parameter 'reference_and_state_interfaces' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (params_.dof_names.empty()) {
    log_and_save_init_error("Parameter 'dof_names' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (params_.control_frequency_hz > 0 &&
      static_cast<unsigned int>(params_.control_frequency_hz) != get_update_rate()) {
    log_and_save_init_error(
      std::format("Parameter 'control_frequency_hz' ({}) does not match update_rate ({}).",
                  params_.control_frequency_hz, get_update_rate()));
    return controller_interface::CallbackReturn::ERROR;
  }
  size_t state_stride = params_.reference_and_state_interfaces.size();
  size_t command_stride = params_.command_interfaces.size();

  // Create Shared Memory Manager
  std::string shm_namespace = params_.shm_namespace;
  auto shared_memory_manager = intrinsic::icon::SharedMemoryManager::Create(shm_namespace,
                                                                            params_.name, &logger_);
  if (!shared_memory_manager.has_value()) {
    log_and_save_init_error(
      std::format("Failed to create SharedMemoryManager: {}",
                  shared_memory_manager.error().message));
    return controller_interface::CallbackReturn::ERROR;
  }
  auto shm_manager = std::move(shared_memory_manager.value());
  // Must be clock driver. If not, return an error.
  if (!params_.drives_realtime_clock) {
    log_and_save_init_error("IconHwmController must be a clock driver.");
    return controller_interface::CallbackReturn::ERROR;
  }
  auto clock_res = intrinsic::RealtimeClock::Create(*shm_manager, &logger_);
  if (!clock_res.has_value()) {
    log_and_save_init_error("Failed to create RealtimeClock.");
    return controller_interface::CallbackReturn::ERROR;
  }
  clock_ = std::move(clock_res.value());
  // Create Ros2HwmImpl
  auto create_impl_result = Ros2HwmImpl::Create(
    Ros2HwmImpl::Params{
      .hardware_component_name = params_.hardware_component_name,
      .num_dofs = params_.dof_names.size(),
      .state_interfaces = &state_interfaces_,
      .command_interfaces = &command_interfaces_,
      .state_stride = state_stride,
      .command_stride = command_stride,
      .has_velocity_state = (state_stride > 1),
      .has_velocity_command = (command_stride > 1),
      .operational_status_topic = params_.operational_status_topic,
      .clear_faults_service = params_.clear_faults_trigger_service,
      .controllers_to_activate = params_.controllers_to_activate,
      .controllers_to_deactivate = params_.controllers_to_deactivate,
      .clock = clock_.get(),
      .logger = &logger_,
    },
    *get_node()
  );
  if (!create_impl_result.has_value()) {
    log_and_save_init_error(
      std::format("Failed to create HWM: {}",
                  ToString(create_impl_result.error())));
    return controller_interface::CallbackReturn::ERROR;
  }
  auto impl = std::move(create_impl_result.value());

  auto create_hwm_runtime_result = intrinsic::icon::HardwareModuleRuntime::Create(
    /*name=*/params_.name,
    /*control_period=*/std::chrono::nanoseconds(std::nano::den / get_update_rate()),
    /*shared_memory_manager=*/std::move(shm_manager),
    /*hardware_module=*/std::move(impl),
    /*logger=*/&logger_,
    /*exit_code_promise=*/{}
  );
  if (!create_hwm_runtime_result.has_value()) {
    log_and_save_init_error(
      std::format("Failed to create HWM runtime:  {}",
                  ToString(create_hwm_runtime_result.error())));
    return controller_interface::CallbackReturn::ERROR;
  }
  hwm_runtime_ = std::move(create_hwm_runtime_result.value());
  bool has_realtime_kernel = realtime_tools::has_realtime_kernel();
  const auto affinity_ints = std::vector<int>{params_.cpu_affinity.begin(),
    params_.cpu_affinity.end()};

  auto run_result = hwm_runtime_->Run(
    /*is_realtime=*/has_realtime_kernel,
    /*cpu_affinity=*/affinity_ints);

  if (!run_result.ok()) {
    log_and_save_init_error(
      std::format("Failed to start HWM runtime: {}",
                  ToString(run_result)));
    return controller_interface::CallbackReturn::ERROR;
  }

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
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Clock driver not initialized.");
    return controller_interface::return_type::ERROR;
  }
  auto now_shm = intrinsic::Now();
  auto deadline = now_shm + period.to_chrono<std::chrono::nanoseconds>();
  auto tick_result = clock_->TickBlockingWithDeadline(now_shm, deadline);
  if (!tick_result.ok()) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Failed to tick the ICON clock in update(). "
      "Resetting clock, will retry next cycle.");
    if (auto reset_status = clock_->Reset(std::chrono::milliseconds(10));
      !reset_status.ok())
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to reset the ICON clock in update().");
      // TODO: Evaluate if we want to return OK for specific reason like
      // keeping the runtime alive, or if we should return ERROR instead.
      return controller_interface::return_type::OK;
    }
  }

  return controller_interface::return_type::OK;
}

void IconHwmController::PublishCurrentHwmState()
{
  icon_hwm_controller_msgs::msg::HardwareModuleState state_msg;
  if(hwm_runtime_ == nullptr) {
    {
      intrinsic::MutexLock l(init_error_state_mutex_);
      if (!init_error_state_.has_value()) {
        return;
      }
      state_msg = *init_error_state_;
    }
  } else {
    tl::expected<intrinsic_fbs::HardwareModuleState, Status> hwm_state =
      hwm_runtime_->GetHardwareModuleState();
    if (!hwm_state.has_value()) {
      return;
    }
    state_msg.code = static_cast<uint8_t>(hwm_state.value().code());
    std::memcpy(
      state_msg.message.data(), hwm_state.value().message()->data(),
      std::min<size_t>(state_msg.message.size(), hwm_state.value().message()->size()));
  }
  hwm_state_publisher_->publish(state_msg);
}

} // namespace icon_hwm_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    icon_hwm_controller::IconHwmController, controller_interface::ControllerInterface)
