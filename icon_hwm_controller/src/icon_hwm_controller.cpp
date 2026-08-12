#include "icon_hwm_controller/icon_hwm_controller.hpp"

#include <algorithm>
#include <cmath>
#include <thread>

#include "rcutils/logging.h"

#include "icon/utils/log.h"
#include "icon/utils/status_and_expected_macros.h"
#include "icon/utils/strerror.h"
#include "icon/utils/time.h"
#include "icon/hal/interfaces/joint_command_utils.h"
#include "icon/hal/interfaces/joint_state_utils.h"
#include "icon/hal/interfaces/joint_limits_utils.h"
#include "icon/hal/interfaces/hardware_module_state_utils.h"
#include "icon/hal/hardware_interface_traits.h"
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

    INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::HardwareModuleState,
                                     intrinsic_fbs::BuildHardwareModuleState,
                                     "intrinsic_fbs.HardwareModuleState")

#if 0
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

using intrinsic::FormatRealtimeStatus;
using intrinsic::FormatStatus;
using intrinsic::OkStatus;
using intrinsic::RealtimeStatus;
using intrinsic::RtOkStatus;
using intrinsic::StatusCode;
using intrinsic::Status;
using intrinsic::ToStatus;


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
                                                  entry.msg.size(),
                                                  entry.msg.data());
                                    })),
     enable_state_(std::make_shared<std::atomic<EnableState>>(EnableState::kUnknown)),
     disable_state_(DisableState::kUnknown)
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

  state_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<icon_hwm_controller_msgs::msg::HardwareModuleState>>(
      get_node()->create_publisher<icon_hwm_controller_msgs::msg::HardwareModuleState>(
          "hardware_module_state", 10));

  // Create Shared Memory Manager
  std::string shm_namespace = params_.shm_namespace;
  auto shared_memory_manager = intrinsic::icon::SharedMemoryManager::Create(shm_namespace,
                                                                            params_.name, &logger_);
  if (!shared_memory_manager.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create SharedMemoryManager: %s",
                 shared_memory_manager.error().message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  shm_manager_ = std::move(shared_memory_manager.value());
  auto domain_socket_server = intrinsic::icon::DomainSocketServer::Create(
      intrinsic::icon::SocketDirectoryFromNamespace(shm_manager_->SharedMemoryNamespace()),
      shm_manager_->ModuleName(),
      intrinsic::icon::DomainSocketServer::kDefaultLockAcquireTimeout,
      &logger_
                                                                          );
  if (!domain_socket_server.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create DomainSocketServer: %s",
                 domain_socket_server.error().message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  domain_socket_server_ = std::move(domain_socket_server.value());

  intrinsic::icon::HardwareInterfaceRegistry registry(*shm_manager_);
  // Advertise IconState
  auto icon_state =
      registry.AdvertiseInterface<intrinsic_fbs::IconState>(intrinsic::icon::kIconStateInterfaceName,
                                                            &logger_);
  if (!icon_state.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to advertise IconState: %s",
                 icon_state.error().message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  icon_state_ = std::move(icon_state.value());

  // Advertise JointPositionState
  // Build a default flatbuffer for JointPositionState with the correct number of DOFs
  auto joint_position_state = registry.AdvertiseMutableStrictInterface<intrinsic_fbs::JointPositionState>(
      "joint_position_state", &logger_,
      params_.dof_names.size()
                                                                                                          );
  if (!joint_position_state.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to advertise JointPositionState: %s",
                 joint_position_state.error().message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  joint_position_state_ = std::move(joint_position_state).value();

  // Advertise JointVelocityState
  // Build a default flatbuffer for JointVelocityState with the correct number of DOFs
  auto joint_velocity_state = registry.AdvertiseMutableStrictInterface<intrinsic_fbs::JointVelocityState>(
      "joint_velocity_state", &logger_,
      params_.dof_names.size()
                                                                                                          );
  if (!joint_velocity_state.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to advertise JointVelocityState: %s",
                 joint_velocity_state.error().message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  joint_velocity_state_ = std::move(joint_velocity_state).value();

  // Advertise JointPositionCommand
  auto joint_position_command = registry.AdvertiseStrictInterface<intrinsic_fbs::JointPositionCommand>(
      "joint_position_command", &logger_,
      params_.dof_names.size());
  if (!joint_position_command.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to advertise JointPositionCommand: %s",
                 joint_position_command.error().message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  joint_position_command_ = std::move(joint_position_command).value();

  // Advertise HardwareModuleState
  {
    auto hardware_module_state = registry.AdvertiseMutableInterface<intrinsic_fbs::HardwareModuleState>(
        "hardware_module_state", &logger_);
    if (!hardware_module_state.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to advertise HardwareModuleState: %s",
                   hardware_module_state.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    hardware_module_state_ = std::move(hardware_module_state).value();
  }

  // Service Clients
  switch_controller_client_ = get_node()->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
  set_hw_state_client_ = get_node()->create_client<controller_manager_msgs::srv::SetHardwareComponentState>("/controller_manager/set_hardware_component_state");

  // Remote Trigger Servers

  auto lock_memory = params_.lock_memory;
  auto cpu_affinity = params_.cpu_affinity;
  auto realtime_priority_low = params_.realtime_priority_low;
  auto realtime_priority_high = params_.realtime_priority_high;
  if (realtime_priority_low != -1 && realtime_priority_high != -1 &&
      realtime_priority_low > realtime_priority_high)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "realtime_priority_low is greater than realtime_priority_high. Ensure that the parameter configuration sets low to be lower than high.");
    return controller_interface::CallbackReturn::ERROR;
  }
  bool has_realtime_kernel = realtime_tools::has_realtime_kernel();

  auto setup_rt_thread = [ = ](int priority) -> Status {
    if (!has_realtime_kernel) {
      return OkStatus();
    }
    if (lock_memory) {
      auto lock_memory_result = realtime_tools::lock_memory();
      if (!lock_memory_result.first) {
        return Status{
          .code = StatusCode::kInternal,
          .message = (std::stringstream()
                      << "Failed to lock memory: " << lock_memory_result.second).str(),
        };
      }
    }
    if (!cpu_affinity.empty()) {
      const auto affinity_as_int = std::vector<int>{cpu_affinity.begin(), cpu_affinity.end()};
      const auto affinity_result = realtime_tools::set_current_thread_affinity(affinity_as_int);
      if (!affinity_result.first) {
        return Status{
          .code = StatusCode::kInternal,
          .message = (std::stringstream()
                      << "Failed to set thread affinity: " << affinity_result.second).str(),
        };
      }
    }
    if (priority >= 0) {
      if (!realtime_tools::configure_sched_fifo(priority)) {
        return Status{
          .code = StatusCode::kInternal,
          .message = (std::stringstream()
                      << "Failed to set realtime priority with error " << errno
                      << " (" << intrinsic::StrError(errno).data() << ")").str(),
        };
      }
    }
    return OkStatus();
  };
  {
    auto prepare_server_result = intrinsic::icon::RemoteTriggerServer::Create(*shm_manager_,
                                                                              "prepare", &logger_, [this](){(void)Prepare();});
    if (!prepare_server_result.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create prepare server: %s",
                   prepare_server_result.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    prepare_server_ = std::make_unique<intrinsic::icon::RemoteTriggerServer>(
        std::move(prepare_server_result.value()));
  }
  {
    auto activate_server_result = intrinsic::icon::RemoteTriggerServer::Create(*shm_manager_,
                                                                               "activate", &logger_, [this](){(void)Activate();});
    if (!activate_server_result.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create activate server: %s",
                   activate_server_result.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    activate_server_ = std::make_unique<intrinsic::icon::RemoteTriggerServer>(
        std::move(activate_server_result.value()));
  }
  {
    auto deactivate_server_result = intrinsic::icon::RemoteTriggerServer::Create(*shm_manager_,
                                                                                 "deactivate", &logger_, [this](){(void)Deactivate();});
    if (!deactivate_server_result.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create deactivate server: %s",
                   deactivate_server_result.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    deactivate_server_ = std::make_unique<intrinsic::icon::RemoteTriggerServer>(
        std::move(deactivate_server_result.value()));
  }
  {
    auto enable_motion_server_result = intrinsic::icon::RemoteTriggerServer::Create(*shm_manager_,
                                                                                    "enable_motion", &logger_, [this](){(void)EnableMotion();});
    if (!enable_motion_server_result.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create enable_motion server: %s",
                   enable_motion_server_result.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    enable_motion_server_ = std::make_unique<intrinsic::icon::RemoteTriggerServer>(
        std::move(enable_motion_server_result.value()));
  }
  {
    auto disable_motion_server_result = intrinsic::icon::RemoteTriggerServer::Create(*shm_manager_,
                                                                                     "disable_motion", &logger_, [this](){(void)DisableMotion();});
    if (!disable_motion_server_result.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create disable_motion server: %s",
                   disable_motion_server_result.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    disable_motion_server_ = std::make_unique<intrinsic::icon::RemoteTriggerServer>(
        std::move(disable_motion_server_result.value()));
  }
  {
    auto clear_faults_server_result = intrinsic::icon::RemoteTriggerServer::Create(*shm_manager_,
                                                                                   "clear_faults", &logger_, [this](){(void)ClearFaults();});
    if (!clear_faults_server_result.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create clear_faults server: %s",
                   clear_faults_server_result.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    clear_faults_server_ = std::make_unique<intrinsic::icon::RemoteTriggerServer>(
        std::move(clear_faults_server_result.value()));
  }
  {
    auto shutdown_server_result = intrinsic::icon::RemoteTriggerServer::Create(*shm_manager_,
                                                                               "shutdown", &logger_, [this](){(void)Shutdown();});
    if (!shutdown_server_result.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create shutdown server: %s",
                   shutdown_server_result.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    shutdown_server_ = std::make_unique<intrinsic::icon::RemoteTriggerServer>(
        std::move(shutdown_server_result.value()));
  }
  {
    auto read_status_server_result = intrinsic::icon::RemoteTriggerServer::Create(*shm_manager_,
                                                                                  "read_status", &logger_, [this](){
                                                                                    (void)ReadStatus();
                                                                                  });
    if (!read_status_server_result.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create read_status server: %s",
                   read_status_server_result.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    read_status_server_ = std::make_unique<intrinsic::icon::RemoteTriggerServer>(
        std::move(read_status_server_result.value()));
  }
  {
    auto apply_command_server_result = intrinsic::icon::RemoteTriggerServer::Create(*shm_manager_,
                                                                                    "apply_command", &logger_, [this](){
                                                                                      (void)ApplyCommand();
                                                                                    });
    if (!apply_command_server_result.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create apply_command server: %s",
                   apply_command_server_result.error().message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    apply_command_server_ = std::make_unique<intrinsic::icon::RemoteTriggerServer>(
        std::move(apply_command_server_result.value()));
  }
  // Start the background threads for remote trigger servers.
  if (auto start_activate_result = activate_server_->StartAsync(
          &logger_,
          std::bind_front(setup_rt_thread, realtime_priority_low));
      !start_activate_result.ok()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to start Activate() server: %s",
                 intrinsic::ToString(start_activate_result).c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (auto start_deactivate_result = deactivate_server_->StartAsync(
          &logger_,
          std::bind_front(setup_rt_thread, realtime_priority_low));
      !start_deactivate_result.ok()){
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to start Deactivate() server: %s",
                 intrinsic::ToString(start_deactivate_result).c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  // ReadStatus and ApplyCommand run much more often than the others, so they get higher priority.
  if (auto start_read_status_result = read_status_server_->StartAsync(
          &logger_,
          std::bind_front(setup_rt_thread, realtime_priority_high));
      !start_read_status_result.ok()){
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to start ReadStatus() server: %s",
                 intrinsic::ToString(start_read_status_result).c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (auto start_apply_command_result = apply_command_server_->StartAsync(
          &logger_,
          std::bind_front(setup_rt_thread, realtime_priority_high));
      !start_apply_command_result.ok()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to start ApplyCommand() server: %s",
                 intrinsic::ToString(start_apply_command_result).c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  // The remaining servers *MUST NOT* run at realtime priority, since they can block and must not delay the execution of any of the realtime threads.
  auto state_change_query_thread_body = [this](){
    while (!stop_requested_) {
      prepare_server_->Query(&logger_);
      enable_motion_server_->Query(&logger_);
      disable_motion_server_->Query(&logger_);
      clear_faults_server_->Query(&logger_);
      shutdown_server_->Query(&logger_);
    }
  };
  state_change_query_thread_ = std::jthread(state_change_query_thread_body);
  // Must be clock driver. If not, return an error.
  if (!params_.drives_realtime_clock) {
    RCLCPP_ERROR(get_node()->get_logger(), "This controller must be a clock driver.");
    return controller_interface::CallbackReturn::ERROR;
  }
  auto clock_res = intrinsic::RealtimeClock::Create(*shm_manager_, &logger_);
  if (!clock_res.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create RealtimeClock.");
    return controller_interface::CallbackReturn::ERROR;
  }
  clock_ = std::move(clock_res.value());

  // Start the domain socket server
  if (auto s = domain_socket_server_->AddSegmentInfoServeShmDescriptors(*shm_manager_); !s.ok()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to start domain socket server: %s",
                 s.message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IconHwmController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  cycle_counter_ = 0;
  fault_status_ = RtOkStatus();
  auto expected = EnableState::kEnabling;
  bool wrote_enable_succeeded = enable_state_->compare_exchange_strong(
      expected, /*desired=*/EnableState::kEnableSucceeded,
      /*success=*/std::memory_order_acq_rel,
      /*failure=*/std::memory_order_acquire);
  if (!wrote_enable_succeeded) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to write enable_state_. Do not manually activate IconHwmController! It self-activates when ICON requests EnableMotion().");
  }
  RCLCPP_INFO(get_node()->get_logger(), "Activated ICON HWM Controller");
  disable_state_.store(DisableState::kUnknown, std::memory_order_release);
  enable_state_->notify_all();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IconHwmController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto expected = DisableState::kDisabling;
  bool wrote_disable_succeeded = disable_state_.compare_exchange_strong(
      expected, /*desired=*/DisableState::kDisableSucceeded,
      /*success=*/std::memory_order_acq_rel,
      /*failure=*/std::memory_order_acquire);
  if (!wrote_disable_succeeded) {
    fault_status_ = FormatRealtimeStatus(
        StatusCode::kInternal,
        "Received an unexpected call to on_deactivate(). It's likely that something is wrong with the ROS2 driver.");
  }
  enable_state_->store(EnableState::kUnknown, std::memory_order_release);
  disable_state_.notify_all();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IconHwmController::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop all of the servers, join any threads, and reset the unique_ptrs.
  stop_requested_ = true;

  domain_socket_server_.reset();
  clock_.reset();
  state_change_query_thread_.join();
  apply_command_server_.reset();
  read_status_server_.reset();
  shutdown_server_.reset();
  clear_faults_server_.reset();
  disable_motion_server_.reset();
  enable_motion_server_.reset();
  deactivate_server_.reset();
  activate_server_.reset();
  prepare_server_.reset();
  set_hw_state_client_.reset();
  switch_controller_client_.reset();
  hardware_module_state_ = {};
  hardware_module_state_ = {};
  joint_position_command_ = {};
  joint_position_state_ = {};
  icon_state_ = {};
  shm_manager_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type IconHwmController::update(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & period)
{
  cycle_counter_++;

  UpdateHwmState();

  DetectFaults();

  // While we're not active (in ICON terms), just run at the rate that the
  // ControllerManager dictates.
  if (auto current_state = state_code_.load();
      current_state == intrinsic_fbs::StateCode::kActivated ||
      current_state == intrinsic_fbs::StateCode::kMotionEnabling ||
      current_state == intrinsic_fbs::StateCode::kMotionEnabled) {
    // clock_ must not be nullptr here (if it is, the initialization failed)
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
  } else {
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(),
                         *get_node()->get_clock(),
                         2000,
                         "HWM is not active, not ticking ICON");
    // Since ICON is not going to send commands, simply loop back the sensed position with
    // a velocity of zero.
    if (auto read_status_result = ReadStatus(); read_status_result.ok()) {
      // Command robot to stay at current position with zero velocity
      auto * pos_state = joint_position_state_.MutableValue();

      const auto * pos_vec = pos_state->position();

      if (pos_vec->size() != params_.dof_names.size()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Position command vector size mismatch.");
        return controller_interface::return_type::ERROR;
      }

      for (size_t i = 0; i < pos_vec->size(); ++i) {
        (void)command_interfaces_[i * params_.command_interfaces.size()].set_value<double>(
            pos_vec->Get(i));
        if (params_.command_interfaces.size() > 1) {
          (void)command_interfaces_[i * params_.command_interfaces.size() + 1].set_value<double>(0.0);
        }
      }
    } else {
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                  *get_node()->get_clock(),
                                  2000,
                                  "Failed to read status while not enabled: " <<
                                  read_status_result.GetMessage());
    }

  }


  return controller_interface::return_type::OK;
}

void IconHwmController::DetectFaults()
{
  fault_status_ = RtOkStatus();
  for (const auto& state_interface : state_interfaces_) {
    if (std::isnan(state_interface.get_optional<double>().value_or(
            std::numeric_limits<double>::quiet_NaN())))
    {
      fault_status_ = FormatRealtimeStatus(
          StatusCode::kInternal,
          "HardwareInterface '{}' is unavailable or reported a NaN value",
          state_interface.get_name());
      break;
    }
  }
}

Status IconHwmController::Prepare()
{
  INTR_RETURN_STATUS_IF_ERROR(ToStatus(SetStateDirectly(intrinsic_fbs::StateCode::kPreparing)));
  if (clock_ != nullptr) {
    auto res = clock_->Reset(std::chrono::seconds(20));
    if (!res.ok()) {
      // TODO(nilsb): prepend "Failed to reset clock: "
      INTR_RETURN_STATUS_IF_ERROR(
          ToStatus(SetStateDirectly(intrinsic_fbs::StateCode::kFaulted, res)));
      return ToStatus(res);
    }
  }

   if (!params_.hardware_component_name.empty()) {
    Status res = CallSetHwState(params_.hardware_component_name, 3); // 3 = ACTIVE
    if (!res.ok()) {
      INTR_RETURN_STATUS_IF_ERROR(
          ToStatus(SetStateDirectly(
              intrinsic_fbs::StateCode::kFaulted,
              FormatRealtimeStatus(
                  res.code,
                  "Failed to activate hardware component'{}': {}",
                  params_.hardware_component_name,
                  res.message))));
      return res;
    }
  }
  // Activate this controller (if it isn't already active), so that the
  // ControllerManager calls `update()`
  if (enable_state_->load(std::memory_order_acquire) != EnableState::kEnableSucceeded) {
    enable_state_->store(EnableState::kEnabling, std::memory_order_release);
    INTR_RETURN_STATUS_IF_ERROR(CallSwitchController(
        params_.controllers_to_activate,
        params_.controllers_to_deactivate));
  }

  EnableState final_state = enable_state_->load(std::memory_order_acquire);
  if (final_state != EnableState::kEnableSucceeded) {
    auto status = FormatRealtimeStatus(
        StatusCode::kInternal,
        "EnableMotion: Failed to switch controllers");
    INTR_RETURN_STATUS_IF_ERROR(
        ToStatus(SetStateDirectly(intrinsic_fbs::StateCode::kFaulted, status)));

    return ToStatus(status);
  }


  return ToStatus(SetStateDirectly(intrinsic_fbs::StateCode::kPrepared));
}

RealtimeStatus IconHwmController::Activate()
{
  INTR_RETURN_STATUS_IF_ERROR(SetStateDirectly(intrinsic_fbs::StateCode::kActivating));
  return SetStateDirectly(intrinsic_fbs::StateCode::kActivated);
}

RealtimeStatus IconHwmController::Deactivate()
{
  INTR_RETURN_STATUS_IF_ERROR(SetStateDirectly(intrinsic_fbs::StateCode::kDeactivating));
  disable_state_.store(DisableState::kDisabling, std::memory_order_release);

  return SetStateDirectly(intrinsic_fbs::StateCode::kDeactivated);
}

Status IconHwmController::EnableMotion()
{
  INTR_RETURN_STATUS_IF_ERROR(
      ToStatus(SetStateDirectly(intrinsic_fbs::StateCode::kMotionEnabling)));
 
  RCLCPP_INFO(get_node()->get_logger(), "EnableMotion succeeded");
  return ToStatus(SetStateDirectly(intrinsic_fbs::StateCode::kMotionEnabled));
}

Status IconHwmController::DisableMotion()
{
  INTR_RETURN_STATUS_IF_ERROR(
      ToStatus(SetStateDirectly(intrinsic_fbs::StateCode::kMotionDisabling)));

  INTR_RETURN_STATUS_IF_ERROR(
      ToStatus(SetStateDirectly(intrinsic_fbs::StateCode::kActivated)));
  return OkStatus();
}

Status IconHwmController::ClearFaults()
{
  INTR_RETURN_STATUS_IF_ERROR(ToStatus(SetStateDirectly(intrinsic_fbs::StateCode::kClearingFaults)));
  INTR_RETURN_STATUS_IF_ERROR(ToStatus(SetStateDirectly(intrinsic_fbs::StateCode::kActivated)));
  fault_status_ = RtOkStatus();
  return OkStatus();
}

Status IconHwmController::Shutdown()
{
  if (clock_ != nullptr) {
    INTR_RETURN_STATUS_IF_ERROR(ToStatus(clock_->Reset(std::chrono::seconds(20))));
  }

  // Deactivate the controllers we activated, and wait until that's done.
  INTR_RETURN_STATUS_IF_ERROR(
      CallSwitchController(
          params_.controllers_to_deactivate,
          params_.controllers_to_activate));

  Status res = CallSetHwState(params_.hardware_component_name, 2); // 2 = INACTIVE
  if (!res.ok()) {
    INTR_RETURN_STATUS_IF_ERROR(
        ToStatus(SetStateDirectly(
            intrinsic_fbs::StateCode::kFaulted,
            FormatRealtimeStatus(
                res.code,
                "Failed to deactivate hardware component'{}': {}",
                params_.hardware_component_name,
                res.message))));
    return res;
  }
  return ToStatus(SetStateDirectly(intrinsic_fbs::StateCode::kDeactivated));
}

RealtimeStatus IconHwmController::ReadStatus()
{
  auto now = intrinsic::Now();
  auto * mutable_pos_state = joint_position_state_.MutableValue();
  auto * pos_vec = mutable_pos_state->mutable_position();

  auto * mutable_vel_state = joint_velocity_state_.MutableValue();
  auto * vel_vec = mutable_vel_state->mutable_velocity();

  size_t num_dofs = params_.dof_names.size();
  for (size_t i = 0; i < num_dofs; ++i) {
    // Interfaces in `state_interfaces_` are in the same order that we listed our desired
    // interfaces in state_interface_configuration()
    //
    // That is, joints appear in the order they do in the configuration,
    // and for each joint, the state interfaces (usually position and velocity) do the same.
    auto pos_val = state_interfaces_[i * params_.reference_and_state_interfaces.size()]
                   .get_optional<double>()
                   .value_or(
                       std::numeric_limits<double>::quiet_NaN());
    pos_vec->Mutate(i, pos_val);

    auto vel_val = state_interfaces_[i * params_.reference_and_state_interfaces.size() + 1]
                   .get_optional<double>()
                   .value_or(std::numeric_limits<double>::quiet_NaN());
    vel_vec->Mutate(i, vel_val);
  }

  joint_position_state_.UpdatedAt(now, &logger_);
  joint_velocity_state_.UpdatedAt(now, &logger_);

  return RtOkStatus();
}

RealtimeStatus IconHwmController::ApplyCommand()
{
  if (!fault_status_.ok()) {
    return fault_status_;
  }

  auto cmd_res = joint_position_command_.Value();
  if (!cmd_res.has_value()) {
    // Command not updated this cycle?
    return cmd_res.error();
  }

  const auto * cmd = cmd_res.value();
  const auto * pos_vec = cmd->position();
  const auto * vel_vec = cmd->velocity_feedforward();

  if (pos_vec->size() != params_.dof_names.size()) {
    return {StatusCode::kInternal, "Position command vector size mismatch."};
  }

  if (vel_vec->size() != params_.dof_names.size()) {
    return {StatusCode::kInternal, "Velocity command vector size mismatch."};
  }

  for (size_t i = 0; i < pos_vec->size(); ++i) {
    if (!command_interfaces_[i * params_.command_interfaces.size()].set_value<double>(
        pos_vec->Get(i))) {
      return FormatRealtimeStatus(
        StatusCode::kInternal,
        "Failed to set position command to joint {}", i);
    }
    if (params_.command_interfaces.size() > 1) {
      if (!command_interfaces_[i * params_.command_interfaces.size() + 1].set_value<double>(
          vel_vec->Get(i))) {
        return FormatRealtimeStatus(
          StatusCode::kInternal,
          "Failed to set velocity command to joint {}", i);
      }
    }
  }
  return RtOkStatus();
}

void IconHwmController::UpdateHwmState()
{
  auto * mutable_state = *hardware_module_state_;

  mutable_state->mutate_code(state_code_.load());

  // Update message if faulted
  if (state_code_.load() == intrinsic_fbs::StateCode::kFaulted ||
      state_code_.load() == intrinsic_fbs::StateCode::kFatallyFaulted)
  {
    auto & msg_bytes = *mutable_state->mutable_message();
    std::string_view fault_message = fault_status_.GetMessage();
    if (msg_bytes.size() > 1) {
      size_t len = std::min(fault_message.length(), static_cast<size_t>(msg_bytes.size() - 1));

      for (size_t i = 0; i < len; ++i) {
        msg_bytes.Mutate(i, fault_message[i]);
      }
      for (size_t i = len; i < msg_bytes.size(); ++i) {
        msg_bytes.Mutate(i, 0);
      }
    }
  }

  hardware_module_state_.UpdatedAt(intrinsic::Now(), &logger_);
}

// Sets the internal state *and* the state in shared memory directly. Only
// call this when you *know* that no other thread/process might be reading
// the state in shared memory at the same time.
// Returns an error if the transition from the current state to `state` is prohibited.
RealtimeStatus IconHwmController::SetStateDirectly(
    intrinsic_fbs::StateCode state,
    RealtimeStatus fault_status,
    bool force,
    bool silent)
{
  auto current_state = state_code_.load();
  auto guard_res = intrinsic::icon::HardwareModuleTransitionGuard(current_state, state);
  if (!force && guard_res != intrinsic::icon::TransitionGuardResult::kAllowed) {
    if (!silent && guard_res == intrinsic::icon::TransitionGuardResult::kProhibited) {
      RCLCPP_ERROR(get_node()->get_logger(), "Switching from %s to %s is prohibited!",
                   intrinsic_fbs::EnumNameStateCode(current_state),
                   intrinsic_fbs::EnumNameStateCode(state));
    }
    return FormatRealtimeStatus(
        StatusCode::kFailedPrecondition,
        "Switching from {} to {} is prohibited!",
        intrinsic_fbs::EnumNameStateCode(current_state),
        intrinsic_fbs::EnumNameStateCode(state));
  }

  const bool state_changed = current_state != state;
  if (!silent && state_changed) {
    if (fault_status.ok()) {
      RCLCPP_INFO(get_node()->get_logger(), "Switching from %s to %s",
                  intrinsic_fbs::EnumNameStateCode(current_state),
                  intrinsic_fbs::EnumNameStateCode(state));
    } else {
      std::string_view fault_message = fault_status.GetMessage();
      RCLCPP_INFO(get_node()->get_logger(), "Switching from %s to %s with message '%.*s'",
                  intrinsic_fbs::EnumNameStateCode(current_state),
                  intrinsic_fbs::EnumNameStateCode(state),
                  static_cast<int>(fault_message.size()), fault_message.data());
    }
  }

  if (state_changed) {
    want_to_publish_state_ = true;
  }
  if (want_to_publish_state_ && state_publisher_) {
    icon_hwm_controller_msgs::msg::HardwareModuleState msg;
    msg.code = static_cast<uint8_t>(state);
    std::string_view fault_message = fault_status.GetMessage();
    size_t len = std::min(fault_message.length(), (size_t)msg.message.size());
    for (size_t i = 0; i < len; ++i) {
      msg.message[i] = fault_message.data()[i];
    }
    for (size_t i = len; i < 256; ++i) {
      msg.message[i] = 0;
    }
    if(state_publisher_->try_publish(msg)) {
      want_to_publish_state_ = false;
    }
  }

  if (!state_changed && fault_status_.code == fault_status.code && fault_status_.GetMessage() == fault_status.GetMessage()) {
    // Don't update timestamp when state and message is the same as before.
    return RtOkStatus();
  }
  // TODO(nilsb): When transitioning *away* from kMotionEnabled, make sure we take
  // appropriate action to disable. Currently this class is a prototype that pretty much
  // treats all transitions as no-ops, but in the future we'll need to do something.
  state_code_.store(state);
  fault_status_ = fault_status;

  if (*hardware_module_state_ != nullptr) {
    auto * mutable_state = *hardware_module_state_;
    mutable_state->mutate_code(state);

    auto * msg_bytes = mutable_state->mutable_message();
    const size_t max_len = msg_bytes->size();
    std::string_view fault_message = fault_status.GetMessage();
    const size_t len = std::min(fault_message.length(), max_len);
    std::memset(msg_bytes->data(), 0, max_len);
    std::memcpy(msg_bytes->data(), fault_message.data(), len);

    hardware_module_state_.UpdatedAt(intrinsic::Now(), &logger_);
  }

  return RtOkStatus();;
}

Status IconHwmController::CallSwitchController(
    const std::vector<std::string> & activate,
    const std::vector<std::string> & deactivate)
{
  if (!switch_controller_client_->wait_for_service(std::chrono::seconds(1))) {
    return {};
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

Status IconHwmController::CallSetHwState(const std::string & name, uint8_t state)
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

} // namespace icon_hwm_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    icon_hwm_controller::IconHwmController, controller_interface::ControllerInterface)
