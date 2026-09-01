// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "flatbuffer_definitions/icon/hal/interfaces/hardware_module_state.fbs.h"
#include "flatbuffer_definitions/icon/hal/interfaces/icon_state.fbs.h"
#include "flatbuffer_definitions/icon/hal/interfaces/joint_command.fbs.h"
#include "flatbuffer_definitions/icon/hal/interfaces/joint_state.fbs.h"
#include "icon/hal/hardware_interface_handle.h"
#include "icon/hal/hardware_module_interface.h"
#include "icon/hal/hardware_module_runtime.h"
#include "icon/hal/realtime_clock.h"
#include "icon/interprocess/remote_trigger/remote_trigger_server.h"
#include "icon/interprocess/shared_memory_lockstep/shared_memory_lockstep.h"
#include "icon/interprocess/shared_memory_manager/domain_socket_server.h"
#include "icon/interprocess/shared_memory_manager/shared_memory_manager.h"
#include "icon/utils/attributes.h"
#include "icon/utils/log.h"
#include "icon/utils/mutex.h"
#include "icon_hwm_controller_msgs/msg/hardware_module_state.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "icon_hwm_controller/icon_hwm_controller_parameters.hpp"

namespace icon_hwm_controller
{

class IconHwmController final : public controller_interface::ControllerInterface {
public:
  IconHwmController();

  // controller_interface::ControllerInterface
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // Bridges logs from underlying Intrinsic libraries (HWM runtime, clock) into ROS 2.
  intrinsic::log::Logger logger_;
  // Clock Driver
  std::unique_ptr<intrinsic::RealtimeClock> clock_;
  // Once initialized, hwm_runtime_ holds raw pointers to clock_ and logger_,
  // so clock_ and logger_ must outlive hwm_runtime_ (which they do, since C++ members
  // are destroyed in reverse order of declaration).
  std::unique_ptr<intrinsic::icon::HardwareModuleRuntime> hwm_runtime_;
  // Parameters
  std::unique_ptr<ParamListener> param_listener_;
  Params params_;

  rclcpp::Publisher<icon_hwm_controller_msgs::msg::HardwareModuleState>::SharedPtr
    hwm_state_publisher_;
  rclcpp::TimerBase::SharedPtr publish_hwm_state_timer_;
  // Protects `init_error_state_` against concurrent access between the lifecycle
  // thread (which records fatal initialization/runtime creation failures during
  // `on_configure()`) and the periodic timer callback thread (which reads and
  // publishes the error state via `PublishCurrentHwmState()` when `hwm_runtime_`
  // is null/failed to instantiate).
  intrinsic::Mutex init_error_state_mutex_;
  std::optional<icon_hwm_controller_msgs::msg::HardwareModuleState> init_error_state_
  INTR_GUARDED_BY(init_error_state_mutex_);

  void PublishCurrentHwmState();
};

}
