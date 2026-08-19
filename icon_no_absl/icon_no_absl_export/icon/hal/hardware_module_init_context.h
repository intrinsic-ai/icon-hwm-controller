#ifndef ICON_HAL_HARDWARE_MODULE_INIT_CONTEXT_H_
#define ICON_HAL_HARDWARE_MODULE_INIT_CONTEXT_H_

#include "icon/hal/hardware_interface_registry.h"

namespace intrinsic::icon {

struct HardwareModuleInitContext {
  HardwareInterfaceRegistry& interface_registry;
  const log::Logger* logger = nullptr;
};

}  // namespace intrinsic::icon

#endif  // ICON_HAL_HARDWARE_MODULE_INIT_CONTEXT_H_
