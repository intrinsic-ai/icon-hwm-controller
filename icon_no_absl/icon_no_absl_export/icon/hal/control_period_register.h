#ifndef ICON_HAL_CONTROL_PERIOD_REGISTER_H_
#define ICON_HAL_CONTROL_PERIOD_REGISTER_H_

#include "flatbuffer_definitions/icon/hal/interfaces/control_period.fbs.h"
#include "icon/hal/hardware_interface_traits.h"
#include "icon/hal/interfaces/control_period_utils.h"

namespace intrinsic::icon {

// Reserved name of the control period interface.
inline constexpr char kControlPeriodInterfaceName[] = "control_period";

namespace hardware_interface_traits {

// Registers the ControlPeriod hardware interface.
// Allows transparently depending on ControlPeriod and can be included in
// multiple files.
//
// Usage:
// #include "icon/hal/control_period_register.h"  // IWYU pragma: keep
INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::ControlPeriod,
                                 intrinsic_fbs::BuildControlPeriod,
                                 "intrinsic_fbs.ControlPeriod")
}  // namespace hardware_interface_traits
}  // namespace intrinsic::icon

#endif  // ICON_HAL_CONTROL_PERIOD_REGISTER_H_
