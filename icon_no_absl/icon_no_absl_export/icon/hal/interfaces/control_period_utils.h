#ifndef ICON_HAL_INTERFACES_CONTROL_PERIOD_UTILS_H_
#define ICON_HAL_INTERFACES_CONTROL_PERIOD_UTILS_H_

#include <chrono>
#include <string>
#include <string_view>

#include "flatbuffers/detached_buffer.h"
#include "flatbuffer_definitions/icon/hal/interfaces/control_period.fbs.h"
#include "icon/hal/hardware_interface_handle.h"
#include "icon/utils/attributes.h"
#include "icon/utils/status.h"

namespace intrinsic_fbs {

// Creates the `ControlPeriod` flatbuffer initialized to an invalid value (0).
// Use `UpdateControlPeriod` to set a value.
INTR_MUST_USE_RESULT flatbuffers::DetachedBuffer BuildControlPeriod();

// Returns a canonical user friendly error message containing the periods in ns,
// as well as the respective frequency in Hertz.
std::string FormatControlPeriodMismatchError(
    std::string_view module_name, std::chrono::nanoseconds expected,
    std::chrono::nanoseconds actual);

}  // namespace intrinsic_fbs

namespace intrinsic::icon {

// Updates the control period in the given ControlPeriod hardware interface handle.
// Returns FailedPrecondition when duration is <= 0.
Status UpdateControlPeriod(
    MutableHardwareInterfaceHandle<intrinsic_fbs::ControlPeriod>& handle,
    std::chrono::nanoseconds duration,
    const log::Logger* logger = nullptr);

}  // namespace intrinsic::icon

#endif  // ICON_HAL_INTERFACES_CONTROL_PERIOD_UTILS_H_
