#include "icon/hal/interfaces/control_period_utils.h"

#include <chrono>
#include <cmath>
#include <format>
#include <limits>
#include <string>
#include <string_view>

#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffer_definitions/icon/hal/interfaces/control_period.fbs.h"
#include "icon/hal/hardware_interface_handle.h"
#include "icon/utils/log.h"
#include "icon/utils/status.h"
#include "icon/utils/time.h"

namespace intrinsic_fbs {

flatbuffers::DetachedBuffer BuildControlPeriod() {
  flatbuffers::FlatBufferBuilder builder;
  builder.Finish(builder.CreateStruct(ControlPeriod(0)));
  return builder.Release();
}

std::string FormatControlPeriodMismatchError(
    std::string_view module_name, std::chrono::nanoseconds expected,
    std::chrono::nanoseconds actual) {
  double expected_hz = std::numeric_limits<double>::quiet_NaN();
  if (expected > std::chrono::nanoseconds::zero()) {
    expected_hz = 1e9 / static_cast<double>(expected.count());
  }

  double actual_hz = std::numeric_limits<double>::quiet_NaN();
  if (actual > std::chrono::nanoseconds::zero()) {
    actual_hz = 1e9 / static_cast<double>(actual.count());
  }

  return std::format(
      "Inconsistent configuration with Hardware Module '{:s}'."
      " ICON ('control_frequency_hz'): {:d} ns ({:.1f} Hz), Hardware Module "
      "reports: {:d} ns ({:.1f} Hz). Check your configuration.",
      module_name, expected.count(), expected_hz,
      actual.count(), actual_hz);
}

}  // namespace intrinsic_fbs

namespace intrinsic::icon {

Status UpdateControlPeriod(
    MutableHardwareInterfaceHandle<intrinsic_fbs::ControlPeriod>& handle,
    std::chrono::nanoseconds duration,
    const log::Logger* logger) {
  if (duration <= std::chrono::nanoseconds::zero()) {
    return FormatStatus(StatusCode::kFailedPrecondition,
                        "Control period must be > 0, got {:d} ns. Check your "
                        "configuration.",
                        duration.count());
  }
  handle->mutate_control_period_ns(duration.count());
  handle.UpdatedAt(Now(), logger);
  return OkStatus();
}

}  // namespace intrinsic::icon
