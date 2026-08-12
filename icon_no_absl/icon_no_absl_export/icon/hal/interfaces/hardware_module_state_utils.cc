#include "icon/hal/interfaces/hardware_module_state_utils.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <string_view>

#include "flatbuffers/detached_buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffer_definitions/icon/hal/interfaces/hardware_module_state.fbs.h"

namespace intrinsic_fbs {

flatbuffers::DetachedBuffer BuildHardwareModuleState() {
  flatbuffers::FlatBufferBuilder builder;
  builder.ForceDefaults(true);

  builder.Finish(builder.CreateStruct(HardwareModuleState()));
  return builder.Release();
}

void SetState(HardwareModuleState* hardware_module_state, StateCode code,
              std::string_view message) {
  if (hardware_module_state == nullptr ||
      hardware_module_state->message() == nullptr) {
    return;
  }
  const size_t copy_length = std::min(
      static_cast<size_t>(hardware_module_state->message()->size() - 1),
      message.size());
  hardware_module_state->mutate_code(code);
  std::memcpy(hardware_module_state->mutable_message()->Data(), message.data(),
              copy_length);
  std::memset(hardware_module_state->mutable_message()->Data() + copy_length,
              '\0',
              hardware_module_state->mutable_message()->size() - copy_length);
}

std::string_view GetMessage(const HardwareModuleState* hardware_module_state) {
  if (hardware_module_state == nullptr ||
      hardware_module_state->message() == nullptr) {
    return "";
  }
  return reinterpret_cast<const char*>(
      hardware_module_state->message()->Data());
}

}  // namespace intrinsic_fbs
