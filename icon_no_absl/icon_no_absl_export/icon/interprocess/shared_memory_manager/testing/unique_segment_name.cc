#include "icon/interprocess/shared_memory_manager/testing/unique_segment_name.h"

#include <cstdint>
#include <random>
#include <sstream>
#include <string>

namespace intrinsic::icon {
namespace {
std::string UniqueName() {
  std::random_device rd;
  std::default_random_engine engine(rd());
  std::uniform_int_distribution<uint64_t> distrib;
  return (std::stringstream() << std::hex << distrib(engine)).str();
}
}  // namespace

std::string UniqueHardwareModuleName() { return UniqueName(); }

std::string UniqueMemoryNamespace() { return UniqueName(); }

}  // namespace intrinsic::icon
