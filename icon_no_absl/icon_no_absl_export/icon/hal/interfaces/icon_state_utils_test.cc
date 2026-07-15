#include "icon/hal/interfaces/icon_state_utils.h"

#include <cstdint>
#include <limits>

#include "flatbuffers/buffer.h"
#include "flatbuffers/detached_buffer.h"
#include "flatbuffers/verifier.h"
#include "flatbuffer_definitions/icon/hal/interfaces/icon_state.fbs.h"
#include "gtest/gtest.h"

using ::intrinsic_fbs::BuildIconState;
using ::intrinsic_fbs::IconState;

namespace intrinsic::hardware {
namespace {

TEST(IconStateTest, BuildAndSet) {
  flatbuffers::DetachedBuffer buffer = BuildIconState();
  flatbuffers::Verifier verifier(buffer.data(), buffer.size());

  const auto icon_state = flatbuffers::GetMutableRoot<IconState>(buffer.data());
  ASSERT_NE(icon_state, nullptr);

  // Does not initialize to zero
  EXPECT_NE(icon_state->current_cycle(), 0);

  icon_state->mutate_current_cycle(std::numeric_limits<uint64_t>::max() - 42);
  EXPECT_EQ(icon_state->current_cycle(),
            std::numeric_limits<uint64_t>::max() - 42);
}

}  // namespace
}  // namespace intrinsic::hardware
