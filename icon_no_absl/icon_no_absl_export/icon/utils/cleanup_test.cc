#include "icon/utils/cleanup.h"

#include <utility>

#include "gtest/gtest.h"

namespace intrinsic::icon {

TEST(Cleanup, InvokesCallbackOnScopeExit) {
  int callback_count = 0;
  {
    Cleanup c([&]() noexcept { ++callback_count; });
  }
  EXPECT_EQ(callback_count, 1);
}

TEST(Cleanup, CanBeCancelled) {
  int callback_count = 0;
  {
    Cleanup c([&]() noexcept { ++callback_count; });
    std::move(c).Cancel();
  }
  EXPECT_EQ(callback_count, 0);
}

}  // namespace intrinsic::icon
