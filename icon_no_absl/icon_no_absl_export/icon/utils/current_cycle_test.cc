#include "icon/utils/current_cycle.h"

#include <cstdint>
#include <limits>

#include "gtest/gtest.h"

namespace intrinsic::icon {

class CurrentCycleTest : public ::testing::Test {
 public:
  CurrentCycleTest() {
    // Initialize the current cycle to zero before every test, since the cycle
    // count uses a static value that would otherwise persist across tests.
    CycleCounter::SetCurrentCycle(0);
  }
};

TEST_F(CurrentCycleTest, StartsAtZero) {
  EXPECT_EQ(CycleCounter::GetCurrentCycle(), 0);
}

TEST_F(CurrentCycleTest, CanIncrement) {
  while (CycleCounter::GetCurrentCycle() < 2000) {
    CycleCounter::IncrementCurrentCycle();
  }
  EXPECT_EQ(CycleCounter::GetCurrentCycle(), 2000);
}

TEST_F(CurrentCycleTest, RollsOver) {
  CycleCounter::SetCurrentCycle(std::numeric_limits<uint64_t>::max());
  CycleCounter::IncrementCurrentCycle();
  EXPECT_EQ(CycleCounter::GetCurrentCycle(), 0);
}

}  // namespace intrinsic::icon
