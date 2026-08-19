#include "icon/hal/hardware_module_util.h"

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "gmock/gmock.h"
#include "flatbuffer_definitions/icon/hal/interfaces/hardware_module_state.fbs.h"
#include "gtest/gtest.h"
#include "icon/utils/status_matchers.h"

namespace intrinsic::icon {
namespace {

using ::testing::HasSubstr;

TEST(HardwareModuleUtilTest, TransitionGuardTransitions) {
  EXPECT_EQ(HardwareModuleTransitionGuard(intrinsic_fbs::StateCode::kPreparing,
                                          intrinsic_fbs::StateCode::kPrepared),
            TransitionGuardResult::kAllowed);
  EXPECT_EQ(HardwareModuleTransitionGuard(intrinsic_fbs::StateCode::kPreparing,
                                          intrinsic_fbs::StateCode::kActivated),
            TransitionGuardResult::kProhibited);
  EXPECT_EQ(
      HardwareModuleTransitionGuard(intrinsic_fbs::StateCode::kDeactivated,
                                    intrinsic_fbs::StateCode::kDeactivating),
      TransitionGuardResult::kNoOp);
  EXPECT_EQ(
      HardwareModuleTransitionGuard(intrinsic_fbs::StateCode::kMotionEnabled,
                                    intrinsic_fbs::StateCode::kFatallyFaulted),
      TransitionGuardResult::kAllowed);
}

TEST(HardwareModuleUtilTest, SharedPromiseWrapperBasic) {
  SharedPromiseWrapper<int> wrapper;
  EXPECT_FALSE(wrapper.HasBeenSet());

  auto future = wrapper.GetSharedFuture();
  EXPECT_TRUE(wrapper.SetValue(42).ok());
  EXPECT_TRUE(wrapper.HasBeenSet());

  EXPECT_EQ(future.get(), 42);

  // Setting again should return an error
  auto status = wrapper.SetValue(100);
  EXPECT_FALSE(status.ok());
}

TEST(HardwareModuleUtilTest, SharedPromiseWrapperMultiThreaded) {
  SharedPromiseWrapper<HardwareModuleExitCode> wrapper;
  auto f1 = wrapper.GetSharedFuture();
  auto f2 = wrapper.GetSharedFuture();

  std::thread t([&wrapper]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_TRUE(
        wrapper.SetValue(HardwareModuleExitCode::kRestartRequested).ok());
  });

  EXPECT_EQ(f1.get(), HardwareModuleExitCode::kRestartRequested);
  EXPECT_EQ(f2.get(), HardwareModuleExitCode::kRestartRequested);

  t.join();
}

TEST(HardwareModuleUtilTest, CreateDotGraphvizStateMachineString) {
  std::string dot = CreateDotGraphvizStateMachineString();
  EXPECT_THAT(dot, HasSubstr("digraph StateMachine"));
  EXPECT_THAT(dot, HasSubstr("kPreparing"));
  EXPECT_THAT(dot, HasSubstr("kPrepared"));
  EXPECT_THAT(dot, HasSubstr("->"));
}

}  // namespace
}  // namespace intrinsic::icon
