#include "icon/utils/status_matchers.h"

#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "icon/utils/status.h"
#include "tl/expected.hpp"

namespace intrinsic::testing {
namespace {

using ::testing::Eq;
using ::testing::HasSubstr;
using ::testing::Not;

TEST(StatusMatchersTest, IsOkWithStatus) {
  Status ok_status = OkStatus();
  EXPECT_THAT(ok_status, IsOk());

  Status err_status{.code = StatusCode::kInternal, .message = "test error"};
  EXPECT_THAT(err_status, Not(IsOk()));
}

TEST(StatusMatchersTest, IsOkWithRealtimeStatus) {
  RealtimeStatus ok_status = RtOkStatus();
  EXPECT_THAT(ok_status, IsOk());

  RealtimeStatus err_status{.code = StatusCode::kInternal};
  EXPECT_THAT(err_status, Not(IsOk()));
}

TEST(StatusMatchersTest, IsOkWithExpected) {
  tl::expected<int, Status> ok_val = 42;
  EXPECT_THAT(ok_val, IsOk());

  tl::expected<int, Status> err_val = tl::make_unexpected(Status{.code = StatusCode::kNotFound});
  EXPECT_THAT(err_val, Not(IsOk()));
}

TEST(StatusMatchersTest, IsOkAndHolds) {
  tl::expected<int, Status> ok_val = 42;
  EXPECT_THAT(ok_val, IsOkAndHolds(Eq(42)));
  EXPECT_THAT(ok_val, IsOkAndHolds(::testing::Ge(40)));

  tl::expected<int, Status> err_val = tl::make_unexpected(Status{.code = StatusCode::kNotFound});
  EXPECT_THAT(err_val, Not(IsOkAndHolds(Eq(42))));
}

TEST(StatusMatchersTest, StatusIsWithStatus) {
  Status err_status{.code = StatusCode::kInvalidArgument, .message = "bad input"};
  EXPECT_THAT(err_status, StatusIs(StatusCode::kInvalidArgument));
  EXPECT_THAT(err_status, StatusIs(StatusCode::kInvalidArgument, HasSubstr("input")));
  EXPECT_THAT(err_status, Not(StatusIs(StatusCode::kInternal)));
}

TEST(StatusMatchersTest, StatusIsWithExpected) {
  tl::expected<int, Status> err_val =
      tl::make_unexpected(Status{.code = StatusCode::kDeadlineExceeded, .message = "timed out"});
  EXPECT_THAT(err_val, StatusIs(StatusCode::kDeadlineExceeded));
  EXPECT_THAT(err_val, StatusIs(StatusCode::kDeadlineExceeded, HasSubstr("timed")));
  EXPECT_THAT(err_val, Not(StatusIs(StatusCode::kOk)));

  tl::expected<int, Status> ok_val = 100;
  EXPECT_THAT(ok_val, Not(StatusIs(StatusCode::kDeadlineExceeded)));
}

}  // namespace
}  // namespace intrinsic::testing
