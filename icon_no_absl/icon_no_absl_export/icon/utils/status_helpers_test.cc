#include "icon/utils/status_helpers.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace intrinsic {
namespace {

TEST(OverwriteIfError, ReturnsOkIfBothAreOk) {
  RealtimeStatus realtime_status = RtOkStatus();
  realtime_status = OverwriteIfError(realtime_status, RtOkStatus());
  EXPECT_TRUE(realtime_status.ok());
}

TEST(OverwriteIfError, ReturnsPreviousErrorIfNewStatusIsOk) {
  Status status = FormatStatus(StatusCode::kInvalidArgument, "");
  status = OverwriteIfError(status, OkStatus());
  EXPECT_EQ(status.code, StatusCode::kInvalidArgument);

  RealtimeStatus realtime_status =
      FormatRealtimeStatus(StatusCode::kInvalidArgument, "");
  realtime_status = OverwriteIfError(realtime_status, RtOkStatus());
  EXPECT_EQ(realtime_status.code, StatusCode::kInvalidArgument);
}

TEST(OverwriteIfError, ReturnsNewErrorIfPreviousIsOk) {
  Status status = OkStatus();
  status =
      OverwriteIfError(status, FormatStatus(StatusCode::kInvalidArgument, ""));
  EXPECT_EQ(status.code, StatusCode::kInvalidArgument);

  RealtimeStatus realtime_status = RtOkStatus();
  realtime_status = OverwriteIfError(
      realtime_status, FormatRealtimeStatus(StatusCode::kInvalidArgument, ""));
  EXPECT_EQ(realtime_status.code, StatusCode::kInvalidArgument);
}

TEST(OverwriteIfError, ReturnsNewErrorIfPreviousIsNotOk) {
  Status status = FormatStatus(StatusCode::kUnavailable, "");
  status =
      OverwriteIfError(status, FormatStatus(StatusCode::kInvalidArgument, ""));
  EXPECT_EQ(status.code, StatusCode::kInvalidArgument);

  RealtimeStatus realtime_status =
      FormatRealtimeStatus(StatusCode::kUnavailable, "");
  realtime_status = OverwriteIfError(
      realtime_status, FormatRealtimeStatus(StatusCode::kInvalidArgument, ""));
  EXPECT_EQ(realtime_status.code, StatusCode::kInvalidArgument);
}

TEST(OverwriteIfNotInError, ReturnsOkIfBothAreOk) {
  Status status = OkStatus();
  status = OverwriteIfNotInError(status, OkStatus());
  EXPECT_TRUE(status.ok());

  RealtimeStatus realtime_status = RtOkStatus();
  realtime_status = OverwriteIfNotInError(realtime_status, RtOkStatus());
  EXPECT_TRUE(realtime_status.ok());
}

TEST(OverwriteIfNotInError, ReturnsPreviousErrorIfNewStatusIsOk) {
  Status status = FormatStatus(StatusCode::kInvalidArgument, "");
  status = OverwriteIfNotInError(status, OkStatus());
  EXPECT_EQ(status.code, StatusCode::kInvalidArgument);

  RealtimeStatus realtime_status =
      FormatRealtimeStatus(StatusCode::kInvalidArgument, "");
  realtime_status = OverwriteIfNotInError(realtime_status, RtOkStatus());
  EXPECT_EQ(realtime_status.code, StatusCode::kInvalidArgument);
}

TEST(OverwriteIfNotInError, ReturnsNewErrorIfPreviousIsOk) {
  Status status = OkStatus();
  status = OverwriteIfNotInError(
      status, FormatStatus(StatusCode::kInvalidArgument, ""));
  EXPECT_EQ(status.code, StatusCode::kInvalidArgument);

  RealtimeStatus realtime_status = RtOkStatus();
  realtime_status = OverwriteIfNotInError(
      realtime_status, FormatRealtimeStatus(StatusCode::kInvalidArgument, ""));
  EXPECT_EQ(realtime_status.code, StatusCode::kInvalidArgument);
}

TEST(OverwriteIfNotInError, ReturnsPreviousErrorIfNewStatusIsNotOk) {
  Status status = FormatStatus(StatusCode::kUnavailable, "");
  status = OverwriteIfNotInError(
      status, FormatStatus(StatusCode::kInvalidArgument, ""));
  EXPECT_EQ(status.code, StatusCode::kUnavailable);

  RealtimeStatus realtime_status =
      FormatRealtimeStatus(StatusCode::kUnavailable, "");
  realtime_status = OverwriteIfNotInError(
      realtime_status, FormatRealtimeStatus(StatusCode::kInvalidArgument, ""));
  EXPECT_EQ(realtime_status.code, StatusCode::kUnavailable);
}

}  // namespace
}  // namespace intrinsic
