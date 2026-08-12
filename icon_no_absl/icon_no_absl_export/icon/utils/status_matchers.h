#ifndef ICON_UTILS_STATUS_MATCHERS_H_
#define ICON_UTILS_STATUS_MATCHERS_H_

#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "icon/utils/status.h"
#include "icon/utils/status_and_expected_macros.h"
#include "tl/expected.hpp"

namespace intrinsic::testing {

namespace internal_status {

template <typename T>
struct StatusExtractor;

template <>
struct StatusExtractor<::intrinsic::Status> {
  static bool IsOk(const ::intrinsic::Status& s) { return s.ok(); }
  static ::intrinsic::StatusCode GetCode(const ::intrinsic::Status& s) { return s.code; }
  static std::string_view GetMessage(const ::intrinsic::Status& s) { return s.message; }
};

template <>
struct StatusExtractor<::intrinsic::RealtimeStatus> {
  static bool IsOk(const ::intrinsic::RealtimeStatus& s) { return s.ok(); }
  static ::intrinsic::StatusCode GetCode(const ::intrinsic::RealtimeStatus& s) { return s.code; }
  static std::string_view GetMessage(const ::intrinsic::RealtimeStatus& s) { return s.GetMessage(); }
};

template <typename T, typename E>
struct StatusExtractor<tl::expected<T, E>> {
  static bool IsOk(const tl::expected<T, E>& e) { return e.has_value(); }
  static ::intrinsic::StatusCode GetCode(const tl::expected<T, E>& e) {
    if (e.has_value()) return ::intrinsic::StatusCode::kOk;
    return StatusExtractor<E>::GetCode(e.error());
  }
  static std::string_view GetMessage(const tl::expected<T, E>& e) {
    if (e.has_value()) return "";
    return StatusExtractor<E>::GetMessage(e.error());
  }
};

}  // namespace internal_status

// Matches a `tl::expected<T, E>` that has a value, and verifies that the held
// value matches `inner_matcher`.
//
// Example:
// ```cpp
// tl::expected<int, Status> result = CalculateValue();
// EXPECT_THAT(result, IsOkAndHolds(Eq(42)));
// EXPECT_THAT(result, IsOkAndHolds(Ge(40)));
// ```
MATCHER_P(IsOkAndHolds, inner_matcher, "") {
  if (!arg.has_value()) {
    *result_listener << "is unexpected status: " << ::intrinsic::ToString(arg.error());
    return false;
  }
  return ::testing::ExplainMatchResult(inner_matcher, arg.value(), result_listener);
}

// Matches any `intrinsic::Status`, `intrinsic::RealtimeStatus`, or
// `tl::expected<T, E>` that represents a successful / OK state.
//
// Example:
// ```cpp
// Status status = DoWork();
// EXPECT_THAT(status, IsOk());
//
// tl::expected<MyType, Status> result = CreateObject();
// EXPECT_THAT(result, IsOk());
// ```
MATCHER(IsOk, "") {
  using CleanT = std::remove_cvref_t<decltype(arg)>;
  if constexpr (requires { internal_status::StatusExtractor<CleanT>::IsOk(arg); }) {
    if (!internal_status::StatusExtractor<CleanT>::IsOk(arg)) {
      *result_listener << "is error";
      return false;
    }
    return true;
  } else if constexpr (requires { arg.ok(); }) {
    return arg.ok();
  } else if constexpr (requires { arg.has_value(); }) {
    return arg.has_value();
  } else {
    static_assert(sizeof(arg) == 0, "Unsupported type for IsOk matcher");
  }
}

// Matches an `intrinsic::Status`, `intrinsic::RealtimeStatus`, or
// `tl::expected<T, E>` whose status code matches `code_matcher` and whose
// message matches `message_matcher`.
//
// Example:
// ```cpp
// Status status = ProcessInput("");
// EXPECT_THAT(status, StatusIs(StatusCode::kInvalidArgument, HasSubstr("empty")));
//
// tl::expected<int, Status> val = GetValue(-1);
// EXPECT_THAT(val, StatusIs(StatusCode::kInvalidArgument, Eq("Negative index")));
// ```
MATCHER_P2(StatusIs, code_matcher, message_matcher, "") {
  using CleanT = std::remove_cvref_t<decltype(arg)>;
  ::intrinsic::StatusCode code;
  std::string_view message;

  if constexpr (requires { internal_status::StatusExtractor<CleanT>::GetCode(arg); }) {
    code = internal_status::StatusExtractor<CleanT>::GetCode(arg);
    message = internal_status::StatusExtractor<CleanT>::GetMessage(arg);
  } else {
    static_assert(sizeof(arg) == 0, "Unsupported type for StatusIs matcher");
  }

  return ::testing::ExplainMatchResult(code_matcher, code, result_listener) &&
         ::testing::ExplainMatchResult(message_matcher, message, result_listener);
}

// Matches an `intrinsic::Status`, `intrinsic::RealtimeStatus`, or
// `tl::expected<T, E>` whose status code matches `code_matcher`.
//
// Example:
// ```cpp
// Status status = ConnectToServer();
// EXPECT_THAT(status, StatusIs(StatusCode::kUnavailable));
// ```
MATCHER_P(StatusIs, code_matcher, "") {
  return ::testing::ExplainMatchResult(StatusIs(code_matcher, ::testing::_), arg, result_listener);
}

}  // namespace intrinsic::testing

namespace intrinsic::icon {
using ::intrinsic::testing::IsOk;
using ::intrinsic::testing::IsOkAndHolds;
using ::intrinsic::testing::StatusIs;
}  // namespace intrinsic::icon

#endif  // ICON_UTILS_STATUS_MATCHERS_H_
