#include "icon/utils/async_request.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <thread>
#include <utility>
#include <vector>

#include "icon/testing/malloc_test.h"
#include "icon/utils/log.h"
#include "icon/utils/mock_log_sink.h"
#include "icon/utils/status.h"
#include "icon/utils/status_and_expected_test_macros.h"
#include "platform/common/buffers/rt_promise.h"

namespace intrinsic::icon {

constexpr std::chrono::milliseconds kWaitTimeout =
    std::chrono::milliseconds(1000);

namespace {
TEST(AsyncRequest, ExampleWorks) {
  std::vector<log::LogEntryWithStorage> log_entries;
  log::Logger logger(log::Logger::Severity::kDebug, log::MockSink(log_entries));
  RealtimeFuture<bool> rt_job_result(&logger);
  INTR_ASSERT_OK_AND_ASSIGN(auto promise, rt_job_result.GetPromise());
  int request_value = 10;
  AsyncRequest<int, bool> request(request_value, std::move(promise));

  auto rt_thread = std::jthread([request = std::move(request),
                                 request_value]() mutable {
    IF_INTRINSIC_MALLOC_TEST_INIT_COUNTER();
    auto& actual_request_value = request.GetRequest();
    IF_INTRINSIC_MALLOC_TEST_EXPECT_NO_ALLOCATIONS();
    EXPECT_EQ(actual_request_value, request_value);
    // Do fancy real time stuff.
    // ...
    IF_INTRINSIC_MALLOC_TEST_INIT_COUNTER();
    INTR_EXPECT_OK(request.SetResponse(true));
    IF_INTRINSIC_MALLOC_TEST_EXPECT_NO_ALLOCATIONS();
  });
  INTR_ASSERT_OK_AND_ASSIGN(bool job_result,
                            rt_job_result.WaitForAndGet(kWaitTimeout));
  rt_thread.join();
  EXPECT_TRUE(job_result);
}

TEST(AsyncRequest, HeapRequestWorks) {
  std::vector<log::LogEntryWithStorage> log_entries;
  log::Logger logger(log::Logger::Severity::kDebug, log::MockSink(log_entries));
  RealtimeFuture<bool> rt_job_result(&logger);
  struct HeapRequest {
    std::vector<int> vec = std::vector<int>(100, -1);
  };
  INTR_ASSERT_OK_AND_ASSIGN(auto promise, rt_job_result.GetPromise());
  IF_INTRINSIC_MALLOC_TEST_INIT_COUNTER();
  HeapRequest request_value;
  EXPECT_EQ(request_value.vec.size(), 100);
  IF_INTRINSIC_MALLOC_TEST_EXPECT_ALLOCATIONS_EQ(1);
  AsyncRequest<HeapRequest, bool> request(request_value, std::move(promise));

  auto rt_thread = std::jthread(
      [request = std::move(request), &request_value]() mutable {
        IF_INTRINSIC_MALLOC_TEST_INIT_COUNTER();
        auto& actual_request_value = request.GetRequest();
        IF_INTRINSIC_MALLOC_TEST_EXPECT_NO_ALLOCATIONS();
        EXPECT_EQ(actual_request_value.vec, request_value.vec);

        IF_INTRINSIC_MALLOC_TEST_INIT_COUNTER();
        auto actual_request_value_moved = request.GetMovedRequest();
        IF_INTRINSIC_MALLOC_TEST_EXPECT_NO_ALLOCATIONS();
        EXPECT_EQ(actual_request_value_moved.vec, request_value.vec);

        // Do fancy real time stuff.
        // ...
        EXPECT_FALSE(request.IsCancelled());
        IF_INTRINSIC_MALLOC_TEST_INIT_COUNTER();
        INTR_EXPECT_OK(request.SetResponse(true));
        IF_INTRINSIC_MALLOC_TEST_EXPECT_NO_ALLOCATIONS();
      });
  INTR_ASSERT_OK_AND_ASSIGN(bool job_result,
                            rt_job_result.WaitForAndGet(kWaitTimeout));
  rt_thread.join();
  EXPECT_TRUE(job_result);
}

TEST(AsyncRequest, CancelWorks) {
  std::vector<log::LogEntryWithStorage> log_entries;
  log::Logger logger(log::Logger::Severity::kDebug, log::MockSink(log_entries));
  RealtimeFuture<bool> rt_job_result(&logger);
  INTR_ASSERT_OK_AND_ASSIGN(auto promise, rt_job_result.GetPromise());
  AsyncRequest<int, bool> request(10, std::move(promise));

  auto rt_thread = std::jthread([request = std::move(request)]() mutable {
    // Do fancy real time stuff.
    // ...
    EXPECT_FALSE(request.IsCancelled());
    INTR_EXPECT_OK(request.Cancel());
    EXPECT_TRUE(request.IsCancelled());
  });
  EXPECT_EQ(rt_job_result.WaitFor(kWaitTimeout).code, StatusCode::kCancelled);
  rt_thread.join();
}

TEST(AsyncRequest, NoPromise) {
  std::vector<log::LogEntryWithStorage> log_entries;
  log::Logger logger(log::Logger::Severity::kDebug, log::MockSink(log_entries));
  RealtimeFuture<bool> rt_job_result(&logger);
  AsyncRequest<int, bool> request(1);

  auto rt_thread =
      std::jthread([request = std::move(request)]() mutable {
        // Requester is not interested in
        // result, but we don't necessarily know
        // that in the rt thread nor need to
        // care.
        INTR_EXPECT_OK(request.SetResponse(true));
        // Same for cancelling: There is no
        // promise, so cancelling always returns
        // true. There cannot be anyone waiting
        // for it.
        INTR_EXPECT_OK(request.Cancel());
      });
  rt_thread.join();
}

}  // namespace
}  // namespace intrinsic::icon
