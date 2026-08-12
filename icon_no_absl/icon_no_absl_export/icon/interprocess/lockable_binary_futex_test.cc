#include "icon/interprocess/lockable_binary_futex.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <thread>

#include "icon/testing/realtime_annotations.h"
#include "icon/utils/status_and_expected_test_macros.h"

using ::testing::Eq;

namespace intrinsic {

namespace {
TEST(SpinlockTest, LockWorks) {
  // Test that Lock() can get a lock when it is unlocked and not when it is
  // locked.
  LockableBinaryFutex mutex;

  INTR_EXPECT_OK(mutex.Lock());
  EXPECT_TRUE(mutex.IsHeld());
  EXPECT_FALSE(mutex.TryLock());
  INTR_EXPECT_OK(mutex.Unlock());
  INTR_EXPECT_OK(mutex.Lock());
  INTR_EXPECT_OK(mutex.Unlock());
}

TEST(SpinlockTest, TryLockWorks) {
  // Test that TryLock() can get a lock when it is unlocked and not when it is
  // locked.
  LockableBinaryFutex mutex;

  auto got_lock = mutex.TryLock();
  EXPECT_TRUE(got_lock);
  if (got_lock) {  // compile time checks force us to check this and unlock
                   // only in if-branch.
    EXPECT_FALSE(mutex.TryLock());
    INTR_EXPECT_OK(mutex.Unlock());
  }
  got_lock = mutex.TryLock();
  EXPECT_TRUE(got_lock);
  if (got_lock) {
    INTR_EXPECT_OK(mutex.Unlock());
  }
}

TEST(SpinlockTest, AvoidsRaceCondition) {
  LockableBinaryFutex mutex;
  int counter = 0;

  auto iterations = 100;
  auto loop_increase = [&]() {
    for (int i = 0; i < iterations; ++i) {
      BinaryFutexLock lock(&mutex);
      size_t c = counter;
      std::this_thread::sleep_for(std::chrono::milliseconds(
          10));  // Sleep to trigger race condition if the lock would not work
      counter = c + 1;
    }
  };
  std::jthread thread1(loop_increase);
  std::jthread thread2(loop_increase);
  thread1.join();
  thread2.join();
  EXPECT_THAT(counter, Eq(iterations * 2));
}

TEST(SpinlockTest, TryLockAvoidsRaceCondition) {
  LockableBinaryFutex mutex;
  int counter = 0;
  std::atomic_int atomic_counter = 0;
  auto iterations = 100;
  auto loop_increase = [&]() {
    for (int i = 0; i < iterations; ++i) {
      if (mutex.TryLock()) {
        atomic_counter++;
        size_t c = counter;

        std::this_thread::sleep_for(std::chrono::milliseconds(
            10));  // Sleep to trigger race condition if the lock would not work
        counter = c + 1;
        INTR_ASSERT_OK(mutex.Unlock());
      }
    }
  };
  std::jthread thread1(loop_increase);
  std::jthread thread2(loop_increase);
  thread1.join();
  thread2.join();
  EXPECT_GT(atomic_counter.load(), 0);
  EXPECT_EQ(counter, atomic_counter.load());
}

TEST(SpinlockTest, IntrGuardedByCompiles) {
  struct Counter {
    LockableBinaryFutex mutex;
    int value INTR_GUARDED_BY(mutex) = 0;
  };
  Counter counter;
  BinaryFutexLock lock(&counter.mutex);
  size_t c = counter.value;
  counter.value = c + 1;
}

TEST(SpinlockTest, IntrGuardedByCompilesWithTryLock) {
  struct Counter {
    LockableBinaryFutex mutex;
    int value INTR_GUARDED_BY(mutex) = 0;
  };
  Counter counter;
  auto locked = counter.mutex.TryLock();
  if (locked) {
    size_t c = counter.value;
    counter.value = c + 1;
    if (true) {
      // Check that 2 return paths are also covered.
      INTR_EXPECT_OK(counter.mutex.Unlock());
      return;
    }
    INTR_EXPECT_OK(counter.mutex.Unlock());
  }
}

}  // namespace

}  // namespace intrinsic
