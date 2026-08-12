#include "icon/utils/mutex.h"

#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "icon/utils/attributes.h"

namespace intrinsic {
namespace {

struct GuardedData {
  Mutex mu;
  int counter INTR_GUARDED_BY(mu) = 0;
};

TEST(MutexTest, BasicLockUnlock) {
  Mutex mu;
  mu.Lock();
  mu.Unlock();
}

TEST(MutexTest, LowercaseLockUnlock) {
  Mutex mu;
  mu.lock();
  mu.unlock();
}

TEST(MutexTest, TryLockSucceedsWhenUnlocked) {
  Mutex mu;
  if (mu.TryLock()) {
    mu.Unlock();
  } else {
    FAIL() << "TryLock failed when unlocked";
  }

  if (mu.try_lock()) {
    mu.unlock();
  } else {
    FAIL() << "try_lock failed when unlocked";
  }
}

TEST(MutexTest, TryLockFailsWhenLocked) {
  Mutex mu;
  mu.Lock();

  std::thread t([&mu]() {
    EXPECT_FALSE(mu.TryLock());
    EXPECT_FALSE(mu.try_lock());
  });
  t.join();

  mu.Unlock();
}

TEST(MutexTest, MutexLockWithPointerGuardsVariable) {
  GuardedData data;
  {
    MutexLock lock(&data.mu);
    data.counter += 1;
    EXPECT_EQ(data.counter, 1);
  }
}

TEST(MutexTest, MutexLockWithReferenceGuardsVariable) {
  GuardedData data;
  {
    MutexLock lock(data.mu);
    data.counter += 5;
    EXPECT_EQ(data.counter, 5);
  }
}

TEST(MutexTest, MultiThreadedCounting) {
  GuardedData data;
  constexpr int kNumThreads = 8;
  constexpr int kIncrementsPerThread = 1000;

  std::vector<std::thread> threads;
  threads.reserve(kNumThreads);
  for (int i = 0; i < kNumThreads; ++i) {
    threads.emplace_back([&data]() {
      for (int j = 0; j < kIncrementsPerThread; ++j) {
        MutexLock lock(data.mu);
        data.counter++;
      }
    });
  }

  for (auto& t : threads) {
    t.join();
  }

  MutexLock lock(data.mu);
  EXPECT_EQ(data.counter, kNumThreads * kIncrementsPerThread);
}

}  // namespace
}  // namespace intrinsic
