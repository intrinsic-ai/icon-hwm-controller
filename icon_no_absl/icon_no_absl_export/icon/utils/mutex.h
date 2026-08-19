#ifndef ICON_UTILS_MUTEX_H_
#define ICON_UTILS_MUTEX_H_

#include <mutex>

#include "icon/utils/attributes.h"

namespace intrinsic {

// Wrapper around `std::mutex` that implements Clang Thread Safety Analysis
// annotations (such as `INTR_GUARDED_BY`, `INTR_LOCKABLE`,
// `INTR_SCOPED_LOCKABLE`).
//
// Move and copy operations are deleted.
class INTR_LOCKABLE Mutex {
 public:
  Mutex() = default;
  ~Mutex() = default;

  Mutex(const Mutex&) = delete;
  Mutex& operator=(const Mutex&) = delete;
  Mutex(Mutex&&) = delete;
  Mutex& operator=(Mutex&&) = delete;

  // Acquires the mutex exclusively, blocking if necessary.
  void Lock() INTR_EXCLUSIVE_LOCK_FUNCTION() { mu_.lock(); }

  // Releases the exclusively held mutex.
  void Unlock() INTR_UNLOCK_FUNCTION() { mu_.unlock(); }

  // Attempts to acquire the mutex without blocking. Returns true on success.
  INTR_MUST_USE_RESULT bool TryLock() INTR_EXCLUSIVE_TRYLOCK_FUNCTION(true) {
    return mu_.try_lock();
  }

  // Standard library naming compatibility methods.
  void lock() INTR_EXCLUSIVE_LOCK_FUNCTION() { mu_.lock(); }

  void unlock() INTR_UNLOCK_FUNCTION() { mu_.unlock(); }

  INTR_MUST_USE_RESULT bool try_lock() INTR_EXCLUSIVE_TRYLOCK_FUNCTION(true) {
    return mu_.try_lock();
  }

  // Asserts that the calling thread holds this mutex.
  void AssertHeld() const INTR_ASSERT_EXCLUSIVE_LOCK() {}

  // Returns the underlying native handle.
  std::mutex::native_handle_type native_handle() { return mu_.native_handle(); }

 private:
  std::mutex mu_;
};

// RAII scoped lock helper that acquires and releases a `Mutex`.
class INTR_SCOPED_LOCKABLE MutexLock {
 public:
  explicit MutexLock(Mutex* mu) INTR_EXCLUSIVE_LOCK_FUNCTION(mu) : mu_(mu) {
    mu_->Lock();
  }

  explicit MutexLock(Mutex& mu) INTR_EXCLUSIVE_LOCK_FUNCTION(mu) : mu_(&mu) {
    mu_->Lock();
  }

  ~MutexLock() INTR_UNLOCK_FUNCTION() { mu_->Unlock(); }

  MutexLock(const MutexLock&) = delete;
  MutexLock& operator=(const MutexLock&) = delete;
  MutexLock(MutexLock&&) = delete;
  MutexLock& operator=(MutexLock&&) = delete;

 private:
  Mutex* mu_;
};

}  // namespace intrinsic

namespace intrinsic::icon {
using Mutex = ::intrinsic::Mutex;
using MutexLock = ::intrinsic::MutexLock;
}  // namespace intrinsic::icon

#endif  // ICON_UTILS_MUTEX_H_
