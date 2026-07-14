#include "icon/utils/current_cycle.h"

#include <atomic>
#include <cstdint>

namespace intrinsic::icon {

uint64_t CycleCounter::GetCurrentCycle() noexcept {
  return current_cycle_.load(std::memory_order_acquire);
}

void CycleCounter::SetCurrentCycle(uint64_t cycle) noexcept {
  current_cycle_.store(cycle, std::memory_order_release);
}

void CycleCounter::IncrementCurrentCycle() noexcept {
  // Since current_cycle_ is unsigned, it overflows to zero automatically,
  // like one would expect (cf.
  // https://en.cppreference.com/cpp/language/operator_arithmetic#Overflows)
  current_cycle_.fetch_add(1, std::memory_order_acq_rel);
}

}  // namespace intrinsic::icon
