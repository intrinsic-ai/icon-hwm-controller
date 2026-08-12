#ifndef PLATFORM_COMMON_BUFFERS_RT_QUEUE_BUFFER_H_
#define PLATFORM_COMMON_BUFFERS_RT_QUEUE_BUFFER_H_

#include <stdlib.h>

#include <atomic>
#include <cstddef>
#include <functional>
#include <iostream>

#include "icon/utils/attributes.h"

namespace intrinsic {
namespace internal {

// A buffer for performing spsc-queue style automatic operations.
template <typename T>
class RtQueueBuffer {
 public:
  explicit RtQueueBuffer(size_t capacity);

  RtQueueBuffer(size_t capacity, std::function<void(T*)> init_function);

  // Gets a pointer to the front element, or nullptr if empty. After a call to
  // Front(), must call DropFront() or KeepFront() prior to subsequent calls to
  // Front().
  INTR_MUST_USE_RESULT T* Front();

  // Removes the front element; no-op if the queue is empty.
  void DropFront();

  // Keeps the front element.
  void KeepFront();

  // Gets a pointer to the next available element, or nullptr if the queue is
  // full. The element should be set and then FinishInsert must be called.
  INTR_MUST_USE_RESULT T* PrepareInsert();

  // Make the element referenced by the return value of PrepareInsert
  // available to the reader.
  void FinishInsert();

  // Returns the number of elements in the buffer. Thread-safe.
  size_t Size() const { return size_.load(std::memory_order_acquire); }

  // Returns true when the buffer is empty. Thread-safe.
  bool Empty() const { return size_.load(std::memory_order_acquire) == 0; }

  // Returns true when the buffer is full. Thread-safe.
  bool Full() const {
    return size_.load(std::memory_order_acquire) == capacity_;
  }

  // Returns the capacity of the buffer.
  size_t Capacity() const { return capacity_; }

  void InitElements(std::function<void(T*)> init_function);

 private:
  // Increases the number of messages stored in the buffer by 1.
  void IncreaseSize() { size_.fetch_add(1, std::memory_order_seq_cst); }

  // Decreases the number of messages stored in the buffer by 1.
  void DecreaseSize() { size_.fetch_sub(1, std::memory_order_seq_cst); }

  bool insert_in_progress_ = false;
  size_t head_ = 0;

  bool front_accessed_ = false;
  size_t tail_ = 0;

  // Memory used as a ring buffer.
  std::atomic_size_t size_ = 0;  // number of messages stored in the buffer
  const size_t capacity_;        // the length of the buffer
  std::unique_ptr<T[]> buffer_;
};

// Implementation of RealtimeQueue functions.
template <typename T>
RtQueueBuffer<T>::RtQueueBuffer(size_t capacity)
    : capacity_(capacity), buffer_(std::make_unique<T[]>(capacity_)) {}

template <typename T>
RtQueueBuffer<T>::RtQueueBuffer(size_t capacity,
                                std::function<void(T*)> init_function)
    : capacity_(capacity), buffer_(std::make_unique<T[]>(capacity_)) {
  InitElements(init_function);
}

template <typename T>
void RtQueueBuffer<T>::InitElements(std::function<void(T*)> init_function) {
  for (size_t i = 0; i < Capacity(); ++i) {
    init_function(&buffer_[i]);
  }
}

// Implementation of RealtimeQueue::Reader functions.
template <typename T>
T* RtQueueBuffer<T>::Front() {
  if (front_accessed_) {
    std::cerr << "KeepFront or DropFront must be called before another "
                 "call to Front is allowed."
              << std::endl;
    std::abort();
  }
  if (Empty()) {
    return nullptr;
  }
  front_accessed_ = true;
  return &buffer_[tail_];
}

template <typename T>
void RtQueueBuffer<T>::KeepFront() {
  if (!front_accessed_) {
    std::cerr << "Front must be called before KeepFront." << std::endl;
    std::abort();
  }
  front_accessed_ = false;
}

template <typename T>
void RtQueueBuffer<T>::DropFront() {
  if (!front_accessed_) {
    std::cerr << "Front must be called before DropFront." << std::endl;
    std::abort();
  }
  front_accessed_ = false;
  DecreaseSize();
  tail_ = (tail_ + 1) % Capacity();
}

template <typename T>
T* RtQueueBuffer<T>::PrepareInsert() {
  if (insert_in_progress_) {
    std::cerr << "FinishInsert must be called before another call to "
                 "PrepareInsert is allowed."
              << std::endl;
    std::abort();
  }
  if (Full()) {
    return nullptr;
  }
  insert_in_progress_ = true;
  return &buffer_[head_];
}

template <typename T>
void RtQueueBuffer<T>::FinishInsert() {
  if (!insert_in_progress_) {
    std::cerr << "PrepareInsert must be called before FinishInsert."
              << std::endl;
    std::abort();
  }
  insert_in_progress_ = false;
  head_ = (head_ + 1) % Capacity();
  IncreaseSize();
}

}  // namespace internal
}  // namespace intrinsic

#endif  // PLATFORM_COMMON_BUFFERS_RT_QUEUE_BUFFER_H_
