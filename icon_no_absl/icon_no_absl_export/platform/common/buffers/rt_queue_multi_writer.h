#ifndef INCODE_PLATFORM_COMMON_BUFFERS_RT_QUEUE_MULTI_WRITER_H_
#define INCODE_PLATFORM_COMMON_BUFFERS_RT_QUEUE_MULTI_WRITER_H_

#include <mutex>

#include "icon/utils/attributes.h"
#include "icon/utils/realtime_guard.h"
#include "icon/utils/status.h"
#include "platform/common/buffers/rt_queue.h"

namespace intrinsic {

// Wrapper around RealtimeQueue<T>::Writer that makes it thread safe for
// multiple concurrent writers. In doing this, we drop the realtime safety of
// the write operation. Any readers of the RealtimeQueue are of course still
// realtime safe.
template <class T>
class RealtimeQueueMultiWriter {
 public:
  // Not copyable or moveable (Copying would undermine the thread safety, and
  // moving would make existing references to a MultiWriter invalid).
  RealtimeQueueMultiWriter(const RealtimeQueueMultiWriter&) = delete;
  RealtimeQueueMultiWriter(RealtimeQueueMultiWriter&&) = delete;

  // Wraps `writer` to provide thread-safe insertion into a RealtimeQueue.
  // `writer` must outlive this RealtimeQueueMultiWriter.
  explicit RealtimeQueueMultiWriter(typename RealtimeQueue<T>::Writer& writer)
      : writer_(writer) {}

  // Moves `value` into the underlying RealtimeQueue if possible.
  // Not realtime safe, but thread safe.
  // Returns ResourceExhausted if the underlying RealtimeQueue is full.
  Status Insert(T&& value) INTR_LOCKS_EXCLUDED(mutex_) {
    INTRINSIC_ASSERT_NON_REALTIME();
    std::lock_guard<std::mutex> l(mutex_);
    // RealtimeQueue::Writer::Insert() makes a copy, negating any performance
    // benefits we get from accepting rvalues.
    T* queue_item = writer_.PrepareInsert();
    if (queue_item == nullptr) {
      return FormatStatus(StatusCode::kResourceExhausted,
                          "RealtimeQueue capacity exhausted");
    }
    *queue_item = std::move(value);
    writer_.FinishInsert();
    return OkStatus();
  }

 private:
  std::mutex mutex_;
  typename RealtimeQueue<T>::Writer& writer_ INTR_GUARDED_BY(mutex_);
};

}  // namespace intrinsic

#endif  // INCODE_PLATFORM_COMMON_BUFFERS_RT_QUEUE_MULTI_WRITER_H_
