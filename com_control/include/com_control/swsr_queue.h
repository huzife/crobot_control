#ifndef COM_CONTROL_SWSR_QUEUE_H
#define COM_CONTROL_SWSR_QUEUE_H

#include <atomic>

/// @class SWSR_Queue
/// @brief A thread-safe, single-write and single-read queue,
///        implemented with ring buffer.
/// @tparam T element type
template <typename T>
class SWSR_Queue {
public:
    SWSR_Queue() = delete;
    SWSR_Queue(const SWSR_Queue&) = delete;
    SWSR_Queue& operator=(const SWSR_Queue&) = delete;
    SWSR_Queue(SWSR_Queue&&) = delete;
    SWSR_Queue& operator=(SWSR_Queue&&) = delete;

    SWSR_Queue(size_t size);
    ~SWSR_Queue();

    /// @brief Push an element if not full
    /// @param[in] val The element to be pushed
    /// @return bool
    /// @retval true    Success
    ///         false   Failed, queue is full
    bool push(T val);

    /// @brief Pop an element if not empty
    /// @param[out] val The element to be popped
    /// @return bool
    /// @retval true    Success
    ///         false   Failed, queue is empty
    bool pop(T& val);

private:
    size_t next_index(size_t index) const;

private:
    std::atomic_size_t head_;
    std::atomic_size_t tail_;
    size_t size_;
    T* buf_;
};

#include "swsr_queue.inl"

#endif // COM_CONTROL_SWSR_QUEUE_H
