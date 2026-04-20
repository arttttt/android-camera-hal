#ifndef BASE_EVENT_QUEUE_H
#define BASE_EVENT_QUEUE_H

#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>

#include "EventFd.h"

namespace android {

/* Bounded MPSC queue with eventfd-based readiness signalling.
 *
 * push() is safe from any thread; tryPop() should be called only from
 * the single consumer registered with fd(). Overflow returns false —
 * policy (log-fatal, retry, drop, error-complete) is the caller's
 * decision.
 *
 * Usage:
 *   1. poll() on fd(). When readable, the consumer is woken.
 *   2. drain() the eventfd to clear the wake-state.
 *   3. tryPop() in a loop until it returns false.
 *   4. Back to poll().
 *
 * drain() can be omitted — the eventfd's counter will just stay
 * non-zero and each iteration will be immediately-readable — but
 * that wastes a poll() call per item. */
template <typename T>
class EventQueue {
public:
    explicit EventQueue(std::size_t capacity) : cap(capacity) {}

    EventQueue(const EventQueue&) = delete;
    EventQueue& operator=(const EventQueue&) = delete;

    bool push(T item) {
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (items.size() >= cap) return false;
            items.push_back(std::move(item));
        }
        wake.notify();
        return true;
    }

    bool tryPop(T& out) {
        std::lock_guard<std::mutex> lock(mutex);
        if (items.empty()) return false;
        out = std::move(items.front());
        items.pop_front();
        return true;
    }

    void drain() { wake.drain(); }

    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mutex);
        return items.size();
    }

    std::size_t capacity() const { return cap; }

    int fd() const { return wake.fd(); }

private:
    std::size_t cap;
    std::deque<T> items;
    mutable std::mutex mutex;
    EventFd wake;
};

} /* namespace android */

#endif /* BASE_EVENT_QUEUE_H */
