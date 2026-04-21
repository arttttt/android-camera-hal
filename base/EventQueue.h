#ifndef BASE_EVENT_QUEUE_H
#define BASE_EVENT_QUEUE_H

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>

#include "EventFd.h"

namespace android {

/* Bounded MPSC queue with eventfd-based readiness signalling.
 *
 * push() is safe from any thread; tryPop() should be called only from
 * the single consumer registered with fd(). Overflow on the non-blocking
 * push() returns false — policy (log-fatal, retry, drop, error-complete)
 * is the caller's decision. pushBlocking() is the opposite: it sleeps
 * on a condvar until a consumer pop opens a slot or requestStop() tears
 * down waiters.
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
    explicit EventQueue(std::size_t capacity) : cap(capacity), stopping(false) {}

    EventQueue(const EventQueue&) = delete;
    EventQueue& operator=(const EventQueue&) = delete;

    bool push(T item) {
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (items.size() >= cap) return false;
            items.push_back(std::move(item));
        }
        notFull.notify_one();
        wake.notify();
        return true;
    }

    /* Block until a slot is available or requestStop() is called.
     * Returns false if woken by stop — caller must handle its item
     * (e.g. error-complete it) instead of leaking. */
    bool pushBlocking(T item) {
        {
            std::unique_lock<std::mutex> lock(mutex);
            notFull.wait(lock, [this] {
                return items.size() < cap ||
                       stopping.load(std::memory_order_acquire);
            });
            if (stopping.load(std::memory_order_acquire)) return false;
            items.push_back(std::move(item));
        }
        notFull.notify_one();
        wake.notify();
        return true;
    }

    bool tryPop(T& out) {
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (items.empty()) return false;
            out = std::move(items.front());
            items.pop_front();
        }
        notFull.notify_one();
        return true;
    }

    void drain() { wake.drain(); }

    /* Wake every pushBlocking() waiter with a false return. Set before
     * joining downstream workers so upstream producers can bail out of
     * their wait instead of holding the join indefinitely. */
    void requestStop() {
        stopping.store(true, std::memory_order_release);
        notFull.notify_all();
    }

    /* Clear the stop flag so pushBlocking resumes blocking behaviour.
     * Call between worker stop and the next worker start. */
    void clearStop() {
        stopping.store(false, std::memory_order_release);
    }

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
    std::condition_variable notFull;
    std::atomic<bool> stopping;
    EventFd wake;
};

} /* namespace android */

#endif /* BASE_EVENT_QUEUE_H */
