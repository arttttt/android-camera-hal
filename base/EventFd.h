#ifndef BASE_EVENT_FD_H
#define BASE_EVENT_FD_H

#include "UniqueFd.h"

namespace android {

/* Linux eventfd wrapper used as cross-thread wake primitive. Non-blocking,
 * CLOEXEC. The internal counter accumulates notify() calls; drain() resets
 * it to 0. A poll(fd, POLLIN) returns readable whenever the counter is
 * non-zero. */
class EventFd {
public:
    EventFd();
    ~EventFd() = default;

    EventFd(EventFd&&) = default;
    EventFd& operator=(EventFd&&) = default;

    EventFd(const EventFd&) = delete;
    EventFd& operator=(const EventFd&) = delete;

    bool isValid() const { return handle.valid(); }
    int  fd()      const { return handle.get(); }

    /* Add 1 to the counter. Safe from any thread. */
    void notify();

    /* Reset the counter to 0. Call before processing pending work to
     * avoid stale wake-ups. Safe only from the reader thread. */
    void drain();

private:
    UniqueFd handle;
};

} /* namespace android */

#endif /* BASE_EVENT_FD_H */
