#include "EventFd.h"

#include <errno.h>
#include <stdint.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <utils/Log.h>

namespace android {

EventFd::EventFd() {
    int fd = ::eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (fd < 0) {
        ALOGE("EventFd: eventfd() failed, errno=%d", errno);
        return;
    }
    handle.reset(fd);
}

void EventFd::notify() {
    if (!handle.valid()) return;
    uint64_t one = 1;
    ssize_t written = ::write(handle.get(), &one, sizeof(one));
    /* EAGAIN on the 64-bit counter overflow is effectively impossible
     * (would take years of notify-without-drain); other errors would
     * indicate the fd was externally closed. Silent is fine — a later
     * notify still wins. */
    (void)written;
}

void EventFd::drain() {
    if (!handle.valid()) return;
    uint64_t counter;
    ssize_t got = ::read(handle.get(), &counter, sizeof(counter));
    /* EAGAIN is normal (nothing to drain). */
    (void)got;
}

} /* namespace android */
