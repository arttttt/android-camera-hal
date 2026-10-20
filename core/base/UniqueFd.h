#ifndef BASE_UNIQUE_FD_H
#define BASE_UNIQUE_FD_H

#include <unistd.h>

namespace android {

/* Move-only RAII wrapper around a POSIX file descriptor. Closes on
 * destruction unless release() is called first. Empty state is fd < 0. */
class UniqueFd {
public:
    UniqueFd() : descriptor(-1) {}
    explicit UniqueFd(int fd) : descriptor(fd) {}

    UniqueFd(UniqueFd&& other) : descriptor(other.descriptor) {
        other.descriptor = -1;
    }

    UniqueFd& operator=(UniqueFd&& other) {
        if (this != &other) {
            reset();
            descriptor = other.descriptor;
            other.descriptor = -1;
        }
        return *this;
    }

    UniqueFd(const UniqueFd&) = delete;
    UniqueFd& operator=(const UniqueFd&) = delete;

    ~UniqueFd() { reset(); }

    int  get()   const { return descriptor; }
    bool valid() const { return descriptor >= 0; }

    int release() {
        int taken = descriptor;
        descriptor = -1;
        return taken;
    }

    void reset(int fd = -1) {
        if (descriptor >= 0 && descriptor != fd) {
            ::close(descriptor);
        }
        descriptor = fd;
    }

private:
    int descriptor;
};

} /* namespace android */

#endif /* BASE_UNIQUE_FD_H */
