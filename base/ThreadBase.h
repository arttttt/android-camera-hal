#ifndef BASE_THREAD_BASE_H
#define BASE_THREAD_BASE_H

#include <atomic>
#include <string>
#include <thread>

#include "EventFd.h"

namespace android {

/* Abstract worker thread with event-driven shutdown.
 *
 * Derived classes implement threadLoop(). The loop is expected to
 * include stopFd() in its poll() set and return promptly when that fd
 * becomes readable. stopRequested() is also available for loops that
 * don't poll (rare — poll-integrated stop is preferred).
 *
 * start(name) spawns the worker via std::thread; the thread name is
 * set via pthread_setname_np (trimmed to 15 chars). start() returns
 * false if the worker is already running or if the stop eventfd could
 * not be created.
 *
 * stop() signals the stop eventfd, then joins. Idempotent. Calling
 * stop() from inside threadLoop() (self-stop) sets the flag and
 * returns without joining to avoid self-deadlock. */
class ThreadBase {
public:
    ThreadBase();
    virtual ~ThreadBase();

    ThreadBase(const ThreadBase&) = delete;
    ThreadBase& operator=(const ThreadBase&) = delete;

    bool start(const char* name);
    void stop();

    bool isRunning() const {
        return running.load(std::memory_order_acquire);
    }

protected:
    virtual void threadLoop() = 0;

    bool stopRequested() const {
        return stopFlag.load(std::memory_order_acquire);
    }

    int stopFd() const { return stopEvent.fd(); }

private:
    void run(std::string name);

    std::thread worker;
    EventFd stopEvent;
    std::atomic<bool> running;
    std::atomic<bool> stopFlag;
};

} /* namespace android */

#endif /* BASE_THREAD_BASE_H */
