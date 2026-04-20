#include "ThreadBase.h"

#include <pthread.h>
#include <string.h>
#include <system_error>

#include <utils/Log.h>

namespace android {

ThreadBase::ThreadBase()
    : running(false),
      stopFlag(false) {}

ThreadBase::~ThreadBase() {
    /* Defensive: if the subclass forgot to stop() explicitly, clean up
     * here. Joining a still-running worker during ~Derived() is
     * dangerous because the worker may touch derived state that's
     * already gone, so derived destructors should call stop() first. */
    stop();
}

bool ThreadBase::start(const char* name) {
    if (isRunning()) {
        ALOGW("ThreadBase(%s): already running", name ? name : "?");
        return false;
    }
    if (worker.joinable()) {
        /* Previous run finished but was never joined. */
        worker.join();
    }
    if (!stopEvent.isValid()) {
        ALOGE("ThreadBase(%s): stop eventfd invalid", name ? name : "?");
        return false;
    }

    stopFlag.store(false, std::memory_order_release);
    running.store(true, std::memory_order_release);

    try {
        worker = std::thread(&ThreadBase::run, this,
                             std::string(name ? name : "worker"));
    } catch (const std::system_error& e) {
        ALOGE("ThreadBase(%s): std::thread failed: %s",
              name ? name : "?", e.what());
        running.store(false, std::memory_order_release);
        return false;
    }
    return true;
}

void ThreadBase::stop() {
    if (std::this_thread::get_id() == worker.get_id()) {
        /* Self-stop: request exit but don't join self. */
        stopFlag.store(true, std::memory_order_release);
        stopEvent.notify();
        return;
    }
    if (!worker.joinable()) return;
    stopFlag.store(true, std::memory_order_release);
    stopEvent.notify();
    worker.join();
}

void ThreadBase::run(std::string name) {
    char buf[16] = {0};
    ::strncpy(buf, name.c_str(), sizeof(buf) - 1);
    ::pthread_setname_np(::pthread_self(), buf);

    threadLoop();

    running.store(false, std::memory_order_release);
}

} /* namespace android */
