#ifndef HAL_PIPELINE_DEVICE_ERROR_REPORTER_H
#define HAL_PIPELINE_DEVICE_ERROR_REPORTER_H

#include <atomic>

#include <utils/Log.h>

#include "mochacamera/ports/ResultSink.h"

namespace android {

/* The one place the HAL tells the framework the device is gone.
 *
 * A device error means the device can no longer produce anything (the
 * GPU was lost, the sensor stopped sending frames); with Camera3 the
 * framework closes it and the app reopens or reports the failure.
 * Whichever part of the pipeline finds that out first calls report();
 * it goes to the result sink once per session, however many parts find
 * it. Session::close rearms it. */
class DeviceErrorReporter {
public:
    explicit DeviceErrorReporter(mocha::camera::ResultSink *sink)
        : sink(sink), reported(false) {}

    void report(const char *why) {
        if (reported.exchange(true)) return;
        ALOGE("device error: %s", why);
        if (sink) sink->deviceError(why);
    }

    void rearm() { reported.store(false); }

private:
    mocha::camera::ResultSink *sink;
    std::atomic<bool>          reported;
};

} /* namespace android */

#endif /* HAL_PIPELINE_DEVICE_ERROR_REPORTER_H */
