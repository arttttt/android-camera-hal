#ifndef HAL_PIPELINE_DEVICE_ERROR_REPORTER_H
#define HAL_PIPELINE_DEVICE_ERROR_REPORTER_H

#include <atomic>

#include <hardware/camera3.h>
#include <utils/Log.h>

namespace android {

/* The one place the HAL tells the framework the device is gone.
 *
 * camera3.h: CAMERA3_MSG_ERROR_DEVICE means the device can no longer
 * produce anything (the GPU was lost, the sensor stopped sending
 * frames); the framework closes it and the app reopens or reports
 * the failure. Whichever part of the pipeline finds that out first
 * calls report(); the message goes out once per session, however
 * many parts find it. Camera rearms the reporter when the session
 * closes.
 *
 * Holds a pointer-to-pointer to the callback ops for the same reason
 * CameraCallbackEmitter does: they are installed by initialize()
 * after the reporter is built. */
class DeviceErrorReporter {
public:
    explicit DeviceErrorReporter(const camera3_callback_ops_t *const *opsPtr)
        : opsPtr(opsPtr), reported(false) {}

    void report(const char *why) {
        if (reported.exchange(true)) return;
        ALOGE("device error: %s — reporting CAMERA3_MSG_ERROR_DEVICE", why);
        const camera3_callback_ops_t *ops = opsPtr ? *opsPtr : nullptr;
        if (!ops) return;
        camera3_notify_msg_t msg;
        msg.type = CAMERA3_MSG_ERROR;
        msg.message.error.frame_number = 0;
        msg.message.error.error_stream = nullptr;
        msg.message.error.error_code   = CAMERA3_MSG_ERROR_DEVICE;
        ops->notify(ops, &msg);
    }

    void rearm() { reported.store(false); }

private:
    const camera3_callback_ops_t *const *opsPtr;
    std::atomic<bool>                    reported;
};

} /* namespace android */

#endif /* HAL_PIPELINE_DEVICE_ERROR_REPORTER_H */
