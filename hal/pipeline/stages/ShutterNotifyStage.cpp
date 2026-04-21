#include "ShutterNotifyStage.h"

#include <utils/Timers.h>

#include "PipelineContext.h"

namespace android {

ShutterNotifyStage::ShutterNotifyStage(const camera3_callback_ops_t *const *ops)
    : callbackOps(ops) {}

void ShutterNotifyStage::process(PipelineContext &ctx) {
    ctx.tShutter = systemTime();

    /* Synthetic contexts — used for prewarm — flow through the full
     * pipeline to exercise the real hot path but must not land on the
     * framework's callback. */
    if (ctx.discardOnDispatch) return;

    const camera3_callback_ops_t *ops = *callbackOps;
    if (!ops) return;

    camera3_notify_msg_t msg;
    msg.type = CAMERA3_MSG_SHUTTER;
    msg.message.shutter.frame_number = ctx.request.frameNumber;
    msg.message.shutter.timestamp    = (uint64_t)ctx.tShutter;
    ops->notify(ops, &msg);
}

} /* namespace android */
