#ifndef HAL_PIPELINE_CAMERA_CALLBACK_EMITTER_H
#define HAL_PIPELINE_CAMERA_CALLBACK_EMITTER_H

#include <hardware/camera3.h>

#include "PartialEmitter.h"

namespace android {

/* PartialEmitter backed by the framework's
 * `camera3_callback_ops_t::process_capture_result`.
 *
 * Holds a pointer-to-pointer because `Camera::mCallbackOps` is set
 * by `initialize()` *after* construction — the indirection lets the
 * emitter pick up the live callback as soon as it's installed
 * without the owner having to re-create the emitter. A null inner
 * pointer (cold-start window before initialize) silently drops
 * emits, matching how `ResultDispatchStage` already gates on
 * `ops != nullptr`. */
class CameraCallbackEmitter : public PartialEmitter {
public:
    explicit CameraCallbackEmitter(const camera3_callback_ops_t *const *opsPtr)
        : opsPtr(opsPtr) {}

    void emit(uint32_t                          frameNumber,
              uint8_t                           partialCount,
              const camera_metadata_t          *result,
              size_t                            numBuffers,
              const camera3_stream_buffer      *buffers) override {
        const camera3_callback_ops_t *ops = opsPtr ? *opsPtr : nullptr;
        if (!ops) return;

        camera3_capture_result cr;
        cr.frame_number       = frameNumber;
        cr.result             = result;
        cr.num_output_buffers = numBuffers;
        cr.output_buffers     = buffers;
        cr.input_buffer       = nullptr;
        cr.partial_result     = partialCount;
        ops->process_capture_result(ops, &cr);
    }

private:
    const camera3_callback_ops_t *const *opsPtr;
};

} /* namespace android */

#endif /* HAL_PIPELINE_CAMERA_CALLBACK_EMITTER_H */
