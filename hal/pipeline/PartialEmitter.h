#ifndef HAL_PIPELINE_PARTIAL_EMITTER_H
#define HAL_PIPELINE_PARTIAL_EMITTER_H

#include <stddef.h>
#include <stdint.h>

#include <hardware/camera3.h>
#include <system/camera_metadata.h>

namespace android {

/* Sink for camera3 partial results. Hides the underlying
 * `camera3_callback_ops_t::process_capture_result` call from
 * producers (the IPA, the result-dispatch stage) so the producers
 * don't reach into the framework callback layer directly.
 *
 * Each call to `emit` issues exactly one `process_capture_result`
 * with the given metadata and counter. Callers are responsible for:
 *
 *   - Setting `partialCount` consistent with what was advertised in
 *     `ANDROID_REQUEST_PARTIAL_RESULT_COUNT`. Counters within one
 *     frame must increase monotonically; the final partial uses
 *     `kPartialResultCount` (the advertised maximum).
 *   - Passing `numBuffers` > 0 only on the final partial. Buffer-
 *     bearing partials in the middle of a sequence are illegal per
 *     camera3 contract.
 *   - Keeping `result` and `buffers` valid until `emit` returns;
 *     framework copies-out synchronously.
 *
 * Implementations may be no-ops when the camera3 callback has not
 * been initialised yet (cold start) — producers don't need to gate
 * on that themselves. */
class PartialEmitter {
public:
    virtual ~PartialEmitter() = default;

    virtual void emit(uint32_t                          frameNumber,
                      uint8_t                           partialCount,
                      const camera_metadata_t          *result,
                      size_t                            numBuffers,
                      const camera3_stream_buffer      *buffers) = 0;
};

} /* namespace android */

#endif /* HAL_PIPELINE_PARTIAL_EMITTER_H */
