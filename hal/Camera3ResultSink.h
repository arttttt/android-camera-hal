/*
 * Copyright (C) 2026 Artem Bambalov
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef HAL_CAMERA3_RESULT_SINK_H
#define HAL_CAMERA3_RESULT_SINK_H

#include <stdint.h>

#include <map>
#include <mutex>
#include <vector>

#include <hardware/camera3.h>

#include "CameraMetadataCompat.h"
#include "mochacamera/ports/ResultSink.h"

namespace android {

/* The core's results as Camera3 wants them: notify() for the shutter
 * and the errors, process_capture_result() for the partial and final
 * results.
 *
 * A frame's metadata starts as its request's own settings -- the echo
 * Camera3 asks for -- so the sink keeps each accepted request's
 * settings until its final result has gone out. Streams are known by
 * the core's StreamId; the table is set at every stream configuration.
 *
 * Callbacks may come from any of the core's threads; the per-frame
 * table is guarded, the framework callbacks are not. */
class Camera3ResultSink : public mocha::camera::ResultSink {
public:
    /* `opsPtr`: the framework's callbacks are installed by initialize()
     * after this sink is built, hence the pointer to the pointer. */
    Camera3ResultSink(const camera3_callback_ops_t *const *opsPtr,
                      float focalLengthMm);

    /* The configured streams, indexed by StreamId. */
    void setStreams(std::vector<camera3_stream_t *> streams);

    /* A request accepted: its settings, for the echo. */
    void expect(uint32_t frameNumber, const CameraMetadata &settings);

    /* A request that will not reach the core after all. */
    void forget(uint32_t frameNumber);

    void shutter(uint32_t frameNumber, int64_t timestampNs) override;
    void partial(uint32_t frameNumber, mocha::camera::PartialResult which,
                 const mocha::camera::Core3AResult &result) override;
    void frame(uint32_t frameNumber, const mocha::camera::CoreFrameResult &result,
               std::span<const mocha::camera::BufferReturn> buffers) override;
    void deviceError(const char *why) override;

private:
    const camera3_callback_ops_t *ops() const { return opsPtr ? *opsPtr : nullptr; }
    camera3_stream_t *stream(mocha::camera::StreamId id) const;
    void notifyError(uint32_t frameNumber, camera3_stream_t *stream, int code);
    void emit(uint32_t frameNumber, uint32_t partialCount,
              const camera_metadata_t *metadata,
              const std::vector<camera3_stream_buffer_t> &buffers);

    const camera3_callback_ops_t *const *opsPtr;
    const float focalLengthMm;

    mutable std::mutex mutex;
    std::vector<camera3_stream_t *>    streams;
    std::map<uint32_t, CameraMetadata> settings;
};

} /* namespace android */

#endif /* HAL_CAMERA3_RESULT_SINK_H */
