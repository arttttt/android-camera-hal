#ifndef HAL_PIPELINE_CAPTURE_REQUEST_H
#define HAL_PIPELINE_CAPTURE_REQUEST_H

#include <stdint.h>
#include <memory>
#include <vector>

#include <camera/CameraMetadata.h>
#include <hardware/camera3.h>

#include "UniqueFd.h"

namespace android {

/* HAL-owned deep copy of camera3_capture_request_t.
 *
 * Produced on the binder thread in Camera::processCaptureRequest and
 * consumed on the request thread. Holds framework-provided buffer
 * handles by raw pointer — the framework guarantees those stay alive
 * until we complete the matching capture result — and takes ownership
 * of any acquire-fence fds via UniqueFd.
 *
 * The inputBuffer slot is reserved for ZSL / reprocess (wired in a
 * later tier); for now it stays null. */
struct CaptureRequest {
    struct Buffer {
        camera3_stream_t *stream;
        buffer_handle_t  *buffer;
        UniqueFd          acquireFence;

        Buffer() : stream(nullptr), buffer(nullptr) {}

        Buffer(Buffer&&) = default;
        Buffer& operator=(Buffer&&) = default;
        Buffer(const Buffer&) = delete;
        Buffer& operator=(const Buffer&) = delete;
    };

    uint32_t                 frameNumber;
    CameraMetadata           settings;
    std::unique_ptr<Buffer>  inputBuffer;
    std::vector<Buffer>      outputBuffers;

    CaptureRequest() : frameNumber(0) {}

    CaptureRequest(CaptureRequest&&) = default;
    CaptureRequest& operator=(CaptureRequest&&) = default;
    CaptureRequest(const CaptureRequest&) = delete;
    CaptureRequest& operator=(const CaptureRequest&) = delete;
};

} /* namespace android */

#endif /* HAL_PIPELINE_CAPTURE_REQUEST_H */
