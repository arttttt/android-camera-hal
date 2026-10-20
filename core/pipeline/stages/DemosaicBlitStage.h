#ifndef HAL_PIPELINE_STAGES_DEMOSAIC_BLIT_STAGE_H
#define HAL_PIPELINE_STAGES_DEMOSAIC_BLIT_STAGE_H

#include <stddef.h>

#include "UniqueFd.h"

namespace mocha::camera {
struct Frame;
class FrameSource;
}

namespace android {

class BufferProcessor;
class DeviceErrorReporter;
class IspFrameRecorder;
class IspJpegRing;

/* Opens the ISP recording for the frame, runs BufferProcessor::processOne
 * for every output buffer (each appending a per-output blit / encode to
 * the open recording), then closes the recording with endFrame to kick off
 * a single GPU submit. Fills in the frame's outputs (release fences,
 * statuses, JPEG snapshots) and returns the submit's
 * completion sync_fd, which the demosaic engine waits on; invalid when
 * nothing was submitted. */
class DemosaicBlitStage {
public:
    struct Deps {
        BufferProcessor     *bufferProcessor;
        mocha::camera::FrameSource *source;  /* for resolution() */
        IspFrameRecorder    *recorder;
        IspJpegRing         *jpegRing;   /* the frame's JPEG snapshots */
        DeviceErrorReporter *deviceError;   /* may be null */
    };

    explicit DemosaicBlitStage(const Deps &deps);

    UniqueFd process(mocha::camera::Frame &frame);

private:
    void reportIfDeviceLost();

    Deps deps;
};

} /* namespace android */

#endif /* HAL_PIPELINE_STAGES_DEMOSAIC_BLIT_STAGE_H */
