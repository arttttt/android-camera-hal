#ifndef HAL_PIPELINE_POSTPROCESSOR_H
#define HAL_PIPELINE_POSTPROCESSOR_H

#include <stddef.h>
#include <stdint.h>

#include <camera/CameraMetadata.h>

#include "JpegSnapshot.h"

namespace android {

/* Producer of a CPU-finalised output buffer from a JpegSnapshot.
 *
 * Concrete implementation today: JpegEncoder (libjpeg-backed). Future
 * candidates: an NV21 / NV12 packer for YUV outputs that need a layout
 * the GPU compute path doesn't write, or a HEIF encoder.
 *
 * Thread model: one PostProcessor instance per stream type, called from
 * either the main pipeline thread (synchronous finalize) or a dedicated
 * worker thread (asynchronous encode). Implementations must be
 * re-entrant only across DIFFERENT instances; a single instance is
 * called serially. */
class PostProcessor {
public:
    virtual ~PostProcessor() {}

    /* Encode `src` into the caller-provided byte buffer `dst` of
     * `dstCapacity` bytes. On success returns true and writes the
     * number of bytes consumed in `*bytesWritten` (must be non-null).
     * On failure returns false; *bytesWritten is undefined.
     *
     * `metadata` carries encoder-specific request knobs (JPEG quality,
     * orientation, GPS, ...). The consumer guarantees src.rgba is valid
     * (post-fence-reap + invalidate) for the duration of this call. */
    virtual bool process(const JpegSnapshot &src,
                          uint8_t *dst, size_t dstCapacity,
                          const CameraMetadata &metadata,
                          size_t *bytesWritten) = 0;
};

}; /* namespace android */

#endif /* HAL_PIPELINE_POSTPROCESSOR_H */
