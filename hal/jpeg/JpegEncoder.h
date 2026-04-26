#ifndef HAL_JPEG_ENCODER_H
#define HAL_JPEG_ENCODER_H

#include <stddef.h>
#include <stdint.h>
#include <camera/CameraMetadata.h>

#include "PostProcessor.h"

namespace android {

/* libjpeg-backed BLOB encoder. Consumes a JpegSnapshot (host-mapped RGBA
 * already populated by the ISP into one of the JPEG ring slots) and
 * writes a JPEG with an EXIF Orientation APP1 marker into the caller's
 * output buffer. The caller is responsible for calling
 * IspPipeline::invalidateJpegSnapshot before invoking process() and
 * releaseJpegSnapshot afterwards.
 *
 * No ISP dependency — the snapshot is the only producer-side input. */
class JpegEncoder : public PostProcessor {
public:
    JpegEncoder() {}

    bool process(const JpegSnapshot &src,
                  uint8_t *dst, size_t dstCapacity,
                  const CameraMetadata &metadata,
                  size_t *bytesWritten) override;
};

}; /* namespace android */

#endif /* HAL_JPEG_ENCODER_H */
