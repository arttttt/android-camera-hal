#ifndef HAL_JPEG_ENCODER_H
#define HAL_JPEG_ENCODER_H

#include <stddef.h>
#include <stdint.h>

#include "PostProcessor.h"

namespace android {

/* libjpeg-backed BLOB encoder. Consumes a JpegSnapshot (host-mapped RGBA
 * already populated by the ISP into one of the JPEG ring slots) and
 * writes a JPEG into the caller's output buffer, with an EXIF APP1
 * marker (buildExifApp1) and, when the request asks for one (a
 * thumbnail size in its JPEG settings), a JPEG thumbnail in it. The caller is responsible for calling
 * IspJpegRing::invalidateJpegSnapshot before invoking process() and
 * releaseJpegSnapshot afterwards.
 *
 * No ISP dependency — the snapshot is the only producer-side input. */
class JpegEncoder : public PostProcessor {
public:
    JpegEncoder() {}

    bool process(const JpegSnapshot &src,
                  unsigned width, unsigned height,
                  uint8_t *dst, size_t dstCapacity,
                  const mocha::camera::FrameSettings::Jpeg &settings,
                  const ExifInfo &exif,
                  size_t *bytesWritten) override;
};

}; /* namespace android */

#endif /* HAL_JPEG_ENCODER_H */
