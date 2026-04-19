#ifndef HAL_JPEG_ENCODER_H
#define HAL_JPEG_ENCODER_H

#include <stddef.h>
#include <stdint.h>
#include <camera/CameraMetadata.h>

namespace android {

class IspPipeline;

/* Raw frame descriptor passed to JpegEncoder — the subset of the V4L2
 * frame state JPEG encoding needs. `frameBuf` is the MMAP pointer and
 * is NULL in DMABUF mode; in that case `srcInputSlot` identifies the
 * Bayer slot the ISP should read from. */
struct JpegSource {
    const uint8_t *frameBuf;
    int            srcInputSlot;
    uint32_t       pixFmt;
    unsigned       width;
    unsigned       height;
};

/* Produces a HAL_PIXEL_FORMAT_BLOB JPEG from a raw Bayer V4L2 frame.
 * Pipeline: ISP demosaic → libjpeg encode with EXIF Orientation tag. */
class JpegEncoder {
public:
    explicit JpegEncoder(IspPipeline *isp);

    /* Encode `src` into the BLOB buffer `dst` (reserved `bufferSize`
     * bytes, which must include the camera3_jpeg_blob footer).
     *
     * cm: reads ANDROID_JPEG_QUALITY and (Bayer path only)
     *     ANDROID_JPEG_ORIENTATION.
     *
     * Returns true on success, with the jpeg_blob footer written at the
     * end of `dst`. */
    bool encode(uint8_t *dst, size_t bufferSize,
                const JpegSource &src,
                const CameraMetadata &cm) const;

private:
    IspPipeline *mIsp;
};

}; /* namespace android */

#endif /* HAL_JPEG_ENCODER_H */
