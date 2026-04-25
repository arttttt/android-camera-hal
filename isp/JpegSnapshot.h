#ifndef ISP_JPEG_SNAPSHOT_H
#define ISP_JPEG_SNAPSHOT_H

#include <stddef.h>
#include <stdint.h>

namespace android {

/* Handle to a host-mapped RGBA8 copy of the ISP scratch image, captured
 * at endFrame time inside the same submit as preview / video blits. The
 * `rgba` pointer becomes valid for CPU read once the frame's submit
 * fence has signalled and IspPipeline::invalidateJpegSnapshot has been
 * called; it stays valid until releaseJpegSnapshot is invoked.
 *
 * The backend owns a small ring of these buffers; JpegWorker holds a
 * snapshot for the duration of one libjpeg encode and releases it
 * afterwards so the slot rotates back. ringSlot is opaque to
 * consumers — pass the snapshot back to IspPipeline unchanged. */
struct JpegSnapshot {
    const uint8_t *rgba;
    unsigned       width;
    unsigned       height;
    size_t         size;     /* = width * height * 4 */
    int            ringSlot; /* backend-internal handle; -1 = invalid */
};

}; /* namespace android */

#endif /* ISP_JPEG_SNAPSHOT_H */
