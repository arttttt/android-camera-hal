#define LOG_TAG "Cam-JPEG"

#include "JpegEncoder.h"

#include <setjmp.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <jpeglib.h>

#include <hardware/camera3.h>
#include <system/camera_metadata.h>
#include <utils/Log.h>

namespace android {

namespace {

constexpr uint8_t kDefaultJpegQuality = 95;

/* libjpeg's default error_exit calls exit(): one bad encode would take
 * the whole camera provider down with it. This one logs the message
 * and jumps back to the encoder, which reports the encode as failed. */
struct JpegErrorMgr {
    struct jpeg_error_mgr pub;
    jmp_buf               jump;
};

void jpegErrorExit(j_common_ptr cinfo) {
    JpegErrorMgr *err = reinterpret_cast<JpegErrorMgr *>(cinfo->err);
    char msg[JMSG_LENGTH_MAX];
    (*cinfo->err->format_message)(cinfo, msg);
    ALOGE("libjpeg: %s", msg);
    longjmp(err->jump, 1);
}

/* Map ANDROID_JPEG_ORIENTATION (CW degrees) to EXIF Orientation tag:
 *   0°  → 1 (normal)
 *   90° → 6 (rotate 90 CW on display)
 *   180°→ 3
 *   270°→ 8 (rotate 270 CW / 90 CCW)
 * Any other value falls through to 1. */
uint16_t exifOrientationFromAndroid(int32_t androidOrientation) {
    switch (androidOrientation) {
        case 90:  return 6;
        case 180: return 3;
        case 270: return 8;
        case 0:
        default:  return 1;
    }
}

/* Encode RGBA (tightly packed, row stride = width*4) into a JPEG byte
 * stream at `dst`, injecting an EXIF Orientation APP1 marker so viewers
 * rotate on display instead of us rotating pixels in HAL.
 *
 * Returns the number of bytes written, or 0 on failure (libjpeg error,
 * or output exceeded `dstLen`). */
size_t encodeRgbaToJpeg(const uint8_t *rgba, uint8_t *dst, size_t dstLen,
                         unsigned width, unsigned height,
                         uint8_t quality, int32_t androidOrientation) {
    struct jpeg_compress_struct cinfo;
    JpegErrorMgr jerr;

    /* Everything the error path frees is set up before setjmp and is
     * volatile where it changes after, so the longjmp leaves no stale
     * copy in a register. No C++ object with a destructor lives across
     * the jump. */
    uint8_t *row = (uint8_t *)malloc((size_t)width * 3);
    if (!row) return 0;
    unsigned long  outSize = dstLen;
    unsigned char *volatile outBuf = dst;

    cinfo.err = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = jpegErrorExit;
    if (setjmp(jerr.jump)) {
        jpeg_destroy_compress(&cinfo);
        if (outBuf != dst) free(outBuf);
        free(row);
        return 0;
    }
    jpeg_create_compress(&cinfo);

    jpeg_mem_dest(&cinfo, (unsigned char **)&outBuf, &outSize);

    cinfo.image_width      = width;
    cinfo.image_height     = height;
    cinfo.input_components = 3;
    cinfo.in_color_space   = JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);

    jpeg_start_compress(&cinfo, TRUE);

    uint16_t exifOri = exifOrientationFromAndroid(androidOrientation);
    const uint8_t exifApp1[] = {
        'E','x','i','f', 0x00, 0x00,
        'I','I', 0x2A, 0x00,
        0x08, 0x00, 0x00, 0x00,
        0x01, 0x00,
        0x12, 0x01,
        0x03, 0x00,
        0x01, 0x00, 0x00, 0x00,
        (uint8_t)(exifOri & 0xFF), (uint8_t)(exifOri >> 8), 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
    };
    jpeg_write_marker(&cinfo, JPEG_APP0 + 1, exifApp1, sizeof(exifApp1));

    /* RGBA → RGB row by row (libjpeg doesn't consume alpha). */
    while (cinfo.next_scanline < cinfo.image_height) {
        const uint8_t *srcRow = rgba + cinfo.next_scanline * width * 4;
        for (unsigned x = 0; x < width; x++) {
            row[x * 3 + 0] = srcRow[x * 4 + 0];
            row[x * 3 + 1] = srcRow[x * 4 + 1];
            row[x * 3 + 2] = srcRow[x * 4 + 2];
        }
        JSAMPROW rowPtr = row;
        jpeg_write_scanlines(&cinfo, &rowPtr, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
    free(row);

    /* jpeg_mem_dest may realloc if the output didn't fit. Copy back
     * to the caller's buffer; fail if it grew past dstLen. */
    if (outBuf != dst) {
        if (outSize <= dstLen) {
            memcpy(dst, outBuf, outSize);
        } else {
            ALOGE("JPEG output %lu > buffer %zu", outSize, dstLen);
            free(outBuf);
            return 0;
        }
        free(outBuf);
    }

    return outSize;
}

} /* namespace */

bool JpegEncoder::process(const JpegSnapshot &src,
                           uint8_t *dst, size_t dstCapacity,
                           const CameraMetadata &metadata,
                           size_t *bytesWritten) {
    if (bytesWritten) *bytesWritten = 0;
    if (!src.rgba || !dst || !bytesWritten) return false;

    uint8_t jpegQuality = kDefaultJpegQuality;
    if (metadata.exists(ANDROID_JPEG_QUALITY))
        jpegQuality = *metadata.find(ANDROID_JPEG_QUALITY).data.u8;

    int32_t jpegOri = 0;
    if (metadata.exists(ANDROID_JPEG_ORIENTATION))
        jpegOri = *metadata.find(ANDROID_JPEG_ORIENTATION).data.i32;

    size_t written = encodeRgbaToJpeg(src.rgba, dst, dstCapacity,
                                        src.width, src.height,
                                        jpegQuality, jpegOri);
    if (written == 0) {
        ALOGE("JPEG image too big for %zu byte buffer", dstCapacity);
        return false;
    }
    *bytesWritten = written;
    return true;
}

}; /* namespace android */
