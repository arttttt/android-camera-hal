#define LOG_TAG "Cam-JPEG"

#include "JpegEncoder.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <linux/videodev2.h>

#include <jpeglib.h>

#include <hardware/camera3.h>
#include <system/camera_metadata.h>
#include <utils/Log.h>

#include "ImageConverter.h"
#include "IspPipeline.h"

namespace android {

namespace {

constexpr uint8_t kDefaultJpegQuality = 95;

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

/* Encode RGBA (tightly packed, row stride = width*4) into a JPEG BLOB
 * at `dst`, injecting an EXIF Orientation APP1 marker so viewers
 * rotate on display instead of us rotating pixels in HAL.
 *
 * Returns pointer past the last written byte on success, or `dst` on
 * failure (output exceeded `dstLen`). */
uint8_t *encodeRgbaToJpeg(const uint8_t *rgba, uint8_t *dst, size_t dstLen,
                           unsigned width, unsigned height,
                           uint8_t quality, int32_t androidOrientation) {
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    unsigned long outSize  = dstLen;
    unsigned char *outBuf  = dst;
    jpeg_mem_dest(&cinfo, &outBuf, &outSize);

    cinfo.image_width      = width;
    cinfo.image_height     = height;
    cinfo.input_components = 3;
    cinfo.in_color_space   = JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);

    jpeg_start_compress(&cinfo, TRUE);

    /* EXIF APP1 segment: just the Orientation tag. Viewers that honour
     * EXIF (every modern gallery app) will apply the rotation on
     * display, so the HAL never rotates pixels. */
    uint16_t exifOri = exifOrientationFromAndroid(androidOrientation);
    const uint8_t exifApp1[] = {
        /* "Exif\0\0" APP1 identifier */
        'E','x','i','f', 0x00, 0x00,
        /* TIFF header — little-endian, first IFD at offset 8 */
        'I','I', 0x2A, 0x00,
        0x08, 0x00, 0x00, 0x00,
        /* IFD0: 1 entry, followed by next-IFD pointer */
        0x01, 0x00,
        /* Orientation entry: tag 0x0112, SHORT (type 3), count 1 */
        0x12, 0x01,
        0x03, 0x00,
        0x01, 0x00, 0x00, 0x00,
        (uint8_t)(exifOri & 0xFF), (uint8_t)(exifOri >> 8), 0x00, 0x00,
        /* Next IFD = 0 (none) */
        0x00, 0x00, 0x00, 0x00,
    };
    jpeg_write_marker(&cinfo, JPEG_APP0 + 1, exifApp1, sizeof(exifApp1));

    /* RGBA → RGB row by row (libjpeg doesn't consume alpha). */
    uint8_t *row = new uint8_t[width * 3];
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
    delete[] row;

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    /* jpeg_mem_dest may realloc if the output didn't fit. Copy back
     * to the caller's buffer; fail if it grew past dstLen. */
    if (outBuf != dst) {
        if (outSize <= dstLen) {
            memcpy(dst, outBuf, outSize);
        } else {
            ALOGE("JPEG output %lu > buffer %zu", outSize, dstLen);
            free(outBuf);
            return dst;
        }
        free(outBuf);
    }

    return dst + outSize;
}

} /* namespace */

JpegEncoder::JpegEncoder(IspPipeline *isp, ImageConverter *converter)
    : mIsp(isp)
    , mConverter(converter) {
}

bool JpegEncoder::encode(uint8_t *dst, size_t bufferSize,
                         const JpegSource &src,
                         const CameraMetadata &cm,
                         uint8_t *rgbaScratch) const {
    const size_t maxImageSize = bufferSize - sizeof(camera3_jpeg_blob);

    uint8_t jpegQuality = kDefaultJpegQuality;
    if (cm.exists(ANDROID_JPEG_QUALITY))
        jpegQuality = *cm.find(ANDROID_JPEG_QUALITY).data.u8;
    ALOGD("JPEG quality = %u", jpegQuality);

    uint8_t *bufEnd = NULL;
    if (src.pixFmt == V4L2_PIX_FMT_UYVY) {
        bufEnd = mConverter->UYVYToJPEG(src.frameBuf, dst,
                                         src.width, src.height,
                                         maxImageSize, jpegQuality);
    } else if (src.pixFmt == V4L2_PIX_FMT_YUYV) {
        bufEnd = mConverter->YUY2ToJPEG(src.frameBuf, dst,
                                         src.width, src.height,
                                         maxImageSize, jpegQuality);
    } else {
        /* Bayer: GPU demosaic straight into the ISP's CPU-mapped
         * buffer, no intermediate RGBA copy. libjpeg reads from that
         * buffer and writes JPEG into `dst`. Orientation is encoded
         * in the EXIF APP1 marker — no pixel rotation in HAL. */
        (void)rgbaScratch; /* unused on the Bayer path; removed in t1.5.3.3 */
        const uint8_t *rgba = mIsp->processToCpu(src.frameBuf,
                                                  src.width, src.height,
                                                  src.pixFmt, src.srcInputSlot);
        if (!rgba) {
            ALOGE("processToCpu failed");
            return false;
        }

        int32_t jpegOri = 0;
        if (cm.exists(ANDROID_JPEG_ORIENTATION))
            jpegOri = *cm.find(ANDROID_JPEG_ORIENTATION).data.i32;

        bufEnd = encodeRgbaToJpeg(rgba, dst, maxImageSize,
                                   src.width, src.height,
                                   jpegQuality, jpegOri);
    }

    if (bufEnd == dst) {
        ALOGE("JPEG image too big!");
        return false;
    }

    camera3_jpeg_blob *jpegBlob = reinterpret_cast<camera3_jpeg_blob *>(dst + maxImageSize);
    jpegBlob->jpeg_blob_id = CAMERA3_JPEG_BLOB_ID;
    jpegBlob->jpeg_size    = (uint32_t)(bufEnd - dst);
    return true;
}

}; /* namespace android */
