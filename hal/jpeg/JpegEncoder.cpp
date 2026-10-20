#define LOG_TAG "Cam-JPEG"

#include "JpegEncoder.h"

#include <setjmp.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <vector>

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
 * stream at `dst`. `app1` (optional) is written as the APP1 marker —
 * the EXIF block, so viewers rotate on display instead of us rotating
 * pixels in HAL.
 *
 * Returns the number of bytes written, or 0 on failure (libjpeg error,
 * or output exceeded `dstLen`). */
size_t encodeRgbaToJpeg(const uint8_t *rgba, unsigned width, unsigned height,
                         uint8_t quality,
                         const uint8_t *app1, size_t app1Len,
                         uint8_t *dst, size_t dstLen) {
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

    if (app1 && app1Len)
        jpeg_write_marker(&cinfo, JPEG_APP0 + 1, app1, (unsigned)app1Len);

    /* RGBA → RGB row by row (libjpeg doesn't consume alpha). */
    while (cinfo.next_scanline < cinfo.image_height) {
        const uint8_t *srcRow = rgba + (size_t)cinfo.next_scanline * width * 4;
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

/* Downscale the RGBA image to an RGBA thumbnail of tw x th: crop the
 * centre to the thumbnail's aspect ratio, then average each source
 * block. A few ms for 320x240 out of a 1080p snapshot. */
void downscaleRgba(const uint8_t *src, unsigned sw, unsigned sh,
                   uint8_t *dst, unsigned tw, unsigned th) {
    /* Crop: the largest centred rectangle with the thumbnail's aspect. */
    unsigned cw = sw, ch = sh;
    if ((uint64_t)sw * th > (uint64_t)sh * tw)
        cw = (unsigned)((uint64_t)sh * tw / th);
    else
        ch = (unsigned)((uint64_t)sw * th / tw);
    const unsigned cx = (sw - cw) / 2;
    const unsigned cy = (sh - ch) / 2;

    for (unsigned y = 0; y < th; ++y) {
        const unsigned y0 = cy + (unsigned)((uint64_t)y * ch / th);
        unsigned y1 = cy + (unsigned)((uint64_t)(y + 1) * ch / th);
        if (y1 <= y0) y1 = y0 + 1;
        for (unsigned x = 0; x < tw; ++x) {
            const unsigned x0 = cx + (unsigned)((uint64_t)x * cw / tw);
            unsigned x1 = cx + (unsigned)((uint64_t)(x + 1) * cw / tw);
            if (x1 <= x0) x1 = x0 + 1;
            uint32_t r = 0, g = 0, b = 0, n = 0;
            for (unsigned yy = y0; yy < y1; ++yy) {
                const uint8_t *p = src + ((size_t)yy * sw + x0) * 4;
                for (unsigned xx = x0; xx < x1; ++xx, p += 4) {
                    r += p[0]; g += p[1]; b += p[2]; ++n;
                }
            }
            uint8_t *d = dst + ((size_t)y * tw + x) * 4;
            d[0] = (uint8_t)(r / n);
            d[1] = (uint8_t)(g / n);
            d[2] = (uint8_t)(b / n);
            d[3] = 0xff;
        }
    }
}

void put16(std::vector<uint8_t> &v, uint16_t x) {
    v.push_back((uint8_t)(x & 0xff));
    v.push_back((uint8_t)(x >> 8));
}

void put32(std::vector<uint8_t> &v, uint32_t x) {
    for (int i = 0; i < 4; ++i) v.push_back((uint8_t)(x >> (8 * i)));
}

/* One little-endian IFD entry whose value fits the 4-byte field. */
void putEntry(std::vector<uint8_t> &v, uint16_t tag, uint16_t type,
              uint32_t value) {
    put16(v, tag);
    put16(v, type);
    put32(v, 1);                       /* count */
    if (type == 3) {                   /* SHORT: left-justified */
        put16(v, (uint16_t)value);
        put16(v, 0);
    } else {                           /* LONG */
        put32(v, value);
    }
}

/* The EXIF APP1 payload: IFD0 with Orientation, and when a thumbnail
 * is given, IFD1 with the JPEG thumbnail after it (Compression = 6,
 * JPEGInterchangeFormat / Length). Offsets are from the TIFF header. */
void buildExifApp1(uint16_t orientation,
                   const uint8_t *thumb, size_t thumbLen,
                   std::vector<uint8_t> *out) {
    std::vector<uint8_t> &v = *out;
    v.clear();
    const uint8_t hdr[] = { 'E','x','i','f', 0x00, 0x00 };
    v.insert(v.end(), hdr, hdr + sizeof(hdr));

    /* TIFF header, little-endian, IFD0 right after it. */
    v.push_back('I'); v.push_back('I');
    put16(v, 0x002A);
    put32(v, 8);

    const uint32_t ifd0Size = 2 + 12 * 1 + 4;
    const uint32_t ifd1Off  = 8 + ifd0Size;
    const uint32_t ifd1Size = 2 + 12 * 3 + 4;
    const uint32_t thumbOff = ifd1Off + ifd1Size;

    /* IFD0: Orientation. */
    put16(v, 1);
    putEntry(v, 0x0112, 3, orientation);
    put32(v, thumb ? ifd1Off : 0);

    if (!thumb) return;

    /* IFD1: the thumbnail. */
    put16(v, 3);
    putEntry(v, 0x0103, 3, 6);                      /* Compression: JPEG */
    putEntry(v, 0x0201, 4, thumbOff);               /* JPEGInterchangeFormat */
    putEntry(v, 0x0202, 4, (uint32_t)thumbLen);     /* ...Length */
    put32(v, 0);
    v.insert(v.end(), thumb, thumb + thumbLen);
}

/* APP1 holds at most 65533 bytes of payload; the thumbnail has to fit
 * in what the EXIF around it leaves. */
constexpr size_t kMaxApp1Payload = 65533;
constexpr size_t kExifOverhead   = 6 + 8 + (2 + 12 + 4) + (2 + 12 * 3 + 4);

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

    /* Thumbnail, when the request asks for one (0x0 means none). */
    int32_t thumbW = 0, thumbH = 0;
    if (metadata.exists(ANDROID_JPEG_THUMBNAIL_SIZE)) {
        const camera_metadata_ro_entry e = metadata.find(ANDROID_JPEG_THUMBNAIL_SIZE);
        if (e.count >= 2) { thumbW = e.data.i32[0]; thumbH = e.data.i32[1]; }
    }
    uint8_t thumbQuality = 90;
    if (metadata.exists(ANDROID_JPEG_THUMBNAIL_QUALITY))
        thumbQuality = *metadata.find(ANDROID_JPEG_THUMBNAIL_QUALITY).data.u8;

    std::vector<uint8_t> thumb;
    if (thumbW > 0 && thumbH > 0) {
        std::vector<uint8_t> small((size_t)thumbW * thumbH * 4);
        downscaleRgba(src.rgba, src.width, src.height,
                      small.data(), (unsigned)thumbW, (unsigned)thumbH);
        thumb.resize(kMaxApp1Payload - kExifOverhead);
        const size_t n = encodeRgbaToJpeg(small.data(), (unsigned)thumbW, (unsigned)thumbH,
                                           thumbQuality, nullptr, 0,
                                           thumb.data(), thumb.size());
        if (n == 0) {
            /* Too big for the EXIF block even at this size: the JPEG
             * goes out without one rather than not at all. */
            ALOGW("thumbnail %dx%d q%u does not fit the EXIF block; omitted",
                  thumbW, thumbH, thumbQuality);
            thumb.clear();
        } else {
            thumb.resize(n);
        }
    }

    std::vector<uint8_t> app1;
    buildExifApp1(exifOrientationFromAndroid(jpegOri),
                  thumb.empty() ? nullptr : thumb.data(), thumb.size(), &app1);

    size_t written = encodeRgbaToJpeg(src.rgba, src.width, src.height,
                                        jpegQuality, app1.data(), app1.size(),
                                        dst, dstCapacity);
    if (written == 0) {
        ALOGE("JPEG encode failed (buffer %zu bytes)", dstCapacity);
        return false;
    }
    *bytesWritten = written;
    return true;
}

}; /* namespace android */
