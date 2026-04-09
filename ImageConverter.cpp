/*
 * Copyright (C) 2015-2016 Antmicro
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <YuvToJpegEncoder.h>
#include <SkStream.h>
#include <libyuv/row.h>
#include <linux/videodev2.h>

#include <utils/Log.h>

#include "Yuv422UyvyToJpegEncoder.h"
#include "ImageConverter.h"
#include "DbgUtils.h"

#define LOG_TAG "Cam-ImageConv"

#define WORKERS_TASKS_NUM 30

/* kMaxStride was removed in newer libyuv; 4096 pixels * 4 bytes is plenty */
#ifndef kMaxStride
#define kMaxStride 16384
#endif

namespace android {

ImageConverter::ImageConverter() {
}

ImageConverter::~ImageConverter() {
}

uint8_t *ImageConverter::YUY2ToRGBA(const uint8_t *src, uint8_t *dst, unsigned width, unsigned height) {
    assert(gWorkers.isRunning());
    assert(src != NULL);
    assert(dst != NULL);
    assert(width > 0);
    assert(height > 0);

    Workers::Task::Function taskFn = [](void *data) {
        ConvertTask::Data *d = static_cast<ConvertTask::Data *>(data);

        SIMD_ALIGNED(uint8 rowy[kMaxStride]);
        SIMD_ALIGNED(uint8 rowu[kMaxStride]);
        SIMD_ALIGNED(uint8 rowv[kMaxStride]);

        for(size_t i = 0; i < d->linesNum; ++i) {
            libyuv::YUY2ToUV422Row_NEON(d->src, rowu, rowv, d->width);
            libyuv::YUY2ToYRow_NEON(d->src, rowy, d->width);
            libyuv::I422ToARGBRow_NEON(rowy, rowu, rowv, d->dst,
                &libyuv::kYuvI601Constants, d->width);
            d->src += d->width * 2;
            d->dst += d->width * 4;
        }
    };

    return splitRunWait(src, dst, width, height, taskFn);
}

uint8_t *ImageConverter::YUY2ToJPEG(const uint8_t *src, uint8_t *dst, unsigned width, unsigned height, size_t dstLen, uint8_t quality) {
    assert(src != NULL);
    assert(dst != NULL);
    assert(width > 0);
    assert(height > 0);
    assert(dstLen > 0);
    assert(quality <= 100);

    /* TODO: do it parallel with libjpeg */

    int strides[] = { (int)width * 2 };
    int offsets[] = { 0 };
    Yuv422IToJpegEncoder encoder(strides);
    SkDynamicMemoryWStream stream;

    encoder.encode(&stream, (void *)src, (int)width, (int)height, offsets, quality);

    if(stream.getOffset() > dstLen)
        return dst;

    stream.copyTo(dst);
    return dst + stream.getOffset();
}

uint8_t *ImageConverter::UYVYToRGBA(const uint8_t *src, uint8_t *dst, unsigned width, unsigned height) {
    assert(gWorkers.isRunning());
    assert(src != NULL);
    assert(dst != NULL);
    assert(width > 0);
    assert(height > 0);

    Workers::Task::Function taskFn = [](void *data) {
        ConvertTask::Data *d = static_cast<ConvertTask::Data *>(data);

        SIMD_ALIGNED(uint8 rowy[kMaxStride]);
        SIMD_ALIGNED(uint8 rowu[kMaxStride]);
        SIMD_ALIGNED(uint8 rowv[kMaxStride]);

        for(size_t i = 0; i < d->linesNum; ++i) {
            libyuv::UYVYToUV422Row_NEON(d->src, rowu, rowv, d->width);
            libyuv::UYVYToYRow_NEON(d->src, rowy, d->width);
            libyuv::I422ToARGBRow_NEON(rowy, rowu, rowv, d->dst,
                &libyuv::kYuvI601Constants, d->width);
            d->src += d->width * 2;
            d->dst += d->width * 4;
        }
    };

    return splitRunWait(src, dst, width, height, taskFn);
}

uint8_t *ImageConverter::UYVYToJPEG(const uint8_t *src, uint8_t *dst, unsigned width, unsigned height, size_t dstLen, uint8_t quality) {
    assert(src != NULL);
    assert(dst != NULL);
    assert(width > 0);
    assert(height > 0);
    assert(dstLen > 0);
    assert(quality <= 100);

    /* TODO: do it parallel with libjpeg */

    int strides[] = { (int)width * 2 };
    int offsets[] = { 0 };
    Yuv422UyvyToJpegEncoder encoder(strides);
    SkDynamicMemoryWStream stream;

    encoder.encode(&stream, (void *)src, (int)width, (int)height, offsets, quality);

    if(stream.getOffset() > dstLen)
        return dst;

    stream.copyTo(dst);
    return dst + stream.getOffset();
}

uint8_t * ImageConverter::splitRunWait(const uint8_t *src, uint8_t *dst, unsigned width, unsigned height, Workers::Task::Function fn) {
    ConvertTask tasks[WORKERS_TASKS_NUM];

    const uint8_t   *srcPtr = src;
    uint8_t         *dstPtr = dst;
    const size_t linesPerTask = (height + WORKERS_TASKS_NUM - 1) / WORKERS_TASKS_NUM;
    for(size_t i = 0; i < WORKERS_TASKS_NUM; ++i) {
        tasks[i].data.src       = srcPtr;
        tasks[i].data.dst       = dstPtr;
        tasks[i].data.width     = width;
        tasks[i].data.linesNum  = linesPerTask;
        if((i + 1) * linesPerTask >= height) {
            tasks[i].data.linesNum = height - i * linesPerTask;
        }

        tasks[i].task = Workers::Task(fn, (void *)&tasks[i].data);
        gWorkers.queueTask(&tasks[i].task);

        srcPtr += linesPerTask * width * 2;
        dstPtr += linesPerTask * width * 4;
    }

    for(size_t i = 0; i < WORKERS_TASKS_NUM; ++i) {
        tasks[i].task.waitForCompletion();
    }

    return dstPtr;
}

/*
 * Bayer pattern red-pixel position from V4L2 fourcc.
 * rX/rY = position of red pixel within 2x2 tile.
 */
static inline void bayerPattern(uint32_t pixFmt, int *rX, int *rY) {
    switch (pixFmt) {
        case V4L2_PIX_FMT_SRGGB8:
        case V4L2_PIX_FMT_SRGGB10: *rX = 0; *rY = 0; break;
        case V4L2_PIX_FMT_SGRBG8:
        case V4L2_PIX_FMT_SGRBG10: *rX = 1; *rY = 0; break;
        case V4L2_PIX_FMT_SGBRG8:
        case V4L2_PIX_FMT_SGBRG10: *rX = 0; *rY = 1; break;
        case V4L2_PIX_FMT_SBGGR8:
        case V4L2_PIX_FMT_SBGGR10: *rX = 1; *rY = 1; break;
        default:                    *rX = 0; *rY = 0; break;
    }
}

static inline bool bayerIs16bit(uint32_t pixFmt) {
    switch (pixFmt) {
        case V4L2_PIX_FMT_SRGGB10:
        case V4L2_PIX_FMT_SGRBG10:
        case V4L2_PIX_FMT_SGBRG10:
        case V4L2_PIX_FMT_SBGGR10:
            return true;
        default:
            return false;
    }
}

/* sRGB gamma LUT — computed once */
static uint8_t sGammaLut[256];
static bool sGammaReady = false;

static void initGammaLut() {
    if (sGammaReady) return;
    for (int i = 0; i < 256; i++) {
        float lin = i / 255.0f;
        float s = (lin <= 0.0031308f) ?
            lin * 12.92f :
            1.055f * powf(lin, 1.0f / 2.4f) - 0.055f;
        int v = (int)(s * 255.0f + 0.5f);
        sGammaLut[i] = v > 255 ? 255 : (v < 0 ? 0 : (uint8_t)v);
    }
    sGammaReady = true;
}

/*
 * Combined demosaic + AWB + CCM + gamma in one pass per line.
 * Avoids second pass over 8MB RGBA buffer.
 */
static inline void demosaicIspLine(const uint8_t *src, uint8_t *dst,
        unsigned width, unsigned height, unsigned y, uint32_t pixFmt,
        bool doIsp, const int16_t *ccm, unsigned wbR, unsigned wbG, unsigned wbB,
        uint64_t *accR, uint64_t *accG, uint64_t *accB) {
    int rX, rY;
    bayerPattern(pixFmt, &rX, &rY);
    bool is16 = bayerIs16bit(pixFmt);
    unsigned bpp = is16 ? 2 : 1;
    unsigned stride = width * bpp;

    const uint8_t *rowBytes  = src + y * stride;
    const uint8_t *rowBytesU = (y > 0)          ? rowBytes - stride : rowBytes;
    const uint8_t *rowBytesD = (y < height - 1) ? rowBytes + stride : rowBytes;
    uint8_t *out = dst + y * width * 4;

    uint64_t sR = 0, sG = 0, sB = 0;

    #define PX(row, x) (is16 ? ((const uint16_t *)(row))[x] >> 2 : (row)[x])

    for (unsigned x = 0; x < width; x++) {
        unsigned c  = PX(rowBytes, x);
        unsigned l  = (x > 0)         ? PX(rowBytes, x - 1) : c;
        unsigned r  = (x < width - 1) ? PX(rowBytes, x + 1) : c;
        unsigned u  = PX(rowBytesU, x);
        unsigned d  = PX(rowBytesD, x);
        int px = x & 1;
        int py = y & 1;

        int R, G, B;
        if (py == rY && px == rX) {
            R = c; G = (l + r + u + d) / 4;
            unsigned ul = (x > 0) ? PX(rowBytesU, x-1) : u;
            unsigned ur = (x < width-1) ? PX(rowBytesU, x+1) : u;
            unsigned dl = (x > 0) ? PX(rowBytesD, x-1) : d;
            unsigned dr = (x < width-1) ? PX(rowBytesD, x+1) : d;
            B = (ul + ur + dl + dr) / 4;
        } else if (py != rY && px != rX) {
            B = c; G = (l + r + u + d) / 4;
            unsigned ul = (x > 0) ? PX(rowBytesU, x-1) : u;
            unsigned ur = (x < width-1) ? PX(rowBytesU, x+1) : u;
            unsigned dl = (x > 0) ? PX(rowBytesD, x-1) : d;
            unsigned dr = (x < width-1) ? PX(rowBytesD, x+1) : d;
            R = (ul + ur + dl + dr) / 4;
        } else {
            G = c;
            if (py == rY) { R = (l + r) / 2; B = (u + d) / 2; }
            else           { B = (l + r) / 2; R = (u + d) / 2; }
        }

        if (doIsp) {
            /* AWB */
            R = (R * wbR) >> 8; if (R > 255) R = 255;
            G = (G * wbG) >> 8; if (G > 255) G = 255;
            B = (B * wbB) >> 8; if (B > 255) B = 255;

            /* CCM (Q10) */
            int rr = (ccm[0]*R + ccm[1]*G + ccm[2]*B) >> 10;
            int gg = (ccm[3]*R + ccm[4]*G + ccm[5]*B) >> 10;
            int bb = (ccm[6]*R + ccm[7]*G + ccm[8]*B) >> 10;
            if (rr < 0) rr = 0; if (rr > 255) rr = 255;
            if (gg < 0) gg = 0; if (gg > 255) gg = 255;
            if (bb < 0) bb = 0; if (bb > 255) bb = 255;

            /* Gamma */
            out[0] = sGammaLut[rr];
            out[1] = sGammaLut[gg];
            out[2] = sGammaLut[bb];
        } else {
            out[0] = R;
            out[1] = G;
            out[2] = B;
        }
        out[3] = 255;
        out += 4;

        /* Accumulate for next frame AWB (sample every 4th) */
        if (doIsp && (x & 3) == 0) { sR += R; sG += G; sB += B; }
    }
    #undef PX

    *accR += sR; *accG += sG; *accB += sB;
}

/*
 * Stock ISP color correction matrices from Xiaomi/NVIDIA calibration profiles.
 * Q10 fixed-point (1024 = 1.0x). Row-major: {R_from_R, R_from_G, R_from_B, ...}
 */
static const int16_t ccm_imx179[9] = {  /* imx179_primax_v2.27.isp, srgbMatrix (D50) */
     1930,  -844,   -61,   /* R' =  1.884*R - 0.824*G - 0.059*B */
     -268,  1654,  -362,   /* G' = -0.262*R + 1.615*G - 0.354*B */
       52,  -822,  1793,   /* B' =  0.051*R - 0.803*G + 1.751*B */
};
static const int16_t ccm_ov5693[9] = {  /* ov5693_sunny_v2.13.isp */
     1912,  -861,   -27,   /* R' =  1.867*R - 0.841*G - 0.026*B */
     -268,  1578,  -286,   /* G' = -0.262*R + 1.541*G - 0.279*B */
      -15,  -663,  1702,   /* B' = -0.015*R - 0.647*G + 1.662*B */
};

/* Persistent AWB gains — updated per frame, applied to next frame */
static unsigned sPrevWbR = 256, sPrevWbG = 256, sPrevWbB = 256;

uint8_t *ImageConverter::BayerToRGBA(const uint8_t *src, uint8_t *dst,
        unsigned width, unsigned height, uint32_t pixFmt, bool softIsp) {
    assert(gWorkers.isRunning());
    assert(src != NULL);
    assert(dst != NULL);

    initGammaLut();

    /* Select CCM */
    const int16_t *ccm = ccm_imx179;
    if (pixFmt == V4L2_PIX_FMT_SBGGR10 || pixFmt == V4L2_PIX_FMT_SBGGR8)
        ccm = ccm_ov5693;

    /* Single pass: demosaic + AWB + CCM + gamma + accumulate for next AWB */
    Workers::Task::Function taskFn = [](void *data) {
        ConvertTask::Data *d = static_cast<ConvertTask::Data *>(data);
        uint64_t sR = 0, sG = 0, sB = 0;
        for (size_t i = 0; i < d->linesNum; ++i) {
            demosaicIspLine(d->src, d->dst, d->width, d->height,
                d->startLine + i, d->pixFmt,
                d->ccm != NULL, d->ccm, d->wbR, d->wbG, d->wbB,
                &sR, &sG, &sB);
        }
        d->sumR = sR; d->sumG = sG; d->sumB = sB;
    };

    const size_t numTasks = 8;  /* match core count better than 30 */
    ConvertTask tasks[8];
    const size_t linesPerTask = (height + numTasks - 1) / numTasks;

    for (size_t i = 0; i < numTasks; ++i) {
        tasks[i].data.src       = src;
        tasks[i].data.dst       = dst;
        tasks[i].data.width     = width;
        tasks[i].data.height    = height;
        tasks[i].data.startLine = i * linesPerTask;
        tasks[i].data.linesNum  = linesPerTask;
        tasks[i].data.pixFmt    = pixFmt;
        tasks[i].data.sumR      = 0;
        tasks[i].data.sumG      = 0;
        tasks[i].data.sumB      = 0;
        tasks[i].data.wbR       = softIsp ? sPrevWbR : 256;
        tasks[i].data.wbG       = softIsp ? sPrevWbG : 256;
        tasks[i].data.wbB       = softIsp ? sPrevWbB : 256;
        tasks[i].data.ccm       = softIsp ? ccm : NULL;
        if ((i + 1) * linesPerTask >= height)
            tasks[i].data.linesNum = height - i * linesPerTask;

        tasks[i].task = Workers::Task(taskFn, (void *)&tasks[i].data);
        gWorkers.queueTask(&tasks[i].task);
    }

    uint64_t totalR = 0, totalG = 0, totalB = 0;
    for (size_t i = 0; i < numTasks; ++i) {
        tasks[i].task.waitForCompletion();
        totalR += tasks[i].data.sumR;
        totalG += tasks[i].data.sumG;
        totalB += tasks[i].data.sumB;
    }

    /* Update AWB gains for next frame */
    if (softIsp && totalR && totalG && totalB) {
        uint64_t avg = (totalR + totalG + totalB) / 3;
        unsigned wbR = (unsigned)((avg * 256ULL) / totalR);
        unsigned wbG = (unsigned)((avg * 256ULL) / totalG);
        unsigned wbB = (unsigned)((avg * 256ULL) / totalB);
        if (wbR < 128) wbR = 128; if (wbR > 1024) wbR = 1024;
        if (wbG < 128) wbG = 128; if (wbG > 1024) wbG = 1024;
        if (wbB < 128) wbB = 128; if (wbB > 1024) wbB = 1024;
        sPrevWbR = wbR; sPrevWbG = wbG; sPrevWbB = wbB;
    }

    return dst + width * height * 4;
}

}; /* namespace android */
