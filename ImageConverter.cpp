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

#include "Yuv422UyvyToJpegEncoder.h"
#include "ImageConverter.h"
#include "DbgUtils.h"

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

/*
 * Simple bilinear Bayer demosaic.
 * Supports RGGB, BGGR, GRBG, GBRG patterns in 8-bit and 10-bit (16-bit LE).
 */
static inline void demosaicLine(const uint8_t *src, uint8_t *dst,
        unsigned width, unsigned height, unsigned y, uint32_t pixFmt) {
    int rX, rY;
    bayerPattern(pixFmt, &rX, &rY);
    bool is16 = bayerIs16bit(pixFmt);
    unsigned bpp = is16 ? 2 : 1;
    unsigned stride = width * bpp;

    const uint8_t *rowBytes  = src + y * stride;
    const uint8_t *rowBytesU = (y > 0)          ? rowBytes - stride : rowBytes;
    const uint8_t *rowBytesD = (y < height - 1) ? rowBytes + stride : rowBytes;
    uint8_t *out = dst + y * width * 4;

    /* Read one pixel, returning 8-bit value */
    #define PX(row, x) (is16 ? ((const uint16_t *)(row))[x] >> 2 : (row)[x])

    for (unsigned x = 0; x < width; x++) {
        unsigned c  = PX(rowBytes, x);
        unsigned l  = (x > 0)         ? PX(rowBytes, x - 1) : c;
        unsigned r  = (x < width - 1) ? PX(rowBytes, x + 1) : c;
        unsigned u  = PX(rowBytesU, x);
        unsigned d  = PX(rowBytesD, x);
        int px = x & 1;
        int py = y & 1;

        uint8_t R, G, B;
        if (py == rY && px == rX) {
            /* Red pixel */
            R = c;
            G = (l + r + u + d) / 4;
            unsigned ul = (x > 0)         ? PX(rowBytesU, x-1) : u;
            unsigned ur = (x < width - 1) ? PX(rowBytesU, x+1) : u;
            unsigned dl = (x > 0)         ? PX(rowBytesD, x-1) : d;
            unsigned dr = (x < width - 1) ? PX(rowBytesD, x+1) : d;
            B = (ul + ur + dl + dr) / 4;
        } else if (py != rY && px != rX) {
            /* Blue pixel */
            B = c;
            G = (l + r + u + d) / 4;
            unsigned ul = (x > 0)         ? PX(rowBytesU, x-1) : u;
            unsigned ur = (x < width - 1) ? PX(rowBytesU, x+1) : u;
            unsigned dl = (x > 0)         ? PX(rowBytesD, x-1) : d;
            unsigned dr = (x < width - 1) ? PX(rowBytesD, x+1) : d;
            R = (ul + ur + dl + dr) / 4;
        } else {
            /* Green pixel */
            G = c;
            if (py == rY) { R = (l + r) / 2; B = (u + d) / 2; }
            else           { B = (l + r) / 2; R = (u + d) / 2; }
        }

        out[0] = R;
        out[1] = G;
        out[2] = B;
        out[3] = 255;
        out += 4;
    }
    #undef PX
}

uint8_t *ImageConverter::BayerToRGBA(const uint8_t *src, uint8_t *dst,
        unsigned width, unsigned height, uint32_t pixFmt) {
    assert(gWorkers.isRunning());
    assert(src != NULL);
    assert(dst != NULL);

    Workers::Task::Function taskFn = [](void *data) {
        ConvertTask::Data *d = static_cast<ConvertTask::Data *>(data);
        for (size_t i = 0; i < d->linesNum; ++i) {
            demosaicLine(d->src, d->dst, d->width, d->height,
                         d->startLine + i, d->pixFmt);
        }
    };

    ConvertTask tasks[WORKERS_TASKS_NUM];
    const size_t linesPerTask = (height + WORKERS_TASKS_NUM - 1) / WORKERS_TASKS_NUM;

    for (size_t i = 0; i < WORKERS_TASKS_NUM; ++i) {
        tasks[i].data.src       = src;
        tasks[i].data.dst       = dst;
        tasks[i].data.width     = width;
        tasks[i].data.height    = height;
        tasks[i].data.startLine = i * linesPerTask;
        tasks[i].data.linesNum  = linesPerTask;
        tasks[i].data.pixFmt    = pixFmt;
        if ((i + 1) * linesPerTask >= height)
            tasks[i].data.linesNum = height - i * linesPerTask;

        tasks[i].task = Workers::Task(taskFn, (void *)&tasks[i].data);
        gWorkers.queueTask(&tasks[i].task);
    }

    for (size_t i = 0; i < WORKERS_TASKS_NUM; ++i)
        tasks[i].task.waitForCompletion();

    return dst + width * height * 4;
}

}; /* namespace android */
