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
 * Simple bilinear Bayer demosaic.
 * Supports RGGB, BGGR, GRBG, GBRG patterns (8-bit).
 */
static inline void demosaicLine(const uint8_t *src, uint8_t *dst,
        unsigned width, unsigned height, unsigned y, uint32_t pixFmt) {
    /*
     * Determine color offsets for this Bayer pattern.
     * For RGGB: even rows = R G R G, odd rows = G B G B
     * rX/rY = red pixel offset within 2x2 block
     */
    int rX = 0, rY = 0; /* red position in 2x2 tile */
    switch (pixFmt) {
        case V4L2_PIX_FMT_SRGGB8: rX = 0; rY = 0; break; /* RGGB */
        case V4L2_PIX_FMT_SGRBG8: rX = 1; rY = 0; break; /* GRBG */
        case V4L2_PIX_FMT_SGBRG8: rX = 0; rY = 1; break; /* GBRG */
        case V4L2_PIX_FMT_SBGGR8: rX = 1; rY = 1; break; /* BGGR */
        default:                   rX = 0; rY = 0; break;
    }

    const uint8_t *row  = src + y * width;
    const uint8_t *rowU = (y > 0)          ? row - width : row;
    const uint8_t *rowD = (y < height - 1) ? row + width : row;
    uint8_t *out = dst + y * width * 4;

    for (unsigned x = 0; x < width; x++) {
        uint8_t r, g, b;
        int px = x & 1;
        int py = y & 1;

        uint8_t left  = (x > 0)         ? row[x - 1] : row[x];
        uint8_t right = (x < width - 1) ? row[x + 1] : row[x];
        uint8_t up    = rowU[x];
        uint8_t down  = rowD[x];

        if (py == rY && px == rX) {
            /* Red pixel */
            r = row[x];
            g = ((unsigned)left + right + up + down) / 4;
            uint8_t uleft  = (x > 0)         ? rowU[x - 1] : rowU[x];
            uint8_t uright = (x < width - 1) ? rowU[x + 1] : rowU[x];
            uint8_t dleft  = (x > 0)         ? rowD[x - 1] : rowD[x];
            uint8_t dright = (x < width - 1) ? rowD[x + 1] : rowD[x];
            b = ((unsigned)uleft + uright + dleft + dright) / 4;
        } else if (py != rY && px != rX) {
            /* Blue pixel */
            b = row[x];
            g = ((unsigned)left + right + up + down) / 4;
            uint8_t uleft  = (x > 0)         ? rowU[x - 1] : rowU[x];
            uint8_t uright = (x < width - 1) ? rowU[x + 1] : rowU[x];
            uint8_t dleft  = (x > 0)         ? rowD[x - 1] : rowD[x];
            uint8_t dright = (x < width - 1) ? rowD[x + 1] : rowD[x];
            r = ((unsigned)uleft + uright + dleft + dright) / 4;
        } else {
            /* Green pixel */
            g = row[x];
            if (py == rY) {
                /* Green on red row */
                r = ((unsigned)left + right) / 2;
                b = ((unsigned)up + down) / 2;
            } else {
                /* Green on blue row */
                b = ((unsigned)left + right) / 2;
                r = ((unsigned)up + down) / 2;
            }
        }

        out[0] = r;
        out[1] = g;
        out[2] = b;
        out[3] = 255;
        out += 4;
    }
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
