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
extern "C" {
#include <jpeglib.h>
}

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

uint8_t * ImageConverter::RGBAToJPEG(const uint8_t *rgba, uint8_t *dst,
        unsigned width, unsigned height, size_t dstLen, uint8_t quality) {
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    /* Output to memory buffer */
    unsigned long outSize = dstLen;
    unsigned char *outBuf = dst;
    jpeg_mem_dest(&cinfo, &outBuf, &outSize);

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);

    jpeg_start_compress(&cinfo, TRUE);

    /* Convert RGBA → RGB row by row */
    uint8_t *row = new uint8_t[width * 3];
    while (cinfo.next_scanline < cinfo.image_height) {
        const uint8_t *src = rgba + cinfo.next_scanline * width * 4;
        for (unsigned x = 0; x < width; x++) {
            row[x * 3 + 0] = src[x * 4 + 0];
            row[x * 3 + 1] = src[x * 4 + 1];
            row[x * 3 + 2] = src[x * 4 + 2];
        }
        JSAMPROW rowPtr = row;
        jpeg_write_scanlines(&cinfo, &rowPtr, 1);
    }
    delete[] row;

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    /* jpeg_mem_dest may realloc — check if it stayed in our buffer */
    if (outBuf != dst) {
        if (outSize <= dstLen) {
            memcpy(dst, outBuf, outSize);
        } else {
            ALOGE("JPEG output %lu > buffer %zu", outSize, dstLen);
            free(outBuf);
            return dst; /* failure */
        }
        free(outBuf);
    }

    return dst + outSize;
}

}; /* namespace android */
