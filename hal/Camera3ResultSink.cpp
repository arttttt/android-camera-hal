/*
 * Copyright (C) 2026 Artem Bambalov
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

#define LOG_TAG "Cam-Camera3ResultSink"

#include "Camera3ResultSink.h"

#include <unistd.h>

#include <utils/Log.h>

#include "metadata/CameraStaticMetadata.h"
#include "metadata/ResultMetadataBuilder.h"

namespace android {

using mocha::camera::BufferReturn;
using mocha::camera::BufferStatus;
using mocha::camera::Core3AResult;
using mocha::camera::CoreFrameResult;
using mocha::camera::FrameOutcome;
using mocha::camera::PartialResult;
using mocha::camera::StreamId;

static_assert((int)BufferStatus::Ok    == CAMERA3_BUFFER_STATUS_OK,    "BufferStatus");
static_assert((int)BufferStatus::Error == CAMERA3_BUFFER_STATUS_ERROR, "BufferStatus");

Camera3ResultSink::Camera3ResultSink(const camera3_callback_ops_t *const *o)
    : opsPtr(o) {}

void Camera3ResultSink::setStreams(std::vector<camera3_stream_t *> s) {
    std::lock_guard<std::mutex> lock(mutex);
    streams = std::move(s);
}

void Camera3ResultSink::expect(uint32_t frameNumber, const CameraMetadata &s) {
    std::lock_guard<std::mutex> lock(mutex);
    settings[frameNumber] = s;
}

void Camera3ResultSink::forget(uint32_t frameNumber) {
    std::lock_guard<std::mutex> lock(mutex);
    settings.erase(frameNumber);
}

camera3_stream_t *Camera3ResultSink::stream(StreamId id) const {
    std::lock_guard<std::mutex> lock(mutex);
    return id < streams.size() ? streams[id] : nullptr;
}

void Camera3ResultSink::notifyError(uint32_t frameNumber, camera3_stream_t *s,
                                    int code) {
    const camera3_callback_ops_t *o = ops();
    if (!o) return;
    camera3_notify_msg_t msg;
    msg.type = CAMERA3_MSG_ERROR;
    msg.message.error.frame_number = frameNumber;
    msg.message.error.error_stream = s;
    msg.message.error.error_code   = code;
    o->notify(o, &msg);
}

void Camera3ResultSink::emit(uint32_t frameNumber, uint32_t partialCount,
                             const camera_metadata_t *metadata,
                             const std::vector<camera3_stream_buffer_t> &buffers) {
    const camera3_callback_ops_t *o = ops();
    if (!o) {
        for (const camera3_stream_buffer_t &b : buffers)
            if (b.release_fence >= 0) ::close(b.release_fence);
        return;
    }
    camera3_capture_result cr;
    cr.frame_number       = frameNumber;
    cr.result             = metadata;
    cr.num_output_buffers = buffers.size();
    cr.output_buffers     = buffers.empty() ? nullptr : buffers.data();
    cr.input_buffer       = nullptr;
    cr.partial_result     = partialCount;
    o->process_capture_result(o, &cr);
}

void Camera3ResultSink::shutter(uint32_t frameNumber, int64_t timestampNs) {
    const camera3_callback_ops_t *o = ops();
    if (!o) return;
    camera3_notify_msg_t msg;
    msg.type = CAMERA3_MSG_SHUTTER;
    msg.message.shutter.frame_number = frameNumber;
    msg.message.shutter.timestamp    = (uint64_t)timestampNs;
    o->notify(o, &msg);
}

void Camera3ResultSink::partial(uint32_t frameNumber, PartialResult which,
                                const Core3AResult &result) {
    CameraMetadata md;
    {
        std::lock_guard<std::mutex> lock(mutex);
        auto it = settings.find(frameNumber);
        if (it == settings.end()) return;
        md = it->second;
    }
    switch (which) {
        case PartialResult::Awb: ResultMetadataBuilder::buildAwb(md, result); break;
        case PartialResult::Ae:  ResultMetadataBuilder::buildAe(md, result);  break;
    }
    const camera_metadata_t *blob = md.getAndLock();
    emit(frameNumber, (uint32_t)which, blob, {});
    md.unlock(blob);
}

void Camera3ResultSink::frame(uint32_t frameNumber, const CoreFrameResult &result,
                              std::span<const BufferReturn> buffers) {
    CameraMetadata md;
    {
        std::lock_guard<std::mutex> lock(mutex);
        auto it = settings.find(frameNumber);
        if (it != settings.end()) {
            md = std::move(it->second);
            settings.erase(it);
        }
    }

    std::vector<camera3_stream_buffer_t> out;
    out.reserve(buffers.size());
    for (const BufferReturn &b : buffers) {
        camera3_stream_buffer_t sb;
        sb.stream        = stream(b.stream);
        sb.buffer        = reinterpret_cast<buffer_handle_t *>(b.buffer);
        sb.status        = (int)b.status;
        sb.acquire_fence = -1;
        sb.release_fence = b.releaseFence;
        out.push_back(sb);
    }

    if (result.outcome == FrameOutcome::Failed) {
        /* One complete failure to Camera3. After ERROR_REQUEST only
         * buffers in error may follow: no metadata, partial_result 0. */
        notifyError(frameNumber, nullptr, CAMERA3_MSG_ERROR_REQUEST);
        if (!out.empty()) emit(frameNumber, 0, nullptr, out);
        return;
    }

    /* The frame went through but an output did not: camera3.h asks for
     * ERROR_BUFFER on its stream before the buffer is returned. */
    for (const camera3_stream_buffer_t &sb : out)
        if (sb.status != CAMERA3_BUFFER_STATUS_OK)
            notifyError(frameNumber, sb.stream, CAMERA3_MSG_ERROR_BUFFER);

    ResultMetadataBuilder::buildFinal(md, result.a3, result.timestampNs,
                                      frameNumber, focalLengthMm);
    const camera_metadata_t *blob = md.getAndLock();
    emit(frameNumber, kPartialResultCount, blob, out);
    md.unlock(blob);
}

void Camera3ResultSink::deviceError(const char *why) {
    ALOGE("reporting CAMERA3_MSG_ERROR_DEVICE: %s", why);
    notifyError(0, nullptr, CAMERA3_MSG_ERROR_DEVICE);
}

} /* namespace android */
