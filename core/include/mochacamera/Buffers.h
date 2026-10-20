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

#ifndef MOCHACAMERA_BUFFERS_H
#define MOCHACAMERA_BUFFERS_H

#include <cstdint>

namespace mocha::camera {

/* An output buffer as the frontend knows it: the core holds it and
 * hands it back, and only the ISP backend and the gralloc port look
 * inside. With Camera3 it is the request's buffer_handle_t* -- a
 * pointer to the handle -- passed through as is. */
struct NativeBuffer;

using StreamId = uint32_t;

/* What the core produces into a stream. A frontend maps its own
 * formats onto these when the streams are configured; a stream it
 * cannot map is refused there. */
enum class PixelFormat : uint8_t {
    Rgba8888,   /* preview, zero-copy from the GPU */
    Ycbcr420,   /* video and YUV readers */
    Blob,       /* JPEG */
};

struct StreamSpec {
    StreamId    id     = 0;
    PixelFormat format = PixelFormat::Rgba8888;
    uint32_t    width  = 0;
    uint32_t    height = 0;
};

/* How an output buffer goes back. */
enum class BufferStatus : uint8_t {
    Ok    = 0,
    Error = 1,
};

/* One output buffer going back with its frame's final result. */
struct BufferReturn {
    StreamId      stream       = 0;
    NativeBuffer *buffer       = nullptr;
    BufferStatus  status       = BufferStatus::Ok;
    /* Owned by the receiver from the call on; -1 when none. */
    int           releaseFence = -1;
};

} /* namespace mocha::camera */

#endif /* MOCHACAMERA_BUFFERS_H */
