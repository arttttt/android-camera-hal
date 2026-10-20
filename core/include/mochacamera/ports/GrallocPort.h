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

#ifndef MOCHACAMERA_PORTS_GRALLOC_PORT_H
#define MOCHACAMERA_PORTS_GRALLOC_PORT_H

#include <cstddef>
#include <cstdint>

#include "mochacamera/Buffers.h"

namespace mocha::camera {

/* CPU access to output buffers: the one place the core reaches the
 * frontend's buffer allocator. The GPU path imports buffers inside the
 * ISP backend instead, which knows NativeBuffer for what it is.
 *
 * Any thread; a buffer locked is unlocked by the same caller. */
class GrallocPort {
public:
    struct Ycbcr {
        uint8_t *y        = nullptr;
        uint8_t *cb       = nullptr;
        uint8_t *cr       = nullptr;
        size_t   yStride  = 0;
        size_t   cStride  = 0;
        size_t   chromaStep = 0;
    };

    virtual ~GrallocPort() = default;

    /* A linear mapping for CPU writes (a BLOB). */
    virtual bool lock(NativeBuffer *buffer, const StreamSpec &stream,
                      uint8_t **data) = 0;

    /* A planar mapping for CPU writes (a YUV output). */
    virtual bool lockYcbcr(NativeBuffer *buffer, const StreamSpec &stream,
                           Ycbcr *planes) = 0;

    virtual void unlock(NativeBuffer *buffer) = 0;
};

} /* namespace mocha::camera */

#endif /* MOCHACAMERA_PORTS_GRALLOC_PORT_H */
