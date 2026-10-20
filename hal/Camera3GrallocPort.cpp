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

#include "Camera3GrallocPort.h"

#include <hardware/gralloc.h>
#include <ui/GraphicBufferMapper.h>
#include <ui/Rect.h>

namespace android {

namespace {

buffer_handle_t handleOf(mocha::camera::NativeBuffer *buffer) {
    return *reinterpret_cast<buffer_handle_t *>(buffer);
}

} /* namespace */

bool Camera3GrallocPort::lock(mocha::camera::NativeBuffer *buffer,
                              const mocha::camera::StreamSpec &stream,
                              uint8_t **data) {
    const Rect rect((int)stream.width, (int)stream.height);
    return GraphicBufferMapper::get().lock(handleOf(buffer),
            GRALLOC_USAGE_SW_WRITE_OFTEN, rect, (void **)data) == NO_ERROR;
}

bool Camera3GrallocPort::lockYcbcr(mocha::camera::NativeBuffer *buffer,
                                   const mocha::camera::StreamSpec &stream,
                                   Ycbcr *planes) {
    android_ycbcr dst = {};
    const Rect rect((int)stream.width, (int)stream.height);
    if (GraphicBufferMapper::get().lockYCbCr(handleOf(buffer),
            GRALLOC_USAGE_SW_WRITE_OFTEN, rect, &dst) != NO_ERROR)
        return false;
    planes->y          = (uint8_t *)dst.y;
    planes->cb         = (uint8_t *)dst.cb;
    planes->cr         = (uint8_t *)dst.cr;
    planes->yStride    = dst.ystride;
    planes->cStride    = dst.cstride;
    planes->chromaStep = dst.chroma_step;
    return true;
}

void Camera3GrallocPort::unlock(mocha::camera::NativeBuffer *buffer) {
    GraphicBufferMapper::get().unlock(handleOf(buffer));
}

} /* namespace android */
