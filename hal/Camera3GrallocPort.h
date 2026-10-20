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

#ifndef HAL_CAMERA3_GRALLOC_PORT_H
#define HAL_CAMERA3_GRALLOC_PORT_H

#include "mochacamera/ports/GrallocPort.h"

namespace android {

/* CPU access to Camera3 output buffers through GraphicBufferMapper.
 * The core's NativeBuffer is the request's buffer_handle_t*. */
class Camera3GrallocPort : public mocha::camera::GrallocPort {
public:
    bool lock(mocha::camera::NativeBuffer *buffer,
              const mocha::camera::StreamSpec &stream, uint8_t **data) override;
    bool lockYcbcr(mocha::camera::NativeBuffer *buffer,
                   const mocha::camera::StreamSpec &stream, Ycbcr *planes) override;
    void unlock(mocha::camera::NativeBuffer *buffer) override;
};

} /* namespace android */

#endif /* HAL_CAMERA3_GRALLOC_PORT_H */
