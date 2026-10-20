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

#ifndef HAL_CAMERA3_REQUEST_H
#define HAL_CAMERA3_REQUEST_H

#include <hardware/camera3.h>

#include "CameraMetadataCompat.h"
#include "mochacamera/Buffers.h"
#include "mochacamera/FrameSettings.h"

namespace android {

/* Camera3 to the core: a request's metadata parsed into FrameSettings,
 * once, when the request is accepted; the core never reads metadata.
 *
 * A key the request leaves out gets the value Camera2 gives it (or no
 * value, where the absence means something of its own). Modes the HAL
 * does not implement are mapped onto ones it does: any auto-exposure
 * mode is AE On. */
mocha::camera::FrameSettings parseFrameSettings(const CameraMetadata &cm);

/* The core's format for a configured stream's HAL pixel format; false
 * for one the core does not produce. */
bool toPixelFormat(int halFormat, mocha::camera::PixelFormat *out);

} /* namespace android */

#endif /* HAL_CAMERA3_REQUEST_H */
