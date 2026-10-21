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

#ifndef CORE_SESSION_CAPTURE_SIZE_H
#define CORE_SESSION_CAPTURE_SIZE_H

#include <optional>
#include <span>

#include "mochacamera/Buffers.h"
#include "mochacamera/Capability.h"
#include "mochacamera/Controls.h"

namespace mocha::camera {

/* The sensor mode to capture at for a set of output streams, or none
 * when no mode can feed them.
 *
 * The modes are the sizes the capability lists. RGBA and JPEG outputs
 * scale, so a mode feeds them when it is at least as wide and as tall
 * as each. A YUV output is the capture itself
 * (Capability::yuvAtCaptureSizeOnly): it names the mode, and every
 * YUV output must be that one size.
 *
 * Without a YUV output, the mode already `running` is kept when it
 * feeds the outputs -- changing modes restarts the sensor and the
 * pipeline, which the stream sets apps switch between (preview, video,
 * a small callback) should not cost. Otherwise the smallest mode that
 * feeds them: the demosaic runs at the capture size. */
std::optional<Size> chooseCaptureSize(const Capability &cap,
                                      std::span<const StreamSpec> outputs,
                                      std::optional<Size> running);

} /* namespace mocha::camera */

#endif /* CORE_SESSION_CAPTURE_SIZE_H */
