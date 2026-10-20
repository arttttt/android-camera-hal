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

#ifndef MOCHACAMERA_CAPABILITY_H
#define MOCHACAMERA_CAPABILITY_H

#include <cstdint>

#include "mochacamera/Controls.h"

namespace mocha::camera {

/* What the camera can do, as the core knows it: the one source the
 * frontend's static characteristics, its request templates and its
 * stream checks read, so what is advertised is what is produced. */
struct Capability {
    struct Ae {
        /* One EV compensation step, and the range in steps. */
        Rational evStep{1, 3};
        int32_t  evMinSteps = -9;
        int32_t  evMaxSteps = 9;
    } ae;
};

} /* namespace mocha::camera */

#endif /* MOCHACAMERA_CAPABILITY_H */
