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

#ifndef THREE_A_REGION_PATCHES_H
#define THREE_A_REGION_PATCHES_H

#include <optional>

#include "ipa/IpaStats.h"
#include "mochacamera/Controls.h"

namespace android {

class FrameMap;

/* The patches of the stats grid a metering region covers on a mode's
 * frame: the region through the frame map, floored and ceiled to whole
 * patches, grown to `minSpan` patches a side around its centre. None
 * for a region that takes no part -- weight 0, one spanning what the
 * frame shows (apps send that for "no region"), one outside it -- or
 * under a map that knows no window; the caller meters its default
 * then. AF and AE map their regions through this one path. */
std::optional<PatchRoi> regionPatches(const FrameMap &map,
                                      const mocha::camera::MeteringRegion &region,
                                      int minSpan);

} /* namespace android */

#endif /* THREE_A_REGION_PATCHES_H */
