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

#ifndef V4L2_FOCUSER_H
#define V4L2_FOCUSER_H

#include <stdint.h>

namespace android {

/* The lens VCM, through its V4L2 subdev. Found from the capture node by
 * the media graph; a camera without one has no focuser and AF stays
 * inactive. */
class Focuser {
public:
    Focuser() = default;
    ~Focuser() { close(); }

    Focuser(const Focuser &) = delete;
    Focuser &operator=(const Focuser &) = delete;

    /* Find and open the lens behind `videoNode`. Idempotent. */
    bool open(const char *videoNode);
    void close();
    bool isOpen() const { return fd >= 0; }

    /* V4L2_CID_FOCUS_ABSOLUTE, in the driver's units. */
    bool setPosition(int32_t position);

private:
    int fd = -1;
};

} /* namespace android */

#endif /* V4L2_FOCUSER_H */
