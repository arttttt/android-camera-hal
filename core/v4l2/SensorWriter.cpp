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

#undef  LOG_TAG
#define LOG_TAG "Cam-SensorWriter"

#include "SensorWriter.h"

#include <log/log.h>

#include <linux/videodev2.h>

#include "sensor/SensorConfig.h"
#include "V4l2Controls.h"
#include "V4l2Device.h"
#include "ViController.h"

namespace android {

namespace {

/* Brackets a set of sensor writes with the driver's group hold, so the
 * sensor latches them on the same frame. Without it exposure and gain
 * of one decision can land a frame apart. The driver enters the hold
 * on the first register write after GROUP_HOLD=1 and releases it,
 * committing all held registers at once, on GROUP_HOLD=0. */
class GroupHold {
public:
    GroupHold(V4l2Device *d, bool on) : dev(d), active(on) {
        if (active) dev->setControl(V4L2_CID_GROUP_HOLD, 1);
    }
    ~GroupHold() {
        if (active) dev->setControl(V4L2_CID_GROUP_HOLD, 0);
    }
private:
    V4l2Device *dev;
    bool        active;
};

} /* namespace */

SensorWriter::SensorWriter(V4l2Device *d, const SensorConfig *c,
                           ViController *v)
    : dev(d), cfg(c), vi(v), groupHold(false) {
    int32_t mn = 0, mx = 0, df = 0;
    groupHold = dev->queryControl(V4L2_CID_GROUP_HOLD, &mn, &mx, &df)
             && mx >= 1;
    ALOGI("sensor group hold %s", groupHold ? "supported" : "not supported");
}

void SensorWriter::frame(const DelayedControls::Batch &values, bool changed) {
    const int32_t frameLen   = values.val[DelayedControls::FRAME_LENGTH];
    const int32_t lineLength = values.val[DelayedControls::LINE_LENGTH];

    /* Before the write reaches the sensor, so a longer frame never
     * meets the old wait. */
    if (vi)
        vi->beforeFrame(cfg->frameDurationNs(frameLen, lineLength));
    if (!changed)
        return;

    GroupHold hold(dev, groupHold);
    /* Single VIDIOC_S_EXT_CTRLS — frame_length must reach the sensor
     * before / together with the exposure that needs it, otherwise
     * the driver clamps the exposure into the old (smaller) frame
     * and the user sees nothing change. */
    V4l2Controls ctrls;
    ctrls.add(V4L2_CID_FRAME_LENGTH, frameLen);
    /* The line before the exposure counted in it: the driver converts
     * µs to lines of the length in effect, and setControls sends the
     * camera class (frame and line length) before the user class
     * (exposure, gain). */
    if (cfg->lineLengthMax > 0)
        ctrls.add(V4L2_CID_LINE_LENGTH, lineLength);
    ctrls.add(V4L2_CID_EXPOSURE, values.val[DelayedControls::EXPOSURE]);
    ctrls.add(V4L2_CID_GAIN,     values.val[DelayedControls::GAIN]);
    dev->setControls(ctrls);
}

} /* namespace android */
