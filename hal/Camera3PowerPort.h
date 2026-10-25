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

#ifndef HAL_CAMERA3_POWER_PORT_H
#define HAL_CAMERA3_POWER_PORT_H

#include <memory>
#include <mutex>
#include <vector>

#include <aidl/android/hardware/power/IPower.h>

#include "PowerHintPolicy.h"
#include "mochacamera/ports/PowerPort.h"

namespace android {

/* The core's PowerPort as AIDL power hints: setBoost(CAMERA_LAUNCH /
 * CAMERA_SHOT) and setMode(CAMERA_STREAMING_LOW / MID / HIGH), what
 * PowerHintPolicy makes of each event. Both calls are oneway.
 *
 * The service is looked up without waiting, when a hint is to go and
 * none is held; with none registered the hint is dropped. A power HAL
 * that dies takes the level on with it; the next event looks up the
 * new one, and a level change reaches it whole. */
class Camera3PowerPort : public mocha::camera::PowerPort {
public:
    void opening() override;
    void streaming(uint64_t pixelsPerSecond) override;
    void stillCapture(uint64_t pixels) override;
    void stopped() override;

    /* The level on and whether the service is held, for dumpsys. */
    void dump(int fd);

private:
    using IPower = aidl::android::hardware::power::IPower;

    void send(const std::vector<PowerHintPolicy::Action> &actions);
    std::shared_ptr<IPower> service();

    std::mutex              mLock;
    PowerHintPolicy         mPolicy;
    std::shared_ptr<IPower> mPower;
    bool                    mMissingLogged = false;
};

} /* namespace android */

#endif /* HAL_CAMERA3_POWER_PORT_H */
