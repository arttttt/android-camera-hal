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

/* The camera provider: the stock legacy 2.4 service on /dev/binder.
 *
 * The stock one puts the process on /dev/vndbinder, for a camera HAL
 * talking to other vendor components; libbinder takes one driver per
 * process, and the power HAL's IPower is on /dev/binder, with
 * system_server and SurfaceFlinger. This HAL talks to nothing on
 * vndbinder -- gralloc and the provider itself are on hwbinder -- so
 * the process takes /dev/binder, as a client only: nothing is
 * registered there. */

#define LOG_TAG "android.hardware.camera.provider@2.4-service.mocha"

#include <android/hardware/camera/provider/2.4/ICameraProvider.h>
#include <binder/ProcessState.h>
#include <hidl/LegacySupport.h>
#include <log/log.h>

using android::hardware::defaultPassthroughServiceImplementation;
using android::hardware::camera::provider::V2_4::ICameraProvider;

int main() {
    ALOGI("CameraProvider@2.4 legacy service is starting, on /dev/binder");
    android::ProcessState::initWithDriver("/dev/binder");

    /* hwbinder's thread pool, as the stock service runs it. */
    return defaultPassthroughServiceImplementation<ICameraProvider>("legacy/0",
                                                                    /*maxThreads*/ 6);
}
