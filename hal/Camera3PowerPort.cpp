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

#define LOG_TAG "Cam-Power"

#include "Camera3PowerPort.h"

#include <cstdio>
#include <string>

#include <android/binder_manager.h>
#include <cutils/log.h>

namespace android {

using aidl::android::hardware::power::Boost;
using aidl::android::hardware::power::Mode;

namespace {

using Hint = PowerHintPolicy::Hint;

Mode modeOf(Hint h) {
    switch (h) {
    case Hint::StreamingLow:  return Mode::CAMERA_STREAMING_LOW;
    case Hint::StreamingMid:  return Mode::CAMERA_STREAMING_MID;
    default:                  return Mode::CAMERA_STREAMING_HIGH;
    }
}

const char *nameOf(Hint h) {
    switch (h) {
    case Hint::Launch:        return "CAMERA_LAUNCH";
    case Hint::Shot:          return "CAMERA_SHOT";
    case Hint::StreamingLow:  return "CAMERA_STREAMING_LOW";
    case Hint::StreamingMid:  return "CAMERA_STREAMING_MID";
    case Hint::StreamingHigh: return "CAMERA_STREAMING_HIGH";
    }
    return "?";
}

} /* namespace */

void Camera3PowerPort::opening() {
    std::lock_guard<std::mutex> lock(mLock);
    send(mPolicy.opening());
}

void Camera3PowerPort::streaming(uint64_t pixelsPerSecond) {
    std::lock_guard<std::mutex> lock(mLock);
    ALOGD("streaming %.1f Mpx/s", (double)pixelsPerSecond / 1e6);
    send(mPolicy.streaming(pixelsPerSecond));
}

void Camera3PowerPort::stillCapture(uint64_t pixels) {
    std::lock_guard<std::mutex> lock(mLock);
    send(mPolicy.stillCapture(pixels));
}

void Camera3PowerPort::stopped() {
    std::lock_guard<std::mutex> lock(mLock);
    send(mPolicy.stopped());
}

std::shared_ptr<Camera3PowerPort::IPower> Camera3PowerPort::service() {
    if (mPower && AIBinder_isAlive(mPower->asBinder().get()))
        return mPower;
    mPower.reset();

    /* checkService does not wait for a service that is not there. */
    const std::string name = std::string(IPower::descriptor) + "/default";
    ndk::SpAIBinder binder(AServiceManager_checkService(name.c_str()));
    if (binder.get() == nullptr) {
        if (!mMissingLogged)
            ALOGI("no %s: the camera sends no power hints", name.c_str());
        mMissingLogged = true;
        return nullptr;
    }
    mMissingLogged = false;
    mPower = IPower::fromBinder(binder);
    return mPower;
}

void Camera3PowerPort::send(const std::vector<PowerHintPolicy::Action> &actions) {
    if (actions.empty()) return;
    const std::shared_ptr<IPower> power = service();
    if (!power) return;

    using Kind = PowerHintPolicy::Action::Kind;
    for (const PowerHintPolicy::Action &a : actions) {
        ndk::ScopedAStatus status;
        switch (a.kind) {
        case Kind::Boost:
            status = power->setBoost(a.hint == Hint::Launch ? Boost::CAMERA_LAUNCH
                                                            : Boost::CAMERA_SHOT,
                                     a.durationMs);
            ALOGD("%s %d ms", nameOf(a.hint), a.durationMs);
            break;
        case Kind::ModeOn:
        case Kind::ModeOff:
            status = power->setMode(modeOf(a.hint), a.kind == Kind::ModeOn);
            ALOGI("%s %s", nameOf(a.hint), a.kind == Kind::ModeOn ? "on" : "off");
            break;
        }
        if (!status.isOk())
            ALOGW("%s: %s", nameOf(a.hint), status.getDescription().c_str());
    }
}

void Camera3PowerPort::dump(int fd) {
    std::lock_guard<std::mutex> lock(mLock);
    const std::optional<Hint> level = mPolicy.level();
    dprintf(fd, "  power: %s, level %s\n",
            mPower ? "IPower/default held" : "no power HAL held",
            level ? nameOf(*level) : "none");
}

} /* namespace android */
