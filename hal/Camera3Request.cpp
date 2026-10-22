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

#include "Camera3Request.h"

#include <string.h>

#include <system/camera_metadata.h>
#include <system/graphics.h>

namespace android {

using namespace mocha::camera;

/* The core's 3A enums carry Camera2's values; each member is checked
 * here, where both are in sight, so a cast is the whole conversion. */
static_assert((uint8_t)AeMode::Off == ANDROID_CONTROL_AE_MODE_OFF, "AeMode");
static_assert((uint8_t)AeMode::On  == ANDROID_CONTROL_AE_MODE_ON,  "AeMode");

static_assert((uint8_t)AeState::Inactive  == ANDROID_CONTROL_AE_STATE_INACTIVE,  "AeState");
static_assert((uint8_t)AeState::Searching == ANDROID_CONTROL_AE_STATE_SEARCHING, "AeState");
static_assert((uint8_t)AeState::Converged == ANDROID_CONTROL_AE_STATE_CONVERGED, "AeState");
static_assert((uint8_t)AeState::Locked    == ANDROID_CONTROL_AE_STATE_LOCKED,    "AeState");

static_assert((uint8_t)AwbMode::Off             == ANDROID_CONTROL_AWB_MODE_OFF,              "AwbMode");
static_assert((uint8_t)AwbMode::Auto            == ANDROID_CONTROL_AWB_MODE_AUTO,             "AwbMode");
static_assert((uint8_t)AwbMode::Incandescent    == ANDROID_CONTROL_AWB_MODE_INCANDESCENT,     "AwbMode");
static_assert((uint8_t)AwbMode::Fluorescent     == ANDROID_CONTROL_AWB_MODE_FLUORESCENT,      "AwbMode");
static_assert((uint8_t)AwbMode::WarmFluorescent == ANDROID_CONTROL_AWB_MODE_WARM_FLUORESCENT, "AwbMode");
static_assert((uint8_t)AwbMode::Daylight        == ANDROID_CONTROL_AWB_MODE_DAYLIGHT,         "AwbMode");
static_assert((uint8_t)AwbMode::CloudyDaylight  == ANDROID_CONTROL_AWB_MODE_CLOUDY_DAYLIGHT,  "AwbMode");
static_assert((uint8_t)AwbMode::Twilight        == ANDROID_CONTROL_AWB_MODE_TWILIGHT,         "AwbMode");
static_assert((uint8_t)AwbMode::Shade           == ANDROID_CONTROL_AWB_MODE_SHADE,            "AwbMode");

static_assert((uint8_t)AwbState::Inactive  == ANDROID_CONTROL_AWB_STATE_INACTIVE,  "AwbState");
static_assert((uint8_t)AwbState::Searching == ANDROID_CONTROL_AWB_STATE_SEARCHING, "AwbState");
static_assert((uint8_t)AwbState::Converged == ANDROID_CONTROL_AWB_STATE_CONVERGED, "AwbState");
static_assert((uint8_t)AwbState::Locked    == ANDROID_CONTROL_AWB_STATE_LOCKED,    "AwbState");

static_assert((uint8_t)AfMode::Off               == ANDROID_CONTROL_AF_MODE_OFF,                "AfMode");
static_assert((uint8_t)AfMode::Auto              == ANDROID_CONTROL_AF_MODE_AUTO,               "AfMode");
static_assert((uint8_t)AfMode::Macro             == ANDROID_CONTROL_AF_MODE_MACRO,              "AfMode");
static_assert((uint8_t)AfMode::ContinuousVideo   == ANDROID_CONTROL_AF_MODE_CONTINUOUS_VIDEO,   "AfMode");
static_assert((uint8_t)AfMode::ContinuousPicture == ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE, "AfMode");
static_assert((uint8_t)AfMode::Edof              == ANDROID_CONTROL_AF_MODE_EDOF,               "AfMode");

static_assert((uint8_t)AfTrigger::Idle   == ANDROID_CONTROL_AF_TRIGGER_IDLE,   "AfTrigger");
static_assert((uint8_t)AfTrigger::Start  == ANDROID_CONTROL_AF_TRIGGER_START,  "AfTrigger");
static_assert((uint8_t)AfTrigger::Cancel == ANDROID_CONTROL_AF_TRIGGER_CANCEL, "AfTrigger");

static_assert((uint8_t)AfState::Inactive         == ANDROID_CONTROL_AF_STATE_INACTIVE,           "AfState");
static_assert((uint8_t)AfState::PassiveScan      == ANDROID_CONTROL_AF_STATE_PASSIVE_SCAN,       "AfState");
static_assert((uint8_t)AfState::PassiveFocused   == ANDROID_CONTROL_AF_STATE_PASSIVE_FOCUSED,    "AfState");
static_assert((uint8_t)AfState::ActiveScan       == ANDROID_CONTROL_AF_STATE_ACTIVE_SCAN,        "AfState");
static_assert((uint8_t)AfState::FocusedLocked    == ANDROID_CONTROL_AF_STATE_FOCUSED_LOCKED,     "AfState");
static_assert((uint8_t)AfState::NotFocusedLocked == ANDROID_CONTROL_AF_STATE_NOT_FOCUSED_LOCKED, "AfState");
static_assert((uint8_t)AfState::PassiveUnfocused == ANDROID_CONTROL_AF_STATE_PASSIVE_UNFOCUSED,  "AfState");

static_assert((uint8_t)CaptureIntent::Custom         == ANDROID_CONTROL_CAPTURE_INTENT_CUSTOM,           "CaptureIntent");
static_assert((uint8_t)CaptureIntent::Preview        == ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW,          "CaptureIntent");
static_assert((uint8_t)CaptureIntent::StillCapture   == ANDROID_CONTROL_CAPTURE_INTENT_STILL_CAPTURE,    "CaptureIntent");
static_assert((uint8_t)CaptureIntent::VideoRecord    == ANDROID_CONTROL_CAPTURE_INTENT_VIDEO_RECORD,     "CaptureIntent");
static_assert((uint8_t)CaptureIntent::VideoSnapshot  == ANDROID_CONTROL_CAPTURE_INTENT_VIDEO_SNAPSHOT,   "CaptureIntent");
static_assert((uint8_t)CaptureIntent::ZeroShutterLag == ANDROID_CONTROL_CAPTURE_INTENT_ZERO_SHUTTER_LAG, "CaptureIntent");
static_assert((uint8_t)CaptureIntent::Manual         == ANDROID_CONTROL_CAPTURE_INTENT_MANUAL,           "CaptureIntent");

namespace {

template <typename T>
bool readU8(const CameraMetadata &cm, uint32_t tag, T *out) {
    if (!cm.exists(tag)) return false;
    *out = (T)*cm.find(tag).data.u8;
    return true;
}

} /* namespace */

FrameSettings parseFrameSettings(const CameraMetadata &cm) {
    FrameSettings s;

    uint8_t aeMode = ANDROID_CONTROL_AE_MODE_ON;
    readU8(cm, ANDROID_CONTROL_AE_MODE, &aeMode);
    s.ae.mode = aeMode == ANDROID_CONTROL_AE_MODE_OFF ? AeMode::Off : AeMode::On;
    uint8_t aeLock = ANDROID_CONTROL_AE_LOCK_OFF;
    readU8(cm, ANDROID_CONTROL_AE_LOCK, &aeLock);
    s.ae.lock = aeLock == ANDROID_CONTROL_AE_LOCK_ON;
    if (cm.exists(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION))
        s.ae.evCompensationSteps =
            *cm.find(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION).data.i32;

    uint8_t awbMode = ANDROID_CONTROL_AWB_MODE_AUTO;
    readU8(cm, ANDROID_CONTROL_AWB_MODE, &awbMode);
    s.awb.mode = awbMode <= ANDROID_CONTROL_AWB_MODE_SHADE ? (AwbMode)awbMode
                                                           : AwbMode::Auto;
    uint8_t awbLock = ANDROID_CONTROL_AWB_LOCK_OFF;
    readU8(cm, ANDROID_CONTROL_AWB_LOCK, &awbLock);
    s.awb.lock = awbLock == ANDROID_CONTROL_AWB_LOCK_ON;
    /* Manual gains count for AWB Off only: in Auto the framework
     * round-trips the published gains into the next request, and taking
     * them would freeze AWB at its first publish. Camera2 orders them
     * (R, Geven, Godd, B); the two greens are calibrated equal. */
    if (s.awb.mode == AwbMode::Off && cm.exists(ANDROID_COLOR_CORRECTION_GAINS)) {
        const camera_metadata_ro_entry_t e = cm.find(ANDROID_COLOR_CORRECTION_GAINS);
        if (e.count == 4) {
            FrameSettings::Awb::Gains g;
            g.r = e.data.f[0];
            g.g = 0.5f * (e.data.f[1] + e.data.f[2]);
            g.b = e.data.f[3];
            s.awb.manualGains = g;
        }
    }

    uint8_t afMode = ANDROID_CONTROL_AF_MODE_OFF;
    readU8(cm, ANDROID_CONTROL_AF_MODE, &afMode);
    s.af.mode = afMode <= ANDROID_CONTROL_AF_MODE_EDOF ? (AfMode)afMode : AfMode::Off;
    uint8_t afTrigger = ANDROID_CONTROL_AF_TRIGGER_IDLE;
    readU8(cm, ANDROID_CONTROL_AF_TRIGGER, &afTrigger);
    s.af.trigger = afTrigger <= ANDROID_CONTROL_AF_TRIGGER_CANCEL ? (AfTrigger)afTrigger
                                                                 : AfTrigger::Idle;
    /* One region at most (MAX_REGIONS advertises one). */
    if (cm.exists(ANDROID_CONTROL_AF_REGIONS)) {
        const camera_metadata_ro_entry_t e = cm.find(ANDROID_CONTROL_AF_REGIONS);
        if (e.count >= 4) {
            MeteringRegion region;
            region.rect.xMin = e.data.i32[0];
            region.rect.yMin = e.data.i32[1];
            region.rect.xMax = e.data.i32[2];
            region.rect.yMax = e.data.i32[3];
            region.weight    = e.count >= 5 ? e.data.i32[4] : 0;
            s.af.region = region;
        }
    }
    if (s.af.mode == AfMode::Off && cm.exists(ANDROID_LENS_FOCUS_DISTANCE))
        s.af.focusDistanceDiopters = *cm.find(ANDROID_LENS_FOCUS_DISTANCE).data.f;

    if (cm.exists(ANDROID_SENSOR_EXPOSURE_TIME))
        s.sensor.exposureTimeNs = *cm.find(ANDROID_SENSOR_EXPOSURE_TIME).data.i64;
    if (cm.exists(ANDROID_SENSOR_SENSITIVITY))
        s.sensor.sensitivityIso = *cm.find(ANDROID_SENSOR_SENSITIVITY).data.i32;

    if (cm.exists(ANDROID_SCALER_CROP_REGION)) {
        const camera_metadata_ro_entry_t e = cm.find(ANDROID_SCALER_CROP_REGION);
        if (e.count >= 4) {
            CropRegion crop;
            crop.left   = e.data.i32[0];
            crop.top    = e.data.i32[1];
            crop.width  = e.data.i32[2];
            crop.height = e.data.i32[3];
            s.crop = crop;
        }
    }

    uint8_t intent = ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;
    readU8(cm, ANDROID_CONTROL_CAPTURE_INTENT, &intent);
    s.intent = intent <= ANDROID_CONTROL_CAPTURE_INTENT_MANUAL ? (CaptureIntent)intent
                                                               : CaptureIntent::Custom;

    if (cm.exists(ANDROID_JPEG_QUALITY))
        s.jpeg.quality = *cm.find(ANDROID_JPEG_QUALITY).data.u8;
    if (cm.exists(ANDROID_JPEG_ORIENTATION))
        s.jpeg.orientationDegrees = *cm.find(ANDROID_JPEG_ORIENTATION).data.i32;
    if (cm.exists(ANDROID_JPEG_THUMBNAIL_SIZE)) {
        const camera_metadata_ro_entry_t e = cm.find(ANDROID_JPEG_THUMBNAIL_SIZE);
        if (e.count >= 2 && e.data.i32[0] > 0 && e.data.i32[1] > 0) {
            s.jpeg.thumbnailWidth  = (uint32_t)e.data.i32[0];
            s.jpeg.thumbnailHeight = (uint32_t)e.data.i32[1];
        }
    }
    if (cm.exists(ANDROID_JPEG_THUMBNAIL_QUALITY))
        s.jpeg.thumbnailQuality = *cm.find(ANDROID_JPEG_THUMBNAIL_QUALITY).data.u8;
    if (cm.exists(ANDROID_JPEG_GPS_COORDINATES)) {
        const camera_metadata_ro_entry_t e = cm.find(ANDROID_JPEG_GPS_COORDINATES);
        if (e.count >= 3) {
            FrameSettings::Jpeg::Gps g;
            g.latitude  = e.data.d[0];
            g.longitude = e.data.d[1];
            g.altitude  = e.data.d[2];
            if (cm.exists(ANDROID_JPEG_GPS_TIMESTAMP))
                g.timestampSec = *cm.find(ANDROID_JPEG_GPS_TIMESTAMP).data.i64;
            if (cm.exists(ANDROID_JPEG_GPS_PROCESSING_METHOD)) {
                const camera_metadata_ro_entry_t m =
                        cm.find(ANDROID_JPEG_GPS_PROCESSING_METHOD);
                /* A byte string, NUL-terminated or not. */
                const char *c = reinterpret_cast<const char *>(m.data.u8);
                g.processingMethod.assign(c, strnlen(c, m.count));
            }
            s.jpeg.gps = std::move(g);
        }
    }

    s.stillCapture = s.intent == CaptureIntent::StillCapture ||
                     s.intent == CaptureIntent::VideoSnapshot ||
                     s.intent == CaptureIntent::ZeroShutterLag;
    return s;
}

bool toPixelFormat(int halFormat, PixelFormat *out) {
    switch (halFormat) {
        case HAL_PIXEL_FORMAT_RGBA_8888:     *out = PixelFormat::Rgba8888; return true;
        case HAL_PIXEL_FORMAT_YCbCr_420_888: *out = PixelFormat::Ycbcr420; return true;
        case HAL_PIXEL_FORMAT_BLOB:          *out = PixelFormat::Blob;     return true;
        default:                             return false;
    }
}

} /* namespace android */
