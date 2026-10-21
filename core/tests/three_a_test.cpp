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

/* Host test for 3A on synthetic statistics: the states describe3A
 * derives, the manual exposure path, AE closing its loop on a linear
 * sensor model (and EV compensation moving it), gray-world AWB finding
 * a scene's neutral, and AF's Camera2 state machine sweeping to a
 * focus peak. Loads the repository's IMX179 tuning. Run by
 * core/tests/run_host_tests.sh from the repository root. */

#include <math.h>

#include <cstdio>
#include <cstring>

#include "3a/AutoExposureController.h"
#include "3a/AutoFocusController.h"
#include "3a/AwbConvergence.h"
#include "3a/GrayWorldAwbController.h"
#include "3a/ThreeAReport.h"
#include "ipa/IpaFrameMeta.h"
#include "ipa/IpaStats.h"
#include "sensor/DelayedControls.h"
#include "sensor/SensorConfig.h"
#include "sensor/SensorTuning.h"
#include "Check.h"

#include <utils/Timers.h>

using namespace android;
using mocha::camera::AeMode;
using mocha::camera::AeState;
using mocha::camera::AfMode;
using mocha::camera::AfState;
using mocha::camera::AfTrigger;
using mocha::camera::AwbMode;
using mocha::camera::AwbState;
using mocha::camera::FrameSettings;
using mocha::camera::MeteringRegion;
using mocha::camera::Rational;

using mocha::camera::test::check;
using mocha::camera::test::finish;

namespace {

constexpr Rational kThirdStop{1, 3};

/* The IMX179 as the driver reports it on the device. */
SensorConfig imx179() {
    SensorConfig c = SensorConfig::imx179();
    c.frameLenDefault = 2564;
    c.frameLenMax     = 0xFFFF;
    c.lineTimeNs      = 13271;   /* HTS 3440 at 259.2 MHz */
    c.lineLengthDefault = 3440;
    c.lineLengthMax   = 0x7FFF;
    c.gainMin         = 256;
    c.gainDefault     = 256;
    c.gainMax         = 4096;
    c.exposureMin     = 13;
    c.exposureDefault = 16000;
    c.exposureMax     = 99000;
    c.focalLengthMm   = 3.5f;
    return c;
}

IpaStats uniformStats(float r, float g, float b, float focus = 0.f) {
    IpaStats s;
    std::memset(&s, 0, sizeof(s));
    for (int py = 0; py < IpaStats::PATCH_Y; ++py) {
        for (int px = 0; px < IpaStats::PATCH_X; ++px) {
            s.rgbMean[py][px][0] = r;
            s.rgbMean[py][px][1] = g;
            s.rgbMean[py][px][2] = b;
            s.focusMetric[py][px] = focus;
        }
    }
    return s;
}

/* AE and AWB states as Camera2's tables have them: from the frame's own
 * mode and lock and the controllers' convergence -- Locked only when the
 * request locks it, an AF sweep showing no state of its own, Inactive
 * in any mode but Auto; the exposure in effect is what was written a
 * delay before, or the defaults. */
void testDescribe3A() {
    const SensorConfig cfg = imx179();
    DelayedControls::Config dcc{};
    dcc.delay[0] = dcc.delay[1] = dcc.delay[2] = dcc.delay[3] = 2;
    DelayedControls dc(dcc);

    AutoFocusController::Report af{AfState::Inactive, AfMode::Off, 0.f};
    FrameSettings s;

    auto r = describe3A(s, af, {false, false}, 10, &dc, cfg);
    check(r.aeState == AeState::Searching);
    check(r.awbState == AwbState::Searching);
    check(r.exposureTimeNs == (int64_t)cfg.exposureDefault * 1000);
    check(r.sensitivityIso == 100);
    check(r.frameDurationNs == 2564LL * 13271);

    r = describe3A(s, af, {true, true}, 10, &dc, cfg);
    check(r.aeState == AeState::Converged);
    check(r.awbState == AwbState::Converged);

    s.ae.lock = true;
    s.awb.lock = true;
    r = describe3A(s, af, {false, false}, 10, &dc, cfg);
    check(r.aeState == AeState::Locked);
    check(r.awbState == AwbState::Locked);

    s = FrameSettings();
    af.afState = AfState::PassiveScan;
    r = describe3A(s, af, {false, true}, 10, &dc, cfg);
    check(r.awbState == AwbState::Converged);
    r = describe3A(s, af, {false, false}, 10, &dc, cfg);
    check(r.awbState == AwbState::Searching);

    s.ae.mode  = AeMode::Off;
    s.awb.mode = AwbMode::Daylight;
    r = describe3A(s, af, {true, true}, 10, &dc, cfg);
    check(r.aeState == AeState::Inactive);
    check(r.awbState == AwbState::Inactive);

    DelayedControls::Batch written{};
    written.has[DelayedControls::EXPOSURE] = true;
    written.val[DelayedControls::EXPOSURE] = 20000;
    written.has[DelayedControls::GAIN]     = true;
    written.val[DelayedControls::GAIN]     = 512;
    written.has[DelayedControls::FRAME_LENGTH] = true;
    written.val[DelayedControls::FRAME_LENGTH] = 4000;
    dc.commit(8, written);
    r = describe3A(FrameSettings(), af, {false, false}, 10, &dc, cfg);
    check(r.exposureTimeNs == 20000000LL);
    check(r.sensitivityIso == 200);
    check(r.frameDurationNs == 4000LL * 13271);
}

/* Manual AE: the requested exposure verbatim, the frame grown to fit a
 * long one; EV compensation scales it by 2^(steps x step). */
void testManualExposure() {
    const SensorConfig cfg = imx179();
    FrameSettings s;
    s.ae.mode = AeMode::Off;
    s.sensor.exposureTimeNs = 60000000;   /* 60 ms: past the default frame */
    s.sensor.sensitivityIso = 400;
    ExposureWriteValues v = AutoExposureController::parseManualSettings(s, cfg, kThirdStop);
    check(v.exposureUs == 60000);
    /* 60 ms is 4521.1 lines: 4522 to hold it whole, plus the margin. */
    check(v.frameLen == 4522 + cfg.maxCoarseDiff);
    check(v.gain == 4 * cfg.gainUnit);

    /* ISO between whole multiples is a fractional gain, not the
     * multiple below it. */
    s.sensor.sensitivityIso = 150;
    v = AutoExposureController::parseManualSettings(s, cfg, kThirdStop);
    check(v.gain == cfg.gainUnit * 3 / 2);
    check(cfg.gainToIso(cfg.isoToGain(1250)) == 1250);

    s.sensor.exposureTimeNs = 10000000;
    s.ae.evCompensationSteps = 3;          /* one stop */
    v = AutoExposureController::parseManualSettings(s, cfg, kThirdStop);
    check(v.exposureUs == 20000);
    check(v.frameLen == cfg.frameLenDefault);
}

/* The OV5693 as the driver reports it: 15-bit line counter, 1080p. */
SensorConfig ov5693() {
    SensorConfig c = SensorConfig::ov5693();
    c.frameLenDefault   = 1654;
    c.frameLenMax       = 0x7FFF;
    c.lineTimeNs        = 25200;   /* HTS 4032 at 160 MHz */
    c.lineLengthDefault = 4032;
    c.lineLengthMax     = 0x7FFF;
    c.gainMin = c.gainDefault = 256;
    c.gainMax         = 4096;
    c.exposureMin     = 51;
    c.exposureDefault = 41530;
    c.exposureMax     = 6700000;
    return c;
}

/* An exposure the line counter holds at the mode's line keeps that line
 * and grows the frame; a longer one takes the shortest line whose
 * longest frame holds it, and the frame is sized in those lines. */
void testLongExposureTiming() {
    const SensorConfig cfg = ov5693();
    const int64_t maxLines = cfg.frameLenMax - cfg.maxCoarseDiff;

    SensorConfig::FrameTiming t = cfg.timingForExposure(800000);
    check(t.lineLength == cfg.lineLengthDefault);
    check(t.frameLen == (int32_t)((800000000LL + 25199) / 25200) + cfg.maxCoarseDiff);

    for (int32_t us : {900000, 1600000, 5000000}) {
        t = cfg.timingForExposure(us);
        const int64_t ns = (int64_t)us * 1000;
        check(t.lineLength > cfg.lineLengthDefault);
        check(t.frameLen <= cfg.frameLenMax);
        /* The frame holds the whole exposure... */
        check((int64_t)(t.frameLen - cfg.maxCoarseDiff) * cfg.lineTimeNsAt(t.lineLength) >= ns);
        /* ...and a line one clock shorter could not. */
        check(maxLines * cfg.lineTimeNsAt(t.lineLength - 1) < ns);
    }

    /* The period follows the line: 1.6 s takes a ~1.6 s frame. */
    t = cfg.timingForExposure(1600000);
    const int64_t period = cfg.frameDurationNs(t.frameLen, t.lineLength);
    check(period >= 1600000000LL && period < 1610000000LL);

    /* Manual AE carries the line to the sensor write. */
    FrameSettings s;
    s.ae.mode = AeMode::Off;
    s.sensor.exposureTimeNs = 1600000000LL;
    const ExposureWriteValues v = AutoExposureController::parseManualSettings(s, cfg, kThirdStop);
    check(v.lineLength == t.lineLength && v.frameLen == t.frameLen);
}

/* AE on a linear sensor: luma proportional to exposure x gain. It
 * converges and holds; one stop of EV compensation doubles the
 * exposure it settles on. */
float settleAe(AutoExposureController &ae, const SensorConfig &cfg,
               int32_t evSteps, float sceneGain, bool *converged) {
    IpaFrameMeta meta;
    meta.aeExposureCompensation = evSteps;
    float totalUs = (float)cfg.exposureDefault;
    float lastTotal = 0.f;
    for (int frame = 0; frame < 400; ++frame) {
        float luma = sceneGain * totalUs;
        if (luma > 1.f) luma = 1.f;
        const IpaStats stats = uniformStats(luma, luma, luma);
        const AeResult r = ae.process(stats, meta, 1.f, 1.f, totalUs, 1);
        if (r.batch.has[DelayedControls::EXPOSURE] && r.batch.has[DelayedControls::GAIN]) {
            /* Auto AE keeps the mode's own frame and names it. */
            check(r.batch.has[DelayedControls::FRAME_LENGTH] &&
                  r.batch.val[DelayedControls::FRAME_LENGTH] == cfg.frameLenDefault);
            check(r.batch.has[DelayedControls::LINE_LENGTH] &&
                  r.batch.val[DelayedControls::LINE_LENGTH] == cfg.lineLengthDefault);
            check(r.batch.val[DelayedControls::EXPOSURE] <= cfg.maxExposureUsDefault());
            lastTotal = totalUs;
            totalUs = (float)r.batch.val[DelayedControls::EXPOSURE]
                    * (float)r.batch.val[DelayedControls::GAIN] / (float)cfg.gainUnit;
        }
    }
    *converged = ae.isConverged() && fabsf(totalUs - lastTotal) <= 0.02f * totalUs;
    return totalUs;
}

void testAeLoop() {
    SensorTuning tuning;
    check(tuning.loadFile("tuning/imx179_primax.json"));
    const SensorConfig cfg = imx179();

    AutoExposureController ae(cfg, &tuning, kThirdStop);
    bool converged = false;
    const float base = settleAe(ae, cfg, 0, 1.f / 80000.f, &converged);
    check(converged);

    AutoExposureController aePlus(cfg, &tuning, kThirdStop);
    const float plusOne = settleAe(aePlus, cfg, 3, 1.f / 80000.f, &converged);
    check(converged);
    const float ratio = plusOne / base;
    check(ratio > 1.7f && ratio < 2.3f);

    /* In the dark AE runs out of exposure inside the frame and goes on
     * in gain; the frame stays the mode's (checked in settleAe). */
    AutoExposureController aeDark(cfg, &tuning, kThirdStop);
    settleAe(aeDark, cfg, 0, 1.f / 80000000.f, &converged);
}

/* Gray-world finds the scene's neutral: gains move from the prior to
 * G/R and G/B of the patches. */
void testGrayWorld() {
    SensorTuning tuning;
    check(tuning.loadFile("tuning/imx179_primax.json"));
    const float prior[3] = {1.f, 1.f, 1.f};
    GrayWorldAwbController awb(&tuning, prior);
    const IpaStats stats = uniformStats(0.30f, 0.40f, 0.50f);
    check(!awb.isConverged());
    bool searchedFirst = false;
    for (int tick = 0; tick < 300; ++tick) {
        const AwbResult r = awb.process(stats, 0.f, AwbMode::Auto);
        if (tick == 0) searchedFirst = !r.converged;
    }
    check(searchedFirst);
    check(awb.isConverged());
    check(fabsf(awb.currentWbR() - 0.40f / 0.30f) < 0.03f);
    check(fabsf(awb.currentWbB() - 0.40f / 0.50f) < 0.03f);

    /* A new light: searching again, then converged on it. */
    const IpaStats warm = uniformStats(0.50f, 0.40f, 0.25f);
    check(!awb.process(warm, 0.f, AwbMode::Auto).converged);
    for (int tick = 0; tick < 300; ++tick)
        awb.process(warm, 0.f, AwbMode::Auto);
    check(awb.isConverged());

    awb.reset();
    check(!awb.isConverged());

    const AwbResult manual = awb.applyManualGains(2.f, 1.f, 1.5f);
    check(static_cast<bool>(manual.gains));
}

/* Convergence on the trend of the gap between the smoothed gains and
 * the target: a cold start far off searches, noise around a settled
 * point stays converged, a new light searches again within a tick or
 * two. */
void testAwbConvergence() {
    AwbConvergence c;
    check(!c.isConverged());
    check(!c.update(1.10f, 1.f, 1.f, 1.f));

    /* Gains closing on the target, as the damping moves them. */
    float wb = 1.10f;
    for (int tick = 0; tick < 60; ++tick) {
        wb += 0.1f * (1.f - wb);
        c.update(wb, 1.f, 1.f, 1.f);
    }
    check(c.isConverged());

    /* The Bayes estimate stepping between CT points: +-6 % about the
     * gains, tick to tick. */
    for (int tick = 0; tick < 100; ++tick)
        check(c.update(1.f, 1.f, tick % 2 ? 1.06f : 0.94f, 1.f));

    /* A new light: the gap stays one way. */
    c.update(1.f, 1.f, 1.f, 1.30f);
    check(!c.update(1.f, 1.f, 1.f, 1.30f));

    c.reset();
    check(!c.isConverged());
    check(!c.update(1.f, 1.f, 1.f, 1.03f));
}

/* AF_REGIONS: a tap moves the focus ROI to it; the same rectangle at
 * weight 0 -- how apps drop a tap -- is no region, the centre default. */
void testAfRegionWeight() {
    SensorTuning tuning;
    check(tuning.loadFile("tuning/imx179_primax.json"));
    AutoFocusController af(&tuning, 3280, 2464);

    auto isCentre = [](const AutoFocusController::FocusRoi &r) {
        return r.pyLo == IpaStats::FOCUS_ROI_PY_LO && r.pyHi == IpaStats::FOCUS_ROI_PY_HI &&
               r.pxLo == IpaStats::FOCUS_ROI_PX_LO && r.pxHi == IpaStats::FOCUS_ROI_PX_HI;
    };
    check(isCentre(af.currentFocusRoi()));

    FrameSettings s;
    s.af.mode = AfMode::Auto;
    MeteringRegion tap;
    tap.rect.xMin = 0;
    tap.rect.yMin = 0;
    tap.rect.xMax = 300;
    tap.rect.yMax = 200;
    tap.weight    = 1000;
    s.af.region = tap;
    af.onSettings(s, 0);
    AutoFocusController::FocusRoi roi = af.currentFocusRoi();
    check(!isCentre(roi));
    check(roi.pxLo == 0 && roi.pyLo == 0);

    tap.weight = 0;
    s.af.region = tap;
    af.onSettings(s, 1);
    check(isCentre(af.currentFocusRoi()));

    tap.weight = 1;
    s.af.region = tap;
    af.onSettings(s, 2);
    check(!isCentre(af.currentFocusRoi()));

    s.af.region.reset();
    af.onSettings(s, 3);
    check(isCentre(af.currentFocusRoi()));
}

/* AF: a triggered sweep in Auto runs as ActiveScan, holds AE / AWB for
 * its length, and locks focused near the sharpness peak; CANCEL
 * returns it to Inactive. */
void testAfSweep() {
    SensorTuning tuning;
    check(tuning.loadFile("tuning/imx179_primax.json"));
    AutoFocusController af(&tuning, 3280, 2464);

    const int peak = 400;
    int lens = -1;
    auto statsAt = [&](int pos) {
        const float d = (float)(pos - peak) / 60.f;
        return uniformStats(0.4f, 0.4f, 0.4f, 50.f / (1.f + d * d));
    };

    FrameSettings s;
    s.af.mode = AfMode::Auto;
    af.onSettings(s, 0);
    af.process(statsAt(0), true, systemTime() + 10000000000LL, true);
    check(af.report().afState == AfState::Inactive);

    s.af.trigger = AfTrigger::Start;
    af.onSettings(s, 1);
    s.af.trigger = AfTrigger::Idle;

    bool sawScan = false;
    bool sweepHeld = false;
    for (int tick = 0; tick < 400; ++tick) {
        const AfResult r = af.process(statsAt(lens < 0 ? 0 : lens), true,
                                      systemTime() + 10000000000LL, true);
        if (r.vcmPosition) lens = *r.vcmPosition;
        if (r.state == AfState::ActiveScan) {
            sawScan = true;
            sweepHeld = sweepHeld || r.sweeping;
        }
        const AfState st = af.report().afState;
        if (st == AfState::FocusedLocked || st == AfState::NotFocusedLocked)
            break;
    }
    check(sawScan);
    check(sweepHeld);
    check(af.report().afState == AfState::FocusedLocked);
    check(lens >= peak - 25 && lens <= peak + 25);

    s.af.trigger = AfTrigger::Cancel;
    af.onSettings(s, 2);
    af.process(statsAt(lens), true, systemTime() + 10000000000LL, true);
    check(af.report().afState == AfState::Inactive);
}

} // namespace

int main() {
    testDescribe3A();
    testManualExposure();
    testLongExposureTiming();
    testAeLoop();
    testAwbConvergence();
    testGrayWorld();
    testAfRegionWeight();
    testAfSweep();
    return finish("three_a_test");
}
