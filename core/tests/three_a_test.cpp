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
#include <stdlib.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <optional>
#include <string>

#include "3a/AutoExposureController.h"
#include "3a/AutoFocusController.h"
#include "3a/AwbConvergence.h"
#include "3a/GrayWorldAwbController.h"
#include "3a/RequestExposure.h"
#include "3a/ThreeAReport.h"
#include "ipa/IpaFrameMeta.h"
#include "ipa/IpaStats.h"
#include "sensor/DelayedControls.h"
#include "sensor/ExposureTarget.h"
#include "sensor/SensorConfig.h"
#include "sensor/SensorGeometry.h"
#include "sensor/SensorTuning.h"
#include "Check.h"

#include <utils/Timers.h>

using namespace android;
using mocha::camera::AeMode;
using mocha::camera::Antibanding;
using mocha::camera::AeState;
using mocha::camera::AfMode;
using mocha::camera::AfState;
using mocha::camera::AfTrigger;
using mocha::camera::AwbMode;
using mocha::camera::AwbState;
using mocha::camera::FpsRange;
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
 * in any mode but Auto; the exposure in effect is the frame's record,
 * or the defaults for a control it has none of. */
void testDescribe3A() {
    const SensorConfig cfg = imx179();
    const DelayedControls::Batch none{};

    AutoFocusController::Report af{AfState::Inactive, AfMode::Off, 0.f};
    FrameSettings s;

    auto r = describe3A(s, af, {false, false}, none, cfg);
    check(r.aeState == AeState::Searching);
    check(r.awbState == AwbState::Searching);
    check(r.exposureTimeNs == (int64_t)cfg.exposureDefault * 1000);
    check(r.sensitivityIso == 100);
    check(r.frameDurationNs == 2564LL * 13271);

    r = describe3A(s, af, {true, true}, none, cfg);
    check(r.aeState == AeState::Converged);
    check(r.awbState == AwbState::Converged);

    s.ae.lock = true;
    s.awb.lock = true;
    r = describe3A(s, af, {false, false}, none, cfg);
    check(r.aeState == AeState::Locked);
    check(r.awbState == AwbState::Locked);

    s = FrameSettings();
    af.afState = AfState::PassiveScan;
    r = describe3A(s, af, {false, true}, none, cfg);
    check(r.awbState == AwbState::Converged);
    r = describe3A(s, af, {false, false}, none, cfg);
    check(r.awbState == AwbState::Searching);

    s.ae.mode  = AeMode::Off;
    s.awb.mode = AwbMode::Daylight;
    r = describe3A(s, af, {true, true}, none, cfg);
    check(r.aeState == AeState::Inactive);
    check(r.awbState == AwbState::Inactive);

    DelayedControls::Batch written{};
    written.has[DelayedControls::EXPOSURE] = true;
    written.val[DelayedControls::EXPOSURE] = 20000;
    written.has[DelayedControls::GAIN]     = true;
    written.val[DelayedControls::GAIN]     = 512;
    written.has[DelayedControls::FRAME_LENGTH] = true;
    written.val[DelayedControls::FRAME_LENGTH] = 4000;
    r = describe3A(FrameSettings(), af, {false, false}, written, cfg);
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

/* Past the analog maximum (x16, ISO 1600) the ISP's digital gain goes
 * on, to x4 (ISO 6400): an exposure is divided as libcamera's AGC
 * divides it -- the exposure up to the mode's frame, then the analog
 * gain to its maximum, then the digital -- a manual ISO above 1600
 * is the analog maximum and the rest digital, and what the frame ran
 * on is reported as the whole ISO, with no step at 1600. */
void testDigitalGain() {
    const SensorConfig cfg = imx179();
    const int32_t unit = SensorConfig::kDigitalGainUnit;
    const float maxUs = (float)cfg.maxExposureUsDefault();

    SensorConfig::ExposureSplit x = cfg.splitExposure(maxUs / 2);
    check(x.gain == cfg.gainUnit && x.digitalGain == unit);
    x = cfg.splitExposure(maxUs * 8);
    check(x.exposureUs == (int32_t)maxUs && x.gain == 8 * cfg.gainUnit && x.digitalGain == unit);
    x = cfg.splitExposure(maxUs * 16 * 3);
    check(x.gain == cfg.gainMax && x.digitalGain == 3 * unit);
    x = cfg.splitExposure(maxUs * 16 * 100);
    check(x.gain == cfg.gainMax && x.digitalGain == cfg.digitalGainMax);

    FrameSettings s;
    s.ae.mode = AeMode::Off;
    s.sensor.exposureTimeNs = 10000000;
    for (int32_t iso = 100; iso <= 6400; iso += 50) {
        s.sensor.sensitivityIso = iso;
        const ExposureWriteValues v =
            AutoExposureController::parseManualSettings(s, cfg, kThirdStop);
        check(v.gain <= cfg.gainMax);
        check(iso <= 1600 ? v.digitalGain == unit : v.gain == cfg.gainMax);
        /* The whole gain is the one asked for, across 1600. */
        const int32_t got = cfg.sensitivityIso(v.gain, v.digitalGain);
        check(got >= iso - iso / 100 && got <= iso + iso / 100);
    }
    s.sensor.sensitivityIso = 12800;
    ExposureWriteValues v = AutoExposureController::parseManualSettings(s, cfg, kThirdStop);
    check(v.gain == cfg.gainMax && v.digitalGain == cfg.digitalGainMax);

    /* What the frame ran on is reported as the whole ISO. */
    AutoFocusController::Report af{AfState::Inactive, AfMode::Off, 0.f};
    DelayedControls::Batch ran = cfg.configureDefaults();
    ran.val[DelayedControls::GAIN]         = cfg.gainMax;
    ran.val[DelayedControls::DIGITAL_GAIN] = 2 * unit;
    check(describe3A(FrameSettings(), af, {false, false}, ran, cfg).sensitivityIso == 3200);
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

/* An exposure is framed at the mode's own line: the frame grows to hold
 * it, up to the longest; one past the longest frame is cut to what it
 * holds -- the line is never stretched -- and manual AE writes that. */
void testLongExposureTiming() {
    const SensorConfig cfg = ov5693();
    const int32_t longest = cfg.maxExposureUs(cfg.frameLenMax);

    SensorConfig::FrameTiming t = cfg.timingForExposure(800000);
    check(t.lineLength == cfg.lineLengthDefault);
    check(t.frameLen == (int32_t)((800000000LL + 25199) / 25200) + cfg.maxCoarseDiff);

    for (int32_t us : {900000, 1600000, 5000000}) {
        t = cfg.timingForExposure(us);
        check(t.lineLength == cfg.lineLengthDefault);
        check(t.frameLen == cfg.frameLenMax);
    }

    FrameSettings s;
    s.ae.mode = AeMode::Off;
    s.sensor.exposureTimeNs = 1600000000LL;
    ExposureWriteValues v = AutoExposureController::parseManualSettings(s, cfg, kThirdStop);
    check(v.lineLength == cfg.lineLengthDefault && v.frameLen == cfg.frameLenMax);
    check(v.exposureUs == longest);
    /* The longest frame holds the exposure it is cut to. */
    check((int64_t)(v.frameLen - cfg.maxCoarseDiff) * cfg.lineTimeNs >= (int64_t)v.exposureUs * 1000);

    s.sensor.exposureTimeNs = 500000000LL;
    v = AutoExposureController::parseManualSettings(s, cfg, kThirdStop);
    check(v.exposureUs == 500000);
}

/* AE on a linear sensor: luma proportional to exposure x gain. It
 * converges and holds; one stop of EV compensation doubles the
 * exposure it settles on. */
float settleAe(AutoExposureController &ae, const SensorConfig &cfg,
               int32_t evSteps, float sceneGain, bool *converged,
               std::optional<FpsRange> fps = std::nullopt,
               DelayedControls::Batch *last = nullptr,
               Antibanding mains = Antibanding::Off) {
    IpaFrameMeta meta;
    meta.aeExposureCompensation = evSteps;
    meta.aeFpsRange = fps;
    meta.aeAntibanding = mains;
    const SensorConfig::FrameLimits limits = cfg.frameLimits(fps);
    float totalUs = (float)cfg.exposureDefault;
    float lastTotal = 0.f;
    for (int frame = 0; frame < 400; ++frame) {
        float luma = sceneGain * totalUs;
        if (luma > 1.f) luma = 1.f;
        const IpaStats stats = uniformStats(luma, luma, luma);
        const AeResult r = ae.process(stats, meta, 1.f, 1.f, totalUs, 1);
        if (r.batch.has[DelayedControls::EXPOSURE] && r.batch.has[DelayedControls::GAIN]) {
            /* Auto AE names the frame, within what the fps range
             * allows (without one, the mode's own), and the line. */
            check(r.batch.has[DelayedControls::FRAME_LENGTH] &&
                  r.batch.val[DelayedControls::FRAME_LENGTH] >= limits.minLen &&
                  r.batch.val[DelayedControls::FRAME_LENGTH] <= limits.maxLen);
            check(r.batch.has[DelayedControls::LINE_LENGTH] &&
                  r.batch.val[DelayedControls::LINE_LENGTH] == cfg.lineLengthDefault);
            check(r.batch.val[DelayedControls::EXPOSURE] <= cfg.maxExposureUs(limits));
            if (last) *last = r.batch;
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

/* The lines an exposure under mains light steps by: half the mains
 * period to the nearest line, none for Off. */
void testFlickerLines() {
    for (const SensorConfig &cfg : {imx179(), ov5693()}) {
        check(cfg.flickerLines(Antibanding::Off) == 0);
        const int64_t n50 = cfg.flickerLines(Antibanding::Hz50);
        const int64_t n60 = cfg.flickerLines(Antibanding::Hz60);
        check(llabs(n50 * cfg.lineTimeNs - 10000000) <= cfg.lineTimeNs / 2);
        check(llabs(n60 * cfg.lineTimeNs - 8333333) <= cfg.lineTimeNs / 2);
    }
}

/* Under flickering light the exposure is cut down to whole half
 * periods -- whole lines the driver's rounding down gives back exactly
 * -- and the gains make up the rest: the total stays. Shorter than a
 * half period, it is left alone; at the longest, the most half periods
 * the frame holds. */
void testFlickerSnap() {
    for (const SensorConfig &cfg : {imx179(), ov5693()}) {
        const SensorConfig::FrameLimits mode = cfg.modeFrame();
        const auto driverLines = [&](int32_t us) {
            return (int64_t)us * 1000 / cfg.lineTimeNs;
        };
        const auto total = [&](const SensorConfig::ExposureSplit &x) {
            return (float)x.exposureUs * (float)x.gain / (float)cfg.gainUnit
                 * (float)x.digitalGain / (float)SensorConfig::kDigitalGainUnit;
        };
        for (Antibanding mains : {Antibanding::Hz50, Antibanding::Hz60}) {
            const int32_t n = cfg.flickerLines(mains);
            const SensorConfig::ExposureSplit mid = cfg.splitExposure(25000.f, mode, n);
            check(driverLines(mid.exposureUs) == 2 * n);
            check(fabsf(total(mid) / 25000.f - 1.f) < 0.01f);
            check(mid.frameLen == cfg.frameLenDefault);

            const SensorConfig::ExposureSplit bright = cfg.splitExposure(5000.f, mode, n);
            check(bright.exposureUs == 5000 && bright.gain == cfg.gainUnit);

            const SensorConfig::ExposureSplit dark =
                cfg.splitExposure((float)cfg.maxExposureUsDefault() * 8, mode, n);
            const int64_t lines = driverLines(dark.exposureUs);
            const int64_t maxLines = driverLines(cfg.maxExposureUs(mode));
            check(lines % n == 0 && lines <= maxLines && lines + n > maxLines);
            check(dark.frameLen == cfg.frameLenDefault);
        }
        /* Off: as without. */
        check(cfg.splitExposure(25000.f, mode, 0).exposureUs == 25000);
    }
}

/* Auto exposure under a 50 Hz mains settles on the same total as
 * without, its exposure on the grid. Indoors: a couple of half periods
 * of exposure and some analog gain (settleAe's linear sensor takes in
 * no digital gain). */
void testAeAntibanding() {
    SensorTuning tuning;
    check(tuning.loadFile("tuning/imx179_primax.json"));
    const SensorConfig cfg = imx179();
    const float dim = 1.f / 150000.f;
    bool converged = false;

    AutoExposureController aeOff(cfg, &tuning, kThirdStop);
    const float off = settleAe(aeOff, cfg, 0, dim, &converged);
    AutoExposureController ae50(cfg, &tuning, kThirdStop);
    DelayedControls::Batch last;
    const float at50 = settleAe(ae50, cfg, 0, dim, &converged, std::nullopt, &last,
                                Antibanding::Hz50);
    check(converged);
    check(fabsf(at50 / off - 1.f) < 0.03f);
    const int64_t lines = (int64_t)last.val[DelayedControls::EXPOSURE] * 1000 / cfg.lineTimeNs;
    const int32_t n = cfg.flickerLines(Antibanding::Hz50);
    check(lines >= n && lines % n == 0);
}

/* A request dividing AE's exposure again within its own frame keeps
 * to its mains. */
void testRequestKeepsItsMains() {
    const SensorConfig cfg = imx179();
    const SensorConfig::ExposureSplit s = cfg.splitExposure(
        (float)cfg.maxExposureUsDefault() * 8, cfg.frameLimits(FpsRange{15, 30}));
    ExposureTarget target;
    DelayedControls::Batch decided = cfg.configureDefaults();
    decided.val[DelayedControls::EXPOSURE]     = s.exposureUs;
    decided.val[DelayedControls::GAIN]         = s.gain;
    decided.val[DelayedControls::DIGITAL_GAIN] = s.digitalGain;
    decided.val[DelayedControls::FRAME_LENGTH] = s.frameLen;
    target.publish(decided);

    FrameSettings req;
    req.ae.fpsRange    = FpsRange{30, 30};
    req.ae.antibanding = Antibanding::Hz60;
    const DelayedControls::Batch b = requestExposure(req, cfg, &target, kThirdStop);
    const int64_t lines = (int64_t)b.val[DelayedControls::EXPOSURE] * 1000 / cfg.lineTimeNs;
    check(lines % cfg.flickerLines(Antibanding::Hz60) == 0);
}

/* The AE target fps range bounds the frame: without one, or at the
 * mode's own rate, the mode's frame; [15, 30] up to 1/15 s; [15, 15]
 * 1/15 s always. Divided within it, the exposure comes first, its
 * frame lengthened for it, the gain after. */
void testFpsRangeFrameLimits() {
    const SensorConfig cfg = imx179();
    const SensorConfig::FrameLimits mode = cfg.modeFrame();
    const int32_t at15 = (int32_t)(1000000000LL / 15 / cfg.lineTimeNs);

    SensorConfig::FrameLimits l = cfg.frameLimits(std::nullopt);
    check(l.minLen == mode.minLen && l.maxLen == mode.maxLen);
    l = cfg.frameLimits(FpsRange{30, 30});
    check(l.minLen == mode.minLen && l.maxLen == mode.maxLen);
    const SensorConfig::FrameLimits wide = cfg.frameLimits(FpsRange{15, 30});
    check(wide.minLen == cfg.frameLenDefault && wide.maxLen == at15);
    const SensorConfig::FrameLimits slow = cfg.frameLimits(FpsRange{15, 15});
    check(slow.minLen == at15 && slow.maxLen == at15);

    const float dark = (float)cfg.maxExposureUsDefault() * 8;
    const SensorConfig::ExposureSplit fixed = cfg.splitExposure(dark);
    const SensorConfig::ExposureSplit longer = cfg.splitExposure(dark, wide);
    check(fixed.frameLen == cfg.frameLenDefault);
    check(longer.exposureUs == cfg.maxExposureUs(wide) && longer.frameLen == wide.maxLen);
    check(longer.gain < fixed.gain);
    /* Light enough for the mode's frame: no longer than it. */
    const SensorConfig::ExposureSplit bright =
        cfg.splitExposure((float)cfg.maxExposureUsDefault() / 2, wide);
    check(bright.frameLen == cfg.frameLenDefault && bright.gain == cfg.gainUnit);
    check(cfg.splitExposure(1000.f, slow).frameLen == at15);
}

/* In the dark, [15, 30] lets AE lengthen the frame before it adds
 * gain; [30, 30] holds the mode's frame and goes on in gain. */
void testAeFpsRange() {
    SensorTuning tuning;
    check(tuning.loadFile("tuning/imx179_primax.json"));
    const SensorConfig cfg = imx179();
    const float dark = 1.f / 8000000.f;
    bool converged = false;

    AutoExposureController aeWide(cfg, &tuning, kThirdStop);
    DelayedControls::Batch wide;
    settleAe(aeWide, cfg, 0, dark, &converged, FpsRange{15, 30}, &wide);
    AutoExposureController aeFixed(cfg, &tuning, kThirdStop);
    DelayedControls::Batch fixed;
    settleAe(aeFixed, cfg, 0, dark, &converged, FpsRange{30, 30}, &fixed);

    check(wide.val[DelayedControls::FRAME_LENGTH] > cfg.frameLenDefault);
    check(wide.val[DelayedControls::EXPOSURE] > cfg.maxExposureUsDefault());
    check(fixed.val[DelayedControls::FRAME_LENGTH] == cfg.frameLenDefault);
    /* The whole gain, analog and digital: the longer frame needs less. */
    const auto wholeGain = [](const DelayedControls::Batch &b) {
        return (int64_t)b.val[DelayedControls::GAIN] * b.val[DelayedControls::DIGITAL_GAIN];
    };
    check(wholeGain(wide) < wholeGain(fixed));
}

/* A request bounds its own frame whatever range AE decided for: a
 * frame too long for it is divided again within it, the gain making up
 * the exposure; too short, lengthened. */
void testRequestBoundsItsFrame() {
    const SensorConfig cfg = imx179();
    const SensorConfig::FrameLimits wide = cfg.frameLimits(FpsRange{15, 30});
    const SensorConfig::ExposureSplit s =
        cfg.splitExposure((float)cfg.maxExposureUsDefault() * 8, wide);

    ExposureTarget target;
    DelayedControls::Batch decided = cfg.configureDefaults();
    decided.val[DelayedControls::EXPOSURE]     = s.exposureUs;
    decided.val[DelayedControls::GAIN]         = s.gain;
    decided.val[DelayedControls::DIGITAL_GAIN] = s.digitalGain;
    decided.val[DelayedControls::FRAME_LENGTH] = s.frameLen;
    target.publish(decided);

    FrameSettings req;
    req.ae.fpsRange = FpsRange{15, 30};
    DelayedControls::Batch b = requestExposure(req, cfg, &target, kThirdStop);
    check(b.val[DelayedControls::FRAME_LENGTH] == s.frameLen);
    check(b.val[DelayedControls::EXPOSURE] == s.exposureUs);

    req.ae.fpsRange = FpsRange{30, 30};
    b = requestExposure(req, cfg, &target, kThirdStop);
    check(b.val[DelayedControls::FRAME_LENGTH] == cfg.frameLenDefault);
    check(b.val[DelayedControls::EXPOSURE] <= cfg.maxExposureUsDefault());
    check(b.val[DelayedControls::GAIN] > s.gain);

    DelayedControls::Batch light = cfg.configureDefaults();
    target.publish(light);
    req.ae.fpsRange = FpsRange{15, 15};
    b = requestExposure(req, cfg, &target, kThirdStop);
    check(b.val[DelayedControls::FRAME_LENGTH] == cfg.frameLimits(FpsRange{15, 15}).minLen);
    check(b.val[DelayedControls::EXPOSURE] == light.val[DelayedControls::EXPOSURE]);
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

/* AE as AF sees it: holding still or not, the light not measured. */
const AutoFocusController::AeView kAeSettled{true, 0.f};
const AutoFocusController::AeView kAeChasing{false, 0.f};

/* AF_REGIONS: a tap moves the focus ROI to it; the same rectangle at
 * weight 0 -- how apps drop a tap -- is no region, the centre default. */
/* IMX179's 1080p frame, as mocha's driver reports its window: the
 * whole active array. */
FrameMap imx179Frame1080() {
    const SensorGeometry g = SensorGeometry::fromModes(
        mocha::camera::Size{3280, 2464},
        {SensorMode{mocha::camera::Size{1920, 1080},
                    mocha::camera::Rect{20, 320, 3240, 1823}}});
    return FrameMap(g.activeArray, g.modes[0]);
}

void testAfRegionWeight() {
    SensorTuning tuning;
    check(tuning.loadFile("tuning/imx179_primax.json"));
    const FrameMap frame = imx179Frame1080();
    AutoFocusController af(&tuning, &frame);

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
    const FrameMap frame = imx179Frame1080();
    AutoFocusController af(&tuning, &frame);

    const int peak = 400;
    int lens = -1;
    auto statsAt = [&](int pos) {
        const float d = (float)(pos - peak) / 60.f;
        return uniformStats(0.4f, 0.4f, 0.4f, 50.f / (1.f + d * d));
    };

    FrameSettings s;
    s.af.mode = AfMode::Auto;
    af.onSettings(s, 0);
    af.process(statsAt(0), true, systemTime() + 10000000000LL, kAeSettled);
    check(af.report().afState == AfState::Inactive);

    s.af.trigger = AfTrigger::Start;
    af.onSettings(s, 1);
    s.af.trigger = AfTrigger::Idle;

    bool sawScan = false;
    bool sweepHeld = false;
    for (int tick = 0; tick < 400; ++tick) {
        const AfResult r = af.process(statsAt(lens < 0 ? 0 : lens), true,
                                      systemTime() + 10000000000LL, kAeSettled);
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
    af.process(statsAt(lens), true, systemTime() + 10000000000LL, kAeSettled);
    check(af.report().afState == AfState::Inactive);
}


/* AF: a trigger while AE is still chasing answers ACTIVE_SCAN at once
 * but leaves the lens and AE alone until AE converges -- the sweep
 * locks AE, and a lock taken mid-chase would hold the wrong exposure
 * for the whole sweep. The wait is capped, and CANCEL drops it. */
void testAfTriggerWaitsForAe() {
    SensorTuning tuning;
    check(tuning.loadFile("tuning/imx179_primax.json"));
    check(tuning.af().triggerAeWaitMs > 0);
    const FrameMap frame = imx179Frame1080();
    const int64_t later = systemTime() + 10000000000LL;
    const IpaStats stats = uniformStats(0.4f, 0.4f, 0.4f, 50.f);

    FrameSettings s;
    s.af.mode = AfMode::Auto;
    auto trigger = [&](AutoFocusController &af, AfTrigger t, uint32_t n) {
        s.af.trigger = t;
        af.onSettings(s, n);
        s.af.trigger = AfTrigger::Idle;
    };

    {
        AutoFocusController af(&tuning, &frame);
        trigger(af, AfTrigger::Start, 0);
        for (int tick = 0; tick < 5; ++tick) {
            const AfResult r = af.process(stats, true, later, kAeChasing);
            check(r.state == AfState::ActiveScan);
            check(!r.sweeping);
            check(!r.vcmPosition);
        }
        /* A second START while waiting changes nothing. */
        trigger(af, AfTrigger::Start, 1);
        check(!af.process(stats, true, later, kAeChasing).sweeping);

        const AfResult r = af.process(stats, true, later, kAeSettled);
        check(r.state == AfState::ActiveScan);
        check(r.sweeping);
        check((bool)r.vcmPosition);
    }

    {
        AutoFocusController af(&tuning, &frame);
        trigger(af, AfTrigger::Start, 0);
        check(!af.process(stats, true, later, kAeChasing).sweeping);
        trigger(af, AfTrigger::Cancel, 1);
        check(af.process(stats, true, later, kAeSettled).state == AfState::Inactive);
        const AfResult r = af.process(stats, true, later, kAeSettled);
        check(r.state == AfState::Inactive);
        check(!r.sweeping);
    }

    {
        AutoFocusController af(&tuning, &frame);
        trigger(af, AfTrigger::Start, 0);
        check(!af.process(stats, true, later, kAeChasing).sweeping);
        usleep((tuning.af().triggerAeWaitMs + 50) * 1000);
        check(af.process(stats, true, later, kAeChasing).sweeping);
    }
}


/* AF in a scene too dark to focus on: no sweep, the lens to the
 * low-light focus -- or to where the last focused sweep left it -- and
 * the verdict unfocused. The rear tuning, which switches the skip on;
 * the scenes are lit relative to its level. */
void testAfLowLightSkip() {
    SensorTuning tuning;
    check(tuning.loadFile("tuning/imx179_primax.json"));
    const float minLevel = tuning.af().lowLightMinLevel;
    check(minLevel > 0.f);
    if (minLevel <= 0.f) return;

    const FrameMap frame = imx179Frame1080();
    const int64_t later = systemTime() + 10000000000LL;
    const AutoFocusController::AeView dark{true, minLevel * 0.1f};
    const AutoFocusController::AeView lit{true, minLevel * 10.f};
    const int peak = 400;
    auto statsAt = [&](int pos) {
        const float d = (float)(pos - peak) / 60.f;
        return uniformStats(0.4f, 0.4f, 0.4f, 50.f / (1.f + d * d));
    };

    FrameSettings s;
    s.af.mode = AfMode::Auto;
    auto trigger = [&](AutoFocusController &af, AfTrigger t, uint32_t n) {
        s.af.trigger = t;
        af.onSettings(s, n);
        s.af.trigger = AfTrigger::Idle;
    };

    /* No focused position yet: the low-light focus, about 0.5 D past
     * infinity. */
    int lowLightPos = -1;
    {
        AutoFocusController af(&tuning, &frame);
        trigger(af, AfTrigger::Start, 0);
        const AfResult r = af.process(statsAt(0), true, later, dark);
        check(!r.sweeping);
        check(r.state == AfState::NotFocusedLocked);
        check((bool)r.vcmPosition);
        if (r.vcmPosition) lowLightPos = *r.vcmPosition;
        check(lowLightPos > 138 && lowLightPos < 200);
    }

    /* Light enough: the sweep runs. */
    {
        AutoFocusController af(&tuning, &frame);
        trigger(af, AfTrigger::Start, 0);
        check(af.process(statsAt(0), true, later, lit).sweeping);
    }

    /* The scene goes dark under a running sweep: it stops there, as a
     * skipped one would, instead of walking the rest of the range. */
    {
        AutoFocusController af(&tuning, &frame);
        trigger(af, AfTrigger::Start, 0);
        int lens = 0;
        for (int tick = 0; tick < 3; ++tick) {
            const AfResult r = af.process(statsAt(lens), true, later, lit);
            check(r.sweeping);
            if (r.vcmPosition) lens = *r.vcmPosition;
        }
        const AfResult r = af.process(statsAt(lens), true, later, dark);
        check(!r.sweeping);
        check(r.state == AfState::NotFocusedLocked);
        check(r.vcmPosition && *r.vcmPosition == lowLightPos);
    }

    /* After a focused sweep, the dark leaves the lens where it was
     * focused. */
    {
        AutoFocusController af(&tuning, &frame);
        trigger(af, AfTrigger::Start, 0);
        int lens = -1;
        for (int tick = 0; tick < 400; ++tick) {
            const AfResult r = af.process(statsAt(lens < 0 ? 0 : lens), true, later, lit);
            if (r.vcmPosition) lens = *r.vcmPosition;
            if (af.report().afState == AfState::FocusedLocked) break;
        }
        check(af.report().afState == AfState::FocusedLocked);
        const int focused = lens;
        trigger(af, AfTrigger::Cancel, 1);
        trigger(af, AfTrigger::Start, 2);
        const AfResult r = af.process(statsAt(lens), true, later, dark);
        check(!r.sweeping);
        check(r.state == AfState::NotFocusedLocked);
        check(r.vcmPosition && *r.vcmPosition == focused);
    }
}

} // namespace

int main() {
    testDescribe3A();
    testManualExposure();
    testDigitalGain();
    testLongExposureTiming();
    testAeLoop();
    testFpsRangeFrameLimits();
    testAeFpsRange();
    testRequestBoundsItsFrame();
    testFlickerLines();
    testFlickerSnap();
    testAeAntibanding();
    testRequestKeepsItsMains();
    testAwbConvergence();
    testGrayWorld();
    testAfRegionWeight();
    testAfSweep();
    testAfTriggerWaitsForAe();
    testAfLowLightSkip();
    return finish("three_a_test");
}
