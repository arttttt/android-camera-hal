#include "Ipa3A.h"

#include <math.h>
#include <stdint.h>

#include <system/camera_metadata.h>

#include <camera/CameraMetadata.h>

#include "3a/AeResult.h"
#include "3a/AfResult.h"
#include "3a/AutoExposureController.h"
#include "3a/AutoFocusController.h"
#include "3a/AutoWhiteBalanceController.h"
#include "3a/AwbResult.h"
#include "IpaFrameMeta.h"
#include "IpaStats.h"
#include "IspPipeline.h"
#include "metadata/CameraStaticMetadata.h"
#include "metadata/ResultMetadataBuilder.h"
#include "pipeline/PartialEmitter.h"
#include "sensor/SensorConfig.h"
#include "sensor/SensorTuning.h"

#define LOG_TAG "Cam-Ipa3A"
#include <utils/Log.h>

namespace android {

namespace {

float awbSceneLightFloorOf(const SensorTuning *t) {
    if (!t) return 0.f;
    const float v = t->awbParams().cStatsDarkThreshold;
    return v > 0.f ? v : 0.f;
}

/* For the diagnostic log: meanLumaInRoi is duplicated here from
 * AutoExposureController so the per-32-frame log can report the raw
 * scene-luma signal AE consumes. Cheap (256 patch reads, the
 * controller already does the same compute internally on its own
 * frame budget). Folds away in step 6 when Ipa3A owns the diagnostic
 * and reads it from AeResult instead. */
float meanLumaInRoiForLog(const IpaStats &stats, const IpaFrameMeta &meta) {
    int pyLo = meta.focusRoiPyLo < 0                 ? 0                 : meta.focusRoiPyLo;
    int pyHi = meta.focusRoiPyHi > IpaStats::PATCH_Y ? IpaStats::PATCH_Y : meta.focusRoiPyHi;
    int pxLo = meta.focusRoiPxLo < 0                 ? 0                 : meta.focusRoiPxLo;
    int pxHi = meta.focusRoiPxHi > IpaStats::PATCH_X ? IpaStats::PATCH_X : meta.focusRoiPxHi;

    int   count = 0;
    float sum   = 0.f;
    for (int py = pyLo; py < pyHi; ++py) {
        for (int px = pxLo; px < pxHi; ++px) {
            sum += stats.rgbMean[py][px][1];
            ++count;
        }
    }
    if (count <= 0) return 0.f;
    return sum / (float)count;
}

} /* namespace */

Ipa3A::Ipa3A(const SensorConfig &cfg, IspPipeline *ispPipeline,
                   AutoFocusController *afCtrl,
                   const SensorTuning *sensorTuning,
                   const float wbGainPrior[3],
                   int16_t *ccmBufQ10)
    : sensorCfg(cfg),
      isp(ispPipeline),
      af(afCtrl),
      tuning(sensorTuning),
      ccmBufferQ10(ccmBufQ10),
      awbSceneLightFloor(awbSceneLightFloorOf(sensorTuning)),
      mAe(new AutoExposureController(cfg, sensorTuning)),
      mAwb(new AutoWhiteBalanceController(sensorTuning, wbGainPrior)),
      frameCount(0) {
    ALOGD("3A knobs: awbSceneLightFloor=%.4f wbPrior=(%.3f,%.3f) "
          "gainMax=%d gainUnit=%d maxExpDef=%d tuningLoaded=%d",
          (double)awbSceneLightFloor,
          (double)mAwb->currentWbR(), (double)mAwb->currentWbB(),
          sensorCfg.gainMax, sensorCfg.gainUnit,
          sensorCfg.maxExposureUsDefault(),
          tuning ? (tuning->isLoaded() ? 1 : 0) : -1);

    /* Seed the shader immediately so the very first frame — before
     * any stats land — renders through the prior's WB gains instead
     * of the IspPipeline's unity defaults. */
    if (isp) {
        const WbGains seed = mAwb->currentGainsQ8();
        isp->setWbGains(seed.r, seed.g, seed.b);
    }
}

Ipa3A::~Ipa3A() = default;

void Ipa3A::reset() {
    mAe->reset();
    mAwb->reset();

    /* Re-seed the shader with the priors so the next session starts
     * from the sensor's calibrated daylight anchor even if the first
     * frame is below the AWB gate. */
    if (isp) {
        const WbGains seed = mAwb->currentGainsQ8();
        isp->setWbGains(seed.r, seed.g, seed.b);
    }
    if (tuning && ccmBufferQ10 && tuning->awbParams().loaded) {
        const float U       = logf(mAwb->currentWbB());
        const int   estCctK = tuning->estimateCctFromU(U);
        tuning->ccmForCctLerpQ10(estCctK, ccmBufferQ10);
    }
}

bool Ipa3A::isAeConverged() const {
    return mAe->isConverged();
}

void Ipa3A::setAeLock(bool lock) {
    mAe->setLock(lock);
}

DelayedControls::Batch Ipa3A::processStats(const IpaProcessParams &params) {
    const IpaStats     &stats = params.stats;
    const IpaFrameMeta &meta  = params.meta;
    DelayedControls::Batch emptyBatch;
    for (int i = 0; i < DelayedControls::COUNT; ++i) {
        emptyBatch.has[i] = false;
        emptyBatch.val[i] = 0;
    }

    /* AWB — coordinator-side gating decides which path runs. The
     * controller is pure compute, never hits the ISP / DelayedControls
     * itself. Auto path requires AWB AUTO mode, no framework lock,
     * no AF sweep (isp->awbLocked, set by AutoFocusController), and
     * a scene above the noise floor. Manual path takes user-provided
     * gains directly. Anything else holds. */
    const float sceneLuma = meanLumaInRoiForLog(stats, meta);
    const bool awbRun = (meta.awbMode == ANDROID_CONTROL_AWB_MODE_AUTO)
                     && (meta.awbLock == ANDROID_CONTROL_AWB_LOCK_OFF)
                     && (isp != nullptr)
                     && !isp->awbLocked()
                     && (sceneLuma >= awbSceneLightFloor);

    AwbResult awbResult;
    if (meta.awbMode == ANDROID_CONTROL_AWB_MODE_OFF
        && meta.manualWbValid && isp != nullptr) {
        awbResult = mAwb->applyManualGains(meta.manualWbR,
                                           meta.manualWbG,
                                           meta.manualWbB);
    } else if (awbRun) {
        awbResult = mAwb->process(stats);
    }

    /* Route AWB result. Gains hit the shader (zero silicon delay);
     * CCM lands in the shared row-major Q10 buffer the demosaic
     * shader reads by pointer. */
    if (awbResult.gains && isp) {
        isp->setWbGains(awbResult.gains->r,
                        awbResult.gains->g,
                        awbResult.gains->b);
    }
    if (awbResult.ccm && ccmBufferQ10) {
        for (int i = 0; i < 9; ++i) ccmBufferQ10[i] = awbResult.ccm->v[i];
    }

    /* Build the per-frame AF report once — used by buildAwbMetadata
     * (AWB-state derivation reads afState) and by ResultDispatchStage
     * later for the AF partial. */
    AutoFocusController::Report afReport;
    afReport.afState = ANDROID_CONTROL_AF_STATE_INACTIVE;
    afReport.afMode  = ANDROID_CONTROL_AF_MODE_OFF;
    afReport.focusDiopter = 0.f;
    if (af) afReport = af->report();

    /* AWB partial — emit immediately so apps gating UI on AWB state
     * see it without waiting for the buffer-bearing final result.
     * The metadata subset includes ANDROID_CONTROL_AWB_STATE /
     * ANDROID_CONTROL_AWB_MODE; everything else lands later. The
     * source CameraMetadata is a copy of the request settings so
     * `cm.exists(ANDROID_CONTROL_AWB_LOCK)` and friends used by the
     * builder's state derivation pick up app-provided values. */
    if (params.emitter) {
        ResultMetadataBuilder::FrameState fs;
        fs.timestampNs       = params.timestampNs;
        fs.frameNumber       = params.frameNumber;
        fs.appliedExposureUs = 0;
        fs.appliedGain       = 0;
        fs.af                = afReport;
        fs.aeConverged       = false;

        CameraMetadata awbMd = params.requestSettings;
        ResultMetadataBuilder::buildAwbMetadata(awbMd, fs);
        const camera_metadata_t *awbBlob = awbMd.getAndLock();
        params.emitter->emit(params.frameNumber, 1, awbBlob, 0, nullptr);
        awbMd.unlock(awbBlob);
    }

    /* AE — coordinator gates the manual-mode skip; otherwise the
     * controller runs and decides internally whether to publish the
     * lock-held value or the freshly-computed batch. The current WB
     * (post-this-tick if AWB ran, last-known otherwise) feeds AE's
     * highlight-protection candidate. */
    AeResult aeResult;
    if (meta.aeMode != ANDROID_CONTROL_AE_MODE_OFF) {
        aeResult = mAe->process(stats, meta,
                                mAwb->currentWbR(),
                                mAwb->currentWbB());
    }

    /* AE partial — emit immediately. Apps watching ANDROID_CONTROL_AE_STATE
     * (e.g. PRECAPTURE / CONVERGED transitions) see them as soon as the
     * IPA tick finishes, ahead of the buffer-bearing final result.
     * appliedExposureUs / appliedGain on this frame come from
     * DelayedControls::applyControls(frameNumber) — what's physically
     * in effect right now, not the IPA's just-decided batch (which
     * lands one delay window later). */
    if (params.emitter) {
        int32_t appliedExpUs = params.sensorCfg.exposureDefault;
        int32_t appliedGain  = params.sensorCfg.gainDefault;
        if (params.delayedControls) {
            const DelayedControls::Batch live =
                params.delayedControls->applyControls(params.frameNumber);
            if (live.has[DelayedControls::EXPOSURE])
                appliedExpUs = live.val[DelayedControls::EXPOSURE];
            if (live.has[DelayedControls::GAIN])
                appliedGain  = live.val[DelayedControls::GAIN];
        }

        ResultMetadataBuilder::FrameState fs;
        fs.timestampNs       = params.timestampNs;
        fs.frameNumber       = params.frameNumber;
        fs.appliedExposureUs = appliedExpUs;
        fs.appliedGain       = appliedGain;
        fs.af                = afReport;
        fs.aeConverged       = mAe->isConverged();

        CameraMetadata aeMd = params.requestSettings;
        ResultMetadataBuilder::buildAeMetadata(aeMd, fs, params.sensorCfg);
        const camera_metadata_t *aeBlob = aeMd.getAndLock();
        params.emitter->emit(params.frameNumber, 2, aeBlob, 0, nullptr);
        aeMd.unlock(aeBlob);
    }

    /* AF — runs every tick the controller is wired in. process()
     * harvests pending edge signals (startSweep / sweepComplete);
     * the coordinator translates them into AE / AWB lock toggles
     * since the controllers themselves are decoupled from each other.
     * VCM writes still happen inside AutoFocusController via mDev
     * (timing matters — sweep-step VCM writes need to land on the
     * frame the state machine decides; threading them through
     * the result would add a frame of latency). */
    if (af) {
        const AfResult afResult = af->process(stats, aeResult.converged);
        if (afResult.startSweep) {
            if (isp) isp->setAwbLock(true);
            mAe->setLock(true);
        }
        if (afResult.sweepComplete) {
            if (isp) isp->setAwbLock(false);
            mAe->setLock(false);
        }
    }

    /* Throttled diagnostic. Same per-32-frame format the previous
     * monolithic loop used; reads state from controllers via
     * accessors + result fields. */
    const bool logTick = ((frameCount++ & 0x1f) == 0u);
    if (logTick) {
        const float totalUs = mAe->currentFilteredTotalUs();
        int32_t diagExp = 0, diagExtraQ8 = 256;
        sensorCfg.splitExposureGain((int32_t)(totalUs + 0.5f),
                                     &diagExp, &diagExtraQ8);
        const int32_t diagGain = (int32_t)(((int64_t)sensorCfg.gainUnit
                                           * diagExtraQ8 + 128) / 256);
        const int32_t diagGainClamped =
            diagGain > sensorCfg.gainMax ? sensorCfg.gainMax
                                          : (diagGain < 1 ? 1 : diagGain);
        const WbGains gQ8 = mAwb->currentGainsQ8();
        ALOGD("3A: frame=%u luma=%.3f iqmHi=%.3f nValid=%d awbRun=%d "
              "lastWb=(%.3f,%.3f) Q8=(%u,%u) estCct=%d "
              "totalUs=%.0f exp=%d gain=%d gainClamp=%d evComp=%d",
              frameCount, (double)sceneLuma, (double)aeResult.iqmHighlight,
              awbResult.validPatchCount, awbRun ? 1 : 0,
              (double)mAwb->currentWbR(), (double)mAwb->currentWbB(),
              gQ8.r, gQ8.b,
              mAwb->currentEstCct(), (double)totalUs, diagExp,
              diagGain, diagGainClamped,
              meta.aeExposureCompensation);
    }

    return aeResult.batch;
}

} /* namespace android */
