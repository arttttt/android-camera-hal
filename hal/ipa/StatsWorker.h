#ifndef HAL_IPA_STATS_WORKER_H
#define HAL_IPA_STATS_WORKER_H

#include <stdint.h>

#include <mutex>

#include "EventFd.h"
#include "IpaStats.h"
#include "NeonStatsEncoder.h"
#include "ThreadBase.h"

namespace android {

/* Dedicated worker thread that reduces raw Bayer into IpaStats via a
 * progressive NEON kernel. Runs outside the GPU-submit critical path
 * so the CPU stats cost overlaps the Vulkan demosaic + blit submit
 * instead of serialising with it.
 *
 * Progressive cadence: one full IpaStats computation spans `phaseCount`
 * submits. Each incoming submit advances the worker's internal phase
 * by one step, doing 1/phaseCount of the patch-row work on the Bayer
 * slot that started the cycle. The final phase combines all partials
 * and publishes the result. This keeps peak CPU per frame at roughly
 * (total_compute / phaseCount) while the design budget for one stats
 * result matches (frame_period × phaseCount). Raising phaseCount buys
 * richer stats headroom later (e.g. per-channel histograms, bigger
 * patch grid) without changing the skeleton.
 *
 * Bayer slot lifetime: with V4L2DEVICE_BUF_COUNT = 8 and the HAL
 * holding at most three slots at a time, a released slot rotates back
 * to the sensor after (5..7) × sensor_period ≈ 80..120 ms. Carrying
 * one Bayer pointer across the phaseCount-frame span (~22 ms at
 * 60 fps, phaseCount = 2) is well inside that window — no memcpy or
 * explicit reservation is needed on this pipeline.
 *
 * Producer (RequestThread's StatsDispatchStage): submit(job) with the
 * latest Bayer pointer and sequence. Submits are latest-wins for the
 * "idle → start cycle" transition: a new job arriving while the
 * worker is mid-cycle leaves the incoming one queued; when the cycle
 * completes, the next wake-up starts a new cycle from that queued
 * job (or waits for a later one).
 *
 * Consumer (PipelineThread's StatsProcessStage): peek(&stats) returns
 * the most recently finalised IpaStats under mutex. Steady-state lag
 * between submit and peek is `phaseCount` frames; the IPA treats
 * stats as an input, not a blocker. */
class StatsWorker : public ThreadBase {
public:
    struct Job {
        const void *bayer;
        unsigned    width;
        unsigned    height;
        uint32_t    pixFmt;
        uint32_t    sequence;
    };

    /* Number of submits a full IpaStats cycle spans. Each computeRange
     * covers 1/phaseCount of the patch rows. Keeping this a small
     * compile-time constant simplifies the phase dispatch; raise it
     * when compute grows or frame times shrink. */
    static constexpr int phaseCount = 2;

    StatsWorker();
    ~StatsWorker() override;

    /* Post the latest work to the worker. Overwrites any pending but
     * not-yet-started job. Safe from any thread. */
    void submit(const Job &job);

    /* Copy the latest published IpaStats into `out` and its source
     * frame sequence into `*sequence`. Returns false on cold start
     * (no publication yet). Safe from any thread. */
    bool peek(IpaStats *out, uint32_t *sequence);

    /* Drop the pending job, the in-progress cycle, and the latest
     * publication. Call between sessions so stats from the previous
     * capture do not leak into the next one. Requires the worker
     * thread to be quiesced (stop() joined) — cycle state (phase,
     * partial) is mutated without locking on the assumption that
     * only the worker thread touches it during a run. Camera's
     * closeDevice meets this precondition via stopWorkers(). */
    void reset();

protected:
    void threadLoop() override;

private:
    NeonStatsEncoder encoder;

    /* Producer → worker: pending latest job + a mutex-guarded "valid"
     * flag, woken via jobEvent. A separate eventfd lets the worker
     * drive its poll() set alongside the base class's stop fd. */
    EventFd    jobEvent;
    std::mutex inLock;
    Job        pending;
    bool       pendingValid;

    /* In-flight cycle state. Carried across `phaseCount` wake-ups.
     * currentJob is captured at the start of a cycle (phase 0) and
     * used for every subsequent phase on the same Bayer slot.
     * `phase` is the number of computeRange() calls already done;
     * the cycle publishes and resets once phase reaches phaseCount. */
    Job                       currentJob;
    int                       phase;
    NeonStatsEncoder::Partial partial;

    /* Worker → consumer: the latest finished IpaStats and its source
     * sequence. Snapshot-copied under outLock. */
    std::mutex outLock;
    IpaStats   latest;
    uint32_t   latestSeq;
    bool       latestValid;
};

} /* namespace android */

#endif /* HAL_IPA_STATS_WORKER_H */
