#ifndef ISP_SENSOR_DELAYED_CONTROLS_H
#define ISP_SENSOR_DELAYED_CONTROLS_H

#include <stdint.h>

#include <mutex>

namespace android {

/* Port of libcamera's DelayedControls. Each control id carries a
 * fixed, per-silicon latency in frames: the value written at request
 * sequence N lands in the kernel-captured frame at sequence N + delay.
 *
 * push() is called from two sources that share the same ring:
 *   - ApplySettingsStage on the RequestThread, for the frame being
 *     dispatched to the sensor right now (seq = request.frameNumber).
 *   - Ipa::processStats result on the PipelineThread, with
 *     seq = inputSequence + delay, publishing a 3A decision whose
 *     effect lands one delay window ahead.
 *
 * applyControls(seq) reports what is physically in effect on frame
 * `seq`: for each control, the value that was pushed at slot
 * (seq - delay). That value is written into result metadata so
 * ANDROID_SENSOR_EXPOSURE_TIME / SENSITIVITY reflect the sensor's
 * actual state rather than the request's target.
 *
 * Last-push-wins when two sources hit the same slot for the same
 * control id; the tagged per-cell sequence number rejects stale reads
 * after a wrap of the ring. */
class DelayedControls {
public:
    enum ControlId {
        EXPOSURE = 0,
        GAIN     = 1,
        COUNT
    };

    struct Batch {
        bool    has[COUNT];
        int32_t val[COUNT];
    };

    struct Config {
        /* In frames. Typical silicon: 2 for exposure, 2 for gain. */
        int     delay[COUNT];
        /* Seed value reported by applyControls() for any (seq, id)
         * that was never written. */
        int32_t defaultValue[COUNT];
    };

    explicit DelayedControls(const Config &config);

    /* Drops every cached write. All applyControls() queries fall back
     * to Config.defaultValue until the next push(). Called on session
     * boundary (closeDevice). */
    void reset();

    /* Stores each set control (batch.has[id] == true) into slot
     * (seq % RING_SIZE), tagged with the full `seq`. Controls not
     * flagged in the batch leave the corresponding cell untouched. */
    void push(uint32_t seq, const Batch &batch);

    /* Reports the value that was written for slot (seq - delay[id]).
     * Returns Config.defaultValue[id] if no matching write exists (the
     * cell was never touched, or the tagged sequence doesn't match
     * because the ring has wrapped past it). Used for result metadata
     * where the caller wants to know what was PHYSICALLY IN EFFECT on
     * frame `seq`. */
    Batch applyControls(uint32_t seq) const;

    /* Reports the controls queued to be WRITTEN to the sensor at frame
     * `seq` — slot `seq % RING_SIZE` directly, tag-checked against
     * `seq`. has[id] == false when no push for this slot has landed.
     * ApplySettingsStage uses this on the write path in auto mode to
     * pull the IPA's pushed batch for the frame being dispatched. */
    Batch pendingWrite(uint32_t seq) const;

private:
    static const int RING_SIZE = 16;

    struct Cell {
        bool     has;
        uint32_t seq;
        int32_t  val;
    };

    Config config;
    Cell   ring[RING_SIZE][COUNT];

    /* Producers (ApplySettingsStage on RequestThread, StatsProcessStage
     * on PipelineThread) and consumers (ApplySettingsStage,
     * ResultMetadataBuilder) run on different threads; the ring is
     * small enough that a single lock is fine. */
    mutable std::mutex mutex;
};

} /* namespace android */

#endif /* ISP_SENSOR_DELAYED_CONTROLS_H */
