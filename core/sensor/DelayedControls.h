#ifndef ISP_SENSOR_DELAYED_CONTROLS_H
#define ISP_SENSOR_DELAYED_CONTROLS_H

#include <stdint.h>

#include <mutex>

namespace android {

/* Port of libcamera's DelayedControls. Each control id carries a
 * fixed, per-silicon latency in frames: the value written at request
 * sequence N lands in the kernel-captured frame at sequence N + delay.
 *
 * commit() records what ApplySettingsStage wrote for a request, and
 * applyControls(seq) reports what is physically in effect on frame
 * `seq`: for each control, the value committed at slot (seq - delay).
 * That value is written into result metadata so
 * ANDROID_SENSOR_EXPOSURE_TIME / SENSITIVITY reflect the sensor's
 * actual state rather than the request's target.
 *
 * The tagged per-cell sequence number rejects stale reads after a wrap
 * of the ring. */
class DelayedControls {
public:
    enum ControlId {
        EXPOSURE     = 0,
        GAIN         = 1,
        /* Lines per frame: the frame period the exposure fits in. The
         * HAL owns it (the drivers no longer stretch it), and the VI
         * capture timeout follows the frames in flight by it. */
        FRAME_LENGTH = 2,
        /* Pixel clocks per line: the mode's own, longer only for an
         * exposure the line counter cannot hold at the mode's line time
         * (a long manual exposure). Times the frame with FRAME_LENGTH. */
        LINE_LENGTH  = 3,
        COUNT
    };

    /* Empty unless set: a control not named in a batch is not
     * written, whoever builds it. */
    struct Batch {
        bool    has[COUNT] = {};
        int32_t val[COUNT] = {};
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
     * to Config.defaultValue until the next commit(). Called on session
     * boundary (Session::close). */
    void reset();

    /* Records what was written to the sensor for request `seq`: each
     * set control (batch.has[id] == true) into slot (seq % RING_SIZE),
     * tagged with the full `seq`. Controls not flagged in the batch
     * leave the corresponding cell untouched. */
    void commit(uint32_t seq, const Batch &batch);

    /* Reports the value that was written for slot (seq - delay[id]).
     * Returns Config.defaultValue[id] if no matching write exists (the
     * cell was never touched, or the tagged sequence doesn't match
     * because the ring has wrapped past it). Used for result metadata
     * where the caller wants to know what was PHYSICALLY IN EFFECT on
     * frame `seq`. */
    Batch applyControls(uint32_t seq) const;

    /* The same, with `fallback` reported for a control that has no
     * matching write instead of Config.defaultValue. Result metadata
     * passes the defaults configureStreams writes before STREAMON, so
     * a cold-start frame reports what the sensor started with rather
     * than 0. (The IPA keeps the plain form: it reads 0 as "nothing
     * applied yet".) */
    Batch applyControls(uint32_t seq, const int32_t fallback[COUNT]) const;

private:
    static const int RING_SIZE = 16;

    struct Cell {
        bool     has;
        uint32_t seq;
        int32_t  val;
    };

    Config config;
    Cell   ring[RING_SIZE][COUNT];

    /* The producer (ApplySettingsStage on the capture engine) and the
     * consumers (the IPA, result metadata) run on different threads;
     * the ring is small enough that a single lock is fine. */
    mutable std::mutex mutex;
};

} /* namespace android */

#endif /* ISP_SENSOR_DELAYED_CONTROLS_H */
