#ifndef ISP_SENSOR_DELAYED_CONTROLS_H
#define ISP_SENSOR_DELAYED_CONTROLS_H

#include <stdint.h>

#include <map>

namespace android {

/* The sensor's controls by the sensor frame they are in effect on, after
 * libcamera's DelayedControls: requests file what they want for a
 * frame, the frames as they arrive say when to write, and what was in
 * effect on a frame is read back by its number.
 *
 * Frames are the sensor's (SensorSequence), and the write is timed by
 * them. The capture engine calls tick(k) as frame k arrives and writes
 * what tick returns once frame k + 1 has started; a control written
 * then is in effect on frame k + delay[id]. So each tick writes, for
 * each control, the value filed for frame k + delay[id], or keeps the
 * last one: values stay until a later
 * frame's are filed, and a frame no request wants runs on what the
 * last did.
 *
 * A frame the capture missed has no tick, and what was filed for the
 * frame its tick would have written is overtaken by the next tick's;
 * the record says what the sensor really ran either way.
 *
 * The capture engine's thread only: it files, ticks and reads. reset()
 * between streams, while it does not run. */
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
        /* In sensor frames: written at tick(k), a control is in effect
         * on frame k + delay. */
        int delay[COUNT];
    };

    explicit DelayedControls(const Config &config);

    /* A new stream: every frame runs on `initial`, every control set
     * -- what configure wrote before STREAMON -- until a write lands;
     * nothing filed, no frame ticked. */
    void reset(const Batch &initial);

    /* The first frame a request can still be filed for: every control
     * of it written at the next tick or later. */
    uint32_t firstFileable() const;

    /* Files the controls set in `batch` for frame `seq`, in effect from
     * it until a later frame's. False, and nothing filed, when `seq` is
     * before firstFileable(). */
    bool file(uint32_t seq, const Batch &batch);

    /* Drops what was filed for frame `seq` and is not written yet: the
     * frames from it run on what was filed before. */
    void unfile(uint32_t seq);

    /* What frame `seq` would run on as things stand, every control set:
     * the latest filed for it or before, or the last written. */
    Batch planned(uint32_t seq) const;

    /* Frame `seq` has arrived. What to write now, every control set:
     * for each control the value filed for frame seq + delay[id], or
     * the last written. `changed`: whether any differs from the last
     * written (or from `initial`). Recorded as in effect on those
     * frames; the record before frame `seq` is let go. */
    Batch tick(uint32_t seq, bool *changed);

    /* What was in effect on frame `seq`, every control set: the last
     * value written for a frame at or before it, or `initial`. Frames
     * from the last tick on. */
    Batch inEffect(uint32_t seq) const;

private:
    int maxDelay() const;

    Config config;
    Batch  initial;
    /* Filed and not written: frame -> value, per control. */
    std::map<uint32_t, int32_t> filed[COUNT];
    /* Written: the frame a value is in effect from -> value, per
     * control. */
    std::map<uint32_t, int32_t> written[COUNT];
    bool     ticked = false;
    uint32_t lastTick = 0;
};

} /* namespace android */

#endif /* ISP_SENSOR_DELAYED_CONTROLS_H */
