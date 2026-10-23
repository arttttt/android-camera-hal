#ifndef ISP_SENSOR_DELAYED_CONTROLS_H
#define ISP_SENSOR_DELAYED_CONTROLS_H

#include <stdint.h>

#include <map>

namespace android {

/* The sensor's controls by the sensor frame they are in effect on, after
 * libcamera's DelayedControls: requests file what they want for a
 * frame, the frames' starts say when to write, and what was in effect
 * on a frame is read back by its number.
 *
 * Frames are the sensor's, numbered by the capture driver, and the
 * write is timed by their starts: the capture engine calls tick(k) as
 * frame k starts and writes what tick returns at once, inside frame k;
 * a control written then is in effect on frame k + delay[id] -- the
 * delays are libcamera's SensorDelays, counted from the frame of the
 * write. So each tick writes, for each control, the value filed for
 * frame k + delay[id], or keeps the last one: values stay until a later
 * frame's are filed, and a frame no request wants runs on what the
 * last did.
 *
 * A request's controls are written together, in one tick, as the
 * sensor's group hold latches them: a request for frame `seq` -- the
 * frame its slowest control reaches -- files each control for the
 * frame its own delay reaches from that tick, seq - (maxDelay - delay).
 * A frame's length is in effect a frame before its exposure is
 * (rolling shutter: the frame before an exposure is what gives it its
 * time), so a request's frame length lands on the frame before its
 * own.
 *
 * A frame whose start the capture missed has no tick, and what was
 * filed for the frame its tick would have written is overtaken by the
 * next tick's; the record says what the sensor really ran either way.
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
        /* The ISP's gain past the sensor's analog gain, Q8 (256 = x1):
         * not written to the sensor. Its delay is the analog gain's,
         * so it is in effect on the frame the analog gain it makes up
         * for is, and that frame's ISP applies it (libcamera's RPi IPA
         * takes its digital gain from the same delayed context). */
        DIGITAL_GAIN = 4,
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

    /* Files the controls set in `batch` for a request on frame `seq`:
     * each for the frame frameOf(seq, id), written with the others,
     * and in effect from there until a later request's. False, and
     * nothing filed, when `seq` is before firstFileable(). */
    bool file(uint32_t seq, const Batch &batch);

    /* Drops what a request on frame `seq` filed and is not written yet:
     * the frames run on what was filed before. */
    void unfile(uint32_t seq);

    /* What a request on frame `seq` would find as things stand, every
     * control set: for each control, the latest filed for its frame
     * frameOf(seq, id) or before, or the last written. */
    Batch planned(uint32_t seq) const;

    /* The frame control `id` of a request on frame `seq` is in effect
     * on: seq - (maxDelay - delay[id]). For a frame so early that this
     * is before the stream's first: nothing earlier is asked about. */
    uint32_t frameOf(uint32_t seq, int id) const;

    /* Before the first tick: the stream's first frame to start is
     * `first`, not 0 -- what was filed moves on by it. */
    void rebase(uint32_t first);

    /* Frame `seq` has started. What to write now, every control set:
     * for each control the value filed for frame seq + delay[id], or
     * the last written. `changed`: whether any differs from the last
     * written (or from `initial`). Recorded as in effect on those
     * frames. */
    Batch tick(uint32_t seq, bool *changed);

    /* What tick(seq) would write as things stand, every control set;
     * nothing changes. */
    Batch peek(uint32_t seq) const;

    /* What was in effect on frame `seq`, every control set: the last
     * value written for a frame at or before it, or `initial`. Frames
     * not forgotten. */
    Batch inEffect(uint32_t seq) const;

    /* Nobody will ask about a request on a frame before `seq` any more
     * (its frame has come): the record behind what such a request needs
     * is let go. */
    void forgetBefore(uint32_t seq);

    /* What a request on frame `seq` ran on: each control as in effect
     * on frameOf(seq, id). */
    Batch inEffectFor(uint32_t seq) const;

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
