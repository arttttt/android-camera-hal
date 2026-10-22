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

#ifndef CORE_SENSOR_FRAME_BINDER_H
#define CORE_SENSOR_FRAME_BINDER_H

#include <stdint.h>

#include <deque>
#include <memory>
#include <optional>
#include <set>
#include <vector>

#include "mochacamera/ports/FrameSource.h"
#include "sensor/DelayedControls.h"

namespace android {

struct SensorConfig;

/* Binds each request to the sensor frame its settings are in effect on.
 *
 * The sensor sets the pace, and the capture driver numbers its frames.
 * Each frame's start ticks DelayedControls: what was filed for the
 * frames a control delay ahead is what to write now, inside the frame,
 * so writes are timed by the frames, not by requests. Each frame, as
 * it arrives, goes to the request that asked for it, or back to the
 * source.
 *
 * A request, as it arrives, is given its frame: the first its settings
 * can still be filed for, and after the frame of the request before it
 * -- one frame each, in order -- and its settings are filed for it. A
 * stream's settings (preview, video) hold on through the frames nobody
 * asks for; a still capture's hold for its own frame only, and the
 * frame after it goes back to the stream's: a long manual shot does not
 * run on through the frames after it, seconds each, before the preview
 * comes back.
 * However many requests are in flight, each gets the frame its settings
 * are in effect on; with fewer in flight than the control delay, the
 * frames between them simply go back unused.
 *
 * A request is given a frame only if the frame ran on its settings, by
 * the record of what was written (each control on the frame its delay
 * put it on, DelayedControls::inEffectFor). A frame the capture missed
 * is gone, and if its start was missed too, so is the write its tick
 * would have made: its request, and the one whose write that was, are
 * settled by the first frame that comes after -- given it if it ran on
 * their settings, sharing it with the request it belongs to, or lost.
 * The requests after them keep their own frames: a missed frame never
 * shifts them.
 *
 * Frames are numbered as the sensor runs them, which is not always as
 * the driver numbers them: a sensor that runs the frame after a line
 * length write without a frame start (SensorConfig::lineLengthHidesFrame)
 * has the driver count one frame fewer from there. The binder knows the
 * writes it asks for, and numbers the driver's frames past such a write
 * one on; nothing is written in a hidden frame, and its tick's writes
 * go out at the next start, as for a start not reported.
 *
 * The record is only as true as the sensor's delays: a sensor that
 * takes a write a frame sooner or later than configured runs every
 * frame on a neighbour's settings, and the record does not see it.
 *
 * The capture engine's thread only. */
class FrameBinder {
public:
    /* A request's claim on its frame. */
    struct Ticket {
        uint32_t frame = 0;   /* the sensor frame asked for */
        DelayedControls::Batch settings;   /* what it filed */
        /* No frame ran on its settings: its frame, or its write, was
         * missed. */
        bool lost = false;
        /* Set when a frame comes for it: the frame, its number, what
         * was in effect on it, when its first line began its exposure
         * (CLOCK_MONOTONIC). */
        std::optional<mocha::camera::BayerLease> bayer;
        uint32_t sensorFrame     = 0;
        int64_t  exposureStartNs = 0;
        DelayedControls::Batch inEffect;
    };

    /* What a frame's start asks of the caller: write `values` (every
     * control set) to the sensor now, inside the frame, if `changed`.
     * Nothing to write unless `ticked`: a start out of order is not
     * written on. The frame runs for `periodNs` (its controls as
     * written).
     *
     * A frame length written early in a frame may stretch that very
     * frame (the IMX179 does, 2.5 ms in), and the VI starts waiting on
     * the next frame as this one starts: its wait must be up a start
     * before the write. `waitPeriodNs` is the longest from one start
     * to the next this tick's write or the next one's makes -- with
     * the frame a line length write hides -- requests are filed two
     * frames ahead, so the next is known -- for the VI's wait now. */
    struct Start {
        bool                   ticked = false;
        DelayedControls::Batch values;
        bool                   changed = false;
        int64_t                periodNs = 0;
        int64_t                waitPeriodNs = 0;
    };

    /* A frame's arrival: wake whoever holds a ticket in `settled`,
     * given the frame or lost. None: nobody asked for the frame, and it
     * went back with its lease. */
    struct Arrival {
        uint32_t sensorFrame = 0;
        std::vector<std::shared_ptr<Ticket>> settled;
    };

    FrameBinder(DelayedControls *controls, const SensorConfig *cfg);

    /* A new stream: nothing is asked for, no frame has come. The
     * driver numbers the stream's frames from 0. DelayedControls must
     * have been reset to what configure wrote. `lines`: the lines a
     * frame reads out, for its exposure start. */
    void reset(uint32_t lines);

    /* A request with `settings` (every control it sets) gets its frame;
     * the ticket fills when the frame comes. `stillCapture`: its
     * settings hold for its frame only. A request let go drops its
     * ticket, and its frame goes back unused. */
    std::shared_ptr<Ticket> request(const DelayedControls::Batch &settings,
                                    bool stillCapture);

    /* A request given up before its frame (flush, the AE mode switch):
     * its settings come off the frame if they are not written yet, and
     * the frame runs on what was filed before -- a long exposure nobody
     * waits for any more is not taken. The caller then drops the
     * ticket. */
    void cancel(const Ticket &ticket);

    /* The driver's frame `sequence` has started. The stream's first
     * start may be any number: the frames asked for before it move on
     * by it. A start not after the last one, or after its own frame
     * came, is not ticked. */
    Start start(uint32_t sequence);

    /* A frame came: the lease of its slot, numbered by the driver. */
    Arrival arrive(mocha::camera::BayerLease lease);

    /* Frames the capture had no buffer for, and frame starts it did not
     * report, since reset(): gaps in the driver's numbers. */
    uint32_t missedFrames() const { return framesMissed; }
    uint32_t missedStarts() const { return startsMissed; }

private:
    /* The sensor frame the driver numbers `sequence`. */
    uint32_t frameOf(uint32_t sequence) const;
    /* Frame `frame` runs without a start. */
    bool hidden(uint32_t frame) const;
    /* The first ticket still waiting; expired ones dropped. */
    std::shared_ptr<Ticket> front();
    void give(Ticket &t, const mocha::camera::BayerLease &lease,
              uint32_t sensorFrame, int64_t endNs);

    DelayedControls    *controls;
    const SensorConfig *cfg;
    uint32_t            lines = 0;

    bool     anyFrame = false;
    uint32_t lastFrame = 0;       /* the last frame that came */
    bool     anyStart = false;
    uint32_t lastStart = 0;       /* the last frame that started */
    uint32_t framesMissed = 0;
    uint32_t startsMissed = 0;
    /* Frames a line length write hid that a start or a frame may still
     * be numbered before: the frame, and the driver's number of the
     * frame it was written in -- the driver's numbers past it are one
     * frame on. Those both have passed are counted in `hiddenPassed`. */
    struct Hidden {
        uint32_t frame;
        uint32_t afterSequence;
    };
    std::deque<Hidden> hiddenAhead;
    uint32_t hiddenPassed = 0;
    /* Frames no request is given: a still's line length write hides
     * them, or would make their writes in a hidden frame. */
    std::set<uint32_t> unserved;
    /* The line length the last tick wrote, if it ticked. */
    std::optional<int32_t> lineLength;
    /* Tickets waiting for their frame, in the order of their frames; a
     * request let go leaves an expired entry. */
    std::deque<std::weak_ptr<Ticket>> wanted;
    bool     anyRequest = false;
    uint32_t lastAsked  = 0;   /* the last frame a request was given */
    /* The last stream request's settings: what the frames after a still
     * capture go back to. */
    std::optional<DelayedControls::Batch> streamSettings;
};

} /* namespace android */

#endif /* CORE_SENSOR_FRAME_BINDER_H */
