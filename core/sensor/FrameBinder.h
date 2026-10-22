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
#include <vector>

#include "mochacamera/ports/FrameSource.h"
#include "sensor/DelayedControls.h"
#include "sensor/SensorSequence.h"

namespace android {

struct SensorConfig;

/* Binds each request to the sensor frame its settings are in effect on.
 *
 * The sensor sets the pace. Each frame, as it arrives, is numbered
 * (SensorSequence) and ticks DelayedControls: what was filed for the
 * frame a control delay ahead is what to write now, so writes are timed
 * by the frames, not by requests. Then the frame goes to the request
 * that asked for it, or back to the source.
 *
 * A request, as it arrives, is given its frame: the first its settings
 * can still be filed for, and after the frame of the request before it
 * -- one frame each, in order -- and its settings are filed for it.
 * However many requests are in flight, each gets the frame its settings
 * are in effect on; with fewer in flight than the control delay, the
 * frames between them simply go back unused.
 *
 * A request is given a frame only if the frame ran on its settings, by
 * the record of what was written. A frame the capture missed is gone,
 * and so is the write its tick would have made: its request, and the
 * one whose write that was, are settled by the first frame that comes
 * after -- given it if it ran on their settings, sharing it with the
 * request it belongs to, or lost. The requests after them keep their
 * own frames: a missed frame never shifts them.
 *
 * The record is only as true as the control delay: a sensor that takes
 * a write a frame sooner or later than configured runs every frame on
 * a neighbour's settings, and the record does not see it. The capture
 * engine times its writes so the delay does not depend on the mode.
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

    /* What a frame's arrival asks of the caller: write `values` (every
     * control set; to the sensor only when `changed`) once the next
     * frame has started, at `nextFrameStartNs` (CLOCK_MONOTONIC) -- the
     * control delay counts from a write inside that frame -- and wake
     * whoever holds a ticket in `settled`, given the frame or lost.
     * None: nobody asked for the frame, and it went back with its
     * lease. */
    struct Arrival {
        uint32_t               sensorFrame = 0;
        DelayedControls::Batch values;
        bool                   changed = false;
        int64_t                nextFrameStartNs = 0;
        std::vector<std::shared_ptr<Ticket>> settled;
    };

    FrameBinder(DelayedControls *controls, const SensorConfig *cfg);

    /* A new stream: frames are numbered from 0, nothing is asked for.
     * DelayedControls must have been reset to what configure wrote.
     * `lines`: the lines a frame reads out, for its exposure start. */
    void reset(uint32_t lines);

    /* A request with `settings` (every control it sets) gets its frame;
     * the ticket fills when the frame comes. A request let go drops its
     * ticket, and its frame goes back unused. */
    std::shared_ptr<Ticket> request(const DelayedControls::Batch &settings);

    /* A request given up before its frame (flush, the AE mode switch):
     * its settings come off the frame if they are not written yet, and
     * the frame runs on what was filed before -- a long exposure nobody
     * waits for any more is not taken. The caller then drops the
     * ticket. */
    void cancel(const Ticket &ticket);

    /* A frame came: the lease of its slot. */
    Arrival arrive(mocha::camera::BayerLease lease);

    /* Frames the capture missed since reset(). */
    uint32_t missed() const { return sequence.uncounted(); }

private:
    /* The first ticket still waiting; expired ones dropped. */
    std::shared_ptr<Ticket> front();
    void give(Ticket &t, const mocha::camera::BayerLease &lease,
              uint32_t sensorFrame, int64_t endNs);

    DelayedControls    *controls;
    const SensorConfig *cfg;
    uint32_t            lines = 0;

    SensorSequence sequence;
    /* Tickets waiting for their frame, in the order of their frames; a
     * request let go leaves an expired entry. */
    std::deque<std::weak_ptr<Ticket>> wanted;
    bool     anyRequest = false;
    uint32_t lastFrame  = 0;   /* the last frame a request was given */
};

} /* namespace android */

#endif /* CORE_SENSOR_FRAME_BINDER_H */
