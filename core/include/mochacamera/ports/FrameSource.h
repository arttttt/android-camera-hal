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

#ifndef MOCHACAMERA_PORTS_FRAME_SOURCE_H
#define MOCHACAMERA_PORTS_FRAME_SOURCE_H

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "mochacamera/Controls.h"

namespace mocha::camera {

/* One Bayer frame as the source handed it out. */
struct BayerFrame {
    /* The source's slot the frame lies in (the ISP's input ring slot
     * when the capture buffers are the ISP's). */
    int            slot   = -1;
    /* A CPU view of the frame, when the source maps its slots; null
     * when they are DMA buffers only the ISP reads. */
    const uint8_t *cpu    = nullptr;
    /* The sensor's pixel format (a V4L2 fourcc). */
    uint32_t       pixFmt = 0;

    /* The capture driver's count of the buffers it completed (damaged
     * ones included; a sensor frame it had no buffer for is not
     * counted), and when the frame's last line reached memory
     * (CLOCK_MONOTONIC, as systemTime). */
    uint32_t sequence    = 0;
    int64_t  timestampNs = 0;
};

/* Where a source slot goes back to once nobody holds it. Called from
 * whichever thread lets the last lease go; the source only records the
 * slot there and requeues it on its own thread. */
class SlotReturn {
public:
    virtual ~SlotReturn() = default;
    virtual void returnSlot(int slot, uint32_t generation) = 0;
};

/* A hold on one source slot -- the Bayer frame in it -- in place of a
 * raw buffer pointer and a reference count kept by hand. Move-only; a
 * second holder (the statistics, which read a frame past the frame's
 * own demosaic) takes an explicit share(). When the last lease is let
 * go, the slot returns to its source by itself, whatever path its
 * holders ended on.
 *
 * `generation` is the source's run the slot was handed out in: a lease
 * outliving a restart of the source returns a slot the restart has
 * requeued already, and the source ignores it. */
class BayerLease {
public:
    BayerLease() = default;
    BayerLease(const BayerFrame &frame, uint32_t generation, SlotReturn *owner)
        : hold(std::make_shared<Hold>(frame, generation, owner)) {}

    BayerLease(BayerLease &&) noexcept = default;
    BayerLease &operator=(BayerLease &&) noexcept = default;
    BayerLease(const BayerLease &) = delete;
    BayerLease &operator=(const BayerLease &) = delete;

    BayerLease share() const {
        BayerLease other;
        other.hold = hold;
        return other;
    }

    void reset() { hold.reset(); }

    explicit operator bool() const { return hold != nullptr; }
    /* Only on a held lease. */
    const BayerFrame &frame() const { return hold->frame; }

private:
    struct Hold {
        Hold(const BayerFrame &frame, uint32_t generation, SlotReturn *owner)
            : frame(frame), generation(generation), owner(owner) {}
        ~Hold() {
            if (owner && frame.slot >= 0)
                owner->returnSlot(frame.slot, generation);
        }
        Hold(const Hold &) = delete;
        Hold &operator=(const Hold &) = delete;

        BayerFrame frame;
        uint32_t   generation;
        SlotReturn *owner;
    };

    std::shared_ptr<Hold> hold;
};

/* Producer of Bayer frames for the core, driven by the capture
 * engine's thread: the engine polls fd() and returnedFd() with its own
 * work and calls back here, so the source's queue has one writer. A
 * frame is handed out as a lease; the lease brings its slot back when
 * its last holder lets go, from any thread, and the slot is requeued on
 * the capture thread. */
class FrameSource {
public:
    virtual ~FrameSource() = default;

    /* A new run. Called while the capture engine is not running. */
    virtual bool start() = 0;

    /* End of the run: the slots returned since the last requeue go
     * back; leases still out return nothing (their run is over). Called
     * while the capture engine is not running. */
    virtual void stop() = 0;

    virtual bool isRunning() const = 0;

    /* The device node, for the capture engine to watch. */
    virtual int fd() const = 0;

    /* Readable when leases have returned slots since requeueReturned(). */
    virtual int returnedFd() const = 0;

    /* On the capture thread, with the revents of fd(): every good frame
     * that is ready, oldest first, as leases; a damaged one goes back at
     * once. A lease let go returns its slot. */
    virtual std::vector<BayerLease> drain(short revents) = 0;

    /* On the capture thread: give the device the slots leases returned. */
    virtual void requeueReturned() = 0;

    /* What the source captures at. */
    virtual Size resolution() const = 0;

    /* The sensor's native size, to map regions given in active-array
     * coordinates to capture coordinates. */
    virtual Size sensorResolution() const = 0;
};

} /* namespace mocha::camera */

#endif /* MOCHACAMERA_PORTS_FRAME_SOURCE_H */
