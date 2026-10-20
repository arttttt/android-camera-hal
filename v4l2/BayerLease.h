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

#ifndef V4L2_BAYER_LEASE_H
#define V4L2_BAYER_LEASE_H

#include <cstdint>
#include <memory>
#include <utility>

#include "V4l2Device.h"

namespace mocha::camera {

/* Where a capture slot goes back to once nobody holds it. Called from
 * whichever thread lets the last lease go; the source only records the
 * slot there and requeues it on its own thread. */
class SlotReturn {
public:
    virtual ~SlotReturn() = default;
    virtual void returnSlot(int index, uint32_t generation) = 0;
};

/* A hold on one capture slot -- the Bayer frame the sensor wrote into
 * it -- in place of a raw buffer pointer and a reference count kept by
 * hand. Move-only; a second holder (the statistics, which read a frame
 * past the frame's own demosaic) takes an explicit share(). When the
 * last lease is let go, the slot returns to its source by itself,
 * whatever path its holders ended on.
 *
 * `generation` is the source's run the slot was handed out in: a lease
 * outliving a restart of the source returns a slot the restart has
 * requeued already, and the source ignores it. */
class BayerLease {
public:
    using VBuffer = android::V4l2Device::VBuffer;

    BayerLease() = default;
    BayerLease(const VBuffer *buffer, uint32_t generation, SlotReturn *owner)
        : hold(std::make_shared<Hold>(buffer, generation, owner)) {}

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
    const VBuffer *buffer() const { return hold ? hold->buffer : nullptr; }
    int index() const { return hold ? hold->buffer->index : -1; }

private:
    struct Hold {
        Hold(const VBuffer *buffer, uint32_t generation, SlotReturn *owner)
            : buffer(buffer), generation(generation), owner(owner) {}
        ~Hold() {
            if (owner && buffer)
                owner->returnSlot(buffer->index, generation);
        }
        Hold(const Hold &) = delete;
        Hold &operator=(const Hold &) = delete;

        const VBuffer *buffer;
        uint32_t generation;
        SlotReturn *owner;
    };

    std::shared_ptr<Hold> hold;
};

} /* namespace mocha::camera */

#endif /* V4L2_BAYER_LEASE_H */
