#include "DelayedControls.h"

namespace android {

DelayedControls::DelayedControls(const Config &cfg) : config(cfg) {
    reset();
}

void DelayedControls::reset() {
    std::lock_guard<std::mutex> lock(mutex);
    for (int s = 0; s < RING_SIZE; ++s) {
        for (int id = 0; id < COUNT; ++id) {
            ring[s][id].has = false;
            ring[s][id].seq = 0;
            ring[s][id].val = 0;
        }
    }
}

void DelayedControls::push(uint32_t seq, const Batch &batch) {
    std::lock_guard<std::mutex> lock(mutex);
    const int slot = static_cast<int>(seq % RING_SIZE);
    for (int id = 0; id < COUNT; ++id) {
        if (!batch.has[id]) continue;
        ring[slot][id].val = batch.val[id];
        ring[slot][id].seq = seq;
        ring[slot][id].has = true;
    }
}

DelayedControls::Batch DelayedControls::applyControls(uint32_t seq) const {
    std::lock_guard<std::mutex> lock(mutex);
    Batch out;
    for (int id = 0; id < COUNT; ++id) {
        const uint32_t want = seq - static_cast<uint32_t>(config.delay[id]);
        const int slot = static_cast<int>(want % RING_SIZE);
        const Cell &cell = ring[slot][id];
        if (cell.has && cell.seq == want) {
            out.has[id] = true;
            out.val[id] = cell.val;
        } else {
            out.has[id] = true;
            out.val[id] = config.defaultValue[id];
        }
    }
    return out;
}

DelayedControls::Batch DelayedControls::pendingWrite(uint32_t seq) const {
    std::lock_guard<std::mutex> lock(mutex);
    Batch out;
    const int slot = static_cast<int>(seq % RING_SIZE);
    for (int id = 0; id < COUNT; ++id) {
        const Cell &cell = ring[slot][id];
        if (cell.has && cell.seq == seq) {
            out.has[id] = true;
            out.val[id] = cell.val;
        } else {
            out.has[id] = false;
            out.val[id] = 0;
        }
    }
    return out;
}

} /* namespace android */
