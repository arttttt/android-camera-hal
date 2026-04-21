#include "DelayedControls.h"

namespace android {

DelayedControls::DelayedControls(const Config &cfg) : config(cfg) {
    reset();
}

void DelayedControls::reset() {
    for (int s = 0; s < RING_SIZE; ++s) {
        for (int id = 0; id < COUNT; ++id) {
            ring[s][id].has = false;
            ring[s][id].seq = 0;
            ring[s][id].val = 0;
        }
    }
}

void DelayedControls::push(uint32_t seq, const Batch &batch) {
    const int slot = static_cast<int>(seq % RING_SIZE);
    for (int id = 0; id < COUNT; ++id) {
        if (!batch.has[id]) continue;
        ring[slot][id].has = true;
        ring[slot][id].seq = seq;
        ring[slot][id].val = batch.val[id];
    }
}

DelayedControls::Batch DelayedControls::applyControls(uint32_t seq) const {
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

} /* namespace android */
