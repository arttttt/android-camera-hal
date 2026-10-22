#include "DelayedControls.h"

#include <algorithm>

namespace android {

DelayedControls::DelayedControls(const Config &cfg) : config(cfg) {
    reset(Batch{});
}

void DelayedControls::reset(const Batch &init) {
    initial = init;
    for (int id = 0; id < COUNT; ++id) {
        initial.has[id] = true;
        filed[id].clear();
        written[id].clear();
    }
    ticked   = false;
    lastTick = 0;
}

int DelayedControls::maxDelay() const {
    int d = 0;
    for (int id = 0; id < COUNT; ++id) d = std::max(d, config.delay[id]);
    return d;
}

uint32_t DelayedControls::firstFileable() const {
    const uint32_t nextTick = ticked ? lastTick + 1 : 0;
    return nextTick + static_cast<uint32_t>(maxDelay());
}

uint32_t DelayedControls::frameOf(uint32_t seq, int id) const {
    return seq - static_cast<uint32_t>(maxDelay() - config.delay[id]);
}

bool DelayedControls::file(uint32_t seq, const Batch &batch) {
    if (seq < firstFileable()) return false;
    for (int id = 0; id < COUNT; ++id)
        if (batch.has[id]) filed[id][frameOf(seq, id)] = batch.val[id];
    return true;
}

void DelayedControls::unfile(uint32_t seq) {
    for (int id = 0; id < COUNT; ++id) filed[id].erase(frameOf(seq, id));
}

DelayedControls::Batch DelayedControls::planned(uint32_t seq) const {
    Batch out;
    for (int id = 0; id < COUNT; ++id) {
        out.has[id] = true;
        auto it = filed[id].upper_bound(frameOf(seq, id));
        if (it != filed[id].begin())
            out.val[id] = std::prev(it)->second;
        else if (!written[id].empty())
            out.val[id] = written[id].rbegin()->second;
        else
            out.val[id] = initial.val[id];
    }
    return out;
}

DelayedControls::Batch DelayedControls::tick(uint32_t seq, bool *changed) {
    Batch out;
    bool differs = false;
    for (int id = 0; id < COUNT; ++id) {
        const uint32_t target = seq + static_cast<uint32_t>(config.delay[id]);
        const int32_t last = written[id].empty()
                           ? initial.val[id] : written[id].rbegin()->second;

        /* The latest filed for the target or before it; anything filed
         * before that was overtaken. */
        int32_t value = last;
        auto it = filed[id].upper_bound(target);
        if (it != filed[id].begin()) {
            value = std::prev(it)->second;
            filed[id].erase(filed[id].begin(), it);
        }

        out.has[id] = true;
        out.val[id] = value;
        if (value != last) {
            differs = true;
            written[id][target] = value;
        }

    }
    ticked   = true;
    lastTick = seq;
    if (changed) *changed = differs;
    return out;
}

DelayedControls::Batch DelayedControls::inEffect(uint32_t seq) const {
    Batch out;
    for (int id = 0; id < COUNT; ++id) {
        out.has[id] = true;
        auto it = written[id].upper_bound(seq);
        out.val[id] = it == written[id].begin() ? initial.val[id]
                                                : std::prev(it)->second;
    }
    return out;
}

void DelayedControls::forgetBefore(uint32_t seq) {
    for (int id = 0; id < COUNT; ++id) {
        /* The value in effect on the earliest frame still asked about
         * stays. */
        auto from = written[id].upper_bound(frameOf(seq, id));
        if (from != written[id].begin())
            written[id].erase(written[id].begin(), std::prev(from));
    }
}

DelayedControls::Batch DelayedControls::inEffectFor(uint32_t seq) const {
    Batch out;
    for (int id = 0; id < COUNT; ++id) {
        out.has[id] = true;
        out.val[id] = inEffect(frameOf(seq, id)).val[id];
    }
    return out;
}

} /* namespace android */
