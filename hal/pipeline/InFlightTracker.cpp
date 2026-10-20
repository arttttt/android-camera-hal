#include "InFlightTracker.h"

#include <utility>

namespace android {

void InFlightTracker::add(std::unique_ptr<PipelineContext> context) {
    if (!context) return;
    uint32_t seq = context->sequence;
    std::lock_guard<std::mutex> lock(mutex);
    contexts[seq] = std::move(context);
}

std::unique_ptr<PipelineContext> InFlightTracker::removeBySequence(uint32_t seq) {
    std::lock_guard<std::mutex> lock(mutex);
    auto it = contexts.find(seq);
    if (it == contexts.end()) return nullptr;
    std::unique_ptr<PipelineContext> owned = std::move(it->second);
    contexts.erase(it);
    return owned;
}

std::vector<std::unique_ptr<PipelineContext>> InFlightTracker::drainAll() {
    std::vector<std::unique_ptr<PipelineContext>> out;
    std::lock_guard<std::mutex> lock(mutex);
    out.reserve(contexts.size());
    for (auto &entry : contexts) {
        out.push_back(std::move(entry.second));
    }
    contexts.clear();
    return out;
}

void InFlightTracker::cancelIf(
        const std::function<bool(const PipelineContext &)> &which) {
    std::lock_guard<std::mutex> lock(mutex);
    for (auto &entry : contexts) {
        if (entry.second && which(*entry.second)) entry.second->status.cancel();
    }
}

void InFlightTracker::cancelAll() {
    cancelIf([](const PipelineContext &) { return true; });
}

size_t InFlightTracker::count() const {
    std::lock_guard<std::mutex> lock(mutex);
    return contexts.size();
}

} /* namespace android */
