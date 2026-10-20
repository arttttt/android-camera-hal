#include "InFlightTracker.h"

#include <chrono>
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
    if (contexts.empty()) emptied.notify_all();
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
    emptied.notify_all();
    return out;
}

void InFlightTracker::cancelIf(
        const std::function<bool(const PipelineContext &)> &which) {
    std::lock_guard<std::mutex> lock(mutex);
    for (auto &entry : contexts) {
        if (entry.second && which(*entry.second)) entry.second->status.cancel();
    }
}

bool InFlightTracker::waitEmpty(int timeoutMs) {
    std::unique_lock<std::mutex> lock(mutex);
    return emptied.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                            [this] { return contexts.empty(); });
}

size_t InFlightTracker::count() const {
    std::lock_guard<std::mutex> lock(mutex);
    return contexts.size();
}

} /* namespace android */
