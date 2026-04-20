#ifndef HAL_PIPELINE_IN_FLIGHT_TRACKER_H
#define HAL_PIPELINE_IN_FLIGHT_TRACKER_H

#include <stddef.h>
#include <stdint.h>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include "PipelineContext.h"

namespace android {

/* Thread-safe registry of PipelineContexts currently in flight.
 *
 * Single-owner semantics: Camera adds a context on processCaptureRequest
 * (binder thread); whichever stage finishes the frame removes it by
 * sequence; drainAll() is used on flush/close to error-complete the
 * remainder. Removing returns the unique_ptr so the context's
 * destructor runs deterministically on the caller's thread. */
class InFlightTracker {
public:
    InFlightTracker() = default;

    InFlightTracker(const InFlightTracker&) = delete;
    InFlightTracker& operator=(const InFlightTracker&) = delete;

    void add(std::unique_ptr<PipelineContext> context);

    std::unique_ptr<PipelineContext> removeBySequence(uint32_t sequence);

    std::vector<std::unique_ptr<PipelineContext>> drainAll();

    size_t count() const;

private:
    mutable std::mutex mutex;
    std::map<uint32_t, std::unique_ptr<PipelineContext>> contexts;
};

} /* namespace android */

#endif /* HAL_PIPELINE_IN_FLIGHT_TRACKER_H */
