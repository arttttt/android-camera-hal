#ifndef HAL_PIPELINE_FRAME_STATUS_H
#define HAL_PIPELINE_FRAME_STATUS_H

#include <atomic>
#include <stdint.h>

namespace android {

/* How a frame ends, decided once.
 *
 * A frame starts Ok and leaves it at most once: a stage that cannot
 * produce it fails it with the reason, a cancellation (flush, the AE
 * mode switch, stop) cancels it. Whichever comes first is the outcome;
 * the later one is refused, so a cancellation arriving from the binder
 * thread never overwrites a stage's failure and a stage never turns a
 * cancelled frame back into a failed one.
 *
 * Written by any thread, read by any thread without a lock: the whole
 * state is one word, settled by compare-exchange. The words are the
 * pipeline's own; which Camera3 message a failed or cancelled frame
 * becomes is the frontend's business (ResultDispatchStage). */
class FrameStatus {
public:
    enum class Kind : uint32_t {
        Ok        = 0,
        Failed    = 1,
        Cancelled = 2,
    };

    FrameStatus() : word(0) {}

    FrameStatus(const FrameStatus&) = delete;
    FrameStatus& operator=(const FrameStatus&) = delete;

    /* True if this call decided the outcome. */
    bool fail(int32_t reason) { return settle(Kind::Failed, reason); }
    bool cancel()             { return settle(Kind::Cancelled, 0); }

    bool ok() const { return kind() == Kind::Ok; }

    Kind kind() const {
        return (Kind)(word.load(std::memory_order_acquire) >> 32);
    }

    /* The failing stage's status_t; 0 unless the frame failed. */
    int32_t reason() const {
        return (int32_t)(uint32_t)word.load(std::memory_order_acquire);
    }

private:
    bool settle(Kind k, int32_t reason) {
        uint64_t expected = 0;
        const uint64_t desired = ((uint64_t)k << 32) | (uint32_t)reason;
        return word.compare_exchange_strong(expected, desired,
                                            std::memory_order_acq_rel,
                                            std::memory_order_acquire);
    }

    std::atomic<uint64_t> word;
};

} /* namespace android */

#endif /* HAL_PIPELINE_FRAME_STATUS_H */
