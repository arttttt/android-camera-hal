#ifndef UTIL_FPS_COUNTER_H
#define UTIL_FPS_COUNTER_H

#include <stddef.h>
#include <utils/Timers.h>

#ifndef NDEBUG
# define NDEBUG 0
#endif

#define FPSCOUNTER_VARIABLE_NAME _fpsCounterState
#define FPSCOUNTER_CLASS_WITH_NS android::DbgUtils::FpsCounter

namespace android {
namespace DbgUtils {

/* Sliding-window FPS sampler — records the timestamp of the last
 * SAMPLES ticks and computes the average rate over any sub-window. */
template<int SAMPLES>
class FpsCounter {
public:
    FpsCounter(): mTimeId(SAMPLES - 1), mSamplesCount(0) {
        for(size_t i = 0; i < SAMPLES; ++i)
            mTime[i] = 0;
    }

    double fps(int samples = SAMPLES - 1) {
        if(samples >= mSamplesCount)
            samples = mSamplesCount - 1;
        if(samples < 1)
            samples = 1;

        unsigned pastTime;
        pastTime = (mTimeId + SAMPLES - samples) % SAMPLES;

        return samples * 1000000000.0f / (mTime[mTimeId] - mTime[pastTime]);
    }

    void tick() {
        mTimeId = (mTimeId + 1) % SAMPLES;
        mTime[mTimeId] = systemTime();
        if(mSamplesCount < SAMPLES)
            ++mSamplesCount;
    }

    nsecs_t mTime[SAMPLES];
    unsigned mTimeId;
    unsigned mSamplesCount;
};

}; /* namespace DbgUtils */
}; /* namespace android */

#if !NDEBUG
# define FPSCOUNTER_HERE(samples) \
    static FPSCOUNTER_CLASS_WITH_NS<samples> FPSCOUNTER_VARIABLE_NAME; \
    FPSCOUNTER_VARIABLE_NAME.tick();

# define FPSCOUNTER_VALUE(samples) \
    (FPSCOUNTER_VARIABLE_NAME.fps(samples))

#else
# define FPSCOUNTER_HERE(samples)
# define FPSCOUNTER_VALUE(samples) (0.0f)
#endif

#endif /* UTIL_FPS_COUNTER_H */
