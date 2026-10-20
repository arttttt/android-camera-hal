#ifndef UTIL_BENCHMARK_H
#define UTIL_BENCHMARK_H

#include <cstdio>
#include <cstring>
#include <stdint.h>
#include <time.h>

#ifndef NDEBUG
# define NDEBUG 0
#endif

#define BENCHMARK_VARIABLE_NAME _benchmarkState
#define BENCHMARK_CLASS_WITH_NS android::DbgUtils::Benchmark

#ifdef HAVE_ANDROID_OS
# include <utils/Vector.h>
# define BENCHMARK_VECTOR android::Vector
/* Android vector's operator [] is const */
# define BENCHMARK_VECTOR_ITEM_EDIT(vec, id) (vec).editItemAt(id)
#else
# include <vector>
# define BENCHMARK_VECTOR std::vector
# define BENCHMARK_VECTOR_ITEM_EDIT(vec, id) (vec)[id]
#endif

namespace android {
namespace DbgUtils {

/* Named-section stopwatch: every BENCHMARK_SECTION("name") block logs
 * per-cycle + SAMPLES-window-average wall time into a vector of
 * Section entries. formatString() serialises the current state. */
template <int SAMPLES>
class Benchmark {
public:
    Benchmark() {}
    ~Benchmark() {}

    int begin(const char *sectionName) {
        int id = mSections.size();
        for(int i = 0; i < mSections.size(); ++i) {
            if(!strcmp(mSections[i].name, sectionName)) {
                id = i;
                break;
            }
        }
        if(id == mSections.size()) {
            Section newSection;
            newSection.name = sectionName;
            for(unsigned i = 0; i < SAMPLES; ++i) {
                newSection.time[i] = 0;
            }
            newSection.timeId = SAMPLES - 1;
            newSection.samplesCount = 0;
            newSection.count = 0;
            mSections.push_back(newSection);
        }
        Section &sec = BENCHMARK_VECTOR_ITEM_EDIT(mSections, id);
        if(sec.count == 0) {
            sec.timeId = (sec.timeId + 1) % SAMPLES;
            sec.time[sec.timeId] = 0;
            if(sec.samplesCount < SAMPLES)
                ++sec.samplesCount;
        }
        ++sec.count;
        sec.time[sec.timeId] -= currentTimeNs();
        return id;
    }

    void end(int id) {
        Section &sec = BENCHMARK_VECTOR_ITEM_EDIT(mSections, id);
        sec.time[sec.timeId] += currentTimeNs();
    }

    bool formatString(char *out, size_t len, int precision) {
        for(int i = 0; i < mSections.size() && len > 0; ++i) {
            Section &sec = BENCHMARK_VECTOR_ITEM_EDIT(mSections, i);
            double t = (double)sec.time[sec.timeId] / 1000000000.0f;
            if(sec.count == 0)
                t = 0.0f;
            double avg = 0.0f;
            for(unsigned j = 0; j < sec.samplesCount; ++j) {
                const unsigned jj = (SAMPLES + sec.timeId - j) % SAMPLES;
                avg += (double)sec.time[jj];
            }
            avg = avg / sec.samplesCount / 1000000000.0f;
            size_t printedNum;
            printedNum = snprintf(out, len, "%s%s[%u]: %.*f (%.*f)",
                                  i != 0 ? "  " : "",
                                  sec.name, sec.count,
                                  precision, t, precision, avg);
            len -= printedNum;
            out += printedNum;
        }
        return (len > 0);
    }

    void newCycle() {
        for(int i = 0; i < mSections.size(); ++i) {
            Section &sec = BENCHMARK_VECTOR_ITEM_EDIT(mSections, i);
            sec.count = 0;
        }
    }

private:
    int64_t currentTimeNs() {
        struct timespec t;
        clock_gettime(CLOCK_MONOTONIC, &t);
        return t.tv_sec * 1000000000LL + t.tv_nsec;
    }

    struct Section {
        const char *name;
        int64_t     time[SAMPLES];
        unsigned    timeId;
        unsigned    samplesCount;
        unsigned    count;
    };
    BENCHMARK_VECTOR<Section> mSections;
};

}; /* namespace DbgUtils */
}; /* namespace android */

#if !NDEBUG
# define BENCHMARK_HERE(samples) \
    static BENCHMARK_CLASS_WITH_NS<samples> BENCHMARK_VARIABLE_NAME; \
    BENCHMARK_VARIABLE_NAME.newCycle();

# define BENCHMARK_SECTION(name) \
    for(int _benchmarkId = BENCHMARK_VARIABLE_NAME.begin(name); \
        _benchmarkId >= 0; \
        BENCHMARK_VARIABLE_NAME.end(_benchmarkId), _benchmarkId = -1)

# define BENCHMARK_STRING(str, len, prec) \
    BENCHMARK_VARIABLE_NAME.formatString(str, len, prec);
#else
# define BENCHMARK_HERE(samples)
# define BENCHMARK_SECTION(name) if(true)
# define BENCHMARK_STRING(str, len, prec) (*str = '\0')
#endif

#undef BENCHMARK_VECTOR

#endif /* UTIL_BENCHMARK_H */
