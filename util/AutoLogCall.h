#ifndef UTIL_AUTO_LOG_CALL_H
#define UTIL_AUTO_LOG_CALL_H

#include <utils/Log.h>

#ifndef NDEBUG
# define NDEBUG 0
#endif

namespace android {
namespace DbgUtils {

/* RAII call-site logger: emits a "+ name" ALOGV on construction and a
 * "- name" ALOGV on destruction, indented by thread-local call depth.
 * Used via the DBGUTILS_AUTOLOGCALL() macro to trace entry/exit of
 * HAL callbacks. */
class AutoLogCall {
public:
    AutoLogCall(const char *name): mName(name) {
        static __thread unsigned level;
        mLevel = &level;
        ALOGV("%*s+ %s", *mLevel * 4, "", name);
        ++(*mLevel);
    }
    ~AutoLogCall() {
        --(*mLevel);
        ALOGV("%*s- %s", *mLevel * 4, "", mName);
    }

private:
    const char *mName;
    unsigned *mLevel;
};

}; /* namespace DbgUtils */
}; /* namespace android */

#if !NDEBUG
# define DBGUTILS_AUTOLOGCALL(name) android::DbgUtils::AutoLogCall _autoLogCall(name)
#else
# define DBGUTILS_AUTOLOGCALL(name)
#endif

#endif /* UTIL_AUTO_LOG_CALL_H */
