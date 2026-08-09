#ifndef UTIL_OPTIONAL_H
#define UTIL_OPTIONAL_H

#include <utility>

namespace android {

/* A minimal stand-in for std::optional.
 *
 * The HAL used std::experimental::optional until the platform dropped it:
 * the header still exists, but its entire body is
 *   #error "<experimental/optional> has been removed. Use <optional> instead."
 * The standard <optional> is not a portable replacement here. The oldest
 * platform branch we build against ships no such header at all, and where
 * it does exist libcxx gates the contents behind `_LIBCPP_STD_VER > 14`, so
 * reaching it means compiling this module as C++17 against some branches
 * and C++14 against others -- a fork in the build flags and a second one in
 * the includes, for a type used in eight places. This header is the third
 * option: same surface, no forks, one language standard everywhere.
 *
 * Deliberately simpler than the standard type. The value is stored inline
 * and always constructed, so `T` must be default constructible; every type
 * we hold (LuxAnchor, BayesParams, WbGains, CcmQ10, int32_t) is a plain
 * aggregate, and the alternative -- aligned storage with placement new --
 * would buy nothing here but lifetime bugs. Empty state costs one default
 * construction, which for these types is a handful of zeroed floats.
 *
 * Not provided, because nothing needs them: in-place construction,
 * comparison operators, value_or, nullopt. Add them if a caller does.
 */
template <typename T>
class Optional {
public:
    Optional() = default;
    Optional(const T &v): mValue(v), mHasValue(true) {}
    Optional(T &&v): mValue(std::move(v)), mHasValue(true) {}

    /* Contextual conversion only: `if (opt)` and `opt && x` work, while
     * an accidental `int n = opt;` does not compile. */
    explicit operator bool() const { return mHasValue; }

    const T &operator*()  const { return mValue; }
    T       &operator*()        { return mValue; }
    const T *operator->() const { return &mValue; }
    T       *operator->()       { return &mValue; }

    void reset() { mValue = T(); mHasValue = false; }

private:
    T    mValue    = T();
    bool mHasValue = false;
};

}  // namespace android

#endif  // UTIL_OPTIONAL_H
