/*
 * Copyright (C) 2026 Artem Bambalov
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CORE_TESTS_CHECK_H
#define CORE_TESTS_CHECK_H

#include <atomic>
#include <cstdio>
#include <source_location>

/* What the host tests assert with. A failed check prints where it is
 * and the test goes on, so one run reports every failure; checks may
 * run on any thread (an engine's, say). */
namespace mocha::camera::test {

inline std::atomic<int> failures{0};

inline void check(bool ok,
                  std::source_location where = std::source_location::current()) {
    if (ok)
        return;
    std::printf("%s:%u: check failed\n", where.file_name(), (unsigned)where.line());
    failures.fetch_add(1, std::memory_order_relaxed);
}

/* main's result: 0 when every check held. */
inline int finish(const char *test) {
    const int failed = failures.load();
    if (failed == 0)
        std::printf("%s: all passed\n", test);
    return failed == 0 ? 0 : 1;
}

} /* namespace mocha::camera::test */

#endif /* CORE_TESTS_CHECK_H */
