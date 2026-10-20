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

/* Host test for PermitPool. Linux (the build server), under
 * ThreadSanitizer:
 *
 *   clang++ -std=c++20 -Wall -Wextra -fsanitize=thread -g -pthread \
 *       -Icore/engine -Ibase -Icore/engine/tests/stub \
 *       core/engine/tests/permit_pool_test.cpp base/EventFd.cpp \
 *       -o out/host-tests/permit_pool_test && out/host-tests/permit_pool_test
 */

#include <poll.h>

#include <atomic>
#include <cstdio>
#include <thread>
#include <utility>
#include <vector>

#include "EventFd.h"
#include "PermitPool.h"

using namespace mocha::camera;
using android::EventFd;

namespace {

int failures = 0;

#define CHECK(cond)                                                \
    do {                                                           \
        if (!(cond)) {                                             \
            std::printf("%s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++;                                            \
        }                                                          \
    } while (0)

bool readable(int fd) {
    pollfd p{fd, POLLIN, 0};
    return ::poll(&p, 1, 0) == 1;
}

/* No more permits out than the pool holds; one given back -- by any
 * path, a move included -- wakes the waiter and can be taken again. */
void testBound() {
    EventFd wake;
    PermitPool pool(2, &wake);
    auto a = pool.tryAcquire();
    auto b = pool.tryAcquire();
    CHECK(a && b);
    CHECK(!pool.tryAcquire());
    CHECK(pool.free() == 0);
    CHECK(!readable(wake.fd()));

    Permit moved = std::move(*a);
    CHECK(!*a);
    CHECK(pool.free() == 0);
    a.reset();
    CHECK(pool.free() == 0);

    moved = Permit();
    CHECK(pool.free() == 1);
    CHECK(readable(wake.fd()));
    wake.drain();
    CHECK(pool.tryAcquire().has_value());   /* taken and given back */
    CHECK(pool.free() == 1);
    b.reset();
    CHECK(pool.free() == 2);
}

/* Many threads taking and giving back: never more out than the pool
 * holds, and all come back. */
void testRace() {
    EventFd wake;
    PermitPool pool(3, &wake);
    std::atomic<int> out{0};
    std::atomic<int> most{0};
    std::vector<std::thread> threads;
    for (int t = 0; t < 6; ++t) {
        threads.emplace_back([&] {
            for (int i = 0; i < 5000; ++i) {
                auto p = pool.tryAcquire();
                if (!p)
                    continue;
                const int now = ++out;
                int seen = most.load();
                while (now > seen && !most.compare_exchange_weak(seen, now)) {
                }
                --out;
            }
        });
    }
    for (std::thread &t : threads)
        t.join();
    CHECK(most.load() <= 3);
    CHECK(pool.free() == 3);
}

} // namespace

int main() {
    testBound();
    testRace();
    if (failures == 0)
        std::printf("permit_pool_test: all passed\n");
    return failures == 0 ? 0 : 1;
}
