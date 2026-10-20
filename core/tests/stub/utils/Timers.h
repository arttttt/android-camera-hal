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

/* The host tests' stand-in for libutils' Timers.h. */

#ifndef CORE_TESTS_STUB_UTILS_TIMERS_H
#define CORE_TESTS_STUB_UTILS_TIMERS_H

#include <stdint.h>
#include <time.h>

typedef int64_t nsecs_t;

static inline nsecs_t systemTime() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (nsecs_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

#endif /* CORE_TESTS_STUB_UTILS_TIMERS_H */
