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

/* The host tests' stand-in for liblog: what base/ logs goes to stderr. */

#ifndef CORE_ENGINE_TESTS_STUB_UTILS_LOG_H
#define CORE_ENGINE_TESTS_STUB_UTILS_LOG_H

#include <cstdio>

#define ALOG_HOST(level, ...) \
    (std::fprintf(stderr, level " " __VA_ARGS__), std::fputc('\n', stderr))
#define ALOGE(...) ALOG_HOST("E", __VA_ARGS__)
#define ALOGW(...) ALOG_HOST("W", __VA_ARGS__)
#define ALOGI(...) ALOG_HOST("I", __VA_ARGS__)
#define ALOGD(...) ((void)0)
#define ALOGV(...) ((void)0)

#endif /* CORE_ENGINE_TESTS_STUB_UTILS_LOG_H */
