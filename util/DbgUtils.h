/*
 * Copyright (C) 2015-2016 Antmicro
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

#ifndef DBGUTILS_H
#define DBGUTILS_H

/* Debug utilities bundle. Pulls in the three standalone helpers so
 * existing call sites keep a single include. Prefer the individual
 * headers (AutoLogCall.h / FpsCounter.h / Benchmark.h) for new code
 * that only uses one. */

#include "AutoLogCall.h"
#include "FpsCounter.h"
#include "Benchmark.h"

#ifndef NDEBUG
# define NDEBUG 0
#endif

#if !NDEBUG
# define DEBUG_CODE(x) x
#else
# define DEBUG_CODE(x)
#endif

#endif /* DBGUTILS_H */
