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

#ifndef CORE_ENGINES_FINISH_ENGINE_H
#define CORE_ENGINES_FINISH_ENGINE_H

#include <memory>
#include <variant>

#include "Engine.h"
#include "Frame.h"

namespace android {
class BufferProcessor;
class StatsProcessStage;
}

namespace mocha::camera {

class JpegEngine;

/* The CPU half of a frame, once its GPU work is done -- what used to be
 * completeCtx on the GPU's thread, now a thread of its own, so the
 * demosaic engine waits on nothing but fences.
 *
 * In order: commit the frame (from here on it completes normally, and
 * no ERROR_REQUEST may follow its partial results); finish the YUV
 * outputs on the CPU; run 3A on
 * the latest statistics, which emits the partial results and owns AF;
 * hand each BLOB output to the JPEG engine. A frame cancelled or failed
 * before the commit skips the work and only lets its JPEG snapshots
 * go. */
class FinishEngine {
public:
    struct Deps {
        android::BufferProcessor   *bufferProcessor;
        android::StatsProcessStage *statsProcess;   /* may be null */
        JpegEngine                 *jpeg;
    };

    explicit FinishEngine(const Deps &deps);
    ~FinishEngine();

    FinishEngine(const FinishEngine &) = delete;
    FinishEngine &operator=(const FinishEngine &) = delete;

    bool start();
    void stop();

    /* False once stopped; the token goes, and the frame with it to its
     * result. */
    bool submit(FrameToken frame);

private:
    using Eng = Engine<FrameToken, std::monostate>;

    void finish(FrameToken &frame);

    Deps deps;
    std::unique_ptr<Eng> engine;
};

} /* namespace mocha::camera */

#endif /* CORE_ENGINES_FINISH_ENGINE_H */
