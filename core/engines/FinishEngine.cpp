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

#include "FinishEngine.h"

#include <utils/Log.h>

#include "BufferProcessor.h"
#include "JpegEngine.h"
#include "stages/StatsProcessStage.h"

#undef  LOG_TAG
#define LOG_TAG "Cam-FinishEngine"

namespace mocha::camera {

FinishEngine::FinishEngine(const Deps &d) : deps(d) {}

FinishEngine::~FinishEngine() {
    stop();
}

bool FinishEngine::start() {
    if (engine)
        return false;
    engine = std::make_unique<Eng>(
            "CamFinish",
            [this](Eng::Id, FrameToken &frame) {
                finish(frame);
                return Eng::Step::done();
            });
    return engine->start();
}

void FinishEngine::stop() {
    if (!engine)
        return;
    engine->stop();
    engine.reset();
}

bool FinishEngine::submit(FrameToken frame) {
    return engine && engine->submit(std::move(frame)).has_value();
}

void FinishEngine::finish(FrameToken &token) {
    Frame &f = *token;

    /* The GPU is done with the frame: commit it before anything of its
     * result goes out. A cancellation from here on is refused and the
     * frame completes normally; one that came first keeps it on the
     * error path. */
    f.status.commit();

    if (!f.status.ok()) {
        deps.bufferProcessor->releaseJpegSnapshots(f);
        return;
    }

    /* YUV outputs repacked on the CPU; BLOBs go to the JPEG engine. */
    deps.bufferProcessor->finalizeCpuOutputs(f);

    /* 3A on the latest statistics; emits the partial results, after the
     * commit above. */
    if (deps.statsProcess)
        deps.statsProcess->process(f);

    for (size_t i = 0; i < f.outputs.size(); ++i) {
        if (f.outputs[i].stream.format != PixelFormat::Blob ||
            f.outputs[i].jpeg.ringSlot < 0)
            continue;
        /* A job the engine refuses gives its output up as it goes. */
        if (!deps.jpeg->submit(JpegEngine::Job(token.share(), i, deps.bufferProcessor)))
            ALOGE("frame %u output %zu: the JPEG engine is stopped",
                  f.frameNumber, i);
    }
}

} /* namespace mocha::camera */
