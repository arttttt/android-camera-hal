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

#include "ResultEngine.h"

#include <chrono>

#include "FrameRegistry.h"
#include "stages/ResultDispatchStage.h"

namespace mocha::camera {

ResultEngine::ResultEngine(const Deps &d) : deps(d) {}

ResultEngine::~ResultEngine() {
    stop();
}

bool ResultEngine::start() {
    if (engine)
        return false;
    /* No stop grace: a frame still waiting when the engine stops is the
     * caller's to hand back (dispatchFailed). */
    engine = std::make_unique<Eng>(
            "CamResult",
            [this](Eng::Id, FrameRef &frame) { return handle(frame); },
            EngineOptions{.maxRunning = 1, .stopGrace = std::chrono::milliseconds(0)});
    return engine->start();
}

void ResultEngine::stop() {
    if (!engine)
        return;
    engine->stop();
    engine.reset();
}

bool ResultEngine::submit(FrameRef frame) {
    return engine && engine->submit(std::move(frame)).has_value();
}

Engine<FrameRef, std::monostate>::Step ResultEngine::handle(FrameRef &frame) {
    return dispatchWhenSettled(frame);
}

Engine<FrameRef, std::monostate>::Step
ResultEngine::dispatchWhenSettled(FrameRef &frame) {
    /* Drained before the count is read: a token going after this
     * point wakes the next poll. */
    settled.drain();
    if (frame->inPipeline.load(std::memory_order_acquire) > 0) {
        return Eng::Step::wait(settled.fd(), [this, &frame] {
            return dispatchWhenSettled(frame);
        });
    }
    /* Not committed means not completed; a committed or failed frame
     * keeps its outcome. */
    frame->status.cancel();
    deps.dispatch->process(*frame);
    deps.registry->remove(frame->frameNumber);
    return Eng::Step::done();
}

void ResultEngine::dispatchFailed(Frame &frame) {
    frame.status.cancel();
    deps.dispatch->process(frame);
}

} /* namespace mocha::camera */
