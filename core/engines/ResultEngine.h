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

#ifndef CORE_ENGINES_RESULT_ENGINE_H
#define CORE_ENGINES_RESULT_ENGINE_H

#include <memory>
#include <variant>

#include "Engine.h"
#include "EventFd.h"
#include "Frame.h"

namespace android { class ResultDispatchStage; }

namespace mocha::camera {

class FrameRegistry;

/* Hands frames back to the framework, strictly in the order they were
 * accepted.
 *
 * A frame is queued here the moment it is accepted, before any other
 * engine sees it. The engine takes one frame at a time (maxRunning 1)
 * and waits until no job holds a token on it any more -- its JPEGs
 * encoded, or its work given up on any path -- then hands its result
 * back and lets the registry forget it. The next frame is not looked at
 * before; that keeps the results in order.
 *
 * A frame that settles without having been committed did not complete
 * (cancelled while waiting for its Bayer, dropped by a stopping engine,
 * its GPU work never done): it is cancelled here and goes out as a
 * failed request. */
class ResultEngine {
public:
    struct Deps {
        android::ResultDispatchStage *dispatch;
        FrameRegistry                *registry;
    };

    explicit ResultEngine(const Deps &deps);
    ~ResultEngine();

    ResultEngine(const ResultEngine &) = delete;
    ResultEngine &operator=(const ResultEngine &) = delete;

    bool start();
    void stop();

    /* False once stopped. */
    bool submit(FrameRef frame);

    /* What a frame's tokens notify when the last of them goes. Lives as
     * long as the engine component, across restarts. */
    android::EventFd *settledWake() { return &settled; }

    /* For close: a frame left over once the engine is stopped goes out
     * as a failed request, from the caller's thread. */
    void dispatchFailed(Frame &frame);

private:
    using Eng = Engine<FrameRef, std::monostate>;

    Eng::Step handle(FrameRef &frame);
    Eng::Step dispatchWhenSettled(FrameRef &frame);

    Deps deps;
    android::EventFd settled;
    std::unique_ptr<Eng> engine;
};

} /* namespace mocha::camera */

#endif /* CORE_ENGINES_RESULT_ENGINE_H */
