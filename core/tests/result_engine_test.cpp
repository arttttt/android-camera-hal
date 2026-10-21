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

/* Host test for the result engine on a fake ResultSink: frames go out
 * in the order they were submitted, whatever order their work ends in;
 * a frame that settles uncommitted goes out failed; buffers carry the
 * right fence and status; a stop hands back what it holds. Run by
 * core/tests/run_host_tests.sh. */

#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <span>
#include <thread>
#include <vector>

#include "Frame.h"
#include "FrameRegistry.h"
#include "ResultEngine.h"
#include "mochacamera/ports/ResultSink.h"
#include "sensor/DelayedControls.h"
#include "sensor/SensorConfig.h"
#include "stages/ResultDispatchStage.h"
#include "Check.h"

using namespace mocha::camera;
using namespace std::chrono_literals;

using mocha::camera::test::check;
using mocha::camera::test::finish;

namespace {

struct Delivered {
    uint32_t frameNumber;
    FrameOutcome outcome;
    std::vector<BufferReturn> buffers;
};

/* A sink that records the final results, and closes the fences it is
 * handed as a frontend would. */
class FakeSink : public ResultSink {
public:
    void shutter(uint32_t, int64_t) override {}
    void partial(uint32_t, PartialResult, const Core3AResult &) override {}
    void frame(uint32_t frameNumber, const CoreFrameResult &result,
               std::span<const BufferReturn> buffers) override {
        std::lock_guard<std::mutex> lock(mutex);
        delivered.push_back(Delivered{frameNumber, result.outcome,
                                      std::vector<BufferReturn>(buffers.begin(), buffers.end())});
        for (const BufferReturn &b : buffers)
            fences.push_back(b.releaseFence);
        changed.notify_all();
    }
    void deviceError(const char *) override {}

    bool waitFor(size_t n, std::chrono::milliseconds limit = 2000ms) {
        std::unique_lock<std::mutex> lock(mutex);
        return changed.wait_for(lock, limit, [&] { return delivered.size() >= n; });
    }
    std::vector<Delivered> snapshot() {
        std::lock_guard<std::mutex> lock(mutex);
        return delivered;
    }

    std::vector<int> fences;

private:
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<Delivered> delivered;
};

struct Rig {
    FakeSink                sink;
    android::SensorConfig   cfg = android::SensorConfig::imx179();
    android::DelayedControls dc{[] {
        android::DelayedControls::Config c{};
        c.delay[0] = c.delay[1] = 2;
        return c;
    }()};
    android::ResultDispatchStage dispatch{{&sink, nullptr, &cfg, &dc}};
    FrameRegistry registry{16};
    ResultEngine  engine{{&dispatch, &registry}};

    Rig() { check(engine.start()); }

    /* A frame accepted the way Session::submit accepts it; the token
     * that keeps it in the pipeline is the caller's. */
    FrameToken accept(uint32_t frameNumber, size_t outputs = 1) {
        auto f = std::make_shared<Frame>();
        f->frameNumber = frameNumber;
        f->outputs.resize(outputs);
        for (size_t i = 0; i < outputs; ++i) {
            f->outputs[i].stream.id = (StreamId)i;
            f->outputs[i].buffer = reinterpret_cast<NativeBuffer *>(0x1000 + i);
        }
        f->settled = engine.settledWake();
        FrameToken token(f);
        registry.add(f);
        check(engine.submit(f));
        return token;
    }
};

bool isOpen(int fd) { return ::fcntl(fd, F_GETFD) != -1; }

/* Work ending in reverse order still gives results in submission
 * order; committed frames complete. */
void testOrder() {
    Rig rig;
    std::vector<FrameToken> tokens;
    for (uint32_t n = 0; n < 5; ++n)
        tokens.push_back(rig.accept(n));
    for (int n = 4; n >= 0; --n) {
        tokens[n]->status.commit();
        tokens[n] = FrameToken();
        std::this_thread::sleep_for(2ms);
    }
    check(rig.sink.waitFor(5));
    const auto got = rig.sink.snapshot();
    check(got.size() == 5);
    for (uint32_t n = 0; n < got.size(); ++n) {
        check(got[n].frameNumber == n);
        check(got[n].outcome == FrameOutcome::Completed);
    }
    check(rig.registry.waitEmpty(1000));
}

/* A frame whose last token goes before it was committed did not
 * complete: it goes out failed, its buffers in error, with the acquire
 * fence it came with as their release fence. */
void testUncommittedFails() {
    Rig rig;
    int pipes[2];
    check(::pipe(pipes) == 0);
    {
        FrameToken token = rig.accept(7);
        token->outputs[0].acquireFence.reset(pipes[0]);
    }
    check(rig.sink.waitFor(1));
    const auto got = rig.sink.snapshot();
    check(got.size() == 1 && got[0].outcome == FrameOutcome::Failed);
    check(got[0].buffers[0].status == BufferStatus::Error);
    check(got[0].buffers[0].releaseFence == pipes[0]);
    ::close(pipes[0]);
    ::close(pipes[1]);
}

/* A committed frame with one failed output completes, that output in
 * error; a GPU release fence is handed over and the frame keeps none. */
void testOutputErrorAndFence() {
    Rig rig;
    int pipes[2];
    check(::pipe(pipes) == 0);
    {
        FrameToken token = rig.accept(3, 2);
        token->outputs[0].releaseFence = pipes[0];
        token->outputs[1].status = BufferStatus::Error;
        token->status.commit();
    }
    check(rig.sink.waitFor(1));
    const auto got = rig.sink.snapshot();
    check(got.size() == 1 && got[0].outcome == FrameOutcome::Completed);
    check(got[0].buffers[0].status == BufferStatus::Ok);
    check(got[0].buffers[0].releaseFence == pipes[0]);
    check(got[0].buffers[1].status == BufferStatus::Error);
    check(rig.registry.waitEmpty(1000));
    /* The frame is gone and did not close what it handed over. */
    check(isOpen(pipes[0]));
    ::close(pipes[0]);
    ::close(pipes[1]);
}

/* A frame still in work when the engine stops is handed back by the
 * caller -- once, failed. */
void testStopHandsBack() {
    Rig rig;
    FrameToken held = rig.accept(11);
    std::this_thread::sleep_for(10ms);
    rig.engine.stop();
    check(rig.sink.snapshot().empty());
    for (const FrameRef &f : rig.registry.takeAll())
        rig.engine.dispatchFailed(*f);
    const auto got = rig.sink.snapshot();
    check(got.size() == 1 && got[0].frameNumber == 11);
    check(got[0].outcome == FrameOutcome::Failed);
    held = FrameToken();
    check(rig.sink.snapshot().size() == 1);
}

/* Tokens dropped on other threads, many frames: every frame out once,
 * in order. */
void testManyThreads() {
    Rig rig;
    constexpr uint32_t kFrames = 200;
    std::vector<std::thread> workers;
    for (uint32_t n = 0; n < kFrames; ++n) {
        FrameToken token = rig.accept(n);
        if (n % 3) token->status.commit();
        workers.emplace_back([t = std::move(token), n]() mutable {
            std::this_thread::sleep_for(std::chrono::microseconds((n * 37) % 500));
            t = FrameToken();
        });
    }
    for (std::thread &w : workers) w.join();
    check(rig.sink.waitFor(kFrames, 5000ms));
    const auto got = rig.sink.snapshot();
    check(got.size() == kFrames);
    for (uint32_t n = 0; n < got.size(); ++n) {
        check(got[n].frameNumber == n);
        check(got[n].outcome == ((n % 3) ? FrameOutcome::Completed : FrameOutcome::Failed));
    }
}

} // namespace

int main() {
    testOrder();
    testUncommittedFails();
    testOutputErrorAndFence();
    testStopHandsBack();
    testManyThreads();
    return finish("result_engine_test");
}
