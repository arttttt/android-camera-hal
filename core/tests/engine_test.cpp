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

/* Host test for the Engine primitive. Linux (the build server), under
 * ThreadSanitizer:
 *
 *   clang++ -std=c++20 -Wall -Wextra -fsanitize=thread -g -pthread \
 *       -Icore/engine -Icore/base -Icore/tests/stub \
 *       core/tests/engine_test.cpp core/base/EventFd.cpp \
 *       -o out/host-tests/engine_test && out/host-tests/engine_test
 */

#include <poll.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "Engine.h"
#include "EventFd.h"
#include "Check.h"

using namespace mocha::camera;
using android::EventFd;
using namespace std::chrono_literals;

using mocha::camera::test::check;
using mocha::camera::test::finish;

namespace {

/* A job that says when it is let go, as a job holding a frame would let
 * the frame go. */
struct Job {
    int n;
    std::shared_ptr<int> held;
};

using Eng = Engine<Job, int>;
using State = Eng::State;
using Step = Eng::Step;

bool waitFor(const std::function<bool()> &pred,
             std::chrono::milliseconds limit = 2000ms) {
    const auto deadline = std::chrono::steady_clock::now() + limit;
    while (!pred()) {
        if (std::chrono::steady_clock::now() > deadline)
            return false;
        std::this_thread::sleep_for(1ms);
    }
    return true;
}

Eng::Options keep(size_t maxRunning = 1) {
    return Eng::Options{.maxRunning = maxRunning, .keepOutcomes = true};
}

/* A synchronous handler: done or refused, collected in order. */
void testSynchronous() {
    Eng e("sync", [](Eng::Id, Job &j) {
        return j.n < 0 ? Step::refused(j.n) : Step::done(j.n * 10);
    }, keep());
    check(e.start());
    const auto a = e.submit(Job{1, nullptr});
    const auto b = e.submit(Job{-2, nullptr});
    check(a && b);
    auto out = e.resolve(*a, true);
    check(out.state == State::Done && out.result && *out.result == 10);
    out = e.resolve(*b, true);
    check(out.state == State::Refused && out.result && *out.result == -2);
    e.stop();
}

/* An asynchronous handler: the job waits on its fence, the continuation
 * finishes it -- and may wait again first. */
void testAsynchronousAndRearm() {
    EventFd first, second;
    std::atomic<int> continuations{0};
    std::thread::id engineThread;
    Eng e("async", [&](Eng::Id, Job &j) {
        engineThread = std::this_thread::get_id();
        return Step::wait(first.fd(), [&, &job = j]() {
            continuations++;
            first.drain();
            return Step::wait(second.fd(), [&]() {
                continuations++;
                check(std::this_thread::get_id() == engineThread);
                return Step::done(job.n + 100);
            });
        });
    }, keep());
    check(e.start());
    const auto id = e.submit(Job{7, nullptr});
    Eng::Outcome out{};
    std::this_thread::sleep_for(20ms);
    check(!e.tryResolve(*id, &out));
    first.notify();
    check(waitFor([&] { return continuations.load() == 1; }));
    check(!e.tryResolve(*id, &out));
    second.notify();
    out = e.resolve(*id, true);
    check(out.state == State::Done && out.result && *out.result == 107);
    check(continuations.load() == 2);
    e.stop();
}

/* No more than maxRunning jobs taken at once; a finished one makes room. */
void testMaxRunning() {
    std::vector<std::unique_ptr<EventFd>> fences;
    for (int i = 0; i < 3; ++i)
        fences.push_back(std::make_unique<EventFd>());
    std::atomic<int> started{0};
    Eng e("ring", [&](Eng::Id, Job &j) {
        started++;
        const int n = j.n;
        return Step::wait(fences[n]->fd(), [n] { return Step::done(n); });
    }, keep(2));
    check(e.start());
    const auto a = e.submit(Job{0, nullptr});
    e.submit(Job{1, nullptr});
    const auto c = e.submit(Job{2, nullptr});
    check(waitFor([&] { return started.load() == 2; }));
    std::this_thread::sleep_for(30ms);
    check(started.load() == 2);
    fences[0]->notify();
    check(e.resolve(*a, true).state == State::Done);
    check(waitFor([&] { return started.load() == 3; }));
    fences[1]->notify();
    fences[2]->notify();
    check(e.resolve(*c, true).state == State::Done);
    e.stop();
}

/* Cancelled while queued: never run. While waiting: let go on the
 * engine's thread, its continuation never runs, its room is freed.
 * While running: runs to its end, outcome not kept. */
void testCancel() {
    EventFd fence;
    std::atomic<int> handled{0};
    std::atomic<int> continued{0};
    std::mutex gateMutex;
    std::condition_variable gateCv;
    bool gateOpen = false;
    std::atomic<bool> inGate{false};
    Eng e("cancel", [&](Eng::Id, Job &j) {
        handled++;
        if (j.n == 3) {
            inGate = true;
            std::unique_lock<std::mutex> lock(gateMutex);
            gateCv.wait(lock, [&] { return gateOpen; });
            return Step::done(3);
        }
        return Step::wait(fence.fd(), [&] {
            continued++;
            return Step::done(0);
        });
    }, keep());
    check(e.start());

    auto waitingHeld = std::make_shared<int>(0);
    auto queuedHeld = std::make_shared<int>(0);
    const auto waitingId = e.submit(Job{1, waitingHeld});
    check(waitFor([&] { return handled.load() == 1; }));
    const auto queuedId = e.submit(Job{2, queuedHeld});

    e.cancel(*queuedId);
    check(queuedHeld.use_count() == 1);
    e.cancel(*waitingId);
    check(waitFor([&] { return waitingHeld.use_count() == 1; }));
    fence.notify();
    std::this_thread::sleep_for(20ms);
    check(continued.load() == 0);
    check(handled.load() == 1);
    Eng::Outcome out{};
    check(e.tryResolve(*waitingId, &out) && out.state == State::Cancelled);
    check(e.tryResolve(*queuedId, &out) && out.state == State::Cancelled);

    /* The room is free: a running job can be taken, and cancelled. */
    const auto runningId = e.submit(Job{3, nullptr});
    check(waitFor([&] { return inGate.load(); }));
    e.cancel(*runningId);
    {
        std::lock_guard<std::mutex> lock(gateMutex);
        gateOpen = true;
    }
    gateCv.notify_all();
    check(e.resolve(*runningId, true).state == State::Cancelled);
    e.stop();
}

/* LatestOnly: while the engine is busy, only the newest job survives. */
void testLatestOnly() {
    std::mutex gateMutex;
    std::condition_variable gateCv;
    bool gateOpen = false;
    std::atomic<bool> inGate{false};
    std::vector<int> seen;
    std::mutex seenMutex;
    Eng e("stats", [&](Eng::Id, Job &j) {
        {
            std::lock_guard<std::mutex> lock(seenMutex);
            seen.push_back(j.n);
        }
        if (j.n == 0) {
            inGate = true;
            std::unique_lock<std::mutex> lock(gateMutex);
            gateCv.wait(lock, [&] { return gateOpen; });
        }
        return Step::done(j.n);
    });
    check(e.start());
    auto held = std::make_shared<int>(0);
    e.submit(Job{0, nullptr}, DropPolicy::LatestOnly);
    check(waitFor([&] { return inGate.load(); }));
    for (int n = 1; n <= 4; ++n)
        e.submit(Job{n, held}, DropPolicy::LatestOnly);
    check(held.use_count() == 2);
    {
        std::lock_guard<std::mutex> lock(gateMutex);
        gateOpen = true;
    }
    gateCv.notify_all();
    check(waitFor([&] {
        std::lock_guard<std::mutex> lock(seenMutex);
        return seen.size() == 2;
    }));
    std::this_thread::sleep_for(20ms);
    {
        std::lock_guard<std::mutex> lock(seenMutex);
        check(seen == (std::vector<int>{0, 4}));
    }
    check(waitFor([&] { return held.use_count() == 1; }));
    e.stop();
}

/* Stopping: a blocked resolve is answered, a queued job is let go, a
 * waiting job whose fence comes within the grace finishes, one whose
 * fence never comes is cancelled after it; nothing is taken afterwards. */
void testStop() {
    EventFd late, never;
    std::atomic<int> handled{0};
    Eng e("stop", [&](Eng::Id, Job &j) {
        handled++;
        const int n = j.n;
        return Step::wait(n == 0 ? late.fd() : never.fd(),
                          [n] { return Step::done(n); });
    }, Eng::Options{.maxRunning = 2, .keepOutcomes = true,
                    .stopGrace = 150ms});
    check(e.setStallWatch(10s, [] {}));
    check(e.start());
    check(!e.setStallWatch(10ms, [] {}));

    auto neverHeld = std::make_shared<int>(0);
    auto queuedHeld = std::make_shared<int>(0);
    const auto lateId = e.submit(Job{0, nullptr});
    const auto neverId = e.submit(Job{1, neverHeld});
    check(waitFor([&] { return handled.load() == 2; }));
    const auto queuedId = e.submit(Job{2, queuedHeld});

    std::atomic<bool> answered{false};
    std::thread resolver([&] {
        check(e.resolve(*queuedId, true).state == State::Cancelled);
        answered = true;
    });
    std::thread fencer([&] {
        std::this_thread::sleep_for(50ms);
        late.notify();
    });

    const auto t0 = std::chrono::steady_clock::now();
    e.stop();
    const auto took = std::chrono::steady_clock::now() - t0;
    resolver.join();
    fencer.join();

    check(answered.load());
    check(took >= 140ms && took < 1000ms);
    check(queuedHeld.use_count() == 1);
    check(neverHeld.use_count() == 1);
    check(handled.load() == 2);
    check(e.resolve(*lateId, true).state == State::Done);
    check(e.resolve(*neverId, true).state == State::Cancelled);
    check(!e.submit(Job{3, nullptr}).has_value());
    check(!e.start());
}

/* A job waiting past the limit stalls once, and keeps waiting. */
void testStall() {
    EventFd fence;
    std::atomic<int> stalls{0};
    Eng e("stall", [&](Eng::Id, Job &) {
        return Step::wait(fence.fd(), [] { return Step::done(1); });
    }, keep());
    check(e.setStallWatch(40ms, [&] { stalls++; }));
    check(e.start());
    const auto id = e.submit(Job{0, nullptr});
    check(waitFor([&] { return stalls.load() == 1; }, 500ms));
    std::this_thread::sleep_for(120ms);
    check(stalls.load() == 1);
    fence.notify();
    check(e.resolve(*id, true).state == State::Done);
    e.stop();
}

/* A watched fd is polled with the work, its callback on the engine's
 * thread; watching is set up before start only. */
void testWatch() {
    EventFd node;
    std::atomic<int> reads{0};
    std::atomic<bool> onEngine{false};
    std::thread::id engineThread;
    Eng e("watch", [&](Eng::Id, Job &) {
        engineThread = std::this_thread::get_id();
        return Step::done(0);
    }, keep());
    check(e.watch(node.fd(), [&](short revents) {
        check(revents & POLLIN);
        node.drain();
        reads++;
        onEngine = std::this_thread::get_id() == engineThread;
    }));
    check(e.start());
    check(!e.watch(node.fd(), [](short) {}));
    const auto id = e.submit(Job{0, nullptr});
    check(e.resolve(*id, true).state == State::Done);
    node.notify();
    check(waitFor([&] { return reads.load() == 1; }));
    check(onEngine.load());
    node.notify();
    check(waitFor([&] { return reads.load() == 2; }));
    e.stop();
}

/* A watch polls for the events it names, not only readable: an
 * eventfd with nothing to read is writable at once. (A V4L2 node's
 * events poll POLLPRI.) */
void testWatchEvents() {
    EventFd node;
    std::atomic<int> calls{0};
    Eng e("watch-events", [](Eng::Id, Job &) { return Step::done(0); },
          keep());
    check(e.watch(node.fd(), [&](short revents) {
        if (revents & POLLOUT && calls.fetch_add(1) == 0)
            check(!(revents & POLLIN));
    }, POLLOUT));
    check(e.start());
    check(waitFor([&] { return calls.load() >= 1; }));
    e.stop();
}

/* An owner on outcomeFd: ack, then look; every outcome is seen. */
void testOutcomeFd() {
    constexpr int kJobs = 200;
    Eng e("outcomes", [](Eng::Id, Job &j) { return Step::done(j.n); },
          keep());
    check(e.start());
    std::vector<Eng::Id> ids;
    for (int n = 0; n < kJobs; ++n)
        ids.push_back(*e.submit(Job{n, nullptr}));
    int collected = 0;
    size_t next = 0;
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (collected < kJobs && std::chrono::steady_clock::now() < deadline) {
        pollfd p{e.outcomeFd(), POLLIN, 0};
        ::poll(&p, 1, 100);
        e.ackOutcomes();
        Eng::Outcome out{};
        while (next < ids.size() && e.tryResolve(ids[next], &out)) {
            check(out.state == State::Done && *out.result ==
                  static_cast<int>(next));
            next++;
            collected++;
        }
    }
    check(collected == kJobs);
    e.stop();
}

/* Producers on many threads, a chain of two engines: no job is lost to
 * a wake that came at the wrong moment. */
void testNoLostWake() {
    constexpr int kProducers = 4;
    constexpr int kPerProducer = 3000;
    std::atomic<int> atEnd{0};
    Eng last("last", [&](Eng::Id, Job &) {
        atEnd++;
        return Step::done(0);
    });
    Eng first("first", [&](Eng::Id, Job &j) {
        check(last.submit(Job{j.n, std::move(j.held)}).has_value());
        return Step::done(0);
    });
    check(last.start());
    check(first.start());
    std::vector<std::thread> producers;
    for (int p = 0; p < kProducers; ++p) {
        producers.emplace_back([&first, p] {
            for (int i = 0; i < kPerProducer; ++i) {
                check(first.submit(Job{p * kPerProducer + i, nullptr})
                              .has_value());
                if (i % 64 == 0)
                    std::this_thread::yield();
            }
        });
    }
    for (std::thread &t : producers)
        t.join();
    check(waitFor([&] { return atEnd.load() == kProducers * kPerProducer; },
                  5000ms));
    first.stop();
    last.stop();
}

/* No grace: a job waiting only for work to come in is cancelled at
 * once on stop. */
void testNoGrace() {
    EventFd never;
    Eng e("nograce", [&](Eng::Id, Job &) {
        return Step::wait(never.fd(), [] { return Step::done(); });
    }, Eng::Options{.maxRunning = 1, .keepOutcomes = true,
                    .stopGrace = 0ms});
    check(e.start());
    auto held = std::make_shared<int>(0);
    const auto id = e.submit(Job{0, held});
    check(waitFor([&] { return held.use_count() == 2; }));
    const auto t0 = std::chrono::steady_clock::now();
    e.stop();
    check(std::chrono::steady_clock::now() - t0 < 100ms);
    check(held.use_count() == 1);
    check(e.resolve(*id, true).state == State::Cancelled);
}

/* An engine never started stops cleanly and refuses work. */
void testNeverStarted() {
    Eng e("idle", [](Eng::Id, Job &) { return Step::done(0); });
    auto held = std::make_shared<int>(0);
    check(e.submit(Job{0, held}).has_value());
    e.stop();
    check(held.use_count() == 1);
    check(!e.submit(Job{1, nullptr}).has_value());
}

} // namespace

int main() {
    testSynchronous();
    testAsynchronousAndRearm();
    testMaxRunning();
    testCancel();
    testLatestOnly();
    testStop();
    testStall();
    testWatch();
    testWatchEvents();
    testOutcomeFd();
    testNoLostWake();
    testNoGrace();
    testNeverStarted();
    return finish("engine_test");
}
