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

/* Host test for the engines' job queue. Linux (the build server):
 *
 *   clang++ -std=c++20 -Wall -Wextra -fsanitize=thread -g -Icore/engine \
 *       core/tests/job_queue_test.cpp -o out/host-tests/job_queue_test \
 *       && out/host-tests/job_queue_test
 */

#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "JobQueue.h"
#include "Check.h"

using namespace mocha::camera;

using mocha::camera::test::check;
using mocha::camera::test::finish;

namespace {

/* A job that says when it is let go, as a job holding a frame would let
 * the frame go. */
struct Job {
    int frame;
    std::shared_ptr<int> held;
};

using Queue = JobQueue<Job, std::string>;
using State = Queue::State;

constexpr DropPolicy kNone = DropPolicy::None;
constexpr DropPolicy kBounded = DropPolicy::Bounded;
constexpr DropPolicy kLatest = DropPolicy::LatestOnly;

/* Queued, taken, done, collected. */
void testOrdinary() {
    Queue q;
    const auto id = q.queue(Job{1, nullptr}, kBounded);
    auto taken = q.take();
    check(taken.has_value());
    check(taken->first == id);
    check(taken->second.frame == 1);
    q.done(id, true, std::string("fence-1"));
    const auto out = q.resolve(id, true);
    check(out.state == State::Done);
    check(out.result && *out.result == "fence-1");
    /* Collected once; asked again it is gone. */
    check(q.resolve(id, true).state == State::Cancelled);
}

/* A newer job drops a droppable one the worker has not taken, and lets
 * its contents go at once; a job that may not be dropped stays. */
void testLatestWins() {
    Queue q;
    auto held = std::make_shared<int>(0);
    const auto first = q.queue(Job{1, held}, kBounded);
    check(held.use_count() == 2);
    const auto second = q.queue(Job{2, nullptr}, kBounded);
    check(held.use_count() == 1);
    check(q.resolve(first, true).state == State::Dropped);
    check(q.droppedCount() == 1);

    auto taken = q.take();
    check(taken && taken->first == second);
    q.done(second, true, std::string("fence-2"));
    check(q.resolve(second, true).state == State::Done);

    const auto keep = q.queue(Job{3, nullptr}, kNone);
    q.queue(Job{4, nullptr}, kBounded);
    taken = q.take();
    check(taken && taken->first == keep);
}

/* Bounded: never two drops in a row -- the second superseded job is run
 * after all, and a job run lets drops happen again. */
void testNotTwiceInARow() {
    Queue q;
    const auto a = q.queue(Job{1, nullptr}, kBounded);
    const auto b = q.queue(Job{2, nullptr}, kBounded);
    const auto c = q.queue(Job{3, nullptr}, kBounded);
    check(q.resolve(a, true).state == State::Dropped);
    auto taken = q.take();
    check(taken && taken->first == b);
    q.done(b, true, std::string("b"));
    taken = q.take();
    check(taken && taken->first == c);
    q.done(c, true, std::string("c"));
    check(q.resolve(b, true).state == State::Done);
    check(q.resolve(c, true).state == State::Done);

    const auto d = q.queue(Job{4, nullptr}, kBounded);
    q.queue(Job{5, nullptr}, kBounded);
    check(q.resolve(d, true).state == State::Dropped);
}

/* LatestOnly: every job still queued is superseded, however many in a
 * row; one already taken runs to its end. */
void testLatestOnly() {
    Queue q;
    auto held = std::make_shared<int>(0);
    const auto a = q.queue(Job{1, held}, kLatest);
    const auto b = q.queue(Job{2, held}, kLatest);
    const auto c = q.queue(Job{3, held}, kLatest);
    check(held.use_count() == 2);
    check(q.resolve(a, true).state == State::Dropped);
    check(q.resolve(b, true).state == State::Dropped);
    check(q.droppedCount() == 2);

    auto taken = q.take();
    check(taken && taken->first == c);
    const auto d = q.queue(Job{4, nullptr}, kLatest);
    const auto e = q.queue(Job{5, nullptr}, kLatest);
    check(q.resolve(d, true).state == State::Dropped);
    q.done(c, true, std::string("c"));
    check(q.resolve(c, true).state == State::Done);
    taken = q.take();
    check(taken && taken->first == e);
}

/* Without waiting: a queued droppable job is dropped then and there; one
 * that may not be dropped is waited for. */
void testResolveWithoutWaiting() {
    Queue q;
    const auto id = q.queue(Job{1, nullptr}, kBounded);
    check(q.resolve(id, false).state == State::Dropped);
    check(q.waiting() == 0);

    const auto kept = q.queue(Job{2, nullptr}, kNone);
    std::thread worker([&q] {
        auto taken = q.take();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        q.done(taken->first, true, std::string("late"));
    });
    const auto out = q.resolve(kept, false);
    check(out.state == State::Done);
    check(out.result && *out.result == "late");
    worker.join();
}

/* tryResolve never waits: false while queued or running, the outcome
 * once there is one, Cancelled for a job it no longer knows. */
void testTryResolve() {
    Queue q;
    Queue::Outcome out{};
    const auto id = q.queue(Job{1, nullptr}, kNone);
    check(!q.tryResolve(id, &out));
    auto taken = q.take();
    check(!q.tryResolve(id, &out));
    q.done(id, false, std::string("why"));
    check(q.tryResolve(id, &out));
    check(out.state == State::Refused);
    check(out.result && *out.result == "why");
    check(q.tryResolve(id, &out));
    check(out.state == State::Cancelled);

    /* Stopped: a job still queued has no outcome coming. */
    const auto late = q.queue(Job{2, nullptr}, kNone);
    q.stop();
    check(q.tryResolve(late, &out));
    check(out.state == State::Cancelled);
}

/* A refusal comes back as such; a cancelled job is never taken, and one
 * cancelled while taken is forgotten when it is done. */
void testRefusedAndCancelled() {
    Queue q;
    const auto refused = q.queue(Job{1, nullptr}, kNone);
    auto taken = q.take();
    q.done(taken->first, false, std::nullopt);
    check(q.resolve(refused, true).state == State::Refused);

    const auto gone = q.queue(Job{2, nullptr}, kNone);
    q.cancel(gone);
    const auto next = q.queue(Job{3, nullptr}, kNone);
    taken = q.take();
    check(taken && taken->first == next);
    q.cancel(next);
    q.done(next, true, std::string("unwanted"));
    check(q.resolve(next, true).state == State::Cancelled);

    /* An owner already waiting on a job cancelled while running is woken
     * when the worker is done with it. */
    const auto waited = q.queue(Job{4, nullptr}, kNone);
    taken = q.take();
    std::thread owner([&q, waited] {
        check(q.resolve(waited, true).state == State::Cancelled);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    q.cancel(waited);
    q.done(waited, true, std::string("unwanted"));
    owner.join();
}

/* Outcomes nobody collects are not kept: done and dropped jobs are
 * forgotten at once. */
void testNoOutcomesKept() {
    Queue q(false);
    const auto a = q.queue(Job{1, nullptr}, kLatest);
    q.queue(Job{2, nullptr}, kLatest);
    check(q.resolve(a, false).state == State::Cancelled);
    auto taken = q.take();
    check(taken && taken->second.frame == 2);
    q.done(taken->first, true, std::string("x"));
    check(q.resolve(taken->first, false).state == State::Cancelled);
    check(q.waiting() == 0);
}

/* A worker that cannot wait: nothing when nothing is queued, the job
 * when one is, and nothing for one dropped before it looked. */
void testTryTake() {
    Queue q;
    check(!q.tryTake().has_value());
    const auto a = q.queue(Job{1, nullptr}, kBounded);
    q.queue(Job{2, nullptr}, kBounded);
    auto taken = q.tryTake();
    check(taken && taken->second.frame == 2);
    check(q.resolve(a, true).state == State::Dropped);
    check(!q.tryTake().has_value());
}

/* Stopping answers the worker and whoever waits, and lets jobs go. */
void testStop() {
    Queue q;
    auto held = std::make_shared<int>(0);
    const auto id = q.queue(Job{1, held}, kNone);
    std::thread waiter([&q, id] {
        check(q.resolve(id, true).state == State::Cancelled);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    q.stop();
    waiter.join();
    check(held.use_count() == 1);
    check(!q.take().has_value());
}

/* A worker and an owner racing over many jobs: every job gets an
 * answer, nothing is answered twice, and drops never come two in a row
 * between jobs run. */
void testRace() {
    Queue q;
    std::atomic<int> ran{0};
    std::thread worker([&] {
        for (;;) {
            auto taken = q.take();
            if (!taken)
                return;
            std::this_thread::sleep_for(std::chrono::microseconds(300));
            q.done(taken->first, true, std::to_string(taken->second.frame));
            ran++;
        }
    });
    int dropped = 0;
    int answered = 0;
    for (int frame = 0; frame < 300; ++frame) {
        const auto id = q.queue(Job{frame, nullptr}, kBounded);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        const auto out = q.resolve(id, frame % 2 == 0);
        answered++;
        if (out.state == State::Dropped)
            dropped++;
        else
            check(out.state == State::Done &&
                  *out.result == std::to_string(frame));
    }
    q.stop();
    worker.join();
    check(answered == 300);
    check(dropped + ran.load() >= 300);
    check(static_cast<int>(q.droppedCount()) == dropped);
}

} // namespace

int main() {
    testOrdinary();
    testLatestWins();
    testNotTwiceInARow();
    testLatestOnly();
    testResolveWithoutWaiting();
    testTryResolve();
    testRefusedAndCancelled();
    testNoOutcomesKept();
    testTryTake();
    testStop();
    testRace();
    return finish("job_queue_test");
}
