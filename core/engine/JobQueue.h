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

#ifndef CORE_ENGINE_JOB_QUEUE_H
#define CORE_ENGINE_JOB_QUEUE_H

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace mocha::camera {

/* Whether a job still queued may be given up for a newer one.
 *
 * None: never -- a frame's own work (demosaic, finish, JPEG, result)
 * is done for every frame that asked for it.
 * Bounded: the newer one supersedes it, but never twice in a row, so a
 * stream of jobs that each arrive too late still moves.
 * LatestOnly: the newer one always supersedes it -- work whose only use
 * is the freshest input (the statistics behind 3A). */
enum class DropPolicy : uint8_t { None, Bounded, LatestOnly };

/* The jobs of one engine, between the threads that queue them and the
 * engine's own thread that runs them.
 *
 * A copy of the composer's JobQueue (android_hardware_nvidia_hwcomposer,
 * compositor/JobQueue.h), kept to the same rules so either can follow the
 * other. What differs: the states are named for running rather than for
 * submitting to hardware, the drop rule is a policy per job instead of a
 * flag, and the owner may ask for an outcome without waiting for it
 * (tryResolve) -- a camera engine asks from its own poll loop.
 *
 * A job is queued, taken by the worker, and done -- run to a result, or
 * refused. Its owner collects the outcome once; after that the job is
 * forgotten. A job still queued may be dropped when a newer one arrives
 * (see DropPolicy); once taken it is past dropping.
 *
 * `Job` is what the worker runs; `Result` what running it gives back.
 * Neither is looked into here, and nothing here is Android's, so the
 * queue is tested on its own. */
template <typename Job, typename Result>
class JobQueue {
public:
    using Id = uint64_t;

    enum class State : uint8_t {
        Queued,     /* waiting for the worker */
        Running,    /* the worker has it */
        Done,       /* run; the result is its outcome */
        Refused,    /* the worker would not, or could not, do it */
        Dropped,    /* superseded before the worker took it */
        Cancelled,  /* no longer wanted, or the queue stopped first */
    };

    struct Outcome {
        State state;
        /* Set for Done, and for Refused when the worker said why. */
        std::optional<Result> result;
    };

    /* After this many Bounded drops in a row the next superseded job is
     * run anyway. */
    static constexpr uint32_t kMostDroppedInARow = 1;

    /* `keepOutcomes`: whether anyone collects them. An engine whose jobs
     * hand their work on by themselves (to the next engine) has no owner
     * asking how they went; there an outcome would only pile up, so a job
     * is forgotten as soon as it is done or dropped. */
    explicit JobQueue(bool keepOutcomes = true) : keepOutcomes(keepOutcomes) {}

    /* Queues `job`; never waits. Jobs still queued that may be dropped
     * are dropped, each by its own policy. */
    Id queue(Job job, DropPolicy drop) {
        std::lock_guard<std::mutex> lock(mutex);
        for (Entry &e : entries) {
            if (e.state != State::Queued || e.drop == DropPolicy::None)
                continue;
            if (e.drop == DropPolicy::Bounded &&
                droppedInARow >= kMostDroppedInARow)
                continue;
            dropLocked(e);
        }
        forgetUncollectedLocked();
        const Id id = ++lastId;
        entries.push_back(Entry{id, State::Queued, drop, std::move(job),
                                std::nullopt});
        changed.notify_all();
        return id;
    }

    /* The worker's side: the oldest queued job, marked as taken; nothing
     * once the queue is stopped. Waits for one. */
    std::optional<std::pair<Id, Job>> take() {
        std::unique_lock<std::mutex> lock(mutex);
        for (;;) {
            if (stopped)
                return std::nullopt;
            if (auto taken = takeLocked())
                return taken;
            changed.wait(lock);
        }
    }

    /* The same for a worker with other work of its own to get on with:
     * the oldest queued job if there is one, nothing otherwise. */
    std::optional<std::pair<Id, Job>> tryTake() {
        std::lock_guard<std::mutex> lock(mutex);
        if (stopped)
            return std::nullopt;
        return takeLocked();
    }

    /* The worker's side: how the job it took went. */
    void done(Id id, bool ran, std::optional<Result> result) {
        std::lock_guard<std::mutex> lock(mutex);
        Entry *e = find(id);
        if (e == nullptr)
            return;
        if (ran)
            droppedInARow = 0;
        /* Whoever waits in resolve() is woken either way: a job forgotten
         * here answers Cancelled. (The composer's copy returns without
         * waking; its owners never wait on a cancelled job.) */
        if (e->cancelled || !keepOutcomes) {
            forget(id);
        } else {
            e->state = ran ? State::Done : State::Refused;
            e->result = std::move(result);
        }
        changed.notify_all();
    }

    /* The owner's side: the job's outcome, and the job forgotten. With
     * `wait`, a job still queued or running is waited for; a dropped one
     * answers at once either way. Without it, a job still queued that may
     * be dropped is dropped now -- the owner will not wait for it -- and
     * one running is waited for, there being no taking it back. */
    Outcome resolve(Id id, bool wait) {
        std::unique_lock<std::mutex> lock(mutex);
        for (;;) {
            Entry *e = find(id);
            if (e == nullptr)
                return Outcome{State::Cancelled, std::nullopt};
            if (e->state == State::Queued && !wait && mayDropLocked(*e))
                dropLocked(*e);
            if (e->state != State::Queued && e->state != State::Running)
                return collectLocked(id, *e);
            if (stopped) {
                forget(id);
                return Outcome{State::Cancelled, std::nullopt};
            }
            changed.wait(lock);
        }
    }

    /* The owner's side, never waiting: the outcome if the job has one --
     * collected and forgotten, as by resolve() -- false while it is still
     * queued or running. A job the queue no longer knows answers
     * Cancelled. */
    bool tryResolve(Id id, Outcome *out) {
        std::lock_guard<std::mutex> lock(mutex);
        Entry *e = find(id);
        if (e == nullptr) {
            *out = Outcome{State::Cancelled, std::nullopt};
            return true;
        }
        if (e->state == State::Queued || e->state == State::Running) {
            if (!stopped)
                return false;
            forget(id);
            *out = Outcome{State::Cancelled, std::nullopt};
            return true;
        }
        *out = collectLocked(id, *e);
        return true;
    }

    /* The owner's side: the job is not wanted any more. One the worker
     * has taken runs to its end, but its outcome is not kept. */
    void cancel(Id id) {
        std::lock_guard<std::mutex> lock(mutex);
        Entry *e = find(id);
        if (e == nullptr)
            return;
        if (e->state == State::Running) {
            e->cancelled = true;
            return;
        }
        forget(id);
    }

    /* No more jobs are taken; the worker's take() answers nothing, and
     * whoever waits in resolve() is answered. Jobs not taken are let go.
     * Jobs the worker has are still told done() and forgotten then. */
    void stop() {
        std::lock_guard<std::mutex> lock(mutex);
        stopped = true;
        for (Entry &e : entries)
            if (e.state == State::Queued)
                e.job.reset();
        changed.notify_all();
    }

    /* For the dump. */
    uint64_t droppedCount() const {
        std::lock_guard<std::mutex> lock(mutex);
        return dropped;
    }
    size_t waiting() const {
        std::lock_guard<std::mutex> lock(mutex);
        size_t n = 0;
        for (const Entry &e : entries)
            if (e.state == State::Queued)
                n++;
        return n;
    }

private:
    struct Entry {
        Id id;
        State state;
        DropPolicy drop;
        std::optional<Job> job;
        std::optional<Result> result;
        bool cancelled = false;
    };

    bool mayDropLocked(const Entry &e) const {
        if (e.drop == DropPolicy::None)
            return false;
        return e.drop == DropPolicy::LatestOnly ||
               droppedInARow < kMostDroppedInARow;
    }

    /* A LatestOnly drop does not count towards the Bounded limit: the
     * two policies are not mixed within one engine, and counting it would
     * only make a Bounded neighbour run a stale job. */
    void dropLocked(Entry &e) {
        e.state = State::Dropped;
        e.job.reset();
        if (e.drop == DropPolicy::Bounded)
            droppedInARow++;
        dropped++;
    }

    std::optional<std::pair<Id, Job>> takeLocked() {
        for (Entry &e : entries) {
            if (e.state != State::Queued)
                continue;
            e.state = State::Running;
            std::pair<Id, Job> taken{e.id, std::move(*e.job)};
            e.job.reset();
            return taken;
        }
        return std::nullopt;
    }

    Outcome collectLocked(Id id, Entry &e) {
        Outcome out{e.state, std::move(e.result)};
        forget(id);
        return out;
    }

    Entry *find(Id id) {
        for (Entry &e : entries)
            if (e.id == id)
                return &e;
        return nullptr;
    }

    /* Dropped entries nobody will collect. */
    void forgetUncollectedLocked() {
        if (keepOutcomes)
            return;
        for (auto it = entries.begin(); it != entries.end();) {
            if (it->state == State::Dropped)
                it = entries.erase(it);
            else
                ++it;
        }
    }

    /* An entry goes once its owner has the outcome, or has given it up; a
     * cancelled one the worker had goes when the worker is done with it. */
    void forget(Id id) {
        for (auto it = entries.begin(); it != entries.end(); ++it) {
            if (it->id == id) {
                entries.erase(it);
                return;
            }
        }
    }

    const bool keepOutcomes;
    mutable std::mutex mutex;
    std::condition_variable changed;
    std::deque<Entry> entries;
    Id lastId = 0;
    uint32_t droppedInARow = 0;
    uint64_t dropped = 0;
    bool stopped = false;
};

} /* namespace mocha::camera */

#endif /* CORE_ENGINE_JOB_QUEUE_H */
