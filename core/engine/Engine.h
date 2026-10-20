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

#ifndef CORE_ENGINE_ENGINE_H
#define CORE_ENGINE_ENGINE_H

#include <poll.h>
#include <pthread.h>

#include <algorithm>
#include <cerrno>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "EventFd.h"
#include "JobQueue.h"

namespace mocha::camera {

/* How an engine runs its jobs; see Engine. */
struct EngineOptions {
    /* Jobs taken at once, running or waiting. */
    size_t maxRunning = 1;
    /* Whether an owner collects outcomes (resolve, tryResolve). An engine
     * whose jobs pass their work on by themselves keeps none; see
     * JobQueue. */
    bool keepOutcomes = false;
};

/* One thread of the camera core, and the only way the core has one.
 *
 * An engine is a JobQueue and the thread that works it, with one life
 * cycle: built and started when the stream starts, stopped and destroyed
 * when it stops -- never restarted, the state worth keeping lives in the
 * components that own engines, not in the engines. A component owns its
 * engine and hands it a handler; nothing inherits a thread.
 *
 * The handler is either synchronous -- it does the work and says so
 * (Step::done, Step::refused) -- or asynchronous: it starts the work and
 * says what to wait for (Step::wait, a fence or another engine's
 * outcomeFd). The engine then keeps the job, polls the fd on its own
 * thread and calls the continuation when it is readable; that again
 * answers with a Step, so a job may wait more than once. There is no
 * third kind.
 *
 * Everything happens on the engine's thread: the handler, the
 * continuations, the callbacks of watched fds, the stall callback, and
 * the destruction of a finished job -- so whatever a job holds (a frame
 * reference, a lease on a capture slot) is let go there.
 *
 * The thread sits in one poll: its wake (submit, cancel, stop), the fds
 * of jobs that wait, and the fds the component watches. The wake is
 * drained before the queue is looked at, so a job queued at any moment
 * is either seen now or wakes the next poll -- the lost wake that hung
 * the pipeline on 20.10 cannot be written again on top of this.
 *
 * `maxRunning` is how many jobs may be taken at once, running or
 * waiting: the depth of a ring of GPU slots, or 1 where jobs must not
 * overlap. */
template <typename Job, typename Result>
class Engine {
    using Queue = JobQueue<Job, Result>;

public:
    using Id = typename Queue::Id;
    using State = typename Queue::State;
    using Outcome = typename Queue::Outcome;

    /* What the handler, or a continuation, tells the engine. */
    class Step {
    public:
        /* Finished; `result` is the outcome. */
        static Step done(Result result) {
            Step s(Kind::Done);
            s.result.emplace(std::move(result));
            return s;
        }

        /* Not done, and not going to be (the JPEG did not encode, the
         * statistics are stale); `why`, when given, goes to the owner. */
        static Step refused(std::optional<Result> why = std::nullopt) {
            Step s(Kind::Refused);
            s.result = std::move(why);
            return s;
        }

        /* Started; `onReady` runs on the engine's thread once `fd` polls
         * readable (or reports an error) and answers how it went. The fd
         * stays the job's: the engine never reads or closes it, and it
         * must stay open until the continuation runs or the job is
         * cancelled. A level-triggered fd that stays readable (an
         * eventfd) must be drained by the continuation before it waits
         * on it again, or the engine spins. */
        static Step wait(int fd, std::function<Step()> onReady) {
            Step s(Kind::Wait);
            s.fd = fd;
            s.onReady = std::move(onReady);
            return s;
        }

    private:
        enum class Kind : uint8_t { Done, Refused, Wait };

        explicit Step(Kind kind) : kind(kind) {}

        Kind kind;
        std::optional<Result> result;
        int fd = -1;
        std::function<Step()> onReady;

        friend class Engine;
    };

    using Handler = std::function<Step(Id, Job &)>;

    using Options = EngineOptions;

    Engine(std::string name, Handler handler, Options options = {})
        : name(std::move(name)),
          handler(std::move(handler)),
          options(options),
          queue(options.keepOutcomes) {
        if (this->options.maxRunning == 0)
            this->options.maxRunning = 1;
    }

    ~Engine() { stop(); }

    Engine(const Engine &) = delete;
    Engine &operator=(const Engine &) = delete;

    /* Before start() only: an fd of the component's own (the V4L2 node)
     * polled with the engine's work; `onReadable` runs on the engine's
     * thread when it polls readable or reports an error. */
    bool watch(int fd, std::function<void()> onReadable) {
        if (started)
            return false;
        watched.push_back(Watched{fd, std::move(onReadable)});
        return true;
    }

    /* Before start() only: a job that has waited longer than `limit`
     * (counted from its first wait) calls `onStall`, once per job. The
     * job keeps waiting; what a stall means is the component's call --
     * in the core, ERROR_DEVICE. */
    bool setStallWatch(std::chrono::milliseconds limit,
                       std::function<void()> onStall) {
        if (started)
            return false;
        stallLimit = limit;
        this->onStall = std::move(onStall);
        return true;
    }

    /* Once. False if the engine was started or stopped before, or its
     * wake could not be made. */
    bool start() {
        std::lock_guard<std::mutex> lock(lifeMutex);
        if (started || stopped || !wake.isValid() || !outcome.isValid())
            return false;
        started = true;
        worker = std::thread([this] { run(); });
        return true;
    }

    /* For good. Jobs still queued are let go; jobs that wait get until
     * their stall limit (1 s without one) to finish, and those that do
     * not are cancelled; whoever waits in resolve() is answered. Joins
     * the thread, unless called from it. Idempotent. */
    void stop() {
        {
            std::lock_guard<std::mutex> lock(lifeMutex);
            stopped = true;
        }
        queue.stop();
        stopping.store(true, std::memory_order_release);
        wake.notify();
        if (worker.joinable() && worker.get_id() != std::this_thread::get_id())
            worker.join();
        outcome.notify();
    }

    /* Never waits. A stopped engine refuses the job (nullopt), and the
     * job is let go -- the producer must deal with the refusal. */
    std::optional<Id> submit(Job job, DropPolicy drop = DropPolicy::None) {
        std::lock_guard<std::mutex> lock(lifeMutex);
        if (stopped)
            return std::nullopt;
        const Id id = queue.queue(std::move(job), drop);
        wake.notify();
        if (drop != DropPolicy::None && options.keepOutcomes)
            outcome.notify();
        return id;
    }

    /* The job is not wanted any more. Queued: forgotten. Waiting: its fd
     * leaves the poll and the job is let go, on the engine's thread.
     * Running: it runs to its end, its outcome not kept. */
    void cancel(Id id) {
        queue.cancel(id);
        {
            std::lock_guard<std::mutex> lock(cancelMutex);
            cancelled.push_back(id);
        }
        wake.notify();
    }

    /* For an owner in a poll loop of its own (keepOutcomes): the outcome
     * if there is one, false while the job is queued, running or
     * waiting. */
    bool tryResolve(Id id, Outcome *out) { return queue.tryResolve(id, out); }

    /* For an owner that may block (keepOutcomes); see JobQueue::resolve.
     * Never from the engine's own thread. */
    Outcome resolve(Id id, bool wait) { return queue.resolve(id, wait); }

    /* Readable when outcomes may have arrived since ackOutcomes() -- the
     * fd another engine waits on. One reader: the owner acks, then looks
     * (tryResolve), in that order, so no outcome is missed. */
    int outcomeFd() const { return outcome.fd(); }
    void ackOutcomes() { outcome.drain(); }

    const std::string &engineName() const { return name; }

private:
    struct Record {
        explicit Record(Job job) : job(std::move(job)) {}

        Job job;
        int fd = -1;
        std::function<Step()> onReady;
        std::chrono::steady_clock::time_point since;
        bool waitedBefore = false;
        bool stalled = false;
    };

    struct Watched {
        int fd;
        std::function<void()> onReadable;
    };

    using Clock = std::chrono::steady_clock;

    static constexpr std::chrono::milliseconds kStopGrace{1000};

    void run() {
        setThreadName();

        std::vector<pollfd> fds;
        std::vector<Id> waitIds;
        while (!stopping.load(std::memory_order_acquire)) {
            buildPollSet(fds, waitIds, true);
            const int n = ::poll(fds.data(), fds.size(), pollTimeoutMs());
            if (n < 0 && errno != EINTR)
                break;

            /* Before anything is looked at: whatever arrives from here on
             * wakes the next poll. */
            wake.drain();
            if (stopping.load(std::memory_order_acquire))
                break;

            applyCancels();
            if (n > 0) {
                const size_t firstWatched = 1;
                for (size_t i = 0; i < watched.size(); ++i)
                    if (fds[firstWatched + i].revents != 0)
                        watched[i].onReadable();
                const size_t firstWait = firstWatched + watched.size();
                for (size_t i = 0; i < waitIds.size(); ++i)
                    if (fds[firstWait + i].revents != 0)
                        resume(waitIds[i]);
            }
            checkStalls();
            takeJobs();
        }

        finishWaiting();
    }

    /* fds: the wake, then the watched fds (unless `withWatched` is off),
     * then one per waiting job, in the order of waitIds. */
    void buildPollSet(std::vector<pollfd> &fds, std::vector<Id> &waitIds,
                      bool withWatched) {
        fds.clear();
        waitIds.clear();
        fds.push_back(pollfd{wake.fd(), POLLIN, 0});
        if (withWatched)
            for (const Watched &w : watched)
                fds.push_back(pollfd{w.fd, POLLIN, 0});
        for (const auto &[id, rec] : waiting) {
            fds.push_back(pollfd{rec->fd, POLLIN, 0});
            waitIds.push_back(id);
        }
    }

    /* Until the nearest stall, or for ever. */
    int pollTimeoutMs() const {
        if (stallLimit.count() <= 0 || !onStall)
            return -1;
        const auto now = Clock::now();
        std::optional<Clock::duration> nearest;
        for (const auto &[id, rec] : waiting) {
            if (rec->stalled)
                continue;
            const auto left = rec->since + stallLimit - now;
            if (!nearest || left < *nearest)
                nearest = left;
        }
        if (!nearest)
            return -1;
        const auto ms = std::chrono::ceil<std::chrono::milliseconds>(*nearest);
        return static_cast<int>(std::max<int64_t>(0, ms.count()));
    }

    void applyCancels() {
        std::vector<Id> ids;
        {
            std::lock_guard<std::mutex> lock(cancelMutex);
            ids.swap(cancelled);
        }
        for (Id id : ids) {
            auto it = waiting.find(id);
            if (it == waiting.end())
                continue; /* queued (already forgotten) or finished */
            std::unique_ptr<Record> rec = std::move(it->second);
            waiting.erase(it);
            rec.reset();
            queue.done(id, false, std::nullopt); /* forgotten: cancelled */
        }
    }

    void resume(Id id) {
        auto it = waiting.find(id);
        if (it == waiting.end())
            return; /* cancelled since the poll */
        std::unique_ptr<Record> rec = std::move(it->second);
        waiting.erase(it);
        /* The continuation is moved out first: it may be replaced by the
         * Step it returns. */
        std::function<Step()> onReady = std::move(rec->onReady);
        settle(id, std::move(rec), onReady());
    }

    void checkStalls() {
        if (stallLimit.count() <= 0 || !onStall)
            return;
        const auto now = Clock::now();
        for (auto &[id, rec] : waiting) {
            if (rec->stalled || now - rec->since < stallLimit)
                continue;
            rec->stalled = true;
            onStall();
        }
    }

    void takeJobs() {
        while (waiting.size() < options.maxRunning) {
            auto taken = queue.tryTake();
            if (!taken)
                return;
            auto rec = std::make_unique<Record>(std::move(taken->second));
            Step step = handler(taken->first, rec->job);
            settle(taken->first, std::move(rec), std::move(step));
        }
    }

    void settle(Id id, std::unique_ptr<Record> rec, Step step) {
        if (step.kind == Step::Kind::Wait) {
            rec->fd = step.fd;
            rec->onReady = std::move(step.onReady);
            if (!rec->waitedBefore) {
                rec->waitedBefore = true;
                rec->since = Clock::now();
            }
            waiting.emplace(id, std::move(rec));
            return;
        }
        /* What the job holds is let go before its outcome shows. */
        rec.reset();
        queue.done(id, step.kind == Step::Kind::Done, std::move(step.result));
        if (options.keepOutcomes)
            outcome.notify();
    }

    /* Stopping: the jobs that wait get their grace, then are cancelled. */
    void finishWaiting() {
        const auto grace = stallLimit.count() > 0 ? stallLimit : kStopGrace;
        const auto deadline = Clock::now() + grace;
        std::vector<pollfd> fds;
        std::vector<Id> waitIds;
        while (!waiting.empty()) {
            applyCancels();
            if (waiting.empty())
                break;
            const auto left = std::chrono::ceil<std::chrono::milliseconds>(
                    deadline - Clock::now());
            if (left.count() <= 0)
                break;
            buildPollSet(fds, waitIds, false);
            const int n = ::poll(fds.data(), fds.size(),
                                 static_cast<int>(left.count()));
            wake.drain();
            if (n <= 0)
                continue;
            for (size_t i = 0; i < waitIds.size(); ++i)
                if (fds[1 + i].revents != 0)
                    resume(waitIds[i]);
        }
        for (auto &[id, rec] : waiting) {
            rec.reset();
            queue.cancel(id);
            queue.done(id, false, std::nullopt);
        }
        waiting.clear();
        outcome.notify();
    }

    void setThreadName() {
        /* The kernel keeps 15 characters. */
        const std::string shortName = name.substr(0, 15);
#ifdef __APPLE__
        pthread_setname_np(shortName.c_str());
#else
        pthread_setname_np(pthread_self(), shortName.c_str());
#endif
    }

    const std::string name;
    const Handler handler;
    Options options;
    Queue queue;

    android::EventFd wake;
    android::EventFd outcome;

    std::vector<Watched> watched;
    std::chrono::milliseconds stallLimit{0};
    std::function<void()> onStall;

    std::mutex lifeMutex;
    bool started = false;
    bool stopped = false;
    std::atomic<bool> stopping{false};

    std::mutex cancelMutex;
    std::vector<Id> cancelled;

    /* The engine thread's own: jobs taken and waiting. */
    std::map<Id, std::unique_ptr<Record>> waiting;

    std::thread worker;
};

} /* namespace mocha::camera */

#endif /* CORE_ENGINE_ENGINE_H */
