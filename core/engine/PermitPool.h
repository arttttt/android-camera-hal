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

#ifndef CORE_ENGINE_PERMIT_POOL_H
#define CORE_ENGINE_PERMIT_POOL_H

#include <cstddef>
#include <mutex>
#include <optional>

#include "EventFd.h"

namespace mocha::camera {

class PermitPool;

/* The right to have one more job in an engine that holds a scarce
 * resource for as long as the job lives -- a capture slot in the
 * demosaic engine. Move-only; given back to its pool when destroyed,
 * on whatever path the job ends (done, refused, cancelled, dropped,
 * stopped), so a bound on the resource is kept by the type rather
 * than by every path remembering to count. */
class Permit {
public:
    Permit() = default;
    Permit(Permit &&other) noexcept : pool(other.pool) { other.pool = nullptr; }
    Permit &operator=(Permit &&other) noexcept {
        if (this != &other) {
            release();
            pool = other.pool;
            other.pool = nullptr;
        }
        return *this;
    }
    Permit(const Permit &) = delete;
    Permit &operator=(const Permit &) = delete;
    ~Permit() { release(); }

    explicit operator bool() const { return pool != nullptr; }

    inline void release();

private:
    friend class PermitPool;
    explicit Permit(PermitPool *pool) : pool(pool) {}

    PermitPool *pool = nullptr;
};

/* A fixed number of permits. Whoever waits for one polls `wake`, which
 * is notified each time a permit comes back; it drains the wake before
 * trying again, so a permit returned in between is not missed. The
 * pool outlives every permit it hands out. */
class PermitPool {
public:
    PermitPool(size_t capacity, android::EventFd *wake)
        : capacity(capacity), available(capacity), wake(wake) {}

    PermitPool(const PermitPool &) = delete;
    PermitPool &operator=(const PermitPool &) = delete;

    std::optional<Permit> tryAcquire() {
        std::lock_guard<std::mutex> lock(mutex);
        if (available == 0)
            return std::nullopt;
        available--;
        return Permit(this);
    }

    size_t free() const {
        std::lock_guard<std::mutex> lock(mutex);
        return available;
    }

    size_t size() const { return capacity; }

private:
    friend class Permit;

    void giveBack() {
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (available < capacity)
                available++;
        }
        if (wake)
            wake->notify();
    }

    const size_t capacity;
    mutable std::mutex mutex;
    size_t available;
    android::EventFd *wake;
};

inline void Permit::release() {
    if (pool == nullptr)
        return;
    PermitPool *p = pool;
    pool = nullptr;
    p->giveBack();
}

} /* namespace mocha::camera */

#endif /* CORE_ENGINE_PERMIT_POOL_H */
