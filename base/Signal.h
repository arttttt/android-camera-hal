#ifndef BASE_SIGNAL_H
#define BASE_SIGNAL_H

#include <functional>
#include <mutex>
#include <utility>
#include <vector>

namespace android {

/* Minimal fan-out of callbacks.
 *
 * connect() returns a token used to disconnect(); emit() invokes slots
 * synchronously on the caller's thread (direct dispatch — no queuing).
 * Safe to connect/disconnect concurrently with emit; a slot snapshot is
 * taken under the mutex and invoked outside it.
 *
 * A slot must not re-enter the same Signal (neither directly nor by
 * emitting another Signal that eventually calls this one). Queued
 * dispatch is out of scope — use an EventQueue if you need a thread
 * hop. */
template <typename... Args>
class Signal {
public:
    using Slot = std::function<void(Args...)>;

    Signal() : nextToken(0) {}

    Signal(const Signal&) = delete;
    Signal& operator=(const Signal&) = delete;

    int connect(Slot slot) {
        std::lock_guard<std::mutex> lock(mutex);
        int token = nextToken++;
        slots.push_back(Entry{token, std::move(slot)});
        return token;
    }

    void disconnect(int token) {
        std::lock_guard<std::mutex> lock(mutex);
        for (auto it = slots.begin(); it != slots.end(); ++it) {
            if (it->token == token) {
                slots.erase(it);
                return;
            }
        }
    }

    void emit(Args... args) {
        std::vector<Entry> snapshot;
        {
            std::lock_guard<std::mutex> lock(mutex);
            snapshot = slots;
        }
        for (auto& entry : snapshot) {
            entry.slot(args...);
        }
    }

private:
    struct Entry {
        int  token;
        Slot slot;
    };

    std::mutex mutex;
    std::vector<Entry> slots;
    int nextToken;
};

} /* namespace android */

#endif /* BASE_SIGNAL_H */
