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

/* Host test for the frame and what holds it: FrameStatus, FrameToken,
 * FrameRegistry, and what a frame gives back when it goes. Run by
 * core/tests/run_host_tests.sh. */

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <thread>
#include <vector>

#include "EventFd.h"
#include "Frame.h"
#include "FrameRegistry.h"
#include "IspPipeline.h"

using namespace mocha::camera;
using namespace std::chrono_literals;

namespace {

int failures = 0;

#define CHECK(cond)                                                \
    do {                                                           \
        if (!(cond)) {                                             \
            std::printf("%s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++;                                            \
        }                                                          \
    } while (0)

bool readable(int fd) {
    pollfd p{fd, POLLIN, 0};
    return ::poll(&p, 1, 0) == 1;
}

bool isOpen(int fd) {
    return ::fcntl(fd, F_GETFD) != -1;
}

/* A JPEG ring that counts what comes back. */
class FakeJpegRing : public android::IspJpegRing {
public:
    void invalidateJpegSnapshot(const android::JpegSnapshot &) override {}
    void releaseJpegSnapshot(const android::JpegSnapshot &snap) override {
        released.push_back(snap.ringSlot);
    }
    std::vector<int> released;
};

/* A frame's outcome is decided once: the first of fail, cancel and
 * commit wins; a committed frame is ok, a failed or cancelled one not. */
void testStatus() {
    {
        android::FrameStatus s;
        CHECK(s.ok());
        CHECK(s.commit());
        CHECK(s.ok());
        CHECK(!s.cancel());
        CHECK(!s.fail(-1));
        CHECK(s.kind() == android::FrameStatus::Kind::Committed);
    }
    {
        android::FrameStatus s;
        CHECK(s.fail(-22));
        CHECK(!s.ok());
        CHECK(!s.cancel());
        CHECK(!s.commit());
        CHECK(s.reason() == -22);
    }
    {
        android::FrameStatus s;
        CHECK(s.cancel());
        CHECK(!s.commit());
        CHECK(s.kind() == android::FrameStatus::Kind::Cancelled);
    }
}

/* Tokens count the jobs on a frame; the last one to go wakes the
 * result engine, and only the last. A moved token is one token. */
void testTokens() {
    android::EventFd settled;
    auto frame = std::make_shared<Frame>();
    frame->settled = &settled;
    {
        FrameToken a(frame);
        CHECK(frame->inPipeline.load() == 1);
        FrameToken b = a.share();
        CHECK(frame->inPipeline.load() == 2);
        FrameToken c = std::move(a);
        CHECK(!a);
        CHECK(frame->inPipeline.load() == 2);
        b = FrameToken();
        CHECK(frame->inPipeline.load() == 1);
        CHECK(!readable(settled.fd()));
    }
    CHECK(frame->inPipeline.load() == 0);
    CHECK(readable(settled.fd()));
}

/* Tokens going on several threads at once: exactly one wake per drop
 * to zero, and the count never goes wrong. */
void testTokensRace() {
    android::EventFd settled;
    auto frame = std::make_shared<Frame>();
    frame->settled = &settled;
    for (int round = 0; round < 200; ++round) {
        settled.drain();
        FrameToken root(frame);
        std::vector<std::thread> threads;
        for (int t = 0; t < 4; ++t) {
            FrameToken share = root.share();
            threads.emplace_back([token = std::move(share)]() mutable {
                FrameToken more = token.share();
                std::this_thread::yield();
            });
        }
        root = FrameToken();
        for (std::thread &t : threads) t.join();
        CHECK(frame->inPipeline.load() == 0);
        CHECK(readable(settled.fd()));
    }
}

/* Over the ceiling the frame is still known (for flush), and add says
 * so; cancelIf cancels what it picks and leaves the committed alone;
 * waitEmpty waits for the last remove. */
void testRegistry() {
    FrameRegistry registry(2);
    std::vector<FrameRef> frames;
    for (uint32_t n = 0; n < 3; ++n) {
        auto f = std::make_shared<Frame>();
        f->frameNumber = n;
        frames.push_back(f);
    }
    CHECK(registry.add(frames[0]));
    CHECK(registry.add(frames[1]));
    CHECK(!registry.add(frames[2]));
    CHECK(registry.count() == 3);

    frames[1]->status.commit();
    registry.cancelIf([](const Frame &f) { return f.frameNumber != 2; });
    CHECK(frames[0]->status.kind() == android::FrameStatus::Kind::Cancelled);
    CHECK(frames[1]->status.kind() == android::FrameStatus::Kind::Committed);
    CHECK(frames[2]->status.ok());

    CHECK(!registry.waitEmpty(20));
    std::thread remover([&registry] {
        std::this_thread::sleep_for(20ms);
        registry.remove(0);
        registry.remove(1);
        registry.remove(2);
    });
    CHECK(registry.waitEmpty(2000));
    remover.join();

    registry.add(frames[0]);
    CHECK(registry.takeAll().size() == 1);
    CHECK(registry.count() == 0);
}

/* A frame that goes without its result having gone out gives back the
 * JPEG snapshots it still holds and closes the release fences it still
 * owns; what was used up stays used up. */
void testFrameGivesBack() {
    FakeJpegRing ring;
    int pipes[2];
    CHECK(::pipe(pipes) == 0);
    {
        auto frame = std::make_shared<Frame>();
        frame->jpegRing = &ring;
        frame->outputs.resize(3);
        frame->outputs[0].jpeg.ringSlot = 1;      /* still held */
        frame->outputs[1].jpeg.ringSlot = -1;     /* used up */
        frame->outputs[2].releaseFence  = pipes[0];
    }
    CHECK(ring.released.size() == 1 && ring.released[0] == 1);
    CHECK(!isOpen(pipes[0]));
    ::close(pipes[1]);
}

} // namespace

int main() {
    testStatus();
    testTokens();
    testTokensRace();
    testRegistry();
    testFrameGivesBack();
    if (failures == 0)
        std::printf("frame_test: all passed\n");
    return failures == 0 ? 0 : 1;
}
