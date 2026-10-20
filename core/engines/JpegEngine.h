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

#ifndef CORE_ENGINES_JPEG_ENGINE_H
#define CORE_ENGINES_JPEG_ENGINE_H

#include <cstddef>
#include <memory>
#include <variant>

#include "Engine.h"
#include "Frame.h"

namespace android { class BufferProcessor; }

namespace mocha::camera {

/* Encodes the BLOB outputs of committed frames, off the GPU's path, so
 * libjpeg runs beside the next frame's demosaic.
 *
 * A job is one BLOB output; it holds a token on its frame, so the
 * result engine waits for it, and it owns the output's JPEG snapshot
 * (a slot of the ISP's ring): a job that ends without encoding --
 * dropped by a stopping engine, say -- gives the slot back and marks
 * the output failed on its way out, so neither the slot nor the buffer
 * is lost. */
class JpegEngine {
public:
    class Job {
    public:
        Job(FrameToken frame, size_t output, android::BufferProcessor *processor)
            : frame(std::move(frame)), output(output), processor(processor) {}
        Job(Job &&other) noexcept;
        Job &operator=(Job &&other) noexcept;
        Job(const Job &) = delete;
        Job &operator=(const Job &) = delete;
        ~Job();

        void encode();

    private:
        void giveUp();

        FrameToken frame;
        size_t output;
        android::BufferProcessor *processor;
        bool finished = false;
    };

    JpegEngine() = default;
    ~JpegEngine();

    JpegEngine(const JpegEngine &) = delete;
    JpegEngine &operator=(const JpegEngine &) = delete;

    bool start();
    void stop();

    /* False once stopped; the job is let go (and gives its output up). */
    bool submit(Job job);

private:
    using Eng = Engine<Job, std::monostate>;

    std::unique_ptr<Eng> engine;
};

} /* namespace mocha::camera */

#endif /* CORE_ENGINES_JPEG_ENGINE_H */
