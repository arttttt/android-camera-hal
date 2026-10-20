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

#include "JpegEngine.h"

#include "BufferProcessor.h"

namespace mocha::camera {

JpegEngine::Job::Job(Job &&other) noexcept
    : frame(std::move(other.frame)),
      output(other.output),
      processor(other.processor),
      finished(other.finished) {
    other.finished = true;
}

JpegEngine::Job &JpegEngine::Job::operator=(Job &&other) noexcept {
    if (this != &other) {
        if (!finished)
            giveUp();
        frame     = std::move(other.frame);
        output    = other.output;
        processor = other.processor;
        finished  = other.finished;
        other.finished = true;
    }
    return *this;
}

JpegEngine::Job::~Job() {
    if (!finished)
        giveUp();
}

void JpegEngine::Job::encode() {
    Frame &f = *frame;
    /* finalizeBlobOutput uses the snapshot up either way. A BLOB with
     * no image in it goes back as a failed buffer. The result engine
     * reads the status only once this job's token is gone. */
    const bool encoded = processor->finalizeBlobOutput(
            f.outputs[output], f.settings.jpeg, f.frameNumber);
    if (!encoded)
        f.outputs[output].status = BufferStatus::Error;
    finished = true;
}

void JpegEngine::Job::giveUp() {
    finished = true;
    if (!frame)
        return;
    processor->releaseJpegSnapshot(*frame, output);
    frame->outputs[output].status = BufferStatus::Error;
}

JpegEngine::~JpegEngine() {
    stop();
}

bool JpegEngine::start() {
    if (engine)
        return false;
    engine = std::make_unique<Eng>(
            "CamJpeg",
            [](Eng::Id, Job &job) {
                job.encode();
                return Eng::Step::done();
            });
    return engine->start();
}

void JpegEngine::stop() {
    if (!engine)
        return;
    engine->stop();
    engine.reset();
}

bool JpegEngine::submit(Job job) {
    return engine && engine->submit(std::move(job)).has_value();
}

} /* namespace mocha::camera */
