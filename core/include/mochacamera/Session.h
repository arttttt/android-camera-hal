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

#ifndef MOCHACAMERA_SESSION_H
#define MOCHACAMERA_SESSION_H

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "mochacamera/Buffers.h"
#include "mochacamera/Capability.h"
#include "mochacamera/Controls.h"
#include "mochacamera/FrameSettings.h"

namespace mocha::camera {

class GrallocPort;
class ResultSink;

/* A capture request as a frontend hands it in: its settings, parsed,
 * and its output buffers. */
struct FrameRequest {
    struct Output {
        StreamSpec    stream;
        NativeBuffer *buffer       = nullptr;
        /* Owned by the core from submit() on; -1 when none. */
        int           acquireFence = -1;
    };

    uint32_t            frameNumber = 0;
    FrameSettings       settings;
    std::vector<Output> outputs;
};

/* The camera core, for one camera: what a frontend opens, configures
 * and feeds requests to. Everything behind it -- the sensor and its
 * lens, the ISP, 3A, JPEG, the engines -- is the core's own; what it
 * has to say comes back through the ResultSink, and it reaches the
 * frontend's buffers only through the GrallocPort.
 *
 * Built once per camera and kept across open / close. Calls come from
 * the frontend's threads: open, close, configure and submit one at a
 * time; flush may run beside submit. */
class Session {
public:
    struct Config {
        /* The capture node, e.g. /dev/video0. */
        std::string devNode;
        bool        facingFront = false;
        /* The tuning to load: sensor and module integrator. */
        std::string sensorName;
        std::string integrator;
        /* The full ISP (WB, CCM, gamma), or demosaic only. */
        bool        softIsp = true;
        /* The mains the templates have auto exposure keep clear of.
         * Nothing on the device says what the local mains is (no modem,
         * no country), so it is set, not found out; 50 Hz is the most of
         * the world's. Off is allowed, but CTS wants 50 or 60 in the
         * templates. */
        Antibanding mains = Antibanding::Hz50;
        /* Requests the framework may have in flight: each stream's
         * max_buffers. More keeps frames coming through a hiccup;
         * fewer shortens the preview's and the shutter's lag -- a
         * request waits this many frames. Capped by the capture slots.
         * Measured on mocha (23.10.2026, auto exposure, both cameras):
         * 4 ran the preview at 28 fps, 5 at 29.7, 6 at 30, a request
         * taking 142, 167 and 198 ms to its result. */
        int         requestsInFlight = 6;
        /* The per-frame PERF lines and the GPU timing, for measuring;
         * off by default (they cost CPU and log bandwidth per frame). */
        bool        perfLog = false;
        /* For measuring where in a frame the sensor latches a write:
         * the write goes this long after the frame's start is reported,
         * not at once. */
        int         writeDelayUs = 0;
        /* The device, as its pictures' EXIF names it. */
        std::string make;
        std::string model;
    };

    Session(const Config &config, ResultSink *sink, GrallocPort *gralloc);
    ~Session();

    Session(const Session &) = delete;
    Session &operator=(const Session &) = delete;

    /* What the camera can do; probed on first use. */
    const Capability &capability();

    /* The sensor's mounting, for the frontend's camera info. */
    int32_t sensorOrientationDegrees() const;

    bool open();
    /* Every frame held goes back through the sink before this returns. */
    void close();

    /* What configure() made of a set of output streams. */
    enum class ConfigureResult {
        Ok,           /* capturing for them, the engines running */
        Unsupported,  /* no sensor mode feeds them; nothing changed */
        Failed,       /* the device did not start */
    };

    /* Capture for these output streams, the engines running. The core
     * picks the sensor mode (chooseCaptureSize): the one running when
     * it feeds the outputs, which then costs nothing, else the
     * smallest that does. A mode change gives every frame held back
     * first. */
    ConfigureResult configure(std::span<const StreamSpec> outputs);

    /* Whether the engines run, so a request can be taken. */
    bool running() const;

    /* Take a request; its result comes back through the sink, in
     * submission order. Only while running(). */
    void submit(FrameRequest request);

    /* Cancel every frame held and wait, up to a second, until none is;
     * the committed ones complete normally. */
    void flush();

    /* A few lines on the session's state, for dumpsys. */
    void dump(int fd);

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

} /* namespace mocha::camera */

#endif /* MOCHACAMERA_SESSION_H */
