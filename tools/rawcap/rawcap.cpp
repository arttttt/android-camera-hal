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

/* Diagnostic standalone: the sensor's black level, per Bayer phase and
 * analog gain. Opens a capture node the way the HAL does (with the
 * camera closed: nothing else may stream), sets each gain in turn at a
 * short exposure, lets the sensor settle, and over the next frames
 * prints each of the four phases' mean, median and 1st percentile of
 * the raw 10-bit samples. Only the numbers leave the device.
 *
 *   rawcap --dev /dev/video0 [--width 1920 --height 1080]
 *          [--exposure-us 100] [--gains 256,1024,4096]
 *          [--frames 10] [--skip 15]
 *
 * Phases are named by position (row, column) of the 2x2 cell the frame
 * starts with: p00 p01 / p10 p11. Which is R, Gr, Gb, B is for a frame
 * of a red object to tell, not for the fourcc: the modes flip. */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <string>
#include <vector>

#include <linux/videodev2.h>

namespace {

constexpr int kBuffers = 4;
constexpr int kLevels  = 1024;

struct Options {
    std::string dev = "/dev/video0";
    unsigned width = 1920;
    unsigned height = 1080;
    int exposureUs = 100;
    std::vector<int> gains = {256, 1024, 4096};
    int frames = 10;
    int skip = 15;
};

bool parseList(const char *s, std::vector<int> *out) {
    out->clear();
    while (*s) {
        char *end = nullptr;
        const long v = strtol(s, &end, 0);
        if (end == s) return false;
        out->push_back((int)v);
        s = *end == ',' ? end + 1 : end;
    }
    return !out->empty();
}

bool parseArgs(int argc, char **argv, Options *o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (i + 1 >= argc) return false;
        const char *v = argv[++i];
        if (a == "--dev")              o->dev = v;
        else if (a == "--width")       o->width = (unsigned)atoi(v);
        else if (a == "--height")      o->height = (unsigned)atoi(v);
        else if (a == "--exposure-us") o->exposureUs = atoi(v);
        else if (a == "--frames")      o->frames = atoi(v);
        else if (a == "--skip")        o->skip = atoi(v);
        else if (a == "--gains") { if (!parseList(v, &o->gains)) return false; }
        else return false;
    }
    return o->frames > 0 && o->skip >= 0;
}

int xioctl(int fd, unsigned long req, void *arg) {
    int r;
    do { r = ioctl(fd, req, arg); } while (r < 0 && errno == EINTR);
    return r;
}

bool setControl(int fd, uint32_t id, int value, const char *name) {
    v4l2_queryctrl q = {};
    q.id = id;
    if (xioctl(fd, VIDIOC_QUERYCTRL, &q) == 0 && (value < q.minimum || value > q.maximum)) {
        fprintf(stderr, "%s %d is outside [%d, %d]\n", name, value, q.minimum, q.maximum);
        return false;
    }
    v4l2_control c = {};
    c.id = id;
    c.value = value;
    if (xioctl(fd, VIDIOC_S_CTRL, &c) < 0) {
        fprintf(stderr, "S_CTRL %s=%d: %s\n", name, value, strerror(errno));
        return false;
    }
    return true;
}

struct Buffer {
    void  *mem = MAP_FAILED;
    size_t len = 0;
};

/* One dequeued frame, its index; -1 on failure. */
int dequeue(int fd) {
    pollfd p = {fd, POLLIN, 0};
    if (poll(&p, 1, 3000) <= 0) {
        fprintf(stderr, "no frame in 3 s\n");
        return -1;
    }
    v4l2_buffer b = {};
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_DQBUF, &b) < 0) {
        fprintf(stderr, "DQBUF: %s\n", strerror(errno));
        return -1;
    }
    return (int)b.index;
}

bool queue(int fd, int index) {
    v4l2_buffer b = {};
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = (uint32_t)index;
    if (xioctl(fd, VIDIOC_QBUF, &b) < 0) {
        fprintf(stderr, "QBUF %d: %s\n", index, strerror(errno));
        return false;
    }
    return true;
}

/* A sample's level in a histogram, and what the histogram says. */
struct Phase {
    std::vector<uint64_t> hist = std::vector<uint64_t>(kLevels, 0);
    uint64_t n = 0;
    uint64_t sum = 0;

    int percentile(double p) const {
        const uint64_t want = (uint64_t)(p * (double)n);
        uint64_t seen = 0;
        for (int v = 0; v < kLevels; ++v) {
            seen += hist[v];
            if (seen > want) return v;
        }
        return kLevels - 1;
    }
};

} /* namespace */

int main(int argc, char **argv) {
    Options o;
    if (!parseArgs(argc, argv, &o)) {
        fprintf(stderr, "usage: rawcap --dev /dev/videoN [--width W --height H] "
                        "[--exposure-us N] [--gains a,b,...] [--frames N] [--skip N]\n");
        return 2;
    }

    const int fd = open(o.dev.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        fprintf(stderr, "open %s: %s\n", o.dev.c_str(), strerror(errno));
        return 1;
    }

    v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
        fprintf(stderr, "G_FMT: %s\n", strerror(errno));
        return 1;
    }
    fmt.fmt.pix.width = o.width;
    fmt.fmt.pix.height = o.height;
    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        fprintf(stderr, "S_FMT %ux%u: %s\n", o.width, o.height, strerror(errno));
        return 1;
    }
    const uint32_t fourcc = fmt.fmt.pix.pixelformat;
    const unsigned w = fmt.fmt.pix.width, h = fmt.fmt.pix.height;
    const unsigned stride = fmt.fmt.pix.bytesperline;
    printf("dev %s %ux%u fourcc %.4s stride %u\n", o.dev.c_str(), w, h,
           (const char *)&fourcc, stride);
    if (stride < w * 2) {
        fprintf(stderr, "not 16 bits a sample: stride %u for width %u\n", stride, w);
        return 1;
    }

    v4l2_requestbuffers req = {};
    req.count = kBuffers;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0 || req.count == 0) {
        fprintf(stderr, "REQBUFS: %s\n", strerror(errno));
        return 1;
    }
    std::vector<Buffer> bufs(req.count);
    for (uint32_t i = 0; i < req.count; ++i) {
        v4l2_buffer b = {};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;
        if (xioctl(fd, VIDIOC_QUERYBUF, &b) < 0) {
            fprintf(stderr, "QUERYBUF %u: %s\n", i, strerror(errno));
            return 1;
        }
        bufs[i].len = b.length;
        bufs[i].mem = mmap(nullptr, b.length, PROT_READ, MAP_SHARED, fd, b.m.offset);
        if (bufs[i].mem == MAP_FAILED) {
            fprintf(stderr, "mmap %u: %s\n", i, strerror(errno));
            return 1;
        }
        if (!queue(fd, (int)i)) return 1;
    }

    if (!setControl(fd, V4L2_CID_EXPOSURE, o.exposureUs, "exposure")) return 1;
    if (!setControl(fd, V4L2_CID_GAIN, o.gains[0], "gain")) return 1;

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        fprintf(stderr, "STREAMON: %s\n", strerror(errno));
        return 1;
    }

    int status = 0;
    printf("gain(Q8) phase   mean  median    p1     p99\n");
    for (int gain : o.gains) {
        if (!setControl(fd, V4L2_CID_GAIN, gain, "gain")) { status = 1; break; }
        Phase ph[4];
        bool ok = true;
        for (int f = 0; f < o.skip + o.frames && ok; ++f) {
            const int idx = dequeue(fd);
            if (idx < 0) { ok = false; break; }
            if (f >= o.skip) {
                const uint8_t *base = (const uint8_t *)bufs[idx].mem;
                for (unsigned y = 0; y < h; ++y) {
                    const uint16_t *row = (const uint16_t *)(base + (size_t)y * stride);
                    for (unsigned x = 0; x < w; ++x) {
                        const unsigned v = row[x] & (kLevels - 1);
                        Phase &p = ph[((y & 1) << 1) | (x & 1)];
                        ++p.hist[v];
                        ++p.n;
                        p.sum += v;
                    }
                }
            }
            ok = queue(fd, idx);
        }
        if (!ok) { status = 1; break; }
        static const char *names[4] = {"p00", "p01", "p10", "p11"};
        for (int i = 0; i < 4; ++i)
            printf("%8d %s %7.2f %6d %6d %6d\n", gain, names[i],
                   (double)ph[i].sum / (double)ph[i].n, ph[i].percentile(0.5),
                   ph[i].percentile(0.01), ph[i].percentile(0.99));
        fflush(stdout);
    }

    xioctl(fd, VIDIOC_STREAMOFF, &type);
    for (Buffer &b : bufs)
        if (b.mem != MAP_FAILED) munmap(b.mem, b.len);
    req.count = 0;
    xioctl(fd, VIDIOC_REQBUFS, &req);
    close(fd);
    return status;
}
