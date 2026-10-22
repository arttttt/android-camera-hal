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

/* Host test for the EXIF APP1 payload: read back as a TIFF reader
 * would -- the header, each IFD's entries in tag order, values out of
 * line at their offsets -- it says what it was given: the date
 * MediaStore takes DATE_TAKEN from, the frame's exposure and ISO, the
 * location, the thumbnail. Run by core/tests/run_host_tests.sh. */

#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "jpeg/Exif.h"
#include "Check.h"

using android::ExifInfo;
using android::buildExifApp1;
using android::exifOverhead;

using mocha::camera::test::check;
using mocha::camera::test::finish;

namespace {

struct Entry {
    uint16_t type  = 0;
    uint32_t count = 0;
    std::vector<uint8_t> data;   /* the value's bytes, wherever they were */
};

/* A little-endian TIFF read back from the APP1 payload. */
class Tiff {
public:
    explicit Tiff(const std::vector<uint8_t> &app1) {
        ok = app1.size() > 14 && memcmp(app1.data(), "Exif\0\0", 6) == 0;
        if (!ok) return;
        t.assign(app1.begin() + 6, app1.end());
        ok = t[0] == 'I' && t[1] == 'I' && u16(2) == 42;
    }

    bool ok = false;

    uint16_t u16(size_t at) const { return (uint16_t)(t[at] | t[at + 1] << 8); }
    uint32_t u32(size_t at) const {
        return (uint32_t)t[at] | (uint32_t)t[at + 1] << 8 |
               (uint32_t)t[at + 2] << 16 | (uint32_t)t[at + 3] << 24;
    }

    /* The IFD at `at`: its entries, and the offset of the next IFD.
     * `sorted` false if its tags are not in ascending order; `inside`
     * false if any value lies outside the TIFF. */
    std::map<uint16_t, Entry> ifd(uint32_t at, uint32_t *next = nullptr) {
        std::map<uint16_t, Entry> m;
        if (at + 2 > t.size()) { inside = false; return m; }
        const uint16_t n = u16(at);
        int last = -1;
        for (uint16_t i = 0; i < n; ++i) {
            const size_t e = at + 2 + 12 * (size_t)i;
            if (e + 12 > t.size()) { inside = false; return m; }
            const uint16_t tag = u16(e);
            if ((int)tag <= last) sorted = false;
            last = tag;
            Entry x;
            x.type  = u16(e + 2);
            x.count = u32(e + 4);
            const size_t unit = x.type == 3 ? 2 : x.type == 4 ? 4 : x.type == 5 ? 8 : 1;
            const size_t len  = unit * x.count;
            size_t from = e + 8;
            if (len > 4) {
                from = u32(e + 8);
                if (from & 1) aligned = false;
            }
            if (from + len > t.size()) { inside = false; continue; }
            x.data.assign(t.begin() + from, t.begin() + from + len);
            m[tag] = x;
        }
        if (next) *next = u32(at + 2 + 12 * (size_t)n);
        return m;
    }

    std::vector<uint8_t> t;
    bool sorted  = true;
    bool inside  = true;
    bool aligned = true;
};

std::string ascii(const std::map<uint16_t, Entry> &m, uint16_t tag) {
    auto it = m.find(tag);
    if (it == m.end() || it->second.type != 2 || it->second.data.empty()) return "<none>";
    return std::string(it->second.data.begin(), it->second.data.end() - 1);
}

uint32_t number(const std::map<uint16_t, Entry> &m, uint16_t tag) {
    auto it = m.find(tag);
    if (it == m.end()) return 0xFFFFFFFF;
    const std::vector<uint8_t> &d = it->second.data;
    if (it->second.type == 3) return (uint32_t)(d[0] | d[1] << 8);
    return (uint32_t)d[0] | (uint32_t)d[1] << 8 | (uint32_t)d[2] << 16 | (uint32_t)d[3] << 24;
}

/* The `i`-th rational, as numerator / denominator. */
double rational(const std::map<uint16_t, Entry> &m, uint16_t tag, size_t i = 0) {
    auto it = m.find(tag);
    if (it == m.end() || it->second.type != 5 || it->second.count <= i) return -1;
    const uint8_t *d = it->second.data.data() + 8 * i;
    const uint32_t num = (uint32_t)d[0] | (uint32_t)d[1] << 8 | (uint32_t)d[2] << 16 | (uint32_t)d[3] << 24;
    const uint32_t den = (uint32_t)d[4] | (uint32_t)d[5] << 8 | (uint32_t)d[6] << 16 | (uint32_t)d[7] << 24;
    return den ? (double)num / den : -1;
}

bool near(double a, double b, double eps) { return a > b - eps && a < b + eps; }

ExifInfo picture() {
    ExifInfo e;
    e.orientationDegrees = 90;
    e.width  = 3264;
    e.height = 2448;
    e.timeSec      = 1792724400;   /* 2026-10-23 03:00:00 UTC */
    e.subSecMs     = 42;
    e.utcOffsetMin = 180;
    e.make  = "Xiaomi";
    e.model = "MI PAD";
    e.exposureNs    = 800000000;
    e.iso           = 400;
    e.fNumber       = 2.0f;
    e.focalLengthMm = 3.3f;
    e.manualExposure = true;
    return e;
}

/* Every tag MediaStore and a gallery read, at what the picture was
 * given; the date local, with its zone and milliseconds. */
void testTags() {
    std::vector<uint8_t> app1;
    buildExifApp1(picture(), nullptr, 0, &app1);
    Tiff t(app1);
    check(t.ok);
    uint32_t next = 1;
    const auto ifd0 = t.ifd(t.u32(4), &next);
    check(next == 0);
    check(ascii(ifd0, 0x010F) == "Xiaomi" && ascii(ifd0, 0x0110) == "MI PAD");
    check(number(ifd0, 0x0112) == 6);
    check(ascii(ifd0, 0x0132) == "2026:10:23 06:00:00");
    check(ifd0.count(0x8825) == 0);

    const auto exif = t.ifd(number(ifd0, 0x8769));
    check(ascii(exif, 0x9003) == "2026:10:23 06:00:00");
    check(ascii(exif, 0x9004) == "2026:10:23 06:00:00");
    check(ascii(exif, 0x9011) == "+03:00");
    check(ascii(exif, 0x9291) == "042");
    check(near(rational(exif, 0x829A), 0.8, 1e-9));
    check(number(exif, 0x8827) == 400);
    check(near(rational(exif, 0x829D), 2.0, 1e-9));
    check(near(rational(exif, 0x920A), 3.3, 1e-9));
    check(number(exif, 0x9209) == 0x20);
    check(number(exif, 0xA002) == 3264 && number(exif, 0xA003) == 2448);
    check(number(exif, 0xA402) == 1 && number(exif, 0xA403) == 0);
    check(t.sorted && t.inside && t.aligned);
}

/* A zone west of UTC, a short exposure: the date goes back a day where
 * it is, and the exposure stays exact. */
void testWestAndShort() {
    ExifInfo e = picture();
    e.timeSec      = 1792728000;   /* 2026-10-23 04:00:00 UTC */
    e.utcOffsetMin = -(5 * 60 + 30);
    e.exposureNs   = 33333000;
    std::vector<uint8_t> app1;
    buildExifApp1(e, nullptr, 0, &app1);
    Tiff t(app1);
    const auto ifd0 = t.ifd(t.u32(4));
    const auto exif = t.ifd(number(ifd0, 0x8769));
    check(ascii(exif, 0x9003) == "2026:10:22 22:30:00");
    check(ascii(exif, 0x9011) == "-05:30");
    check(near(rational(exif, 0x829A), 0.033333, 1e-9));
}

/* The request's location: south and west, below the sea, its UTC time
 * and date, and how it was found. */
void testGps() {
    ExifInfo e = picture();
    ExifInfo::Gps g;
    g.latitude  = -33.8568;
    g.longitude = -151.2153;
    g.altitude  = -12.5;
    g.timestampSec = 1792724461;   /* 2026-10-23 03:01:01 UTC */
    g.processingMethod = "GPS";
    e.gps = g;
    std::vector<uint8_t> app1;
    buildExifApp1(e, nullptr, 0, &app1);
    Tiff t(app1);
    const auto ifd0 = t.ifd(t.u32(4));
    const auto gps  = t.ifd(number(ifd0, 0x8825));
    check(ascii(gps, 0x0001) == "S" && ascii(gps, 0x0003) == "W");
    const double lat = rational(gps, 0x0002, 0) + rational(gps, 0x0002, 1) / 60 +
                       rational(gps, 0x0002, 2) / 3600;
    const double lon = rational(gps, 0x0004, 0) + rational(gps, 0x0004, 1) / 60 +
                       rational(gps, 0x0004, 2) / 3600;
    check(near(lat, 33.8568, 1e-6) && near(lon, 151.2153, 1e-6));
    check(gps.count(0x0005) && gps.at(0x0005).data[0] == 1);
    check(near(rational(gps, 0x0006), 12.5, 1e-9));
    check(rational(gps, 0x0007, 0) == 3 && rational(gps, 0x0007, 1) == 1 &&
          rational(gps, 0x0007, 2) == 1);
    check(ascii(gps, 0x001D) == "2026:10:23");
    const std::vector<uint8_t> &m = gps.at(0x001B).data;
    check(m.size() == 11 && memcmp(m.data(), "ASCII\0\0\0GPS", 11) == 0);
    check(t.sorted && t.inside && t.aligned);
}

/* The thumbnail sits in IFD1, where its offset says, and the overhead
 * is all the payload but the thumbnail. */
void testThumbnail() {
    const std::vector<uint8_t> thumb = {0xFF, 0xD8, 1, 2, 3, 4, 5, 0xFF, 0xD9};
    std::vector<uint8_t> app1;
    buildExifApp1(picture(), thumb.data(), thumb.size(), &app1);
    check(app1.size() == exifOverhead(picture()) + thumb.size());
    Tiff t(app1);
    uint32_t ifd1 = 0;
    t.ifd(t.u32(4), &ifd1);
    check(ifd1 != 0);
    const auto m = t.ifd(ifd1);
    check(number(m, 0x0103) == 6);
    const uint32_t at = number(m, 0x0201);
    check(number(m, 0x0202) == thumb.size());
    check(at + thumb.size() <= t.t.size() &&
          memcmp(t.t.data() + at, thumb.data(), thumb.size()) == 0);
}

} // namespace

int main() {
    testTags();
    testWestAndShort();
    testGps();
    testThumbnail();
    return finish("exif_test");
}
