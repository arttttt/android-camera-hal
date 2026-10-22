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

#include "Exif.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <algorithm>
#include <numeric>

namespace android {

namespace {

enum Type : uint16_t {
    kByte      = 1,
    kAscii     = 2,
    kShort     = 3,
    kLong      = 4,
    kRational  = 5,
    kUndefined = 7,
};

void put16(std::vector<uint8_t> &v, uint16_t x) {
    v.push_back((uint8_t)(x & 0xff));
    v.push_back((uint8_t)(x >> 8));
}

void put32(std::vector<uint8_t> &v, uint32_t x) {
    for (int i = 0; i < 4; ++i) v.push_back((uint8_t)(x >> (8 * i)));
}

/* One IFD: its entries, sorted by tag as TIFF wants them, and the
 * values that do not fit an entry's four bytes after them, each on a
 * word boundary. */
class Ifd {
public:
    void ascii(uint16_t tag, const std::string &s) {
        std::vector<uint8_t> b(s.begin(), s.end());
        b.push_back(0);
        add(tag, kAscii, (uint32_t)b.size(), std::move(b));
    }
    void shortValue(uint16_t tag, uint16_t x) {
        std::vector<uint8_t> b;
        put16(b, x);
        add(tag, kShort, 1, std::move(b));
    }
    void longValue(uint16_t tag, uint32_t x) {
        std::vector<uint8_t> b;
        put32(b, x);
        add(tag, kLong, 1, std::move(b));
    }
    void rationals(uint16_t tag, std::initializer_list<std::pair<uint32_t, uint32_t>> r) {
        std::vector<uint8_t> b;
        for (const auto &[num, den] : r) { put32(b, num); put32(b, den); }
        add(tag, kRational, (uint32_t)r.size(), std::move(b));
    }
    void bytes(uint16_t tag, uint16_t type, std::vector<uint8_t> b) {
        const uint32_t n = (uint32_t)b.size();
        add(tag, type, n, std::move(b));
    }

    size_t size() const {
        size_t n = 2 + 12 * entries.size() + 4;
        for (const Entry &e : entries)
            if (e.data.size() > 4) n += (e.data.size() + 1) & ~(size_t)1;
        return n;
    }

    /* Written where the TIFF header is `at` bytes behind, the next IFD
     * at `next` (0: none). */
    void write(std::vector<uint8_t> &v, uint32_t at, uint32_t next) {
        std::sort(entries.begin(), entries.end(),
                  [](const Entry &a, const Entry &b) { return a.tag < b.tag; });
        uint32_t data = at + 2 + 12 * (uint32_t)entries.size() + 4;
        put16(v, (uint16_t)entries.size());
        for (const Entry &e : entries) {
            put16(v, e.tag);
            put16(v, e.type);
            put32(v, e.count);
            if (e.data.size() <= 4) {
                v.insert(v.end(), e.data.begin(), e.data.end());
                for (size_t i = e.data.size(); i < 4; ++i) v.push_back(0);
            } else {
                put32(v, data);
                data += (uint32_t)((e.data.size() + 1) & ~(size_t)1);
            }
        }
        put32(v, next);
        for (const Entry &e : entries) {
            if (e.data.size() <= 4) continue;
            v.insert(v.end(), e.data.begin(), e.data.end());
            if (e.data.size() & 1) v.push_back(0);
        }
    }

private:
    struct Entry {
        uint16_t tag;
        uint16_t type;
        uint32_t count;
        std::vector<uint8_t> data;
    };
    /* A tag given again replaces its value: a pointer to another IFD
     * is entered before that IFD's offset is known. */
    void add(uint16_t tag, uint16_t type, uint32_t count, std::vector<uint8_t> b) {
        for (Entry &e : entries)
            if (e.tag == tag) { e = {tag, type, count, std::move(b)}; return; }
        entries.push_back({tag, type, count, std::move(b)});
    }
    std::vector<Entry> entries;
};

/* JPEG_ORIENTATION (clockwise degrees) as the EXIF Orientation: how a
 * viewer turns the stored picture. */
uint16_t exifOrientation(int32_t degrees) {
    switch (degrees) {
        case 90:  return 6;
        case 180: return 3;
        case 270: return 8;
        default:  return 1;
    }
}

/* "YYYY:MM:DD HH:MM:SS" of `sec` shifted by `offsetMin` (the local time
 * when the offset is the zone's). */
std::string dateTime(int64_t sec, int32_t offsetMin) {
    const time_t t = (time_t)(sec + (int64_t)offsetMin * 60);
    struct tm tm;
    gmtime_r(&t, &tm);
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d:%02d:%02d %02d:%02d:%02d",
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec);
    return buf;
}

/* "+HH:MM" */
std::string offsetTime(int32_t offsetMin) {
    const int32_t a = abs(offsetMin);
    char buf[8];
    snprintf(buf, sizeof(buf), "%c%02d:%02d", offsetMin < 0 ? '-' : '+',
             a / 60 % 100, a % 60);
    return buf;
}

std::pair<uint32_t, uint32_t> reduced(uint64_t num, uint64_t den) {
    const uint64_t g = std::gcd(num, den);
    if (g > 1) { num /= g; den /= g; }
    while (num > UINT32_MAX || den > UINT32_MAX) { num /= 2; den /= 2; }
    return {(uint32_t)num, (uint32_t)(den ? den : 1)};
}

/* Degrees as degrees, minutes and seconds to 1/1000. */
void putDms(Ifd &ifd, uint16_t tag, double degrees) {
    const double a = fabs(degrees);
    const uint32_t d = (uint32_t)a;
    const double m = (a - d) * 60;
    const uint32_t mi = (uint32_t)m;
    const uint32_t s = (uint32_t)llround((m - mi) * 60 * 1000);
    ifd.rationals(tag, {{d, 1}, {mi, 1}, {s, 1000}});
}

constexpr uint32_t kTiffHeader = 8;
constexpr size_t   kIfd1Size   = 2 + 12 * 3 + 4;

void build(const ExifInfo &info, const uint8_t *thumb, size_t thumbLen,
           std::vector<uint8_t> *out) {
    const std::string when   = dateTime(info.timeSec, info.utcOffsetMin);
    const std::string offset = offsetTime(info.utcOffsetMin);
    char subSec[8];
    snprintf(subSec, sizeof(subSec), "%03d", info.subSecMs % 1000);

    Ifd ifd0, exif, gps;
    if (!info.make.empty())  ifd0.ascii(0x010F, info.make);
    if (!info.model.empty()) ifd0.ascii(0x0110, info.model);
    ifd0.shortValue(0x0112, exifOrientation(info.orientationDegrees));
    ifd0.rationals(0x011A, {{72, 1}});            /* XResolution */
    ifd0.rationals(0x011B, {{72, 1}});            /* YResolution */
    ifd0.shortValue(0x0128, 2);                   /* ResolutionUnit: inch */
    ifd0.ascii(0x0132, when);                     /* DateTime */
    ifd0.shortValue(0x0213, 1);                   /* YCbCrPositioning: centred */
    ifd0.longValue(0x8769, 0);                    /* the Exif IFD, set below */
    if (info.gps) ifd0.longValue(0x8825, 0);      /* the GPS IFD, set below */

    if (info.exposureNs && *info.exposureNs > 0)
        exif.rationals(0x829A, {reduced((uint64_t)*info.exposureNs, 1000000000ull)});
    if (info.fNumber && *info.fNumber > 0)
        exif.rationals(0x829D, {{(uint32_t)lroundf(*info.fNumber * 100), 100}});
    exif.shortValue(0x8822, info.manualExposure ? 1 : 2);  /* ExposureProgram */
    if (info.iso && *info.iso > 0)
        exif.shortValue(0x8827, (uint16_t)std::min(*info.iso, 65535));
    exif.bytes(0x9000, kUndefined, {'0', '2', '3', '2'});  /* ExifVersion */
    exif.ascii(0x9003, when);                     /* DateTimeOriginal */
    exif.ascii(0x9004, when);                     /* DateTimeDigitized */
    exif.ascii(0x9010, offset);                   /* OffsetTime */
    exif.ascii(0x9011, offset);                   /* OffsetTimeOriginal */
    exif.ascii(0x9012, offset);                   /* OffsetTimeDigitized */
    exif.bytes(0x9101, kUndefined, {1, 2, 3, 0}); /* ComponentsConfiguration: YCbCr */
    exif.shortValue(0x9209, 0x20);                /* Flash: no flash function */
    if (info.focalLengthMm && *info.focalLengthMm > 0)
        exif.rationals(0x920A, {{(uint32_t)lroundf(*info.focalLengthMm * 1000), 1000}});
    exif.ascii(0x9290, subSec);                   /* SubSecTime */
    exif.ascii(0x9291, subSec);                   /* SubSecTimeOriginal */
    exif.ascii(0x9292, subSec);                   /* SubSecTimeDigitized */
    exif.bytes(0xA000, kUndefined, {'0', '1', '0', '0'});  /* FlashpixVersion */
    exif.shortValue(0xA001, 1);                   /* ColorSpace: sRGB */
    exif.longValue(0xA002, info.width);           /* PixelXDimension */
    exif.longValue(0xA003, info.height);          /* PixelYDimension */
    exif.shortValue(0xA402, info.manualExposure ? 1 : 0);      /* ExposureMode */
    exif.shortValue(0xA403, info.manualWhiteBalance ? 1 : 0);  /* WhiteBalance */

    if (info.gps) {
        const ExifInfo::Gps &g = *info.gps;
        gps.bytes(0x0000, kByte, {2, 3, 0, 0});   /* GPSVersionID */
        gps.ascii(0x0001, g.latitude < 0 ? "S" : "N");
        putDms(gps, 0x0002, g.latitude);
        gps.ascii(0x0003, g.longitude < 0 ? "W" : "E");
        putDms(gps, 0x0004, g.longitude);
        gps.bytes(0x0005, kByte, {(uint8_t)(g.altitude < 0 ? 1 : 0)});
        gps.rationals(0x0006, {{(uint32_t)llround(fabs(g.altitude) * 1000), 1000}});
        const time_t t = (time_t)g.timestampSec;
        struct tm tm;
        gmtime_r(&t, &tm);
        gps.rationals(0x0007, {{(uint32_t)tm.tm_hour, 1}, {(uint32_t)tm.tm_min, 1},
                               {(uint32_t)tm.tm_sec, 1}});
        if (!g.processingMethod.empty()) {
            std::vector<uint8_t> m = {'A', 'S', 'C', 'I', 'I', 0, 0, 0};
            m.insert(m.end(), g.processingMethod.begin(), g.processingMethod.end());
            gps.bytes(0x001B, kUndefined, std::move(m));
        }
        char date[16];
        snprintf(date, sizeof(date), "%04d:%02d:%02d",
                 tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);
        gps.ascii(0x001D, date);
    }

    const uint32_t ifd0At = kTiffHeader;
    const uint32_t exifAt = ifd0At + (uint32_t)ifd0.size();
    const uint32_t gpsAt  = exifAt + (uint32_t)exif.size();
    const uint32_t ifd1At = gpsAt + (info.gps ? (uint32_t)gps.size() : 0);
    ifd0.longValue(0x8769, exifAt);
    if (info.gps) ifd0.longValue(0x8825, gpsAt);

    std::vector<uint8_t> &v = *out;
    v.clear();
    const uint8_t hdr[] = { 'E', 'x', 'i', 'f', 0x00, 0x00 };
    v.insert(v.end(), hdr, hdr + sizeof(hdr));
    v.push_back('I'); v.push_back('I');
    put16(v, 0x002A);
    put32(v, ifd0At);

    std::vector<uint8_t> tiff;
    ifd0.write(tiff, ifd0At, thumb ? ifd1At : 0);
    exif.write(tiff, exifAt, 0);
    if (info.gps) gps.write(tiff, gpsAt, 0);
    v.insert(v.end(), tiff.begin(), tiff.end());

    if (!thumb) return;
    /* IFD1: the thumbnail, right after it. */
    const uint32_t thumbAt = ifd1At + (uint32_t)kIfd1Size;
    put16(v, 3);
    const auto entry = [&v](uint16_t tag, uint16_t type, uint32_t value) {
        put16(v, tag); put16(v, type); put32(v, 1);
        if (type == kShort) { put16(v, (uint16_t)value); put16(v, 0); }
        else put32(v, value);
    };
    entry(0x0103, kShort, 6);                     /* Compression: JPEG */
    entry(0x0201, kLong, thumbAt);                /* JPEGInterchangeFormat */
    entry(0x0202, kLong, (uint32_t)thumbLen);     /* ...Length */
    put32(v, 0);
    v.insert(v.end(), thumb, thumb + thumbLen);
}

} /* namespace */

void buildExifApp1(const ExifInfo &info, const uint8_t *thumb, size_t thumbLen,
                   std::vector<uint8_t> *out) {
    build(info, thumb, thumbLen, out);
}

size_t exifOverhead(const ExifInfo &info) {
    std::vector<uint8_t> v;
    build(info, nullptr, 0, &v);
    return v.size() + kIfd1Size;
}

} /* namespace android */
