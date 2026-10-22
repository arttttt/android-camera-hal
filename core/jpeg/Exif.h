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

#ifndef HAL_JPEG_EXIF_H
#define HAL_JPEG_EXIF_H

#include <stddef.h>
#include <stdint.h>

#include <optional>
#include <string>
#include <vector>

namespace android {

/* What a JPEG's EXIF says about its picture. The date is when the
 * picture was encoded, by the wall clock, as libcamera's Android
 * adapter and the emulated camera date theirs: MediaStore takes the
 * picture's DATE_TAKEN from DateTimeOriginal, and a picture without one
 * sorts apart from the rest. */
struct ExifInfo {
    /* JPEG_ORIENTATION, clockwise degrees. */
    int32_t  orientationDegrees = 0;
    uint32_t width  = 0;
    uint32_t height = 0;

    /* Seconds since the epoch, the milliseconds past them, and the
     * local zone's offset from UTC in minutes. */
    int64_t  timeSec      = 0;
    int32_t  subSecMs     = 0;
    int32_t  utcOffsetMin = 0;

    std::string make;
    std::string model;

    /* What the frame ran on; absent: not written. */
    std::optional<int64_t> exposureNs;
    std::optional<int32_t> iso;
    std::optional<float>   fNumber;
    std::optional<float>   focalLengthMm;
    /* The exposure and the white balance were the request's own. */
    bool manualExposure     = false;
    bool manualWhiteBalance = false;

    /* JPEG_GPS_*, when the request gave a location. */
    struct Gps {
        double      latitude  = 0;   /* degrees, north positive */
        double      longitude = 0;   /* degrees, east positive */
        double      altitude  = 0;   /* metres above sea level */
        int64_t     timestampSec = 0; /* UTC seconds since the epoch */
        std::string processingMethod;
    };
    std::optional<Gps> gps;
};

/* The EXIF APP1 payload ("Exif\0\0", the TIFF header, IFD0, the Exif
 * IFD, the GPS IFD when there is a location, and IFD1 with the JPEG
 * thumbnail when `thumb` is given). Little-endian. */
void buildExifApp1(const ExifInfo &info,
                   const uint8_t *thumb, size_t thumbLen,
                   std::vector<uint8_t> *out);

/* How many bytes buildExifApp1 adds to a thumbnail of the same info:
 * what an APP1 marker leaves for it. */
size_t exifOverhead(const ExifInfo &info);

} /* namespace android */

#endif /* HAL_JPEG_EXIF_H */
