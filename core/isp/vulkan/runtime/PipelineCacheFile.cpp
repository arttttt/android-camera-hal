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

#include "PipelineCacheFile.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <string>

namespace android {

namespace {

constexpr uint32_t kMagic   = 0x4350434d;  /* "MCPC" */
constexpr uint32_t kVersion = 1;

/* magic, version, key, data size */
constexpr size_t kHeaderSize = 4 + 4 + 8 + 4;

/* VkPipelineCacheHeaderVersionOne: header size, header version, vendor
 * id, device id, the cache UUID. */
constexpr size_t   kVkHeaderSize      = 4 + 4 + 4 + 4 + 16;
constexpr uint32_t kVkHeaderVersionOne = 1;

template <typename T>
void put(std::vector<uint8_t> &out, T v) {
    const uint8_t *p = reinterpret_cast<const uint8_t *>(&v);
    out.insert(out.end(), p, p + sizeof(v));
}

template <typename T>
T get(const uint8_t *p) {
    T v;
    memcpy(&v, p, sizeof(v));
    return v;
}

} /* namespace */

uint64_t pipelineSourcesKey(std::initializer_list<const char *> sources) {
    uint64_t h = 0xcbf29ce484222325ull;
    for (const char *s : sources) {
        /* The terminator too, so ("ab","c") and ("a","bc") differ. */
        for (const char *c = s;; ++c) {
            h ^= (uint8_t)*c;
            h *= 0x100000001b3ull;
            if (!*c) break;
        }
    }
    return h;
}

std::vector<uint8_t> encodePipelineCache(uint64_t key, const std::vector<uint8_t> &data) {
    std::vector<uint8_t> out;
    out.reserve(kHeaderSize + data.size());
    put<uint32_t>(out, kMagic);
    put<uint32_t>(out, kVersion);
    put<uint64_t>(out, key);
    put<uint32_t>(out, (uint32_t)data.size());
    out.insert(out.end(), data.begin(), data.end());
    return out;
}

std::vector<uint8_t> decodePipelineCache(const std::vector<uint8_t> &file, uint64_t key,
                                         const PipelineCacheDevice &device) {
    if (file.size() < kHeaderSize) return {};
    const uint8_t *p = file.data();
    if (get<uint32_t>(p) != kMagic || get<uint32_t>(p + 4) != kVersion ||
        get<uint64_t>(p + 8) != key)
        return {};
    const uint32_t size = get<uint32_t>(p + 16);
    if (size != file.size() - kHeaderSize || size < kVkHeaderSize) return {};

    const uint8_t *vk = p + kHeaderSize;
    const uint32_t vkHeaderSize = get<uint32_t>(vk);
    if (vkHeaderSize < kVkHeaderSize || vkHeaderSize > size ||
        get<uint32_t>(vk + 4) != kVkHeaderVersionOne ||
        get<uint32_t>(vk + 8) != device.vendorId ||
        get<uint32_t>(vk + 12) != device.deviceId ||
        memcmp(vk + 16, device.uuid, sizeof(device.uuid)) != 0)
        return {};
    return std::vector<uint8_t>(vk, vk + size);
}

std::vector<uint8_t> readPipelineCacheFile(const char *path) {
    const int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return {};
    std::vector<uint8_t> bytes;
    uint8_t chunk[16384];
    for (;;) {
        const ssize_t n = read(fd, chunk, sizeof(chunk));
        if (n < 0 && errno == EINTR) continue;
        if (n < 0) { bytes.clear(); break; }
        if (n == 0) break;
        bytes.insert(bytes.end(), chunk, chunk + n);
    }
    close(fd);
    return bytes;
}

bool writePipelineCacheFile(const char *path, const std::vector<uint8_t> &bytes) {
    const std::string tmp = std::string(path) + ".tmp";
    const int fd = open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
    if (fd < 0) return false;
    size_t done = 0;
    while (done < bytes.size()) {
        const ssize_t n = write(fd, bytes.data() + done, bytes.size() - done);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) break;
        done += (size_t)n;
    }
    const bool written = done == bytes.size() && fsync(fd) == 0;
    close(fd);
    if (!written || rename(tmp.c_str(), path) != 0) {
        unlink(tmp.c_str());
        return false;
    }
    return true;
}

}; /* namespace android */
