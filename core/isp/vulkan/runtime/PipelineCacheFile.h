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

#ifndef VULKAN_PIPELINE_CACHE_FILE_H
#define VULKAN_PIPELINE_CACHE_FILE_H

#include <stdint.h>

#include <initializer_list>
#include <vector>

namespace android {

/* The device a pipeline cache's data was made on, as the data's own
 * Vulkan header (VkPipelineCacheHeaderVersionOne) names it: data from
 * another driver or GPU is no use to this one. */
struct PipelineCacheDevice {
    uint32_t vendorId;
    uint32_t deviceId;
    uint8_t  uuid[16];
};

/* The key of the pipelines a cache holds: FNV-1a over their shaders'
 * sources. A file for other sources is dropped whole, so pipelines
 * the HAL no longer builds do not pile up in it. */
uint64_t pipelineSourcesKey(std::initializer_list<const char *> sources);

/* The file: our header (a magic, the format's version, the key, the
 * data's size), then the data as the driver gave it. */
std::vector<uint8_t> encodePipelineCache(uint64_t key, const std::vector<uint8_t> &data);

/* The data from a file's bytes; empty when the file is not this
 * format, is for another key, is cut short, or its data is not for
 * `device`. */
std::vector<uint8_t> decodePipelineCache(const std::vector<uint8_t> &file, uint64_t key,
                                         const PipelineCacheDevice &device);

/* The whole file; empty when it cannot be read. */
std::vector<uint8_t> readPipelineCacheFile(const char *path);

/* Replaces the file whole: written beside it, then renamed over it, so
 * a reader sees the old file or the new one, never half. */
bool writePipelineCacheFile(const char *path, const std::vector<uint8_t> &bytes);

}; /* namespace android */

#endif /* VULKAN_PIPELINE_CACHE_FILE_H */
