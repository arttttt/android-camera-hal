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

/* Host test for the pipeline cache's file: what is kept is what comes
 * back, and a file for other shaders, another device, or cut short is
 * not taken. */

#include "isp/vulkan/runtime/PipelineCacheFile.h"
#include "Check.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <string>

using android::PipelineCacheDevice;
using android::decodePipelineCache;
using android::encodePipelineCache;
using android::pipelineSourcesKey;
using android::readPipelineCacheFile;
using android::writePipelineCacheFile;
using mocha::camera::test::check;
using mocha::camera::test::finish;

namespace {

const PipelineCacheDevice kDevice = {0x10de, 0xea, {1, 2, 3, 4, 5, 6, 7, 8,
                                                    9, 10, 11, 12, 13, 14, 15, 16}};

/* Data as a driver gives it: the Vulkan header naming `device`, then
 * the driver's own bytes. */
std::vector<uint8_t> driverData(const PipelineCacheDevice &device, size_t body) {
    std::vector<uint8_t> d(32 + body, 0);
    const uint32_t header[4] = {32, 1, device.vendorId, device.deviceId};
    memcpy(d.data(), header, sizeof(header));
    memcpy(d.data() + 16, device.uuid, 16);
    for (size_t i = 0; i < body; i++) d[32 + i] = (uint8_t)(i * 7);
    return d;
}

void testRoundTrip() {
    const uint64_t key = pipelineSourcesKey({"compute", "blit"});
    const std::vector<uint8_t> data = driverData(kDevice, 1000);
    check(decodePipelineCache(encodePipelineCache(key, data), key, kDevice) == data);
}

void testOtherSources() {
    const std::vector<uint8_t> data = driverData(kDevice, 100);
    const std::vector<uint8_t> file =
        encodePipelineCache(pipelineSourcesKey({"compute", "blit"}), data);
    check(decodePipelineCache(file, pipelineSourcesKey({"compute v2", "blit"}), kDevice).empty());
    /* Where one source ends and the next begins counts. */
    check(pipelineSourcesKey({"ab", "c"}) != pipelineSourcesKey({"a", "bc"}));
}

void testOtherDevice() {
    const uint64_t key = pipelineSourcesKey({"compute"});
    const std::vector<uint8_t> file = encodePipelineCache(key, driverData(kDevice, 100));
    PipelineCacheDevice other = kDevice;
    other.uuid[15] ^= 1;
    check(decodePipelineCache(file, key, other).empty());
    other = kDevice;
    other.deviceId++;
    check(decodePipelineCache(file, key, other).empty());
}

void testDamaged() {
    const uint64_t key = pipelineSourcesKey({"compute"});
    std::vector<uint8_t> file = encodePipelineCache(key, driverData(kDevice, 100));
    std::vector<uint8_t> cut(file.begin(), file.end() - 1);
    check(decodePipelineCache(cut, key, kDevice).empty());
    check(decodePipelineCache({}, key, kDevice).empty());
    file[0] ^= 0xff;
    check(decodePipelineCache(file, key, kDevice).empty());
    /* Too short to hold the Vulkan header. */
    check(decodePipelineCache(encodePipelineCache(key, {1, 2, 3}), key, kDevice).empty());
}

void testFile() {
    char dir[] = "/tmp/pipeline_cache_testXXXXXX";
    check(mkdtemp(dir) != nullptr);
    const std::string path = std::string(dir) + "/cache.bin";
    check(readPipelineCacheFile(path.c_str()).empty());

    const std::vector<uint8_t> first(70000, 0x5a);
    check(writePipelineCacheFile(path.c_str(), first));
    check(readPipelineCacheFile(path.c_str()) == first);
    const std::vector<uint8_t> second(10, 0xa5);
    check(writePipelineCacheFile(path.c_str(), second));
    check(readPipelineCacheFile(path.c_str()) == second);
    /* Nothing is left beside it. */
    check(access((path + ".tmp").c_str(), F_OK) != 0);

    unlink(path.c_str());
    rmdir(dir);
    check(!writePipelineCacheFile((std::string(dir) + "/cache.bin").c_str(), first));
}

} /* namespace */

int main() {
    testRoundTrip();
    testOtherSources();
    testOtherDevice();
    testDamaged();
    testFile();
    return finish("pipeline_cache_file_test");
}
