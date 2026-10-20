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

#define LOG_TAG "Cam-MediaGraph"

#include "MediaGraph.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <unistd.h>

#include <linux/media.h>
#include <utils/Log.h>

namespace android {
namespace MediaGraph {

namespace {

/* Find the source entity of an incoming link to `sinkId`. The kernel
 * `MEDIA_IOC_ENUM_LINKS` ioctl reports outgoing links only, so we
 * walk every entity and check which one's outgoing list contains a
 * link landing on `sinkId`. Used twice in lensSubdevFor to step the
 * graph (video ← sensor ← lens). Returns 0 if no such link exists. */
uint32_t mediaSinkSource(int mediaFd, uint32_t sinkId)
{
    struct media_entity_desc ent;
    uint32_t id = 0;
    while (1) {
        memset(&ent, 0, sizeof(ent));
        ent.id = id | MEDIA_ENT_ID_FLAG_NEXT;
        if (ioctl(mediaFd, MEDIA_IOC_ENUM_ENTITIES, &ent) < 0)
            break;
        id = ent.id;
        if (ent.id == sinkId || ent.links == 0)
            continue;

        struct media_pad_desc  pads[16];
        struct media_link_desc lnks[64];
        struct media_links_enum links;
        memset(pads, 0, sizeof(pads));
        memset(lnks, 0, sizeof(lnks));
        memset(&links, 0, sizeof(links));
        links.entity = ent.id;
        links.pads = pads;
        links.links = lnks;
        if (ioctl(mediaFd, MEDIA_IOC_ENUM_LINKS, &links) < 0)
            continue;

        for (unsigned i = 0; i < ent.links; i++)
            if (lnks[i].sink.entity == sinkId)
                return ent.id;
    }
    return 0;
}

} /* namespace */

std::string lensSubdevFor(const char *videoNode)
{
    if (!videoNode) {
        ALOGW("lensSubdevFor: no video device path");
        return std::string();
    }

    struct stat st;
    if (stat(videoNode, &st) < 0) {
        ALOGW("lensSubdevFor: stat(%s): %s", videoNode, strerror(errno));
        return std::string();
    }
    const unsigned videoMajor = major(st.st_rdev);
    const unsigned videoMinor = minor(st.st_rdev);
    ALOGI("lensSubdevFor: video dev %s -> %u:%u",
          videoNode, videoMajor, videoMinor);

    int mediaFd = open("/dev/media0", O_RDWR);
    if (mediaFd < 0) {
        ALOGW("lensSubdevFor: open /dev/media0: %s", strerror(errno));
        return std::string();
    }

    /* Find the entity whose dev matches our open video device. */
    uint32_t videoId = 0;
    struct media_entity_desc ent;
    uint32_t id = 0;
    while (1) {
        memset(&ent, 0, sizeof(ent));
        ent.id = id | MEDIA_ENT_ID_FLAG_NEXT;
        if (ioctl(mediaFd, MEDIA_IOC_ENUM_ENTITIES, &ent) < 0)
            break;
        id = ent.id;
        if (ent.v4l.major == videoMajor && ent.v4l.minor == videoMinor) {
            videoId = ent.id;
            break;
        }
    }
    if (!videoId) {
        ALOGW("lensSubdevFor: no media entity for %u:%u",
              videoMajor, videoMinor);
        close(mediaFd);
        return std::string();
    }
    ALOGI("lensSubdevFor: video entity id=%u name='%s'", videoId, ent.name);

    /* Walk video <- sensor <- lens. */
    const uint32_t sensorId = mediaSinkSource(mediaFd, videoId);
    const uint32_t lensId   = sensorId ? mediaSinkSource(mediaFd, sensorId) : 0;
    ALOGI("lensSubdevFor: sensor entity id=%u, lens entity id=%u",
          sensorId, lensId);
    if (!lensId) {
        ALOGW("lensSubdevFor: no lens link from sensor (video=%u sensor=%u)",
              videoId, sensorId);
        close(mediaFd);
        return std::string();
    }

    /* Read the lens entity to get its dev(major:minor). */
    memset(&ent, 0, sizeof(ent));
    ent.id = lensId;
    if (ioctl(mediaFd, MEDIA_IOC_ENUM_ENTITIES, &ent) < 0) {
        ALOGW("lensSubdevFor: ENUM_ENTITIES(lens=%u): %s",
              lensId, strerror(errno));
        close(mediaFd);
        return std::string();
    }
    close(mediaFd);

    const unsigned lensMajor = ent.v4l.major;
    const unsigned lensMinor = ent.v4l.minor;
    ALOGI("lensSubdevFor: lens entity '%s' dev=%u:%u",
          ent.name, lensMajor, lensMinor);

    /* Find /dev/v4l-subdev* with the lens's dev. */
    DIR *dir = opendir("/dev");
    if (!dir) {
        ALOGE("lensSubdevFor: opendir /dev: %s", strerror(errno));
        return std::string();
    }
    struct dirent *de;
    while ((de = readdir(dir)) != nullptr) {
        if (strncmp(de->d_name, "v4l-subdev", 10) != 0)
            continue;
        char path[64];
        snprintf(path, sizeof(path), "/dev/%s", de->d_name);
        if (stat(path, &st) < 0)
            continue;
        if (major(st.st_rdev) != lensMajor || minor(st.st_rdev) != lensMinor)
            continue;
        closedir(dir);
        return std::string(path);
    }
    closedir(dir);
    ALOGW("lensSubdevFor: no /dev/v4l-subdev* with dev %u:%u",
          lensMajor, lensMinor);
    return std::string();
}

} /* namespace MediaGraph */
} /* namespace android */
