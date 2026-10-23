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
 * link landing on `sinkId`. Steps the graph upstream (video ← sensor
 * ← lens). Returns 0 if no such link exists. */
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

/* The entity whose device is `videoNode`'s; 0 when there is none. */
uint32_t videoEntity(int mediaFd, const char *who, const char *videoNode)
{
    struct stat st;
    if (stat(videoNode, &st) < 0) {
        ALOGW("%s: stat(%s): %s", who, videoNode, strerror(errno));
        return 0;
    }
    const unsigned videoMajor = major(st.st_rdev);
    const unsigned videoMinor = minor(st.st_rdev);

    struct media_entity_desc ent;
    uint32_t id = 0;
    while (1) {
        memset(&ent, 0, sizeof(ent));
        ent.id = id | MEDIA_ENT_ID_FLAG_NEXT;
        if (ioctl(mediaFd, MEDIA_IOC_ENUM_ENTITIES, &ent) < 0)
            break;
        id = ent.id;
        if (ent.v4l.major == videoMajor && ent.v4l.minor == videoMinor) {
            ALOGI("%s: %s is entity id=%u name='%s'", who, videoNode,
                  ent.id, ent.name);
            return ent.id;
        }
    }
    ALOGW("%s: no media entity for %s (%u:%u)", who, videoNode,
          videoMajor, videoMinor);
    return 0;
}

/* The /dev/v4l-subdev* node of device major:minor; empty when none. */
std::string subdevNode(const char *who, unsigned devMajor, unsigned devMinor)
{
    DIR *dir = opendir("/dev");
    if (!dir) {
        ALOGE("%s: opendir /dev: %s", who, strerror(errno));
        return std::string();
    }
    struct dirent *de;
    while ((de = readdir(dir)) != nullptr) {
        if (strncmp(de->d_name, "v4l-subdev", 10) != 0)
            continue;
        char path[64];
        snprintf(path, sizeof(path), "/dev/%s", de->d_name);
        struct stat st;
        if (stat(path, &st) < 0)
            continue;
        if (major(st.st_rdev) != devMajor || minor(st.st_rdev) != devMinor)
            continue;
        closedir(dir);
        return std::string(path);
    }
    closedir(dir);
    ALOGW("%s: no /dev/v4l-subdev* with dev %u:%u", who, devMajor, devMinor);
    return std::string();
}

/* The subdev node of the entity `steps` links upstream of `videoNode`
 * (1 the sensor, 2 the lens in front of it), when it is of `type`;
 * empty otherwise. */
std::string upstreamSubdev(const char *who, const char *videoNode,
                           int steps, uint32_t type)
{
    if (!videoNode) {
        ALOGW("%s: no video device path", who);
        return std::string();
    }
    int mediaFd = open("/dev/media0", O_RDWR);
    if (mediaFd < 0) {
        ALOGW("%s: open /dev/media0: %s", who, strerror(errno));
        return std::string();
    }

    uint32_t entityId = videoEntity(mediaFd, who, videoNode);
    for (int i = 0; i < steps && entityId; i++)
        entityId = mediaSinkSource(mediaFd, entityId);
    if (!entityId) {
        /* A fixed-focus module: the kernel links a lens into its sensor
         * only where the DT pairs one (associated-sensor). */
        ALOGI("%s: nothing %d link(s) upstream of %s", who, steps, videoNode);
        close(mediaFd);
        return std::string();
    }

    struct media_entity_desc ent;
    memset(&ent, 0, sizeof(ent));
    ent.id = entityId;
    if (ioctl(mediaFd, MEDIA_IOC_ENUM_ENTITIES, &ent) < 0) {
        ALOGW("%s: ENUM_ENTITIES(%u): %s", who, entityId, strerror(errno));
        close(mediaFd);
        return std::string();
    }
    close(mediaFd);
    if (ent.type != type) {
        ALOGW("%s: entity '%s' is of type 0x%x, not 0x%x",
              who, ent.name, ent.type, type);
        return std::string();
    }
    ALOGI("%s: entity '%s' dev=%u:%u", who, ent.name,
          ent.v4l.major, ent.v4l.minor);
    return subdevNode(who, ent.v4l.major, ent.v4l.minor);
}

} /* namespace */

std::string sensorSubdevFor(const char *videoNode)
{
    return upstreamSubdev("sensorSubdevFor", videoNode, 1,
                          MEDIA_ENT_T_V4L2_SUBDEV_SENSOR);
}

std::string lensSubdevFor(const char *videoNode)
{
    return upstreamSubdev("lensSubdevFor", videoNode, 2,
                          MEDIA_ENT_T_V4L2_SUBDEV_LENS);
}

} /* namespace MediaGraph */
} /* namespace android */
