#ifndef V4L2_RESOLUTION_H
#define V4L2_RESOLUTION_H

namespace android {

/* Width x height pair used across V4L2 + stream/metadata code. Lives
 * here rather than nested inside V4l2Device because non-V4L2 consumers
 * (static metadata, Camera::processCaptureRequest, StreamConfig) all
 * work with the same shape. */
struct Resolution {
    unsigned width;
    unsigned height;
};

}; /* namespace android */

#endif /* V4L2_RESOLUTION_H */
