#ifndef HAL_METADATA_REQUEST_TEMPLATE_BUILDER_H
#define HAL_METADATA_REQUEST_TEMPLATE_BUILDER_H

#include <system/camera_metadata.h>

namespace android {

class V4l2Device;

/* Builds the default capture request metadata for a given Camera3
 * template (CAMERA3_TEMPLATE_*). Stateless — called once per template
 * type and the result cached by the caller.
 *
 * The template is parameterised by the sensor's active array size
 * (for initial crop / AE-AWB-AF region defaults) and the camera
 * facing (the default AF mode differs between the fixed-focus front
 * lens and the VCM-driven back lens). */
class RequestTemplateBuilder {
public:
    static camera_metadata_t *build(int type, V4l2Device *dev, int facing);
};

}; /* namespace android */

#endif /* HAL_METADATA_REQUEST_TEMPLATE_BUILDER_H */
