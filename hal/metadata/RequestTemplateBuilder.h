#ifndef HAL_METADATA_REQUEST_TEMPLATE_BUILDER_H
#define HAL_METADATA_REQUEST_TEMPLATE_BUILDER_H

#include <system/camera_metadata.h>

#include "mochacamera/Capability.h"

namespace android {

/* Builds the default capture request metadata for a given Camera3
 * template (CAMERA3_TEMPLATE_*). Stateless — called once per template
 * type and the result cached by the caller.
 *
 * The template is parameterised by the capability: the active array
 * (initial crop / AE-AWB-AF region defaults) and the AF modes (Auto
 * where the lens moves, Off for a fixed lens). */
class RequestTemplateBuilder {
public:
    static camera_metadata_t *build(int type, const mocha::camera::Capability &cap);
};

}; /* namespace android */

#endif /* HAL_METADATA_REQUEST_TEMPLATE_BUILDER_H */
