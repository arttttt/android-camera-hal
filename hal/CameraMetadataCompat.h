#ifndef BASE_CAMERA_METADATA_COMPAT_H
#define BASE_CAMERA_METADATA_COMPAT_H

/* Where this module gets android::CameraMetadata from.
 *
 * The obvious source is <camera/CameraMetadata.h> in libcamera_client, and
 * that is what this module used to include. libcamera_client is a framework
 * library: neither vendor_available nor part of the VNDK, so a HAL that links
 * it is a HAL that cannot be loaded from a vendor namespace. Ours is
 * installed in /vendor/lib/hw and dlopened by a vendor process, so the only
 * reason it has worked is that this board has no VNDK yet and a single
 * namespace still spans /system and /vendor.
 *
 * The class was never the problem. AOSP keeps a second copy of the very same
 * CameraMetadata.cpp for exactly this case, in
 *
 *     hardware/interfaces/camera/common/1.0/default/
 *
 * built as android.hardware.camera.common@1.0-helper: vendor_available, and
 * meant to be linked by camera HALs. Same source, same API. The only
 * difference is the namespace it is declared in.
 *
 * So the include moves and the name is reintroduced below under the one this
 * module has always used. Every call site keeps saying CameraMetadata and
 * keeps meaning what it meant; what changes is which library answers for it,
 * and whether this module can live in a vendor namespace at all.
 *
 * The file is not called CameraMetadata.h on purpose. base/ is on the include
 * path, so that name would shadow the header included below and a module
 * would quietly include itself.
 */

#include <CameraMetadata.h>

namespace android {

using CameraMetadata =
        ::android::hardware::camera::common::V1_0::helper::CameraMetadata;

} /* namespace android */

#endif /* BASE_CAMERA_METADATA_COMPAT_H */
