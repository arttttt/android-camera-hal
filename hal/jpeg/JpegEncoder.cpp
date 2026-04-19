#define LOG_TAG "Cam-JPEG"

#include "JpegEncoder.h"

#include <stdint.h>
#include <linux/videodev2.h>

#include <libyuv/rotate_argb.h>

#include <hardware/camera3.h>
#include <system/camera_metadata.h>
#include <utils/Log.h>

#include "ImageConverter.h"
#include "IspPipeline.h"

namespace android {

namespace {

constexpr uint8_t kDefaultJpegQuality = 95;

} /* namespace */

JpegEncoder::JpegEncoder(IspPipeline *isp, ImageConverter *converter)
    : mIsp(isp)
    , mConverter(converter) {
}

bool JpegEncoder::encode(uint8_t *dst, size_t bufferSize,
                         const JpegSource &src,
                         const CameraMetadata &cm,
                         uint8_t *rgbaScratch) const {
    const size_t maxImageSize = bufferSize - sizeof(camera3_jpeg_blob);

    uint8_t jpegQuality = kDefaultJpegQuality;
    if (cm.exists(ANDROID_JPEG_QUALITY))
        jpegQuality = *cm.find(ANDROID_JPEG_QUALITY).data.u8;
    ALOGD("JPEG quality = %u", jpegQuality);

    uint8_t *bufEnd = NULL;
    if (src.pixFmt == V4L2_PIX_FMT_UYVY) {
        bufEnd = mConverter->UYVYToJPEG(src.frameBuf, dst,
                                         src.width, src.height,
                                         maxImageSize, jpegQuality);
    } else if (src.pixFmt == V4L2_PIX_FMT_YUYV) {
        bufEnd = mConverter->YUY2ToJPEG(src.frameBuf, dst,
                                         src.width, src.height,
                                         maxImageSize, jpegQuality);
    } else {
        /* Bayer: synchronous demosaic + rotate + JPEG encode. */
        mIsp->processSync(src.frameBuf, rgbaScratch,
                          src.width, src.height, src.pixFmt,
                          src.srcInputSlot);

        int32_t jpegOri = 0;
        if (cm.exists(ANDROID_JPEG_ORIENTATION))
            jpegOri = *cm.find(ANDROID_JPEG_ORIENTATION).data.i32;

        unsigned outW = src.width;
        unsigned outH = src.height;
        libyuv::RotationMode rot = libyuv::kRotate0;
        if (jpegOri == 90) {
            rot  = libyuv::kRotate90;
            outW = src.height;
            outH = src.width;
        } else if (jpegOri == 180) {
            rot = libyuv::kRotate180;
        } else if (jpegOri == 270) {
            rot  = libyuv::kRotate270;
            outW = src.height;
            outH = src.width;
        }

        if (rot != libyuv::kRotate0) {
            uint8_t *rotBuf = new uint8_t[outW * outH * 4];
            libyuv::ARGBRotate(rgbaScratch, src.width * 4,
                               rotBuf, outW * 4,
                               src.width, src.height, rot);
            bufEnd = ImageConverter::RGBAToJPEG(rotBuf, dst,
                                                outW, outH,
                                                maxImageSize, jpegQuality);
            delete[] rotBuf;
        } else {
            bufEnd = ImageConverter::RGBAToJPEG(rgbaScratch, dst,
                                                outW, outH,
                                                maxImageSize, jpegQuality);
        }
    }

    if (bufEnd == dst) {
        ALOGE("JPEG image too big!");
        return false;
    }

    camera3_jpeg_blob *jpegBlob = reinterpret_cast<camera3_jpeg_blob *>(dst + maxImageSize);
    jpegBlob->jpeg_blob_id = CAMERA3_JPEG_BLOB_ID;
    jpegBlob->jpeg_size    = (uint32_t)(bufEnd - dst);
    return true;
}

}; /* namespace android */
