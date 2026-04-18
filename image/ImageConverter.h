#ifndef IMAGECONVERTER_H
#define IMAGECONVERTER_H

#include <stdint.h>
#include "Workers.h"

namespace android {

class ImageConverter
{
public:
    ImageConverter();
    ~ImageConverter();

    uint8_t * YUY2ToRGBA(const uint8_t *src, uint8_t *dst, unsigned width, unsigned height);
    uint8_t * YUY2ToJPEG(const uint8_t *src, uint8_t *dst, unsigned width, unsigned height, size_t dstLen, uint8_t quality);

    uint8_t * UYVYToRGBA(const uint8_t *src, uint8_t *dst, unsigned width, unsigned height);
    uint8_t * UYVYToJPEG(const uint8_t *src, uint8_t *dst, unsigned width, unsigned height, size_t dstLen, uint8_t quality);

    uint8_t * BayerToRGBA(const uint8_t *src, uint8_t *dst, unsigned width, unsigned height, uint32_t pixFmt, bool softIsp = true);

    /* Encode RGBA buffer to JPEG. Returns pointer past end of JPEG data, or dst on failure. */
    static uint8_t * RGBAToJPEG(const uint8_t *rgba, uint8_t *dst, unsigned width, unsigned height, size_t dstLen, uint8_t quality);

protected:
    uint8_t * splitRunWait(const uint8_t *src, uint8_t *dst, unsigned width, unsigned height, Workers::Task::Function fn);

private:
    struct ConvertTask {
        Workers::Task task;
        struct Data {
            const uint8_t  *src;
            uint8_t        *dst;
            size_t          width;
            size_t          height;
            size_t          linesNum;
            size_t          startLine;
            uint32_t        pixFmt;
            /* Gray-world AWB accumulation */
            uint64_t        sumR, sumG, sumB;
            /* WB gains (Q8: 256 = 1.0x) */
            unsigned        wbR, wbG, wbB;
            /* CCM (Q10: 1024 = 1.0x), row-major 3x3 */
            const int16_t  *ccm;
        } data;
    };
};

}; /* namespace android */

#endif // IMAGECONVERTER_H
