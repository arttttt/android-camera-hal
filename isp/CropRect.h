#ifndef ISP_CROP_RECT_H
#define ISP_CROP_RECT_H

namespace android {

/* Source-space rectangle for ISP-to-gralloc blits. Interpreted as
 * "sample this sub-region of the ISP scratch image, scale to fit the
 * destination". An identity blit passes the full source extent:
 * {0, 0, srcW, srcH}. */
struct CropRect {
    int x;
    int y;
    int w;
    int h;
};

}; /* namespace android */

#endif /* ISP_CROP_RECT_H */
