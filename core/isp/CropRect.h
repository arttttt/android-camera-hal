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

/* What an output of width x height samples of the frame's crop: the
 * largest rectangle with the output's aspect ratio, centred in it.
 * Camera2 asks for this of every output -- each keeps its own aspect,
 * and outputs of different aspects see different parts of the crop,
 * none of them stretched. */
CropRect cropForOutput(const CropRect &frame, unsigned width, unsigned height);

}; /* namespace android */

#endif /* ISP_CROP_RECT_H */
