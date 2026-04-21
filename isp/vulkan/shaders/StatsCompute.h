#ifndef ISP_VULKAN_SHADERS_STATS_COMPUTE_H
#define ISP_VULKAN_SHADERS_STATS_COMPUTE_H

namespace android {

/* Compute shader: scratch RGBA image → IpaStats layout.
 *
 * Dispatch shape: 16 x 16 workgroups (one per output patch),
 * local_size_x = 256 flat. Each workgroup owns one patch of the
 * 16 x 16 output grid.
 *
 * Cache plan — "compute luma once per pixel, re-use for everything
 * that needs it":
 *
 *   1. Cooperative load pass — 256 threads walk the patch-plus-halo
 *      extent (patchW+2) x (patchH+2), and for each covered pixel
 *      they issue ONE texelFetch and ONE dot(rgb, LUMA_COEF). The
 *      resulting luma is stored in a shared tile sLuma[H+2][W+2].
 *   2. ONE barrier.
 *   3. Compute pass — 256 threads stride over the patch's interior
 *      pixels. Histogram bin comes from sLuma[centre], the 3 x 3
 *      Sobel reads nine neighbours from sLuma (no recomputation).
 *      The RGB sum for the patch mean needs the colour values, so
 *      the centre pixel's RGB is re-fetched from the scratch image
 *      once — Kepler's L1 texture cache keeps those lines hot from
 *      the load pass so the hit rate is effectively 100%.
 *   4. Tree-reduce the per-thread RGB and sharpness sums across the
 *      workgroup, thread 0 writes the patch outputs. A final set of
 *      128 threads flushes the privatized histogram to global.
 *
 * Shared memory budget per workgroup (1080p maximum):
 *   sLuma[69][122]  = 33672 B   (patch 120x67 + 1-pixel halo)
 *   sHist[128]      =   512 B
 *   sReduce[256]    =  4096 B   (vec4: .xyz rgb sum, .w sharp sum)
 *   ----------------
 *   total           = 38280 B   under the 48 KB per-SMX budget.
 *
 * Occupancy trade-off: 38 KB shared means one workgroup per SMX on
 * GK20A, about 256 of 2048 thread slots active (~12 %). Low, but
 * the shader is compute-bound on shared-memory reads and ALU — it
 * barely misses from texture after the load pass — so there is
 * little memory latency to hide. Barrier count drops from ~20 K
 * per frame (the sub-tile-looped predecessor) to 10 per workgroup.
 *
 * Resolution limits: sLuma is sized for 1080p. Higher-resolution
 * previews (e.g. 3264 x 2448 snapshot path) do not drive stats —
 * stats only fires through recordGrallocBlit and the caller keeps
 * preview ≤ 1920 x 1080. Smaller resolutions (720p, 480p) fit with
 * plenty of slack.
 *
 * Caller contract (see VulkanStatsEncoder::recordDispatch):
 *   - Output buffer must be zero before dispatch (histogram uses
 *     atomicAdd).
 *   - Scratch image SHADER_WRITE -> SHADER_READ barrier upstream.
 *   - Stats buffer SHADER_WRITE -> HOST_READ barrier downstream. */
static const char kStatsComputeGlsl[] =
    "#version 450\n"
    "layout(local_size_x = 256, local_size_y = 1, local_size_z = 1) in;\n"
    "layout(set = 0, binding = 0) uniform sampler2D scratchTex;\n"
    "layout(std430, set = 0, binding = 1) buffer Stats {\n"
    "    uint  lumaHist [128];\n"
    "    float rgbMean  [768];   /* 16*16*3 flat */\n"
    "    float sharpness[256];   /* 16*16     */\n"
    "} stats;\n"
    "layout(push_constant) uniform PC { int imgW; int imgH; } pc;\n"
    "\n"
    "const int MAX_PATCH_X = 122;   /* ceil(1920/16) + 2 halo */\n"
    "const int MAX_PATCH_Y = 69;    /* ceil(1080/16) + 2 halo */\n"
    "\n"
    "shared float sLuma   [MAX_PATCH_Y][MAX_PATCH_X];\n"
    "shared uint  sHist   [128];\n"
    "shared vec4  sReduce [256];\n"
    "\n"
    "const vec3 LUMA_COEF = vec3(0.299, 0.587, 0.114);\n"
    "\n"
    "void main() {\n"
    "    uint patchX   = gl_WorkGroupID.x;\n"
    "    uint patchY   = gl_WorkGroupID.y;\n"
    "    uint patchIdx = patchY * 16u + patchX;\n"
    "    uint tid      = gl_LocalInvocationIndex;\n"
    "\n"
    "    int px0 = int(patchX) * pc.imgW / 16;\n"
    "    int py0 = int(patchY) * pc.imgH / 16;\n"
    "    int px1 = (int(patchX) + 1) * pc.imgW / 16;\n"
    "    int py1 = (int(patchY) + 1) * pc.imgH / 16;\n"
    "    int patchW = px1 - px0;\n"
    "    int patchH = py1 - py0;\n"
    "    int tileW  = patchW + 2;\n"
    "    int tileH  = patchH + 2;\n"
    "    uint tilePx  = uint(tileW * tileH);\n"
    "    uint patchPx = uint(patchW * patchH);\n"
    "\n"
    "    if (tid < 128u) sHist[tid] = 0u;\n"
    "\n"
    "    /* Load pass: every tile cell (patch interior + 1-pixel halo)\n"
    "     * is read once, its luma is computed once, and the result is\n"
    "     * parked in shared memory. 256 threads strided across the\n"
    "     * flat tile extent — warp-peers hit adjacent scratch texels\n"
    "     * so the driver's coalescer gets along well with L1. */\n"
    "    for (uint i = tid; i < tilePx; i += 256u) {\n"
    "        int tx = int(i) % tileW;\n"
    "        int ty = int(i) / tileW;\n"
    "        int gx = clamp(px0 + tx - 1, 0, pc.imgW - 1);\n"
    "        int gy = clamp(py0 + ty - 1, 0, pc.imgH - 1);\n"
    "        vec3 rgb = texelFetch(scratchTex, ivec2(gx, gy), 0).rgb;\n"
    "        sLuma[ty][tx] = dot(rgb, LUMA_COEF);\n"
    "    }\n"
    "    barrier();\n"
    "\n"
    "    vec3  rgbSum   = vec3(0.0);\n"
    "    float sharpSum = 0.0;\n"
    "\n"
    "    for (uint p = tid; p < patchPx; p += 256u) {\n"
    "        int px = int(p) % patchW;\n"
    "        int py = int(p) / patchW;\n"
    "        int gx = px0 + px;\n"
    "        int gy = py0 + py;\n"
    "\n"
    "        /* RGB for patch mean — one re-fetch of the centre pixel.\n"
    "         * The load pass just touched this exact texel, so L1 is\n"
    "         * warm. No RGB goes into shared memory (vec3 per cell\n"
    "         * would push shared past the budget). */\n"
    "        vec3 rgb = texelFetch(scratchTex, ivec2(gx, gy), 0).rgb;\n"
    "        rgbSum += rgb;\n"
    "\n"
    "        /* Sobel neighbours + centre bin — pure shared reads. */\n"
    "        int lx = px + 1;\n"
    "        int ly = py + 1;\n"
    "        float centerL = sLuma[ly    ][lx    ];\n"
    "        float tl      = sLuma[ly - 1][lx - 1];\n"
    "        float t       = sLuma[ly - 1][lx    ];\n"
    "        float tr      = sLuma[ly - 1][lx + 1];\n"
    "        float ll      = sLuma[ly    ][lx - 1];\n"
    "        float rr      = sLuma[ly    ][lx + 1];\n"
    "        float bl      = sLuma[ly + 1][lx - 1];\n"
    "        float bb      = sLuma[ly + 1][lx    ];\n"
    "        float br      = sLuma[ly + 1][lx + 1];\n"
    "        float Gx = (tr + 2.0 * rr + br) - (tl + 2.0 * ll + bl);\n"
    "        float Gy = (bl + 2.0 * bb + br) - (tl + 2.0 * t  + tr);\n"
    "        sharpSum += Gx * Gx + Gy * Gy;\n"
    "\n"
    "        uint bin = uint(clamp(centerL * 128.0, 0.0, 127.0));\n"
    "        atomicAdd(sHist[bin], 1u);\n"
    "    }\n"
    "\n"
    "    sReduce[tid] = vec4(rgbSum, sharpSum);\n"
    "    barrier();\n"
    "    for (uint s = 128u; s > 0u; s >>= 1u) {\n"
    "        if (tid < s) sReduce[tid] += sReduce[tid + s];\n"
    "        barrier();\n"
    "    }\n"
    "\n"
    "    if (tid == 0u) {\n"
    "        float inv = 1.0 / float(patchW * patchH);\n"
    "        stats.rgbMean[patchIdx * 3u + 0u] = sReduce[0].x * inv;\n"
    "        stats.rgbMean[patchIdx * 3u + 1u] = sReduce[0].y * inv;\n"
    "        stats.rgbMean[patchIdx * 3u + 2u] = sReduce[0].z * inv;\n"
    "        stats.sharpness[patchIdx]         = sReduce[0].w;\n"
    "    }\n"
    "\n"
    "    if (tid < 128u) {\n"
    "        atomicAdd(stats.lumaHist[tid], sHist[tid]);\n"
    "    }\n"
    "}\n";

} /* namespace android */

#endif /* ISP_VULKAN_SHADERS_STATS_COMPUTE_H */
