#ifndef ISP_VULKAN_SHADERS_STATS_COMPUTE_H
#define ISP_VULKAN_SHADERS_STATS_COMPUTE_H

namespace android {

/* Compute shader: scratch RGBA image → IpaStats layout.
 *
 * Dispatch shape: 16 × 16 workgroups (one per output patch),
 * local_size_x = 256 flat. One workgroup reduces its patch to
 * partial histogram, RGB mean and Tenengrad sharpness.
 *
 * Engineering decisions calibrated for Kepler SM 3.2 (GK20A) — one
 * SMX, 192 CUDA cores, 48 KB shared, soft-atomic shared memory,
 * 32-thread warps executing in lockstep.
 *
 *   A. Per-warp privatized histogram — shared layout is
 *      sHist[8][128]; warp W of the workgroup only ever touches
 *      sHist[W][*]. Cross-warp contention is zero by construction,
 *      intra-warp contention capped at 32-way even on a flat luma
 *      scene. Final pass flushes the eight bands into the global
 *      histogram with 128 parallel atomics.
 *
 *   B. No shared luma tile — Sobel reads the eight neighbours of
 *      each centre pixel via direct texelFetch. Warp-peer threads
 *      stride along adjacent patch columns, so consecutive
 *      invocations share seven of the nine texels, and Kepler's
 *      per-SMX L1 texture cache delivers those repeat reads at
 *      near-zero cost. Dropping the tile also removes the bank-
 *      conflict risk a 122-wide row layout would have produced.
 *
 *   C. Shared memory budget kept small —
 *        sHist   [8][128] =  4096 B
 *        sReduce [256]    =  4096 B   (vec4: .xyz rgb, .w sharp)
 *        total            =  8192 B   → six workgroups can coexist
 *                                      on one SMX (full ~75 %
 *                                      occupancy of 2048 thread
 *                                      slots), giving the warp
 *                                      scheduler room to hide any
 *                                      texture latency.
 *
 *   D. Barrier count: 2 at init + log2(256) = 8 in the tree reduce
 *      = 10 barriers per workgroup. Compare with an earlier sub-
 *      tile revision that executed roughly 80 barriers per
 *      workgroup × 256 workgroups ≈ 20 K barriers per frame.
 *
 * Uneven patch sizes: px0 = patchIdx * imgW / 16 and
 * px1 = (patchIdx + 1) * imgW / 16 — row and column widths sum to
 * exactly imgW/imgH and every pixel is covered. patchW*patchH is
 * the divisor when writing the mean.
 *
 * Caller contract (see VulkanStatsEncoder::recordDispatch):
 *   - Output buffer zero-filled before dispatch (histogram uses
 *     atomicAdd on global memory).
 *   - Scratch image SHADER_WRITE → SHADER_READ barrier upstream.
 *   - Stats buffer SHADER_WRITE → HOST_READ barrier downstream. */
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
    "const uint WARPS_PER_WG = 8u;\n"
    "\n"
    "shared uint sHist  [WARPS_PER_WG][128];\n"
    "shared vec4 sReduce[256];\n"
    "\n"
    "const vec3 LUMA_COEF = vec3(0.299, 0.587, 0.114);\n"
    "\n"
    "float lumaAt(int x, int y) {\n"
    "    int cx = clamp(x, 0, pc.imgW - 1);\n"
    "    int cy = clamp(y, 0, pc.imgH - 1);\n"
    "    return dot(texelFetch(scratchTex, ivec2(cx, cy), 0).rgb, LUMA_COEF);\n"
    "}\n"
    "\n"
    "void main() {\n"
    "    uint patchX   = gl_WorkGroupID.x;\n"
    "    uint patchY   = gl_WorkGroupID.y;\n"
    "    uint patchIdx = patchY * 16u + patchX;\n"
    "    uint tid      = gl_LocalInvocationIndex;\n"
    "    uint warpId   = tid / 32u;\n"
    "\n"
    "    int px0 = int(patchX) * pc.imgW / 16;\n"
    "    int py0 = int(patchY) * pc.imgH / 16;\n"
    "    int px1 = (int(patchX) + 1) * pc.imgW / 16;\n"
    "    int py1 = (int(patchY) + 1) * pc.imgH / 16;\n"
    "    int patchW = px1 - px0;\n"
    "    int patchH = py1 - py0;\n"
    "    uint patchPx = uint(patchW * patchH);\n"
    "\n"
    "    /* Zero all eight histogram bands cooperatively: 8*128 = 1024\n"
    "     * uints, 4 writes per thread. */\n"
    "    for (uint i = tid; i < WARPS_PER_WG * 128u; i += 256u) {\n"
    "        sHist[i / 128u][i % 128u] = 0u;\n"
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
    "        vec3 rgb = texelFetch(scratchTex, ivec2(gx, gy), 0).rgb;\n"
    "        rgbSum += rgb;\n"
    "\n"
    "        float centerL = dot(rgb, LUMA_COEF);\n"
    "        uint bin = uint(clamp(centerL * 128.0, 0.0, 127.0));\n"
    "        atomicAdd(sHist[warpId][bin], 1u);\n"
    "\n"
    "        /* Sobel neighbourhood — warp-peer threads process adjacent\n"
    "         * patch columns, so the top/bottom row taps overlap across\n"
    "         * the warp and land in Kepler's L1 texture cache. */\n"
    "        float tl = lumaAt(gx - 1, gy - 1);\n"
    "        float t  = lumaAt(gx    , gy - 1);\n"
    "        float tr = lumaAt(gx + 1, gy - 1);\n"
    "        float l  = lumaAt(gx - 1, gy    );\n"
    "        float r  = lumaAt(gx + 1, gy    );\n"
    "        float bl = lumaAt(gx - 1, gy + 1);\n"
    "        float b  = lumaAt(gx    , gy + 1);\n"
    "        float br = lumaAt(gx + 1, gy + 1);\n"
    "        float Gx = (tr + 2.0 * r + br) - (tl + 2.0 * l + bl);\n"
    "        float Gy = (bl + 2.0 * b + br) - (tl + 2.0 * t + tr);\n"
    "        sharpSum += Gx * Gx + Gy * Gy;\n"
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
    "    /* Merge the eight per-warp bands into the global histogram.\n"
    "     * 128 threads each sum their bin across warps and emit one\n"
    "     * global atomicAdd. */\n"
    "    if (tid < 128u) {\n"
    "        uint sum = 0u;\n"
    "        for (uint w = 0u; w < WARPS_PER_WG; w++) {\n"
    "            sum += sHist[w][tid];\n"
    "        }\n"
    "        atomicAdd(stats.lumaHist[tid], sum);\n"
    "    }\n"
    "}\n";

} /* namespace android */

#endif /* ISP_VULKAN_SHADERS_STATS_COMPUTE_H */
