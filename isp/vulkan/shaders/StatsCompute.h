#ifndef ISP_VULKAN_SHADERS_STATS_COMPUTE_H
#define ISP_VULKAN_SHADERS_STATS_COMPUTE_H

namespace android {

/* Compute shader: scratch RGBA image → IpaStats layout.
 *
 * Dispatch shape: 16 x 16 x 1 workgroups, one workgroup per output
 * patch. local_size_x = 256 (flat) — each workgroup reduces a patch
 * of ~imgW/16 x imgH/16 pixels to three outputs:
 *   - 128-bin Rec. 601 luma histogram (global across all workgroups,
 *     privatized in shared memory and merged via atomicAdd on exit).
 *   - 16 x 16 per-patch RGB mean, tree-reduced across 256 threads.
 *   - 16 x 16 per-patch Tenengrad sharpness, Sum(Gx^2 + Gy^2) with
 *     3 x 3 Sobel on luma.
 *
 * Strategy: per-thread stride loop over the patch pixels. One pixel
 * per loop iteration reads its 3 x 3 luma neighbourhood via nine
 * direct texelFetch calls — adjacent threads in a warp map onto
 * adjacent patch pixels, so the neighbour fetches land on the same
 * scratch texels the warp-peers are sampling; Kepler's per-SMX L1
 * texture cache absorbs >99% of them. Histogram writes go to a
 * shared privatized bank; the global flush is one atomicAdd per bin
 * at workgroup exit.
 *
 * An earlier implementation cached luma in a halo-padded shared tile
 * per 16 x 16 sub-tile and looped sub-tiles within the workgroup.
 * It looked cheaper on paper (1 texelFetch per pixel instead of 9,
 * Sobel reads from shared) but synchronising 256 threads with two
 * barriers per sub-tile costs ~15 µs per barrier on GK20A — roughly
 * 20 K barriers per frame total, which measured as +300 ms post
 * time on 1080p. The texture-cache variant drops barriers from the
 * hot loop entirely (only init-hist + log2(256) tree-reduce remain).
 *
 * Shared memory budget per workgroup:
 *   sHist[128]      =  512 B
 *   sReduce[256]    = 4096 B   (vec4: .xyz = rgb sum, .w = sharp sum)
 *   -----------------------
 *   total           = 4608 B   — ~10 workgroups can reside on one
 *                                SMX at once; register pressure is
 *                                the tighter constraint.
 *
 * Uneven patch sizes: pc.imgW / 16 may not divide evenly. Patch
 * bounds are px0 = patchIdx * imgW / 16, px1 = (patchIdx+1) * imgW / 16
 * — rows and columns sum to imgW/imgH exactly, every pixel is covered.
 * patchW * patchH per patch is the divisor for the mean.
 *
 * The caller is responsible for:
 *   - Clearing the output buffer before dispatch (histogram uses
 *     atomicAdd on global memory, so it must start at zero).
 *     VulkanStatsEncoder::recordDispatch does this via CmdFillBuffer.
 *   - Scratch image SHADER_WRITE -> SHADER_READ barrier before the
 *     stats dispatch.
 *   - Stats buffer SHADER_WRITE -> HOST_READ barrier after (recorded
 *     inside VulkanStatsEncoder::recordDispatch). */
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
    "shared uint sHist   [128];\n"
    "shared vec4 sReduce [256];   /* .xyz = rgb sum, .w = sharp sum */\n"
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
    "\n"
    "    int px0 = int(patchX) * pc.imgW / 16;\n"
    "    int py0 = int(patchY) * pc.imgH / 16;\n"
    "    int px1 = (int(patchX) + 1) * pc.imgW / 16;\n"
    "    int py1 = (int(patchY) + 1) * pc.imgH / 16;\n"
    "    int patchW = px1 - px0;\n"
    "    int patchH = py1 - py0;\n"
    "    uint patchPx = uint(patchW * patchH);\n"
    "\n"
    "    if (tid < 128u) sHist[tid] = 0u;\n"
    "    barrier();\n"
    "\n"
    "    vec3  rgbSum   = vec3(0.0);\n"
    "    float sharpSum = 0.0;\n"
    "\n"
    "    for (uint p = tid; p < patchPx; p += 256u) {\n"
    "        uint px = p % uint(patchW);\n"
    "        uint py = p / uint(patchW);\n"
    "        int gx = px0 + int(px);\n"
    "        int gy = py0 + int(py);\n"
    "\n"
    "        vec3 rgb = texelFetch(scratchTex, ivec2(gx, gy), 0).rgb;\n"
    "        rgbSum += rgb;\n"
    "        float centerL = dot(rgb, LUMA_COEF);\n"
    "\n"
    "        uint bin = uint(clamp(centerL * 128.0, 0.0, 127.0));\n"
    "        atomicAdd(sHist[bin], 1u);\n"
    "\n"
    "        /* 3x3 Sobel from L1-cached neighbour reads. The warp-peer\n"
    "         * threads visit adjacent patch pixels, so the neighbour\n"
    "         * coords overlap heavily and the cache line stays hot. */\n"
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
    "    if (tid < 128u) {\n"
    "        atomicAdd(stats.lumaHist[tid], sHist[tid]);\n"
    "    }\n"
    "}\n";

} /* namespace android */

#endif /* ISP_VULKAN_SHADERS_STATS_COMPUTE_H */
