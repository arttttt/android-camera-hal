#ifndef ISP_VULKAN_SHADERS_STATS_COMPUTE_H
#define ISP_VULKAN_SHADERS_STATS_COMPUTE_H

namespace android {

/* Compute shader: scratch RGBA image → IpaStats layout.
 *
 * Dispatch shape: 16 x 16 x 1 workgroups, one workgroup per output
 * patch. local_size 16 x 16 (256 threads) linearly indexed.
 *
 * Per frame, reduces the scratch image to:
 *   - 128-bin Rec. 601 luma histogram (global, privatized in shared
 *     memory and merged via atomicAdd at workgroup exit).
 *   - 16 x 16 per-patch RGB mean (tree-reduced in shared memory,
 *     thread 0 writes the result; no atomicAdd on the output).
 *   - 16 x 16 per-patch Tenengrad sharpness — sum of Gx^2 + Gy^2 with
 *     a 3 x 3 Sobel on luma. Not normalized by pixel count — AF does
 *     peak detection, absolute magnitude is free telemetry.
 *
 * Optimization plan:
 *   - Each pixel is read from scratchTex ONCE by the load pass, its
 *     luma computed ONCE and kept in shared memory for Sobel. Centre
 *     pixels reload their RGB via a second texelFetch — that hits the
 *     L1 texture cache from the preceding load pass (warm line), so
 *     it is effectively free.
 *   - Sobel's 3 x 3 stencil reads nine neighbours from shared memory
 *     (fast) rather than re-issuing nine texelFetches per pixel.
 *   - Sub-tile loop inside the workgroup: each iteration loads an
 *     18 x 18 halo tile (16 centre + 1-pixel border) into shared
 *     luma, processes the 16 x 16 interior. The patch is tiled across
 *     the full patchW x patchH extent; partial final rows/cols are
 *     masked out via in-patch bounds check.
 *   - Histogram atomicAdd lives on shared memory (fast on Kepler),
 *     not global memory. Workgroup exit flushes 128 bins to global
 *     with one atomicAdd per bin — 256 workgroups x 128 = ~32 K
 *     global atomics total, versus ~2 M for a naive per-pixel scheme.
 *
 * Shared memory budget per workgroup:
 *   sLuma[18][18]   = 1296 B   (luma tile + halo)
 *   sHist[128]      =  512 B   (privatized histogram)
 *   sReduce[256][4] = 4096 B   (RGB + sharp tree reduction)
 *   --------------------------
 *   total           = 5904 B   (well under the 48 KB per-SMX budget;
 *                               allows 8 concurrent workgroups per
 *                               SMX on GK20A Kepler → full occupancy)
 *
 * Uneven patch sizes: imgW / 16 may not divide evenly (1920 / 16 = 120
 * exact, 1080 / 16 = 67.5). Patch bounds are px0 = patchIdx * imgW / 16
 * and px1 = (patchIdx + 1) * imgW / 16, so the rows/cols sum to exactly
 * imgW / imgH and every pixel is covered. patchW * patchH per patch is
 * used for mean normalization.
 *
 * The caller is responsible for:
 *   - Clearing the output buffer before dispatch (shader uses
 *     atomicAdd on the histogram band, so it must start at zero).
 *     VulkanStatsEncoder::recordDispatch does this via CmdFillBuffer.
 *   - Scratch image SHADER_WRITE -> SHADER_READ barrier before the
 *     stats dispatch.
 *   - Stats buffer SHADER_WRITE -> HOST_READ barrier after (recorded
 *     inside VulkanStatsEncoder::recordDispatch). */
static const char kStatsComputeGlsl[] =
    "#version 450\n"
    "layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;\n"
    "layout(set = 0, binding = 0) uniform sampler2D scratchTex;\n"
    "layout(std430, set = 0, binding = 1) buffer Stats {\n"
    "    uint  lumaHist [128];\n"
    "    float rgbMean  [768];   /* 16*16*3 flat */\n"
    "    float sharpness[256];   /* 16*16     */\n"
    "} stats;\n"
    "layout(push_constant) uniform PC { int imgW; int imgH; } pc;\n"
    "\n"
    "shared float sLuma[18][18];\n"
    "shared uint  sHist[128];\n"
    "shared vec4  sReduce[256];   /* .xyz = rgb sum, .w = sharp sum */\n"
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
    "\n"
    "    if (tid < 128u) sHist[tid] = 0u;\n"
    "    barrier();\n"
    "\n"
    "    vec3  rgbSum   = vec3(0.0);\n"
    "    float sharpSum = 0.0;\n"
    "\n"
    "    const vec3 LUMA_COEF = vec3(0.299, 0.587, 0.114);\n"
    "\n"
    "    for (int ty = 0; ty < patchH; ty += 16) {\n"
    "        for (int tx = 0; tx < patchW; tx += 16) {\n"
    "            /* Cooperative halo load: 18*18 = 324 pixels over\n"
    "             * 256 threads, 1-2 loads per thread. */\n"
    "            for (uint i = tid; i < 324u; i += 256u) {\n"
    "                uint tileX = i % 18u;\n"
    "                uint tileY = i / 18u;\n"
    "                int gx = clamp(px0 + tx - 1 + int(tileX), 0, pc.imgW - 1);\n"
    "                int gy = clamp(py0 + ty - 1 + int(tileY), 0, pc.imgH - 1);\n"
    "                vec3 rgb = texelFetch(scratchTex, ivec2(gx, gy), 0).rgb;\n"
    "                sLuma[tileY][tileX] = dot(rgb, LUMA_COEF);\n"
    "            }\n"
    "            barrier();\n"
    "\n"
    "            uint cx = gl_LocalInvocationID.x;\n"
    "            uint cy = gl_LocalInvocationID.y;\n"
    "            if (tx + int(cx) < patchW && ty + int(cy) < patchH) {\n"
    "                /* Warm L1 line: this pixel was just loaded above. */\n"
    "                int gx = px0 + tx + int(cx);\n"
    "                int gy = py0 + ty + int(cy);\n"
    "                vec3 rgb = texelFetch(scratchTex, ivec2(gx, gy), 0).rgb;\n"
    "                rgbSum += rgb;\n"
    "\n"
    "                float centerL = sLuma[cy + 1u][cx + 1u];\n"
    "                float tl = sLuma[cy     ][cx     ];\n"
    "                float t  = sLuma[cy     ][cx + 1u];\n"
    "                float tr = sLuma[cy     ][cx + 2u];\n"
    "                float l  = sLuma[cy + 1u][cx     ];\n"
    "                float r  = sLuma[cy + 1u][cx + 2u];\n"
    "                float bl = sLuma[cy + 2u][cx     ];\n"
    "                float b  = sLuma[cy + 2u][cx + 1u];\n"
    "                float br = sLuma[cy + 2u][cx + 2u];\n"
    "                float Gx = (tr + 2.0 * r + br) - (tl + 2.0 * l + bl);\n"
    "                float Gy = (bl + 2.0 * b + br) - (tl + 2.0 * t + tr);\n"
    "                sharpSum += Gx * Gx + Gy * Gy;\n"
    "\n"
    "                uint bin = uint(clamp(centerL * 128.0, 0.0, 127.0));\n"
    "                atomicAdd(sHist[bin], 1u);\n"
    "            }\n"
    "            barrier();\n"
    "        }\n"
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
