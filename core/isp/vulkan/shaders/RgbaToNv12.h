#ifndef ISP_VULKAN_SHADERS_RGBA_TO_NV12_H
#define ISP_VULKAN_SHADERS_RGBA_TO_NV12_H

namespace android {

/* Compute shader: RGBA8 scratch image → NV12 in a single storage buffer.
 *
 * One invocation handles a 4×2 Y block — 8 Y samples and 2 interleaved
 * UV pairs (chroma downsampled 2×2 by averaging). Writes 1 uint per Y
 * row (4 packed bytes) + 1 uint for the UV pair (U0 V0 U1 V1) — no
 * atomics, all writes aligned.
 *
 * Layout in outBuf (uint array, little-endian bytes):
 *   Y plane      at [0, w*h/4) uints
 *   UV plane     at [w*h/4, w*h*3/8) uints   (interleaved UVUV...)
 *
 * BT.601 limited-range coefficients, hardcoded. Every Android consumer
 * without explicit colour-space metadata interprets NV12 this way —
 * keeps us one less static-metadata key short of spec. If the consumer
 * set explicitly requests BT.709 full-range one day, the conversion
 * matrix becomes a push-constant array and this shader grows an if().
 *
 * Requires width % 4 == 0 and height % 2 == 0; the advertised output
 * resolutions in static metadata must satisfy both. */
static const char kRgbaToNv12ComputeGlsl[] =
    "#version 450\n"
    "layout(local_size_x = 8, local_size_y = 8) in;\n"
    "layout(set = 0, binding = 0) uniform sampler2D scratchTex;\n"
    "layout(std430, set = 0, binding = 1) buffer OutBuf { uint data[]; } outBuf;\n"
    "layout(push_constant) uniform PC {\n"
    "    int w;       /* Y plane width in pixels */\n"
    "    int h;       /* Y plane height in pixels */\n"
    "    int uvBase;  /* start of UV plane in uint units = w*h/4 */\n"
    "    int stride;  /* Y-row stride in uint units = w/4          */\n"
    "} pc;\n"
    "float rgb2y (vec3 rgb) { return dot(rgb, vec3( 0.257,  0.504,  0.098)) * 255.0 + 16.0; }\n"
    "float rgb2cb(vec3 rgb) { return dot(rgb, vec3(-0.148, -0.291,  0.439)) * 255.0 + 128.0; }\n"
    "float rgb2cr(vec3 rgb) { return dot(rgb, vec3( 0.439, -0.368, -0.071)) * 255.0 + 128.0; }\n"
    "uint toByte(float v) { return uint(clamp(v + 0.5, 0.0, 255.0)); }\n"
    "void main() {\n"
    "    int x = int(gl_GlobalInvocationID.x);  /* in [0, w/4) */\n"
    "    int y = int(gl_GlobalInvocationID.y);  /* in [0, h/2) */\n"
    "    if (4*x >= pc.w || 2*y >= pc.h) return;\n"
    "\n"
    "    vec3 p00 = texelFetch(scratchTex, ivec2(4*x+0, 2*y+0), 0).rgb;\n"
    "    vec3 p01 = texelFetch(scratchTex, ivec2(4*x+1, 2*y+0), 0).rgb;\n"
    "    vec3 p02 = texelFetch(scratchTex, ivec2(4*x+2, 2*y+0), 0).rgb;\n"
    "    vec3 p03 = texelFetch(scratchTex, ivec2(4*x+3, 2*y+0), 0).rgb;\n"
    "    vec3 p10 = texelFetch(scratchTex, ivec2(4*x+0, 2*y+1), 0).rgb;\n"
    "    vec3 p11 = texelFetch(scratchTex, ivec2(4*x+1, 2*y+1), 0).rgb;\n"
    "    vec3 p12 = texelFetch(scratchTex, ivec2(4*x+2, 2*y+1), 0).rgb;\n"
    "    vec3 p13 = texelFetch(scratchTex, ivec2(4*x+3, 2*y+1), 0).rgb;\n"
    "\n"
    "    uint y0 = toByte(rgb2y(p00))        | (toByte(rgb2y(p01)) << 8)\n"
    "            | (toByte(rgb2y(p02)) << 16)| (toByte(rgb2y(p03)) << 24);\n"
    "    uint y1 = toByte(rgb2y(p10))        | (toByte(rgb2y(p11)) << 8)\n"
    "            | (toByte(rgb2y(p12)) << 16)| (toByte(rgb2y(p13)) << 24);\n"
    "\n"
    "    outBuf.data[(2*y + 0) * pc.stride + x] = y0;\n"
    "    outBuf.data[(2*y + 1) * pc.stride + x] = y1;\n"
    "\n"
    "    vec3 avgL = (p00 + p01 + p10 + p11) * 0.25;\n"
    "    vec3 avgR = (p02 + p03 + p12 + p13) * 0.25;\n"
    "    uint uv = toByte(rgb2cb(avgL))        | (toByte(rgb2cr(avgL)) << 8)\n"
    "            | (toByte(rgb2cb(avgR)) << 16)| (toByte(rgb2cr(avgR)) << 24);\n"
    "    outBuf.data[pc.uvBase + y * pc.stride + x] = uv;\n"
    "}\n";

}; /* namespace android */

#endif /* ISP_VULKAN_SHADERS_RGBA_TO_NV12_H */
