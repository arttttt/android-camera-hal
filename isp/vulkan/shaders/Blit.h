#ifndef ISP_VULKAN_SHADERS_BLIT_H
#define ISP_VULKAN_SHADERS_BLIT_H

namespace android {

/* Vertex + fragment used for the scratch-image → gralloc blit pipeline.
 * The vertex shader emits a full-screen triangle with no buffer input;
 * the fragment shader samples the scratch storage image at its own
 * gl_FragCoord, so the driver's ROP path is the one writing to the
 * gralloc colour attachment — that's the only blocklinear-aware write
 * surface Tegra Vulkan exposes to userspace. */

static const char kBlitVertexGlsl[] =
    "#version 450\n"
    "void main() {\n"
    "    float x = float((gl_VertexIndex << 1) & 2) * 2.0 - 1.0;\n"
    "    float y = float(gl_VertexIndex & 2) * 2.0 - 1.0;\n"
    "    gl_Position = vec4(x, y, 0.0, 1.0);\n"
    "}\n";

/* Push-constant block: crop rect (in scratch-image coords) + scratch
 * extent (srcW, srcH) + destination extent (outW, outH). The host
 * struct BlitPushConstants in VulkanIspPipeline.cpp must mirror this
 * layout exactly. */
static const char kBlitFragmentGlsl[] =
    "#version 450\n"
    "layout(set = 0, binding = 3) uniform sampler2D scratchTex;\n"
    "layout(push_constant) uniform BlitPC {\n"
    "    int cropX; int cropY; int cropW; int cropH;\n"
    "    int srcW;  int srcH;  int outW;  int outH;\n"
    "} pc;\n"
    "layout(location = 0) out vec4 outColor;\n"
    "void main() {\n"
    "    if (pc.cropW == pc.outW && pc.cropH == pc.outH) {\n"
    "        /* 1:1 crop (possibly offset) — no scaling. One texel per\n"
    "         * fragment, sampler filter skipped. */\n"
    "        ivec2 srcPix = ivec2(pc.cropX, pc.cropY) + ivec2(gl_FragCoord.xy);\n"
    "        outColor = texelFetch(scratchTex, srcPix, 0);\n"
    "    } else {\n"
    "        /* Hardware bilinear via the sampler — one texture() call\n"
    "         * covers the four-tap interpolation and the texture cache\n"
    "         * keeps the overlap between neighbour fragments hot. */\n"
    "        vec2 uv = (vec2(pc.cropX, pc.cropY)\n"
    "                 + gl_FragCoord.xy * vec2(pc.cropW, pc.cropH)\n"
    "                                   / vec2(pc.outW, pc.outH))\n"
    "                 / vec2(pc.srcW, pc.srcH);\n"
    "        outColor = texture(scratchTex, uv);\n"
    "    }\n"
    "}\n";

}; /* namespace android */

#endif /* ISP_VULKAN_SHADERS_BLIT_H */
