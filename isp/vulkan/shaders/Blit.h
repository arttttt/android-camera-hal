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

static const char kBlitFragmentGlsl[] =
    "#version 450\n"
    "layout(set = 0, binding = 3) uniform sampler2D scratchTex;\n"
    "layout(location = 0) out vec4 outColor;\n"
    "void main() {\n"
    "    /* texelFetch matches the previous imageLoad pixel-for-pixel.\n"
    "     * The sampler is present for the forthcoming texture()-based\n"
    "     * crop+scale path; texelFetch ignores its filter/addressing. */\n"
    "    outColor = texelFetch(scratchTex, ivec2(gl_FragCoord.xy), 0);\n"
    "}\n";

}; /* namespace android */

#endif /* ISP_VULKAN_SHADERS_BLIT_H */
