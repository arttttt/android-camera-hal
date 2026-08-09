/* Probe: can NV Vulkan driver on Tegra K1 create a STORAGE_BIT VkImage
 * over a gralloc-allocated RGBA8888 buffer (imported via the
 * VK_ANDROID_native_buffer extension)?
 *
 * The answer drives the scratch-buffer design for the camera HAL:
 *
 *   - PASS: scratch can become gralloc-backed, demosaic compute shader
 *     writes to it as a storage image (current pattern), and NvBlit
 *     reads the same handle directly. Plan 2a is straightforward.
 *
 *   - FAIL (CreateImage returns ERROR_FORMAT_NOT_SUPPORTED or similar):
 *     gralloc-imported VkImages are limited to color-attachment/sampled
 *     usage on this driver. Then either (a) convert demosaic from
 *     compute to fragment shader, or (b) add an extra render-pass /
 *     vkCmdCopyImage from a pure-Vulkan scratch into a gralloc-backed
 *     scratch (effectively a bridge).
 *
 * Run: adb push out/.../vk_storage_probe /data/local/tmp/ && adb shell ...
 */

#define LOG_TAG "vk_storage_probe"

#include <dlfcn.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <vector>

#include <hardware/gralloc.h>
#include <ui/GraphicBuffer.h>
#include <utils/Log.h>

#include <vulkan/vulkan.h>

using namespace android;

/* VK_ANDROID_native_buffer — not in the android-24 vulkan.h. */
#define VK_STRUCTURE_TYPE_NATIVE_BUFFER_ANDROID  ((VkStructureType)1000010000)
typedef struct VkNativeBufferANDROID {
    VkStructureType    sType;
    const void        *pNext;
    const void        *handle;
    int                stride;
    int                format;
    int                usage;
} VkNativeBufferANDROID;

/* HMI mirror — minimal struct subset to call the open() method on
 * vulkan.tegra.so without pulling in <hardware/hwvulkan.h>. */
struct hw_module_methods_inline {
    int (*open)(const void *module, const char *id, void **device);
};
struct hw_module_inline {
    uint32_t                   tag;
    uint16_t                   module_api_version;
    uint16_t                   hal_api_version;
    const char                *id;
    const char                *name;
    const char                *author;
    hw_module_methods_inline  *methods;
    void                      *dso;
    uint32_t                   reserved[25];
};
struct hw_device_inline {
    uint32_t  tag;
    uint32_t  version;
    void     *module;
    uint32_t  reserved[12];
    int     (*close)(void *device);
};
struct hwvulkan_device_inline {
    hw_device_inline                              common;
    PFN_vkEnumerateInstanceExtensionProperties    EnumerateInstanceExtensionProperties;
    PFN_vkCreateInstance                          CreateInstance;
    PFN_vkGetInstanceProcAddr                     GetInstanceProcAddr;
};

#define P(...)    do { fprintf(stderr, __VA_ARGS__); ALOGI(__VA_ARGS__); } while (0)
#define FAIL(...) do { fprintf(stderr, "FAIL: " __VA_ARGS__); fprintf(stderr, "\n"); ALOGE("FAIL: " __VA_ARGS__); return 1; } while (0)

int main() {
    /* --- load NV vulkan via HMI, mirror of HalHmiVulkanLoader. --- */
    void *dso = dlopen("/system/vendor/lib/hw/vulkan.tegra.so",
                       RTLD_NOW | RTLD_LOCAL);
    if (!dso) FAIL("dlopen vulkan.tegra.so: %s", dlerror());

    hw_module_inline *mod = (hw_module_inline *)dlsym(dso, "HMI");
    if (!mod || !mod->methods || !mod->methods->open)
        FAIL("HMI symbol or open method missing");

    void *raw = NULL;
    if (mod->methods->open(mod, "vk0", &raw) != 0 || !raw)
        FAIL("HMI open(vk0) failed");

    hwvulkan_device_inline *hd = (hwvulkan_device_inline *)raw;
    PFN_vkGetInstanceProcAddr gipa = hd->GetInstanceProcAddr;
    PFN_vkCreateInstance      ci   = hd->CreateInstance;
    if (!gipa || !ci) FAIL("hwvulkan_device missing entry points");
    P("HMI loaded vulkan.tegra.so\n");

    /* --- minimal instance: no extensions, no layers. --- */
    VkApplicationInfo ai = {};
    ai.sType            = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    ai.apiVersion       = VK_MAKE_VERSION(1, 0, 0);
    ai.pApplicationName = "vk_storage_probe";
    ai.pEngineName      = "vk_storage_probe";

    VkInstanceCreateInfo ici_inst = {};
    ici_inst.sType            = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    ici_inst.pApplicationInfo = &ai;

    VkInstance inst = VK_NULL_HANDLE;
    VkResult r = ci(&ici_inst, NULL, &inst);
    if (r != VK_SUCCESS) FAIL("vkCreateInstance: %d", (int)r);
    P("vkCreateInstance ok inst=%p\n", inst);

    auto INST = [&](const char *n) { return gipa(inst, n); };
    auto EnumPhys   = (PFN_vkEnumeratePhysicalDevices)        INST("vkEnumeratePhysicalDevices");
    auto GetQF      = (PFN_vkGetPhysicalDeviceQueueFamilyProperties) INST("vkGetPhysicalDeviceQueueFamilyProperties");
    auto EnumDevExt = (PFN_vkEnumerateDeviceExtensionProperties) INST("vkEnumerateDeviceExtensionProperties");
    auto CreateDev  = (PFN_vkCreateDevice)                    INST("vkCreateDevice");
    auto DestroyDev = (PFN_vkDestroyDevice)                   INST("vkDestroyDevice");
    auto GetDevPA   = (PFN_vkGetDeviceProcAddr)               INST("vkGetDeviceProcAddr");
    auto DestroyInst= (PFN_vkDestroyInstance)                 INST("vkDestroyInstance");
    if (!EnumPhys || !GetQF || !EnumDevExt || !CreateDev || !DestroyDev || !GetDevPA || !DestroyInst)
        FAIL("instance PFNs missing");

    uint32_t nphys = 0;
    EnumPhys(inst, &nphys, NULL);
    if (nphys == 0) FAIL("no physical devices");
    std::vector<VkPhysicalDevice> phys(nphys);
    EnumPhys(inst, &nphys, phys.data());
    VkPhysicalDevice pd = phys[0];
    P("physical devices=%u, using [0]=%p\n", nphys, pd);

    uint32_t nqf = 0;
    GetQF(pd, &nqf, NULL);
    std::vector<VkQueueFamilyProperties> qf(nqf);
    GetQF(pd, &nqf, qf.data());
    uint32_t qfi = UINT32_MAX;
    for (uint32_t i = 0; i < nqf; i++) {
        if (qf[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) { qfi = i; break; }
    }
    if (qfi == UINT32_MAX) FAIL("no graphics queue family");
    P("queue family idx=%u (graphics)\n", qfi);

    uint32_t next = 0;
    EnumDevExt(pd, NULL, &next, NULL);
    std::vector<VkExtensionProperties> exts(next);
    EnumDevExt(pd, NULL, &next, exts.data());
    bool haveAnb = false;
    for (auto &e : exts) {
        if (strcmp(e.extensionName, "VK_ANDROID_native_buffer") == 0) {
            haveAnb = true; break;
        }
    }
    if (!haveAnb) FAIL("device doesn't expose VK_ANDROID_native_buffer");
    P("VK_ANDROID_native_buffer extension available\n");

    /* --- create device with VK_ANDROID_native_buffer enabled. --- */
    float q_prio = 1.0f;
    VkDeviceQueueCreateInfo qci = {};
    qci.sType            = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    qci.queueFamilyIndex = qfi;
    qci.queueCount       = 1;
    qci.pQueuePriorities = &q_prio;

    const char *anbName = "VK_ANDROID_native_buffer";
    VkDeviceCreateInfo dci = {};
    dci.sType                   = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    dci.queueCreateInfoCount    = 1;
    dci.pQueueCreateInfos       = &qci;
    dci.enabledExtensionCount   = 1;
    dci.ppEnabledExtensionNames = &anbName;

    VkDevice dev = VK_NULL_HANDLE;
    r = CreateDev(pd, &dci, NULL, &dev);
    if (r != VK_SUCCESS) FAIL("vkCreateDevice: %d", (int)r);
    P("vkCreateDevice ok dev=%p\n", dev);

    auto CreateImage  = (PFN_vkCreateImage)  GetDevPA(dev, "vkCreateImage");
    auto DestroyImage = (PFN_vkDestroyImage) GetDevPA(dev, "vkDestroyImage");
    if (!CreateImage || !DestroyImage) {
        DestroyDev(dev, NULL); DestroyInst(inst, NULL);
        FAIL("device PFNs missing");
    }

    /* --- allocate gralloc RGBA. usage = HW_RENDER | HW_2D so gralloc
     *     picks a layout that's both Vulkan-renderable and HW VIC
     *     readable. --- */
    const uint32_t W = 64, H = 64;
    sp<GraphicBuffer> gb(new GraphicBuffer(W, H,
        HAL_PIXEL_FORMAT_RGBA_8888,
        GRALLOC_USAGE_HW_RENDER | GRALLOC_USAGE_HW_2D));
    if (gb->initCheck() != NO_ERROR) {
        DestroyDev(dev, NULL); DestroyInst(inst, NULL);
        FAIL("gralloc alloc");
    }
    ANativeWindowBuffer *anwb = gb->getNativeBuffer();
    /* usage widened from int to uint64_t at some point along the branches
     * this builds against, so print it through a fixed width rather than
     * matching whichever type is in scope. */
    P("gralloc alloc ok handle=%p stride=%u usage=0x%llx format=%d\n",
      anwb->handle, anwb->stride,
      (unsigned long long) anwb->usage, anwb->format);

    /* --- the test: VkCreateImage with STORAGE | SAMPLED usage over
     *     gralloc handle. --- */
    VkNativeBufferANDROID nb = {};
    nb.sType  = VK_STRUCTURE_TYPE_NATIVE_BUFFER_ANDROID;
    nb.handle = anwb->handle;
    nb.stride = anwb->stride;
    nb.format = anwb->format;
    nb.usage  = anwb->usage;

    VkImageCreateInfo ici = {};
    ici.sType         = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    ici.pNext         = &nb;
    ici.imageType     = VK_IMAGE_TYPE_2D;
    ici.format        = VK_FORMAT_R8G8B8A8_UNORM;
    ici.extent.width  = W;
    ici.extent.height = H;
    ici.extent.depth  = 1;
    ici.mipLevels     = 1;
    ici.arrayLayers   = 1;
    ici.samples       = VK_SAMPLE_COUNT_1_BIT;
    ici.tiling        = VK_IMAGE_TILING_OPTIMAL;
    ici.usage         = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    ici.sharingMode   = VK_SHARING_MODE_EXCLUSIVE;
    ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    VkImage img = VK_NULL_HANDLE;
    r = CreateImage(dev, &ici, NULL, &img);
    P("vkCreateImage(STORAGE|SAMPLED, gralloc-imported) = %d\n", (int)r);

    int rc = 0;
    if (r == VK_SUCCESS) {
        P("PASS: scratch can be gralloc-backed with storage write\n");
        DestroyImage(dev, img, NULL);
    } else {
        P("FAIL-storage: VkResult=%d, drop STORAGE_BIT and retry\n", (int)r);
        rc = 1;

        /* sanity: same usage minus STORAGE — should match the existing
         * VulkanGrallocCache COLOR_ATTACHMENT path, just to confirm it's
         * the STORAGE bit specifically that's rejected. */
        ici.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        VkImage img2 = VK_NULL_HANDLE;
        r = CreateImage(dev, &ici, NULL, &img2);
        P("control: vkCreateImage(COLOR_ATTACHMENT|SAMPLED) = %d %s\n",
          (int)r, r == VK_SUCCESS ? "(works)" : "(also fails)");
        if (img2 != VK_NULL_HANDLE) DestroyImage(dev, img2, NULL);
    }

    DestroyDev(dev, NULL);
    DestroyInst(inst, NULL);
    if (hd->common.close) hd->common.close(hd);
    dlclose(dso);
    return rc;
}
