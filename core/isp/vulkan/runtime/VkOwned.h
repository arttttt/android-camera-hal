/*
 * Copyright (C) 2026 Artem Bambalov
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ISP_VULKAN_RUNTIME_VK_OWNED_H
#define ISP_VULKAN_RUNTIME_VK_OWNED_H

#define VK_USE_PLATFORM_ANDROID_KHR
#include <vulkan/vulkan.h>

namespace android {

/* One Vulkan object and how it is destroyed, bound when it is created:
 *
 *     createX(dev, &info, NULL, mThing.receive(dev, pfn->DestroyX));
 *
 * reset() destroys it (once, and only if it exists); so does the
 * destructor. Code that used to pair every handle with an
 * "if (h) { vkDestroyX(...); h = VK_NULL_HANDLE; }" in a teardown list
 * resets the object instead, and an object never created needs no
 * check. The caller still decides when -- objects must go before their
 * device, and after the GPU is idle. */
template <typename T>
class VkOwned {
public:
    using Destroy = void (VKAPI_PTR *)(VkDevice, T, const VkAllocationCallbacks *);

    VkOwned() = default;
    ~VkOwned() { reset(); }

    VkOwned(const VkOwned &) = delete;
    VkOwned &operator=(const VkOwned &) = delete;

    /* For a vkCreate* call: the previous object destroyed, the slot to
     * write the new handle into. */
    T *receive(VkDevice device, Destroy destroy) {
        reset();
        dev = device;
        destroyFn = destroy;
        return &handle;
    }

    void reset() {
        if (handle != VK_NULL_HANDLE && destroyFn)
            destroyFn(dev, handle, nullptr);
        handle = VK_NULL_HANDLE;
    }

    T get() const { return handle; }
    operator T() const { return handle; }
    explicit operator bool() const { return handle != VK_NULL_HANDLE; }

    /* For APIs that take an array of handles. */
    const T *addr() const { return &handle; }

private:
    T        handle    = VK_NULL_HANDLE;
    VkDevice dev       = VK_NULL_HANDLE;
    Destroy  destroyFn = nullptr;
};

} /* namespace android */

#endif /* ISP_VULKAN_RUNTIME_VK_OWNED_H */
