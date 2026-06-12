# Platform abstraction: core / adapter split

Target architecture for surviving Android version bumps and for the
planned **native binderized HIDL port** (no legacy-HAL wrapping) on
lineage-15.1+. Decided June 2026; this document records the layout,
the seams, the migration order, and the evidence behind them.

## Evidence base: the N→O port took 3 commits

The lineage-15.1 bring-up needed exactly three changes, all in the
platform rim, zero in the camera logic:

| Commit | What broke | Class |
|---|---|---|
| `1af4575` | `GraphicBuffer` wrap ctor changed | libui C++ ABI leaked into core (`BufferProcessor`) |
| `839730c` | `vulkan.h` left the NDK prebuilt path | platform header layout in build glue |
| `VulkanPfn` guards | O's vulkan.h ships the KHR types we polyfill | unguarded polyfill |

Untouched: `v4l2/`, `isp/` (demosaic, ROP, encode), `hal/ipa/`, NEON
stats, pipeline threading, JPEG, metadata builders. The "core is
platform-independent" claim is a measurement, not an estimate.

The lesson generalizes: **C APIs survive version bumps, framework C++
classes do not.** `camera3.h`, `camera_metadata_t`, gralloc module
ops, `sync_fd` are vendor ABI and effectively frozen;
`android::GraphicBuffer` / `Fence` / `CameraMetadata` are internal
framework classes Google rebuilds the world around every release.

## Target layout

```
/vendor/lib/libmochacam.so          core: V4L2 source, Vulkan ISP, 3A/IPA,
                                    stats, pipeline, JPEG, metadata builders
/vendor/lib/hw/camera.tegra.so      adapter-camera3: thin camera3 shell
                                    (lineage-14.1; transitional on 15.1)
/vendor/bin/hw/android.hardware.camera.provider@2.4-service.mocha
                                    adapter-hidl: native ICameraProvider/
                                    ICameraDevice/ICameraDeviceSession
                                    (lineage-15.1+ end state)
```

Repo layout mirrors it: `core/`, `adapter-camera3/`, `adapter-hidl/`,
with `include/mochacam/` as the core's public interface and a build
check that nothing under `core/` includes `ui/*`, `camera/CameraMetadata.h`
or other framework C++ headers.

Branch matrix:

| | lineage-14.1 | lineage-15.1 transitional | lineage-15.1+ target |
|---|---|---|---|
| libmochacam.so | yes | yes | yes (same sources) |
| camera.tegra.so | yes | yes (passthrough) | kept as an A/B fallback, then dropped |
| HIDL service | – | – | yes (binderized) |

## ABI of the core boundary

- **C++ interface is fine.** Core and adapters always build together
  from one tree per branch; there is no goal of carrying one binary
  across system versions (open source — rebuilding is cheap). The
  .so boundary exists for dependency discipline and reuse of sources,
  not for binary stability.
- **Data currencies are C types regardless:** `buffer_handle_t`,
  `int sync_fd`, `camera_metadata_t*`, own POD structs for stream
  config. Not for ABI reasons — because these are the native types of
  both worlds the adapters live in:

  | Entity | camera3 (N..) | HIDL device@3.2 |
  |---|---|---|
  | stream buffer | `buffer_handle_t` | `hidl_handle` → `native_handle_t` |
  | fences | `int sync_fd` | `hidl_handle` wrapping a sync fd |
  | metadata | `camera_metadata_t*` | `hidl_vec<uint8_t>` = serialized `camera_metadata_t` |

  A core speaking these types makes either adapter a translation-free
  shim. `android::GraphicBuffer`/`Fence`/`CameraMetadata` do not exist
  in HIDL interfaces at all.

## The seams (work items)

1. **Evict libui from core.** Three sites: the preview zero-copy wrap
   in `BufferProcessor::tryZeroCopy` (fill a plain `ANativeWindowBuffer`
   struct by hand — it is stable C from `system/window.h`; the
   GraphicBuffer wrapper class adds nothing), the scratch allocation in
   `VulkanIspPipeline::createScratchImage`, and the CPU lock/unlock
   paths — both via a four-op gralloc micro-wrapper (alloc / free /
   lock / unlock) owned by core with a swappable backend (gralloc0
   module now, IMapper/IAllocator HIDL on newer bases). This seam also
   erases the only commit that differentiates the branches today.
2. **Vendor the Khronos Vulkan headers** (1.0.61+) into the repo;
   include ours everywhere, delete the `VulkanPfn.h` polyfills, drop
   the platform include path from build glue. The loader is already
   dlopen-based — headers are the only platform coupling.
3. **`android::Fence` → `sync_wait()`** (libsync). Two call sites.
4. **Metadata builders on `camera_metadata_t` C API** (or a 50-line
   own wrapper). `android::CameraMetadata` stays adapter-side only;
   conversion at the rim is free (it wraps the same pointer).

## Native HIDL adapter (lineage-15.1+ end state)

`ICameraProvider@2.4` registered as `legacy/0` (cameraserver's naming
convention), `ICameraDevice@3.2` (`getCameraCharacteristics` = static
metadata as `hidl_vec`), `ICameraDeviceSession@3.2`:
`constructDefaultRequestSettings` = RequestTemplateBuilder,
`configureStreams` = StreamConfig policy, `processCaptureRequest` +
`notify`/`processCaptureResult` = the existing async pipeline, which
already matches the HIDL model (non-blocking submit, results via
callback).

Implementation notes collected up front:

- **Buffer cache protocol**: device@3.2 sends each stream buffer once
  as `(bufferId, handle)`, then by id. The session owns an
  import-cache keyed by bufferId, flushed on stream reconfigure/close
  — this is exactly what the gralloc micro-wrapper is for.
- **FMQ**: `getCaptureRequest/ResultMetadataQueue` may legally return
  zero-sized queues at first (metadata travels in hidl args); add FMQ
  later if binder overhead ever shows.
- **Vendor process**: the HAL leaves cameraserver. /dev/video*, nvmap,
  nvhost, the Vulkan blob must be reachable from the service's domain
  (15.1 runs permissive — not a blocker, but a new debugging surface).
- **No flash**: `setTorchMode` → `METHOD_NOT_SUPPORTED`.
- Performance is a non-issue: buffers cross binder as handles, never
  as pixels — the standard Treble arrangement.

## Migration order (and why)

1. **Finish the passthrough 15.1 bring-up to a working camera on
   device.** The camera3 shell is needed alive for 14.1 anyway, and it
   gives a known-good reference of core-on-O before any HIDL code
   exists. Never debug two unknowns at once.
2. **Seams 1–4** on the live system, verified on device with the
   established loop (mmm → push → photo/video). After them the
   branches differ by build glue only.
3. **Split the .so** — mechanical once seams hold: three build
   modules, `include/mochacam/` as the contract.
4. **Write adapter-hidl against the library.** Switch 15.1 by
   device.mk/manifest; keep camera.tegra as the A/B fallback during
   stabilization — one core, two transports is a debugging gift.
5. lineage-14.1 keeps adapter-camera3 + the same core sources
   indefinitely; 16.0+ is a provider 2.5/2.6 bump in the adapter.
