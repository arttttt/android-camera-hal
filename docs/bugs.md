# Known bugs

Bugs found during testing that aren't being fixed right now. Each entry
should name the symptom, where it lives, what's known about the cause,
and when it's expected to get fixed (tier in `roadmap.md`, or "TBD").

## Video recording — pending verification after YUV_420_888 landed

**Pre-fix symptom:** Video record failed (exact mode unknown — never
observed). Root cause: `StreamConfig::normalize` was remapping
`HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED` → `RGBA_8888`
indiscriminately, but video-encoder streams
(`GRALLOC_USAGE_HW_VIDEO_ENCODER`) expect NV12 / YV12.

**State after Tier 2 YUV_420_888:** `StreamConfig` now resolves
`IMPLEMENTATION_DEFINED` with `HW_VIDEO_ENCODER` usage to
`YCbCr_420_888`, and `BufferProcessor` produces NV12 via GPU →
layout-appropriate repack into gralloc. The known root cause is gone.

**Still to verify on-device:** open a video-recording app (camera
default), record, confirm playback. If NV21 is what the Tegra
encoder requests, we'll see `YUV layout not supported` in logcat —
that's the NV21 path tracked in
[open-questions.md](open-questions.md).

## Open Camera photos come out sideways (app-side, not HAL)

**Symptom:** Photos taken with Open Camera (Mark Harman) come out
landscape when the tablet is held portrait, because `EXIF Orientation
= 1`. Libre Camera on the same device rotates correctly.

**Investigation (2026-04-20):** the HAL writes an EXIF Orientation
APP1 marker matching `ANDROID_JPEG_ORIENTATION` from the request.
Logs show Open Camera sending `req=0` regardless of how the tablet is
held; diagnostic on the Mi Pad 1 confirmed the device's rotation
sensors are broken and `mUserRotation` is locked to 0. Open Camera
relies on those sensors to compute orientation; Libre Camera uses a
different strategy and works.

**Status:** **not a HAL bug**. No fix planned here. Users on this
device can either unbreak the sensors or use Libre Camera. If
anything, the workaround would be in the app, not the HAL.
