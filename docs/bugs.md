# Known bugs

Bugs found during testing that aren't being fixed right now. Each entry
should name the symptom, where it lives, what's known about the cause,
and when it's expected to get fixed (tier in `roadmap.md`, or "TBD").

## Video recording does not work

**Symptom:** Video record fails (exact failure mode unknown — not
investigated). User confirmed during Tier 1.5 step 2 that video has
never been verified in recent sessions. Unclear whether it ever worked
on the Tegra K1 port, or when it broke.

**Location:** likely `StreamConfig::normalize` or `BufferProcessor`.
The HAL remaps `HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED` →
`HAL_PIXEL_FORMAT_RGBA_8888` indiscriminately, but video-encoder
streams (`GRALLOC_USAGE_HW_VIDEO_ENCODER`) typically expect YUV (NV12
/ YV12) — feeding them RGBA likely trips the encoder.

**Expected behaviour:** video-encoder streams keep
`IMPLEMENTATION_DEFINED` (or a YUV format advertised in static
characteristics) and the HAL produces YUV output for them.

**Likely cause:** pre-existing. No `YUV_420_888` output path exists
(listed as Tier 2 item "YUV_420_888 output"); until that lands, any
video-encoder stream falls through the generic remap and gets fed
RGBA.

**Fix plan:** folds into Tier 2 "YUV_420_888 output". Short-term,
could be rejected in `StreamConfig::normalize` with BAD_VALUE so the
framework picks a different configuration, but that's a worse UX
than "appears configurable, then fails on first frame".

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
