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

## JPEG orientation not applied

**Symptom:** Photos from both back and front cameras come out on their
side — 1920×1080 pixels, no EXIF Orientation tag, no pixel rotation.
Reproducible in OpenCamera on Shield Tablet.

**Location:** `hal/Camera.cpp` — `HAL_PIXEL_FORMAT_BLOB` branch in
`processCaptureRequest`, the `jpegOri` read + `libyuv::ARGBRotate`
path.

**Expected behaviour:** pixels rotated by `ANDROID_JPEG_ORIENTATION`
(HAL's current contract) *or* EXIF Orientation tag written by the JPEG
encoder (stdlib contract). Either satisfies viewers.

**Likely cause:** `cm.exists(ANDROID_JPEG_ORIENTATION)` returns false,
or value is 0. Default request template in
`constructDefaultRequestSettings` hardcodes `jpegOrientation = 0`;
OpenCamera may not override. Unconfirmed — needs a `jpegOri` log line.

**Confirmed pre-existing:** photo taken before the echo-metadata
changes (`IMG_20260419_035946.jpg`, 03:59) shows the same symptom.

**Fix plan:** folds into Tier 1.5 item 4 (zero-copy JPEG). The new
shader-rotation path naturally respects `JPEG_ORIENTATION`. If Tier 1.5
slips, short-term fix is to log `jpegOri` and investigate why it's 0.
