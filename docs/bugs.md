# Known bugs

Bugs found during testing that aren't being fixed right now. Each entry
should name the symptom, where it lives, what's known about the cause,
and when it's expected to get fixed (tier in `roadmap.md`, or "TBD").

## AF trigger starts sweep even with `AF_MODE = OFF`

**Symptom:** HAL runs a contrast-detect sweep when the camera app has
turned autofocus off. Reported by user during Tier 1 testing.

**Location:** `hal/Camera.cpp` — the `ANDROID_CONTROL_AF_TRIGGER` block
in `processCaptureRequest` (the `trigger == AF_TRIGGER_START` branch
starts a sweep without checking `afMode`).

**Expected behaviour:** with `AF_MODE = OFF`, the HAL must not sweep.
Sweeps are only valid in `AF_MODE_AUTO` / `MACRO` / `CONTINUOUS_PICTURE`.
Manual focus from `LENS_FOCUS_DISTANCE` is the only VCM motion allowed
when `AF_MODE = OFF`.

**Likely cause:** the trigger handler doesn't guard on `afMode`. Also
worth auditing whether the `CONTINUOUS_PICTURE` periodic re-trigger
can bleed state if mode changes mid-session (`mAfSweepActive` cleanup
on mode transition).

**Fix plan:** will be covered when the 3A module lands (Tier 3), since
AF behaviour gets moved out of `Camera.cpp` entirely. Short-term fix
if the bug blocks anyone: add `&& afMode != AF_MODE_OFF` to the trigger
guard, and clear `mAfSweepActive` on `AF_MODE` transition.

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
