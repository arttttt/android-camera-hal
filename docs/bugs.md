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

## AE over-brightens preview (both cameras, IMX179 noticeable more)

**Symptom:** With 3A auto, preview is consistently ~1/2 stop brighter
than a reasonable middle-grey exposure. IMX179 (main) is worse than
OV5693 (front). On IMX179 low-light scenes the preview is bright
enough that the sensor hits `gain=64` (post-Q8-kernel-patch max),
which exposes a separate blue/green cast problem — see the AE
section of [tier3_architecture.md](tier3_architecture.md) and the
"gain-dependent colour cast" note below.

**Current derivation (hal/ipa/BasicIpa.cpp):**
`aeSetpoint = pow(mid / 255, 2.2)` where `mid = (HigherTarget +
LowerTarget) / 2` from `active.ae.MeanAlg`. On the shipped tunings:

- IMX179: `mid = 115` → setpoint ≈ **0.174** linear
- OV5693: `mid = 130` → setpoint ≈ **0.227** linear

0.174 is the photographic 18 % middle-grey target. That's correct
for a scene whose gray-world average *is* 18 % reflectance; typical
indoor shots average 10-15 %, so AE's ratio drive pushes one half-
stop past where the scene would naturally land.

**Candidate causes (untested, ranked by likely impact):**

1. **No lens-shading correction.** `reserved.lensShading` +
   `reserved.falloff_srfc` are unused. IMX179 typically has 20-30 %
   corner falloff; the AWB / AE stats include the darker corner
   patches, pushing mean luma down and AE exposure up. This is the
   most likely root and matches the IMX179-worse-than-OV5693
   pattern (IMX179's optics have stronger vignetting on this
   module).
2. **Gamma exponent.** Our `pow(x, 2.2)` is the approximate sRGB
   decode, but sRGB's true inverse is piecewise with an effective
   exponent closer to 2.4 for the non-linear segment. Using 2.4
   would shift the target to 0.147 / 0.195 — about 15 % darker.
3. **NVIDIA's MeanAlg target is in a domain other than sRGB.** If
   it's `Y = 0.3R + 0.6G + 0.1B` of post-WB raw, our linear G-only
   histogram samples a different slice of the metric.
4. **`SlopFactor=0.3`, `ToleranceOut=0.8`, `{Min,Max}RGBtoYMix`** —
   additional MeanAlg knobs we read but don't use for setpoint
   shaping.

**Planned work (TBD):** start with (1) lens shading as the biggest
lever; (2) sRGB-correct gamma decode if the cast persists after LSC.

## Gain-dependent blue/green colour cast at high analog gain (IMX179)

**Symptom:** In low-light scenes on the main camera, preview picks
up a blue/green cast once BasicIpa drives `gain` toward the 64x
post-kernel-patch ceiling. Disabling AE in the camera app removes
the cast — the framework picks a more moderate gain and the image
goes darker instead.

**Suspected cause:** sensor-level black-level drift per channel at
high analog gain. Our `opticalBlack.r=64` static subtract is
calibrated at unity gain; the real channel-specific black level
drifts upward with analog gain, and after the static subtract R/G/B
end up with non-equal offsets. WB amplifies the imbalance, CCM
propagates it.

**Planned fixes (TBD):**

- **VFRTable** (`reserved.ae.VFRTable`) — let AE drop frame-rate to
  extend exposure in low light instead of reaching for max gain.
  Standard NVIDIA behaviour; the table is already in the tuning.
- **Gain-dependent optical black** — honest fix if the tuning or
  sensor OTP carries per-gain black-level curves. Need to dig.
- **Soft cap on max gain** — interim mitigation; `gain<=32` loses
  one stop of low-light capability but likely removes the cast.

## Auto AE cannot be turned off

**Symptom:** Switching the camera app to manual AE (AE_MODE=OFF) does
not actually disable auto AE — BasicIpa keeps driving exposure / gain
regardless of the request mode.

**Status:** Logged, no fix scheduled. Tier TBD.

## Exposure compensation ignored while auto AE is on

**Symptom:** With auto AE active, the framework's exposure-compensation
control (ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION) has no visible
effect on preview brightness.

**Status:** Logged, no fix scheduled. Tier TBD.

## V4L2 fd reopened on every camera open — should only reopen on stream config change

**Symptom:** After dropping `V4L2DEVICE_OPEN_ONCE` the V4L2 fd now closes
on every `closeDevice` and reopens on every `openDevice`. That carries
the full `S_FMT` + `REQBUFS` + buffer reallocation cost on each
open/close cycle, even when the next session uses the exact same
stream configuration as the previous one.

**Better policy (TBD, needs investigation):** keep the fd open across
`closeDevice → openDevice` and only reopen when `configureStreams`
actually requests a different resolution / pixel format / memory type
than what the device is already set up for. May need to keep more than
just the fd alive — current `cleanup()` also drops `mFormat`, the
DMABUF binding, and the buffer mappings. Investigate what's safe to
keep across a session boundary vs what the V4L2 contract requires
fresh.

**Status:** Logged, no fix scheduled. Tier TBD. Removing the original
flag was the right move; the replacement just over-cleans.

## Tap-to-focus ignored — AF_REGIONS not consumed

**Symptom:** Tapping a non-centre region in the camera app does not
move the AF ROI. The lens still sweeps based on the centre patches
regardless of where the user tapped.

**Cause:** `AutoFocusController` ignores
`ANDROID_CONTROL_AF_REGIONS` from request settings. The AF ROI is
hardcoded to the centre 8×8 patches via `kRoiPatchLo / kRoiPatchHi`
in `hal/3a/AutoFocusController.cpp`. The default request template
sets `AF_REGIONS` to the full sensor rectangle and no per-request
value is read.

**Status:** Logged, no fix scheduled. Tier TBD. When fixed, the
encoder's spatial Sobel/greenSq ROI also needs to follow the
dynamic AF region.

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
