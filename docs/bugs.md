# Known bugs

Bugs found during testing that aren't being fixed right now. Each entry
should name the symptom, where it lives, what's known about the cause,
and when it's expected to get fixed (tier in `roadmap.md`, or "TBD").

## AE over-brightens preview (both cameras, IMX179 noticeable more)

**Symptom:** With 3A auto, preview is consistently ~1/2 stop brighter
than a reasonable middle-grey exposure. IMX179 (main) is worse than
OV5693 (front). On IMX179 low-light scenes the preview is bright
enough that the sensor hits `gain=64` (post-Q8-kernel-patch max),
which exposes a separate blue/green cast problem — see the AE
section of [tier3_architecture.md](tier3_architecture.md) and the
"gain-dependent colour cast" note below.

**Current derivation (hal/3a/AutoExposureController.cpp):**
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
up a blue/green cast once Ipa3A drives `gain` toward the 64x
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

## ISO sticks after manual → auto switch in Camera2 apps

**Symptom:** open camera in `AE_MODE = ON`, AE adapts normally. Set
manual ISO via the app (`AE_MODE = OFF` + `SENSOR_SENSITIVITY` +
`SENSOR_EXPOSURE_TIME`). Switch back to auto. ISO stops responding
to scene changes — preview "stuck" at a fixed ISO regardless of how
much the scene brightens or dims. Reproduces on IMX179 (rear) with
the test config; presumably on OV5693 too.

**Diagnosis attempts (2026-05-04):** two patches landed and were
both dropped after they didn't fix the user-visible behaviour.

1. *parseManualSettings stale-field guard* — `AutoExposureController::
   parseManualSettings` honoured `SENSOR_EXPOSURE_TIME` /
   `SENSOR_SENSITIVITY` from the request unconditionally, including
   on the auto cold-start fallback path inside `ApplySettingsStage`.
   Apps that don't strip those fields when switching to auto would
   leak manual values into the first few auto frames. Patch made the
   parser ignore them when `aeMode != OFF`. Didn't fix it; sensor
   still appeared stuck (now at the cfg-default gain ≈ ISO 100,
   which on IMX179 happens to look like the manual ISO 100 the user
   had just set).
2. *Auto-fallback writes last-applied state instead of defaults* —
   second attempt: when the IPA's push hadn't landed for the
   current frame yet, `ApplySettingsStage` queried
   `delayedControls.applyControls(frame)` and wrote the silicon's
   currently-applied exposure / gain instead of stomping with cfg
   defaults. Idea was that the sensor stays at the prior manual
   operating point until IPA takes over, AE controller adapts on
   stats reflecting that real exposure, no defaults interlude.
   User confirmed still stuck.

Both patches were dropped via `git reset --hard 0577a9c +
--force-with-lease`; history clean.

**Suspects still open:**
- `aeLockHeld` not clearing on the `OFF → ON` mode transition.
  AF triggered a sweep at the wrong moment, `setLock(true)` was
  called, but the matching `setLock(false)` never fired (or fired
  before the diag log captured it). The lock branch in
  `AutoExposureController::process` then re-publishes the held
  exposure / gain forever.
- `filteredTotalUs` carrying a stale state from a long-past auto
  session, with the LPF damping = 0.05 simply not converging fast
  enough to look responsive. Visually indistinguishable from
  "stuck" if the recovery takes more than a few seconds.
- A race between `ApplySettingsStage` (RequestThread) reading
  `pendingWrite(frame)` and the IPA (PipelineThread) writing the
  same slot — if the read consistently wins, the auto branch
  always falls through to its fallback and the IPA's decisions
  never reach the sensor.

**Where to look next:**
- Per-frame log of `aeLockHeld`, `aeMode`, `aeLock` and the
  raw V4L2 write source for ~5 s spanning the manual→auto switch.
  The throttled `3A:` log catches one sample per 32 frames, which
  is too coarse to localise the lock entry / exit edges.
- Confirm whether `mAe->setLock(false)` is called via
  `afResult.sweepComplete` after a sweep that started during the
  manual phase — print AF state transitions alongside.
- Check if the IPA push for stats-from-frame-N actually lands at
  slot `N + delay` before `ApplySettingsStage`'s read for frame
  `N + delay` runs. If not, the race theory is the bug.

**Status:** open, no fix planned for this iteration. Doesn't block
the AWB-Bayes plan; address as a separate Tier 4 AE-stability item.

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
