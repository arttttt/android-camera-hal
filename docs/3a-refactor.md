# 3A refactor — partial-driven coordination, pure controllers

Migration plan for splitting `BasicIpa` into a thin coordinator
(`Ipa3A`) over three pure-math controllers (`AutoExposureController`,
`AutoWhiteBalanceController`, `AutoFocusController`), with each
controller's result published to the framework as a separate Camera2
partial result instead of being merged into a single per-frame
metadata blob.

## Why

Current state (`hal/ipa/BasicIpa.cpp`):

- AE, AWB, CCM math all live in `BasicIpa::processStats` — one class,
  three separate concerns. SRP violation grew over time as 3A
  features matured.
- AE math sits in `BasicIpa`; AE V4L2 apply sits in
  `hal/3a/ExposureControl`. The split is along the wrong seam — math
  vs apply are coupled inside one subsystem, not between subsystems.
- `AutoFocusController` reaches into `Ipa::isAeConverged()` /
  `Ipa::setAeLock()` / `IspPipeline::setAwbLocked()` directly to
  coordinate locks and retrigger gating — controllers know about each
  other through opaque pointer dependencies.
- `PARTIAL_RESULT_COUNT = 1` in static characteristics. Result
  metadata is built once at end-of-pipeline by `ResultMetadataBuilder`
  pulling from multiple sources (Ipa convergence flag, AfController
  state, IspPipeline awb-lock flag, DelayedControls applied values).
  Contract-correct but doesn't use the framework's partial-result
  feature even though the natural shape of 3A is incremental.

Target state:

- Three controllers, each pure: `process(stats, meta, deps...) →
  Result`. No I/O on backends from inside controllers. State internal
  to each (lastWb, filteredTotalUs, AfState etc.), inputs/outputs
  explicit in signatures.
- `Ipa3A` is a thin sequential dispatcher: call AWB → emit partial →
  call AE (with AWB result) → emit partial → call AF (with AE
  convergence) → emit partial. Routes Result outputs to backends
  (`IspPipeline::setWbGains`, `DelayedControls::push`,
  `V4l2Device::setControl` for VCM) directly, no abstraction layer.
- `ExposureControl` removed. AE manual-mode math (clamps, EV-comp,
  frame_length compute) becomes a static helper on
  `AutoExposureController`; V4L2 apply becomes a direct
  `V4l2Device::setControls` call from `ApplySettingsStage`.
- Locking becomes implicit: when AF is sweeping, `Ipa3A` simply
  doesn't call AE/AWB controllers that frame. No lock flags
  threaded through anywhere.
- `PARTIAL_RESULT_COUNT` bumped (likely to 3 — AWB / AE / AF — final
  capture metadata folds into the last partial). Apps that gate UI on
  e.g. AE state see it without waiting for the full result.

## Out of scope

- Changing 3A algorithms themselves. AE remains EV-space LPF +
  highlight constraint + dead-band + asymmetric speed. AWB remains
  gray-world + 96-patch confidence gate + EMA-relax-to-prior + CCT
  LERP. AF remains CDAF state machine. The only goal is
  re-organisation; behaviour at any given frame should be byte-identical
  before/after each migration step.
- Bayesian AWB, lens-shading correction, noise model tuning JSON, and
  the rest of `libcamera-steal-list.md`. Those are quality
  improvements; this refactor is pure structural.
- Async per-controller threading. Controllers run sequentially in the
  existing `StatsProcessStage` callback on `PipelineThread`. No new
  thread infrastructure.

## Migration sequence

Each step compiles cleanly, deploys, and is smoke-tested on the
device before the next step lands. Each is a single atomic commit
unless explicitly noted. Commit messages do not contain phase /
step labels (per `feedback_no_internal_stage_labels.md`).

### Step 1 — `PartialEmitter` infrastructure + bump `PARTIAL_RESULT_COUNT`

**Goal:** make the dispatch path capable of emitting multiple
`process_capture_result` calls per frame with correct
`partial_result` counter sequencing, without changing the source
of metadata yet.

**Files touched:**
- `hal/metadata/CameraStaticMetadata.cpp` — bump
  `ANDROID_REQUEST_PARTIAL_RESULT_COUNT` from 1 to 3.
- `hal/pipeline/stages/ResultDispatchStage.{h,cpp}` — accept a
  `PartialEmitter` callback the IPA / metadata builder can invoke;
  drive `partial_result` counter (1..N) and final-flag logic.
- `hal/Camera.cpp` — wire callback through
  `processCaptureResult(..., partial_result)`.

**Acceptance:**
- HAL still emits exactly one full result per frame (no behaviour
  change yet) — but emitted as `partial_result = 3` (final) with all
  metadata. Single-emit path works through new infrastructure.
- Bumped `PARTIAL_RESULT_COUNT` advertised correctly via
  `cameraInfo()`.

**Test:**
- Open Camera, Libre Camera, LineageOS stock camera — preview, photo
  capture, video record. No regression vs current build.
- `logcat | grep "process_capture_result"` shows partial_result=3 on
  every frame, no skipped counters.
- Apps that don't use partials should be unaffected (final partial
  carries all metadata as before).

**Regression watch:**
- Apps that listen on `partial_result` count change behaviour
  prematurely. If observed, pin emission at partial=N=count
  (matches "no progressive partials" semantics until later steps
  actually populate intermediate partials).
- `AVAILABLE_RESULT_KEYS` validation in Camera2 framework — should
  still pass since the final partial contains the complete set.

**Estimated LOC:** ~120 (mostly plumbing).

### Step 2 — Define controller interfaces and Result structs

**Goal:** establish the shape of pure-return controllers without
moving any logic yet.

**Files touched:**
- `hal/3a/AeResult.h`, `hal/3a/AwbResult.h`, `hal/3a/AfResult.h` —
  new. Pure data, no methods.
- (No common abstract `ThreeAController` base class — controllers
  have different signatures and the abstraction would be thin
  scaffolding; revisit if a use case appears.)

```cpp
struct AeResult {
    DelayedControls::Batch batch;       // exposure, gain
    uint8_t                state;       // ANDROID_CONTROL_AE_STATE_*
    bool                   converged;
};

struct AwbResult {
    std::optional<WbGains> gains;       // r/g/b Q8
    std::optional<CcmQ10>  ccm;         // 9 entries row-major
    int                    estCct;      // K, for diagnostics
    uint8_t                state;       // ANDROID_CONTROL_AWB_STATE_*
};

struct AfResult {
    std::optional<int32_t> vcmPosition;
    bool                   startSweep;
    bool                   sweepComplete;
    uint8_t                state;       // ANDROID_CONTROL_AF_STATE_*
};
```

**Acceptance:** code compiles. No behaviour change.

**Test:** sanity build only, no on-device test.

**Estimated LOC:** ~60.

### Step 3 — Extract `AutoWhiteBalanceController`

**Goal:** AWB math + CCM update lives in its own class. `BasicIpa`
calls it, takes the `AwbResult`, and routes outputs the same way it
did inline.

**Files touched:**
- `hal/3a/AutoWhiteBalanceController.{h,cpp}` — new. Owns
  `lastWbR/B`, `wbRPrior/BPrior`, `awbDamping`, `awbMinChannel`,
  `awbSceneLightFloor`, GrayLine clamp parameters. Public API:
  `AwbResult process(stats, meta)`, `void reset()`,
  `WbGains currentGains() const` (for AE highlight constraint).
- `hal/ipa/BasicIpa.{h,cpp}` — owns an instance of
  `AutoWhiteBalanceController`, delegates AWB block of
  `processStats` to it, applies `result.gains` via
  `IspPipeline::setWbGains` and `result.ccm` to `mCcmBuffer`.

**Acceptance:**
- AWB behaviour bit-identical to before (same `lastWb` evolution,
  same gate decisions, same CCM LERP).
- BasicIpa AWB code reduced to a delegation call + result
  application.

**Test:**
- Static dim scene front cam — `lastWb` evolution matches pre-step
  log (compare frame-by-frame `lastWb=(R,B)` values for first 100
  frames after open).
- Coloured scene (warm light) on rear cam — CCM LERP visibly
  follows estCct same as before.
- AWB lock toggle — gains freeze/resume identically.

**Regression watch:**
- Order of `setWbGains` vs `setCcm` calls — must remain unchanged
  from current `BasicIpa::processStats` ordering.
- WbGains structure layout — accidentally swapping R/B between
  controller's internal model and `IspPipeline::setWbGains(r,g,b)`
  argument order.

**Estimated LOC:** ~250 (controller ~200, BasicIpa edits ~50).

### Step 4 — Extract `AutoExposureController`

**Goal:** AE math (auto path with EV-space LPF / dead-band /
highlight constraint / asymmetric speed) lives in its own class.
Manual-mode parsing (currently `ExposureControl::onSettings`) lives
as a static helper on the same class.

**Files touched:**
- `hal/3a/AutoExposureController.{h,cpp}` — new. Owns
  `filteredTotalUs`, `smoothedLuma`, `lockedBiasedTotalUs`,
  `lastEvComp`, `aeLockHeld`, `aeConvergedFrames`. All
  `aeXxx`-prefixed members from BasicIpa. Public API:
  `AeResult process(stats, meta, currentWb)`, `void reset()`,
  `bool isConverged() const`, `void setLock(bool)`.
- `hal/3a/AutoExposureController.{h,cpp}` static helper:
  `static DelayedControls::Batch parseManualSettings(meta,
  sensorCfg)` — pure deterministic mapping of `ANDROID_SENSOR_*`
  metadata to `Batch`. No state.
- `hal/ipa/BasicIpa.{h,cpp}` — owns an instance, delegates AE block.
  Pulls current WB from `mAwb->currentGains()` if AWB skipped this
  tick, otherwise from fresh `awbResult.gains`.

**Acceptance:**
- AE behaviour bit-identical (same `filteredTotalUs` evolution, same
  IQM-highlight constraint application).
- AE_LOCK + EV-comp work identically.
- Manual mode: `ApplySettingsStage` still uses the old
  `ExposureControl::onSettings` path until Step 7 — no change yet.

**Test:**
- Static dim scene front cam — gain trajectory matches pre-step
  exactly across step transitions and stationary phases.
- Step transitions (cover/uncover lens) — convergence within τ=4
  frames as before.
- AE lock + EV slider — exposure tracks slider through EMA-smoothing
  same as before.

**Regression watch:**
- Highlight constraint silently broken if `currentWb` arg not
  threaded correctly (e.g. AWB skipped this frame, fall back to
  `mAwb->currentGains()` returning `wbGainPrior` instead of last
  converged WB).
- EV compensation factor applied at the wrong stage in the new
  `process` signature.

**Estimated LOC:** ~350 (controller ~300, BasicIpa edits ~50).

### Step 5 — Refactor `AutoFocusController` to pure return-style

**Goal:** AF stops writing V4L2 directly and stops dereferencing
`Ipa::isAeConverged()` / `IspPipeline::setAwbLocked()`. Returns
`AfResult` from `process(stats, meta, aeConverged)`. Keeps
`onSettings(meta, frameNumber)` for RequestThread-side trigger
handling (unchanged).

**Files touched:**
- `hal/3a/AutoFocusController.{h,cpp}` — extensive refactor.
  Internal state machine logic kept; "side effects" replaced with
  Result population. Drop `mIpa`, `mIsp`, `mDev` (focuser) members
  except where genuinely required for setup (focuser open path stays
  on the V4l2Device).
- `hal/ipa/BasicIpa.{h,cpp}` — calls AF after AE, passes
  `aeResult.converged`. Routes `afResult.vcmPosition` to focuser via
  `V4l2Device::setControl(V4L2_CID_FOCUS_ABSOLUTE, ...)`.
- `hal/pipeline/stages/StatsProcessStage.cpp` — single call site
  becomes IPA-only; no separate `af->onStats()` invocation.

**Acceptance:**
- AF state machine timing identical (Coarse1/Coarse2/Fine/Settle
  durations, parabolic peak, retrigger logic all preserved).
- VCM writes happen at exactly the same frames they did before.
- AE/AWB lock during sweep happens implicitly: BasicIpa doesn't call
  `mAe->process` or `mAwb->process` while AF reports
  `state == ACTIVE_SCAN`.

**Test:**
- Tap-to-focus on tap region — AF sweep starts, AE/AWB visibly
  freeze through sweep duration, both resume after focus lock.
- Continuous AF retrigger on scene change — same behaviour as before
  (no spurious triggers, real composition changes fire it).
- Manual focus mode — focus distance from request applied to VCM
  identically.
- Front camera (fixed-focus OV5693) — AF mode reports
  `INACTIVE`/`PASSIVE_*` correctly, no VCM writes attempted.

**Regression watch:**
- AE-converged gate logic for retrigger — if `aeConverged` arg
  threaded wrong, AF will retrigger spuriously during AE convergence
  (or never retrigger at all).
- Sweep-locking implicit logic — if BasicIpa calls AE during sweep,
  AF score curve gets brightness-modulated.
- Focuser V4L2 setControl path — current code uses
  `V4l2Device::openFocuser` opened on a separate subdev; that stays.
  Just the write call moves.

**Estimated LOC:** ~400 (heaviest refactor in the chain).

### Step 6 — Rename `BasicIpa` → `Ipa3A`, slim down to coordinator

**Goal:** the rename + make the body of `processStats` reflect its
final form (sequential dispatch + partial emit + backend routing).

**Files touched:**
- `hal/ipa/BasicIpa.{h,cpp}` → `hal/ipa/Ipa3A.{h,cpp}`. Mechanical
  rename plus body simplification — the file at this point is mostly
  delegation, ready for partial-emit additions in Step 8.
- `hal/Camera.{h,cpp}` — references updated.
- `hal/pipeline/stages/StatsProcessStage.cpp` — references updated.
- Memory: `project_tier3_progress.md`, `MEMORY.md` index.

**Acceptance:**
- File renamed, all references updated.
- `Ipa3A::processStats` is short — three controller calls plus
  routing logic, no embedded math.

**Test:**
- Smoke run on device — preview + photo + video on both cameras.
  No behaviour change expected.

**Regression watch:**
- Stale references to `BasicIpa` in `Android.mk`, includes elsewhere,
  symbol names. Build catches most.
- Memory file paths — if `MEMORY.md` index entry name changes.

**Estimated LOC:** ~80 (mostly rename + small body cleanup).

### Step 7 — Remove `ExposureControl`

**Goal:** delete the class. Manual-mode AE math goes through
`AutoExposureController::parseManualSettings` (added in Step 4).
V4L2 writes happen direct from `ApplySettingsStage` via
`V4l2Device::setControls`.

**Files touched:**
- `hal/3a/ExposureControl.{h,cpp}` — **deleted**.
- `hal/pipeline/stages/ApplySettingsStage.cpp` — refactored:
  - Manual path: call `AutoExposureController::parseManualSettings(
    meta, sensorCfg)`, write `Batch` via
    `V4l2Device::setControls(batch)`.
  - Auto path: read `delayedControls->pendingWrite(frameNumber)`,
    write batch via `V4l2Device::setControls`.
  - Cold-start fallback: if auto path returns empty batch, fall back
    to `parseManualSettings`.
- `hal/Camera.{h,cpp}` — drop `mExposure` member, drop
  `ExposureControl` includes / dependencies.
- `Android.mk` — remove `ExposureControl.cpp` from sources.

**Acceptance:**
- All AE V4L2 writes still happen at the right frames.
- Applied exposure / gain reported in result metadata correctly
  (via `DelayedControls::applyControls(seq)`).
- Manual exposure mode still functional in apps that use it.

**Test:**
- Open Camera in manual mode (set ISO 400, exposure 1/60) —
  applied values reach the sensor, preview shows the requested
  brightness.
- Open Camera mode-switch auto ↔ manual ↔ auto — exposure tracks
  through transitions same as before.
- LineageOS Camera in default auto — no behaviour change.

**Regression watch:**
- Frame-length math in `parseManualSettings` — `ExposureControl::
  onSettings` had a "grow frame_length on long manual exposure" path
  (commit `3e4e2d9`). That logic must move with the math.
- Applied state cache — `ExposureControl::report()` was used by
  `ResultMetadataBuilder` to pull `appliedExposureUs` /
  `appliedGain`. After removal, builder must read from
  `DelayedControls::applyControls(frameNumber)` exclusively (which
  is the source of truth anyway, just verify no stale references).
- AE_MODE manual ↔ auto transition handling — current code in
  `Camera.cpp` has `mLastAeMode` tracking for in-HAL flush triggers.
  Make sure it still works.

**Estimated LOC:** -200 / +50 (net deletion).

### Step 8 — Wire partial emits per controller

**Goal:** the actual structural payoff. Each controller's result
becomes its own partial.

**Files touched:**
- `hal/ipa/Ipa3A.cpp` — receives `PartialEmitter` reference in
  `processStats(stats, meta, sequence, emitter)`. After each
  controller call, builds the subset of metadata for that controller
  (`buildAwbMetadata(awbResult)`, `buildAeMetadata(aeResult)`,
  `buildAfMetadata(afResult)`) and calls
  `emitter.emit(metadata, partial=N)`.
- `hal/metadata/ResultMetadataBuilder.{h,cpp}` — split monolithic
  `build()` into `buildAwbMetadata`, `buildAeMetadata`,
  `buildAfMetadata`, `buildBaseMetadata` (sensor info, lens info,
  capture metadata). Final partial is `buildBaseMetadata` — fired
  from `ResultDispatchStage` for buffer-bearing emit.
- `hal/pipeline/stages/StatsProcessStage.cpp` — passes
  `PartialEmitter` from the dispatch path into `Ipa3A::processStats`.

**Acceptance:**
- Each frame produces exactly 3 partial results from the IPA tick
  + 1 final partial from buffer-bearing dispatch = 4 emits per
  frame. (Or 3 if base metadata folds into AF partial as the final
  one. Pick at implementation time based on which carries
  buffer pointers cleanest.)
- Each partial has correct `partial_result` counter, framework
  expects monotonic 1..N.
- All metadata that was in the pre-step single-result still appears
  in *some* partial; the union equals the pre-step metadata set.

**Test:**
- All 3 cameras × all 3 apps smoke run. No visible regression.
- `dumpsys media.camera` or similar — verify partial result counter
  matches `PARTIAL_RESULT_COUNT` from characteristics.
- Apps that latch on `ANDROID_CONTROL_AE_STATE` (e.g. AE indicator
  in Open Camera) — should react sooner now (after AE partial,
  before AF partial). Not a regression, just a timing improvement.

**Regression watch:**
- Frame number consistency across partials of the same frame.
  `ResultDispatchStage` previously held the single emit; partial
  emits from IPA tick must use the same `frameNumber`.
- Buffer release: `process_capture_result` with `output_buffers !=
  null` must come once per frame (the final partial). All
  intermediate partials carry zero buffers. Camera3 framework
  enforces this; getting it wrong gives `BAD_VALUE`.
- Metadata key partitioning — every key that the framework expects
  on at least one result for a given frame must be present in some
  partial. Missing keys cause apps to see `null` where they
  shouldn't. Cross-check `AVAILABLE_RESULT_KEYS`.

**Estimated LOC:** ~250 (mostly metadata builder splitting).

### Step 9 — Cleanup pass + memory + docs sync

**Goal:** loose ends.

- Verify no `BasicIpa` / `ExposureControl` references remain in
  `tools/`, `tests/`, build files, doc strings.
- Update `README.md` (top-level) 3A description to reflect
  three-controller + coordinator + partial-driven shape.
- Update `docs/architecture.md` 3A summary.
- Update `docs/tier3_architecture.md` BasicIpa AE loop section,
  rename to "Ipa3A coordination + per-controller behaviour".
- Update memory `project_tier3_progress.md` AE design section.
- Remove this `docs/3a-refactor.md` once the refactor is done — the
  document is migration-only, not steady-state docs.

**Acceptance:**
- All docs reflect post-refactor state.
- Memory consistent with code.

**Estimated LOC:** doc-only changes, ~150 across files.

## Total estimated effort

~1700 LOC delta across 9 commits. ~3 commits per session is
reasonable; budget 3-4 sessions including on-device smoke testing
between every step.

## Risks

- **Partial result counter bugs.** The framework is strict — wrong
  counter, missing partials, partials out of order all cause errors.
  Step 1 should be smoke-tested very carefully before later steps
  build on it.
- **Behaviour drift during extraction.** "Bit-identical" claim is
  hard to prove without per-frame log diffing. Mitigation: keep the
  `Cam-BasicIpa: 3A:` log line format stable across steps so log
  comparison stays valid; add per-frame log temporarily during
  controller extraction if needed.
- **Lock semantics during AF sweep.** Implicit locking via
  "don't call controller this tick" is cleaner than explicit lock
  flags but easier to get subtly wrong (e.g. AE controller's
  internal EMA state isn't refreshed during the lock, so it resumes
  with stale smoothed luma post-sweep). Verify post-sweep AE
  resumes from a sensible point on a step scene.
- **`AutoFocusController` is the heaviest refactor** (Step 5) and
  the one most prone to regressing CDAF behaviour. Land it first
  on rear cam (where AF actually moves) and verify peak-finding /
  retrigger before the other steps' assumptions stack on top.

## Test matrix

Each step that touches behaviour gets a smoke-test pass on the
device covering at minimum:

| App | Camera | Mode | What to verify |
|---|---|---|---|
| Open Camera | rear (IMX179) | Auto photo | AE convergence on dim/bright, AWB neutral, AF on tap |
| Open Camera | rear | Manual ISO/shutter | exposure values reach sensor |
| Libre Camera | rear | Auto photo | sanity (different Camera2 client) |
| LineageOS Camera | rear | Video record | preview + recording, no dropped frames |
| Open Camera | front (OV5693) | Auto photo | AE on dim scene (no pulsation) |
| Open Camera | front | AE_LOCK | exposure freezes, EV slider still moves |

Specific scenarios to check after relevant steps:

- **After Step 1:** preview FPS unchanged, no extra latency from
  partial emit machinery (even though only one partial is fired).
- **After Step 3:** AWB drift trajectory on dim front-cam scene
  matches pre-step — log `lastWb=(R,B)` per 32 frames and diff.
- **After Step 4:** AE gain trajectory on a step transition matches
  pre-step within ±1 LSB — log `gain=` per 32 frames and diff.
- **After Step 5:** AF tap-to-focus end-to-end + continuous
  retrigger on real composition change vs hand-held panning (the
  latter must NOT retrigger).
- **After Step 7:** manual exposure mode on Open Camera —
  ISO 400 / shutter 1/60 reaches sensor and preview is correctly
  bright/dim.
- **After Step 8:** apps that gate UI on AE state should react
  sooner; no broken UI elements.

## Open questions

- **Does any current Camera2 client on Mocha care about partial
  ordering?** If a downstream consumer does `result.get(KEY)` on
  partial 1 expecting AE state but it's not there yet, it gets
  `null` until partial 2. Most well-written clients merge partials
  before reading; but verify on the three test apps.
- **Is `PARTIAL_RESULT_COUNT = 3` the right number?** Could be 4 if
  shutter-notify is treated as its own partial (it's already a
  separate `notify(shutter)` callback so probably not). Stick with
  3 = AWB + AE + AF until a reason to change appears.
- **Should `AutoFocusController::onSettings(meta, frameNumber)`
  also become return-style?** It's called from
  `ApplySettingsStage` (RequestThread) for trigger handling, not
  from the IPA tick. If yes, scope creeps; if no, we have one
  remaining direct-write path for AF trigger. Recommendation:
  leave `onSettings` unchanged for this refactor pass — its scope
  is small and the trigger handling is correct as-is.

## Definition of done

- All nine steps shipped on master.
- `BasicIpa.cpp` / `BasicIpa.h` removed; `ExposureControl.cpp` /
  `ExposureControl.h` removed.
- `Ipa3A::processStats` body is < 50 LOC of dispatch + routing.
- `AutoExposureController`, `AutoWhiteBalanceController`,
  `AutoFocusController` each have public surface < 5 methods,
  zero direct backend dependencies.
- `PARTIAL_RESULT_COUNT = 3` (or 4) advertised; framework receives
  partials in expected order.
- All three test apps work on both cameras with no observable
  regression vs pre-refactor master.
- This document is deleted; permanent docs (`README.md`,
  `architecture.md`, `tier3_architecture.md`) updated to reflect
  steady state.
