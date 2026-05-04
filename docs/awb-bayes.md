# AWB Bayesian — migration plan

**Status:** open. Transient document — delete on completion (same
pattern as the 3A refactor plan).

**Why:** the current `AutoWhiteBalanceController` is gray-world over
the 16×16 patch grid with a 96-patch confidence gate and EMA-relax
to a `wbGainPrior` calibrated at one fixed CCT (FusionInitLight,
~5000-6000 K daylight on IMX179). On non-neutral scenes — laptop
display + keyboard, walls under mixed indoor light, anything where
the patch-grid average is biased away from a true scene-neutral —
gray-world systematically undershoots R-correction and the rendered
white drifts toward cyan. Confirmed in the 2026-05-04 diagnostic
trace: `lastWb = (1.679, 1.665)` versus the calibrated daylight
prior of `(1.993, 1.881)` — ~16 % R-deficit on a static
ноут+стена+клавиатура scene.

The structural fix is RPi-style **Bayesian AWB with lux-conditioned
priors**: `posterior(CT) ∝ prior(CT | lux) × likelihood(stats | CT)`
with a per-zone error clamp (`deltaLimit`), synthetic bias samples
toward a fallback CCT (`biasCT`), and an off-curve fine search
(`transversePos / transverseNeg`). Computationally cheap (~80-160 K
fp ops/frame, <0.2 % CPU on Cortex-A15); the cost is the calibration
session and the tuning-data extension.

## What this preserves

- The `AwbResult` interface — controller swap is contained, the
  coordinator routes the same `gains` / `ccm` / `estCct` / `state`
  payload it does today.
- The 3A async pipeline — Bayesian still ticks once per stats
  delivery on `StatsProcessStage`, lives in `Ipa3A::processStats`
  alongside AE / AF.
- `IpaStats::rgbMean[16][16][3]` as the per-zone input — same as
  gray-world. No new statistics producer; NEON encoder unchanged.
- The shader-side WB stage and the CCM-LERP CCT-driven path —
  Bayesian replaces the *estimation* of CCT, not how WB / CCM are
  applied downstream.
- Manual AWB modes (`AWB_MODE_OFF` + `COLOR_CORRECTION_GAINS`)
  remain via `applyManualGains`.

## What this requires

1. **Lux pipe from AE.** Gray-world is luminance-blind; Bayesian
   needs a current-frame lux estimate to look up the right prior.
   AE has the data (converged exposure × gain × scene luma); we
   just don't propagate it. One number through `IpaProcessParams`
   or a new `AeResult.luxIndex`.

2. **Tuning extension.** Two new pieces of per-sensor calibration:
   - **CT curves `ctR(t)`, `ctB(t)`** — at each calibrated CCT,
     the raw G/R and G/B ratios the sensor records on a true grey
     card (post-OB-subtract). Existing `colorCorrection.Set[].cct`
     and `wbGain` already encode this — they can be reused as PWL
     control points without a fresh session.
   - **Priors `prior(CT | lux)`** — for a few lux brackets (e.g.
     50 / 400 / 1500 / 5000), a PWL of `(CT, log-likelihood)`
     pairs that biases the search toward warmer CCTs at low lux
     and daylight at high lux. **This needs a fresh calibration
     session** — see step 9.

3. **New `BayesianAwbController`** behind the same `AwbResult`
   interface. Selected via `active.awb.algorithm = "bayes"` in
   the per-sensor tuning JSON; default `"grayworld"` keeps the
   current path on sensors without a bayes calibration.

## Migration steps

Each step is one bisectable commit, compiles + smoke-passes on its
own. Order chosen so the live algorithm flips only at step 10 —
everything before that is dark-launched scaffolding.

### Step 1 — Tuning schema for `awb.bayes` + algorithm switch

`SensorTuning` parses a new optional `active.awb.bayes` JSON block
with: `priors[]` (lux-conditioned PWLs), `ctCurveR` / `ctCurveB`
(falls back to `colorCorrection.Set[]` when absent), `transversePos`,
`transverseNeg`, `deltaLimit`, `biasCT`, `biasProportion`, `modes{}`.
Adds `awb.algorithm` field, defaults to `"grayworld"`. No consumer
change yet — gray-world keeps running.

`tools/isp_to_json.py` preserved-section list grows by one
(`awb.bayes`) so re-runs don't drop the fresh section.

### Step 2 — Lux index in AE result

`AutoExposureController` computes an absolute-ish lux index from
the converged `filteredTotalUs` against a per-sensor calibration
anchor (`active.ae.luxAnchor = {totalUs, gain, sceneLuma → lux}`,
one entry per sensor). Surfaces as `AeResult.luxIndex`. `Ipa3A`
caches the last value for AWB consumption. No behaviour change
yet — gray-world doesn't read it.

### Step 3 — `Awb` interface + factory

`AutoWhiteBalanceController` becomes `Awb` — a pure-virtual base
declaring `process(stats, luxIndex) → AwbResult` and
`applyManualGains(...) → AwbResult` plus the existing
`reset / currentWb* / currentGainsQ8 / currentEstCct` accessors.
Existing class renames to `GrayWorldAwbController`, implements `Awb`
unchanged (luxIndex argument ignored). `Ipa3A` creates the impl via
a factory keyed off `tuning.awb.algorithm`. No `BayesianAwbController`
yet; factory always returns gray-world. Smoke-pass: same output as
today.

### Step 4 — `BayesianAwbController` skeleton

Empty `BayesianAwbController` implementing `Awb` — `process` returns
the cold-start `wbGainPrior` every tick, no real estimation. Factory
gains a `"bayes"` branch. Algorithm switch in tuning still defaults
to `"grayworld"`; bayes path is only reachable with an explicit
override JSON. Smoke-pass: gray-world unchanged on production
tunings.

### Step 5 — coarseSearch over CT curves

Implement `coarseSearch(stats)`:
- Per-zone normalise `R/(G+1)`, `B/(G+1)` (post-saturation /
  noise-floor filter — same patch filter as gray-world).
- For each CT in the search range (log-step traversal of
  `ctCurveR` / `ctCurveB` domain), compute candidate
  `gainR = 1/ctR(t)`, `gainB = 1/ctB(t)`, accumulate
  `Σ_zones [(gainR·R−1)² + (gainB·B−1)²]`.
- Pick the CT minimising the sum. No prior, no clamp, no off-curve
  refinement yet — that's steps 6 / 7.

`BayesianAwbController` now produces a real CT estimate but no
likelihood smoothing. Bayes-path output is bounded but jittery on
real frames.

### Step 6 — Prior interpolation by lux + deltaLimit clamp

- `interpolatePrior(luxIndex)` builds a PWL by interpolating
  between the two `priors[]` entries bracketing the current lux.
- `coarseSearch` accumulates `Σ min(δ², deltaLimit) − prior(t)`
  instead of plain sum-of-squares. `deltaLimit` (default 0.2)
  caps the contribution of any single off-grey zone — protects
  against scene-color dominance (the laptop-screen case).

This is where the cyan-cast actually starts to fix on real data:
prior pulls the search toward the scene-appropriate CCT band even
when gray-world average is biased.

### Step 7 — Bias samples + fineSearch

- `bias_proportion × Σzones.counted` synthetic samples added at
  `(R, G, B) = (1/biasCT_R, 1, 1/biasCT_B)` so the search has a
  fallback target when the real sample count is low / unconvincing.
- `fineSearch(t)` searches ±N steps off the CT curve in the
  perpendicular direction, scaled by `transversePos / Neg`. Picks
  the off-curve `(gainR, gainB)` minimising the same likelihood +
  prior. Provides green ↔ magenta correction the on-curve search
  alone can't express.

### Step 8 — Startup-snap + IIR damping for output

- First N frames after `reset()` (`startupFrames`, default 10):
  speed = 1.0 (instant snap to the bayes output), so cold-start
  doesn't crawl from the prior the way gray-world does today.
- Steady state: speed = 0.05 (per-sensor tunable), IIR-smooth
  the published gains. Same output shape as gray-world's EMA but
  decoupled from the per-tick estimate so big bayes jumps still
  damp at the consumer.

`BayesianAwbController` is feature-complete at this point. Still
behind tuning switch, still off in production.

### Step 9 — Mode CT-ranges (manual AWB presets)

`process(stats, luxIndex, mode)` honours mode-specific CT ranges
from `tuning.awb.bayes.modes`:
- `AUTO` → full search range
- `INCANDESCENT` → e.g. 2500-3000 K
- `FLUORESCENT` → e.g. 4000-4500 K
- `DAYLIGHT` → e.g. 5500-6500 K
- `CLOUDY` / `SHADE` / `WARM_FLUORESCENT` → analogously

Same coarseSearch / fineSearch / prior pipeline, just a clipped
`[ctLo, ctHi]` window. Closes the open-question item from
`docs/open-questions.md` (manual AWB presets).

### Step 10 — Calibration session + tuning JSON updates

**User task, not a code commit.** Grey-card session for
IMX179 + OV5693:

- 4-5 illuminants spanning the working CCT range
  (incandescent ~2700 K, warm LED ~3000 K, cool fluorescent
  ~4500 K, daylight ~5500-6500 K, optionally shade ~7500 K).
- For each: full-frame grey card, mid-luminance (no clipping),
  capture 10-20 frames, average raw R / G / B post-OB. Records
  `(CT, R/G, B/G)` per illuminant.
- Multiple lux levels at the same illuminant where practical
  (e.g. daylight at midday vs evening) for prior PWL anchors.

Lands as a tuning JSON edit per sensor: `awb.bayes.ctCurveR /
ctCurveB / priors / modes` populated. `awb.algorithm` flipped to
`"bayes"`. `tools/isp_to_json.py` does not regenerate this
section.

### Step 11 — Production flip

Smoke test on both sensors with the new tuning. Validate cyan-cast
fixed on the original repro scene + no regression on simple
neutral / colour-dominant scenes. If solid: keep
`awb.algorithm = "bayes"` shipped. If a sensor isn't calibrated
(or fails QA), it stays on `"grayworld"` until calibration lands.

### Step 12 — Cleanup

If both production sensors run bayes long enough to validate:

- Drop the `GrayWorldAwbController` class + factory branch +
  schema's `algorithm` field. Bayes becomes the only path.
- Update docs: `architecture.md`, `tier3_architecture.md`,
  `roadmap.md`, top-level `README.md`. Remove the AWB-stability
  open-question entries that the new design closes.
- Update memory: `project_awb_design.md` rewrites to describe
  the bayes design + retired the gray-world rationale.
- Delete this `docs/awb-bayes.md` plan + remove its entry from
  `docs/README.md`.

If gray-world is still wanted as a fallback (e.g. for sensors
that ship without calibration data), keep it but document in
`architecture.md` as an opt-out path.

## Open subordinate questions

- **`luxIndex` definition.** RPi formula needs a reference frame
  with known lux (typically 400 lx D55 grey card at iso 100 + 1/100
  s). We don't have that anchor; nearest available is the EV target
  AE converges to on a known-illumination scene. Option: ship one
  hard-coded anchor per sensor in tuning, refine after first round
  of bayes data.
- **Whether to keep `clampU` / `UtoCCT` polynomial paths** alive
  for the gray-world fallback. They cost tuning maintenance; if
  bayes is shipped on both sensors, gray-world's own CCT estimator
  becomes vestigial.
- **CCM-LERP source of CCT.** Bayes outputs a CCT estimate that
  feeds the same `ccmForCctLerpQ10` path. The transition from
  gray-world's clampU-pinned 4709 K to bayes's free CCT may
  expose CCM-LERP behaviour we haven't seen (e.g. estCct = 6500 K
  selecting an extrapolated CCM beyond the calibrated brackets).
  Possible mitigation: extend `colorCorrection.Set[]` with one
  more high-CCT anchor during the calibration session.
