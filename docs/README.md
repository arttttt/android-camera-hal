# Developer documentation

Design notes, architectural analysis, and a roadmap for the Tegra K1
/ Mi Pad 1 Camera HAL3 implementation in this repository.

| Document | What's inside |
|----------|---------------|
| [architecture.md](architecture.md) | Component overview, six-thread topology, request lifecycle, stream config, 3A summary, per-module tuning |
| [tier3_architecture.md](tier3_architecture.md) | The async-pipeline design spec: data flow, queue shapes, stage contracts, fence-fd plumbing |
| [isp-pipeline.md](isp-pipeline.md) | Vulkan ISP detail: produce-once API, slot ring, fragment-ROP write path, Tegra K1 driver quirks |
| [camera3-compliance.md](camera3-compliance.md) | What's NOT claimed (RAW, MANUAL_*, REPROCESSING, DEPTH_OUTPUT, CONSTRAINED_HIGH_SPEED_VIDEO) and what each would mandate |
| [latency-and-buffers.md](latency-and-buffers.md) | What was solved (drain-to-latest, async capture, fence-fd poll, DelayedControls, produce-once + worker split) and the production PERF model |
| [neon-stats-review.md](neon-stats-review.md) | Review of the NEON statistics kernel — correctness + register pressure analysis |
| [open-questions.md](open-questions.md) | Open architecture questions worth investigating |
| [open-source-references.md](open-source-references.md) | Patterns from libcamera / RkISP1 / RPi worth borrowing |
| [libcamera-steal-list.md](libcamera-steal-list.md) | Ranked, scoped TODO list of concrete patterns to lift from libcamera, with file paths on both sides |
| [awb-bayes.md](awb-bayes.md) | Migration plan: gray-world AWB → RPi-style Bayesian AWB with lux-conditioned priors. Transient; deleted on completion. |
| [vic-encoder-path.md](vic-encoder-path.md) | The dormant HW VIC encoder-stream writer: tile-scramble evidence matrix, the UART-printk 5 fps post-mortem, and the ABI-first re-enable roadmap |
| [roadmap.md](roadmap.md) | Done items + open work |
| [bugs.md](bugs.md) | Known bugs (deferred or won't-fix), each with location and likely cause |

## Scope

These docs describe the HAL on `master`. They are developer-facing —
targeted at someone picking up the codebase to extend 3A, work on
the ISP, improve Camera2 compliance, or port to a different SoC.

They deliberately do **not** cover:

- How to build the HAL — see top-level [`README.md`](../README.md).
- Sensor-specific tuning values — those live in
  `tuning/<sensor>_<integrator>.json` and per-module overrides.

## Reading order

If you're new to the codebase: start with
[architecture.md](architecture.md) for the component map and
request lifecycle, then dive into
[tier3_architecture.md](tier3_architecture.md) for the threading /
queue / fence design.

If you're investigating the ISP itself: jump to
[isp-pipeline.md](isp-pipeline.md). It assumes you've read
architecture.md first.

If you're investigating preview latency: read
[latency-and-buffers.md](latency-and-buffers.md). It cross-
references the PERF log fields the production HAL emits.

If you're scoping a new feature: skim
[roadmap.md](roadmap.md) and [bugs.md](bugs.md) for what's
already shipped, what's deferred, and what's known-broken.

If you're considering a major refactor: read
[open-source-references.md](open-source-references.md) first —
most of what you'd design from scratch has been solved better
elsewhere.
