# NEON stats review

Review of the CPU NEON stats path (commits `3de7668` … `c600f4a`) and the
surrounding changes that deleted the GPU stats backend (`a774f4e`, `78f13d8`).

Scope:
- `hal/ipa/NeonStatsEncoder.{h,cpp}` — the encoder itself.
- `hal/ipa/StatsWorker.{h,cpp}` — the worker thread + progressive cadence.
- `hal/pipeline/stages/StatsDispatchStage.{h,cpp}`
- `hal/pipeline/stages/StatsProcessStage.{h,cpp}`
- `hal/Camera.cpp` — wiring, start/stop/reset.
- Residual surface in `isp/vulkan/VulkanIspPipeline.{h,cpp}` and the remaining
  shaders after the GPU stats backend was dropped.

## 1. Correctness

The encoder math is correct.

- CFA phase table `phaseChannel[4][4]` matches `IspParams::bayerPhaseFromFourcc`
  for all four Bayer phases; `chanEven` / `chanOdd` correctly index the two
  channels that appear in a single scan-line.
- `vld2q_u16` deinterleave: `val[0]` holds the even lanes, `val[1]` the odd
  ones. For `gIsEven` we pick lane 0; for `gIsOdd` lane 1. All of
  `tl/tc/tr/ll/rr/bl/bc/br` are loaded via `vld2q_u16(row + x ± {0, ±2})` —
  the 8 picked lanes line up exactly with the 8 G samples in the chunk in
  both phases.
- Sobel coefficients are the standard 3×3, applied on the G sub-lattice with
  a 2-pixel stride (2× spatial scale vs a true per-pixel Sobel). Signs and
  tap weights verified.
- Bounds guard:
  ```
  (y >= 2) && (y + 2 < height) && (x0 >= 2) && (simdEnd + 2 <= width)
  ```
  The widest read in the SIMD Sobel is `vld2q_u16(row + x + 2)` at the last
  chunk `x = simdEnd - 16`, which reaches index `simdEnd + 1`. The guard
  `simdEnd + 2 <= width` covers that. The y side is similarly tight.
- Patch partitioning (`bx[]`, `by[]` via proportional u64) handles sizes that
  don't divide `PATCH_{X,Y}` evenly (e.g. `1080 / 16 = 67.5` → alternating
  67/68-row stripes).
- Sobel coverage: every G in the patch row gets exactly one contribution,
  via one of (a) the NEON fast path, (b) the scalar fallback inside the
  `!neonSobel` branch of the SIMD loop, (c) the scalar tail loop for
  `[simdEnd, x1)`.
- Overflow headroom:
  - `lumaHist` u32: up to ~1 M G-pixels/frame at 1080p. ✓
  - `sumCh` u64 + `cntCh` u32 per patch: ~8k samples/patch, values ≤ 1023. ✓
  - `sharpSum` float: see §3.
- `histShift` = 3 on 10-bit, 1 on 8-bit → bin ∈ [0, 127]; the `> 127` clamp
  is defensive.
- Progressive contract: with `phaseCount = 2` and `PATCH_Y = 16`, phase 0
  covers `[0, 8)`, phase 1 covers `[8, 16)` — the documented "exactly once"
  property holds.

## 2. Issues

### 2.1 Real bug — session state leaks across `configureStreams`

`Camera::closeDevice` calls `mStatsWorker->reset()` (`hal/Camera.cpp:229`).
`Camera::configureStreams` does `stopWorkers()` → V4L2 reconfig →
`startWorkers()` with no reset in between.

If a reconfigure lands while the worker is mid-cycle (`phase != 0`), the
next `startWorkers()` run resumes on the old `currentJob.bayer` pointer.

- **Same resolution**: one frame of mixed stats (old partial + new slot
  content).
- **Different resolution**: `VulkanIspPipeline::ensureBuffers` →
  `VulkanInputRing::ensureSize` unmaps the old ring. The next
  `computeRange(currentJob.bayer, …)` dereferences freed memory.

Android camera HAL3 allows `configure_streams` to be called multiple times
on the same `open`, so this is reachable.

**Fix**: add `if (mStatsWorker) mStatsWorker->reset();` between
`errorCompletePendingRequests()` and the V4L2 stop at the top of
`configureStreams`. Same pattern as `closeDevice`.

### 2.2 Vestigial GPU-stats surface after `78f13d8`

Leftovers from the deleted GPU stats backend:

1. `isp/vulkan/VulkanIspPipeline.h:191-197` — comment on
   `TIMESTAMPS_PER_SLOT = 4` still labels query 2 as "after stats". The
   dispatch is gone; query 2 now bounds a no-op region between the scratch
   barrier and the blit.
2. `isp/vulkan/VulkanIspPipeline.cpp:236-241` — rollback-hint comment
   ("Encoder allocation stays in place until the full removal commit…") is
   stale. The full removal landed.
3. `isp/vulkan/VulkanIspPipeline.cpp:1195-1199` —
   `PERF-GPU: … stats=%lluus …` now prints a near-zero barrier cost. Either
   drop the column and shrink `TIMESTAMPS_PER_SLOT` to 3, or rename to
   `barrier=`.
4. `hal/ipa/IpaStats.h:8-14` — header comment describes the struct as
   "written by the GPU stats compute pass" and points at
   `isp/vulkan/io/VulkanStatsEncoder`, a file that no longer exists. Update
   to reference `NeonStatsEncoder`.

## 3. Performance

### 3.1 Current cost

Measurement from commit `be6c5cb`: "~8 ms of a 22–33 ms budget" per full
cycle. With `phaseCount = 2` that is ~4 ms / phase, which matches a
first-principles estimate:

| Per phase at 1920×1080 10-bit          |                               |
|----------------------------------------|-------------------------------|
| Bayer rows covered                      | ~540 (= 8 patch rows × ~67)   |
| Bytes read from slot                    | ~2 MB                         |
| Cortex-A15 NEON memory-linear bandwidth | ~1–2 GB/s                     |
| → memory-bound floor                    | ~1–2 ms                       |
| NEON arithmetic per 16-px chunk         | ~25 ops (wide, Sobel included)|
| Chunks / phase                          | ~3700                         |
| → compute-bound floor                   | ~60–100 µs                    |

So the phase is **memory-bound**. The reported ~4 ms includes Sobel-load
redundancy and `memset` / `hsum` overheads on top of the 2 MB read.

Cache: active tile per row is ~20 KB (1 row × 2 bytes/px × 1920 + 4 Sobel
rows × slack). Cortex-A15 L1D is 32 KB, L2 is 2 MB — tile fits in L1, full
slot fits in L2. No thrashing expected.

Memory bandwidth use averaged over frame time: 2 MB × 2 phases × 60 fps =
240 MB/s. Well below the ~6 GB/s DRAM ceiling on Tegra K1.

### 3.2 Where the phase time goes (estimated)

| Component                                      | Share   |
|------------------------------------------------|---------|
| Sobel neighbor loads (3 × 32 B per chunk × 3 rows) | ~45 %   |
| rgbMean + histogram `vld2q` on center row      | ~15 %   |
| Sobel arithmetic (widen, sub, shl, cvt, mla)   | ~20 %   |
| Histogram scalar extract via `vst1q_u16` stack  | ~5 %    |
| Per-row `hsum_u32x4` + partial u64 adds         | ~5 %    |
| Scalar tail / edge fallback                     | ~5 %    |
| Cache invalidate (`VkMappedMemoryRange`)        | ~5 %    |

### 3.3 Optimisation opportunities, ranked

**Bigger wins (combined ~30–40 % phase time)**:

1. **Slide the 3-row Sobel window across chunks.** Today every chunk re-issues
   `vld2q` for `rowN / rowY / rowS` at `x - 2`, `x`, `x + 2`. Between
   consecutive chunks, the loads at `x` and `x + 2` of chunk N become the
   loads at `x - 2` and `x` of chunk N+1 — they are being redone. Keeping
   nine `uint16x8_t` registers (or a ring of three per row) across the inner
   loop cuts total `vld2q` count from 9 to ~3 per chunk. Memory-bound path,
   biggest lever.
2. **Skip Sobel when the AF path doesn't need it.** Sobel is ~65 % of the
   phase. AF only consumes sharpness while hunting. A simple `enableSobel`
   flag driven by the AF controller collapses the non-hunting frames to
   rgbMean + histogram.

**Medium wins (~5–10 %)**:

3. **Integer sharpness accumulator.** Swap `float sharpSum` for `uint64_t`
   (signed square fits in u32; sum across patch fits in u64 with huge
   headroom). Drops the four `vcvtq_f32_s32` and four `vmlaq_f32` per chunk
   in favour of `vmull_s16 / vmlal_s16`. Removes the `float` precision
   concern too (see §3.5). Requires `IpaStats::sharpness` to move to u64 or
   be normalised in `finalize`.
4. **Hoist rgbMean accumulator across rows.** `accEven`/`accOdd` reset per
   (patch, row) and horizontal-summed every row. Hoisting to per-patch
   removes ~67 horizontal reductions. Minor.
5. **Static dispatch on `gIsEven`.** Two specialised inner loops instead of
   a per-chunk `? val[0] : val[1]` select. Predictable branch is already
   cheap, so this is worth only a few percent.

**Structural (needs rework)**:

6. **Thread the phase across 2 cores.** Tegra K1 has 4 × Cortex-A15. Split
   the patch-row range of the current phase in half and dispatch to a second
   worker. Memory-bound so speedup ≤ 2× minus sync. Halves the per-phase
   tail. Complicates the worker state machine; probably not worth it until
   stats grow (per-channel histograms, variance, finer grid).
7. **Drop `phaseCount`, vectorise harder.** If §1 + §2 + §3 land, the full
   cycle may fit inside one frame period, which removes the progressive
   plumbing. Revisit after measurement.

### 3.4 Non-opportunities / confirmed fine

- `invalidateBayer` every frame: nvmap cache invalidate of a 4 MB mapping is
  tens of µs at most, and it overlaps with the Vulkan submit on the other
  thread. Not worth special-casing.
- Unaligned `vld2q` on ARMv7: the Bayer slot is page-aligned; row offsets
  like `row + x - 2` are 2-byte aligned but A15 handles unaligned NEON
  without a penalty for reads of this size.
- 8-bit scalar fallback: sensor is 10-bit in normal operation, 8-bit path is
  a test fixture. Not worth vectorising.

### 3.5 Precision note (informational)

With 10-bit data, a single pixel's `Gx² + Gy²` is ≤ ~32 M. Per-patch sum
(~4 k G-pixels / patch) reaches ~1.3e11. `float` has ~7 decimal digits of
precision, so accumulators above 16 M lose unit precision — at 1.3e11 we
are at ~5 decimal digits of precision (order 10⁴ ULPs). Fine for AF
peak-detect (relative comparison); if a future consumer does ratios or
subtractions between patches, it will see the quantisation. Another reason
to prefer the integer accumulator from §3.3 (3).

## 4. `StatsWorker` thread model

Works as documented. Notes worth keeping in mind:

- Bayer-slot lifetime: at `V4L2DEVICE_BUF_COUNT = 8`, `PIPELINE_MAX_IN_FLIGHT = 1`,
  `PIPELINE_QUEUE_CAPACITY = 1`, worst-case 3 slots held → ~5 free → ~80–
  100 ms before a released slot is re-dequeued by the sensor. With
  `phaseCount = 2` the cycle spans ~33 ms at 60 fps, well inside the
  rotation window. Budget shrinks linearly in `phaseCount`; recompute
  before raising it.
- `submit` is latest-wins at the **start** of a cycle. A submit that lands
  while the worker is mid-cycle (phase > 0) is consumed only as the wake-up
  event for the next phase — its `bayer` pointer is not picked up until the
  current cycle finishes and the next `phase == 0` branch runs. Intentional;
  documented.
- `peek` under `outLock`, `submit` under `inLock`, no interaction. ✓
- `reset()` mutates cycle state (`phase`, `partial`) without locking, on the
  assumption that the worker thread is quiesced. `closeDevice` satisfies
  that via `stopWorkers()`. `configureStreams` does not call `reset` at all
  — see §2.1.
