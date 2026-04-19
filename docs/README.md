# Developer Documentation

This directory contains design notes, architectural analysis, and a roadmap for
the Tegra T124 Android Camera HAL3 implementation in this repository.

| Document | What's inside |
|----------|--------------|
| [architecture.md](architecture.md) | How the current HAL is structured — components, request flow, threading, ISP abstraction |
| [camera3-compliance.md](camera3-compliance.md) | Gaps between the current implementation and the Camera3 HAL contract, prioritized |
| [latency-and-buffers.md](latency-and-buffers.md) | V4L2 buffer-queue latency analysis, frame-skip strategies, delayed-controls problem |
| [open-source-references.md](open-source-references.md) | What libcamera, RkISP1, RPi, cros-camera and AOSP V4L2 HAL do differently — patterns worth stealing |
| [roadmap.md](roadmap.md) | Prioritized list of improvements with effort estimates |
| [isp-pipeline.md](isp-pipeline.md) | Current IspPipeline abstraction and the two live backends (Vulkan soft-ISP / Hw) |
| [bugs.md](bugs.md) | Known bugs found during testing but deferred; each with location, likely cause, and planned fix tier |

## Scope

These docs describe the HAL as implemented on branch `master`. They are
developer-facing — targeted at someone picking up the codebase to extend 3A,
reduce latency, improve Camera3 compliance, or port to a different SoC.

They deliberately do **not** cover:

- How to build the HAL (see top-level `README`)
- How to flash or run on device
- Sensor-specific tuning values (those live in `sensors/` / `SensorConfig`)

## Reading order

If you're new to the codebase: start with [architecture.md](architecture.md),
then [camera3-compliance.md](camera3-compliance.md). The roadmap references
both.

If you're investigating preview latency: jump straight to
[latency-and-buffers.md](latency-and-buffers.md).

If you're planning a rewrite or major refactor: read
[open-source-references.md](open-source-references.md) first — most of what
you'd design from scratch has been solved better elsewhere.
