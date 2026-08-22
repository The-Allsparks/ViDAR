# ViDAR FTC lifecycle and VisionPortal ownership

ViDAR separates **long-lived perception state** from **ephemeral FTC camera resources**. Teams must call `close()` on OpMode stop so VisionPortals and workers release.

## Ownership

| Resource | Owner | Lifetime |
|----------|--------|----------|
| `HardwareMap` / webcam names | Robot application (FTC config) | OpMode / RC process |
| `VisionPortal` instances | `VidarVision` inside `VidarVisionAttachment` | From `attachVision` until `detachVision` / `close` |
| Contour / tag processors, frame mailboxes | Attachment | Same as portals |
| `VidarGlobalVisionWorker` | Attachment (multi-cam) | Started on attach; `shutdownAndJoin` on close |
| `VidarRuntime` singleton | Process | Survives Auto → TeleOp |
| World model, tag decode worker, observation worker | Runtime | Until `VidarRuntime.shutdown()` |
| Published snapshots | Runtime | Continuously updated while attached; coasts when detached |

The robot app **supplies** hardware. ViDAR **must not** hide portal ownership: portals exist only while vision is attached.

## OpMode phases

| Phase | Team responsibility | ViDAR behavior |
|-------|---------------------|----------------|
| `init` | `VidarSpatial.create(...)` (attaches vision) | Builds portals; starts workers as configured |
| `init_loop` | Alliance poll / telemetry | Perception already advancing in background |
| `start` / `loop` | `spatial.update()` then read `snapshot()` | Do not construct portals in `loop` |
| `stop` | **`spatial.close()`** | `detachVision()` → attachment `close()` → each `VisionPortal.close()`, global worker join |
| RC exit (optional) | `VidarRuntime.shutdown()` | Detach + stop observation/tag workers; clear singleton |

`init` must not block forever on missing cameras: failed webcam slots are nulled and surface as diagnostics / warnings.

## `close()` vs `shutdown()`

| Call | Use when | Releases |
|------|----------|----------|
| `VidarSpatial.close()` → `detachVision()` | End of **every** OpMode | Portals, processors, mailboxes, global vision worker, fusion engine binding |
| `VidarRuntime.shutdown()` | Robot Controller exit / full teardown | Above + observation worker + tag decode worker + singleton |

After `close()`, the next OpMode’s `create()` reuses the runtime, rebinds odom/alliance suppliers, and attaches fresh portals.

## Repeated Auto → TeleOp

```
RC start → runtime getOrCreate
Auto INIT → create / attachVision
Auto STOP → close / detachVision
TeleOp INIT → create (applyBootstrap + attachVision)
TeleOp STOP → close / detachVision
RC exit → shutdown (optional)
```

World tracks persist across detach and **coast/TTL** using observation time (see world-model tests). Snapshots while detached are published from the frozen world without live cameras.

## Sample OpModes

Built-in OpModes that construct `VidarSpatial` call `close()` on the stop path. CI enforces that (`tests/architecture/test_hotpath_guards.py`). ROI calibration uses a lone `VidarVision` and calls `vision.close()`.

## Related

- [INSTALL.md](INSTALL.md)
- [SYSTEM_DESIGN.md](SYSTEM_DESIGN.md)
- [#33](https://github.com/The-Allsparks/ViDAR/issues/33)
