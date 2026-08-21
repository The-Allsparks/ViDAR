# ViDAR priority ledger

Living ledger for orchestrator selection. GitHub issues are authoritative; this file is the in-repo snapshot.

**Updated:** 2026-08-20 (post-#34)  
**Identity:** `TA-C-GHill`  
**Max active implementation PRs:** 1

## Priority model

Score ready issues by: safety, correctness, dependency-unblocking, architectural leverage, user value, learning value, risk reduction, implementation confidence, effort, hardware dependency, external dependency.

Default order: safety blockers → correctness blockers → CI/build → multi-issue blockers → architectural seams → tests for upcoming work → small user-facing slices → measured performance → docs → optional advanced → cosmetic.

**FTC readiness gate:** [#33](https://github.com/The-Allsparks/ViDAR/issues/33) is the first implementation/readiness priority (org-wide with [FORGE#4](https://github.com/The-Allsparks/FORGE/issues/4)). Do not claim FTC readiness from desktop tests alone. ViDAR remains passive (no drivetrain command).

## Current cycle

| Field | Value |
|-------|--------|
| Selected issue | [#29](https://github.com/The-Allsparks/ViDAR/issues/29) (first software child of [#33](https://github.com/The-Allsparks/ViDAR/issues/33)) |
| Why highest priority | Smallest non-hardware #33 acceptance item: stop compiling against unpinned SDK tip |
| Why ready | Tag `v11.2.1` exists; no Control Hub required |
| Dependencies | #34 ledger sync merged (`95cedfd`) |
| Expected deliverable | `java-compile` clones `--branch v11.2.1`; docs name the pin |
| Hardware required | No |
| Branch | `fix/pin-ftc-sdk-java-compile` |
| Last delivered | [#34](https://github.com/The-Allsparks/ViDAR/issues/34) via [#35](https://github.com/The-Allsparks/ViDAR/pull/35) |

## Ledger

| Issue | Priority | Readiness | Dependencies | Status | Branch / PR | Blocker | Next action |
|-------|----------|-----------|--------------|--------|-------------|---------|-------------|
| **#33 FTC packaging & lifecycle** | **P0 readiness** | **Active epic** | FORGE#4 (combined) | Open | — | Hardware for USB rows | Split children; start with #29 |
| #34 Record #33 as first priority | P0 docs | Done | #33 | **Merged** #35 `95cedfd` | — | — | Closed |
| #19 Roadmap epic | P0 process | Active | — | Open | — | — | Keep checklist in sync |
| #29 Pin FTC SDK in java-compile | P0 child of #33 | In progress | #33 | This PR | `fix/pin-ftc-sdk-java-compile` | — | Merge when CI green |
| #13 Geometry-authoritative ranging | Done | Done | #17 | **Merged** #18 | — | — | Closed |
| #20 Worker failure visibility | Done | Done | #18 | **Merged** #31 | — | — | Closed |
| #21 Frame-gated world association | Done | Done | #20 | **Merged** #32 `e6006b0` | — | — | Closed |
| #22 Sim range-fusion parity | P2 | Ready | #13 done | Open | — | Behind #33 gate for readiness claims | After first #33 children |
| #23 java-pure world/runtime tests | P2 | Partial | #21 done | Open | — | — | Close or extend after review |
| #24 java-pure required check | P2 | Ready (settings) | None | Open | — | Settings | After current PR |
| #25 Actions permissions + pins | P2 | Ready | None | Open | — | — | After #29 or with it |
| #26 Hardware validation log | P1 under #33 | Blocked | Hardware | Open | — | No Control Hub | Do not invent results |
| #27 Control Hub / desktop benches | P1 under #33 | Partial | Hardware for hub | Open | — | Hardware | Desktop OK later |
| #16 Calibration visualization | P3 | Ready | #17 done | Open | — | Behind #33 | After packaging gate |
| #14 Intrinsics quality metadata | P3 | Ready | None hard | Open | — | Behind #33 | After #16 |
| #28 Repo hygiene templates | P4 | Ready | None | Open | — | — | After P0 children |
| #15 Ceres extrinsic optimizer | P5 | Not ready | #14 | Open / later | — | Readiness gate | Keep deferred |

## Roadmap phases (adapted)

```
P0 FTC packaging & lifecycle (#33)  — versioned install, SDK pin, stop()/VisionPortal, Hub validation
Foundation                          — packages, runtime/attachment, JSON (done)
Safety and correctness              — #13, #20, #21 (done on main)
Architecture stabilization          — association clock done; more lifecycle tests open
Passive observability               — sim parity (#22), calibration viz (#16)
Testing and simulation              — required java-pure, benches
Basic integration                   — Pedro bridge (done); field validation (#26)
Advanced integration                — BEACON/HELM/ECHO/TRACE contracts under #33
Active behavior                     — out of scope for ViDAR
Performance optimization            — only with measurements (#27)
Release readiness                   — SECURITY, templates, first release
```
