# ViDAR priority ledger

Living ledger for orchestrator selection. GitHub issues are authoritative; this file is the in-repo snapshot.

**Updated:** 2026-08-21 (after #63; #42 in progress)  
**Identity:** `TA-C-GHill`  
**Max active implementation PRs:** 1

## Priority model

Score ready issues by: safety, correctness, dependency-unblocking, architectural leverage, user value, learning value, risk reduction, implementation confidence, effort, hardware dependency, external dependency.

Default order: safety blockers → correctness blockers → CI/build → multi-issue blockers → architectural seams → tests for upcoming work → small user-facing slices → measured performance → docs → optional advanced → cosmetic.

**FTC readiness gate:** [#33](https://github.com/The-Allsparks/ViDAR/issues/33) is the first implementation/readiness priority (org-wide with [FORGE#4](https://github.com/The-Allsparks/FORGE/issues/4)). Do not claim FTC readiness from desktop tests alone. ViDAR remains passive (no drivetrain command).

## Current cycle

| Field | Value |
|-------|--------|
| Selected issue | [#42](https://github.com/The-Allsparks/ViDAR/issues/42) JSON VidarSettings tuning |
| Why highest priority | Single edit path; stop reading deprecated merge-radius constant |
| Why ready | #41/#62 on main |
| Dependencies | None hard |
| Expected deliverable | `world` + fusion ranked keys in season JSON; VidarSettings from season; docs |
| Hardware required | No |
| Branch | `refactor/json-vidarsettings-tuning-surface` |
| Last delivered | [#41](https://github.com/The-Allsparks/ViDAR/issues/41) via [#63](https://github.com/The-Allsparks/ViDAR/pull/63) |

## Ledger

| Issue | Priority | Readiness | Dependencies | Status | Branch / PR | Blocker | Next action |
|-------|----------|-----------|--------------|--------|-------------|---------|-------------|
| **#33 FTC packaging & lifecycle** | **P0 readiness** | **Active epic** | FORGE#4 | Open | — | Hardware for USB rows | Hub/#26/#27 next software |
| #38 Package cycles | P1 | **Done** | #37 | **Closed** #60 | — | — | frame→detect removed |
| #61 Install/lifecycle docs | P0 under #33 | **Done** | #33 | **Merged** | — | — | VERSION + close() guard |
| #43 java-pure FusionEngine/Spatial | P2 | **Done** | #23 | **Closed** #62 | — | — | isUsableCamera seam |
| #23 java-pure world/runtime tests | P2 | **Done** | #21 | **Closed** | — | — | TTL + #43 |
| #37 Quality/CI epic | P1 | Active | This audit | Open | — | — | Children remain |
| #25 Actions permissions + pins | P2 | **Done** | None | **Closed** #48 | — | — | SHA pins |
| #44 Metrics percentiles | P1 | **Done** | #37 | **Closed** #59 | — | — | Discover Tick ms |
| #40 Tick lock / mailbox / snapshots | P1 | Ready | #44, #27 | Open | — | Measure first | After #27 Hub/desktop |
| #41 TagGate static state | P2 | **Done** | — | **Closed** #63 | — | — | Runtime-owned gate |
| #42 JSON single tuning surface | P2 | **Active** | — | Open | `refactor/json-vidarsettings-tuning-surface` | — | Land PR |
| #22 Sim range-fusion parity | P2 | Ready | #13 done | Open | — | Behind #33 for readiness claims | After #42 |
| #26 Hardware validation log | P1 under #33 | Blocked | Hardware | Open | — | No Control Hub | Do not invent results |
| #27 Control Hub / desktop benches | P1 under #33 | Partial | Hardware for hub; #44 | Open | — | Hardware | Desktop OK; Hub blocked |
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
