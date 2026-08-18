# ViDAR priority ledger

Living ledger for orchestrator selection. GitHub issues are authoritative; this file is the in-repo snapshot.

**Updated:** 2026-08-18  
**Identity:** `TA-C-GHill`  
**Automatic merge:** authorized this cycle by “proceed” after #18 gates passed  
**Max active implementation PRs:** 1

## Priority model

Score ready issues by: safety, correctness, dependency-unblocking, architectural leverage, user value, learning value, risk reduction, implementation confidence, effort, hardware dependency, external dependency.

Default order: safety blockers → correctness blockers → CI/build → multi-issue blockers → architectural seams → tests for upcoming work → small user-facing slices → measured performance → docs → optional advanced → cosmetic.

## Current cycle

| Field | Value |
|-------|--------|
| Selected issue | [#20](https://github.com/The-Allsparks/ViDAR/issues/20) |
| Why highest priority | Silent worker exceptions hide dead perception behind last-good snapshots |
| Why ready | #13/#18 merged; no hardware required; acceptance is JVM-testable |
| Dependencies | None |
| Expected deliverable | Worker records last error + count; `VidarDiagnostics` / Discover show it; worker stays alive |
| Expected validation | java-pure worker test + CI |
| Hardware required | No |
| Branch | `fix/observation-worker-failure-visibility` (planned) |
| Pull request | not opened yet |
| CI status | — |
| Merge status | — |
| Last delivered | [#13](https://github.com/The-Allsparks/ViDAR/issues/13) via [#18](https://github.com/The-Allsparks/ViDAR/pull/18) merge `65af3c2` |

#18 is merged. One implementation issue at a time: #20 next.

## Ledger

| Issue | Priority | Readiness | Dependencies | Status | Subagent | Branch | PR | CI | Merge | Blocker | Next action |
|-------|----------|-----------|--------------|--------|----------|--------|----|----|-------|---------|-------------|
| #19 Roadmap epic | P0 process | Active | — | Open | orchestrator | `docs/initial-deep-audit` | docs PR this cycle | — | — | — | Keep ledger in sync |
| #13 Geometry-authoritative ranging | P0 | Done | #17 done | **Merged** | review | `feature/geometry-authoritative-ranging` | #18 | pass | **merged** `65af3c2` | — | Closed |
| #20 Worker failure visibility (S3) | P1 | **Ready** | #18 done | Selected | pending | `fix/observation-worker-failure-visibility` | — | — | — | — | Implement |
| #21 Frame-generation association (A2/C4) | P1 | Ready | #18 done | Open | — | — | — | — | — | Wait for #20 PR | After #20 |
| #22 Sim range-fusion parity (C2) | P1 | Ready | #13/#18 done | Open | — | — | — | — | — | Wait for #20 PR | After #20/#21 |
| #23 Runtime/world java-pure tests (T1) | P2 | Blocked | #21 preferred first | Filed | — | — | — | — | — | #21 | After #21 or same slice |
| #24 java-pure required check (T2) | P2 | Ready (settings) | None | Filed | — | — | — | — | — | Settings | After current implementation PR |
| #25 Actions permissions + pins (Dep1) | P2 | Ready | #18 done | Open | — | — | — | — | — | Wait for #20 PR | After P1 |
| #16 Calibration visualization | P3 | Ready | #17 done | Open | — | — | — | — | — | After P1 | After P1 slices |
| #14 Intrinsics quality metadata | P3 | Ready | None hard | Open | — | — | — | — | — | After P1 | After #16 or with docs |
| #26 Hardware validation log | P3 | Blocked | Hardware | Filed | — | — | — | — | — | No Control Hub | Do not invent results |
| #27 Control Hub / desktop benchmarks | P3 | Partial | Hardware for hub | Filed | — | — | — | — | — | Hardware | Desktop after P1 |
| #15 Ceres extrinsic optimizer | P5 | Not ready | #13 done; needs #14 | Open / later | — | — | — | — | — | Readiness gate | Keep deferred |
| #28 Repo hygiene templates | P4 | Ready | #18 done | Open | — | — | — | — | — | After P1 | After P1 |
| #29 Pin FTC SDK in java-compile | P4 | Ready | #18 done | Open | — | — | — | — | — | After P1 | After P1 |
| API `sourceCount` 0–3 | P0 docs | Done | #18 | **Merged** | review | same as #13 | #18 | pass | merged | — | Closed with #13 |

## Roadmap phases (adapted)

```
Foundation                 — packages, runtime/attachment split, JSON config (done)
Safety and correctness     — #13, worker failures, frame-gated tracks (current)
Architecture stabilization — association clock, java-pure coverage of runtime
Passive observability      — sim parity, calibration visualization (#16)
Testing and simulation     — required java-pure, lifecycle tests, benches
Basic integration          — Pedro bridge (done); field validation
Advanced integration       — TRACE/HELM contracts (not in this repo)
Active behavior            — out of scope for ViDAR
Performance optimization   — only with measurements (P1)
Release readiness          — versioning, SECURITY, templates, first release
```
