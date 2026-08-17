# ViDAR priority ledger

Living ledger for orchestrator selection. GitHub issues are authoritative; this file is the in-repo snapshot.

**Updated:** 2026-08-17  
**Identity:** `TA-C-GHill`  
**Automatic merge:** false  
**Max active implementation PRs:** 1

## Priority model

Score ready issues by: safety, correctness, dependency-unblocking, architectural leverage, user value, learning value, risk reduction, implementation confidence, effort, hardware dependency, external dependency.

Default order: safety blockers → correctness blockers → CI/build → multi-issue blockers → architectural seams → tests for upcoming work → small user-facing slices → measured performance → docs → optional advanced → cosmetic.

## Current cycle

| Field | Value |
|-------|--------|
| Selected issue | [#13](https://github.com/The-Allsparks/ViDAR/issues/13) |
| Why highest priority | Correctness of range (geometry vs heuristic average) unblocks trustworthy observations |
| Why ready | Vitelli XYZ (#17) merged; acceptance criteria testable without hardware |
| Dependencies | None remaining |
| Expected deliverable | Geometry-authoritative `fuseRangeWeighted` + tests + docs |
| Expected validation | Python + `RangeFusionTest` + CI |
| Hardware required | No (Discover telemetry remaining is optional field check) |
| Subagent | Review of existing PR #18 (implementation already on branch) |
| Branch | `feature/geometry-authoritative-ranging` |
| Pull request | [#18](https://github.com/The-Allsparks/ViDAR/pull/18) |
| CI status | Required checks **pass** on `c39bf9a` (sourceCount docs follow-up included) |
| Merge status | **Blocked on human approval** (`AUTOMATIC_MERGE=false`) |
| Subagent | [Issue #13 review](bb6236dd-c1e1-4f4b-b6ea-752cedbb7baa) complete; split = keep one issue |

Do not start another implementation issue until #18 is merged, closed, or explicitly deferred.

## Ledger

| Issue | Priority | Readiness | Dependencies | Status | Subagent | Branch | PR | CI | Merge | Blocker | Next action |
|-------|----------|-----------|--------------|--------|----------|--------|----|----|-------|---------|-------------|
| #19 Roadmap epic | P0 process | Active | — | Open | orchestrator | `docs/initial-deep-audit` | not opened (single-PR rule) | — | — | PR #18 | Keep ledger in sync |
| #13 Geometry-authoritative ranging | P0 | Ready | #17 done | In PR | review | `feature/geometry-authoritative-ranging` | #18 | pass | wait human | Merge auth | Human approve/merge |
| #20 Worker failure visibility (S3) | P1 | Blocked | #18 | Filed | — | — | — | — | — | Open PR #18 | Implement after #18 |
| #21 Frame-generation association (A2/C4) | P1 | Blocked | #18 | Filed | — | — | — | — | — | Open PR #18 | Implement after #18 |
| #22 Sim range-fusion parity (C2) | P1 | Blocked | #13/#18 | Filed | — | — | — | — | — | Open PR #18 | Implement after #18 |
| #23 Runtime/world java-pure tests (T1) | P2 | Blocked | #21 preferred first | Filed | — | — | — | — | — | Open PR #18 | After #21 or same slice |
| #24 java-pure required check (T2) | P2 | Ready (settings) | None | Filed | — | — | — | — | — | Human/settings | After #18 to avoid mid-review churn |
| #25 Actions permissions + pins (Dep1) | P2 | Blocked | #18 | Filed | — | — | — | — | — | Open PR #18 | Implement after #18 |
| #16 Calibration visualization | P3 | Ready after #17 | #17 done | Open | — | — | — | — | — | Open PR #18 | After P1 slices |
| #14 Intrinsics quality metadata | P3 | Ready | None hard | Open | — | — | — | — | — | Open PR #18 | After #16 or with docs |
| #26 Hardware validation log | P3 | Blocked | Hardware | Filed | — | — | — | — | — | No Control Hub in this audit | Do not invent results |
| #27 Control Hub / desktop benchmarks | P3 | Partial | Hardware for hub | Filed | — | — | — | — | — | Hardware | Desktop after #18 |
| #15 Ceres extrinsic optimizer | P5 | Not ready | #13, #14, mount workflow | Open / later | — | — | — | — | — | Readiness gate | Keep deferred |
| #28 Repo hygiene templates | P4 | Blocked | #18 | Filed | — | — | — | — | — | Open PR #18 | After P1 |
| #29 Pin FTC SDK in java-compile | P4 | Blocked | #18 | Filed | — | — | — | — | — | Open PR #18 | After P1 |
| API `sourceCount` 0–2 vs 0–3 | P0 docs | Ready | #18 | In PR #18 | review | same as #13 | #18 | pass | wait | — | Fix on #18 before merge if still wrong |

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
