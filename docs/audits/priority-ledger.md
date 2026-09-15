# ViDAR priority ledger

Living ledger for orchestrator selection. GitHub issues are authoritative; this file is the in-repo snapshot.

**Updated:** 2026-09-15 (#77 closed; #78 next)  
**Identity:** `TA-C-GHill`  
**Max active implementation PRs:** 1  
**Work order:** [plans/2026-09-14-issue-audit-and-implementation.md](../plans/2026-09-14-issue-audit-and-implementation.md)

## Priority model

Score ready issues by: safety, correctness, dependency-unblocking, architectural leverage, user value, learning value, risk reduction, implementation confidence, effort, hardware dependency, external dependency.

Default order: safety blockers → correctness blockers → CI/build → multi-issue blockers → architectural seams → tests for upcoming work → small user-facing slices → measured performance → docs → optional advanced → cosmetic.

**Do not claim match-ready vision** from desktop tests. ViDAR remains passive (no drivetrain command). Packaging epic [#33](https://github.com/The-Allsparks/ViDAR/issues/33) is **closed**; remaining FTC honesty is Hub rows in [validation-log.md](../validation-log.md).

## Current cycle

| Field | Value |
|-------|--------|
| Selected | **#78** fixture JSON loader (`fixtures[]` + `VidarFixtureSpec`) |
| Why highest priority | #76 accepted. #77 cited JSON is on `main` ([#114](https://github.com/The-Allsparks/ViDAR/pull/114), [#115](https://github.com/The-Allsparks/ViDAR/pull/115)). Schema next; no OpenCV yet. |
| Why ready | Empty `fixtures[]` = today's behavior. Official FLOWER poses already transcribed. |
| Expected deliverable | Optional `fixtures[]` + `VidarFixtureSpec`. BIOBUZZ `namedPoses` / `flowerGeometry` stay as the holding pen until wrap. |
| Branch | none yet |
| Last delivered | #77 via #114 / #115 |

## Ledger

| Issue | Priority | Readiness | Dependencies | Status | Next action |
|-------|----------|-----------|--------------|--------|-------------|
| **#76 Field Fixture architecture** | **P0 gate** | **Accepted** 2026-09-14 | — | Open (record) | Children unblocked; do not relitigate locks |
| **#97 Pose observation contract** | **P0 gate** | Design review | #96 | Open / ready | Pick **A (default-off)** or B |
| **#96 Localization epic** | P0 product | Blocked on #97 | #75/#76 | Open | Accept #107 shop locks with #96 |
| **#75 Fixture state epic** | P0 product | Blocked on #76 | — | Open | Implementation after #76 accept |
| #28 SECURITY / templates | P2 hygiene | **Ready** | None | Open | Small docs PR |
| #77 BIOBUZZ transcription | P0 data | **Done** | V1 + Field CAD STEP | **Closed** #114/#115 | Holding pen: `namedPoses` + `flowerGeometry` |
| #39 God-method split | P1 seam | **Ready** | None | Open | Land **before** fixture OpenCV |
| #78 Fixture JSON loader | P1 | **Ready** | #76 accept, #77 | Open | First implementation PR; do not invent poses |
| #79 + #99 Projector / frames | P1 | Blocked on #76 | #76 | Open | Prefer **one PR** |
| #81 FixtureStore | P1 | Blocked on #76 | #78 | Open | Do not overload world model |
| #80 Detector interface | P1 | Blocked on #76 | #81 | Open | Existing workers only |
| #82 Ordered stack | P1 | Blocked | #80 | Open | Physical labels only |
| #83 FLOWER state | P1 | Blocked | #77, #82 | Open | No owner field |
| #86 HIVE orientation | P1 | Blocked | #77, #80 | Open | Cheap cruise; coast on tag cooldown |
| #85 Multi-cam merge | P1 | Blocked | #81 | Open | Conflict -> UNKNOWN |
| #84 Public API | P1 | Blocked | #81 | Open | Additive snapshot |
| #87 State overlays | P2 | Blocked | #84 | Open | Default off; not a duplicate of #110 |
| #88 Frame corpus | P2 | Blocked | #76 | Open | Narrow images, not TRACE replay |
| #89 Fixture docs | P2 | Blocked | #84 | Open | After names stabilize |
| #98 Pose history | P1 under #96 | Blocked on #97 | #97 | Open | Before identical FLOWERs |
| #104 Association gate | P0 safety | Blocked | #98, #101 | Open | Wrong FLOWER must not snap pose |
| #102 FLOWER pose obs | P1 | Blocked | #97, #104 | Open | Flag default off |
| #91 HIVE kinematics pose | P1 | Blocked | #76 amend, #77, #86 | Open | Not SDK `robotPose` |
| #105 / #106 / #107 / #108 | P1 scheduling | Blocked | #97 | Open | Opportunistic tags after pose path |
| #110 Localization overlays | P2 | Blocked | #87 | Open | Reuse #87 plumbing |
| #111 Localization tests | P1 | Blocked | #104 | Open | CI must catch wrong-flower snap |
| #112 720p 4-cam bench | P1 Hub | Blocked | Hardware | Open | Slice of #26 |
| #26 Validation log | P1 Hub | Blocked | Hardware | Open | Do not invent results |
| #40 Tick lock / mailbox | P1 | Blocked | Hub Tick ms | Open | Percentiles already in closed #44 |
| #14 Intrinsics quality | P3 | Ready | — | Open | Optional calibration day |
| #16 Calibration viz | P3 | Ready | Vitelli done | Open | Mount axes, not PnP drawings |
| #47 Spotless | P4 | Ready | — | Open | Dedicated PR only |
| #15 Ceres | P5 | Deferred | #14/#16 | Open | Prerequisites 1-2 landed; still later |
| #113 Partial constraints | P5 | Deferred | #96 | Open | Do not block Waves A-D |
| #19 Process epic | P0 process | Active | — | Open | Keep checklist in sync |
| #37 Quality epic | P1 process | Active | — | Open | Remaining: #39, #40, #47 |

## Roadmap phases (adapted)

```
Packaging (#33)                     — done (Hub USB still open)
Gate 0 design reviews               — accept #76, #97, #107
Gate 1 hygiene / seams              — #28, #77 cited, #39
Wave A fixture foundation           — #78, #79+#99, #81, #80
Wave B BIOBUZZ fixture state        — #82-#89
Wave C pose observations            — #98, #101, #104, #103, #102, #91, #111
Wave D pose-aware tags              — #106, #105, #107, #108, #110, #109
Wave E hardware truth               — #112, #26, #40, #100
Later                               — #15 Ceres, #113, #47 Spotless
```
