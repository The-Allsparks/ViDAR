# ViDAR issue audit and implementation plan

**Date:** 2026-09-14  
**Repo:** [The-Allsparks/ViDAR](https://github.com/The-Allsparks/ViDAR) at `main` (`9cc69a4`)  
**Open issues audited:** 44  
**Closed since last ledger:** packaging epic [#33](https://github.com/The-Allsparks/ViDAR/issues/33), session/version [#69](https://github.com/The-Allsparks/ViDAR/issues/69), SDK 12 crop decode [#90](https://github.com/The-Allsparks/ViDAR/issues/90), 720p MJPEG [#92](https://github.com/The-Allsparks/ViDAR/issues/92)

This file is the work-order after the 2026-09-14 GitHub audit. GitHub issues remain authoritative for acceptance criteria.

**Status 2026-09-15:** Gate 0 [#76](https://github.com/The-Allsparks/ViDAR/issues/76) **accepted** (including HIVE kinematics and adaptive-auto amendments). [#77](https://github.com/The-Allsparks/ViDAR/issues/77) **closed** ([#114](https://github.com/The-Allsparks/ViDAR/pull/114), [#115](https://github.com/The-Allsparks/ViDAR/pull/115)): cited HIVE tags, FLOWER CAD poses, and `flowerGeometry`. Next implementation is Wave A **#78** (`fixtures[]` + `VidarFixtureSpec`). [#97](https://github.com/The-Allsparks/ViDAR/issues/97) A vs B still gates pose fusion. Do not start OpenCV fixture detectors in the loader PR.

---

## Audit verdict

As of the 2026-09-14 snapshot (`9cc69a4`): none of the 44 open issues was implemented, there was no `vidar.fixture` package, `apriltags.tags` was `[]`, there was no `fixtures[]`, and `TagDecodeBudget` was a 1-second metronome (`DECODE_INTERVAL_MS = 1000`). **Superseded 2026-09-15:** [#77](https://github.com/The-Allsparks/ViDAR/issues/77) filled HIVE IDs 30-45 and FLOWER geometry ([#114](https://github.com/The-Allsparks/ViDAR/pull/114), [#115](https://github.com/The-Allsparks/ViDAR/pull/115)). Still no `vidar.fixture` and still no `fixtures[]` on season JSON. The backlog remains two product epics plus leftover quality/release work from August.

The problem is not missing tickets. It is **stale metadata**, **overlapping children**, and **design reviews that have not been accepted**. Until those gates land, 34 of 44 issues correctly stay blocked.

### What shipped (do not re-open)

| Closed | What it was |
|--------|-------------|
| #13 / #18 | Ground-plane range authoritative |
| #20, #21 | Worker failures + frame-gated world TTL |
| #22 | Sim range-fusion parity |
| #23, #24, #43 | java-pure lifecycle / FusionEngine seams; java-pure required |
| #25 | Actions pins + permissions |
| #27 | Desktop bench + Hub Tick-ms **procedure** (Hub rows still empty) |
| #29, #90 | FTC SDK pin; current pin **v12.0** with cluster crop decode |
| #33, #69 | Packaging, `VidarVersion.SEMVER`, attach/detach JVM tests, INSTALL / LIFECYCLE docs |
| #38, #41, #42, #44, #45, #46 | Cycles, TagGate, JSON surface, tick percentiles, default JSON, hide `runtime()` |
| #92 | 1280x720 MJPEG + crop-only AprilTag decode |

[#44](https://github.com/The-Allsparks/ViDAR/issues/44) (metrics percentiles) is **closed**. Open [#40](https://github.com/The-Allsparks/ViDAR/issues/40) is the remaining lock / mailbox / 1 kHz tick work, still waiting on Hub numbers.

### Code facts that still match the open issues

- `VidarObservationWorker` still `sleepQuiet(1)` and fusion still runs under the runtime lock.
- `VidarContourProcessor` is still ~1038 lines; `processElementPass` is still the element/plate god method. Split this **before** fixture OpenCV lands (#39).
- `docs/validation-log.md` Hub table is still empty. Desktop 2026-08-22 bench is recorded. Do not invent Hub FPS.
- **Superseded 2026-09-15:** `config/seasons/2026-biobuzz.json` `apriltags.tags` is HIVE IDs **30-45** (`localization: false`), not `[]`. Snapshot on `9cc69a4` was empty; #77 transcribed from Competition Manual V1 Figure 9-17 ([#114](https://github.com/The-Allsparks/ViDAR/pull/114) / [#115](https://github.com/The-Allsparks/ViDAR/pull/115)). Still no `fixtures[]`.
- Dependabot **does** exist (`.github/dependabot.yml`). #28's "no dependabot" evidence was stale. SECURITY.md, CODE_OF_CONDUCT, and issue/PR templates are still missing.

### Issues not closed (and why)

No open issue was completed in code. Overlaps were **split in comments**, not merged, so acceptance criteria stay on the owning ticket:

| Pair | Split |
|------|--------|
| #87 vs #110 | #87 = fixture **state** overlays (stack, freshness, ROI). #110 = localization overlays (reprojection, association, scheduler mode). #110 reuses #87 plumbing. |
| #79 vs #99 | #99 = timestamped camera-to-world **contract**. #79 = `VidarFieldProjector` implementation. Same PR unless #99 stays docs-only. |
| #26 vs #112 | #26 = whole Phase 6 validation log. #112 = the 720p four-camera performance row that localization needs. |
| #91 vs #102 | #91 = unique HIVE cluster kinematics. #102 = visually identical FLOWERs (needs #104). |

[#15](https://github.com/The-Allsparks/ViDAR/issues/15) (Ceres) and [#113](https://github.com/The-Allsparks/ViDAR/issues/113) (partial constraints) stay deferred. Vitelli XYZ and geometry-authoritative ranging **did** land; Ceres is still "not near-term," not "blocked on missing PRs."

---

## Locked product rules (do not relitigate in implementation PRs)

These are already in GitHub comments on #75 / #76 / #96. Implementation PRs that violate them should be rejected.

1. ViDAR is **passive**. No motors. Canonical fused pose is Pedro / TeamCode, not ViDAR.
2. ViDAR publishes **physical** fixture facts (stack labels, opening, HIVE orientation, timestamps, freshness). TeamCode owns BIOBUZZ meaning (top/bottom nectar, apparent owner). TeamCode auto / teleop / HELM own strategy ([HELM#46](https://github.com/The-Allsparks/HELM/issues/46); [#76 adaptive-auto amendment](https://github.com/The-Allsparks/ViDAR/issues/76)).
3. Do not add `vidar.season.biobuzz` or `if (seasonId == "2026-biobuzz")` in detect/runtime.
4. Copy-out interpreter samples are allowed. Runtime must not auto-load them.
5. Cameras stay **level** and look **across** at the low HIVE CELL. No look-up cheap silhouette detector.
6. Do not use SDK `robotPose` / static tag metadata. BIOBUZZ tags move. Scout observations still never rewrite pose.
7. HIVE **state** must not wait on expensive AprilTag decode. During tag cooldown, HIVE freshness **coasts** (STALE / UNKNOWN).
8. AprilTag decode is **opportunistic** (design in #107): configurable ~5 s cooldown after a trusted observation, decaying quality until ~30 s, recovery only when pose is actually sick (#108). 5/30 are starting defaults, not architecture.
9. Four identical FLOWERs require **ambiguity gating** (#104) before they may emit pose observations.
10. Capture-time pose (`VidarOdomHistory.at(captureTimeNanos)`), never the newest odom, for projection.
11. Adaptive auto is a **snapshot consumer**, not a ViDAR mode. UNKNOWN at PLAY runs the conventional Pedro path. No wait/until API. TRANSITIONING/UNKNOWN must not change a committed plan. No Pedro/HELM names on `VidarFixtureStatus`. Auto vs teleop tightness uses `age`, not a second freshness enum.

---

## Gate 0 — accept design reviews (no product code)

Do not open implementation PRs for #75 or #96 children until these comments exist.

| Issue | Decision needed | Default recommendation |
|-------|-----------------|------------------------|
| [#76](https://github.com/The-Allsparks/ViDAR/issues/76) | Accept Field Fixture architecture, including the 2026-09-13 HIVE kinematics amendment, STATIC_FIELD pose observations under #96, and the 2026-09-14 adaptive auto consumer | Accept as written in the issue + later comments |
| [#97](https://github.com/The-Allsparks/ViDAR/issues/97) | Pose observation consumer: **A** (gated `VidarLocalizationFusion`, default-off) vs **B** (snapshot-only) | **A, default-off** (reuse Pedro bridge; fixture observations cannot change pose until a flag is on) |
| [#96](https://github.com/The-Allsparks/ViDAR/issues/96) / [#107](https://github.com/The-Allsparks/ViDAR/issues/107) | Accept opportunistic tag scheduler shop locks (2026-09-14) | Accept; retune 5/30 from TRACE after Hub data |

Until then, the only coding that is in-bounds is Gate 1 hygiene and the #39 split (those do not implement fixtures).

---

## Proposed implementation order

One implementation PR at a time (ledger rule). Prefer java-pure tests over Hub stories. Do not mix Spotless, Dependabot, or format blast with behavior.

### Gate 1 — hygiene and seams (parallel with Gate 0 accepts)

| Order | Issue | Why now | Hardware |
|-------|-------|---------|----------|
| 1a | [#28](https://github.com/The-Allsparks/ViDAR/issues/28) | SECURITY.md, CoC, issue/PR templates. Unblocked: PR #18 merged, dependabot already exists. | No |
| 1b | [#77](https://github.com/The-Allsparks/ViDAR/issues/77) | Transcribe V1 FLOWER/HIVE poses and cluster IDs with figure cites. Shop notes already list 30-45 / 3.25 in / 36h11; still peer-check Figure 9-17 / Onshape. **Do not invent missing IDs.** | No (docs + JSON) |
| 1c | [#39](https://github.com/The-Allsparks/ViDAR/issues/39) | Split `processElementPass` and fusion tick **before** fixture detectors grow those methods. Behavior-neutral. | No |
| 1d | [#19](https://github.com/The-Allsparks/ViDAR/issues/19) / ledger | Keep the process epic and this file in sync after each merge. | No |

Optional in this gate, not on the fixture critical path: [#14](https://github.com/The-Allsparks/ViDAR/issues/14) (intrinsics quality metadata) and [#16](https://github.com/The-Allsparks/ViDAR/issues/16) (calibration viz). Both are `status:ready`. Vitelli XYZ already merged. Do them if a shop day is calibration-shaped; do not block FLOWERs on them.

### Wave A — fixture foundation (after #76 accept)

Teachable order. #77 can land during Gate 1; Waves A–B stay blocked on empty or uncited field JSON.

| Order | Issue | Deliverable |
|-------|-------|-------------|
| A1 | [#78](https://github.com/The-Allsparks/ViDAR/issues/78) | Optional `fixtures[]` + `VidarFixtureSpec` loader. Empty list = today's behavior. Config must not import detect/fusion/world/tag/schedule. |
| A2 | [#99](https://github.com/The-Allsparks/ViDAR/issues/99) + [#79](https://github.com/The-Allsparks/ViDAR/issues/79) | Timestamped camera-to-world contract, then `VidarFieldProjector` using capture-time pose. Prefer **one PR**. |
| A3 | [#81](https://github.com/The-Allsparks/ViDAR/issues/81) | `vidar.fixture` store: identity-keyed, freshness CURRENT/STALE/UNKNOWN/TRANSITIONING, age from capture time, survives `detachVision()`. Do not overload `VidarWorldModel`. |
| A4 | [#80](https://github.com/The-Allsparks/ViDAR/issues/80) | `VidarFixtureStateDetector` + budgeted ROI on **existing** workers. No second thread pool. |

### Wave B — BIOBUZZ fixture state (still not pose)

| Order | Issue | Deliverable |
|-------|-------|-------------|
| B1 | [#82](https://github.com/The-Allsparks/ViDAR/issues/82) | Generic ordered stack detector. Physical palette keys only. No owner field. |
| B2 | [#83](https://github.com/The-Allsparks/ViDAR/issues/83) | Four STATIC_FIELD FLOWERs from transcribed JSON. Blocked on #77 + #82. |
| B3 | [#86](https://github.com/The-Allsparks/ViDAR/issues/86) | HIVE orientation. Cheap cruise path; expensive cluster IDs only when the scheduler allows. Coasts during tag cooldown. Blocked on #77 + #80. |
| B4 | [#85](https://github.com/The-Allsparks/ViDAR/issues/85) | Multi-camera identity merge. Disagreement -> UNKNOWN / TRANSITIONING, never a silent pick. |
| B5 | [#84](https://github.com/The-Allsparks/ViDAR/issues/84) | Additive `VidarSpatial` / snapshot fixture list. No `apparentOwner`. |
| B6 | [#87](https://github.com/The-Allsparks/ViDAR/issues/87) | Discover telemetry + optional overlays (default off). No TRACE Maven dependency. |
| B7 | [#88](https://github.com/The-Allsparks/ViDAR/issues/88) | Narrow recorded-frame corpus. Not a TRACE replay product. Collection can start as soon as a camera exists. |
| B8 | [#89](https://github.com/The-Allsparks/ViDAR/issues/89) | Docs + extension guide after names stabilize. |

Copy-out TeamCode interpreter sample ships with B5/B8, not inside `vidar.detect`.

### Wave C — pose observations (after #97 A/B)

Canonical pose stays outside ViDAR. This wave **emits and optionally gates** observations.

| Order | Issue | Deliverable |
|-------|-------|-------------|
| C0 | [#98](https://github.com/The-Allsparks/ViDAR/issues/98) | Consume authoritative fused pose + history for projection (needed before identical FLOWERs). |
| C1 | [#101](https://github.com/The-Allsparks/ViDAR/issues/101) | Generic visual-landmark geometry on `VidarFixtureSpec`. |
| C2 | [#104](https://github.com/The-Allsparks/ViDAR/issues/104) | Identical-fixture association + uniqueness gate. Safety issue: wrong FLOWER must not snap pose. |
| C3 | [#103](https://github.com/The-Allsparks/ViDAR/issues/103) | Confidence / covariance so fusion can reject junk. |
| C4 | [#102](https://github.com/The-Allsparks/ViDAR/issues/102) | Pose observations from known FLOWER geometry. Flag default **off**. |
| C5 | [#91](https://github.com/The-Allsparks/ViDAR/issues/91) | Robot pose from **low CELL cluster + HIVE kinematics**. Unique IDs, so association is easier than FLOWERs, but still not SDK `robotPose`. Blocked on #76 amendment accept + #77 + #86. |
| C6 | [#111](https://github.com/The-Allsparks/ViDAR/issues/111) | Synthetic / replay tests that fail CI when association snaps to the wrong flower. Can start as soon as C2 types exist. |

### Wave D — pose-aware vision and tag scheduling

| Order | Issue | Deliverable |
|-------|-------|-------------|
| D1 | [#106](https://github.com/The-Allsparks/ViDAR/issues/106) | Fixture visibility prediction + camera scoring. |
| D2 | [#105](https://github.com/The-Allsparks/ViDAR/issues/105) | Pose-aware landmark ROI (replace static 65% bands for fixtures/tags). |
| D3 | [#107](https://github.com/The-Allsparks/ViDAR/issues/107) | Opportunistic AprilTag scheduler. Scout gets cheaper during cooldown (same stack, predicted ROI). |
| D4 | [#108](https://github.com/The-Allsparks/ViDAR/issues/108) | Distress / recovery override. Not "the robot bumped." Hitting 30 s without a tag is **not** recovery if FLOWER association is still unique. |
| D5 | [#110](https://github.com/The-Allsparks/ViDAR/issues/110) | Localization debug overlays on top of #87. |
| D6 | [#109](https://github.com/The-Allsparks/ViDAR/issues/109) | ROI / scheduler / localization fields on `VidarMetrics`. TRACE adapter later; no TRACE compile dependency now. |

### Wave E — hardware truth (cannot close from CI)

| Order | Issue | Deliverable |
|-------|-------|-------------|
| E1 | [#112](https://github.com/The-Allsparks/ViDAR/issues/112) | 720p four-camera benchmark: processed FPS/camera and ~20 ms loop, or an honest fail. |
| E2 | [#26](https://github.com/The-Allsparks/ViDAR/issues/26) | Fill `docs/validation-log.md` on a real Hub. Desktop FPS is not a Hub pass. |
| E3 | [#40](https://github.com/The-Allsparks/ViDAR/issues/40) | After Hub Tick ms exists: consider tick-on-mailbox, lock shrink, mailbox copy. Measure first. Tick p50/p95/max already exist from closed #44. |
| E4 | [#100](https://github.com/The-Allsparks/ViDAR/issues/100) | Four-camera extrinsics good enough for geometric localization. Uses #14 / #16 artifacts. |

### Explicitly later

| Issue | Why later |
|-------|-----------|
| [#15](https://github.com/The-Allsparks/ViDAR/issues/15) | Ceres. CAD/tape mounts are enough until Hub calibration exists. |
| [#113](https://github.com/The-Allsparks/ViDAR/issues/113) | Partial constraints when full pose is unsafe. Do not block Waves A–D. |
| [#47](https://github.com/The-Allsparks/ViDAR/issues/47) | Spotless baseline. Dedicated PR after behavior waves, never mixed. |

---

## What "done" looks like for BIOBUZZ shop use

Students can:

1. Open Discover and read fixture id, physical stack or HIVE orientation, freshness, and age.
2. Say out loud: "ViDAR saw POLLEN, BLUE, POLLEN, RED. TeamCode decides owner."
3. Explain why a FLOWER pose observation is dropped when two flowers look equally likely.
4. Explain why AprilTags are quiet for a few seconds after a trusted observation, and why HIVE state still ages.
5. Show a validation-log row with Hub firmware, camera count, 720p, Tick ms, and pass/fail — or show that the row is still blank.

ViDAR still does not pick a scoring target, command drive, or claim match-ready vision without Hub rows.

---

## Suggested next human actions

1. Maintainer comment **accept** (or request changes) on #76, #97 (pick A or B), and #107 shop locks.
2. File #77 transcription from V1 + Onshape with cited figures (may proceed as docs/JSON even while #76 is in review, but do not invent IDs).
3. Small PR for #28 templates.
4. Behavior-neutral PR for #39 so fixture code has somewhere to land.
5. After #76 accept: #78 loader, then projector+store.

---

## Audit actions taken on GitHub (2026-09-14)

See comments on the epics and the leftover August issues. Summary:

- No issues closed (none were implemented; overlaps kept with a documented split).
- #28 unblocked (`status:ready`); PR #18 and "no dependabot" evidence were stale.
- #40 duplicate `status:ready` + `status:blocked` reduced to `status:blocked` (Hub Tick ms).
- #15 stays `status:blocked` as **deferred**, with a note that XYZ + ranging prerequisites already landed.
- #16 "depends on Vitelli XYZ" marked landed; issue already `status:ready`.
- #19 / #37 checklists refreshed to match closed children.
