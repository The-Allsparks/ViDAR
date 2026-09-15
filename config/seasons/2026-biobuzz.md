# BIOBUZZ 2026-27 field transcription

Season JSON: [`2026-biobuzz.json`](2026-biobuzz.json). This is the citation sidecar for [ViDAR#77](https://github.com/The-Allsparks/ViDAR/issues/77). Do not treat FORGE notes or HTML scraper leftovers as the rules authority.

**BIOBUZZ AprilTags are 3.25 in 36h11 clusters on CELL bottoms, not DECODE 8.125 in goal tags.** `localization: false` means the tag is **not** a static SDK landmark. HIVE kinematics pose is [ViDAR#91](https://github.com/The-Allsparks/ViDAR/issues/91), not this JSON flag.

## FIRST documents used

| Document | Version | Date | What we copied |
|----------|---------|------|----------------|
| BIOBUZZ Competition Manual (English PDF) | V1 | 12 Sep 2026 (Kickoff) | §§9.5–9.9, Figures 9-12, 9-15, 9-16, **9-17**; §10.3.1 MATCH-start HIVE tilt |
| Event Field Setup Guide | V1.0 | 12 Sep 2026 | §9.3 HIVE AprilTag labels; §10 Flower Installation / Flower Locations; §11.1 MATCH-start HIVE tilt |
| Onshape Field CAD | Version 1 | 12 Sep 2026 (Playing Field page) | **Not measured.** Public document `BIOBUZZ™ Playing Field` (`a355e772e3d24813de7852ee`). Anonymous assembly API returned HTTP 401. No inch xy/yaw was invented from the 144 in FIELD. |
| FLOWER Scoring Volume CAD | Version 1 | 12 Sep 2026 | Linked from Competition Manual CAD Reference 10-4 / [scoring-volume](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/scoring-volume). Used only to confirm the scoring volume is the FLOWER rings — not a field-pose table. |
| Playing Field hub | — | — | https://ftc-resources.firstinspires.org/ftc/archive/2027/field |

Local FORGE copies (PDFs gitignored): `FORGE/season/2026-2027-biobuzz/official/`. HTML extract is lossy (PDF Figure 9-17 dropped some ID glyphs in text extraction; the **figure image** is authoritative).

## HIVE AprilTags (Figure 9-17)

Audience is at the **bottom** of Figure 9-17. Red is **left** from the audience (§9.5). IDs left-to-right on the figure (`stickerOrder` 0 = leftmost):

| Figure label | Alliance | Cell | IDs L→R |
|--------------|----------|------|---------|
| Red Scoring Tags | red | opposite audience | 33, 32, 31, 30 |
| Blue Scoring Tags | blue | opposite audience | 45, 44, 43, 42 |
| Red Audience Tags | red | audience | 34, 35, 36, 37 |
| Blue Audience Tags | blue | audience | 38, 39, 40, 41 |

Manual prose (§9.9) lists the same four ranges. Size **3.25 in**, family **36h11**. Cluster of four on each CELL **bottom**, facing the TILES, bottom edge toward field center (Figure 9-16). Figure 9-15: 3.25 in square; 6.5 in between the first pair of tag centers; 7.0 in across the middle gap; reference holes 0.5 in diameter, 9.938 in from the front of the CELL to the reference-hole centerline.

**Omitted:** per-tag `positionIn` / `orientationDeg`. Tags move when a HIVE tips. Figure 9-17 does not publish inch coordinates. MATCH-start tilt is described (§10.3.1 / Setup Guide §11.1) but is not a static field pose. That work is #91.

## FLOWERs (`namedPoses`)

Four FLOWERs on the perimeter, one mid-wall (Figure 9-17; Setup Guide §10.4 “Flower Locations”). No official `flower_1`… names in V1 — JSON ids are wall names for later `fixtures[]` wrap (#78).

| id | Cited numbers | Omitted |
|----|---------------|---------|
| `flower_audience` | z = **21.5 in** (top opening above TILES, §9.7 / Figure 9-12); audience wall | xy, yaw |
| `flower_opposite_audience` | z = 21.5 in; wall opposite audience | xy, yaw |
| `flower_red` | z = 21.5 in; red wall (left from audience) | xy, yaw |
| `flower_blue` | z = 21.5 in; blue wall | xy, yaw |
| `hive_structure` | x=0, y=0 from “center of the FIELD” (§9.6); z = **43.95 in** pivot (§9.6.1); CELLs ~**18.8 in** apart (§9.6.2) | yaw |

Figure 9-12 also cites a 4.0 in top opening, 1.25 in backstop, 3.55 × 3.57 in retrieval opening. Those are FLOWER geometry, not field xy.

## CAD vs manual

No numeric disagreement was recorded because Onshape transforms were not read. If a later CAD measure disagrees with 21.5 in / 43.95 in / 18.8 in / 3.25 in, **keep both numbers** — do not silently pick.

## NECTAR

§9.8: approximately **3.6 in** (9.1 cm) Gopher ResisDent balls, red `am-5852_red` and blue `am-5852_blue`. HSV in JSON starts from this file’s alliance plate **primary** hues and is **not** field-tuned. Element specs do not load `hsvWrap` (plates only), so red NECTAR does not include the plate wrap band 168–179.

## Not in this file

`fixtures[]` waits for #78. `robot.json` camera mounts unchanged. `VidarLocalizationFusion` is not taught that HIVE tags are static landmarks.
