# BIOBUZZ 2026-27 field transcription

Season JSON: [`2026-biobuzz.json`](2026-biobuzz.json). This is the citation sidecar for [ViDAR#77](https://github.com/The-Allsparks/ViDAR/issues/77). Do not treat FORGE notes or HTML scraper leftovers as the rules authority.

**BIOBUZZ AprilTags are 3.25 in 36h11 clusters on CELL bottoms, not DECODE 8.125 in goal tags.** `localization: false` means the tag is **not** a static SDK landmark. HIVE kinematics pose is [ViDAR#91](https://github.com/The-Allsparks/ViDAR/issues/91), not this JSON flag.

## FIRST documents used

| Document | Version | Date | What we copied |
|----------|---------|------|----------------|
| BIOBUZZ Competition Manual (English PDF) | V1 | 12 Sep 2026 (Kickoff) | sections 9.5-9.9, Figures 9-12, 9-15, 9-16, **9-17**; section 10.3.1 MATCH-start HIVE tilt |
| Event Field Setup Guide | V1.0 | 12 Sep 2026 | section 9.3 HIVE AprilTag labels; section 10 Flower Installation / Flower Locations; section 11.1 MATCH-start HIVE tilt |
| Onshape Field CAD | Version 1 | 12 Sep 2026 (Playing Field page) | Public document `BIOBUZZ Playing Field` (`a355e772e3d24813de7852ee`). Anonymous assembly API still 401. Pipe OD/spacing and FLOWER xy/yaw measured from FIRST Field CAD STEP `BIOBUZZ_Full Field.20260912.step`. |
| Field CAD (STEP) | V1 20260912 | 12 Sep 2026 | [field-cad-step](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/field-cad-step). `am-5862` pipes; four `am-5855` FLOWER poses mapped to FTC x/y/yaw. |
| FLOWER Scoring Volume CAD | Version 1 | 12 Sep 2026 | Linked from Competition Manual CAD Reference 10-4 / [scoring-volume](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/scoring-volume). Used only to confirm the scoring volume is the FLOWER rings - not a field-pose table. |
| Playing Field hub | - | - | https://ftc-resources.firstinspires.org/ftc/archive/2027/field |

Local FORGE copies (PDFs gitignored): `FORGE/season/2026-2027-biobuzz/official/`. HTML extract is lossy (PDF Figure 9-17 dropped some ID glyphs in text extraction; the **figure image** is authoritative).

## HIVE AprilTags (Figure 9-17)

Audience is at the **bottom** of Figure 9-17. Red is **left** from the audience (section 9.5). IDs left-to-right on the figure (`stickerOrder` 0 = leftmost):

| Figure label | Alliance | Cell | IDs L->R |
|--------------|----------|------|----------|
| Red Scoring Tags | red | opposite audience | 33, 32, 31, 30 |
| Blue Scoring Tags | blue | opposite audience | 45, 44, 43, 42 |
| Red Audience Tags | red | audience | 34, 35, 36, 37 |
| Blue Audience Tags | blue | audience | 38, 39, 40, 41 |

Manual prose (section 9.9) lists the same four ranges. Size **3.25 in**, family **36h11**. Cluster of four on each CELL **bottom**, facing the TILES, bottom edge toward field center (Figure 9-16). Figure 9-15: 3.25 in square; 6.5 in between the first pair of tag centers; 7.0 in across the middle gap; reference holes 0.5 in diameter, 9.938 in from the front of the CELL to the reference-hole centerline.

**Omitted:** per-tag `positionIn` / `orientationDeg`. Tags move when a HIVE tips. Figure 9-17 does not publish inch coordinates. MATCH-start tilt is described (section 10.3.1 / Setup Guide section 11.1) but is not a static field pose. That work is #91.

## FLOWERs (`fixtures[]` + `flowerGeometry`)

Shared part geometry (opening height, pipes) is `flowerGeometry`. Four identical `static_field` fixtures place that geometry on the field: `position.x/y` and `orientationDeg.yaw`. Height is **not** on the pose (not copied as `z`). `visualLandmarks` (#101) is not in this wrap, so pipe points stay on `flowerGeometry` until that lands. Do not grow `namedPoses` as a second FLOWER API.

Local frame: origin at the top-opening center in XY; +Z up; +Y out the opening into the FIELD; +X right-handed. Yaw 0 faces +X.

STEP is Y-up. FTC mapping: X=STEP X, Y=STEP Z, Z=STEP Y. Audience is the STEP -Z wall (red audience cell STEP X<0, blue STEP X>0).

| id | x | y | yaw | Wall |
|----|---|---|-----|------|
| `flower_audience` | -23.3926 | -68.0416 | 90 | audience (-Y) |
| `flower_opposite_audience` | 23.3926 | 68.0416 | -90 | opposite audience (+Y) |
| `flower_red` | -68.0416 | 23.3926 | 0 | red (-X, left from audience) |
| `flower_blue` | 68.0416 | -23.3926 | 180 | blue (+X) |

`hive_structure` stays in `namedPoses` (x=0, y=0, z=**43.95 in** pivot, section 9.6.1). It is not a `static_field` FLOWER fixture.

Pose origin is the 3.45 in HIPS-pipe square centroid. Setup Guide 10.4 / Figure 9-17 look mid-wall and publish **no inches**. CAD is **23.3926 in** from each wall midpoint. Keep both.

Figure 9-12 opening / backstop / retrieval numbers and STEP pipes stay in `flowerGeometry`. Four FLOWERs must not emit pose obs (#104 / #102). CAD green RGB (95, 167, 61) is not a detector.

### HIPS pipes (ranging)

Competition Manual section 9.7 only says four HIPS pipes. STEP `am-5862: Flower HIPS Pipe` (16 instances):

| Quantity | Value | Source |
|----------|-------|--------|
| Outer diameter | **1.05 in** | STEP cylinder radius 0.013335 m |
| Inner diameter | **0.85 in** | STEP cylinder radius 0.010795 m |
| Wall | **0.10 in** | OD-ID |
| Count / layout | 4 pipes, **3.45 in x 3.45 in** square of vertical axes | 4 FLOWERs x 4 pipes |
| Wall-parallel pair spacing | **3.45 in** (front pair and back pair) | Same square, plane parallel to the wall |
| Front-to-back depth | **3.45 in** | Same square, toward field center |
| Color | CAD green RGB **(95, 167, 61)** | STEP `COLOUR_RGB`; Assembly renders match. Not HSV. |

Assembly Guide 5.4 / 5.5: two **front** (field-side) pipes and two **back** (wall-side) pipes. Apparent front-pair gap and pipe width give range; front vs back pair as a trapezoid gives yaw. Loaders parse FLOWER **poses** from `fixtures[]` and still ignore `flowerGeometry` until #101.

## CAD vs manual

Pipe OD/spacing are **CAD-only** (not in section 9.7). Opening height **21.5 in** stays on `flowerGeometry` from the manual. Setup-guide mid-wall vs CAD 23.3926 in offset: **keep both**. If a later CAD measure disagrees with 21.5 in / 43.95 in / 18.8 in / 3.25 in, keep both numbers - do not silently pick.

## NECTAR

Section 9.8: approximately **3.6 in** (9.1 cm) Gopher ResisDent balls, red `am-5852_red` and blue `am-5852_blue`. HSV in JSON starts from this file's alliance plate **primary** hues and is **not** field-tuned. Element specs do not load `hsvWrap` (plates only), so red NECTAR does not include the plate wrap band 168-179.

## Not in this file

`namedPoses` keeps `hive_structure` only (HIVE `april_tag` + `tagIds` fixture can wait). `robot.json` camera mounts unchanged. `VidarLocalizationFusion` is not taught that HIVE tags are static landmarks.
