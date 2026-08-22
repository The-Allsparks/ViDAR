# ViDAR browser simulator

Local vision preview that mirrors on-robot Java tuning (`sim/vidar-tuning.json`).

## Run

From the repo root:

```powershell
.\scripts\serve_sim.ps1
```

Or:

```bash
python scripts/serve_sim.py
```

Open **http://localhost:8765**.

## Parity with robot code

| Feature | Sim module | Java |
|---------|------------|------|
| Contour element + plate detection | `contour-processor.js` | `VidarContourProcessor` |
| Range fusion (size + floor LUT + ground plane) | `geometry.js` | `VidarRangeFusion` / `VidarGeometry` |
| Temporal filter (3/5) | `temporal-filter.js` | `VidarTemporalFilter` |
| Tag scout + gated decode | `tag-pipeline.js` | `VidarTagScoutRunner` / `VidarAdaptiveTagProcessor` |
| Profile tag ROI (upper fraction) | `tag-pipeline.js` `tagRoi()` | `VidarFrameRegions.tagRoi()` |
| `elementId` on detections | `contour-processor.js` | `VidarElementObservation.elementId` |
| Motion tracks (predict / gate / associate) | `spatial-tracks.js` | `VidarTrackAssociator` |
| Offensive lane (foe density) | `offensive-lane.js` | `VidarOffensiveLaneAnalysis` |

Ground-plane ranging needs mount + intrinsics on each camera in `geometry.cameras` inside `vidar-tuning.json` (`mountX/Y/Z`, `mountPitchDeg`, `principalPointX/Y`). Without those fields the sim falls back to size + floor heuristics only (same as missing extrinsics on hub). Set `"legacyRangeFusion": true` under `geometry` to restore the pre-#22 SIZE+FLOOR average for rollback.

Parity tests: `python -m pytest tests/test_sim_geometry_parity.py -v` (runs `sim/js/geometry.test.mjs` when Node is installed).

## Spatial preview sidebar

- **Motion tracks** — uses confirmed detections plus tag odom (`tagState.odom`) when present. Remembered tracks render as dashed circles with `T{id}` labels.
- **Offensive lane** — counts foe tracks in left / center / right thirds of the forward cone (same defaults as `VidarConfig`).

## Captures

**Capture still** saves frame, process view, mask, and JSON metadata under `captures/` when the server is running.
