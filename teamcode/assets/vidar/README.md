# ViDAR TeamCode assets

Install copy for FTC projects: copy this folder to `TeamCode/src/main/assets/vidar/`.

| File | Role |
|------|------|
| `default-season.json` / `default-robot.json` | **Generated** fallbacks — do not hand-edit |
| `season.json` / `robot.json` (your files) | Team overrides (copy from `config/seasons/` / `config/robots/`) |

Authoritative defaults live under
`teamcode/org/firstinspires/ftc/teamcode/vidar/config/bundled/`.
Regenerate the copies here with:

```bash
python scripts/generate_default_config_assets.py
```

CI (`tests/architecture/test_asset_parity.py`) fails if the copies drift.
