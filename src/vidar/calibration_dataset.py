"""Offline calibration dataset format validation (Java parity)."""

from __future__ import annotations

import json
from typing import Any


FORMAT_VERSION = "1.0"


def validate_record_json(line: str) -> list[str]:
    errors: list[str] = []
    if not line or not line.strip():
        return ["empty line"]
    try:
        record: dict[str, Any] = json.loads(line.strip())
    except json.JSONDecodeError as exc:
        return [f"invalid json: {exc}"]

    for key in ("formatVersion", "cameraId"):
        if not record.get(key):
            errors.append(f"missing {key}")
    if "timestampNanos" not in record:
        errors.append("missing timestampNanos")
    intr = record.get("intrinsics")
    if not isinstance(intr, dict):
        errors.append("missing intrinsics")
    else:
        for key in ("fx", "fy", "imageWidth", "imageHeight"):
            if intr.get(key, 0) <= 0:
                errors.append(f"invalid {key}")
    if "robotTCamera" not in record:
        errors.append("missing robotTCamera")
    corners = record.get("markerCorners")
    if corners is not None and len(corners) != 4:
        errors.append("markerCorners must have 4 entries")
    return errors
