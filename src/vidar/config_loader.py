from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from vidar.models import (
    Alliance,
    AprilTagSpec,
    ElementDetectorType,
    CameraMount,
    CameraProfile,
    CameraRoiConfig,
    ElementShape,
    ElementSpec,
    FieldSpec,
    HsvRange,
    PlateSpec,
    RobotConfig,
    RobotDimensions,
    SeasonConfig,
)
from vidar.units import DistanceUnit


def _read_dist(raw: dict[str, Any], *keys: str, default: float | None = None) -> float:
    """Read a distance field; accepts legacy ``*Dist`` and ``*In`` keys."""
    for key in keys:
        if key in raw:
            return float(raw[key])
    if default is not None:
        return default
    raise KeyError(f"Missing distance key (tried {keys})")


def _parse_hsv(raw: dict[str, Any]) -> HsvRange:
    return HsvRange(
        h_min=int(raw["hMin"]),
        h_max=int(raw["hMax"]),
        s_min=int(raw["sMin"]),
        s_max=int(raw["sMax"]),
        v_min=int(raw["vMin"]),
        v_max=int(raw["vMax"]),
    )


def _parse_shape(value: str) -> ElementShape:
    try:
        return ElementShape(value.lower())
    except ValueError:
        return ElementShape.CIRCLE


def _parse_detector(value: str) -> ElementDetectorType:
    mapping = {
        "color_blob": ElementDetectorType.COLOR_BLOB,
        "color_blob_with_local_hough": ElementDetectorType.COLOR_BLOB_WITH_LOCAL_HOUGH,
    }
    return mapping.get(value.lower(), ElementDetectorType.COLOR_BLOB_WITH_LOCAL_HOUGH)


def _parse_alliance(value: str) -> Alliance:
    return Alliance.RED if value.lower() == "red" else Alliance.BLUE


def _parse_element(raw: dict[str, Any]) -> ElementSpec:
    hsv = _parse_hsv(raw["hsv"])
    filters = raw.get("filters") or {}
    morph = raw.get("morphology") or {}
    hough = raw.get("hough") or {}
    interior = raw.get("interior") or {}
    max_aspect_ratio = float(filters.get("maxAspectRatio", 2.0))
    return ElementSpec(
        id=str(raw["id"]),
        label=str(raw.get("label", raw["id"])),
        shape=_parse_shape(str(raw.get("shape", "circle"))),
        diameter=_read_dist(raw, "diameter", "diameterDist", "diameterIn"),
        detector=_parse_detector(str(raw.get("detector", "color_blob_with_local_hough"))),
        hsv=hsv,
        min_area_px=float(filters.get("minAreaPx", 45)),
        max_area_px=float(filters.get("maxAreaPx", 12000)),
        min_width_px=float(filters.get("minWidthPx", 8)),
        max_width_px=float(filters.get("maxWidthPx", 80)),
        min_height_px=float(filters.get("minHeightPx", 8)),
        max_height_px=float(filters.get("maxHeightPx", 80)),
        max_aspect_ratio=max_aspect_ratio,
        min_circularity=float(filters.get("minCircularity", 0.55)),
        min_rectangularity=float(filters.get("minRectangularity", 0.0)),
        min_aspect=float(filters.get("minAspect", 1.0)),
        max_aspect=float(filters.get("maxAspect", max_aspect_ratio)),
        min_fill_ratio=float(filters.get("minFillRatio", 0.55)),
        min_interior_score=float(filters.get("minInteriorScore", 0.12)),
        interior_bright=int(interior.get("brightMin", 90)),
        interior_spread=int(interior.get("spreadMax", 60)),
        hole_dark_max=int(interior.get("holeDarkMax", 45)),
        morph_erode_passes=int(morph.get("erodePasses", 0)),
        morph_dilate_passes=int(morph.get("dilatePasses", 0)),
        morph_open_passes=int(morph.get("openPasses", 1)),
        morph_close_passes=int(morph.get("closePasses", 2)),
        hough_dp=float(hough.get("dp", 1.2)),
        hough_min_dist=float(hough.get("minDist", 24)),
        hough_param1=float(hough.get("param1", 80)),
        hough_param2=float(hough.get("param2", 11)),
        hough_min_radius=int(hough.get("minRadius", 8)),
        hough_max_radius=int(hough.get("maxRadius", 36)),
        hough_min_interior=float(hough.get("minInterior", 0.14)),
        min_area_hough_px=float(hough.get("minAreaPx", 45)),
    )


def _parse_plate(raw: dict[str, Any]) -> PlateSpec:
    hsv = _parse_hsv(raw["hsv"])
    wrap_raw = raw.get("hsvWrap")
    filters = raw.get("filters") or {}
    white = raw.get("whiteDigit") or {}
    return PlateSpec(
        alliance=_parse_alliance(str(raw["alliance"])),
        hsv=hsv,
        width=_read_dist(raw, "width", "widthDist", "widthIn", default=12.0),
        hsv_wrap=_parse_hsv(wrap_raw) if wrap_raw else None,
        min_area_px=float(filters.get("minAreaPx", 120)),
        max_area_px=float(filters.get("maxAreaPx", 12000)),
        min_aspect=float(filters.get("minAspect", 1.15)),
        max_aspect=float(filters.get("maxAspect", 4.5)),
        min_rectangularity=float(filters.get("minRectangularity", 0.45)),
        min_white_ratio=float(filters.get("minWhiteRatio", 0.12)),
        white_sample_grid=int(white.get("sampleGrid", 5)),
        white_bright_min=int(white.get("brightMin", 175)),
        white_spread_max=int(white.get("spreadMax", 55)),
    )


def _parse_roi(raw: dict[str, Any] | None) -> CameraRoiConfig:
    if not raw:
        return CameraRoiConfig.default()
    element = raw.get("element") or {}
    plate = raw.get("plate") or {}
    tag = raw.get("tag") or {}
    return CameraRoiConfig(
        element_lower_fraction=float(
            element.get("lowerFraction", raw.get("elementLowerFraction", 0.65))
        ),
        plate_start_fraction=float(plate.get("startFraction", raw.get("plateStartFraction", 0.30))),
        plate_band_fraction=float(plate.get("bandFraction", raw.get("plateBandFraction", 0.40))),
        tag_upper_fraction=float(tag.get("upperFraction", raw.get("tagUpperFraction", 0.65))),
        element_enabled=bool(element.get("enabled", raw.get("elementEnabled", True))),
        plate_enabled=bool(plate.get("enabled", raw.get("plateEnabled", True))),
        tag_enabled=bool(tag.get("enabled", raw.get("tagEnabled", True))),
    )


def _merge_dict(defaults: dict[str, Any] | None, override: dict[str, Any] | None) -> dict[str, Any]:
    merged = dict(defaults or {})
    if override:
        merged.update(override)
    return merged


def _read_mount_position(mount: dict[str, Any], axis: str) -> float:
    position = mount.get("position") or mount.get("positionIn") or {}
    legacy_flat = f"{axis}Dist"
    legacy_in = f"{axis}In"
    legacy_key = f"mount{axis.upper()}{axis[1:]}Dist" if len(axis) == 1 else f"mount{axis[0].upper()}{axis[1:]}Dist"
    legacy_key_in = f"mount{axis.upper()}{axis[1:]}In" if len(axis) == 1 else f"mount{axis[0].upper()}{axis[1:]}In"
    if axis in position:
        return float(position[axis])
    if f"{axis}Dist" in position:
        return float(position[f"{axis}Dist"])
    if f"{axis}In" in position:
        return float(position[f"{axis}In"])
    for key in (axis, legacy_flat, legacy_in, legacy_key, legacy_key_in):
        if key in mount:
            return float(mount[key])
    return 0.0


def _read_mount_orientation(mount: dict[str, Any], name: str) -> float:
    orientation = mount.get("orientationDeg") or {}
    flat_key = f"{name}Deg"
    legacy_key = f"mount{name[0].upper()}{name[1:]}Deg"
    if name in orientation:
        return float(orientation[name])
    if name == "yaw" and "bearing" in orientation:
        return float(orientation["bearing"])
    if flat_key in mount:
        return float(mount[flat_key])
    if legacy_key in mount:
        return float(mount[legacy_key])
    return 0.0


def _build_camera_profile_json(
    cam: dict[str, Any],
    camera_defaults: dict[str, Any] | None,
    mount_defaults: dict[str, Any] | None,
) -> dict[str, Any]:
    camera = _merge_dict(camera_defaults, cam.get("camera"))
    mount = _merge_dict(mount_defaults, cam.get("mount"))
    legacy = cam.get("profile")
    if legacy:
        camera = _merge_dict(camera, legacy)
        mount = _merge_dict(mount, legacy)

    name = cam.get("name") or mount.get("name") or camera.get("name") or "camera"
    floor_lut = camera.get("floorLut")
    if not floor_lut:
        raise ValueError(f'Camera "{name}" missing floorLut (set cameraDefaults.floorLut)')

    return {
        "name": name,
        "bearingDeg": mount.get("bearingDeg", camera.get("bearingDeg", 0)),
        "horizonRowPx": camera.get("horizonRowPx", 12),
        "focalLengthPx": camera.get("focalLengthPx", 340),
        "focalLengthYPx": camera.get("focalLengthYPx", camera.get("focalLengthPx", 340)),
        "principalPointX": camera.get("principalPointX", 320),
        "principalPointY": camera.get("principalPointY", 240),
        "horizontalFovDeg": camera.get("horizontalFovDeg", 70),
        "verticalFovDeg": camera.get("verticalFovDeg", 55),
        "plateWidth": _read_dist(camera, "plateWidth", "plateWidthDist", "plateWidthIn", default=12.0),
        "floorLut": floor_lut,
        "roi": camera.get("roi"),
        "mountX": _read_mount_position(mount, "x"),
        "mountY": _read_mount_position(mount, "y"),
        "mountZ": _read_mount_position(mount, "z"),
        "mountYawDeg": _read_mount_orientation(mount, "yaw"),
        "mountPitchDeg": _read_mount_orientation(mount, "pitch"),
        "mountRollDeg": _read_mount_orientation(mount, "roll"),
    }


def _parse_camera_profile(raw: dict[str, Any]) -> CameraProfile:
    lut = raw["floorLut"]
    cy = tuple(float(pt["cy"]) for pt in lut)
    dist = tuple(
        float(pt.get("dist", pt.get("distIn", pt.get("distDist")))) for pt in lut
    )
    return CameraProfile(
        name=str(raw["name"]),
        bearing_deg=float(raw["bearingDeg"]),
        horizon_row_px=int(raw.get("horizonRowPx", 12)),
        focal_length_px=float(raw.get("focalLengthPx", 340)),
        floor_cy_px=cy,
        floor_dist=dist,
        focal_length_y_px=float(raw.get("focalLengthYPx", raw.get("focalLengthPx", 340))),
        principal_point_x=float(raw.get("principalPointX", 320)),
        principal_point_y=float(raw.get("principalPointY", 240)),
        horizontal_fov_deg=float(raw.get("horizontalFovDeg", 70)),
        vertical_fov_deg=float(raw.get("verticalFovDeg", 55)),
        plate_width=float(raw.get("plateWidth", 12.0)),
        mount_x=float(raw.get("mountX", 0)),
        mount_y=float(raw.get("mountY", 0)),
        mount_z=float(raw.get("mountZ", 0)),
        mount_yaw_deg=float(raw.get("mountYawDeg", 0)),
        mount_pitch_deg=float(raw.get("mountPitchDeg", 0)),
        mount_roll_deg=float(raw.get("mountRollDeg", 0)),
        roi_config=_parse_roi(raw.get("roi")),
    )


def _parse_default_tag_size(data: dict[str, Any]) -> float:
    block = data.get("apriltags")
    if isinstance(block, dict):
        for key in ("defaultSize", "defaultSizeDist", "defaultSizeIn"):
            if key in block:
                return float(block[key])
    for key in ("defaultTagSize", "defaultTagSizeDist", "defaultTagSizeIn"):
        if key in data:
            return float(data[key])
    return 8.125


def _read_tag_coord(tag: dict[str, Any], position: dict[str, Any] | None, axis: str) -> float:
    if position and axis in position:
        return float(position[axis])
    for key in (f"{axis}Dist", f"{axis}In"):
        if key in tag:
            return float(tag[key])
    return float("nan")


def _read_tag_orientation(
    tag: dict[str, Any], orientation: dict[str, Any] | None, axis: str
) -> float:
    if orientation and axis in orientation:
        return float(orientation[axis])
    flat = f"{axis}Deg"
    if flat in tag:
        return float(tag[flat])
    return 0.0


def _parse_april_tag(raw: dict[str, Any], default_size: float) -> AprilTagSpec:
    position = raw.get("position") or raw.get("positionIn")
    orientation = raw.get("orientationDeg")
    size = float(raw.get("size", raw.get("sizeIn", default_size)))
    return AprilTagSpec(
        id=int(raw["id"]),
        name=str(raw.get("name", f"tag_{raw['id']}")),
        size=size,
        x=_read_tag_coord(raw, position, "x"),
        y=_read_tag_coord(raw, position, "y"),
        z=_read_tag_coord(raw, position, "z"),
        yaw_deg=_read_tag_orientation(raw, orientation, "yaw"),
        pitch_deg=_read_tag_orientation(raw, orientation, "pitch"),
        roll_deg=_read_tag_orientation(raw, orientation, "roll"),
        localization=bool(raw.get("localization", True)),
    )


def _parse_april_tags(data: dict[str, Any]) -> tuple[tuple[AprilTagSpec, ...], float]:
    default_size = _parse_default_tag_size(data)
    block = data.get("apriltags")
    tags_raw: list[Any] | None = None
    if isinstance(block, dict) and "tags" in block:
        tags_raw = block["tags"]
    elif isinstance(block, list):
        tags_raw = block
    if not tags_raw:
        return (), default_size
    return tuple(_parse_april_tag(item, default_size) for item in tags_raw), default_size


def parse_season(data: dict[str, Any]) -> SeasonConfig:
    fusion = data.get("fusion") or {}
    if "elements" not in data:
        raise ValueError('Season JSON requires "elements" array')
    elements_raw = data["elements"]

    field_raw = data.get("field") or {}
    april_tags, default_tag_size = _parse_april_tags(data)
    return SeasonConfig(
        season_id=str(data["seasonId"]),
        season_name=str(data.get("seasonName", data["seasonId"])),
        field=FieldSpec(
            length=_read_dist(field_raw, "length", "lengthDist", "lengthIn", default=691.2),
            width=_read_dist(field_raw, "width", "widthDist", "widthIn", default=317.0),
        ),
        elements=tuple(_parse_element(item) for item in elements_raw),
        plates=tuple(_parse_plate(item) for item in data["plates"]),
        april_tags=april_tags,
        default_tag_size=default_tag_size,
        min_element_confidence=float(fusion.get("minElementConfidence", 0.35)),
        min_plate_confidence=float(fusion.get("minPlateConfidence", 0.35)),
        max_range_mismatch_ratio=float(fusion.get("maxRangeMismatchRatio", 0.28)),
        distance_unit=DistanceUnit.from_json(data.get("distanceUnit")),
    )


def parse_robot(data: dict[str, Any]) -> RobotConfig:
    cameras_json = data["cameras"]
    camera_defaults = data.get("cameraDefaults")
    mount_defaults = data.get("mountDefaults")
    requested = int(data.get("cameraCount", len(cameras_json)))
    count = max(1, min(4, min(requested, len(cameras_json))))

    mounts: list[CameraMount] = []
    for i in range(count):
        cam = cameras_json[i]
        if "index" in cam and int(cam["index"]) != i:
            raise ValueError(f'Camera entry index {cam["index"]} must match array position {i}')
        profile_json = _build_camera_profile_json(cam, camera_defaults, mount_defaults)
        profile = _parse_camera_profile(profile_json)
        mounts.append(CameraMount(webcam_name=str(cam["webcamName"]), profile=profile))

    alliance = data.get("alliance") or {}
    dimensions_raw = data.get("dimensions") or data.get("dimensionsIn") or data.get("dimensions") or {}
    unit_override = (
        DistanceUnit.from_json(data["distanceUnit"]) if "distanceUnit" in data else None
    )
    return RobotConfig(
        robot_name=str(data.get("robotName", "robot")),
        active_camera_index=int(data.get("activeCameraIndex", 0)),
        cameras=tuple(mounts),
        dimensions=RobotDimensions(
            length=_read_dist(dimensions_raw, "length", "lengthDist", "lengthIn", default=13.0),
            width=_read_dist(dimensions_raw, "width", "widthDist", "widthIn", default=13.0),
            height=_read_dist(dimensions_raw, "height", "heightDist", "heightIn", default=18.0),
        ),
        default_alliance=_parse_alliance(str(alliance.get("defaultAlliance", "red"))),
        distance_unit_override=unit_override,
    )


def load_season(path: str | Path) -> SeasonConfig:
    with open(path, encoding="utf-8") as handle:
        return parse_season(json.load(handle))


def load_robot(path: str | Path) -> RobotConfig:
    with open(path, encoding="utf-8") as handle:
        return parse_robot(json.load(handle))


def default_season(repo_root: Path | None = None) -> SeasonConfig:
    root = repo_root or Path(__file__).resolve().parents[2]
    return load_season(root / "config" / "seasons" / "2026-biobuzz.json")


def default_robot(repo_root: Path | None = None) -> RobotConfig:
    root = repo_root or Path(__file__).resolve().parents[2]
    return load_robot(root / "config" / "robots" / "example-robot.json")
