from __future__ import annotations

import os
import time
from dataclasses import replace

import cv2

from vidar.camera.manager import CameraManager
from vidar.config import load_config
from vidar.detection.pipeline import DetectionPipeline
from vidar.output.telemetry import TelemetryPublisher
from vidar.units import effective_distance_unit, format_distance


def run() -> None:
    config = load_config()
    if os.environ.get("VIDAR_CAMERA_MODE"):
        config = replace(config, camera_mode=os.environ["VIDAR_CAMERA_MODE"])

    print(
        f"[vidar] season={config.season.season_id} robot={config.robot.robot_name} "
        f"mode={config.camera_mode} cameras={config.camera_count}"
    )
    print(
        f"[vidar] capture={config.capture_width}x{config.capture_height} "
        f"roi_scale={config.process_roi_scale} target_fps={config.fps_target}"
    )

    cameras = CameraManager(config)
    pipeline = DetectionPipeline(config)
    telemetry = TelemetryPublisher(config.telemetry_port)
    dist_unit = effective_distance_unit(config.robot, config.season)

    frame_count = 0
    window_started = time.perf_counter()
    loop_started = time.perf_counter()

    try:
        while True:
            all_detections = []

            for camera_index, ok, frame in cameras.read_all():
                if not ok or frame is None:
                    continue

                detections = pipeline.detect(frame, camera_index)
                all_detections.extend(detections)

                if config.show_debug:
                    annotated = pipeline.annotate(frame, detections)
                    cv2.imshow(f"ViDAR {config.robot.cameras[camera_index].profile.name}", annotated)

            frame_count += 1
            elapsed = time.perf_counter() - loop_started
            fps = frame_count / elapsed if elapsed > 0 else 0.0

            if frame_count % config.print_fps_every == 0:
                summary = ", ".join(
                    f"{det.category}:{det.label}@({det.cx:.0f},{det.cy:.0f})"
                    + (
                        f" {format_distance(det.range, dist_unit)}"
                        if det.range is not None
                        else ""
                    )
                    for det in all_detections[:8]
                )
                print(f"[vidar] fps={fps:.1f} detections={len(all_detections)} {summary}")

            telemetry.publish(all_detections, fps)

            if config.show_debug and cv2.waitKey(1) & 0xFF == ord("q"):
                break

            if config.camera_mode == "mock":
                target_frame_time = 1.0 / max(config.fps_target, 1)
                spent = time.perf_counter() - window_started
                if spent < target_frame_time:
                    time.sleep(target_frame_time - spent)
                window_started = time.perf_counter()

    except KeyboardInterrupt:
        print("[vidar] stopped")
    finally:
        telemetry.close()
        cameras.release()
        if config.show_debug:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    run()
