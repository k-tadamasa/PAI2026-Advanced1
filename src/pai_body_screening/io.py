from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from .collision import CollisionResult
from .iftomm import IftommConfig, TrajectoryResult
from .models import MeshData


def _first(mapping: dict[str, Any], names: tuple[str, ...]) -> Any:
    for name in names:
        if name in mapping and mapping[name] is not None:
            return mapping[name]
    raise KeyError(f"none of the required keys are present: {', '.join(names)}")


def load_mesh_json(
    path: str | Path,
    *,
    face_part_ids_path: str | Path | None = None,
    camera_position: list[float] | None = None,
) -> MeshData:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("input JSON must be an object")
    mesh_payload = payload.get("mesh", payload)
    if not isinstance(mesh_payload, dict):
        raise ValueError("mesh must be a JSON object")
    resolved_camera = camera_position
    if resolved_camera is None:
        resolved_camera = payload.get(
            "camera_position_m", mesh_payload.get("camera_position_m")
        )
    if resolved_camera is None:
        resolved_camera = payload.get(
            "camera_position", mesh_payload.get("camera_position")
        )
    if resolved_camera is None:
        raise KeyError("camera_position_m is required for front-face selection")
    if face_part_ids_path is None:
        face_part_ids = _first(
            mesh_payload,
            ("face_part_ids", "human3d_face_part_ids", "part_ids"),
        )
    else:
        face_part_ids = json.loads(
            Path(face_part_ids_path).read_text(encoding="utf-8")
        )
        if isinstance(face_part_ids, dict):
            face_part_ids = _first(
                face_part_ids,
                ("face_part_ids", "human3d_face_part_ids", "part_ids"),
            )
    return MeshData.from_arrays(
        vertices=_first(mesh_payload, ("vertices", "vertices_world_m")),
        faces=_first(mesh_payload, ("faces", "triangles")),
        face_part_ids=face_part_ids,
        camera_position=resolved_camera,
    )


def save_trajectory_json(
    path: str | Path,
    result: TrajectoryResult,
    collisions: CollisionResult,
    config: IftommConfig,
) -> None:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    poses = []
    for index, (point, normal, quaternion, collision) in enumerate(
        zip(
            result.smoothed_points,
            result.smoothed_normals,
            result.smoothed_quaternions_wxyz,
            collisions.pose_collision,
        )
    ):
        poses.append(
            {
                "index": index,
                "position_m": point.tolist(),
                "surface_normal": normal.tolist(),
                "orientation_wxyz": quaternion.tolist(),
                "collision": bool(collision),
                "display_color": "#ef4444" if collision else "#22c55e",
                "colliding_face_ids": collisions.colliding_face_ids[index],
                "collision_components": (
                    collisions.colliding_components[index]
                    if collisions.colliding_components
                    else []
                ),
            }
        )
    payload = {
        "schema_version": "pai2026_kitahara_surface_raster_v2",
        "method": "Kitahara_body_surface_raster_1p5_lane_spacing",
        "scope": {
            "front_only": config.front_only,
            "human3d_part_ids": sorted(config.part_ids),
            "collision_affects_planning": False,
            "collision_model": collisions.collision_model,
        },
        "parameters": {
            "standoff_m": config.standoff_m,
            "ellipsoid_radii_m": list(config.ellipsoid_radii_m),
            "lane_count": config.lane_count,
            "lane_spacing_scale": config.lane_spacing_scale,
            "central_lane_gap_scale": config.central_lane_gap_scale,
            "flip_md_at_center": config.flip_md_at_center,
            "lane_half_width_m": config.lane_half_width_m,
            "local_fit_half_window": config.local_fit_half_window,
            "interpolation_gap_m": config.interpolation_gap_m,
            "loess_window_points": config.loess_window_points,
            "lane_reference_y_m": result.lane_reference_y_m.tolist(),
            "target_coverage_ratio": config.target_coverage_ratio,
            "front_normal_max_angle_deg": config.front_normal_max_angle_deg,
            "front_width_fraction": config.front_width_fraction,
            "front_top_margin_m": config.front_top_margin_m,
            "maximum_orientation_step_deg": config.maximum_orientation_step_deg,
        },
        "metrics": {
            "coverage_ratio": result.coverage_ratio,
            "target_area_m2": result.target_area_m2,
            "inspected_area_m2": result.inspected_area_m2,
            "path_length_m": result.path_length_m,
            "planning_time_s": result.planning_time_s,
            "control_point_count": len(result.control_points),
            "trajectory_pose_count": len(result.smoothed_points),
            "collision_pose_count": int(np.count_nonzero(collisions.pose_collision)),
            "stop_reason": result.stop_reason,
        },
        "control_points": [
            {
                "face_id": int(face_id),
                "position_m": point.tolist(),
                "surface_normal": normal.tolist(),
            }
            for face_id, point, normal in zip(
                result.control_face_ids,
                result.control_points,
                result.control_normals,
            )
        ],
        "trajectory": poses,
    }
    output_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
