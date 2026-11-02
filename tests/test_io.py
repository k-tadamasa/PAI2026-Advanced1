from __future__ import annotations

import json

from pai_body_screening.collision import (
    DetectorGripperMeshModel,
    evaluate_detector_gripper_collisions,
)
from pai_body_screening.iftomm import IftommConfig, generate_trajectory
from pai_body_screening.io import load_mesh_json, save_trajectory_json
from pai_body_screening.models import MeshData


def test_json_round_trip_and_collision_colours(
    tmp_path,
    demo_mesh: MeshData,
    collision_model: DetectorGripperMeshModel,
) -> None:
    input_path = tmp_path / "mesh.json"
    input_path.write_text(
        json.dumps(
            {
                "camera_position_m": demo_mesh.camera_position.tolist(),
                "mesh": {
                    "vertices": demo_mesh.vertices.tolist(),
                    "faces": demo_mesh.faces.tolist(),
                    "face_part_ids": demo_mesh.face_part_ids.tolist(),
                },
            }
        ),
        encoding="utf-8",
    )
    loaded = load_mesh_json(input_path)
    config = IftommConfig(target_coverage_ratio=0.80)
    result = generate_trajectory(loaded, config)
    collisions = evaluate_detector_gripper_collisions(
        loaded,
        result.smoothed_points,
        result.smoothed_quaternions_wxyz,
        collision_model,
    )
    output_path = tmp_path / "trajectory.json"
    save_trajectory_json(output_path, result, collisions, config)
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["scope"]["human3d_part_ids"] == [2, 7, 14, 15]
    assert payload["scope"]["collision_affects_planning"] is False
    assert payload["scope"]["collision_model"] == "md_gripper_triangle_mesh"
    assert {pose["display_color"] for pose in payload["trajectory"]} <= {
        "#22c55e",
        "#ef4444",
    }


def test_external_face_part_mapping(tmp_path, demo_mesh: MeshData) -> None:
    input_path = tmp_path / "job.json"
    input_path.write_text(
        json.dumps(
            {
                "camera_position_m": demo_mesh.camera_position.tolist(),
                "mesh": {
                    "vertices": demo_mesh.vertices.tolist(),
                    "faces": demo_mesh.faces.tolist(),
                },
            }
        ),
        encoding="utf-8",
    )
    parts_path = tmp_path / "face-parts.json"
    parts_path.write_text(
        json.dumps(demo_mesh.face_part_ids.tolist()), encoding="utf-8"
    )
    loaded = load_mesh_json(input_path, face_part_ids_path=parts_path)
    assert loaded.face_part_ids.tolist() == demo_mesh.face_part_ids.tolist()
