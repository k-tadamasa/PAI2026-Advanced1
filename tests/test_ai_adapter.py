from __future__ import annotations

import json

import pytest

from pai_body_screening.ai_adapter import (
    prepare_planner_input,
    validate_human3d_segfit_result,
)
from pai_body_screening.io import load_mesh_json
from pai_body_screening.models import MeshData


def _result_from_mesh(mesh: MeshData) -> dict:
    return {
        "vertices": mesh.vertices.tolist(),
        "faces": mesh.faces.tolist(),
        "human3d_points": [[0.0, 0.0, 0.0], [0.0, 0.1, 0.0]],
        "human3d_part_ids": [7, 15],
        "human_count": 1,
        "body_model": "neutral",
        "fitting_mode": "standard",
        "fitting_loss": 0.01,
    }


def test_prepare_ai_input(tmp_path, demo_mesh: MeshData) -> None:
    result_path = tmp_path / "result.json"
    result_path.write_text(json.dumps(_result_from_mesh(demo_mesh)), encoding="utf-8")
    parts_path = tmp_path / "parts.json"
    parts_path.write_text(json.dumps(demo_mesh.face_part_ids.tolist()), encoding="utf-8")
    output_path = tmp_path / "prepared.json"
    summary = prepare_planner_input(
        result_path,
        parts_path,
        output_path,
        demo_mesh.camera_position.tolist(),
    )
    assert summary.pipeline.startswith("Human3D")
    assert summary.human3d_point_count == 2
    loaded = load_mesh_json(output_path)
    assert len(loaded.faces) == len(demo_mesh.faces)


def test_ai_contract_rejects_mismatched_labels(demo_mesh: MeshData) -> None:
    payload = _result_from_mesh(demo_mesh)
    payload["human3d_part_ids"] = [7]
    with pytest.raises(ValueError, match="same length"):
        validate_human3d_segfit_result(payload)
