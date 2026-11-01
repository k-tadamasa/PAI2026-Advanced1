from __future__ import annotations

import numpy as np
import pytest

from pai_body_screening.models import MeshData
from pai_body_screening.roi import TARGET_PART_IDS, select_target_faces


def test_roi_contains_only_requested_parts(demo_mesh: MeshData) -> None:
    mask = select_target_faces(demo_mesh)
    selected_parts = set(demo_mesh.face_part_ids[mask].astype(int).tolist())
    assert selected_parts == TARGET_PART_IDS
    assert 4 not in selected_parts


def test_roi_never_falls_back_to_full_body(demo_mesh: MeshData) -> None:
    with pytest.raises(ValueError, match="ROI is empty"):
        select_target_faces(demo_mesh, {999})


def test_back_facing_mesh_is_rejected(demo_mesh: MeshData) -> None:
    reversed_mesh = MeshData.from_arrays(
        demo_mesh.vertices,
        demo_mesh.faces[:, ::-1],
        demo_mesh.face_part_ids,
        demo_mesh.camera_position,
    )
    with pytest.raises(ValueError, match="ROI is empty"):
        select_target_faces(reversed_mesh)
