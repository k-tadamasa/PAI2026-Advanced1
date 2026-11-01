from __future__ import annotations

import numpy as np

from pai_body_screening.collision import DetectorBoxProxy, evaluate_collisions
from pai_body_screening.iftomm import IftommConfig, generate_trajectory
from pai_body_screening.models import MeshData
from pai_body_screening.roi import TARGET_PART_IDS


def test_publication_baseline_reaches_target_coverage(demo_mesh: MeshData) -> None:
    config = IftommConfig(target_coverage_ratio=0.95, maximum_control_points=500)
    result = generate_trajectory(demo_mesh, config)
    assert result.coverage_ratio >= 0.95
    assert result.stop_reason == "target_coverage_reached"
    assert len(result.control_points) >= 2
    assert len(result.smoothed_points) >= len(result.control_points)
    assert result.path_length_m > 0.0
    assert np.all(np.isfinite(result.smoothed_quaternions_wxyz))
    selected_parts = set(
        demo_mesh.face_part_ids[result.control_face_ids].astype(int).tolist()
    )
    assert selected_parts <= TARGET_PART_IDS


def test_collision_is_evaluated_after_planning(demo_mesh: MeshData) -> None:
    result = generate_trajectory(
        demo_mesh,
        IftommConfig(target_coverage_ratio=0.80, maximum_control_points=300),
    )
    conservative_proxy = DetectorBoxProxy(
        center_local_m=(0.0, 0.0, 0.0),
        half_extents_m=(0.080, 0.050, 0.050),
        margin_m=0.0,
    )
    collisions = evaluate_collisions(
        demo_mesh,
        result.smoothed_points,
        result.smoothed_normals,
        conservative_proxy,
    )
    assert collisions.pose_collision.shape == (len(result.smoothed_points),)
    assert np.any(collisions.pose_collision)
    # Collision post-processing cannot alter the planned path.
    assert result.coverage_ratio >= 0.80
