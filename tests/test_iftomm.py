from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from pai_body_screening.collision import (
    DetectorGripperMeshModel,
    evaluate_detector_gripper_collisions,
)
from pai_body_screening.iftomm import IftommConfig, _lane_centres, generate_trajectory
from pai_body_screening.models import MeshData
from pai_body_screening.roi import TARGET_PART_IDS


def test_paper_raster_reaches_target_coverage(demo_mesh: MeshData) -> None:
    config = IftommConfig(target_coverage_ratio=0.90)
    result = generate_trajectory(demo_mesh, config)
    assert result.coverage_ratio >= 0.90
    assert result.stop_reason == "paper_raster_complete"
    assert len(result.control_points) >= 2
    assert len(result.smoothed_points) >= 2
    assert len(result.lane_reference_y_m) == 4
    assert np.all(np.diff(result.lane_reference_y_m) > 0.0)
    assert result.path_length_m > 0.0
    assert np.all(np.isfinite(result.smoothed_quaternions_wxyz))
    frames = Rotation.from_quat(
        result.smoothed_quaternions_wxyz[:, [1, 2, 3, 0]]
    ).as_matrix()
    inspection_alignment = np.einsum(
        "ij,ij->i", frames[:, :, 0], result.smoothed_normals
    )
    assert np.allclose(inspection_alignment, -1.0, atol=1e-8)
    orientation_steps_deg = np.rad2deg(
        (Rotation.from_matrix(frames[:-1]).inv() * Rotation.from_matrix(frames[1:])).magnitude()
    )
    assert np.max(orientation_steps_deg) <= config.maximum_orientation_step_deg + 0.1
    central_y = float(np.mean(result.lane_reference_y_m[1:3]))
    before_flip = result.smoothed_points[:, 1] < central_y - 0.02
    after_flip = result.smoothed_points[:, 1] > central_y + 0.02
    assert np.median(result.smoothed_scan_axes[before_flip, 2]) > 0.9
    assert np.median(result.smoothed_scan_axes[after_flip, 2]) < -0.9
    selected_parts = set(
        demo_mesh.face_part_ids[result.control_face_ids].astype(int).tolist()
    )
    assert selected_parts <= TARGET_PART_IDS


def test_four_lane_spacing_is_one_point_five_times_six_lane_reference() -> None:
    centroids = np.column_stack(
        (np.zeros(101), np.linspace(-0.4, 0.4, 101), np.zeros(101))
    )
    reference = _lane_centres(centroids, 6)
    widened = _lane_centres(centroids, 4)
    np.testing.assert_allclose(
        np.diff(widened).mean(),
        1.5 * np.diff(reference).mean(),
        rtol=0.0,
        atol=1e-12,
    )


def test_central_lane_gap_is_widened_without_changing_outer_intervals() -> None:
    centroids = np.column_stack(
        (np.zeros(101), np.linspace(-0.4, 0.4, 101), np.zeros(101))
    )
    lanes = _lane_centres(centroids, 4, central_gap_scale=1.2)
    np.testing.assert_allclose(
        lanes,
        [-0.3, -0.12, 0.12, 0.3],
        atol=1e-12,
    )
    np.testing.assert_allclose(
        np.diff(lanes),
        [0.18, 0.24, 0.18],
        atol=1e-12,
    )


def test_collision_is_evaluated_after_planning(
    demo_mesh: MeshData,
    collision_model: DetectorGripperMeshModel,
) -> None:
    result = generate_trajectory(
        demo_mesh,
        IftommConfig(target_coverage_ratio=0.80),
    )
    collisions = evaluate_detector_gripper_collisions(
        demo_mesh,
        result.smoothed_points,
        result.smoothed_quaternions_wxyz,
        collision_model,
    )
    assert collisions.pose_collision.shape == (len(result.smoothed_points),)
    assert collisions.collision_model == "md_gripper_triangle_mesh"
    # Collision post-processing cannot alter the planned path.
    assert result.coverage_ratio >= 0.80
