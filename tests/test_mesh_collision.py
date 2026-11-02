from pathlib import Path

import numpy as np

from pai_body_screening.collision import (
    DETECTOR_COMPONENT_ID,
    DetectorGripperMeshModel,
    _triangle_pair_intersections,
    evaluate_detector_gripper_collisions,
)
from pai_body_screening.models import MeshData


ASSET_ROOT = Path(__file__).resolve().parents[1] / "web/public/assets"


def test_triangle_intersection_rejects_disjoint_coplanar_faces() -> None:
    first = np.asarray([[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 2.0, 0.0]]])
    second = np.asarray([[[2.0, 2.0, 0.0], [1.1, 2.0, 0.0], [2.0, 1.1, 0.0]]])
    assert not _triangle_pair_intersections(first, second, 1.0e-9)[0]


def _human_mesh_from_component_triangle(triangle: np.ndarray) -> MeshData:
    return MeshData.from_arrays(
        vertices=triangle,
        faces=[[0, 1, 2]],
        face_part_ids=[2],
        camera_position=[-2.0, 0.0, 1.0],
    )


def test_real_detector_mesh_collision_reports_detector_component() -> None:
    model = DetectorGripperMeshModel.from_asset_root(ASSET_ROOT)
    components = model.component_triangles_in_control_frame()
    mesh = _human_mesh_from_component_triangle(components[DETECTOR_COMPONENT_ID][0])

    collision = evaluate_detector_gripper_collisions(
        mesh,
        points=[[0.0, 0.0, 0.0]],
        orientations_wxyz=[[1.0, 0.0, 0.0, 0.0]],
        model=model,
    )

    assert collision.collision_model == "md_gripper_triangle_mesh"
    assert collision.pose_collision.tolist() == [True]
    assert DETECTOR_COMPONENT_ID in collision.colliding_components[0]
    assert collision.colliding_face_ids == [[0]]


def test_real_gripper_mesh_collision_reports_exact_link() -> None:
    model = DetectorGripperMeshModel.from_asset_root(ASSET_ROOT)
    components = model.component_triangles_in_control_frame()
    component = "gripper_r_rh_p12_rn_r2"
    mesh = _human_mesh_from_component_triangle(components[component][0])

    collision = evaluate_detector_gripper_collisions(
        mesh,
        points=[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
        orientations_wxyz=[
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
        ],
        model=model,
    )

    assert component in collision.colliding_components[0]
    assert collision.component_collision[component].tolist() == [True, False]
    assert collision.pose_collision.tolist() == [True, False]
