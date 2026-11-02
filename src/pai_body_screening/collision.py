from __future__ import annotations

import struct
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.spatial.transform import Rotation

from .models import FloatArray, MeshData


DETECTOR_COMPONENT_ID = "metal_detector_link"
GRIPPER_COMPONENT_IDS = (
    "gripper_r_rh_p12_rn_base",
    "gripper_r_rh_p12_rn_r1",
    "gripper_r_rh_p12_rn_r2",
    "gripper_r_rh_p12_rn_l1",
    "gripper_r_rh_p12_rn_l2",
)
GRIPPER_OPEN_80_PERCENT_RAD = 0.30585790515379546

_BINARY_STL_DTYPE = np.dtype(
    [
        ("normal", "<f4", (3,)),
        ("vertices", "<f4", (3, 3)),
        ("attribute", "<u2"),
    ]
)


@dataclass
class CollisionResult:
    pose_collision: NDArray[np.bool_]
    colliding_face_ids: list[list[int]]
    colliding_components: list[list[str]] = field(default_factory=list)
    component_collision: dict[str, NDArray[np.bool_]] = field(default_factory=dict)
    collision_model: str = "md_gripper_triangle_mesh"


@dataclass(frozen=True)
class DetectorGripperMeshModel:
    """Public MD and right-gripper triangle meshes in their link frames."""

    detector_triangles_m: FloatArray
    gripper_triangles_m: dict[str, FloatArray]

    @classmethod
    def from_asset_root(cls, asset_root: str | Path) -> "DetectorGripperMeshModel":
        root = Path(asset_root)
        gripper_root = (
            root
            / "robot/ffw_description/meshes/common/rh_p12_rn_a"
        )
        filenames = {
            "gripper_r_rh_p12_rn_base": "base.stl",
            "gripper_r_rh_p12_rn_r1": "r1.stl",
            "gripper_r_rh_p12_rn_r2": "r2.stl",
            "gripper_r_rh_p12_rn_l1": "l1.stl",
            "gripper_r_rh_p12_rn_l2": "l2.stl",
        }
        return cls(
            detector_triangles_m=_load_binary_stl(
                root / "detector/MD_no_hole_poly.stl"
            ),
            gripper_triangles_m={
                component: _load_binary_stl(gripper_root / filename, scale=0.001)
                for component, filename in filenames.items()
            },
        )

    def component_triangles_in_control_frame(
        self,
        gripper_joint_rad: float = GRIPPER_OPEN_80_PERCENT_RAD,
    ) -> dict[str, FloatArray]:
        """Return exact collision triangles relative to the MD control frame."""

        detector_from_arm = _transform(
            translation=(0.0, 0.0, -0.220551),
            rotation=Rotation.from_euler("x", np.pi).as_matrix(),
        )
        control_from_detector = _transform(
            translation=(0.205701, 0.0, 0.0),
            rotation=Rotation.from_euler(
                "xyz", (-0.5 * np.pi, -0.5 * np.pi, 0.0)
            ).as_matrix(),
        )
        control_from_arm = detector_from_arm @ control_from_detector
        arm_from_control = np.linalg.inv(control_from_arm)

        detector_mesh_from_detector = _transform(
            translation=(-0.075000, 0.044794, -0.017715),
            rotation=Rotation.from_euler("z", -0.5 * np.pi).as_matrix(),
        )
        components = {
            DETECTOR_COMPONENT_ID: _apply_transform(
                self.detector_triangles_m,
                np.linalg.inv(control_from_detector) @ detector_mesh_from_detector,
            )
        }

        gripper_base_from_arm = _transform(
            translation=(0.0, 0.0, -0.078),
            rotation=Rotation.from_euler("xyz", (0.0, np.pi, np.pi)).as_matrix(),
        )
        joint = float(gripper_joint_rad)
        link_from_arm: dict[str, FloatArray] = {
            "gripper_r_rh_p12_rn_base": gripper_base_from_arm,
        }
        link_from_arm["gripper_r_rh_p12_rn_r1"] = (
            gripper_base_from_arm
            @ _transform(
                translation=(0.0, 0.008, 0.048),
                rotation=Rotation.from_rotvec((joint, 0.0, 0.0)).as_matrix(),
            )
        )
        link_from_arm["gripper_r_rh_p12_rn_r2"] = (
            link_from_arm["gripper_r_rh_p12_rn_r1"]
            @ _transform(
                translation=(0.0, 0.0493634, 0.0285),
                rotation=Rotation.from_rotvec((-joint, 0.0, 0.0)).as_matrix(),
            )
        )
        link_from_arm["gripper_r_rh_p12_rn_l1"] = (
            gripper_base_from_arm
            @ _transform(
                translation=(0.0, -0.008, 0.048),
                rotation=Rotation.from_rotvec((-joint, 0.0, 0.0)).as_matrix(),
            )
        )
        link_from_arm["gripper_r_rh_p12_rn_l2"] = (
            link_from_arm["gripper_r_rh_p12_rn_l1"]
            @ _transform(
                translation=(0.0, -0.0493634, 0.0285),
                rotation=Rotation.from_rotvec((joint, 0.0, 0.0)).as_matrix(),
            )
        )
        for component in GRIPPER_COMPONENT_IDS:
            components[component] = _apply_transform(
                self.gripper_triangles_m[component],
                arm_from_control @ link_from_arm[component],
            )
        return components


def _load_binary_stl(path: Path, *, scale: float = 1.0) -> FloatArray:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"binary STL is truncated: {path}")
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + triangle_count * _BINARY_STL_DTYPE.itemsize
    if len(data) != expected_size:
        raise ValueError(f"expected a binary STL with {triangle_count} faces: {path}")
    records = np.frombuffer(
        data,
        dtype=_BINARY_STL_DTYPE,
        count=triangle_count,
        offset=84,
    )
    return np.asarray(records["vertices"], dtype=np.float64) * float(scale)


def _transform(
    *,
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rotation: FloatArray | None = None,
) -> FloatArray:
    value = np.eye(4, dtype=np.float64)
    value[:3, 3] = np.asarray(translation, dtype=np.float64)
    if rotation is not None:
        value[:3, :3] = rotation
    return value


def _apply_transform(triangles: FloatArray, transform: FloatArray) -> FloatArray:
    return triangles @ transform[:3, :3].T + transform[:3, 3]


def _triangle_pair_intersections(
    first: FloatArray,
    second: FloatArray,
    tolerance_m: float,
) -> NDArray[np.bool_]:
    """Vectorized separating-axis test for paired 3-D triangles."""

    if len(first) == 0:
        return np.zeros(0, dtype=bool)
    first_edges = np.stack(
        (
            first[:, 1] - first[:, 0],
            first[:, 2] - first[:, 1],
            first[:, 0] - first[:, 2],
        ),
        axis=1,
    )
    second_edges = np.stack(
        (
            second[:, 1] - second[:, 0],
            second[:, 2] - second[:, 1],
            second[:, 0] - second[:, 2],
        ),
        axis=1,
    )
    first_normals = np.cross(first_edges[:, 0], first_edges[:, 1])
    second_normals = np.cross(second_edges[:, 0], second_edges[:, 1])
    edge_crosses = np.cross(
        first_edges[:, :, None, :], second_edges[:, None, :, :]
    ).reshape(len(first), 9, 3)
    all_edges = np.concatenate((first_edges, second_edges), axis=1)
    first_planar_axes = np.cross(first_normals[:, None, :], all_edges)
    second_planar_axes = np.cross(second_normals[:, None, :], all_edges)
    axes = np.concatenate(
        (
            first_normals[:, None, :],
            second_normals[:, None, :],
            edge_crosses,
            first_planar_axes,
            second_planar_axes,
        ),
        axis=1,
    )
    lengths = np.linalg.norm(axes, axis=2)
    valid = lengths > 1e-12
    normalized = axes / np.where(valid, lengths, 1.0)[:, :, None]
    first_projection = np.einsum("pvc,pac->pva", first, normalized)
    second_projection = np.einsum("pvc,pac->pva", second, normalized)
    first_before_second = np.max(first_projection, axis=1) < (
        np.min(second_projection, axis=1) - tolerance_m
    )
    second_before_first = np.max(second_projection, axis=1) < (
        np.min(first_projection, axis=1) - tolerance_m
    )
    separated = valid & (first_before_second | second_before_first)
    return ~np.any(separated, axis=1)


def _mesh_collision_face_ids(
    human_triangles: FloatArray,
    human_minimum: FloatArray,
    human_maximum: FloatArray,
    component_triangles: FloatArray,
    tolerance_m: float,
) -> list[int]:
    component_minimum = np.min(component_triangles, axis=(0, 1))
    component_maximum = np.max(component_triangles, axis=(0, 1))
    candidate_ids = np.flatnonzero(
        np.all(human_maximum >= component_minimum - tolerance_m, axis=1)
        & np.all(human_minimum <= component_maximum + tolerance_m, axis=1)
    )
    if not len(candidate_ids):
        return []

    component_face_minimum = np.min(component_triangles, axis=1)
    component_face_maximum = np.max(component_triangles, axis=1)
    candidate_minimum = human_minimum[candidate_ids]
    candidate_maximum = human_maximum[candidate_ids]
    broad_phase = np.all(
        component_face_maximum[:, None, :] >= candidate_minimum[None, :, :] - tolerance_m,
        axis=2,
    ) & np.all(
        component_face_minimum[:, None, :] <= candidate_maximum[None, :, :] + tolerance_m,
        axis=2,
    )
    component_ids, local_human_ids = np.nonzero(broad_phase)
    if not len(component_ids):
        return []
    intersections = _triangle_pair_intersections(
        component_triangles[component_ids],
        human_triangles[candidate_ids[local_human_ids]],
        tolerance_m,
    )
    return (
        np.unique(candidate_ids[local_human_ids[intersections]])
        .astype(int)
        .tolist()
    )


def evaluate_detector_gripper_collisions(
    mesh: MeshData,
    points: ArrayLike,
    orientations_wxyz: ArrayLike,
    model: DetectorGripperMeshModel,
    *,
    gripper_joint_rad: float = GRIPPER_OPEN_80_PERCENT_RAD,
    contact_tolerance_m: float = 1.0e-6,
) -> CollisionResult:
    """Intersect the real MD/gripper triangle meshes with the human mesh."""

    positions = np.asarray(points, dtype=np.float64)
    quaternions = np.asarray(orientations_wxyz, dtype=np.float64)
    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if quaternions.shape != (len(positions), 4):
        raise ValueError("orientations_wxyz must have shape (N, 4)")
    if contact_tolerance_m < 0.0:
        raise ValueError("contact_tolerance_m must be non-negative")

    frames = Rotation.from_quat(quaternions[:, [1, 2, 3, 0]]).as_matrix()
    human_triangles = mesh.vertices[mesh.faces]
    human_minimum = np.min(human_triangles, axis=1)
    human_maximum = np.max(human_triangles, axis=1)
    local_components = model.component_triangles_in_control_frame(gripper_joint_rad)
    component_collision = {
        component: np.zeros(len(positions), dtype=bool)
        for component in local_components
    }
    pose_collision = np.zeros(len(positions), dtype=bool)
    colliding_face_ids: list[list[int]] = []
    colliding_components: list[list[str]] = []

    for pose_index, (position, rotation) in enumerate(
        zip(positions, frames, strict=True)
    ):
        pose_faces: set[int] = set()
        pose_components: list[str] = []
        for component, local_triangles in local_components.items():
            world_triangles = local_triangles @ rotation.T + position
            hit_ids = _mesh_collision_face_ids(
                human_triangles,
                human_minimum,
                human_maximum,
                world_triangles,
                contact_tolerance_m,
            )
            if hit_ids:
                component_collision[component][pose_index] = True
                pose_components.append(component)
                pose_faces.update(hit_ids)
        pose_collision[pose_index] = bool(pose_components)
        colliding_face_ids.append(sorted(pose_faces))
        colliding_components.append(pose_components)

    return CollisionResult(
        pose_collision=pose_collision,
        colliding_face_ids=colliding_face_ids,
        colliding_components=colliding_components,
        component_collision=component_collision,
        collision_model="md_gripper_triangle_mesh",
    )
