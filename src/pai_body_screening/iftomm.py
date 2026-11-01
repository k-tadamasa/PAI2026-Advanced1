from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
from time import perf_counter

import numpy as np
from numpy.typing import NDArray
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation, Slerp

from .models import FloatArray, MeshData, face_geometry, surface_frame
from .roi import TARGET_PART_IDS, select_target_faces


@dataclass(frozen=True)
class IftommConfig:
    """Parameters reported by the attached IFToMM publication."""

    standoff_m: float = 0.055
    ellipsoid_radii_m: tuple[float, float, float] = (0.110, 0.111, 0.0906)
    coverage_weight: float = 1.0
    distance_weight: float = 0.5
    lateral_weight: float = 0.8
    reference_distance_m: float = 0.400
    target_coverage_ratio: float = 0.99
    sample_spacing_m: float = 0.010
    maximum_control_points: int = 800
    part_ids: frozenset[int] = TARGET_PART_IDS
    front_only: bool = True

    def validate(self) -> None:
        if self.standoff_m <= 0.0:
            raise ValueError("standoff_m must be positive")
        if min(self.ellipsoid_radii_m) <= 0.0:
            raise ValueError("ellipsoid radii must be positive")
        if self.reference_distance_m <= 0.0:
            raise ValueError("reference_distance_m must be positive")
        if not 0.0 < self.target_coverage_ratio <= 1.0:
            raise ValueError("target_coverage_ratio must be in (0, 1]")
        if self.maximum_control_points < 2:
            raise ValueError("maximum_control_points must be at least two")


@dataclass
class TrajectoryResult:
    target_face_mask: NDArray[np.bool_]
    inspected_face_mask: NDArray[np.bool_]
    control_points: FloatArray
    control_normals: FloatArray
    control_face_ids: NDArray[np.int64]
    smoothed_points: FloatArray
    smoothed_normals: FloatArray
    smoothed_quaternions_wxyz: FloatArray
    coverage_ratio: float
    target_area_m2: float
    inspected_area_m2: float
    path_length_m: float
    planning_time_s: float
    stop_reason: str
    selection_scores: list[float] = field(default_factory=list)


def _adjacency(faces: NDArray[np.int64], target_ids: NDArray[np.int64]) -> list[set[int]]:
    vertex_faces: dict[int, list[int]] = defaultdict(list)
    target_set = set(int(value) for value in target_ids)
    for face_id in target_ids:
        for vertex_id in faces[face_id]:
            vertex_faces[int(vertex_id)].append(int(face_id))
    neighbors = [set() for _ in range(len(faces))]
    for incident in vertex_faces.values():
        incident_set = set(incident) & target_set
        for face_id in incident_set:
            neighbors[face_id].update(incident_set - {face_id})
    return neighbors


def _covered_faces_at_pose(
    mesh: MeshData,
    face_ids: NDArray[np.int64],
    center: FloatArray,
    normal: FloatArray,
    radii: FloatArray,
) -> NDArray[np.bool_]:
    frame = surface_frame(normal)
    triangles = mesh.vertices[mesh.faces[face_ids]]
    local = (triangles - center[None, None, :]) @ frame
    normalized = local / radii[None, None, :]
    vertices_inside = np.sum(normalized * normalized, axis=2) <= 1.0 + 1e-12
    return np.all(vertices_inside, axis=1)


def _initial_face(
    target_ids: NDArray[np.int64], centroids: FloatArray
) -> int:
    # Deterministic top-right start, matching the published implementation's
    # upper-edge initialization while avoiding any body-specific vertex ID.
    order = np.lexsort((target_ids, -centroids[target_ids, 1], -centroids[target_ids, 2]))
    return int(target_ids[order[0]])


def _smooth_path(
    points: FloatArray,
    normals: FloatArray,
    spacing_m: float,
) -> tuple[FloatArray, FloatArray, FloatArray]:
    if len(points) == 1:
        frame = surface_frame(normals[0])
        quat_xyzw = Rotation.from_matrix(frame).as_quat()
        quat_wxyz = quat_xyzw[[3, 0, 1, 2]][None, :]
        return points.copy(), normals.copy(), quat_wxyz

    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    keep = np.concatenate(([True], distances > 1e-9))
    points = points[keep]
    normals = normals[keep]
    if len(points) == 1:
        return _smooth_path(points, normals, spacing_m)

    arc = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))))
    sample_count = max(2, int(np.ceil(arc[-1] / spacing_m)) + 1)
    sample_arc = np.linspace(0.0, arc[-1], sample_count)
    smoothed = np.column_stack(
        [CubicSpline(arc, points[:, axis])(sample_arc) for axis in range(3)]
    )

    frames = np.stack([surface_frame(normal) for normal in normals])
    rotations = Rotation.from_matrix(frames)
    interpolated_rotations = Slerp(arc, rotations)(sample_arc)
    interpolated_frames = interpolated_rotations.as_matrix()
    smooth_normals = interpolated_frames[:, :, 0]
    quat_xyzw = interpolated_rotations.as_quat()
    quat_wxyz = quat_xyzw[:, [3, 0, 1, 2]]
    return smoothed, smooth_normals, quat_wxyz


def generate_trajectory(
    mesh: MeshData,
    config: IftommConfig = IftommConfig(),
) -> TrajectoryResult:
    """Generate the publication-baseline coverage-frontier trajectory.

    Collision information is deliberately absent from the selection score.
    Collision is evaluated after planning so that it remains a visible future
    limitation, as stated in the IFToMM paper.
    """

    config.validate()
    started = perf_counter()
    target_mask = select_target_faces(
        mesh, config.part_ids, front_only=config.front_only
    )
    target_ids = np.flatnonzero(target_mask).astype(np.int64)
    centroids, normals, areas = face_geometry(mesh)

    # Make every selected normal point toward the camera.  This also protects
    # against an isolated reversed triangle without changing mesh topology.
    to_camera = mesh.camera_position[None, :] - centroids
    flip = np.einsum("ij,ij->i", normals, to_camera) < 0.0
    normals[flip] *= -1.0
    offset_points = centroids + config.standoff_m * normals
    adjacency = _adjacency(mesh.faces, target_ids)
    target_area = float(np.sum(areas[target_ids]))
    radii = np.asarray(config.ellipsoid_radii_m, dtype=np.float64)

    inspected = np.zeros(len(mesh.faces), dtype=bool)
    current_face = _initial_face(target_ids, centroids)
    control_face_ids: list[int] = []
    control_points: list[FloatArray] = []
    control_normals: list[FloatArray] = []
    scores: list[float] = []
    stop_reason = "maximum_control_points"

    for step in range(config.maximum_control_points):
        current_point = offset_points[current_face]
        current_normal = normals[current_face]
        control_face_ids.append(current_face)
        control_points.append(current_point.copy())
        control_normals.append(current_normal.copy())

        covered_here = _covered_faces_at_pose(
            mesh, target_ids, current_point, current_normal, radii
        )
        inspected[target_ids[covered_here]] = True
        inspected_area = float(np.sum(areas[target_mask & inspected]))
        if inspected_area / target_area >= config.target_coverage_ratio:
            stop_reason = "target_coverage_reached"
            break

        covered_target_ids = target_ids[inspected[target_ids]]
        frontier: set[int] = set()
        for face_id in covered_target_ids:
            frontier.update(
                neighbor
                for neighbor in adjacency[int(face_id)]
                if target_mask[neighbor] and not inspected[neighbor]
            )

        if not frontier:
            # A disconnected target component is resumed from its nearest
            # uninspected face.  The transition remains visible in the output.
            remaining = target_ids[~inspected[target_ids]]
            if len(remaining) == 0:
                stop_reason = "all_target_faces_covered"
                break
            distances = np.linalg.norm(offset_points[remaining] - current_point, axis=1)
            frontier = {int(remaining[int(np.argmin(distances))])}

        best_face: int | None = None
        best_score = -np.inf
        for candidate in sorted(frontier):
            candidate_point = offset_points[candidate]
            candidate_normal = normals[candidate]
            covered_candidate = _covered_faces_at_pose(
                mesh, target_ids, candidate_point, candidate_normal, radii
            )
            newly_covered_ids = target_ids[covered_candidate & ~inspected[target_ids]]
            new_area = float(np.sum(areas[newly_covered_ids]))
            displacement = candidate_point - current_point
            distance = float(np.linalg.norm(displacement))
            lateral = abs(float(displacement[1]))
            score = (
                config.coverage_weight * new_area / target_area
                - config.distance_weight * distance / config.reference_distance_m
                - config.lateral_weight * lateral / config.reference_distance_m
            )
            if score > best_score + 1e-15:
                best_face = candidate
                best_score = score

        if best_face is None:
            stop_reason = "no_candidate"
            break
        current_face = best_face
        scores.append(float(best_score))

    points_array = np.asarray(control_points, dtype=np.float64)
    normals_array = np.asarray(control_normals, dtype=np.float64)
    smooth_points, smooth_normals, smooth_quaternions = _smooth_path(
        points_array, normals_array, config.sample_spacing_m
    )
    inspected_area = float(np.sum(areas[target_mask & inspected]))
    path_length = float(np.sum(np.linalg.norm(np.diff(smooth_points, axis=0), axis=1)))
    return TrajectoryResult(
        target_face_mask=target_mask,
        inspected_face_mask=inspected,
        control_points=points_array,
        control_normals=normals_array,
        control_face_ids=np.asarray(control_face_ids, dtype=np.int64),
        smoothed_points=smooth_points,
        smoothed_normals=smooth_normals,
        smoothed_quaternions_wxyz=smooth_quaternions,
        coverage_ratio=inspected_area / target_area,
        target_area_m2=target_area,
        inspected_area_m2=inspected_area,
        path_length_m=path_length,
        planning_time_s=perf_counter() - started,
        stop_reason=stop_reason,
        selection_scores=scores,
    )
