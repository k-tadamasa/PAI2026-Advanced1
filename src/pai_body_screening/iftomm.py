from __future__ import annotations

from dataclasses import dataclass, field
from time import perf_counter

import numpy as np
from numpy.typing import NDArray
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

from .models import FloatArray, MeshData, face_geometry, surface_frame
from .roi import (
    FRONT_NORMAL_MAX_ANGLE_DEG,
    FRONT_TOP_MARGIN_M,
    FRONT_WIDTH_FRACTION,
    TARGET_PART_IDS,
    select_target_faces,
)


@dataclass(frozen=True)
class IftommConfig:
    """Parameters of Kitahara et al.'s body-surface raster trajectory.

    The paper used six fixed ``y_ref`` values for its 0.6 m wide point-cloud
    ROI. This assignment uses four lines in the same selected ROI. The two
    outer intervals retain the four-bin spacing while the central interval is
    widened to keep the two inner lines farther apart.
    """

    standoff_m: float = 0.055
    ellipsoid_radii_m: tuple[float, float, float] = (0.110, 0.111, 0.0906)
    # Six SI-style lanes are the 1.0x reference. Four equal-bin lanes over the
    # same ROI provide the outer-spacing baseline; the central gap is widened
    # independently below.
    lane_count: int = 4
    lane_spacing_scale: float = 1.5
    central_lane_gap_scale: float = 1.2
    flip_md_at_center: bool = True
    lane_half_width_m: float = 0.002
    local_fit_half_window: int = 5
    interpolation_gap_m: float = 0.002
    loess_window_points: int = 50
    target_coverage_ratio: float = 0.95
    sample_spacing_m: float = 0.010
    maximum_orientation_step_deg: float = 1.5
    part_ids: frozenset[int] = TARGET_PART_IDS
    front_only: bool = True
    front_normal_max_angle_deg: float = FRONT_NORMAL_MAX_ANGLE_DEG
    front_width_fraction: float = FRONT_WIDTH_FRACTION
    front_top_margin_m: float = FRONT_TOP_MARGIN_M

    def validate(self) -> None:
        if self.standoff_m <= 0.0:
            raise ValueError("standoff_m must be positive")
        if min(self.ellipsoid_radii_m) <= 0.0:
            raise ValueError("ellipsoid radii must be positive")
        if self.lane_count < 2:
            raise ValueError("lane_count must be at least two")
        if self.lane_spacing_scale <= 0.0:
            raise ValueError("lane_spacing_scale must be positive")
        if not 1.0 <= self.central_lane_gap_scale < self.lane_count - 1:
            raise ValueError(
                "central_lane_gap_scale must be in [1, lane_count - 1)"
            )
        if self.central_lane_gap_scale != 1.0 and self.lane_count < 4:
            raise ValueError("a widened central lane gap requires at least four lanes")
        if self.central_lane_gap_scale != 1.0 and self.lane_count % 2 != 0:
            raise ValueError("a widened central lane gap requires an even lane_count")
        if self.flip_md_at_center and self.lane_count % 2 != 0:
            raise ValueError("the central MD flip requires an even lane_count")
        if self.lane_half_width_m <= 0.0:
            raise ValueError("lane_half_width_m must be positive")
        if self.local_fit_half_window < 1:
            raise ValueError("local_fit_half_window must be positive")
        if self.interpolation_gap_m <= 0.0:
            raise ValueError("interpolation_gap_m must be positive")
        if self.loess_window_points < 3:
            raise ValueError("loess_window_points must be at least three")
        if not 0.0 < self.target_coverage_ratio <= 1.0:
            raise ValueError("target_coverage_ratio must be in (0, 1]")
        if self.sample_spacing_m <= 0.0:
            raise ValueError("sample_spacing_m must be positive")
        if self.maximum_orientation_step_deg <= 0.0:
            raise ValueError("maximum_orientation_step_deg must be positive")
        if not 0.0 < self.front_normal_max_angle_deg < 90.0:
            raise ValueError("front_normal_max_angle_deg must be in (0, 90)")
        if not 0.0 < self.front_width_fraction <= 1.0:
            raise ValueError("front_width_fraction must be in (0, 1]")
        if self.front_top_margin_m < 0.0:
            raise ValueError("front_top_margin_m must be non-negative")


@dataclass
class TrajectoryResult:
    target_face_mask: NDArray[np.bool_]
    inspected_face_mask: NDArray[np.bool_]
    control_points: FloatArray
    control_normals: FloatArray
    control_scan_axes: FloatArray
    control_face_ids: NDArray[np.int64]
    lane_reference_y_m: FloatArray
    smoothed_points: FloatArray
    smoothed_normals: FloatArray
    smoothed_scan_axes: FloatArray
    smoothed_quaternions_wxyz: FloatArray
    coverage_ratio: float
    target_area_m2: float
    inspected_area_m2: float
    path_length_m: float
    planning_time_s: float
    stop_reason: str
    selection_scores: list[float] = field(default_factory=list)


def _covered_faces_at_pose(
    mesh: MeshData,
    face_ids: NDArray[np.int64],
    center: FloatArray,
    normal: FloatArray,
    radii: FloatArray,
    scan_direction: FloatArray | None = None,
) -> NDArray[np.bool_]:
    frame = surface_frame(normal, scan_direction)
    centroids = np.mean(mesh.vertices[mesh.faces[face_ids]], axis=1)
    local = (centroids - center[None, :]) @ frame
    normalized = local / radii[None, :]
    return np.sum(normalized * normalized, axis=1) <= 1.0 + 1e-12


def covered_face_ids_at_pose(
    mesh: MeshData,
    face_ids: NDArray[np.int64],
    center: FloatArray,
    normal: FloatArray,
    radii: FloatArray,
    scan_direction: FloatArray | None = None,
) -> NDArray[np.int64]:
    """Return faces whose representative point is in the detector ellipsoid.

    The paper evaluates inspected point-cloud samples.  One triangle centroid
    is therefore used as the corresponding public mesh sample.
    """

    covered = _covered_faces_at_pose(
        mesh,
        face_ids,
        center,
        normal,
        radii,
        scan_direction,
    )
    return face_ids[covered]


def _normalise_rows(values: FloatArray, fallback: FloatArray) -> FloatArray:
    result = np.asarray(values, dtype=np.float64).copy()
    lengths = np.linalg.norm(result, axis=1)
    invalid = lengths <= 1e-12
    result[invalid] = fallback
    lengths = np.linalg.norm(result, axis=1)
    return result / np.maximum(lengths[:, None], 1e-12)


def _loess(values: FloatArray, window: int) -> FloatArray:
    """MATLAB ``smoothdata(..., 'loess', window)`` compatible local fit."""

    source = np.asarray(values, dtype=np.float64)
    one_dimensional = source.ndim == 1
    if one_dimensional:
        source = source[:, None]
    count = len(source)
    if count < 3:
        return values.copy()
    width = min(max(3, int(window)), count)
    result = np.empty_like(source)
    indices = np.arange(count, dtype=np.float64)
    for index in range(count):
        start = max(0, min(index - width // 2, count - width))
        stop = start + width
        local_x = indices[start:stop] - float(index)
        scale = float(np.max(np.abs(local_x)))
        if scale <= 1e-12:
            result[index] = source[index]
            continue
        distance = np.clip(np.abs(local_x) / scale, 0.0, 1.0)
        weights = (1.0 - distance**3) ** 3
        design = np.column_stack((np.ones(width), local_x))
        weighted_design = design * np.sqrt(weights[:, None])
        weighted_values = source[start:stop] * np.sqrt(weights[:, None])
        coefficients, *_ = np.linalg.lstsq(
            weighted_design,
            weighted_values,
            rcond=None,
        )
        result[index] = coefficients[0]
    return result[:, 0] if one_dimensional else result


def _lane_centres(
    target_centroids: FloatArray,
    lane_count: int,
    central_gap_scale: float = 1.0,
) -> FloatArray:
    if lane_count < 2:
        raise ValueError("lane_count must be at least two")
    if not 1.0 <= central_gap_scale < lane_count - 1:
        raise ValueError("central_gap_scale must be in [1, lane_count - 1)")
    if central_gap_scale != 1.0 and lane_count < 4:
        raise ValueError("a widened central lane gap requires at least four lanes")
    lower = float(np.min(target_centroids[:, 1]))
    upper = float(np.max(target_centroids[:, 1]))
    width = upper - lower
    base_spacing = width / lane_count
    first_center = lower + 0.5 * base_spacing
    last_center = upper - 0.5 * base_spacing
    gaps = np.full(lane_count - 1, base_spacing, dtype=np.float64)
    if central_gap_scale != 1.0:
        if lane_count % 2 != 0:
            raise ValueError("a widened central lane gap requires an even lane_count")
        central_index = lane_count // 2 - 1
        central_gap = central_gap_scale * base_spacing
        outer_gap = (
            (last_center - first_center) - central_gap
        ) / (lane_count - 2)
        gaps.fill(outer_gap)
        gaps[central_index] = central_gap
    return first_center + np.r_[0.0, np.cumsum(gaps)]


def _local_lane_offsets(
    mesh: MeshData,
    target_ids: NDArray[np.int64],
    centroids: FloatArray,
    normals: FloatArray,
    lane_y: float,
    config: IftommConfig,
) -> FloatArray:
    """Build one paper-style constant-y lane by local x-z line fitting."""

    triangles = mesh.vertices[mesh.faces[target_ids]]
    crossing = (np.min(triangles[:, :, 1], axis=1) <= lane_y) & (
        np.max(triangles[:, :, 1], axis=1) >= lane_y
    )
    crossing_ids = target_ids[crossing]
    reference_points: list[FloatArray] = []
    reference_face_ids: list[int] = []
    for face_id in crossing_ids:
        triangle = mesh.vertices[mesh.faces[face_id]]
        intersections: list[FloatArray] = []
        for first, second in ((0, 1), (1, 2), (2, 0)):
            start = triangle[first]
            end = triangle[second]
            start_delta = float(start[1] - lane_y)
            end_delta = float(end[1] - lane_y)
            if abs(start_delta) <= 1e-12:
                intersections.append(start.copy())
            if start_delta * end_delta < 0.0:
                fraction = -start_delta / (end_delta - start_delta)
                intersections.append(start + fraction * (end - start))
        if not intersections:
            continue
        point = np.mean(np.asarray(intersections, dtype=np.float64), axis=0)
        point[1] = lane_y
        reference_points.append(point)
        reference_face_ids.append(int(face_id))

    required = 2 * config.local_fit_half_window + 3
    if len(reference_points) < required:
        # Fallback for coarse synthetic meshes: the paper's +/-2 mm strip,
        # with nearest-y expansion only if the strip remains too sparse.
        target_centroids = centroids[target_ids]
        lateral_distance = np.abs(target_centroids[:, 1] - lane_y)
        selected_local = np.flatnonzero(
            lateral_distance <= config.lane_half_width_m
        )
        if len(selected_local) < required:
            selected_local = np.argsort(lateral_distance)[
                : min(required, len(target_ids))
            ]
        reference_points = [
            np.asarray(
                [centroids[face_id, 0], lane_y, centroids[face_id, 2]],
                dtype=np.float64,
            )
            for face_id in target_ids[selected_local]
        ]
        reference_face_ids = target_ids[selected_local].astype(int).tolist()

    points = np.asarray(reference_points, dtype=np.float64)
    selected_ids = np.asarray(reference_face_ids, dtype=np.int64)
    order = np.argsort(points[:, 2], kind="stable")
    points = points[order]
    selected_ids = selected_ids[order]

    half_window = min(
        config.local_fit_half_window,
        max(1, (len(points) - 1) // 2),
    )
    lane: list[FloatArray] = []
    for index in range(half_window, len(points) - half_window):
        local_ids = selected_ids[index - half_window : index + half_window + 1]
        xz = centroids[local_ids][:, [0, 2]]
        xz -= np.mean(xz, axis=0)
        covariance = xz.T @ xz
        _, vectors = np.linalg.eigh(covariance)
        tangent_xz = vectors[:, -1]
        if tangent_xz[1] < 0.0:
            tangent_xz *= -1.0
        outward = np.asarray([-tangent_xz[1], 0.0, tangent_xz[0]])
        mean_normal = np.mean(normals[local_ids], axis=0)
        if float(outward @ mean_normal) < 0.0:
            outward *= -1.0
        outward /= np.linalg.norm(outward)
        lane.append(points[index] + config.standoff_m * outward)
    if len(lane) < 2:
        raise ValueError(f"lane at y={lane_y:.4f} m has too few surface samples")
    lane_points = np.asarray(lane, dtype=np.float64)
    return lane_points[np.argsort(lane_points[:, 2], kind="stable")]


def _interpolate_polyline(points: FloatArray, maximum_gap_m: float) -> FloatArray:
    dense = [points[0]]
    for start, end in zip(points[:-1], points[1:], strict=True):
        distance = float(np.linalg.norm(end - start))
        steps = max(1, int(np.ceil(distance / maximum_gap_m)))
        for fraction in np.linspace(0.0, 1.0, steps + 1)[1:]:
            dense.append((1.0 - fraction) * start + fraction * end)
    return np.asarray(dense, dtype=np.float64)


def _paper_control_path(
    mesh: MeshData,
    target_ids: NDArray[np.int64],
    centroids: FloatArray,
    normals: FloatArray,
    config: IftommConfig,
) -> tuple[FloatArray, FloatArray, NDArray[np.int64], FloatArray]:
    lane_centres = _lane_centres(
        centroids[target_ids],
        config.lane_count,
        config.central_lane_gap_scale,
    )
    ordered_lanes: list[FloatArray] = []
    for lane_index, lane_y in enumerate(lane_centres):
        lane = _local_lane_offsets(
            mesh,
            target_ids,
            centroids,
            normals,
            float(lane_y),
            config,
        )
        # Figure 6 starts at the upper-right and alternates down/up.
        if lane_index % 2 == 0:
            lane = lane[::-1]
        ordered_lanes.append(lane)
    desired = _interpolate_polyline(
        np.vstack(ordered_lanes),
        config.interpolation_gap_m,
    )

    target_vertex_ids = np.unique(mesh.faces[target_ids].reshape(-1))
    surface_points = np.vstack((mesh.vertices[target_vertex_ids], centroids[target_ids]))
    surface_tree = cKDTree(surface_points)
    _, nearest_indices = surface_tree.query(desired, k=1)
    nearest = surface_points[np.asarray(nearest_indices, dtype=np.int64)]
    outward = desired - nearest
    lengths = np.linalg.norm(outward, axis=1)

    face_tree = cKDTree(centroids[target_ids])
    _, nearest_face_local = face_tree.query(nearest, k=1)
    control_face_ids = target_ids[np.asarray(nearest_face_local, dtype=np.int64)]
    fallback = normals[control_face_ids]
    invalid = lengths <= 1e-12
    outward[invalid] = fallback[invalid]
    outward = _normalise_rows(outward, np.asarray([-1.0, 0.0, 0.0]))
    modified = nearest + config.standoff_m * outward
    return modified, outward, control_face_ids, lane_centres


def _resample_path(
    points: FloatArray,
    normals: FloatArray,
    spacing_m: float,
    maximum_orientation_step_deg: float,
    central_flip_y_m: float | None = None,
) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray]:
    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    keep = np.concatenate(([True], distances > 1e-9))
    points = points[keep]
    normals = normals[keep]
    arc = np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
    )
    sample_count = max(2, int(np.ceil(float(arc[-1]) / spacing_m)) + 1)
    sample_arc = np.linspace(0.0, float(arc[-1]), sample_count)
    sampled_points = np.column_stack(
        [CubicSpline(arc, points[:, axis])(sample_arc) for axis in range(3)]
    )
    sampled_normals = np.column_stack(
        [np.interp(sample_arc, arc, normals[:, axis]) for axis in range(3)]
    )
    sampled_normals = _normalise_rows(
        sampled_normals,
        np.asarray([-1.0, 0.0, 0.0]),
    )
    frames = np.stack(
        [
            surface_frame(
                normal,
                np.asarray(
                    [
                        0.0,
                        0.0,
                        -1.0
                        if central_flip_y_m is not None
                        and point[1] > central_flip_y_m
                        else 1.0,
                    ]
                ),
            )
            for point, normal in zip(sampled_points, sampled_normals, strict=True)
        ]
    )
    rotations = Rotation.from_matrix(frames)
    maximum_step_rad = np.deg2rad(maximum_orientation_step_deg)
    dense_points = [sampled_points[0]]
    dense_rotations = [rotations[0]]
    for index in range(len(sampled_points) - 1):
        angle = float((rotations[index].inv() * rotations[index + 1]).magnitude())
        translation = float(
            np.linalg.norm(sampled_points[index + 1] - sampled_points[index])
        )
        steps = max(
            1,
            int(np.ceil(angle / maximum_step_rad)),
            int(np.ceil(translation / spacing_m)),
        )
        fractions = np.linspace(0.0, 1.0, steps + 1)[1:]
        pair = Rotation.from_quat(
            np.vstack((rotations[index].as_quat(), rotations[index + 1].as_quat()))
        )
        interpolated = Slerp([0.0, 1.0], pair)(fractions)
        for fraction, rotation in zip(fractions, interpolated, strict=True):
            dense_points.append(
                (1.0 - fraction) * sampled_points[index]
                + fraction * sampled_points[index + 1]
            )
            dense_rotations.append(rotation)

    final_points = np.asarray(dense_points, dtype=np.float64)
    final_rotations = Rotation.from_quat(
        np.vstack([rotation.as_quat() for rotation in dense_rotations])
    )
    final_frames = final_rotations.as_matrix()
    final_normals = -final_frames[:, :, 0]
    final_scan_axes = final_frames[:, :, 2]
    quaternion_xyzw = final_rotations.as_quat()
    quaternion_wxyz = quaternion_xyzw[:, [3, 0, 1, 2]]
    for index in range(1, len(quaternion_wxyz)):
        if float(quaternion_wxyz[index - 1] @ quaternion_wxyz[index]) < 0.0:
            quaternion_wxyz[index] *= -1.0
    return final_points, final_normals, final_scan_axes, quaternion_wxyz


def generate_trajectory(
    mesh: MeshData,
    config: IftommConfig = IftommConfig(),
) -> TrajectoryResult:
    """Generate the six-column body-surface trajectory from the SI paper.

    Collision information is deliberately absent from planning. It is
    evaluated after trajectory generation and displayed as future work.
    """

    config.validate()
    started = perf_counter()
    target_mask = select_target_faces(
        mesh,
        config.part_ids,
        front_only=config.front_only,
        front_normal_max_angle_deg=config.front_normal_max_angle_deg,
        front_width_fraction=config.front_width_fraction,
        front_top_margin_m=config.front_top_margin_m,
    )
    target_ids = np.flatnonzero(target_mask).astype(np.int64)
    if len(target_ids) == 0:
        raise ValueError("target ROI contains no faces")
    centroids, normals, areas = face_geometry(mesh)
    to_camera = mesh.camera_position[None, :] - centroids
    flip = np.einsum("ij,ij->i", normals, to_camera) < 0.0
    normals[flip] *= -1.0

    control_points, control_normals, control_face_ids, lane_centres = (
        _paper_control_path(
            mesh,
            target_ids,
            centroids,
            normals,
            config,
        )
    )
    loess_points = _loess(control_points, config.loess_window_points)
    loess_normals = _normalise_rows(
        _loess(control_normals, config.loess_window_points),
        np.asarray([-1.0, 0.0, 0.0]),
    )
    central_flip_y_m = (
        float(
            0.5
            * (
                lane_centres[config.lane_count // 2 - 1]
                + lane_centres[config.lane_count // 2]
            )
        )
        if config.flip_md_at_center
        else None
    )
    control_frames = np.stack(
        [
            surface_frame(
                normal,
                np.asarray(
                    [
                        0.0,
                        0.0,
                        -1.0
                        if central_flip_y_m is not None
                        and point[1] > central_flip_y_m
                        else 1.0,
                    ]
                ),
            )
            for point, normal in zip(control_points, control_normals, strict=True)
        ]
    )
    control_scan_axes = control_frames[:, :, 2]
    (
        smooth_points,
        smooth_normals,
        smooth_scan_axes,
        smooth_quaternions,
    ) = _resample_path(
        loess_points,
        loess_normals,
        config.sample_spacing_m,
        config.maximum_orientation_step_deg,
        central_flip_y_m,
    )

    inspected = np.zeros(len(mesh.faces), dtype=bool)
    radii = np.asarray(config.ellipsoid_radii_m, dtype=np.float64)
    for point, normal, scan_axis in zip(
        smooth_points,
        smooth_normals,
        smooth_scan_axes,
        strict=True,
    ):
        covered = _covered_faces_at_pose(
            mesh,
            target_ids,
            point,
            normal,
            radii,
            scan_axis,
        )
        inspected[target_ids[covered]] = True

    target_area = float(np.sum(areas[target_ids]))
    inspected_area = float(np.sum(areas[target_mask & inspected]))
    coverage_ratio = inspected_area / target_area
    stop_reason = (
        "paper_raster_complete"
        if coverage_ratio >= config.target_coverage_ratio
        else "paper_raster_below_target_coverage"
    )
    path_length = float(
        np.sum(np.linalg.norm(np.diff(smooth_points, axis=0), axis=1))
    )
    return TrajectoryResult(
        target_face_mask=target_mask,
        inspected_face_mask=inspected,
        control_points=control_points,
        control_normals=control_normals,
        control_scan_axes=control_scan_axes,
        control_face_ids=control_face_ids,
        lane_reference_y_m=lane_centres,
        smoothed_points=smooth_points,
        smoothed_normals=smooth_normals,
        smoothed_scan_axes=smooth_scan_axes,
        smoothed_quaternions_wxyz=smooth_quaternions,
        coverage_ratio=coverage_ratio,
        target_area_m2=target_area,
        inspected_area_m2=inspected_area,
        path_length_m=path_length,
        planning_time_s=perf_counter() - started,
        stop_reason=stop_reason,
        selection_scores=[],
    )
