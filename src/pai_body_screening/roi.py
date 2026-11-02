from __future__ import annotations

from collections.abc import Iterable

import numpy as np
from numpy.typing import NDArray

from .models import MeshData, face_geometry


# Human3D: right thigh, torso, left thigh, pelvis.
TARGET_PART_IDS = frozenset({2, 7, 14, 15})
FRONT_NORMAL_MAX_ANGLE_DEG = 75.0
FRONT_WIDTH_FRACTION = 0.80
FRONT_TOP_MARGIN_M = 0.100


def front_normal_mask(
    normals: NDArray[np.float64],
    max_angle_deg: float = FRONT_NORMAL_MAX_ANGLE_DEG,
) -> NDArray[np.bool_]:
    """Select normals whose XY projection is within the given angle of -X."""

    values = np.asarray(normals, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("normals must have shape (N, 3)")
    if not 0.0 < max_angle_deg < 90.0:
        raise ValueError("max_angle_deg must be in (0, 90)")
    xy_norm = np.linalg.norm(values[:, :2], axis=1)
    cosine = np.full(len(values), -np.inf, dtype=np.float64)
    valid = xy_norm > 1e-12
    cosine[valid] = -values[valid, 0] / xy_norm[valid]
    # A strict comparison excludes faces exactly on the angular boundary.
    return valid & (cosine > np.cos(np.deg2rad(max_angle_deg)))


def central_front_bounds_mask(
    centroids: NDArray[np.float64],
    angular_mask: NDArray[np.bool_],
    *,
    width_fraction: float = FRONT_WIDTH_FRACTION,
    top_margin_m: float = FRONT_TOP_MARGIN_M,
) -> NDArray[np.bool_]:
    """Crop an angular front ROI to its central width and lowered top bound."""

    points = np.asarray(centroids, dtype=np.float64)
    candidates = np.asarray(angular_mask, dtype=bool)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("centroids must have shape (N, 3)")
    if candidates.shape != (len(points),):
        raise ValueError("angular_mask must contain one value per centroid")
    if not 0.0 < width_fraction <= 1.0:
        raise ValueError("width_fraction must be in (0, 1]")
    if top_margin_m < 0.0:
        raise ValueError("top_margin_m must be non-negative")
    if not np.any(candidates):
        return np.zeros(len(points), dtype=bool)

    candidate_points = points[candidates]
    minimum_y = float(np.min(candidate_points[:, 1]))
    maximum_y = float(np.max(candidate_points[:, 1]))
    center_y = 0.5 * (minimum_y + maximum_y)
    half_width = 0.5 * (maximum_y - minimum_y) * width_fraction
    maximum_z = float(np.max(candidate_points[:, 2])) - top_margin_m
    return (
        candidates
        & (points[:, 1] >= center_y - half_width)
        & (points[:, 1] <= center_y + half_width)
        & (points[:, 2] <= maximum_z)
    )


def select_target_faces(
    mesh: MeshData,
    part_ids: Iterable[int] = TARGET_PART_IDS,
    *,
    front_only: bool = True,
    front_normal_max_angle_deg: float = FRONT_NORMAL_MAX_ANGLE_DEG,
    front_width_fraction: float = FRONT_WIDTH_FRACTION,
    front_top_margin_m: float = FRONT_TOP_MARGIN_M,
) -> NDArray[np.bool_]:
    """Select the explicitly allowed Human3D parts and, optionally, front faces.

    The function intentionally raises when no target remains.  It never falls
    back to scanning the full body.
    """

    allowed = np.asarray(sorted(set(int(value) for value in part_ids)), dtype=np.int64)
    if len(allowed) == 0:
        raise ValueError("at least one target part ID is required")
    selected = np.isin(mesh.face_part_ids, allowed)
    if front_only:
        centroids, normals, areas = face_geometry(mesh)
        selected &= front_normal_mask(normals, front_normal_max_angle_deg)
        selected &= areas > 1e-12
        selected = central_front_bounds_mask(
            centroids,
            selected,
            width_fraction=front_width_fraction,
            top_margin_m=front_top_margin_m,
        )
    if not np.any(selected):
        raise ValueError(
            "ROI is empty; check Human3D labels, mesh winding, and camera position"
        )
    return selected
