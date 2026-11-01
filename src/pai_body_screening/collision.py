from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .models import FloatArray, MeshData, face_geometry, surface_frame


@dataclass(frozen=True)
class DetectorBoxProxy:
    """Conservative box approximation used only to colour collisions.

    The box extends from the sensing centre along the negative local-y handle
    direction.  It is intentionally not used by the trajectory generator.
    """

    center_local_m: tuple[float, float, float] = (0.0, -0.18, 0.0)
    half_extents_m: tuple[float, float, float] = (0.035, 0.23, 0.060)
    margin_m: float = 0.005


@dataclass
class CollisionResult:
    pose_collision: NDArray[np.bool_]
    colliding_face_ids: list[list[int]]


def evaluate_collisions(
    mesh: MeshData,
    points: ArrayLike,
    normals: ArrayLike,
    proxy: DetectorBoxProxy = DetectorBoxProxy(),
) -> CollisionResult:
    positions = np.asarray(points, dtype=np.float64)
    directions = np.asarray(normals, dtype=np.float64)
    if positions.shape != directions.shape or positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError("points and normals must both have shape (N, 3)")
    centroids, _, _ = face_geometry(mesh)
    triangles = mesh.vertices[mesh.faces]
    samples = np.concatenate((triangles, centroids[:, None, :]), axis=1)
    center_local = np.asarray(proxy.center_local_m, dtype=np.float64)
    half = np.asarray(proxy.half_extents_m, dtype=np.float64) + proxy.margin_m
    flags = np.zeros(len(positions), dtype=bool)
    face_hits: list[list[int]] = []
    for pose_index, (position, normal) in enumerate(zip(positions, directions)):
        frame = surface_frame(normal)
        box_center = position + frame @ center_local
        local_samples = (samples - box_center[None, None, :]) @ frame
        inside = np.all(np.abs(local_samples) <= half[None, None, :], axis=2)
        hit_ids = np.flatnonzero(np.any(inside, axis=1)).astype(int).tolist()
        flags[pose_index] = bool(hit_ids)
        face_hits.append(hit_ids)
    return CollisionResult(pose_collision=flags, colliding_face_ids=face_hits)
