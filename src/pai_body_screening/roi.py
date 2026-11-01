from __future__ import annotations

from collections.abc import Iterable

import numpy as np
from numpy.typing import NDArray

from .models import MeshData, face_geometry


# Human3D: right thigh, torso, left thigh, pelvis.
TARGET_PART_IDS = frozenset({2, 7, 14, 15})


def select_target_faces(
    mesh: MeshData,
    part_ids: Iterable[int] = TARGET_PART_IDS,
    *,
    front_only: bool = True,
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
        to_camera = mesh.camera_position[None, :] - centroids
        selected &= np.einsum("ij,ij->i", normals, to_camera) > 0.0
        selected &= areas > 1e-12
    if not np.any(selected):
        raise ValueError(
            "ROI is empty; check Human3D labels, mesh winding, and camera position"
        )
    return selected
