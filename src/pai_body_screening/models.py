from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray


FloatArray = NDArray[np.float64]
IntArray = NDArray[np.int64]


@dataclass(frozen=True)
class MeshData:
    """Triangular human mesh and Human3D face-part labels.

    ``face_part_ids`` must contain one Human3D part ID per triangle.  The
    camera position is used only to select the front-facing surface.
    """

    vertices: FloatArray
    faces: IntArray
    face_part_ids: IntArray
    camera_position: FloatArray

    @classmethod
    def from_arrays(
        cls,
        vertices: ArrayLike,
        faces: ArrayLike,
        face_part_ids: ArrayLike,
        camera_position: ArrayLike,
    ) -> "MeshData":
        mesh = cls(
            vertices=np.asarray(vertices, dtype=np.float64),
            faces=np.asarray(faces, dtype=np.int64),
            face_part_ids=np.asarray(face_part_ids, dtype=np.int64),
            camera_position=np.asarray(camera_position, dtype=np.float64),
        )
        mesh.validate()
        return mesh

    def validate(self) -> None:
        if self.vertices.ndim != 2 or self.vertices.shape[1] != 3:
            raise ValueError("vertices must have shape (N, 3)")
        if self.faces.ndim != 2 or self.faces.shape[1] != 3:
            raise ValueError("faces must have shape (M, 3)")
        if self.face_part_ids.shape != (len(self.faces),):
            raise ValueError("face_part_ids must contain one ID per face")
        if self.camera_position.shape != (3,):
            raise ValueError("camera_position must have shape (3,)")
        if len(self.vertices) == 0 or len(self.faces) == 0:
            raise ValueError("mesh must not be empty")
        if np.min(self.faces) < 0 or np.max(self.faces) >= len(self.vertices):
            raise ValueError("faces contain an out-of-range vertex index")
        if not (
            np.all(np.isfinite(self.vertices))
            and np.all(np.isfinite(self.camera_position))
        ):
            raise ValueError("mesh coordinates must be finite")


def face_geometry(mesh: MeshData) -> tuple[FloatArray, FloatArray, FloatArray]:
    triangles = mesh.vertices[mesh.faces]
    first_edges = triangles[:, 1] - triangles[:, 0]
    second_edges = triangles[:, 2] - triangles[:, 0]
    cross = np.cross(first_edges, second_edges)
    doubled_area = np.linalg.norm(cross, axis=1)
    normals = np.zeros_like(cross)
    valid = doubled_area > 1e-12
    normals[valid] = cross[valid] / doubled_area[valid, None]
    centroids = np.mean(triangles, axis=1)
    return centroids, normals, doubled_area * 0.5


def surface_frame(normal: ArrayLike) -> FloatArray:
    """Return a right-handed detector frame with local x along the normal."""

    local_x = np.asarray(normal, dtype=np.float64)
    norm = float(np.linalg.norm(local_x))
    if norm <= 1e-12:
        raise ValueError("surface normal must be non-zero")
    local_x = local_x / norm
    up = np.asarray([0.0, 0.0, 1.0])
    local_y = np.cross(up, local_x)
    if np.linalg.norm(local_y) <= 1e-8:
        local_y = np.cross(np.asarray([0.0, 1.0, 0.0]), local_x)
    local_y = local_y / np.linalg.norm(local_y)
    local_z = np.cross(local_x, local_y)
    local_z = local_z / np.linalg.norm(local_z)
    return np.column_stack((local_x, local_y, local_z))
