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
    camera position is retained for outward-normal disambiguation after the
    fixed -X front-ROI selection.
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


def surface_frame(
    normal: ArrayLike,
    scan_direction: ArrayLike | None = None,
) -> FloatArray:
    """Return the IFToMM detector frame for one outward surface normal.

    Local +X points toward the human, local +Y follows the detector long
    axis, and local +Z follows the upward scan direction.  The physical
    handle lies on local -Y.
    """

    outward = np.asarray(normal, dtype=np.float64)
    norm = float(np.linalg.norm(outward))
    if norm <= 1e-12:
        raise ValueError("surface normal must be non-zero")
    inspection = -outward / norm
    scan_hint = np.asarray(
        [0.0, 0.0, 1.0] if scan_direction is None else scan_direction,
        dtype=np.float64,
    )
    if scan_hint.shape != (3,) or not np.all(np.isfinite(scan_hint)):
        raise ValueError("scan_direction must contain three finite values")
    if np.linalg.norm(scan_hint) <= 1e-12:
        scan_hint = np.asarray([0.0, 0.0, 1.0])
    scan = scan_hint - inspection * float(scan_hint @ inspection)
    if np.linalg.norm(scan) <= 1e-8:
        scan_hint = np.asarray([0.0, 1.0, 0.0])
        scan = scan_hint - inspection * float(scan_hint @ inspection)
    scan /= np.linalg.norm(scan)
    detector_long = np.cross(scan, inspection)
    detector_long /= np.linalg.norm(detector_long)
    scan = np.cross(inspection, detector_long)
    scan /= np.linalg.norm(scan)
    return np.column_stack((inspection, detector_long, scan))
