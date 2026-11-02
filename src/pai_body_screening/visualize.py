from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

from .collision import CollisionResult
from .iftomm import TrajectoryResult
from .models import MeshData


def save_preview(
    path: str | Path,
    mesh: MeshData,
    result: TrajectoryResult,
    collisions: CollisionResult,
) -> None:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(8, 8), constrained_layout=True)
    axis = fig.add_subplot(111, projection="3d")

    background = Poly3DCollection(
        mesh.vertices[mesh.faces],
        facecolors=(0.65, 0.68, 0.72, 0.12),
        edgecolors="none",
    )
    axis.add_collection3d(background)

    target_ids = np.flatnonzero(result.target_face_mask)
    target_colors = np.where(
        result.inspected_face_mask[target_ids, None],
        np.asarray([[0.13, 0.77, 0.37, 0.48]]),
        np.asarray([[0.96, 0.25, 0.25, 0.55]]),
    )
    target = Poly3DCollection(
        mesh.vertices[mesh.faces[target_ids]],
        facecolors=target_colors,
        edgecolors="none",
    )
    axis.add_collection3d(target)

    if len(result.smoothed_points) >= 2:
        segments = np.stack(
            (result.smoothed_points[:-1], result.smoothed_points[1:]), axis=1
        )
        segment_collision = collisions.pose_collision[:-1] | collisions.pose_collision[1:]
        colors = ["#ef4444" if value else "#22c55e" for value in segment_collision]
        axis.add_collection3d(Line3DCollection(segments, colors=colors, linewidths=3.0))
    axis.scatter(
        result.control_points[:, 0],
        result.control_points[:, 1],
        result.control_points[:, 2],
        s=8,
        c="#2563eb",
        label="IFToMM control points",
    )

    minimum = np.min(mesh.vertices, axis=0)
    maximum = np.max(mesh.vertices, axis=0)
    center = (minimum + maximum) * 0.5
    radius = float(np.max(maximum - minimum) * 0.56)
    axis.set_xlim(center[0] - radius, center[0] + radius)
    axis.set_ylim(center[1] - radius, center[1] + radius)
    axis.set_zlim(center[2] - radius, center[2] + radius)
    axis.set_box_aspect((1, 1, 1))
    axis.set_xticks([])
    axis.set_xlabel("")
    axis.set_ylabel("y [m]")
    axis.set_zlabel("z [m]")
    fig.suptitle(
        f"Front torso/pelvis/thigh scan | coverage {100 * result.coverage_ratio:.2f}%\n"
        "green: clear, red: MD / right-gripper mesh collision (future work)",
        fontsize=14,
    )
    axis.view_init(elev=8, azim=180)
    axis.legend(loc="upper right")
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)
