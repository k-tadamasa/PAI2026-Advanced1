#!/usr/bin/env python3
"""Render a one-minute public simulation video from planner JSON outputs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
from matplotlib.font_manager import FontProperties
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Line3DCollection


TARGET_PART_IDS = {2, 7, 14, 15}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", default="outputs/trajectory.json")
    parser.add_argument("--ai-input", default="outputs/ai-planner-input.json")
    parser.add_argument(
        "--output", default="output/video/PAI最終課題_TADAMASA_NDA.mp4"
    )
    parser.add_argument(
        "--poster", default="output/video/PAI最終課題_TADAMASA_NDA_poster.png"
    )
    parser.add_argument("--stride", type=int, default=2)
    parser.add_argument("--fps", type=int, default=5)
    parser.add_argument("--max-frames", type=int)
    return parser.parse_args()


def elbow_position(shoulder: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Return a stable illustrative two-link elbow pose ending at target."""
    link_1 = 0.57
    link_2 = 0.57
    vector = target - shoulder
    distance = float(np.linalg.norm(vector))
    direction = vector / max(distance, 1e-9)
    reachable = float(np.clip(distance, 0.08, link_1 + link_2 - 1e-4))
    along = (link_1**2 - link_2**2 + reachable**2) / (2.0 * reachable)
    height = np.sqrt(max(0.0, link_1**2 - along**2))
    bend = np.cross(direction, np.array([0.0, 1.0, 0.0]))
    if np.linalg.norm(bend) < 1e-6:
        bend = np.array([0.0, 0.0, 1.0])
    bend /= np.linalg.norm(bend)
    return shoulder + direction * along + bend * height


def main() -> int:
    args = parse_args()
    trajectory_payload = json.loads(Path(args.trajectory).read_text(encoding="utf-8"))
    ai_payload = json.loads(Path(args.ai_input).read_text(encoding="utf-8"))
    poses = trajectory_payload["trajectory"][:: max(1, args.stride)]
    if args.max_frames is not None:
        poses = poses[: max(0, args.max_frames)]
    if not poses:
        raise ValueError("trajectory contains no selected poses")

    all_poses = trajectory_payload["trajectory"]
    trajectory = np.asarray([pose["position_m"] for pose in all_poses], dtype=float)
    collisions = np.asarray([pose["collision"] for pose in all_poses], dtype=bool)
    vertices = np.asarray(ai_payload["mesh"]["vertices"], dtype=float)
    faces = np.asarray(ai_payload["mesh"]["faces"], dtype=int)
    part_ids = np.asarray(ai_payload["mesh"]["face_part_ids"], dtype=int)
    roi_faces = faces[np.isin(part_ids, list(TARGET_PART_IDS))]
    roi_centroids = vertices[roi_faces].mean(axis=1)

    plt.style.use("dark_background")
    figure = plt.figure(figsize=(9.6, 5.4), dpi=100, facecolor="#08111f")
    axis = figure.add_subplot(111, projection="3d", facecolor="#08111f")
    figure.subplots_adjust(left=0.0, right=0.78, bottom=0.02, top=0.89)

    vertex_step = max(1, len(vertices) // 1800)
    roi_step = max(1, len(roi_centroids) // 1800)
    body = vertices[::vertex_step]
    roi = roi_centroids[::roi_step]
    axis.scatter(body[:, 0], body[:, 1], body[:, 2], s=1.0, c="#94a3b8", alpha=0.12)
    axis.scatter(roi[:, 0], roi[:, 1], roi[:, 2], s=2.0, c="#34d399", alpha=0.28)

    path_segments = np.stack([trajectory[:-1], trajectory[1:]], axis=1)
    segment_collision = collisions[:-1] | collisions[1:]
    path_colors = np.where(segment_collision[:, None], "#ef4444", "#22c55e").ravel()
    axis.add_collection3d(
        Line3DCollection(path_segments, colors=path_colors, linewidths=1.8, alpha=0.72)
    )

    base = np.array([0.75, 0.0, 0.18])
    shoulder = np.array([0.75, 0.0, 0.85])
    axis.plot(
        [base[0], shoulder[0]],
        [base[1], shoulder[1]],
        [base[2], shoulder[2]],
        color="#60a5fa",
        linewidth=7,
        solid_capstyle="round",
    )
    arm_line, = axis.plot([], [], [], color="#60a5fa", linewidth=7, solid_capstyle="round")
    joint_points, = axis.plot([], [], [], "o", color="#dbeafe", markersize=6)
    tool_marker, = axis.plot([], [], [], "o", color="#fbbf24", markersize=8)
    progress_line, = axis.plot([], [], [], color="#f8fafc", linewidth=2.4, alpha=0.9)

    axis.set_xlim(0.50, 1.60)
    axis.set_ylim(-0.65, 0.65)
    axis.set_zlim(0.0, 1.75)
    axis.set_box_aspect((1.25, 1.5, 1.75))
    axis.view_init(elev=11, azim=-128)
    axis.set_axis_off()

    jp_font = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    figure.suptitle(
        "PAI最終課題_TADAMASA_NDA",
        x=0.39,
        y=0.97,
        color="#f8fafc",
        fontsize=17,
        fontweight="bold",
        fontproperties=jp_font,
    )
    figure.text(
        0.39,
        0.91,
        "AI body-surface inference + public IFToMM trajectory",
        color="#cbd5e1",
        fontsize=10,
        ha="center",
    )
    status = figure.text(0.80, 0.55, "", color="#f8fafc", fontsize=9.5, va="top")
    figure.text(
        0.80,
        0.95,
        "Pipeline\n3D point cloud\n  ↓\nHuman3D\n  ↓\nSegFit / SMPL-X\n  ↓\nFront ROI\n  ↓\nIFToMM trajectory",
        color="#cbd5e1",
        fontsize=9.5,
        va="top",
        linespacing=1.35,
    )
    figure.text(
        0.80,
        0.25,
        "Result\ncoverage  99.03%\npath      6.41 m\nposes     628\n\nCollision avoidance:\nfuture work",
        color="#cbd5e1",
        fontsize=9.5,
        va="top",
        linespacing=1.35,
    )
    axis.legend(
        handles=[
            Line2D([0], [0], color="#22c55e", lw=3, label="safe proxy"),
            Line2D([0], [0], color="#ef4444", lw=3, label="collision proxy"),
            Line2D([0], [0], color="#60a5fa", lw=5, label="robot arm"),
        ],
        loc="lower left",
        framealpha=0.20,
        fontsize=8,
    )

    def update(frame_index: int):
        pose = poses[frame_index]
        target = np.asarray(pose["position_m"], dtype=float)
        elbow = elbow_position(shoulder, target)
        arm = np.vstack([shoulder, elbow, target])
        arm_line.set_data_3d(arm[:, 0], arm[:, 1], arm[:, 2])
        joint_points.set_data_3d(arm[:, 0], arm[:, 1], arm[:, 2])
        tool_marker.set_data_3d([target[0]], [target[1]], [target[2]])
        original_index = int(pose["index"])
        visited = trajectory[: original_index + 1]
        progress_line.set_data_3d(visited[:, 0], visited[:, 1], visited[:, 2])
        collision_text = "YES (red)" if pose["collision"] else "NO (green)"
        status.set_text(
            f"Scan {frame_index + 1:03d}/{len(poses):03d}\n"
            f"trajectory pose {original_index + 1}/628\n\n"
            f"collision proxy\n{collision_text}\n\n"
            "Front ROI\ntorso / pelvis / thighs"
        )
        status.set_color("#fca5a5" if pose["collision"] else "#bbf7d0")
        return arm_line, joint_points, tool_marker, progress_line, status

    output = Path(args.output)
    poster = Path(args.poster)
    output.parent.mkdir(parents=True, exist_ok=True)
    poster.parent.mkdir(parents=True, exist_ok=True)
    update(0)
    figure.savefig(poster, dpi=100, facecolor=figure.get_facecolor())

    import imageio_ffmpeg

    plt.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
    writer = animation.FFMpegWriter(
        fps=args.fps,
        codec="libx264",
        bitrate=2600,
        metadata={"title": "PAI最終課題_TADAMASA_NDA", "artist": "TADAMASA_NDA"},
        extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"],
    )
    with writer.saving(figure, str(output), dpi=100):
        for frame_index in range(len(poses)):
            update(frame_index)
            writer.grab_frame(facecolor=figure.get_facecolor())
            if frame_index % 50 == 0 or frame_index + 1 == len(poses):
                print(f"video frame {frame_index + 1}/{len(poses)}", flush=True)
    plt.close(figure)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
