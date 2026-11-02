#!/usr/bin/env python3
"""Prepare private, local-only assets for the modular presentation video.

The source files are read from the protected body_screening workspace, but this
script writes only to the PAI2026 submission workspace.  The generated
``private-*.json`` file is ignored by Git and is used only while rendering.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any


BODY_SCREENING_SOURCE_ROOT = Path(
    os.environ.get(
        "BODY_SCREENING_SOURCE_ROOT",
        Path(__file__).resolve().parents[3] / "body_screening",
    )
)
DEFAULT_JOB = (
    BODY_SCREENING_SOURCE_ROOT
    / "runtime/pose-estimation/jobs/72edf173577d4538852991e8d2250e76"
)
DEFAULT_GROUND_TRUTH = (
    BODY_SCREENING_SOURCE_ROOT
    / "runtime/execution-evaluation/f23ada90-9f15-4dbd-b34d-313814569fe6"
    / "ground-truth-human-v2.ply"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--job-dir", type=Path, default=DEFAULT_JOB)
    parser.add_argument("--ground-truth", type=Path, default=DEFAULT_GROUND_TRUTH)
    parser.add_argument(
        "--scene",
        type=Path,
        default=Path("web/public/data/private-ai-scene.json"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("web/public/data/private-presentation.json"),
    )
    return parser.parse_args()


def read_ascii_ply(path: Path) -> tuple[list[list[float]], list[list[int]]]:
    with path.open("r", encoding="utf-8") as stream:
        if stream.readline().strip() != "ply":
            raise ValueError(f"not a PLY file: {path}")
        vertex_count = 0
        face_count = 0
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"truncated PLY header: {path}")
            fields = line.strip().split()
            if fields[:2] == ["element", "vertex"]:
                vertex_count = int(fields[2])
            elif fields[:2] == ["element", "face"]:
                face_count = int(fields[2])
            elif fields == ["end_header"]:
                break
        vertices = [
            [float(value) for value in stream.readline().split()[:3]]
            for _ in range(vertex_count)
        ]
        faces: list[list[int]] = []
        for _ in range(face_count):
            values = [int(value) for value in stream.readline().split()]
            if not values:
                continue
            polygon = values[1 : values[0] + 1]
            for corner in range(1, len(polygon) - 1):
                faces.append([polygon[0], polygon[corner], polygon[corner + 1]])
    return vertices, faces


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    args = parse_args()
    job = load_json(args.job_dir / "job.json")
    result = load_json(args.job_dir / "result.json")
    input_colors = load_json(args.job_dir / "input-colors.json")
    input_points, _ = read_ascii_ply(args.job_dir / "input.ply")
    subject_vertices, subject_faces = read_ascii_ply(args.ground_truth)
    subject_meta = load_json(args.ground_truth.with_suffix(args.ground_truth.suffix + ".json"))
    scene = load_json(args.scene)

    if len(input_points) != len(input_colors):
        raise ValueError("input point and RGB arrays have different lengths")

    # Keep the whole acquisition count as evidence, while rendering only the
    # subject-centred working volume so the body remains legible in 1080p.
    selected_indices = [
        index
        for index, (x, y, z) in enumerate(input_points)
        if 0.55 <= x <= 2.15 and -1.15 <= y <= 1.15 and -0.05 <= z <= 2.15
    ]
    raw_points = [input_points[index] for index in selected_indices]
    raw_colors = [input_colors[index] for index in selected_indices]

    payload = {
        "schema_version": "pai2026_presentation_v1",
        "subject": {
            "id": subject_meta.get("human_id", "petite_female"),
            "pose": subject_meta.get("body_pose_id", "T_pose"),
            "includes_rendered_garments": subject_meta.get(
                "includes_rendered_garments", True
            ),
            "vertices": subject_vertices,
            "faces": subject_faces,
            "meshes": subject_meta.get("meshes", []),
        },
        "acquisition": {
            "backend": job["acquisition_backend"],
            "point_count": job["point_count"],
            "displayed_point_count": len(raw_points),
            "points": raw_points,
            "colors": raw_colors,
        },
        "human3d": {
            "points": result["human3d_points"],
            "part_ids": result["human3d_part_ids"],
            "point_count": len(result["human3d_points"]),
        },
        "smplx": {
            "vertices": result["vertices"],
            "faces": result["faces"],
            "body_model": result["body_model"],
            "fitting_loss": result["fitting_loss"],
        },
        "metrics": {
            "coverage_ratio": scene["metrics"]["coverage_ratio"],
            "inspection_time_s": scene["trajectory"][-1]["robot"]["time_s"],
            "path_length_m": scene["metrics"]["path_length_m"],
            "planning_time_s": scene["metrics"]["planning_time_s"],
            "pose_estimation_time_s": job["duration_s"],
            "collision_pose_count": scene["metrics"]["collision_pose_count"],
            "scan_speed_scale": scene["parameters"].get("scan_speed_scale", 1.0),
            "trajectory_pose_count": scene["metrics"]["trajectory_pose_count"],
        },
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
        encoding="utf-8",
    )
    print(
        f"{args.output}: subject={len(subject_vertices)} vertices, "
        f"raw={len(raw_points)}/{len(input_points)} points, "
        f"Human3D={len(result['human3d_points'])} points"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
