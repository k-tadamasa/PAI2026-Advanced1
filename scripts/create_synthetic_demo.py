#!/usr/bin/env python3
"""Create a small labelled mesh for a dependency-free public smoke test.

This is not an AI result and is never used as evidence for the final report.
It only verifies the trajectory pipeline before a Human3D/SegFit result is
supplied.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def add_patch(
    vertices: list[list[float]],
    faces: list[list[int]],
    part_ids: list[int],
    *,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
    part_id: int,
    columns: int,
    rows: int,
    x: float = 0.0,
) -> None:
    base = len(vertices)
    for row in range(rows + 1):
        z = z_min + (z_max - z_min) * row / rows
        for column in range(columns + 1):
            y = y_min + (y_max - y_min) * column / columns
            vertices.append([x, y, z])
    stride = columns + 1
    for row in range(rows):
        for column in range(columns):
            lower_left = base + row * stride + column
            lower_right = lower_left + 1
            upper_left = lower_left + stride
            upper_right = upper_left + 1
            # Winding gives -x, toward the camera at x=-2.
            faces.append([lower_left, upper_right, lower_right])
            faces.append([lower_left, upper_left, upper_right])
            part_ids.extend([part_id, part_id])


def create_payload() -> dict:
    vertices: list[list[float]] = []
    faces: list[list[int]] = []
    part_ids: list[int] = []
    add_patch(
        vertices,
        faces,
        part_ids,
        y_min=-0.26,
        y_max=0.26,
        z_min=0.92,
        z_max=1.52,
        part_id=7,
        columns=8,
        rows=10,
    )
    add_patch(
        vertices,
        faces,
        part_ids,
        y_min=-0.25,
        y_max=0.25,
        z_min=0.72,
        z_max=0.92,
        part_id=15,
        columns=8,
        rows=4,
    )
    add_patch(
        vertices,
        faces,
        part_ids,
        y_min=-0.24,
        y_max=-0.025,
        z_min=0.18,
        z_max=0.72,
        part_id=2,
        columns=4,
        rows=9,
    )
    add_patch(
        vertices,
        faces,
        part_ids,
        y_min=0.025,
        y_max=0.24,
        z_min=0.18,
        z_max=0.72,
        part_id=14,
        columns=4,
        rows=9,
    )
    # Excluded head patch proves that ROI selection does not silently expand.
    add_patch(
        vertices,
        faces,
        part_ids,
        y_min=-0.12,
        y_max=0.12,
        z_min=1.52,
        z_max=1.76,
        part_id=4,
        columns=4,
        rows=4,
    )
    return {
        "camera_position_m": [-2.0, 0.0, 1.1],
        "mesh": {
            "vertices": vertices,
            "faces": faces,
            "face_part_ids": part_ids,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="/tmp/pai2026-synthetic-mesh.json")
    args = parser.parse_args()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(create_payload(), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
