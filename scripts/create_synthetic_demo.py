#!/usr/bin/env python3
"""Create an anonymous curved body surface for the public web demo.

This geometry is generated from simple ellipses. It is not an AI result and
does not contain an identifiable scan or redistributed SMPL-X model data.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def add_front_patch(
    vertices: list[list[float]],
    faces: list[list[int]],
    part_ids: list[int],
    *,
    z_min: float,
    z_max: float,
    part_id: int,
    columns: int,
    rows: int,
    center_y: float = 0.0,
    half_width_bottom: float,
    half_width_top: float,
    depth_bottom: float,
    depth_top: float,
) -> None:
    base = len(vertices)
    for row in range(rows + 1):
        t = row / rows
        z = z_min + (z_max - z_min) * t
        half_width = half_width_bottom + (half_width_top - half_width_bottom) * t
        depth = depth_bottom + (depth_top - depth_bottom) * t
        for column in range(columns + 1):
            lateral = -1.0 + 2.0 * column / columns
            y = center_y + lateral * half_width
            # Camera is at negative X. This is the curved front half of an
            # elliptical body section centered around X=1.20 m.
            x = 1.20 - depth * (max(0.0, 1.0 - lateral * lateral) ** 0.5)
            vertices.append([x, y, z])
    stride = columns + 1
    for row in range(rows):
        for column in range(columns):
            lower_left = base + row * stride + column
            lower_right = lower_left + 1
            upper_left = lower_left + stride
            upper_right = upper_left + 1
            # Winding gives -X, toward the camera at X=-2.
            faces.append([lower_left, upper_right, lower_right])
            faces.append([lower_left, upper_left, upper_right])
            part_ids.extend([part_id, part_id])


def create_payload() -> dict:
    vertices: list[list[float]] = []
    faces: list[list[int]] = []
    part_ids: list[int] = []
    add_front_patch(
        vertices,
        faces,
        part_ids,
        z_min=0.92,
        z_max=1.52,
        part_id=7,
        columns=24,
        rows=24,
        half_width_bottom=0.20,
        half_width_top=0.28,
        depth_bottom=0.12,
        depth_top=0.14,
    )
    add_front_patch(
        vertices,
        faces,
        part_ids,
        z_min=0.72,
        z_max=0.92,
        part_id=15,
        columns=24,
        rows=10,
        half_width_bottom=0.24,
        half_width_top=0.20,
        depth_bottom=0.15,
        depth_top=0.12,
    )
    add_front_patch(
        vertices,
        faces,
        part_ids,
        z_min=0.18,
        z_max=0.72,
        part_id=2,
        columns=14,
        rows=22,
        center_y=-0.13,
        half_width_bottom=0.085,
        half_width_top=0.105,
        depth_bottom=0.095,
        depth_top=0.12,
    )
    add_front_patch(
        vertices,
        faces,
        part_ids,
        z_min=0.18,
        z_max=0.72,
        part_id=14,
        columns=14,
        rows=22,
        center_y=0.13,
        half_width_bottom=0.085,
        half_width_top=0.105,
        depth_bottom=0.095,
        depth_top=0.12,
    )
    # Excluded head patch proves that ROI selection remains restricted.
    add_front_patch(
        vertices,
        faces,
        part_ids,
        z_min=1.52,
        z_max=1.76,
        part_id=4,
        columns=12,
        rows=10,
        half_width_bottom=0.12,
        half_width_top=0.10,
        depth_bottom=0.10,
        depth_top=0.095,
    )
    return {
        "schema_version": "pai2026_anonymous_public_mesh_v1",
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
