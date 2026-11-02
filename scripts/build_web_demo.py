#!/usr/bin/env python3
"""Build the anonymous static scene consumed by the public web application."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from pai_body_screening.collision import (
    DetectorGripperMeshModel,
    evaluate_detector_gripper_collisions,
)
from pai_body_screening.iftomm import IftommConfig, generate_trajectory
from pai_body_screening.io import load_mesh_json
from pai_body_screening.web_export import build_web_scene


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mesh", default="web/public/data/public-demo-mesh.json")
    parser.add_argument("--output", default="web/public/data/demo-scene.json")
    parser.add_argument("--collision-assets", default="web/public/assets")
    parser.add_argument("--central-lane-gap-scale", type=float, default=1.2)
    args = parser.parse_args()

    mesh = load_mesh_json(args.mesh)
    config = IftommConfig(central_lane_gap_scale=args.central_lane_gap_scale)
    result = generate_trajectory(mesh, config)
    collision_model = DetectorGripperMeshModel.from_asset_root(args.collision_assets)
    collisions = evaluate_detector_gripper_collisions(
        mesh,
        result.smoothed_points,
        result.smoothed_quaternions_wxyz,
        collision_model,
    )
    payload = build_web_scene(mesh, result, collisions, config)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )
    print(
        f"{output}: {len(payload['trajectory'])} poses, "
        f"coverage={payload['metrics']['coverage_ratio']:.2%}, "
        f"mesh-collisions={payload['metrics']['collision_pose_count']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
