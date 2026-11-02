#!/usr/bin/env python3
"""Adaptively subdivide robot control poses while preserving the MD path."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pai_body_screening.web_export import densify_web_scene_controls


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scan-speed-scale", type=float, default=2.0)
    parser.add_argument("--maximum-position-step-mm", type=float, default=2.0)
    parser.add_argument("--maximum-orientation-step-deg", type=float, default=0.5)
    parser.add_argument("--maximum-base-step-mm", type=float, default=2.0)
    parser.add_argument("--maximum-base-yaw-step-deg", type=float, default=0.5)
    parser.add_argument("--maximum-joint-step-deg", type=float, default=0.5)
    parser.add_argument("--maximum-lift-step-mm", type=float, default=2.0)
    args = parser.parse_args()

    scene = json.loads(args.input.read_text(encoding="utf-8"))
    payload = densify_web_scene_controls(
        scene,
        scan_speed_scale=args.scan_speed_scale,
        maximum_position_step_m=args.maximum_position_step_mm / 1000.0,
        maximum_orientation_step_deg=args.maximum_orientation_step_deg,
        maximum_base_step_m=args.maximum_base_step_mm / 1000.0,
        maximum_base_yaw_step_deg=args.maximum_base_yaw_step_deg,
        maximum_joint_step_deg=args.maximum_joint_step_deg,
        maximum_lift_step_m=args.maximum_lift_step_mm / 1000.0,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
        encoding="utf-8",
    )
    print(
        f"{args.output}: {len(scene['trajectory'])} -> "
        f"{len(payload['trajectory'])} control poses, "
        f"duration={payload['trajectory'][-1]['robot']['time_s']:.3f} s, "
        f"average={payload['parameters']['control_pose_rate_average_hz']:.1f} poses/s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
