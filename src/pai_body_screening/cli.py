from __future__ import annotations

import argparse
import json

from .collision import DetectorBoxProxy, evaluate_collisions
from .iftomm import IftommConfig, generate_trajectory
from .io import load_mesh_json, save_trajectory_json
from .visualize import save_preview


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate the public IFToMM body-screening trajectory."
    )
    parser.add_argument("input", help="Human3D/SegFit mesh JSON")
    parser.add_argument("--output", default="outputs/trajectory.json")
    parser.add_argument("--preview", default="outputs/trajectory-preview.png")
    parser.add_argument(
        "--face-parts",
        help="Optional external JSON array with one Human3D ID per SMPL-X face",
    )
    parser.add_argument(
        "--camera-position",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        help="Override camera position in metres",
    )
    parser.add_argument("--coverage", type=float, default=0.99)
    parser.add_argument("--sample-spacing", type=float, default=0.010)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    mesh = load_mesh_json(
        args.input,
        face_part_ids_path=args.face_parts,
        camera_position=args.camera_position,
    )
    config = IftommConfig(
        target_coverage_ratio=args.coverage,
        sample_spacing_m=args.sample_spacing,
    )
    result = generate_trajectory(mesh, config)
    collisions = evaluate_collisions(
        mesh,
        result.smoothed_points,
        result.smoothed_normals,
        DetectorBoxProxy(),
    )
    save_trajectory_json(args.output, result, collisions, config)
    save_preview(args.preview, mesh, result, collisions)
    print(
        json.dumps(
            {
                "coverage_percent": round(100.0 * result.coverage_ratio, 3),
                "path_length_m": round(result.path_length_m, 4),
                "planning_time_s": round(result.planning_time_s, 4),
                "control_points": len(result.control_points),
                "collision_poses": int(collisions.pose_collision.sum()),
                "output": args.output,
                "preview": args.preview,
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
