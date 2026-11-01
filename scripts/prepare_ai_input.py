#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json

from pai_body_screening.ai_adapter import prepare_planner_input


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Package an external Human3D + SegFit result for planning."
    )
    parser.add_argument("--result", required=True, help="Completed SegFit result.json")
    parser.add_argument(
        "--face-parts",
        required=True,
        help="Human3D part ID for each fitted SMPL-X face",
    )
    parser.add_argument("--camera-position", nargs=3, type=float, required=True)
    parser.add_argument("--output", default="outputs/ai-planner-input.json")
    args = parser.parse_args()
    summary = prepare_planner_input(
        args.result,
        args.face_parts,
        args.output,
        args.camera_position,
    )
    print(json.dumps(summary.__dict__, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
