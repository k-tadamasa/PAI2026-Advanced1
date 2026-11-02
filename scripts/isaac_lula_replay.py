#!/usr/bin/env python3
"""Replay a planned Cartesian trajectory with Isaac Sim Lula IK.

Run this file with Isaac Sim's ``python.sh``.  Robot assets and Lula
configuration are external inputs and are not redistributed by this project.
The script does not save the opened USD stage and does not perform collision
avoidance; red trajectory points remain visible as future-work limitations.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--stage", required=True, help="Existing USD stage")
    parser.add_argument("--robot-prim", required=True)
    parser.add_argument("--robot-description", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--end-effector", required=True)
    parser.add_argument("--frames-per-pose", type=int, default=2)
    parser.add_argument("--hold-frames", type=int, default=240)
    parser.add_argument("--position-only", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--execution-report", default="outputs/isaac-execution.json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})

    import numpy as np
    from pxr import Gf, UsdGeom

    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation as Articulation
    from isaacsim.core.utils.stage import open_stage
    from isaacsim.robot_motion.motion_generation import (
        ArticulationKinematicsSolver,
        LulaKinematicsSolver,
    )

    payload = json.loads(Path(args.trajectory).read_text(encoding="utf-8"))
    poses = payload["trajectory"]
    if not poses:
        raise ValueError("trajectory contains no poses")

    open_stage(args.stage)
    world = World(stage_units_in_meters=1.0)
    articulation = world.scene.add(Articulation(args.robot_prim, name="pai2026_robot"))
    world.reset()

    lula = LulaKinematicsSolver(
        robot_description_path=args.robot_description,
        urdf_path=args.urdf,
    )
    ik = ArticulationKinematicsSolver(articulation, lula, args.end_effector)

    # Per-point red/green colours make collision limitations explicit in video.
    stage = world.stage
    points_prim = UsdGeom.Points.Define(stage, "/World/PAI2026Trajectory")
    points_prim.CreatePointsAttr(
        [Gf.Vec3f(*[float(value) for value in pose["position_m"]]) for pose in poses]
    )
    points_prim.CreateWidthsAttr([0.018] * len(poses))
    points_prim.CreateDisplayColorAttr(
        [
            Gf.Vec3f(0.94, 0.18, 0.18)
            if pose["collision"]
            else Gf.Vec3f(0.13, 0.77, 0.37)
            for pose in poses
        ]
    )

    execution = []
    for pose in poses:
        base_position, base_orientation = articulation.get_world_pose()
        lula.set_robot_base_pose(base_position, base_orientation)
        target_position = np.asarray(pose["position_m"], dtype=np.float64)
        target_orientation = None
        if not args.position_only:
            target_orientation = np.asarray(
                pose["orientation_wxyz"], dtype=np.float64
            )
        action, success = ik.compute_inverse_kinematics(
            target_position, target_orientation
        )
        if success:
            articulation.apply_action(action)
        for _ in range(max(1, args.frames_per_pose)):
            world.step(render=True)
        execution.append(
            {
                "index": int(pose["index"]),
                "ik_success": bool(success),
                "mesh_collision": bool(pose["collision"]),
            }
        )

    for _ in range(max(0, args.hold_frames)):
        world.step(render=True)

    report_path = Path(args.execution_report)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(
            {
                "schema_version": "pai2026_isaac_lula_replay_v1",
                "collision_avoidance": False,
                "pose_count": len(execution),
                "ik_success_count": sum(item["ik_success"] for item in execution),
                "execution": execution,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
