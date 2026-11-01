#!/usr/bin/env python3
"""Run the public trajectory with Isaac Sim's built-in Franka robot.

The scene is assembled from Isaac Sim standard assets and the public JSON
inputs.  No private stage, robot model, or research-platform source is used.
Use Isaac Sim's ``python.sh`` to run this file.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


TARGET_PART_IDS = {2, 7, 14, 15}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", default="outputs/trajectory.json")
    parser.add_argument("--ai-input", default="outputs/ai-planner-input.json")
    parser.add_argument("--execution-report", default="outputs/franka-execution.json")
    parser.add_argument("--capture-dir")
    parser.add_argument("--capture-stride", type=int, default=2)
    parser.add_argument("--max-poses", type=int)
    parser.add_argument("--settle-frames", type=int, default=3)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


def make_human_mesh(stage, mesh_payload) -> None:
    from pxr import Gf, Sdf, UsdGeom

    vertices = mesh_payload["vertices"]
    faces = mesh_payload["faces"]
    part_ids = mesh_payload["face_part_ids"]
    mesh = UsdGeom.Mesh.Define(stage, "/World/Human3DMesh")
    mesh.CreatePointsAttr([Gf.Vec3f(*[float(v) for v in point]) for point in vertices])
    mesh.CreateFaceVertexCountsAttr([3] * len(faces))
    mesh.CreateFaceVertexIndicesAttr([int(index) for face in faces for index in face])
    mesh.CreateDoubleSidedAttr(True)

    colors = [
        Gf.Vec3f(0.18, 0.78, 0.52) if int(part_id) in TARGET_PART_IDS else Gf.Vec3f(0.72, 0.76, 0.82)
        for part_id in part_ids
    ]
    color = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.uniform
    )
    color.Set(colors)


def make_trajectory(stage, poses) -> None:
    from pxr import Gf, UsdGeom

    points = UsdGeom.Points.Define(stage, "/World/IFToMMTrajectory")
    points.CreatePointsAttr(
        [Gf.Vec3f(*[float(value) for value in pose["position_m"]]) for pose in poses]
    )
    points.CreateWidthsAttr([0.012] * len(poses))
    points.CreateDisplayColorAttr(
        [
            Gf.Vec3f(0.94, 0.18, 0.18)
            if pose["collision"]
            else Gf.Vec3f(0.13, 0.77, 0.37)
            for pose in poses
        ]
    )


def main() -> int:
    args = parse_args()
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": args.headless,
            "renderer": "RaytracedLighting",
            "width": args.width,
            "height": args.height,
            "multi_gpu": False,
            "max_gpu_count": 1,
            "disable_viewport_updates": not bool(args.capture_dir),
        }
    )

    import numpy as np
    from pxr import Gf, UsdGeom, UsdLux

    from isaacsim.core.api import World
    from isaacsim.core.api.objects import FixedCuboid
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.robot.manipulators.examples.franka import KinematicsSolver
    from isaacsim.storage.native import get_assets_root_path

    trajectory_payload = json.loads(Path(args.trajectory).read_text(encoding="utf-8"))
    ai_payload = json.loads(Path(args.ai_input).read_text(encoding="utf-8"))
    poses = trajectory_payload["trajectory"]
    if args.max_poses is not None:
        poses = poses[: max(0, args.max_poses)]
    if not poses:
        raise ValueError("trajectory contains no poses")

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = world.stage

    assets_root = get_assets_root_path()
    if not assets_root:
        raise RuntimeError("Isaac Sim assets root is unavailable")
    franka_usd = assets_root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
    franka_prim = add_reference_to_stage(franka_usd, "/World/Franka")
    franka_prim.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
    franka_prim.GetVariantSet("Mesh").SetVariantSelection("Quality")
    robot = world.scene.add(SingleArticulation("/World/Franka", name="public_franka"))

    world.scene.add(
        FixedCuboid(
            prim_path="/World/RobotPedestal",
            name="robot_pedestal",
            position=np.array([0.75, 0.0, 0.425]),
            scale=np.array([0.40, 0.45, 0.85]),
            color=np.array([0.12, 0.18, 0.28]),
        )
    )
    make_human_mesh(stage, ai_payload["mesh"])
    make_trajectory(stage, poses)

    marker = UsdGeom.Sphere.Define(stage, "/World/CurrentTarget")
    marker.CreateRadiusAttr(0.027)
    marker.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.68, 0.05)])
    marker_translate = marker.AddTranslateOp()
    marker_translate.Set(Gf.Vec3d(*poses[0]["position_m"]))

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(900.0)
    key = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    key.CreateIntensityAttr(1800.0)
    key.CreateAngleAttr(0.6)

    world.reset()
    base_position = np.array([0.75, 0.0, 0.85], dtype=np.float64)
    base_orientation = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    robot.set_world_pose(base_position, base_orientation)
    solver = KinematicsSolver(robot, end_effector_frame_name="right_gripper")
    solver.get_kinematics_solver().set_robot_base_pose(base_position, base_orientation)

    writer = None
    capture_indices = set()
    if args.capture_dir:
        import omni.replicator.core as rep

        capture_dir = Path(args.capture_dir).resolve()
        capture_dir.mkdir(parents=True, exist_ok=True)
        camera = rep.create.camera(
            position=(3.0, -2.9, 2.15),
            look_at=(1.05, 0.0, 1.0),
            focal_length=44,
        )
        render_product = rep.create.render_product(camera, (args.width, args.height))
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(output_dir=str(capture_dir), rgb=True)
        writer.attach([render_product])
        capture_indices = set(range(0, len(poses), max(1, args.capture_stride)))

    for _ in range(20):
        world.step(render=True)

    execution = []
    controller = robot.get_articulation_controller()
    for local_index, pose in enumerate(poses):
        target = np.asarray(pose["position_m"], dtype=np.float64)
        action, success = solver.compute_inverse_kinematics(target, None)
        if success:
            controller.apply_action(action)
        marker_translate.Set(Gf.Vec3d(*target.tolist()))
        for _ in range(max(1, args.settle_frames)):
            world.step(render=writer is None)
        if writer is not None and local_index in capture_indices:
            rep.orchestrator.step(delta_time=0.0, rt_subframes=2, pause_timeline=False)
        execution.append(
            {
                "trajectory_index": int(pose["index"]),
                "ik_success": bool(success),
                "collision_proxy": bool(pose["collision"]),
            }
        )
        if local_index % 50 == 0 or local_index + 1 == len(poses):
            print(f"pose {local_index + 1}/{len(poses)} ik_success={success}", flush=True)

    if writer is not None:
        import omni.replicator.core as rep

        writer.detach()
        rep.orchestrator.wait_until_complete()

    output = Path(args.execution_report)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(
            {
                "schema_version": "pai2026_franka_demo_v1",
                "private_platform_used": False,
                "collision_avoidance": False,
                "position_only_ik": True,
                "pose_count": len(execution),
                "ik_success_count": sum(row["ik_success"] for row in execution),
                "captured_frame_count": len(capture_indices),
                "execution": execution,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(output)
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
