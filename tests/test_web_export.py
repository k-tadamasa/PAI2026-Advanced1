import numpy as np
from scipy.spatial.transform import Rotation

from pai_body_screening.collision import (
    DetectorGripperMeshModel,
    evaluate_detector_gripper_collisions,
)
from pai_body_screening.iftomm import IftommConfig, generate_trajectory
from pai_body_screening.web_export import (
    _SCREENING_RIGHT_ARM_RAD,
    _ffw_detector_control_pose,
    _ffw_world_control_pose_and_jacobian,
    build_web_scene,
)


def test_web_scene_has_monotonic_coverage_and_mobile_state(
    demo_mesh,
    collision_model: DetectorGripperMeshModel,
):
    config = IftommConfig(target_coverage_ratio=0.90)
    result = generate_trajectory(demo_mesh, config)
    collisions = evaluate_detector_gripper_collisions(
        demo_mesh,
        result.smoothed_points,
        result.smoothed_quaternions_wxyz,
        collision_model,
    )
    scene = build_web_scene(demo_mesh, result, collisions, config)

    assert scene["schema_version"] == "pai2026_web_scene_v7"
    assert scene["scope"]["robot_model"] == "ffw_sg2_mecanum"
    assert scene["scope"]["collision_model"] == "md_gripper_triangle_mesh"
    assert scene["parameters"]["lane_count"] == 4
    assert scene["parameters"]["lane_spacing_scale"] == 1.5
    assert scene["parameters"]["central_lane_gap_scale"] == 1.2
    assert scene["parameters"]["flip_md_at_center"] is True
    assert scene["robot"]["ik_solver"] == "TORM path-wise whole-body IK"
    torm = scene["robot"]["torm"]
    assert torm["optimized_knot_count"] < torm["dense_pose_count"]
    assert torm["final_smoothness_cost"] <= torm["initial_smoothness_cost"]
    assert torm["maximum_md_position_deviation_m"] <= 0.0005
    assert torm["maximum_md_orientation_deviation_rad"] <= np.deg2rad(0.5)
    assert torm["position_tracking_priority"] == "hard"
    assert torm["maximum_requested_md_position_error_m"] <= 1.0e-8
    # The anonymous coarse demo mesh can require a relaxed wrist orientation
    # at its synthetic lane transitions; position remains the hard task.
    assert torm["maximum_requested_md_orientation_error_rad"] <= np.deg2rad(12.0)
    assert not torm["collision_gradient_enabled"]
    assert scene["parameters"]["scan_speed_scale"] == 2.0
    assert np.isclose(scene["parameters"]["sample_period_s"], 0.10)
    assert np.isclose(
        scene["parameters"]["maximum_control_position_step_m"], 0.002
    )
    assert np.isclose(
        scene["parameters"]["maximum_control_joint_step_deg"], 0.5
    )
    assert scene["metrics"]["source_trajectory_pose_count"] == len(
        result.smoothed_points
    )
    assert len(scene["trajectory"]) > len(result.smoothed_points)
    assert scene["metrics"]["trajectory_pose_count"] == len(scene["trajectory"])
    source_duration = (len(result.smoothed_points) - 1) * 0.10
    assert np.isclose(scene["trajectory"][-1]["robot"]["time_s"], source_duration / 2)
    coverage = np.asarray(
        [frame["coverage_ratio"] for frame in scene["trajectory"]]
    )
    assert np.all(np.diff(coverage) >= -1e-12)
    assert coverage[-1] > 0.0
    robot_states = [frame["robot"] for frame in scene["trajectory"]]
    assert len(robot_states[0]["wheel_angle_rad"]) == 4
    assert len(robot_states[0]["left_arm_joint_rad"]) == 7
    assert len(robot_states[0]["right_arm_joint_rad"]) == 7
    assert all(-0.502 <= state["lift_position_m"] <= 0.002 for state in robot_states)
    assert all(
        np.isclose(state["gripper_joint_rad"], 0.30585790515379546)
        for state in robot_states
    )
    positions = np.asarray([frame["position_m"] for frame in scene["trajectory"]])
    orientations = np.asarray(
        [frame["orientation_wxyz"] for frame in scene["trajectory"]]
    )[:, [1, 2, 3, 0]]
    base_positions = np.asarray(
        [state["base_position_m"] for state in robot_states]
    )
    base_yaw = np.unwrap(
        np.asarray([state["base_yaw_rad"] for state in robot_states])
    )
    right_arm = np.asarray(
        [state["right_arm_joint_rad"] for state in robot_states]
    )
    lift = np.asarray([state["lift_position_m"] for state in robot_states])
    assert np.max(np.linalg.norm(np.diff(positions, axis=0), axis=1)) <= 0.002001
    orientation_steps = (
        Rotation.from_quat(orientations[:-1]).inv()
        * Rotation.from_quat(orientations[1:])
    ).magnitude()
    assert np.rad2deg(np.max(orientation_steps)) <= 0.501
    assert np.max(
        np.linalg.norm(np.diff(base_positions[:, :2], axis=0), axis=1)
    ) <= 0.00205
    assert np.rad2deg(np.max(np.abs(np.diff(base_yaw)))) <= 0.501
    assert np.rad2deg(
        np.max(np.abs(np.diff(right_arm, axis=0)))
    ) <= 0.501
    assert np.max(np.abs(np.diff(lift))) <= 0.00205
    orientation_errors = []
    height_errors = []
    position_errors = []
    for frame in scene["trajectory"]:
        state = frame["robot"]
        actual = _ffw_detector_control_pose(
            np.asarray(state["right_arm_joint_rad"]), state["lift_position_m"]
        )
        base_rotation = Rotation.from_euler(
            "z", state["base_yaw_rad"]
        ).as_matrix()
        world_rotation = base_rotation @ actual[:3, :3]
        world_position = np.asarray(state["base_position_m"]) + (
            base_rotation @ actual[:3, 3]
        )
        quaternion = np.asarray(frame["orientation_wxyz"])
        target_rotation = Rotation.from_quat(quaternion[[1, 2, 3, 0]]).as_matrix()
        orientation_errors.append(
            np.linalg.norm(
                    Rotation.from_matrix(target_rotation.T @ world_rotation).as_rotvec()
            )
        )
        height_errors.append(abs(actual[2, 3] - frame["position_m"][2]))
        position_errors.append(
            np.linalg.norm(world_position - frame["position_m"])
        )
    assert np.rad2deg(max(orientation_errors)) < 12.0
    assert max(position_errors) < 1e-8
    assert max(height_errors) < 1e-8


def test_whole_body_spatial_jacobian_matches_finite_difference():
    configuration = np.r_[0.2, -0.1, 0.3, -0.2, _SCREENING_RIGHT_ARM_RAD]
    pose, analytic = _ffw_world_control_pose_and_jacobian(configuration)
    numerical = np.zeros_like(analytic)
    epsilon = 1e-7
    for index in range(len(configuration)):
        displaced = configuration.copy()
        displaced[index] += epsilon
        displaced_pose, _ = _ffw_world_control_pose_and_jacobian(displaced)
        numerical[:3, index] = Rotation.from_matrix(
            displaced_pose[:3, :3] @ pose[:3, :3].T
        ).as_rotvec() / epsilon
        numerical[3:, index] = (
            displaced_pose[:3, 3] - pose[:3, 3]
        ) / epsilon
    np.testing.assert_allclose(analytic, numerical, atol=5e-8)
