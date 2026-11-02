from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .collision import (
    DETECTOR_COMPONENT_ID,
    GRIPPER_COMPONENT_IDS,
    GRIPPER_OPEN_80_PERCENT_RAD,
    CollisionResult,
)
from .iftomm import IftommConfig, TrajectoryResult, covered_face_ids_at_pose
from .models import MeshData, face_geometry
from .torm_pathwise_optimizer import (
    TormPathwiseParameters,
    _recover_pose_path,
    optimize_torm_path,
)


@dataclass(frozen=True)
class IdealMecanumConfig:
    """Presentation kinematics for an ideal holonomic mobile base.

    The base follows the detector trajectory without wheel slip, acceleration
    limits, or collision avoidance. This is a visualization model rather than
    a claim of dynamically feasible robot motion.
    """

    # Exact URDF FK of metal_detector_control_point from base_link for the
    # saved body_screening screening posture and lift_joint=0. The posture
    # metrics JSON contains a Pinocchio-generated frame value, but that frame
    # is not the rendered URDF control point; using it visibly detaches the MD
    # inspection head from the planned ellipsoid by about 17 cm.
    detector_offset_from_base_m: tuple[float, float, float] = (
        0.5951499339628517,
        -0.14673909597410348,
        1.0637559122724243,
    )
    lift_min_m: float = -0.50
    lift_max_m: float = 0.0
    wheel_radius_m: float = 0.0759
    wheel_x_m: float = 0.319
    wheel_y_m: float = 0.2755
    sample_period_s: float = 0.10
    scan_speed_scale: float = 2.0
    # The exported simulation is subdivided by state-space motion, not by a
    # nominal video/display rate.  Every adjacent control pose must satisfy
    # all of these limits.
    maximum_control_position_step_m: float = 0.002
    maximum_control_orientation_step_deg: float = 0.5
    maximum_control_base_step_m: float = 0.002
    maximum_control_base_yaw_step_deg: float = 0.5
    maximum_control_joint_step_deg: float = 0.5
    maximum_control_lift_step_m: float = 0.002
    # Current body_screening TORM settings.  Collision remains post-evaluation
    # only in this public assignment, so TORM optimizes task-preserving whole-
    # body smoothness without an obstacle gradient.
    torm_maximum_iterations: int = 40
    torm_smoothness_weight: float = 0.10
    torm_learning_rate: float = 0.03
    torm_pose_damping: float = 1.0e-5
    torm_maximum_pose_recovery_steps: int = 24
    # TORM owns key configurations; its task-nullspace correction is expanded
    # to dense display samples and projected back to every prescribed pose.
    torm_knot_stride: int = 8


_SCREENING_LEFT_ARM_RAD = np.asarray(
    [
        0.5855215787887573,
        0.5629076361656189,
        1.308357834815979,
        -2.444450855255127,
        1.2329732179641724,
        0.8182942867279053,
        -0.15740178525447845,
    ],
    dtype=np.float64,
)
_SCREENING_RIGHT_ARM_RAD = np.asarray(
    [
        -0.7174805402755737,
        -0.9297605752944946,
        1.4275997877120972,
        -1.4617946147918701,
        0.09939765930175781,
        0.5262883305549622,
        -0.6870188117027283,
    ],
    dtype=np.float64,
)
_RIGHT_ARM_LOWER_RAD = np.asarray(
    [-3.14, -3.14, -3.14, -2.9361, -3.14, -1.57, -1.5804],
    dtype=np.float64,
)
_RIGHT_ARM_UPPER_RAD = np.asarray(
    [3.14, 0.0, 3.14, 1.0786, 3.14, 1.57, 1.8201],
    dtype=np.float64,
)
_GRIPPER_OPEN_80_PERCENT_RAD = GRIPPER_OPEN_80_PERCENT_RAD
_RIGHT_JOINT_ORIGINS_M = np.asarray(
    [
        (0.0, -0.1045, 0.0),
        (0.0, -0.123, 0.0),
        (0.0, 0.0, -0.165),
        (0.041004, 0.0, -0.135),
        (-0.041004, 0.0, -0.1489),
        (0.0, 0.0, -0.1041),
        (0.0, 0.0, -0.0885),
    ],
    dtype=np.float64,
)
_RIGHT_JOINT_AXES = np.asarray(
    [
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
    ],
    dtype=np.float64,
)


def _transform(
    rotation: np.ndarray | None = None,
    translation: np.ndarray | None = None,
) -> np.ndarray:
    value = np.eye(4, dtype=np.float64)
    if rotation is not None:
        value[:3, :3] = rotation
    if translation is not None:
        value[:3, 3] = translation
    return value


def _ffw_detector_control_pose(right_arm_rad: np.ndarray, lift_m: float) -> np.ndarray:
    """FFW-SG2 base-to-IFToMM-control transform copied from body_screening."""

    pose = _transform(translation=np.asarray([-0.0199, 0.0, 1.4316 + lift_m]))
    for origin, axis, value in zip(
        _RIGHT_JOINT_ORIGINS_M, _RIGHT_JOINT_AXES, right_arm_rad
    ):
        pose = pose @ _transform(
            rotation=Rotation.from_rotvec(axis * value).as_matrix(),
            translation=origin,
        )
    # arm_r_link7 -> metal_detector_link -> metal_detector_control_point.
    # The final RPY aligns local +X/+Y/+Z with inspection/long/scan.
    mount_rotation = Rotation.from_euler("x", np.pi).as_matrix() @ Rotation.from_euler(
        "xyz", (-0.5 * np.pi, -0.5 * np.pi, 0.0)
    ).as_matrix()
    return pose @ _transform(
        rotation=mount_rotation,
        translation=np.asarray([0.205701, 0.0, -0.220551]),
    )


def _ffw_detector_control_pose_and_jacobian(
    right_arm_rad: np.ndarray,
    lift_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return base-frame control pose and spatial Jacobian for lift + arm.

    The Jacobian rows are angular velocity followed by linear velocity, which
    matches the Drake spatial-Jacobian callback used by body_screening's TORM
    adapter. Columns are ``lift_joint`` then ``arm_r_joint1..7``.
    """

    pose = _transform(translation=np.asarray([-0.0199, 0.0, 1.4316 + lift_m]))
    joint_positions: list[np.ndarray] = []
    joint_axes: list[np.ndarray] = []
    for origin, axis, value in zip(
        _RIGHT_JOINT_ORIGINS_M, _RIGHT_JOINT_AXES, right_arm_rad, strict=True
    ):
        joint_positions.append(pose[:3, :3] @ origin + pose[:3, 3])
        joint_axes.append(pose[:3, :3] @ axis)
        pose = pose @ _transform(
            rotation=Rotation.from_rotvec(axis * value).as_matrix(),
            translation=origin,
        )

    mount_rotation = Rotation.from_euler("x", np.pi).as_matrix() @ Rotation.from_euler(
        "xyz", (-0.5 * np.pi, -0.5 * np.pi, 0.0)
    ).as_matrix()
    control_pose = pose @ _transform(
        rotation=mount_rotation,
        translation=np.asarray([0.205701, 0.0, -0.220551]),
    )
    control_position = control_pose[:3, 3]
    jacobian = np.zeros((6, 8), dtype=np.float64)
    jacobian[3:, 0] = np.asarray([0.0, 0.0, 1.0])
    for index, (axis, position) in enumerate(
        zip(joint_axes, joint_positions, strict=True), start=1
    ):
        jacobian[:3, index] = axis
        jacobian[3:, index] = np.cross(axis, control_position - position)
    return control_pose, jacobian


def _ffw_world_control_pose_and_jacobian(
    configuration: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return world pose/Jacobian for TORM's 11 active whole-body DoFs.

    DoF order is ``base_x, base_y, base_yaw, lift_joint, arm_r_joint1..7``.
    """

    values = np.asarray(configuration, dtype=np.float64)
    if values.shape != (11,):
        raise ValueError("whole-body configuration must contain 11 values")
    base_x, base_y, base_yaw, lift_m = values[:4]
    local_pose, local_jacobian = _ffw_detector_control_pose_and_jacobian(
        values[4:], float(lift_m)
    )
    base_rotation = Rotation.from_euler("z", float(base_yaw)).as_matrix()
    world_pose = np.eye(4, dtype=np.float64)
    world_pose[:3, :3] = base_rotation @ local_pose[:3, :3]
    local_position_world = base_rotation @ local_pose[:3, 3]
    world_pose[:3, 3] = np.asarray([base_x, base_y, 0.0]) + local_position_world

    jacobian = np.zeros((6, 11), dtype=np.float64)
    jacobian[3:, 0] = np.asarray([1.0, 0.0, 0.0])
    jacobian[3:, 1] = np.asarray([0.0, 1.0, 0.0])
    yaw_axis = np.asarray([0.0, 0.0, 1.0])
    jacobian[:3, 2] = yaw_axis
    jacobian[3:, 2] = np.cross(yaw_axis, local_position_world)
    jacobian[:3, 3:] = base_rotation @ local_jacobian[:3]
    jacobian[3:, 3:] = base_rotation @ local_jacobian[3:]
    return world_pose, jacobian


def _project_control_height_exactly(
    right_arm_rad: np.ndarray,
    lift_m: float,
    target_z_m: float,
    config: IdealMecanumConfig,
) -> tuple[np.ndarray, float]:
    """Project one MD control point onto its prescribed world height.

    Planar base X/Y are projected analytically elsewhere.  This active-set
    projection gives the remaining Z coordinate hard priority while choosing
    a lift/arm correction that changes orientation as little as possible.
    """

    values = np.concatenate(([lift_m], np.asarray(right_arm_rad, dtype=np.float64)))
    lower = np.concatenate(([config.lift_min_m], _RIGHT_ARM_LOWER_RAD))
    upper = np.concatenate(([config.lift_max_m], _RIGHT_ARM_UPPER_RAD))
    for _ in range(24):
        pose, jacobian = _ffw_detector_control_pose_and_jacobian(
            values[1:], float(values[0])
        )
        height_error = float(target_z_m - pose[2, 3])
        if abs(height_error) <= 1.0e-11:
            break

        height_jacobian = jacobian[5]
        blocked_low = (values <= lower + 1.0e-12) & (
            height_jacobian * height_error < 0.0
        )
        blocked_high = (values >= upper - 1.0e-12) & (
            height_jacobian * height_error > 0.0
        )
        free = ~(blocked_low | blocked_high)
        if not np.any(free):
            break

        angular = jacobian[:3, free]
        # Lift has no angular component and is therefore preferred whenever it
        # is inside its range. At a lift limit, the redundant arm direction
        # with the smallest first-order orientation change is selected.
        metric = 100.0 * angular.T @ angular + 1.0e-4 * np.eye(np.count_nonzero(free))
        direction = np.linalg.solve(metric, height_jacobian[free])
        denominator = float(height_jacobian[free] @ direction)
        if denominator <= 1.0e-14:
            break
        update = np.zeros_like(values)
        update[free] = direction * (height_error / denominator)
        update[0] = float(np.clip(update[0], -0.002, 0.002))
        update[1:] = np.clip(update[1:], -0.01, 0.01)
        candidate = np.clip(values + update, lower, upper)
        if float(np.linalg.norm(candidate - values)) <= 1.0e-14:
            break
        values = candidate

    final_pose = _ffw_detector_control_pose(values[1:], float(values[0]))
    final_error = abs(float(final_pose[2, 3] - target_z_m))
    if final_error > 1.0e-8:
        raise RuntimeError(
            "position-priority IK could not realize the prescribed MD height: "
            f"error={final_error:.9f} m"
        )
    return values[1:], float(values[0])


def _refine_orientation_at_fixed_height(
    right_arm_rad: np.ndarray,
    lift_m: float,
    base_yaw_rad: float,
    target_rotation: np.ndarray,
    target_z_m: float,
    config: IdealMecanumConfig,
) -> tuple[np.ndarray, float, float]:
    """Minimize MD orientation error without releasing the hard position task."""

    values = np.concatenate(
        ([lift_m], np.asarray(right_arm_rad, dtype=np.float64), [base_yaw_rad])
    )
    lower = np.concatenate(
        ([config.lift_min_m], _RIGHT_ARM_LOWER_RAD, [-np.pi])
    )
    upper = np.concatenate(
        ([config.lift_max_m], _RIGHT_ARM_UPPER_RAD, [np.pi])
    )

    def orientation_error(candidate: np.ndarray) -> np.ndarray:
        local_pose = _ffw_detector_control_pose(candidate[1:8], float(candidate[0]))
        base_rotation = Rotation.from_euler("z", float(candidate[8])).as_matrix()
        return Rotation.from_matrix(
            target_rotation @ (base_rotation @ local_pose[:3, :3]).T
        ).as_rotvec()

    for _ in range(48):
        local_pose, local_jacobian = _ffw_detector_control_pose_and_jacobian(
            values[1:8], float(values[0])
        )
        base_rotation = Rotation.from_euler("z", float(values[8])).as_matrix()
        error = Rotation.from_matrix(
            target_rotation @ (base_rotation @ local_pose[:3, :3]).T
        ).as_rotvec()
        error_norm = float(np.linalg.norm(error))
        if error_norm <= 1.0e-7:
            break

        angular_jacobian = np.zeros((3, 9), dtype=np.float64)
        angular_jacobian[:, :8] = base_rotation @ local_jacobian[:3]
        angular_jacobian[:, 8] = np.asarray([0.0, 0.0, 1.0])
        height_jacobian = np.r_[local_jacobian[5], 0.0]
        height_norm_squared = float(height_jacobian @ height_jacobian)
        if height_norm_squared <= 1.0e-14:
            break
        nullspace = np.eye(9) - np.outer(
            height_jacobian, height_jacobian
        ) / height_norm_squared
        task_jacobian = angular_jacobian @ nullspace
        update = nullspace @ task_jacobian.T @ np.linalg.solve(
            task_jacobian @ task_jacobian.T + 1.0e-5 * np.eye(3),
            error,
        )
        update[0] = float(np.clip(update[0], -0.005, 0.005))
        update[1:8] = np.clip(update[1:8], -0.04, 0.04)
        update[8] = float(np.clip(update[8], -0.04, 0.04))

        accepted = False
        for scale in (1.0, 0.5, 0.25, 0.125, 0.0625):
            candidate = np.clip(values + scale * update, lower, upper)
            try:
                projected_arm, projected_lift = _project_control_height_exactly(
                    candidate[1:8],
                    float(candidate[0]),
                    target_z_m,
                    config,
                )
            except RuntimeError:
                continue
            candidate[0] = projected_lift
            candidate[1:8] = projected_arm
            if float(np.linalg.norm(orientation_error(candidate))) < error_norm - 1.0e-10:
                values = candidate
                accepted = True
                break
        if not accepted:
            break

    return values[1:8], float(values[0]), float(values[8])


def _torm_refine_whole_body_path(
    initial_trajectory: np.ndarray,
    config: IdealMecanumConfig,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Apply current body_screening TORM path-wise IK to all scan knots.

    As in body_screening, the sequential IK result defines the exact task pose
    that TORM must preserve. The public assignment intentionally supplies no
    collision gradient: collisions are evaluated and coloured only after path
    generation, as required by the assignment scope.
    """

    seed_poses = [
        _ffw_world_control_pose_and_jacobian(configuration)[0]
        for configuration in initial_trajectory
    ]
    desired_positions = np.asarray([pose[:3, 3] for pose in seed_poses])
    desired_rotations = np.asarray([pose[:3, :3] for pose in seed_poses])

    def dense_pose_error(configuration: np.ndarray, index: int) -> np.ndarray:
        actual, _ = _ffw_world_control_pose_and_jacobian(configuration)
        orientation_error = Rotation.from_matrix(
            desired_rotations[index] @ actual[:3, :3].T
        ).as_rotvec()
        position_error = desired_positions[index] - actual[:3, 3]
        return np.concatenate((orientation_error, position_error))

    def dense_pose_jacobian(configuration: np.ndarray, _index: int) -> np.ndarray:
        return _ffw_world_control_pose_and_jacobian(configuration)[1]

    knot_indices = np.unique(
        np.concatenate(
            (
                np.arange(
                    0,
                    len(initial_trajectory),
                    config.torm_knot_stride,
                    dtype=np.int64,
                ),
                np.asarray([len(initial_trajectory) - 1], dtype=np.int64),
            )
        )
    )
    knot_trajectory = initial_trajectory[knot_indices]

    def knot_pose_error(configuration: np.ndarray, index: int) -> np.ndarray:
        return dense_pose_error(configuration, int(knot_indices[index]))

    def knot_pose_jacobian(configuration: np.ndarray, index: int) -> np.ndarray:
        return dense_pose_jacobian(configuration, int(knot_indices[index]))

    xy_margin = 2.0
    lower = np.concatenate(
        (
            np.min(initial_trajectory[:, :2], axis=0) - xy_margin,
            [-np.pi, config.lift_min_m],
            _RIGHT_ARM_LOWER_RAD,
        )
    )
    upper = np.concatenate(
        (
            np.max(initial_trajectory[:, :2], axis=0) + xy_margin,
            [np.pi, config.lift_max_m],
            _RIGHT_ARM_UPPER_RAD,
        )
    )
    maximum_step = np.asarray(
        [0.020, 0.020, 0.040, 0.020] + [0.050] * 7,
        dtype=np.float64,
    )
    result = optimize_torm_path(
        knot_trajectory,
        pose_error=knot_pose_error,
        pose_jacobian=knot_pose_jacobian,
        position_lower=lower,
        position_upper=upper,
        maximum_step=maximum_step,
        collision_cost_gradient=None,
        parameters=TormPathwiseParameters(
            maximum_iterations=config.torm_maximum_iterations,
            smoothness_weight=config.torm_smoothness_weight,
            collision_weight=0.0,
            learning_rate=config.torm_learning_rate,
            pose_damping=config.torm_pose_damping,
            maximum_pose_recovery_steps=config.torm_maximum_pose_recovery_steps,
            convergence_tolerance=1.0e-7,
        ),
    )
    knot_correction = result.trajectory - knot_trajectory
    dense_correction = PchipInterpolator(
        knot_indices,
        knot_correction,
        axis=0,
    )(np.arange(len(initial_trajectory), dtype=np.float64))
    refined_trajectory = np.minimum(
        np.maximum(initial_trajectory + dense_correction, lower), upper
    )
    refined_trajectory = _recover_pose_path(
        refined_trajectory,
        dense_pose_error,
        dense_pose_jacobian,
        lower,
        upper,
        maximum_step,
        TormPathwiseParameters(
            pose_damping=config.torm_pose_damping,
            maximum_pose_recovery_steps=config.torm_maximum_pose_recovery_steps,
            convergence_tolerance=1.0e-7,
        ),
    )
    refined_errors = np.asarray(
        [
            dense_pose_error(configuration, index)
            for index, configuration in enumerate(refined_trajectory)
        ]
    )
    maximum_orientation_error = float(
        np.linalg.norm(refined_errors[:, :3], axis=1).max(initial=0.0)
    )
    maximum_position_error = float(
        np.linalg.norm(refined_errors[:, 3:], axis=1).max(initial=0.0)
    )
    if maximum_position_error > 0.0005:
        raise RuntimeError(
            "TORM path-wise IK exceeded the 0.5 mm MD preservation bound: "
            f"{maximum_position_error:.6f} m"
        )
    if maximum_orientation_error > np.deg2rad(0.5):
        raise RuntimeError(
            "TORM path-wise IK exceeded the 0.5 degree MD preservation bound: "
            f"{np.rad2deg(maximum_orientation_error):.6f} deg"
        )
    diagnostics: dict[str, Any] = {
        "solver": "TORM two-stage path-wise whole-body IK",
        "adapter": "NumPy FFW-SG2 11-DoF analytic spatial Jacobian",
        "upstream_revision": "c19690af549eb362c3d2359c757a6dcc89ec65e7",
        "iterations": result.iterations,
        "converged": result.converged,
        "accepted_update_count": result.accepted_update_count,
        "dense_pose_count": len(initial_trajectory),
        "optimized_knot_count": len(knot_indices),
        "knot_stride": config.torm_knot_stride,
        "initial_smoothness_cost": result.initial_smoothness_cost,
        "final_smoothness_cost": result.final_smoothness_cost,
        "maximum_md_position_deviation_m": maximum_position_error,
        "maximum_md_orientation_deviation_rad": maximum_orientation_error,
        "collision_gradient_enabled": False,
    }
    return refined_trajectory, diagnostics


def _screening_robot_states(
    points: np.ndarray,
    orientations_wxyz: np.ndarray,
    config: IdealMecanumConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
    """Seed each pose, then solve path-wise 11-DoF tracking with TORM."""

    if orientations_wxyz.shape != (len(points), 4):
        raise ValueError("orientations_wxyz must have shape (N, 4)")

    # The primary seed preserves the requested detector orientation. Its tiny
    # height residue is projected exactly. Only poses that cannot be projected
    # within limits use the position-priority arm+yaw fallback below.
    orientation_seed = np.concatenate((_SCREENING_RIGHT_ARM_RAD, [0.0, 0.0]))
    orientation_lower = np.concatenate(
        (_RIGHT_ARM_LOWER_RAD, [config.lift_min_m, -np.pi])
    )
    orientation_upper = np.concatenate(
        (_RIGHT_ARM_UPPER_RAD, [config.lift_max_m, np.pi])
    )
    orientation_previous = orientation_seed.copy()
    orientation_previous_previous = orientation_seed.copy()

    seed = np.concatenate((_SCREENING_RIGHT_ARM_RAD, [0.0]))
    lower = np.concatenate((_RIGHT_ARM_LOWER_RAD, [-np.pi]))
    upper = np.concatenate((_RIGHT_ARM_UPPER_RAD, [np.pi]))
    seed_pose = _ffw_detector_control_pose(seed[:7], 0.0)
    configured_offset = np.asarray(config.detector_offset_from_base_m, dtype=np.float64)
    previous = seed.copy()
    previous_previous = seed.copy()
    previous_lift = 0.0
    previous_base_xy: np.ndarray | None = None
    previous_point_xy: np.ndarray | None = None
    base_xyz = np.zeros_like(points)
    right_arm = np.empty((len(points), 7), dtype=np.float64)
    lift = np.empty(len(points), dtype=np.float64)
    base_yaw = np.empty(len(points), dtype=np.float64)

    for index, (point, quaternion_wxyz) in enumerate(
        zip(points, orientations_wxyz, strict=True)
    ):
        target_z = float(point[2])
        target_rotation = Rotation.from_quat(
            np.asarray(quaternion_wxyz, dtype=np.float64)[[1, 2, 3, 0]]
        ).as_matrix()

        def resolved_pose(values: np.ndarray) -> tuple[np.ndarray, float]:
            zero_lift_pose = _ffw_detector_control_pose(values[:7], 0.0)
            required_lift = target_z - float(zero_lift_pose[2, 3])
            resolved_lift = float(
                np.clip(required_lift, config.lift_min_m, config.lift_max_m)
            )
            return _ffw_detector_control_pose(values[:7], resolved_lift), resolved_lift

        def base_xy_for_pose(yaw: float, detector_pose: np.ndarray) -> np.ndarray:
            local_xy = configured_offset[:2] + (
                detector_pose[:2, 3] - seed_pose[:2, 3]
            )
            base_rotation = Rotation.from_euler("z", yaw).as_matrix()
            return point[:2] - (base_rotation @ np.r_[local_xy, 0.0])[:2]

        def base_xy_for(values: np.ndarray, detector_pose: np.ndarray) -> np.ndarray:
            return base_xy_for_pose(float(values[7]), detector_pose)

        def residual(values: np.ndarray) -> np.ndarray:
            detector_pose, resolved_lift = resolved_pose(values)
            base_rotation = Rotation.from_euler("z", values[7]).as_matrix()
            rotation_error = Rotation.from_matrix(
                target_rotation.T @ base_rotation @ detector_pose[:3, :3]
            ).as_rotvec()
            continuity_weights = np.sqrt(
                np.asarray([1.0] * 7 + [15.0], dtype=np.float64)
            )
            maximum_prediction_step = np.asarray(
                [np.deg2rad(4.0)] * 7 + [np.deg2rad(4.0)],
                dtype=np.float64,
            )
            bounded_previous_step = np.clip(
                previous - previous_previous,
                -maximum_prediction_step,
                maximum_prediction_step,
            )
            predicted = previous + 0.75 * bounded_previous_step
            predicted = np.clip(predicted, lower, upper)
            acceleration_weights = np.sqrt(
                np.asarray([2.0] * 7 + [30.0], dtype=np.float64)
            )
            secondary = [
                0.01 * (values[:7] - _SCREENING_RIGHT_ARM_RAD),
                np.asarray([0.01 * resolved_lift, 0.01 * values[7]]),
                continuity_weights * (values - previous),
                np.asarray([np.sqrt(5.0) * (resolved_lift - previous_lift)]),
                acceleration_weights * (values - predicted),
            ]
            if previous_base_xy is not None and previous_point_xy is not None:
                expected_base_xy = previous_base_xy + (point[:2] - previous_point_xy)
                secondary.append(
                    np.sqrt(200.0)
                    * (base_xy_for(values, detector_pose) - expected_base_xy)
                )
            return np.concatenate(
                (
                    # Lexicographic approximation: position is the hard task;
                    # orientation is optimized only inside that position
                    # solution. This lets low thigh samples remain realizable
                    # at the lift limit by using the redundant arm posture.
                    [1.0e7 * (detector_pose[2, 3] - target_z)],
                    5000.0 * rotation_error,
                    *secondary,
                )
            )

        def orientation_first_residual(values: np.ndarray) -> np.ndarray:
            detector_pose = _ffw_detector_control_pose(values[:7], float(values[7]))
            base_rotation = Rotation.from_euler("z", values[8]).as_matrix()
            rotation_error = Rotation.from_matrix(
                target_rotation.T @ base_rotation @ detector_pose[:3, :3]
            ).as_rotvec()
            maximum_prediction_step = np.asarray(
                [np.deg2rad(4.0)] * 7 + [0.025, np.deg2rad(4.0)],
                dtype=np.float64,
            )
            predicted = orientation_previous + 0.75 * np.clip(
                orientation_previous - orientation_previous_previous,
                -maximum_prediction_step,
                maximum_prediction_step,
            )
            predicted = np.clip(predicted, orientation_lower, orientation_upper)
            secondary = [
                0.01 * (values[:7] - _SCREENING_RIGHT_ARM_RAD),
                np.asarray([0.01 * values[7], 0.01 * values[8]]),
                np.sqrt(np.asarray([1.0] * 7 + [5.0, 15.0]))
                * (values - orientation_previous),
                np.sqrt(np.asarray([2.0] * 7 + [12.0, 30.0]))
                * (values - predicted),
            ]
            if previous_base_xy is not None and previous_point_xy is not None:
                expected_base_xy = previous_base_xy + (point[:2] - previous_point_xy)
                secondary.append(
                    np.sqrt(200.0)
                    * (
                        base_xy_for_pose(float(values[8]), detector_pose)
                        - expected_base_xy
                    )
                )
            return np.concatenate(
                (
                    [1000.0 * (detector_pose[2, 3] - target_z)],
                    5000.0 * rotation_error,
                    *secondary,
                )
            )

        orientation_solution = least_squares(
            orientation_first_residual,
            orientation_previous,
            bounds=(orientation_lower, orientation_upper),
            max_nfev=150,
            ftol=1e-8,
            xtol=1e-8,
            gtol=1e-8,
        )
        candidate_arm = orientation_solution.x[:7]
        candidate_lift = float(orientation_solution.x[7])
        candidate_yaw = float(orientation_solution.x[8])
        try:
            candidate_arm, candidate_lift = _project_control_height_exactly(
                candidate_arm,
                candidate_lift,
                target_z,
                config,
            )
        except RuntimeError:
            fallback_solution = least_squares(
                residual,
                previous,
                bounds=(lower, upper),
                max_nfev=150,
                ftol=1e-8,
                xtol=1e-8,
                gtol=1e-8,
            )
            candidate_arm = fallback_solution.x[:7]
            candidate_yaw = float(fallback_solution.x[7])
            _, candidate_lift = resolved_pose(fallback_solution.x)

        candidate_arm, candidate_lift, candidate_yaw = (
            _refine_orientation_at_fixed_height(
                candidate_arm,
                candidate_lift,
                candidate_yaw,
                target_rotation,
                target_z,
                config,
            )
        )
        detector_pose = _ffw_detector_control_pose(candidate_arm, candidate_lift)
        previous_previous = previous.copy()
        previous = np.concatenate((candidate_arm, [candidate_yaw]))
        orientation_previous_previous = orientation_previous.copy()
        orientation_previous = np.concatenate(
            (candidate_arm, [candidate_lift, candidate_yaw])
        )
        # The saved body_screening control-point metric is authoritative for
        # the seed XY. FK deltas preserve the same reference while the arm bends.
        base_xyz[index, :2] = base_xy_for_pose(candidate_yaw, detector_pose)
        right_arm[index] = candidate_arm
        lift[index] = candidate_lift
        base_yaw[index] = candidate_yaw
        previous_lift = candidate_lift
        previous_base_xy = base_xyz[index, :2].copy()
        previous_point_xy = point[:2].copy()

    initial_whole_body = np.column_stack(
        (base_xyz[:, :2], base_yaw, lift, right_arm)
    )
    refined_whole_body, torm_diagnostics = _torm_refine_whole_body_path(
        initial_whole_body,
        config,
    )
    base_xyz[:, :2] = refined_whole_body[:, :2]
    base_yaw = refined_whole_body[:, 2]
    lift = refined_whole_body[:, 3]
    right_arm = refined_whole_body[:, 4:]

    # Give the requested control-point position hard priority after TORM.
    # Lift/redundant-arm projection realizes Z, then the ideal holonomic base
    # translations realize X/Y analytically. Orientation remains the secondary
    # objective and is changed only if a lift limit makes that necessary.
    for index, point in enumerate(points):
        right_arm[index], lift[index] = _project_control_height_exactly(
            right_arm[index],
            float(lift[index]),
            float(point[2]),
            config,
        )
        target_rotation = Rotation.from_quat(
            np.asarray(orientations_wxyz[index], dtype=np.float64)[[1, 2, 3, 0]]
        ).as_matrix()
        right_arm[index], lift[index], base_yaw[index] = (
            _refine_orientation_at_fixed_height(
                right_arm[index],
                float(lift[index]),
                float(base_yaw[index]),
                target_rotation,
                float(point[2]),
                config,
            )
        )
        detector_pose = _ffw_detector_control_pose(right_arm[index], float(lift[index]))
        base_rotation = Rotation.from_euler("z", base_yaw[index]).as_matrix()
        base_xyz[index, :2] = point[:2] - (
            base_rotation @ detector_pose[:3, 3]
        )[:2]

    requested_position_errors: list[float] = []
    requested_orientation_errors: list[float] = []
    for index, (point, quaternion_wxyz) in enumerate(
        zip(points, orientations_wxyz, strict=True)
    ):
        configuration = np.concatenate(
            (
                base_xyz[index, :2],
                [base_yaw[index], lift[index]],
                right_arm[index],
            )
        )
        actual_pose, _ = _ffw_world_control_pose_and_jacobian(configuration)
        requested_rotation = Rotation.from_quat(
            np.asarray(quaternion_wxyz, dtype=np.float64)[[1, 2, 3, 0]]
        ).as_matrix()
        requested_position_errors.append(
            float(np.linalg.norm(actual_pose[:3, 3] - point))
        )
        requested_orientation_errors.append(
            float(
                Rotation.from_matrix(
                    requested_rotation @ actual_pose[:3, :3].T
                ).magnitude()
            )
        )
    maximum_requested_position_error = max(requested_position_errors, default=0.0)
    maximum_requested_orientation_error = max(
        requested_orientation_errors, default=0.0
    )
    if maximum_requested_position_error > 1.0e-8:
        raise RuntimeError(
            "position-priority IK did not exactly track the requested MD path: "
            f"error={maximum_requested_position_error:.9f} m"
        )
    torm_diagnostics.update(
        {
            "position_tracking_priority": "hard",
            "maximum_requested_md_position_error_m": maximum_requested_position_error,
            "maximum_requested_md_orientation_error_rad": maximum_requested_orientation_error,
        }
    )

    return base_xyz, right_arm, lift, base_yaw, torm_diagnostics


def _ideal_mobile_states(
    points: np.ndarray,
    normals: np.ndarray,
    orientations_wxyz: np.ndarray,
    config: IdealMecanumConfig,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    base_xyz, right_arm, lift, base_yaw, torm_diagnostics = _screening_robot_states(
        points, orientations_wxyz, config
    )

    wheel_angles = np.zeros((len(points), 4), dtype=np.float64)
    for index in range(1, len(points)):
        world_delta = base_xyz[index, :2] - base_xyz[index - 1, :2]
        previous_yaw = base_yaw[index - 1]
        cosine = float(np.cos(previous_yaw))
        sine = float(np.sin(previous_yaw))
        dx = cosine * world_delta[0] + sine * world_delta[1]
        dy = -sine * world_delta[0] + cosine * world_delta[1]
        dyaw = base_yaw[index] - previous_yaw
        turn = (config.wheel_x_m + config.wheel_y_m) * dyaw
        increments = np.asarray(
            [
                dx - dy - turn,
                dx + dy + turn,
                dx + dy - turn,
                dx - dy + turn,
            ],
            dtype=np.float64,
        ) / config.wheel_radius_m
        wheel_angles[index] = wheel_angles[index - 1] + increments

    states: list[dict[str, Any]] = []
    for index, (
        base,
        yaw,
        lift_position,
        wheel,
        normal,
        right_joint_values,
    ) in enumerate(
        zip(base_xyz, base_yaw, lift, wheel_angles, normals, right_arm, strict=True)
    ):
        normal_xy = np.asarray(normal[:2], dtype=np.float64)
        states.append(
            {
                "time_s": index * config.sample_period_s,
                "base_position_m": base.tolist(),
                "base_yaw_rad": float(yaw),
                "lift_position_m": float(lift_position),
                "left_arm_joint_rad": _SCREENING_LEFT_ARM_RAD.tolist(),
                "right_arm_joint_rad": right_joint_values.tolist(),
                "gripper_joint_rad": _GRIPPER_OPEN_80_PERCENT_RAD,
                "wheel_angle_rad": wheel.tolist(),
                "inspection_normal_yaw_rad": float(
                    np.arctan2(normal_xy[1], normal_xy[0])
                ),
                "inspection_normal_pitch_rad": float(
                    np.arctan2(normal[2], max(np.linalg.norm(normal_xy), 1e-12))
                ),
            }
        )
    return states, torm_diagnostics


def densify_web_scene_controls(
    scene: dict[str, Any],
    *,
    scan_speed_scale: float = 2.0,
    maximum_position_step_m: float = 0.002,
    maximum_orientation_step_deg: float = 0.5,
    maximum_base_step_m: float = 0.002,
    maximum_base_yaw_step_deg: float = 0.5,
    maximum_joint_step_deg: float = 0.5,
    maximum_lift_step_m: float = 0.002,
) -> dict[str, Any]:
    """Densify robot control poses without changing the prescribed MD path.

    TORM remains responsible for the source poses.  Each source interval is
    adaptively subdivided until MD translation/orientation and robot base,
    lift, and arm-joint changes all fall below explicit state-space limits.
    The ideal Mecanum base and lift are then projected so the rendered MD
    centre tracks every generated control position exactly.
    """

    if scan_speed_scale <= 0.0:
        raise ValueError("scan_speed_scale must be positive")
    limits = {
        "maximum_position_step_m": maximum_position_step_m,
        "maximum_orientation_step_deg": maximum_orientation_step_deg,
        "maximum_base_step_m": maximum_base_step_m,
        "maximum_base_yaw_step_deg": maximum_base_yaw_step_deg,
        "maximum_joint_step_deg": maximum_joint_step_deg,
        "maximum_lift_step_m": maximum_lift_step_m,
    }
    if any(value <= 0.0 for value in limits.values()):
        raise ValueError("all adaptive control-pose limits must be positive")

    payload = deepcopy(scene)
    source = payload.get("trajectory", [])
    if len(source) < 2:
        return payload
    if "source_trajectory_pose_count" in payload.get("metrics", {}):
        raise ValueError("scene is already densified; use the original scene")

    source_times = np.asarray(
        [frame["robot"]["time_s"] for frame in source], dtype=np.float64
    )
    if np.any(np.diff(source_times) <= 0.0):
        raise ValueError("source trajectory times must be strictly increasing")
    source_duration = float(source_times[-1] - source_times[0])
    target_duration = source_duration / scan_speed_scale

    def rows(path: tuple[str, ...]) -> np.ndarray:
        values = []
        for frame in source:
            value: Any = frame
            for key in path:
                value = value[key]
            values.append(value)
        return np.asarray(values, dtype=np.float64)

    source_points = rows(("position_m",))
    source_base_xyz = rows(("robot", "base_position_m"))
    source_base_yaw = np.unwrap(rows(("robot", "base_yaw_rad")))
    source_right_arm = rows(("robot", "right_arm_joint_rad"))
    source_lift = rows(("robot", "lift_position_m"))
    source_actual_rotation = Rotation.from_matrix(
        np.asarray(
            [
                Rotation.from_euler("z", yaw).as_matrix()
                @ _ffw_detector_control_pose(right_arm, lift)[:3, :3]
                for yaw, right_arm, lift in zip(
                    source_base_yaw,
                    source_right_arm,
                    source_lift,
                    strict=True,
                )
            ]
        )
    )
    orientation_steps = (
        source_actual_rotation[:-1].inv() * source_actual_rotation[1:]
    ).magnitude()

    # A small margin absorbs the non-linearity introduced when interpolated
    # arm states are projected back to exact MD positions.
    subdivision_safety_factor = 0.98
    required_subdivisions = np.maximum.reduce(
        [
            np.ceil(
                np.linalg.norm(np.diff(source_points, axis=0), axis=1)
                / (subdivision_safety_factor * maximum_position_step_m)
            ),
            np.ceil(
                np.degrees(orientation_steps)
                / (subdivision_safety_factor * maximum_orientation_step_deg)
            ),
            np.ceil(
                np.linalg.norm(np.diff(source_base_xyz[:, :2], axis=0), axis=1)
                / (subdivision_safety_factor * maximum_base_step_m)
            ),
            np.ceil(
                np.degrees(np.abs(np.diff(source_base_yaw)))
                / (subdivision_safety_factor * maximum_base_yaw_step_deg)
            ),
            np.ceil(
                np.degrees(
                    np.max(np.abs(np.diff(source_right_arm, axis=0)), axis=1)
                )
                / (subdivision_safety_factor * maximum_joint_step_deg)
            ),
            np.ceil(
                np.abs(np.diff(source_lift))
                / (subdivision_safety_factor * maximum_lift_step_m)
            ),
            np.ones(len(source) - 1, dtype=np.float64),
        ]
    ).astype(np.int64)

    left_values = [0]
    right_values = [1]
    alpha_values = [0.0]
    queried_source_time_values = [float(source_times[0])]
    for source_index, subdivision_count in enumerate(required_subdivisions):
        for subdivision_index in range(1, int(subdivision_count) + 1):
            fraction = subdivision_index / float(subdivision_count)
            left_values.append(source_index)
            right_values.append(source_index + 1)
            alpha_values.append(fraction)
            queried_source_time_values.append(
                float(
                    source_times[source_index]
                    + fraction
                    * (source_times[source_index + 1] - source_times[source_index])
                )
            )

    left = np.asarray(left_values, dtype=np.int64)
    right = np.asarray(right_values, dtype=np.int64)
    alpha = np.asarray(alpha_values, dtype=np.float64)
    queried_source_times = np.asarray(
        queried_source_time_values, dtype=np.float64
    )
    target_times = (queried_source_times - source_times[0]) / scan_speed_scale
    target_count = len(target_times)

    def interpolate(values: np.ndarray) -> np.ndarray:
        weight = alpha.reshape((-1,) + (1,) * (values.ndim - 1))
        return values[left] * (1.0 - weight) + values[right] * weight

    points = interpolate(source_points)
    normals = interpolate(rows(("surface_normal",)))
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1.0e-12)
    right_arm = interpolate(source_right_arm)
    left_arm = interpolate(rows(("robot", "left_arm_joint_rad")))
    gripper = interpolate(rows(("robot", "gripper_joint_rad")))
    base_yaw = interpolate(source_base_yaw)
    source_base_z = rows(("robot", "base_position_m"))[:, 2]
    base_z = interpolate(source_base_z)

    base_xyz = np.zeros((target_count, 3), dtype=np.float64)
    lift = np.zeros(target_count, dtype=np.float64)
    orientations_wxyz = np.zeros((target_count, 4), dtype=np.float64)
    for index in range(target_count):
        zero_lift_pose = _ffw_detector_control_pose(right_arm[index], 0.0)
        lift[index] = points[index, 2] - base_z[index] - zero_lift_pose[2, 3]
        # Dense interpolation can exceed the nominal endpoint slightly because
        # arm interpolation and the hard Z projection are evaluated in a
        # different order. Keep the MD position exact and permit at most 2 mm
        # of presentation-only numerical extrapolation around the lift limits.
        if not -0.502 <= lift[index] <= 0.002:
            raise RuntimeError(
                "retimed lift left the FFW-SG2 presentation range: "
                f"index={index}, lift={lift[index]:.6f} m"
            )
        control_pose = _ffw_detector_control_pose(right_arm[index], lift[index])
        base_rotation = Rotation.from_euler("z", base_yaw[index]).as_matrix()
        base_xyz[index, :2] = points[index, :2] - (
            base_rotation @ control_pose[:3, 3]
        )[:2]
        base_xyz[index, 2] = base_z[index]
        actual_rotation = base_rotation @ control_pose[:3, :3]
        quaternion_xyzw = Rotation.from_matrix(actual_rotation).as_quat()
        orientations_wxyz[index] = quaternion_xyzw[[3, 0, 1, 2]]

    wheel_radius = float(payload["robot"]["wheel_radius_m"])
    wheel_x, wheel_y = payload["robot"]["wheel_positions_m"]
    wheel_angles = np.zeros((target_count, 4), dtype=np.float64)
    wheel_angles[0] = rows(("robot", "wheel_angle_rad"))[0]
    for index in range(1, target_count):
        world_delta = base_xyz[index, :2] - base_xyz[index - 1, :2]
        cosine = float(np.cos(base_yaw[index - 1]))
        sine = float(np.sin(base_yaw[index - 1]))
        dx = cosine * world_delta[0] + sine * world_delta[1]
        dy = -sine * world_delta[0] + cosine * world_delta[1]
        dyaw = base_yaw[index] - base_yaw[index - 1]
        turn = (float(wheel_x) + float(wheel_y)) * dyaw
        wheel_angles[index] = wheel_angles[index - 1] + np.asarray(
            [dx - dy - turn, dx + dy + turn, dx + dy - turn, dx - dy + turn]
        ) / wheel_radius

    event_indices = np.where(alpha >= 1.0 - 1.0e-12, right, left)
    trajectory: list[dict[str, Any]] = []
    previous_event_index = -1
    for index, event_index in enumerate(event_indices):
        newly_inspected: list[int] = []
        for source_index in range(previous_event_index + 1, int(event_index) + 1):
            newly_inspected.extend(source[source_index]["newly_inspected_face_ids"])
        previous_event_index = max(previous_event_index, int(event_index))
        event = source[int(event_index)]
        normal_xy = normals[index, :2]
        trajectory.append(
            {
                "index": index,
                "position_m": points[index].tolist(),
                "surface_normal": normals[index].tolist(),
                "orientation_wxyz": orientations_wxyz[index].tolist(),
                "collision": bool(event["collision"]),
                "colliding_face_ids": event["colliding_face_ids"],
                "collision_components": event["collision_components"],
                "newly_inspected_face_ids": newly_inspected,
                "coverage_ratio": float(event["coverage_ratio"]),
                "robot": {
                    "time_s": float(target_times[index]),
                    "base_position_m": base_xyz[index].tolist(),
                    "base_yaw_rad": float(base_yaw[index]),
                    "lift_position_m": float(lift[index]),
                    "left_arm_joint_rad": left_arm[index].tolist(),
                    "right_arm_joint_rad": right_arm[index].tolist(),
                    "gripper_joint_rad": float(gripper[index]),
                    "wheel_angle_rad": wheel_angles[index].tolist(),
                    "inspection_normal_yaw_rad": float(
                        np.arctan2(normal_xy[1], normal_xy[0])
                    ),
                    "inspection_normal_pitch_rad": float(
                        np.arctan2(
                            normals[index, 2],
                            max(np.linalg.norm(normal_xy), 1.0e-12),
                        )
                    ),
                },
            }
        )

    control_time_steps = np.diff(target_times)
    parameters = payload["parameters"]
    parameters["source_sample_period_s"] = float(parameters["sample_period_s"])
    parameters["scan_speed_scale"] = scan_speed_scale
    parameters["maximum_control_position_step_m"] = maximum_position_step_m
    parameters["maximum_control_orientation_step_deg"] = (
        maximum_orientation_step_deg
    )
    parameters["maximum_control_base_step_m"] = maximum_base_step_m
    parameters["maximum_control_base_yaw_step_deg"] = maximum_base_yaw_step_deg
    parameters["maximum_control_joint_step_deg"] = maximum_joint_step_deg
    parameters["maximum_control_lift_step_m"] = maximum_lift_step_m
    parameters["control_time_step_median_s"] = float(
        np.median(control_time_steps)
    )
    parameters["control_pose_rate_average_hz"] = float(
        (target_count - 1) / target_duration
    )
    metrics = payload["metrics"]
    metrics["source_trajectory_pose_count"] = int(metrics["trajectory_pose_count"])
    metrics["trajectory_pose_count"] = target_count
    metrics["maximum_subdivisions_per_source_interval"] = int(
        np.max(required_subdivisions)
    )
    metrics["inspection_time_s"] = target_duration
    payload["scope"]["robot_motion"] = (
        "ideal_holonomic_mecanum_visualization_2x_scan"
    )
    payload["robot"]["scan_speed_scale"] = scan_speed_scale
    payload["robot"]["control_pose_count"] = target_count
    payload["trajectory"] = trajectory
    payload["schema_version"] = "pai2026_web_scene_v7"
    return payload


def build_web_scene(
    mesh: MeshData,
    result: TrajectoryResult,
    collisions: CollisionResult,
    config: IftommConfig,
    mobile_config: IdealMecanumConfig = IdealMecanumConfig(),
) -> dict[str, Any]:
    """Build the static public scene payload consumed by the web viewer."""

    target_ids = np.flatnonzero(result.target_face_mask).astype(np.int64)
    _, _, areas = face_geometry(mesh)
    target_area = float(np.sum(areas[target_ids]))
    radii = np.asarray(config.ellipsoid_radii_m, dtype=np.float64)
    inspected = np.zeros(len(mesh.faces), dtype=bool)
    coverage_frames: list[dict[str, Any]] = []

    for point, normal, scan_axis in zip(
        result.smoothed_points,
        result.smoothed_normals,
        result.smoothed_scan_axes,
        strict=True,
    ):
        covered_ids = covered_face_ids_at_pose(
            mesh,
            target_ids,
            point,
            normal,
            radii,
            scan_axis,
        )
        new_ids = covered_ids[~inspected[covered_ids]]
        inspected[new_ids] = True
        inspected_area = float(np.sum(areas[result.target_face_mask & inspected]))
        coverage_frames.append(
            {
                "newly_inspected_face_ids": new_ids.astype(int).tolist(),
                "coverage_ratio": inspected_area / target_area,
            }
        )

    mobile_states, torm_diagnostics = _ideal_mobile_states(
        result.smoothed_points,
        result.smoothed_normals,
        result.smoothed_quaternions_wxyz,
        mobile_config,
    )
    trajectory = []
    for index, (point, normal, quaternion, collision, coverage, mobile) in enumerate(
        zip(
            result.smoothed_points,
            result.smoothed_normals,
            result.smoothed_quaternions_wxyz,
            collisions.pose_collision,
            coverage_frames,
            mobile_states,
        )
    ):
        trajectory.append(
            {
                "index": index,
                "position_m": point.tolist(),
                "surface_normal": normal.tolist(),
                "orientation_wxyz": quaternion.tolist(),
                "collision": bool(collision),
                "colliding_face_ids": collisions.colliding_face_ids[index],
                "collision_components": (
                    collisions.colliding_components[index]
                    if collisions.colliding_components
                    else []
                ),
                **coverage,
                "robot": mobile,
            }
        )

    scene = {
        "schema_version": "pai2026_web_scene_v5",
        "title": "PAI Body Screening Viewer",
        "account": {"name": "TADAMASA_NDA", "id": "46962"},
        "method": "Kitahara_body_surface_raster_1p5_lane_spacing",
        "scope": {
            "front_only": config.front_only,
            "human3d_part_ids": sorted(config.part_ids),
            "collision_affects_planning": False,
            "robot_model": "ffw_sg2_mecanum",
            "robot_motion": "ideal_holonomic_mecanum_visualization",
            "collision_model": collisions.collision_model,
        },
        "parameters": {
            "standoff_m": config.standoff_m,
            "ellipsoid_radii_m": list(config.ellipsoid_radii_m),
            "target_coverage_ratio": config.target_coverage_ratio,
            "lane_count": config.lane_count,
            "lane_spacing_scale": config.lane_spacing_scale,
            "central_lane_gap_scale": config.central_lane_gap_scale,
            "flip_md_at_center": config.flip_md_at_center,
            "lane_half_width_m": config.lane_half_width_m,
            "local_fit_half_window": config.local_fit_half_window,
            "interpolation_gap_m": config.interpolation_gap_m,
            "loess_window_points": config.loess_window_points,
            "lane_reference_y_m": result.lane_reference_y_m.tolist(),
            "front_normal_max_angle_deg": config.front_normal_max_angle_deg,
            "front_width_fraction": config.front_width_fraction,
            "front_top_margin_m": config.front_top_margin_m,
            "maximum_orientation_step_deg": config.maximum_orientation_step_deg,
            "sample_period_s": mobile_config.sample_period_s,
        },
        "metrics": {
            "coverage_ratio": result.coverage_ratio,
            "target_area_m2": result.target_area_m2,
            "inspected_area_m2": result.inspected_area_m2,
            "path_length_m": result.path_length_m,
            "planning_time_s": result.planning_time_s,
            "control_point_count": len(result.control_points),
            "trajectory_pose_count": len(result.smoothed_points),
            "collision_pose_count": int(np.count_nonzero(collisions.pose_collision)),
            "detector_collision_pose_count": int(
                np.count_nonzero(
                    collisions.component_collision.get(
                        DETECTOR_COMPONENT_ID,
                        np.zeros(len(collisions.pose_collision), dtype=bool),
                    )
                )
            ),
            "gripper_collision_pose_count": int(
                np.count_nonzero(
                    np.logical_or.reduce(
                        [
                            collisions.component_collision.get(
                                component,
                                np.zeros(len(collisions.pose_collision), dtype=bool),
                            )
                            for component in GRIPPER_COMPONENT_IDS
                        ]
                    )
                )
            ),
            "stop_reason": result.stop_reason,
        },
        "human_mesh": {
            "vertices": mesh.vertices.tolist(),
            "faces": mesh.faces.astype(int).tolist(),
            "face_part_ids": mesh.face_part_ids.astype(int).tolist(),
            "target_face_ids": target_ids.astype(int).tolist(),
        },
        "detector": {
            "visual_asset": "assets/detector/MD.stl",
            "collision_asset": "assets/detector/MD_no_hole_poly.stl",
            "ellipsoid_radii_m": list(config.ellipsoid_radii_m),
            "standoff_m": config.standoff_m,
        },
        "robot": {
            "id": "ffw_sg2_mecanum",
            "motion_model": (
                "ideal Mecanum XY; bounded lift and continuous right-arm "
                "TORM path-wise 11-DoF IK; no dynamics"
            ),
            "ik_solver": "TORM path-wise whole-body IK",
            "torm": torm_diagnostics,
            "wheel_radius_m": mobile_config.wheel_radius_m,
            "wheel_positions_m": [mobile_config.wheel_x_m, mobile_config.wheel_y_m],
            "arm_visualization": (
                "body_screening saved bilateral screening posture; MD fixed to "
                "right wrist link; bounded lift and TORM whole-body pose IK"
            ),
            "collision_components": [
                DETECTOR_COMPONENT_ID,
                *GRIPPER_COMPONENT_IDS,
            ],
        },
        "collision": {
            "model": collisions.collision_model,
            "method": "triangle-triangle intersection after URDF forward kinematics",
            "contact_tolerance_m": 1.0e-6,
            "affects_planning": False,
        },
        "trajectory": trajectory,
    }
    return densify_web_scene_controls(
        scene,
        scan_speed_scale=mobile_config.scan_speed_scale,
        maximum_position_step_m=mobile_config.maximum_control_position_step_m,
        maximum_orientation_step_deg=(
            mobile_config.maximum_control_orientation_step_deg
        ),
        maximum_base_step_m=mobile_config.maximum_control_base_step_m,
        maximum_base_yaw_step_deg=(
            mobile_config.maximum_control_base_yaw_step_deg
        ),
        maximum_joint_step_deg=mobile_config.maximum_control_joint_step_deg,
        maximum_lift_step_m=mobile_config.maximum_control_lift_step_m,
    )
