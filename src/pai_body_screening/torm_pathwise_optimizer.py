"""Algorithm-preserving TORM core for path-wise inverse kinematics.

This module ports the robot-independent optimization structure from TORM while
leaving forward kinematics and collision evaluation behind callbacks. The
adapter boundary is intentional: upstream TORM uses a KDL serial chain and
MoveIt distance fields, while this public demo uses a NumPy FFW-SG2 whole-body
model (ideal Mecanum X/Y/yaw, lift, and the seven right-arm joints).

Upstream project: https://github.com/cheulkang/TORM
Pinned reference revision: c19690af549eb362c3d2359c757a6dcc89ec65e7

Software License Agreement (BSD License)

Copyright (c) 2009, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of Willow Garage nor the names of its contributors may be
  used to endorse or promote products derived from this software without
  specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Original TORM implementation modified by Mincheul Kang. This callback-based
Python/NumPy port is adapted from the current body_screening implementation.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np


PoseError = Callable[[np.ndarray, int], np.ndarray]
PoseJacobian = Callable[[np.ndarray, int], np.ndarray]
CollisionCostGradient = Callable[[np.ndarray, int], tuple[float, np.ndarray]]


@dataclass(frozen=True)
class TormPathwiseParameters:
    """Numerical parameters for TORM's alternating two-stage update."""

    maximum_iterations: int = 24
    smoothness_weight: float = 0.10
    collision_weight: float = 1.0
    learning_rate: float = 0.04
    pose_damping: float = 1.0e-4
    preconditioner_ridge: float = 1.0e-4
    maximum_pose_recovery_steps: int = 4
    convergence_tolerance: float = 1.0e-7
    fixed_start_configuration: bool = False

    def validate(self) -> None:
        if self.maximum_iterations < 1:
            raise ValueError("maximum_iterations must be positive")
        if self.maximum_pose_recovery_steps < 1:
            raise ValueError("maximum_pose_recovery_steps must be positive")
        for name in (
            "smoothness_weight",
            "collision_weight",
            "learning_rate",
            "pose_damping",
            "preconditioner_ridge",
            "convergence_tolerance",
        ):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")


@dataclass(frozen=True)
class TormPathwiseResult:
    trajectory: np.ndarray
    iterations: int
    converged: bool
    initial_smoothness_cost: float
    final_smoothness_cost: float
    initial_collision_cost: float
    final_collision_cost: float
    initial_maximum_pose_error: float
    final_maximum_pose_error: float
    accepted_update_count: int


def second_difference_matrix(point_count: int) -> np.ndarray:
    """Return the acceleration finite-difference operator used by TORM."""

    if point_count < 2:
        raise ValueError("a path-wise trajectory requires at least two points")
    if point_count == 2:
        return np.zeros((0, 2), dtype=np.float64)
    matrix = np.zeros((point_count - 2, point_count), dtype=np.float64)
    rows = np.arange(point_count - 2)
    matrix[rows, rows] = 1.0
    matrix[rows, rows + 1] = -2.0
    matrix[rows, rows + 2] = 1.0
    return matrix


def smoothness_cost(
    trajectory: np.ndarray,
    difference: np.ndarray | None = None,
) -> float:
    values = np.asarray(trajectory, dtype=np.float64)
    if values.ndim != 2:
        raise ValueError("trajectory must be an N x DoF matrix")
    operator = (
        second_difference_matrix(len(values)) if difference is None else difference
    )
    acceleration = operator @ values
    return 0.5 * float(np.sum(acceleration * acceleration))


def _maximum_pose_error(trajectory: np.ndarray, pose_error: PoseError) -> float:
    return max(
        (
            float(np.linalg.norm(pose_error(point, index)))
            for index, point in enumerate(trajectory)
        ),
        default=0.0,
    )


def _collision_terms(
    trajectory: np.ndarray,
    callback: CollisionCostGradient | None,
) -> tuple[float, np.ndarray]:
    gradient = np.zeros_like(trajectory)
    if callback is None:
        return 0.0, gradient
    cost = 0.0
    for index, point in enumerate(trajectory):
        point_cost, point_gradient = callback(point, index)
        point_gradient = np.asarray(point_gradient, dtype=np.float64)
        if point_gradient.shape != point.shape:
            raise ValueError("collision gradient must have one value per active DoF")
        if not np.isfinite(point_cost) or not np.all(np.isfinite(point_gradient)):
            raise ValueError("collision callback returned a non-finite value")
        cost += float(point_cost)
        gradient[index] = point_gradient
    return cost, gradient


def _recover_pose_path(
    trajectory: np.ndarray,
    pose_error: PoseError,
    pose_jacobian: PoseJacobian,
    lower: np.ndarray,
    upper: np.ndarray,
    maximum_step: np.ndarray,
    parameters: TormPathwiseParameters,
) -> np.ndarray:
    recovered = trajectory.copy()
    for _ in range(parameters.maximum_pose_recovery_steps):
        largest_error = 0.0
        for index, point in enumerate(recovered):
            error = np.asarray(pose_error(point, index), dtype=np.float64).reshape(-1)
            jacobian = np.asarray(pose_jacobian(point, index), dtype=np.float64)
            if jacobian.shape != (len(error), recovered.shape[1]):
                raise ValueError(
                    "pose Jacobian shape does not match the task and active DoFs"
                )
            largest_error = max(largest_error, float(np.linalg.norm(error)))
            increment_lower = np.maximum(-maximum_step, lower - point)
            increment_upper = np.minimum(maximum_step, upper - point)
            increment = np.zeros(recovered.shape[1], dtype=np.float64)
            free = np.ones(recovered.shape[1], dtype=bool)
            for _active_set_iteration in range(recovered.shape[1] + 1):
                if not np.any(free):
                    break
                fixed = ~free
                residual = error - jacobian[:, fixed] @ increment[fixed]
                free_jacobian = jacobian[:, free]
                gram = free_jacobian @ free_jacobian.T
                gram.flat[:: len(gram) + 1] += parameters.pose_damping
                try:
                    task_step = np.linalg.solve(gram, residual)
                except np.linalg.LinAlgError:
                    task_step = np.linalg.pinv(gram, rcond=1.0e-10) @ residual
                free_increment = free_jacobian.T @ task_step
                free_indices = np.flatnonzero(free)
                below = free_increment < increment_lower[free_indices]
                above = free_increment > increment_upper[free_indices]
                saturated = below | above
                if not np.any(saturated):
                    increment[free_indices] = free_increment
                    break
                saturated_indices = free_indices[saturated]
                increment[saturated_indices] = np.where(
                    below[saturated],
                    increment_lower[saturated_indices],
                    increment_upper[saturated_indices],
                )
                free[saturated_indices] = False
            recovered[index] = np.minimum(
                np.maximum(point + increment, lower), upper
            )
        if largest_error <= parameters.convergence_tolerance:
            break
    return recovered


def optimize_torm_path(
    initial_trajectory: np.ndarray,
    *,
    pose_error: PoseError,
    pose_jacobian: PoseJacobian,
    position_lower: np.ndarray,
    position_upper: np.ndarray,
    maximum_step: np.ndarray,
    collision_cost_gradient: CollisionCostGradient | None = None,
    parameters: TormPathwiseParameters | None = None,
) -> TormPathwiseResult:
    """Optimize all path configurations with TORM's two-stage structure.

    Stage one applies the CHOMP/TORM smoothness metric and optional collision
    gradient over the complete trajectory. Stage two projects every knot back
    to its prescribed task pose using damped Jacobian inverse kinematics.
    """

    settings = parameters or TormPathwiseParameters()
    settings.validate()
    trajectory = np.asarray(initial_trajectory, dtype=np.float64).copy()
    if trajectory.ndim != 2 or trajectory.shape[0] < 2 or trajectory.shape[1] < 1:
        raise ValueError(
            "initial_trajectory must contain at least two configurations"
        )
    if not np.all(np.isfinite(trajectory)):
        raise ValueError("initial_trajectory contains non-finite values")
    dof_count = trajectory.shape[1]
    lower = np.broadcast_to(
        np.asarray(position_lower, dtype=np.float64), (dof_count,)
    ).copy()
    upper = np.broadcast_to(
        np.asarray(position_upper, dtype=np.float64), (dof_count,)
    ).copy()
    step_limit = np.broadcast_to(
        np.asarray(maximum_step, dtype=np.float64), (dof_count,)
    ).copy()
    if np.any(lower > upper) or np.any(step_limit <= 0.0):
        raise ValueError("invalid position bounds or maximum_step")
    if np.any(trajectory < lower) or np.any(trajectory > upper):
        raise ValueError("initial trajectory violates position bounds")
    fixed_start = trajectory[0].copy()

    difference = second_difference_matrix(len(trajectory))
    metric = difference.T @ difference
    preconditioner = np.linalg.inv(
        metric + settings.preconditioner_ridge * np.eye(len(trajectory))
    )

    initial_smoothness = smoothness_cost(trajectory, difference)
    initial_collision, _ = _collision_terms(
        trajectory, collision_cost_gradient
    )
    initial_pose_error = _maximum_pose_error(trajectory, pose_error)
    accepted_updates = 0
    converged = False
    iteration_count = 0

    def objective(path: np.ndarray) -> tuple[float, float, float]:
        path_smoothness = smoothness_cost(path, difference)
        path_collision, _ = _collision_terms(path, collision_cost_gradient)
        path_pose = sum(
            float(np.dot(error, error))
            for index, point in enumerate(path)
            for error in [
                np.asarray(pose_error(point, index), dtype=np.float64)
            ]
        )
        total = (
            settings.smoothness_weight * path_smoothness
            + settings.collision_weight * path_collision
            + path_pose
        )
        return total, path_smoothness, path_collision

    current_objective, _, _ = objective(trajectory)
    for iteration in range(settings.maximum_iterations):
        iteration_count = iteration + 1
        smoothness_gradient = metric @ trajectory
        _, collision_gradient = _collision_terms(
            trajectory, collision_cost_gradient
        )
        combined_gradient = (
            settings.smoothness_weight * smoothness_gradient
            + settings.collision_weight * collision_gradient
        )
        preconditioned_gradient = preconditioner @ combined_gradient
        nullspace_gradient = np.empty_like(preconditioned_gradient)
        for index, (point, point_gradient) in enumerate(
            zip(trajectory, preconditioned_gradient, strict=True)
        ):
            jacobian = np.asarray(pose_jacobian(point, index), dtype=np.float64)
            if jacobian.ndim != 2 or jacobian.shape[1] != trajectory.shape[1]:
                raise ValueError(
                    "pose Jacobian must have one column per active DoF"
                )
            gram = jacobian @ jacobian.T
            gram.flat[:: len(gram) + 1] += settings.pose_damping
            task_component = jacobian @ point_gradient
            try:
                multiplier = np.linalg.solve(gram, task_component)
            except np.linalg.LinAlgError:
                multiplier = np.linalg.pinv(gram, rcond=1.0e-10) @ task_component
            nullspace_gradient[index] = point_gradient - jacobian.T @ multiplier
        proposed_increment = -settings.learning_rate * nullspace_gradient
        proposed_increment = np.minimum(
            np.maximum(proposed_increment, -step_limit[None, :]),
            step_limit[None, :],
        )
        if settings.fixed_start_configuration:
            proposed_increment[0] = 0.0
        update_norm = float(np.max(np.abs(proposed_increment), initial=0.0))
        if update_norm <= settings.convergence_tolerance:
            converged = True
            break

        accepted = False
        scale = 1.0
        while scale >= 1.0 / 64.0:
            candidate = np.minimum(
                np.maximum(trajectory + scale * proposed_increment, lower), upper
            )
            candidate = _recover_pose_path(
                candidate,
                pose_error,
                pose_jacobian,
                lower,
                upper,
                step_limit,
                settings,
            )
            if settings.fixed_start_configuration:
                candidate[0] = fixed_start
            candidate_objective, _, _ = objective(candidate)
            if candidate_objective <= current_objective + 1.0e-12:
                trajectory = candidate
                improvement = current_objective - candidate_objective
                current_objective = candidate_objective
                accepted_updates += 1
                accepted = True
                if improvement <= settings.convergence_tolerance:
                    converged = True
                break
            scale *= 0.5
        if not accepted or converged:
            converged = converged or not accepted
            break

    final_collision, _ = _collision_terms(trajectory, collision_cost_gradient)
    return TormPathwiseResult(
        trajectory=trajectory,
        iterations=iteration_count,
        converged=converged,
        initial_smoothness_cost=initial_smoothness,
        final_smoothness_cost=smoothness_cost(trajectory, difference),
        initial_collision_cost=initial_collision,
        final_collision_cost=final_collision,
        initial_maximum_pose_error=initial_pose_error,
        final_maximum_pose_error=_maximum_pose_error(trajectory, pose_error),
        accepted_update_count=accepted_updates,
    )
