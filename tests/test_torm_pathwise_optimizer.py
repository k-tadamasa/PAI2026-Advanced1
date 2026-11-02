import numpy as np

from pai_body_screening.torm_pathwise_optimizer import (
    TormPathwiseParameters,
    optimize_torm_path,
    second_difference_matrix,
    smoothness_cost,
)


def test_second_difference_does_not_penalize_affine_paths() -> None:
    path = np.column_stack(
        [np.linspace(-1.0, 1.0, 9), np.linspace(2.0, -3.0, 9)]
    )
    assert second_difference_matrix(len(path)).shape == (7, 9)
    assert smoothness_cost(path) < 1.0e-24


def test_torm_reduces_nullspace_roughness_and_preserves_task_path() -> None:
    point_count = 13
    target = np.linspace(-0.4, 0.7, point_count)
    nullspace_noise = np.asarray(
        [
            0.0,
            0.18,
            -0.16,
            0.20,
            -0.14,
            0.17,
            -0.18,
            0.16,
            -0.12,
            0.10,
            -0.08,
            0.06,
            0.0,
        ]
    )
    initial = np.column_stack(
        [
            target - nullspace_noise,
            nullspace_noise,
            np.linspace(0.1, 0.3, point_count),
        ]
    )

    def pose_error(configuration: np.ndarray, index: int) -> np.ndarray:
        return np.asarray([target[index] - configuration[0] - configuration[1]])

    def pose_jacobian(_configuration: np.ndarray, _index: int) -> np.ndarray:
        return np.asarray([[1.0, 1.0, 0.0]])

    result = optimize_torm_path(
        initial,
        pose_error=pose_error,
        pose_jacobian=pose_jacobian,
        position_lower=np.full(3, -2.0),
        position_upper=np.full(3, 2.0),
        maximum_step=np.full(3, 0.08),
        parameters=TormPathwiseParameters(
            maximum_iterations=30,
            smoothness_weight=1.0,
            learning_rate=0.08,
            maximum_pose_recovery_steps=3,
            convergence_tolerance=1.0e-10,
        ),
    )

    assert result.accepted_update_count > 0
    assert result.final_smoothness_cost < 0.05 * result.initial_smoothness_cost
    np.testing.assert_allclose(
        result.trajectory[:, 0] + result.trajectory[:, 1],
        target,
        atol=1e-7,
    )
