import numpy as np

from pai_body_screening.roi import central_front_bounds_mask, front_normal_mask


def _normal_at_xy_angle(angle_deg: float) -> np.ndarray:
    angle_rad = np.deg2rad(angle_deg)
    # A non-zero Z component proves that selection uses only the XY projection.
    return np.asarray([-np.cos(angle_rad), np.sin(angle_rad), 0.75])


def test_front_normal_mask_uses_strict_75_degree_xy_projection() -> None:
    normals = np.stack(
        [
            _normal_at_xy_angle(0.0),
            _normal_at_xy_angle(74.999),
            _normal_at_xy_angle(75.0),
            _normal_at_xy_angle(75.001),
            np.asarray([0.0, 0.0, 1.0]),
        ]
    )

    assert front_normal_mask(normals).tolist() == [True, True, False, False, False]


def test_central_front_bounds_keep_80_percent_and_lower_top_by_100_mm() -> None:
    centroids = np.asarray(
        [
            [0.0, -0.50, 0.50],
            [0.0, -0.40, 0.80],
            [0.0, 0.00, 0.80],
            [0.0, 0.40, 0.80],
            [0.0, 0.50, 1.00],
        ]
    )

    selected = central_front_bounds_mask(
        centroids,
        np.ones(5, dtype=bool),
        width_fraction=0.80,
        top_margin_m=0.100,
    )

    assert selected.tolist() == [False, True, True, True, False]
