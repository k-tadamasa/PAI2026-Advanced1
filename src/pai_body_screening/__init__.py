"""Public, publication-safe body-screening trajectory demo."""

from .iftomm import IftommConfig, TrajectoryResult, generate_trajectory
from .models import MeshData
from .roi import TARGET_PART_IDS, select_target_faces

__all__ = [
    "IftommConfig",
    "MeshData",
    "TARGET_PART_IDS",
    "TrajectoryResult",
    "generate_trajectory",
    "select_target_faces",
]
