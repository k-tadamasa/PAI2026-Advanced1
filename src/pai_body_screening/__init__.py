"""Public, publication-safe body-screening trajectory demo."""

from .iftomm import IftommConfig, TrajectoryResult, generate_trajectory
from .models import MeshData
from .roi import TARGET_PART_IDS, select_target_faces
from .web_export import (
    IdealMecanumConfig,
    build_web_scene,
    densify_web_scene_controls,
)

__all__ = [
    "IftommConfig",
    "IdealMecanumConfig",
    "MeshData",
    "TARGET_PART_IDS",
    "TrajectoryResult",
    "build_web_scene",
    "densify_web_scene_controls",
    "generate_trajectory",
    "select_target_faces",
]
