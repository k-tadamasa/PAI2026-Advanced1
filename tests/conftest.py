from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

from pai_body_screening.models import MeshData


def _load_demo_module():
    path = Path(__file__).parents[1] / "scripts" / "create_synthetic_demo.py"
    spec = importlib.util.spec_from_file_location("create_synthetic_demo", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("failed to load synthetic demo helper")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture()
def demo_mesh() -> MeshData:
    payload = _load_demo_module().create_payload()
    return MeshData.from_arrays(
        payload["mesh"]["vertices"],
        payload["mesh"]["faces"],
        payload["mesh"]["face_part_ids"],
        payload["camera_position_m"],
    )
