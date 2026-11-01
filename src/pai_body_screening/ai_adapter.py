from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class AiInferenceSummary:
    pipeline: str
    body_model: str
    human_count: int
    human3d_point_count: int
    fitted_vertex_count: int
    fitted_face_count: int
    fitting_loss: float | None
    fitting_mode: str | None


def validate_human3d_segfit_result(payload: dict[str, Any]) -> AiInferenceSummary:
    """Validate the externally produced Human3D + SegFit inference contract."""

    required = (
        "vertices",
        "faces",
        "human3d_points",
        "human3d_part_ids",
    )
    missing = [key for key in required if key not in payload]
    if missing:
        raise ValueError(f"AI result is missing: {', '.join(missing)}")
    if len(payload["human3d_points"]) != len(payload["human3d_part_ids"]):
        raise ValueError("Human3D points and part IDs must have the same length")
    if not payload["vertices"] or not payload["faces"]:
        raise ValueError("SegFit mesh must not be empty")
    human_count = int(payload.get("human_count", 1))
    if human_count < 1:
        raise ValueError("Human3D did not detect a person")
    fitting_loss = payload.get("fitting_loss")
    return AiInferenceSummary(
        pipeline="Human3D semantic inference + SegFit SMPL-X fitting",
        body_model=str(payload.get("body_model", "unknown")),
        human_count=human_count,
        human3d_point_count=len(payload["human3d_points"]),
        fitted_vertex_count=len(payload["vertices"]),
        fitted_face_count=len(payload["faces"]),
        fitting_loss=float(fitting_loss) if fitting_loss is not None else None,
        fitting_mode=(
            str(payload["fitting_mode"])
            if payload.get("fitting_mode") is not None
            else None
        ),
    )


def prepare_planner_input(
    result_path: str | Path,
    face_part_ids_path: str | Path,
    output_path: str | Path,
    camera_position_m: list[float],
) -> AiInferenceSummary:
    """Package external inference output without redistributing model weights."""

    result = json.loads(Path(result_path).read_text(encoding="utf-8"))
    if not isinstance(result, dict):
        raise ValueError("AI result JSON must be an object")
    summary = validate_human3d_segfit_result(result)
    face_part_ids = json.loads(
        Path(face_part_ids_path).read_text(encoding="utf-8")
    )
    if isinstance(face_part_ids, dict):
        for key in ("face_part_ids", "human3d_face_part_ids", "part_ids"):
            if key in face_part_ids:
                face_part_ids = face_part_ids[key]
                break
    if not isinstance(face_part_ids, list):
        raise ValueError("face part mapping must be a JSON array")
    if len(face_part_ids) != len(result["faces"]):
        raise ValueError("face part mapping must contain one ID per fitted face")
    if len(camera_position_m) != 3:
        raise ValueError("camera_position_m must contain x, y, z")

    output = {
        "schema_version": "pai2026_human3d_segfit_input_v1",
        "ai_inference": asdict(summary),
        "camera_position_m": [float(value) for value in camera_position_m],
        "mesh": {
            "vertices": result["vertices"],
            "faces": result["faces"],
            "face_part_ids": face_part_ids,
        },
    }
    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(output, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return summary
