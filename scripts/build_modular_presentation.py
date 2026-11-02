#!/usr/bin/env python3
"""Build replaceable 00-07 clips and concatenate the final submission video."""

from __future__ import annotations

import argparse
import os
import signal
import socket
import subprocess
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / "web"
FFMPEG = ROOT / ".venv/lib/python3.12/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2"
DEFAULT_OP = ROOT / "output/video/source/PAI応用1_最終課題_OP.mp4"
DEFAULT_ED = ROOT / "output/video/source/PAI応用1_最終課題_ED.mp4"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--op", type=Path, default=DEFAULT_OP)
    parser.add_argument("--ed", type=Path, default=DEFAULT_ED)
    parser.add_argument("--port", type=int, default=5174)
    parser.add_argument("--force", action="store_true", help="Regenerate existing clips.")
    parser.add_argument(
        "--external-server",
        action="store_true",
        help="Use an already-running Vite server on --port.",
    )
    return parser.parse_args()


def run(*args: str | Path, cwd: Path = ROOT) -> None:
    command = [str(value) for value in args]
    print("+", " ".join(command), flush=True)
    subprocess.run(command, cwd=cwd, check=True)


def normalize(source: Path, destination: Path) -> None:
    run(
        FFMPEG, "-y", "-i", source, "-an", "-vf",
        "scale=1920:1080:force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:(oh-ih)/2,fps=30,format=yuv420p",
        "-c:v", "libx264", "-preset", "medium", "-crf", "18", "-movflags", "+faststart",
        "-metadata", "title=PAI最終課題_TADAMASA_NDA", "-metadata", "artist=TADAMASA_NDA",
        destination,
    )


def wait_for_server(port: int, timeout_s: float = 45.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=1.0):
                return
        except OSError:
            time.sleep(0.25)
    raise TimeoutError(f"Vite did not become ready on port {port}")


def main() -> int:
    args = parse_args()
    segments = ROOT / "output/video/segments"
    posters = ROOT / "output/video/posters"
    final_dir = ROOT / "output/video/final"
    segments.mkdir(parents=True, exist_ok=True)
    posters.mkdir(parents=True, exist_ok=True)
    final_dir.mkdir(parents=True, exist_ok=True)

    baseline_scene = ROOT / "output/video/source/private-ai-scene-baseline.json"
    private_scene = WEB / "public/data/private-ai-scene.json"
    if baseline_scene.exists():
        run(
            "python3",
            ROOT / "scripts/densify_web_scene_controls.py",
            "--input", baseline_scene,
            "--output", private_scene,
            "--scan-speed-scale", "2",
            "--maximum-position-step-mm", "2",
            "--maximum-orientation-step-deg", "0.5",
            "--maximum-base-step-mm", "2",
            "--maximum-base-yaw-step-deg", "0.5",
            "--maximum-joint-step-deg", "0.5",
            "--maximum-lift-step-mm", "2",
        )
    run(ROOT / ".venv/bin/python", ROOT / "scripts/prepare_presentation_assets.py")
    run("npm", "run", "typecheck", cwd=WEB)
    op_clip = segments / "00_op.mp4"
    ed_clip = segments / "07_ed.mp4"
    if args.force or not op_clip.exists():
        normalize(args.op, op_clip)
    if args.force or not ed_clip.exists():
        normalize(args.ed, ed_clip)

    server_log = None
    server = None
    if not args.external_server:
        server_log = (ROOT / "output/video/vite-presentation.log").open("w", encoding="utf-8")
        server = subprocess.Popen(
            ["npm", "run", "dev", "--", "--host", "127.0.0.1", "--port", str(args.port), "--strictPort"],
            cwd=WEB,
            stdout=server_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    base = f"http://127.0.0.1:{args.port}/"
    try:
        wait_for_server(args.port)
        presentation = WEB / "scripts/record-presentation.mjs"
        section = WEB / "scripts/record-section.mjs"
        intro_clip = segments / "01_project_overview.mp4"
        subject_clip = segments / "02_subject_orbit.mp4"
        pipeline_clip = segments / "03_ai_pipeline.mp4"
        overview_clip = segments / "04_robot_overview_2x.mp4"
        closeup_clip = segments / "05_detector_closeup_2x.mp4"
        results_clip = segments / "06_results.mp4"
        if args.force or not intro_clip.exists():
            run("node", presentation, f"{base}?presentation=intro", intro_clip, posters / "01_project_overview.png", cwd=WEB)
        if args.force or not subject_clip.exists():
            run("node", presentation, f"{base}?presentation=subject", subject_clip, posters / "02_subject.png", cwd=WEB)
        if args.force or not pipeline_clip.exists():
            run("node", presentation, f"{base}?presentation=pipeline", pipeline_clip, posters / "03_pipeline.png", cwd=WEB)
        if args.force or not overview_clip.exists():
            run(
                "node", section,
                f"{base}?scene=data/private-ai-scene.json&capture=1&view=overview&speed=2",
                overview_clip, posters / "04_overview.png", "overview", "2", cwd=WEB,
            )
        if args.force or not closeup_clip.exists():
            run(
                "node", section,
                f"{base}?scene=data/private-ai-scene.json&capture=1&view=detector&speed=2",
                closeup_clip, posters / "05_closeup.png", "detector", "2", cwd=WEB,
            )
        if args.force or not results_clip.exists():
            run("node", presentation, f"{base}?presentation=metrics", results_clip, posters / "06_results.png", cwd=WEB)
    finally:
        if server is not None and server.poll() is None:
            os.killpg(server.pid, signal.SIGTERM)
            try:
                server.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(server.pid, signal.SIGKILL)
        if server_log is not None:
            server_log.close()

    ordered = [
        "00_op.mp4",
        "01_project_overview.mp4",
        "02_subject_orbit.mp4",
        "03_ai_pipeline.mp4",
        "04_robot_overview_2x.mp4",
        "05_detector_closeup_2x.mp4",
        "06_results.mp4",
        "07_ed.mp4",
    ]
    manifest = segments / "concat.txt"
    manifest.write_text("".join(f"file '{name}'\n" for name in ordered), encoding="utf-8")
    final = final_dir / "PAI最終課題_TADAMASA_NDA.mp4"
    run(FFMPEG, "-y", "-f", "concat", "-safe", "0", "-i", manifest, "-c", "copy", "-movflags", "+faststart", final)
    print(f"\nSegments: {segments}\nFinal: {final}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
