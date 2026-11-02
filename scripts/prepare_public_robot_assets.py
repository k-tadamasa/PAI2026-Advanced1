#!/usr/bin/env python3
"""Prepare licensed browser assets for the public FFW-SG2 Mecanum viewer.

Sources must be explicit local checkouts of the public upstream repositories.
The script strips the original FFW mobile base and collision meshes, keeps the
official upper-body visual hierarchy, and copies the official Ridgeback base
meshes. It never writes to either source checkout.
"""

from __future__ import annotations

import argparse
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


def _copy(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def _ffw_source_paths(source: Path) -> tuple[Path, Path, Path]:
    """Resolve either an upstream ai_worker checkout or its pinned asset snapshot."""

    upstream_package = source / "ffw_description"
    candidates = (
        upstream_package / "urdf/ffw_sg2_follower/ffw_sg2_follower.urdf",
        upstream_package / "urdf/ffw_sg2_rev1_follower/ffw_sg2_follower.urdf",
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate, upstream_package, source / "LICENSE"

    snapshot_urdf = source / "urdf/ffw_sg2_follower.urdf"
    if snapshot_urdf.exists():
        return (
            snapshot_urdf,
            source,
            source / "licenses/ROBOTIS-ai_worker-APACHE-2.0.txt",
        )
    raise FileNotFoundError(f"FFW-SG2 URDF was not found under {source}")


def prepare_ffw(source: Path, destination: Path) -> None:
    urdf_source, package_root, license_source = _ffw_source_paths(source)
    tree = ET.parse(urdf_source)
    root = tree.getroot()

    hidden_links = {
        "base_link",
        "left_wheel_steer_link",
        "left_wheel_drive_link",
        "right_wheel_steer_link",
        "right_wheel_drive_link",
        "rear_wheel_steer_link",
        "rear_wheel_drive_link",
    }
    for link in root.findall("link"):
        for collision in list(link.findall("collision")):
            link.remove(collision)
        if link.get("name") in hidden_links:
            for visual in list(link.findall("visual")):
                link.remove(visual)
        for visual in list(link.findall("visual")):
            mesh = visual.find("./geometry/mesh")
            if mesh is None:
                continue
            filename = mesh.get("filename", "")
            if "d405.stl" in filename:
                geometry = visual.find("geometry")
                if geometry is not None:
                    geometry.remove(mesh)
                    ET.SubElement(geometry, "box", {"size": "0.023 0.042 0.042"})
                continue
            if filename.startswith("file://"):
                link.remove(visual)

    # The pinned body_screening FFW reference contains these coordinate-only
    # end-effector links.  Older public ai_worker snapshots omit them, so add
    # them without inventing display geometry.
    for side in ("l", "r"):
        link_name = f"end_effector_{side}_link"
        joint_name = f"end_effector_{side}_joint"
        if root.find(f"./link[@name='{link_name}']") is not None:
            continue
        joint = ET.SubElement(root, "joint", {"name": joint_name, "type": "fixed"})
        ET.SubElement(joint, "parent", {"link": f"arm_{side}_link7"})
        ET.SubElement(joint, "child", {"link": link_name})
        ET.SubElement(joint, "origin", {"rpy": "0 0 0", "xyz": "0 0 -0.215"})
        ET.SubElement(root, "link", {"name": link_name})

    detector = ET.SubElement(root, "link", {"name": "metal_detector_link"})
    visual = ET.SubElement(detector, "visual")
    ET.SubElement(
        visual,
        "origin",
        {"xyz": "-0.075000 0.044794 -0.017715", "rpy": "0 0 -1.57079632679"},
    )
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(
        geometry,
        "mesh",
        {"filename": "package://pai_detector/MD.stl", "scale": "1 1 1"},
    )
    material = ET.SubElement(visual, "material", {"name": "detector_orange"})
    ET.SubElement(material, "color", {"rgba": "0.96 0.55 0.05 1"})
    mount = ET.SubElement(
        root, "joint", {"name": "metal_detector_mount_joint", "type": "fixed"}
    )
    ET.SubElement(
        mount,
        "origin",
        {"xyz": "0 0 -0.220551", "rpy": "3.14159265359 0 0"},
    )
    ET.SubElement(mount, "parent", {"link": "arm_r_link7"})
    ET.SubElement(mount, "child", {"link": "metal_detector_link"})
    frame = ET.SubElement(root, "link", {"name": "metal_detector_detection_frame"})
    frame_joint = ET.SubElement(
        root, "joint", {"name": "metal_detector_detection_joint", "type": "fixed"}
    )
    ET.SubElement(frame_joint, "origin", {"xyz": "0.131700 0 0", "rpy": "0 0 0"})
    ET.SubElement(frame_joint, "parent", {"link": "metal_detector_link"})
    ET.SubElement(frame_joint, "child", {"link": frame.get("name", "")})
    control_point = ET.SubElement(root, "link", {"name": "metal_detector_control_point"})
    control_joint = ET.SubElement(
        root, "joint", {"name": "metal_detector_control_point_joint", "type": "fixed"}
    )
    ET.SubElement(
        control_joint,
        "origin",
        {
            "xyz": "0.205701 0 0",
            "rpy": "-1.57079632679 -1.57079632679 0",
        },
    )
    ET.SubElement(control_joint, "parent", {"link": "metal_detector_link"})
    ET.SubElement(control_joint, "child", {"link": control_point.get("name", "")})

    destination.mkdir(parents=True, exist_ok=True)
    urdf_destination = destination / "urdf/ffw_sg2_web.urdf"
    urdf_destination.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(urdf_destination, encoding="utf-8", xml_declaration=True)

    copied: set[str] = set()
    for mesh in root.findall(".//visual/geometry/mesh"):
        filename = mesh.get("filename", "")
        prefix = "package://ffw_description/"
        if not filename.startswith(prefix):
            continue
        relative = filename[len(prefix) :]
        if relative in copied:
            continue
        copied.add(relative)
        _copy(package_root / relative, destination / relative)

    _copy(license_source, destination / "LICENSE-APACHE-2.0.txt")


def prepare_ridgeback(source: Path, destination: Path) -> None:
    for mesh in (source / "meshes").glob("*.stl"):
        _copy(mesh, destination / "meshes" / mesh.name)
    _copy(source.parent / "LICENSE", destination / "LICENSE-BSD-3-CLAUSE.txt")
    readme = source / "README.md"
    if readme.exists():
        _copy(readme, destination / "UPSTREAM_README.md")


def prepare_detector(visual: Path, collision: Path, destination: Path) -> None:
    _copy(visual, destination / "MD.stl")
    _copy(collision, destination / "MD_no_hole_poly.stl")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ai-worker", type=Path, required=True)
    parser.add_argument("--ridgeback-description", type=Path, required=True)
    parser.add_argument("--md-visual", type=Path, required=True)
    parser.add_argument("--md-collision", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("web/public/assets"))
    args = parser.parse_args()

    prepare_ffw(args.ai_worker.resolve(), args.output / "robot/ffw_description")
    prepare_ridgeback(
        args.ridgeback_description.resolve(), args.output / "robot/ridgeback_description"
    )
    prepare_detector(
        args.md_visual.resolve(), args.md_collision.resolve(), args.output / "detector"
    )
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
