from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


URDF = (
    Path(__file__).parents[1]
    / "web/public/assets/robot/ffw_description/urdf/ffw_sg2_web.urdf"
)


def _joint(root: ET.Element, name: str) -> ET.Element:
    joint = root.find(f"./joint[@name='{name}']")
    assert joint is not None, name
    return joint


def test_right_hand_and_detector_match_body_screening_reference():
    root = ET.parse(URDF).getroot()

    expected_links = {
        "arm_r_link7",
        "camera_r_link",
        "gripper_r_rh_p12_rn_base",
        "gripper_r_rh_p12_rn_r1",
        "gripper_r_rh_p12_rn_r2",
        "gripper_r_rh_p12_rn_l1",
        "gripper_r_rh_p12_rn_l2",
        "end_effector_r_link",
        "metal_detector_link",
        "metal_detector_control_point",
    }
    actual_links = {link.get("name") for link in root.findall("link")}
    assert expected_links <= actual_links

    gripper_mount = _joint(root, "gripper_r_joint")
    assert gripper_mount.find("parent").get("link") == "arm_r_link7"
    assert gripper_mount.find("origin").get("xyz") == "0 0 -0.078"
    assert gripper_mount.find("origin").get("rpy") == "0 3.14159265359 3.14159265359"

    detector_mount = _joint(root, "metal_detector_mount_joint")
    assert detector_mount.find("parent").get("link") == "arm_r_link7"
    assert detector_mount.find("origin").get("xyz") == "0 0 -0.220551"
    assert detector_mount.find("origin").get("rpy") == "3.14159265359 0 0"

    detector_visual = root.find("./link[@name='metal_detector_link']/visual")
    assert detector_visual is not None
    assert detector_visual.find("origin").get("xyz") == "-0.075000 0.044794 -0.017715"
    assert detector_visual.find("origin").get("rpy") == "0 0 -1.57079632679"

    detection_frame = _joint(root, "metal_detector_detection_joint")
    assert detection_frame.find("parent").get("link") == "metal_detector_link"
    assert detection_frame.find("origin").get("xyz") == "0.131700 0 0"
    assert detection_frame.find("origin").get("rpy") == "0 0 0"

    control_point = _joint(root, "metal_detector_control_point_joint")
    assert control_point.find("parent").get("link") == "metal_detector_link"
    assert control_point.find("origin").get("xyz") == "0.205701 0 0"
    assert control_point.find("origin").get("rpy") == "-1.57079632679 -1.57079632679 0"

    # body_screening derives this point from the dimensioned inspection-head
    # center in MD.stl and the detector visual transform.  Lock the same
    # geometry relationship into the public asset.
    center_in_stl = np.asarray([0.04479426, 0.280701, 0.01771529])
    visual_translation = np.asarray([-0.075000, 0.044794, -0.017715])
    visual_rotation = np.asarray(
        [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    )
    center_in_link = visual_translation + visual_rotation @ center_in_stl
    np.testing.assert_allclose(center_in_link, [0.205701, 0.0, 0.0], atol=3e-7)

    end_effector = _joint(root, "end_effector_r_joint")
    assert end_effector.find("parent").get("link") == "arm_r_link7"
    assert end_effector.find("origin").get("xyz") == "0 0 -0.215"

    camera_geometry = root.find("./link[@name='camera_r_link']/visual/geometry/box")
    assert camera_geometry is not None
    assert camera_geometry.get("size") == "0.023 0.042 0.042"
