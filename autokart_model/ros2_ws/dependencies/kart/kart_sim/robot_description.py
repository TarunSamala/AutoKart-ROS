"""Dependency-free URDF/SDF and TF contract validation."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, pi, sin
from pathlib import Path
from xml.etree import ElementTree as ET

from .config import FullKartConfig
from .ros2_sil import FrameTree, Transform
from .types import SimulationError


REQUIRED_LINKS = {
    "base_link",
    "chassis_link",
    "front_axle_link",
    "rear_axle_link",
    "front_left_wheel_link",
    "front_right_wheel_link",
    "rear_left_wheel_link",
    "rear_right_wheel_link",
    "steering_column_link",
    "steering_actuator_link",
    "brake_actuator_link",
    "motor_link",
    "battery_link",
    "vcu_link",
    "compute_link",
    "camera_link",
    "camera_optical_frame",
    "depth_camera_link",
    "lidar_link",
    "imu_link",
    "gps_link",
}

REQUIRED_JOINTS = {
    "front_left_steering_joint",
    "front_right_steering_joint",
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_solid_axle_joint",
    "steering_column_joint",
    "steering_actuator_joint",
    "brake_actuator_joint",
}


@dataclass(frozen=True)
class DescriptionReport:
    format: str
    name: str
    link_count: int
    joint_count: int
    total_mass_kg: float
    issues: tuple[str, ...]


def validate_urdf(path: str | Path, configuration: FullKartConfig | None = None) -> DescriptionReport:
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise SimulationError("URDF root must be robot")
    links = {element.attrib["name"]: element for element in root.findall("link")}
    joints = {element.attrib["name"]: element for element in root.findall("joint")}
    issues: list[str] = []
    missing_links = sorted(REQUIRED_LINKS - links.keys())
    missing_joints = sorted(REQUIRED_JOINTS - joints.keys())
    if missing_links:
        issues.append(f"missing links: {missing_links}")
    if missing_joints:
        issues.append(f"missing joints: {missing_joints}")
    total_mass = 0.0
    for name, link in links.items():
        inertial = link.find("inertial")
        if inertial is None:
            if name not in {"base_link", "camera_optical_frame"}:
                issues.append(f"{name}: missing inertial")
            continue
        mass_node = inertial.find("mass")
        inertia = inertial.find("inertia")
        if mass_node is None or float(mass_node.attrib.get("value", 0)) <= 0:
            issues.append(f"{name}: mass must be positive")
        else:
            total_mass += float(mass_node.attrib["value"])
        if inertia is None or any(float(inertia.attrib.get(axis, 0)) <= 0 for axis in ("ixx", "iyy", "izz")):
            issues.append(f"{name}: principal inertia must be positive")
        for collision in link.findall("collision"):
            if collision.find("./geometry/mesh") is not None:
                issues.append(f"{name}: visual mesh cannot be unreviewed collision geometry")
    children: set[str] = set()
    for name, joint in joints.items():
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            issues.append(f"{name}: missing parent or child")
            continue
        child_name = child.attrib["link"]
        if child_name in children:
            issues.append(f"{child_name}: duplicate joint parent")
        children.add(child_name)
        joint_type = joint.attrib.get("type")
        if joint_type in {"revolute", "prismatic"}:
            limit = joint.find("limit")
            if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
                issues.append(f"{name}: bounded joint is missing limits")
            elif float(limit.attrib["lower"]) >= float(limit.attrib["upper"]):
                issues.append(f"{name}: invalid joint limits")
    if configuration is not None:
        expected = {
            "front_axle_mount": configuration.kart.wheelbase_m / 2,
            "rear_solid_axle_joint": -configuration.kart.wheelbase_m / 2,
        }
        for joint_name, expected_x in expected.items():
            joint = joints.get(joint_name)
            origin = None if joint is None else joint.find("origin")
            actual_x = None if origin is None else float(origin.attrib.get("xyz", "0 0 0").split()[0])
            if actual_x is None or abs(actual_x - expected_x) > 1e-6:
                issues.append(f"{joint_name}: wheelbase does not match kart configuration")
    return DescriptionReport("URDF", root.attrib.get("name", ""), len(links), len(joints), total_mass, tuple(issues))


def validate_sdf(path: str | Path) -> DescriptionReport:
    root = ET.parse(path).getroot()
    if root.tag != "sdf":
        raise SimulationError("SDF root must be sdf")
    model = root.find("model")
    world = root.find("world")
    if model is None and world is None:
        raise SimulationError("SDF must contain a model or world")
    parent = model if model is not None else world
    assert parent is not None
    links = parent.findall(".//link")
    joints = parent.findall(".//joint")
    issues: list[str] = []
    total_mass = 0.0
    for link in links:
        mass = link.find("./inertial/mass")
        if mass is not None:
            value = float(mass.text or 0)
            total_mass += value
            if value <= 0:
                issues.append(f"{link.attrib.get('name')}: non-positive mass")
        for collision in link.findall("collision"):
            if collision.find("./geometry/mesh") is not None:
                issues.append(f"{link.attrib.get('name')}: unreviewed mesh collision")
    if model is not None:
        names = {link.attrib.get("name") for link in links}
        for required in {
            "base_link", "front_left_wheel_link", "front_right_wheel_link",
            "rear_left_wheel_link", "rear_right_wheel_link", "lidar_link", "imu_link",
        }:
            if required not in names:
                issues.append(f"missing SDF link: {required}")
        if model.find(".//plugin") is not None:
            plugin_text = ET.tostring(model, encoding="unicode").lower()
            if "diff_drive" in plugin_text or "ackermann" in plugin_text:
                issues.append("standard drive plugin prohibited for solid-axle engineering model")
    else:
        if world.find("physics") is None:
            issues.append("world missing physics configuration")
        if world.find("light") is None:
            issues.append("world missing lighting")
    return DescriptionReport(
        "SDF",
        parent.attrib.get("name", ""),
        len(links),
        len(joints),
        total_mass,
        tuple(issues),
    )


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = cos(roll / 2), sin(roll / 2)
    cp, sp = cos(pitch / 2), sin(pitch / 2)
    cy, sy = cos(yaw / 2), sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def autokart_frame_tree(namespace: str = "", timestamp_s: float = 0.0) -> FrameTree:
    prefix = namespace.strip("/")

    def frame(name: str) -> str:
        return f"{prefix}/{name}" if prefix else name

    tree = FrameTree()
    tree.add(Transform(frame("map"), frame("odom"), static=False, timestamp_s=timestamp_s))
    tree.add(Transform(frame("odom"), frame("base_link"), static=False, timestamp_s=timestamp_s))
    tree.add(Transform(frame("base_link"), frame("chassis_link")))
    for child, translation in {
        "front_axle_link": (0.575, 0.0, 0.14),
        "rear_axle_link": (-0.575, 0.0, 0.14),
        "camera_link": (0.64, 0.0, 0.72),
        "depth_camera_link": (0.60, 0.0, 0.64),
        "lidar_link": (0.20, 0.0, 0.82),
        "imu_link": (0.05, 0.0, 0.30),
        "gps_link": (-0.20, 0.0, 0.80),
    }.items():
        tree.add(Transform(frame("chassis_link"), frame(child), translation))
    tree.add(Transform(
        frame("camera_link"),
        frame("camera_optical_frame"),
        rotation_xyzw=_quaternion_from_rpy(-pi / 2, 0, -pi / 2),
    ))
    for child, parent in {
        "front_left_wheel_link": "front_axle_link",
        "front_right_wheel_link": "front_axle_link",
        "rear_left_wheel_link": "rear_axle_link",
        "rear_right_wheel_link": "rear_axle_link",
    }.items():
        tree.add(Transform(frame(parent), frame(child), static=False, timestamp_s=timestamp_s))
    return tree

