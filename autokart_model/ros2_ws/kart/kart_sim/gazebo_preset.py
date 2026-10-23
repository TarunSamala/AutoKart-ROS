"""Validate Gazebo fidelity presets and generate deterministic runtime SDF."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

import yaml

from .types import SimulationError

SENSORS = ("rgb_camera", "depth_camera", "lidar", "imu")


def load_gazebo_preset(path: str | Path) -> dict[str, Any]:
    source = Path(path)
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SimulationError("Gazebo preset must be a mapping")
    if payload.get("schema_version") != 1:
        raise SimulationError("Gazebo preset schema_version must be 1")
    if payload.get("physics_authority") != "internal":
        raise SimulationError("Phase 16 presets require internal physics authority")
    if payload.get("gazebo_role") != "visual_sensor_follower":
        raise SimulationError("Phase 16 presets require visual_sensor_follower role")
    if payload.get("vehicle_dynamics_authority") is not False:
        raise SimulationError("Gazebo dynamics authority must be disabled")
    rates = payload.get("rates", {})
    if float(rates.get("physics_hz", 0.0)) != 1000.0:
        raise SimulationError(
            "qualified presets retain the validated 1000 Hz physics rate"
        )
    world_step_hz = float(rates.get("gazebo_world_step_hz", 0.0))
    follower_hz = float(rates.get("gazebo_follow_hz", 0.0))
    if world_step_hz <= 0.0 or follower_hz <= 0.0:
        raise SimulationError("Gazebo world and follower rates must be positive")
    if world_step_hz < follower_hz:
        raise SimulationError("Gazebo world step rate cannot be below follower rate")
    for name in SENSORS:
        if name not in payload.get("sensors", {}):
            raise SimulationError(f"Gazebo preset is missing sensor: {name}")
    for name in ("rgb_camera", "depth_camera"):
        sensor = payload["sensors"][name]
        if min(int(sensor.get("width", 0)), int(sensor.get("height", 0))) <= 0:
            raise SimulationError(f"{name} dimensions must be positive")
        if float(sensor.get("rate_hz", 0.0)) <= 0.0:
            raise SimulationError(f"{name} rate must be positive")
    lidar = payload["sensors"]["lidar"]
    if int(lidar.get("samples", 0)) <= 0 or float(lidar.get("rate_hz", 0.0)) <= 0:
        raise SimulationError("LiDAR samples and rate must be positive")
    if float(payload["sensors"]["imu"].get("rate_hz", 0.0)) <= 0.0:
        raise SimulationError("IMU rate must be positive")
    return payload


def _sensor(model: ET.Element, name: str) -> ET.Element:
    result = model.find(f".//sensor[@name='{name}']")
    if result is None:
        raise SimulationError(f"base model is missing sensor: {name}")
    return result


def _set_text(parent: ET.Element, path: str, value: object) -> None:
    element = parent.find(path)
    if element is None:
        raise SimulationError(f"base SDF is missing required element: {path}")
    if isinstance(value, bool):
        element.text = "true" if value else "false"
    else:
        element.text = str(value)


def prepare_runtime(
    preset_path: str | Path,
    runtime_directory: str | Path,
    *,
    project_root: str | Path = ".",
) -> dict[str, Any]:
    preset_source = Path(preset_path).resolve()
    preset = load_gazebo_preset(preset_source)
    root = Path(project_root).resolve()
    destination = Path(runtime_directory).resolve() / preset["name"]
    model_directory = destination / "models" / "autokart"
    model_directory.mkdir(parents=True, exist_ok=True)

    model_source = root / "simulation/models/autokart/model.sdf"
    tree = ET.parse(model_source)
    model = tree.getroot().find("model")
    if model is None:
        raise SimulationError("base AutoKart model has no model element")
    rendering = preset["rendering"]
    for name in ("rgb_camera", "depth_camera"):
        sensor = _sensor(model, name)
        settings = preset["sensors"][name]
        if not bool(settings.get("enabled", True)):
            parent = model.find(
                f".//sensor[@name='{name}']/.."
            )
            if parent is None:
                raise SimulationError(f"cannot remove disabled sensor: {name}")
            parent.remove(sensor)
            continue
        _set_text(sensor, "update_rate", settings["rate_hz"])
        _set_text(sensor, "visualize", bool(rendering["sensor_visualization"]))
        _set_text(sensor, "camera/image/width", settings["width"])
        _set_text(sensor, "camera/image/height", settings["height"])
        plugin = sensor.find("plugin")
        if plugin is None:
            raise SimulationError(f"{name} delivery plugin is missing")
        _set_text(plugin, "processing_latency_s", settings["processing_latency_s"])
        _set_text(plugin, "transport_latency_s", settings["transport_latency_s"])
        _set_text(plugin, "queue_capacity", settings["queue_capacity"])
    lidar = _sensor(model, "lidar")
    lidar_settings = preset["sensors"]["lidar"]
    if not bool(lidar_settings.get("enabled", True)):
        parent = model.find(".//sensor[@name='lidar']/..")
        if parent is None:
            raise SimulationError("cannot remove disabled LiDAR")
        parent.remove(lidar)
    else:
        _set_text(lidar, "update_rate", lidar_settings["rate_hz"])
        _set_text(lidar, "visualize", bool(rendering["sensor_visualization"]))
        _set_text(lidar, "ray/scan/horizontal/samples", lidar_settings["samples"])
        lidar_plugin = lidar.find("plugin")
        if lidar_plugin is None:
            raise SimulationError("LiDAR delivery plugin is missing")
        _set_text(
            lidar_plugin, "processing_latency_s",
            lidar_settings["processing_latency_s"],
        )
        _set_text(
            lidar_plugin, "transport_latency_s",
            lidar_settings["transport_latency_s"],
        )
        _set_text(lidar_plugin, "queue_capacity", lidar_settings["queue_capacity"])
    imu_sensor = _sensor(model, "imu")
    imu_settings = preset["sensors"]["imu"]
    if (
        not bool(imu_settings.get("enabled", True))
        or imu_settings.get("source") == "internal"
    ):
        parent = model.find(".//sensor[@name='imu']/..")
        if parent is None:
            raise SimulationError("cannot remove inactive Gazebo IMU")
        parent.remove(imu_sensor)
    else:
        _set_text(imu_sensor, "update_rate", imu_settings["rate_hz"])
    follower_plugin = model.find("plugin[@name='autokart_gazebo_follower']")
    if follower_plugin is None:
        raise SimulationError("base follower plugin is missing")
    _set_text(
        follower_plugin, "maximum_state_age_s",
        preset["synchronization"]["maximum_state_age_s"],
    )
    ros_output_rate = follower_plugin.find("ros_output_rate_hz")
    if ros_output_rate is None:
        ros_output_rate = ET.SubElement(follower_plugin, "ros_output_rate_hz")
    ros_output_rate.text = str(preset["rates"]["ros_output_hz"])
    tree.write(
        model_directory / "model.sdf",
        encoding="utf-8",
        xml_declaration=True,
    )
    shutil.copy2(
        root / "simulation/models/autokart/model.config",
        model_directory / "model.config",
    )

    world_tree = ET.parse(
        root / "simulation/worlds/autokart_gazebo_follower_world.sdf"
    )
    world = world_tree.getroot().find("world")
    if world is None:
        raise SimulationError("base follower world has no world element")
    world_step_hz = float(preset["rates"]["gazebo_world_step_hz"])
    scheduler_target_rtf = float(
        preset["rates"].get("gazebo_scheduler_target_rtf", 1.0)
    )
    if scheduler_target_rtf <= 0.0:
        raise SimulationError("Gazebo scheduler target RTF must be positive")
    _set_text(world, "physics/max_step_size", f"{1.0 / world_step_hz:.12g}")
    _set_text(
        world,
        "physics/real_time_update_rate",
        f"{world_step_hz * scheduler_target_rtf:.12g}",
    )
    _set_text(world, "scene/shadows", bool(rendering["shadows"]))
    for light in world.findall("light"):
        cast = light.find("cast_shadows")
        if cast is not None:
            cast.text = "true" if rendering["shadows"] else "false"
    world_path = destination / "world.sdf"
    world_tree.write(world_path, encoding="utf-8", xml_declaration=True)
    manifest = {
        "preset": preset["name"],
        "preset_source": str(preset_source),
        "model_path": str(model_directory / "model.sdf"),
        "model_search_path": str(destination / "models"),
        "world_path": str(world_path),
        "software_render": bool(rendering["software_render"]),
        "sensor_fidelity": preset["sensors"],
        "performance_target": preset["performance_target"],
        "execution": preset.get("execution", {}),
        "gazebo_scheduler_target_rtf": scheduler_target_rtf,
    }
    (destination / "runtime_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description="Prepare a Gazebo fidelity preset")
    parser.add_argument("preset", type=Path)
    parser.add_argument("--runtime-dir", type=Path, required=True)
    parser.add_argument("--project-root", type=Path, default=Path.cwd())
    arguments = parser.parse_args()
    print(json.dumps(
        prepare_runtime(
            arguments.preset,
            arguments.runtime_dir,
            project_root=arguments.project_root,
        ),
        indent=2,
        sort_keys=True,
    ))


if __name__ == "__main__":
    main()
