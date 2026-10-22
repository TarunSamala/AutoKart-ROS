"""One-source deterministic track validation and backend conversion."""

from __future__ import annotations

from dataclasses import asdict
from hashlib import sha256
import json
from math import ceil, floor, hypot
from pathlib import Path
from typing import Any

import yaml

from .cones import boundary_cones
from .track import Track
from .types import SimulationError


def _orientation(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def detect_self_intersections(track: Track) -> tuple[tuple[int, int], ...]:
    segments = [
        ((track.points[i][0], track.points[i][1]), (track.points[j][0], track.points[j][1]))
        for i, j in track._segments
    ]
    intersections = []
    for first in range(len(segments)):
        for second in range(first + 1, len(segments)):
            if second in {first - 1, first + 1} or (
                track.closed and {first, second} == {0, len(segments) - 1}
            ):
                continue
            a, b = segments[first]
            c, d = segments[second]
            if _orientation(a, b, c) * _orientation(a, b, d) < 0 and _orientation(c, d, a) * _orientation(c, d, b) < 0:
                intersections.append((first, second))
    return tuple(intersections)


def validate_track_source(path: str | Path, *, minimum_radius_m: float = 1.5) -> dict[str, Any]:
    track = Track.from_yaml(path)
    issues = list(track.validate())
    intersections = detect_self_intersections(track)
    if intersections:
        issues.append(f"ERROR: centreline self-intersections {intersections}")
    maximum_curvature = max(abs(track.sample(index * 0.25).curvature_1pm) for index in range(max(1, int(track.length_m / 0.25))))
    minimum_radius = float("inf") if maximum_curvature < 1e-9 else 1.0 / maximum_curvature
    if minimum_radius < minimum_radius_m:
        issues.append(f"ERROR: minimum sampled radius {minimum_radius:.3f} m below {minimum_radius_m:.3f} m")
    source_bytes = Path(path).read_bytes()
    return {
        "name": track.name,
        "length_m": track.length_m,
        "minimum_radius_m": minimum_radius,
        "self_intersections": intersections,
        "issues": tuple(issues),
        "version_sha256": sha256(source_bytes).hexdigest(),
    }


def dashboard_geometry(track: Track, *, spacing_m: float = 0.5) -> dict[str, Any]:
    samples = []
    count = max(2, int(ceil(track.length_m / spacing_m)))
    for index in range(count + (0 if track.closed else 1)):
        s_m = min(track.length_m, index * track.length_m / count)
        point = track.sample(s_m)
        samples.append({
            "s_m": s_m,
            "center": [point.x_m, point.y_m, point.z_m],
            "left": list(track.world_from_track(s_m, point.left_width_m)),
            "right": list(track.world_from_track(s_m, -point.right_width_m)),
            "bank_rad": point.bank_rad,
            "curvature_1pm": point.curvature_1pm,
        })
    return {"name": track.name, "closed_loop": track.closed, "length_m": track.length_m, "samples": samples}


def cone_layout(track: Track, *, spacing_m: float = 3.0) -> dict[str, Any]:
    return {
        "track": track.name,
        "ground_truth_only": True,
        "cones": [
            {
                **asdict(cone),
                "semantic_class": cone.semantic_class.value,
            }
            for cone in boundary_cones(track, spacing_m)
        ],
    }


def occupancy_map(track: Track, *, resolution_m: float = 0.1, margin_m: float = 2.0) -> tuple[str, dict[str, Any]]:
    boundaries = dashboard_geometry(track, spacing_m=max(resolution_m, 0.25))["samples"]
    coordinates = [
        (point[key][0], point[key][1])
        for point in boundaries
        for key in ("left", "right")
    ]
    minimum_x = floor((min(x for x, _ in coordinates) - margin_m) / resolution_m) * resolution_m
    minimum_y = floor((min(y for _, y in coordinates) - margin_m) / resolution_m) * resolution_m
    maximum_x = ceil((max(x for x, _ in coordinates) + margin_m) / resolution_m) * resolution_m
    maximum_y = ceil((max(y for _, y in coordinates) + margin_m) / resolution_m) * resolution_m
    width = int(round((maximum_x - minimum_x) / resolution_m)) + 1
    height = int(round((maximum_y - minimum_y) / resolution_m)) + 1
    grid = [[100 for _ in range(width)] for _ in range(height)]  # unknown/occupied by default
    for row in range(height):
        y_m = minimum_y + row * resolution_m
        for column in range(width):
            x_m = minimum_x + column * resolution_m
            projection = track.project(x_m, y_m)
            if min(projection.left_margin_m, projection.right_margin_m) >= 0:
                grid[height - 1 - row][column] = 0
    pgm = "P2\n# AutoKart deterministic occupancy map\n{} {}\n100\n{}".format(
        width,
        height,
        "\n".join(" ".join(map(str, row)) for row in grid),
    )
    metadata = {
        "image": f"{track.name}.pgm",
        "resolution": resolution_m,
        "origin": [minimum_x, minimum_y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
        "unknown_is_free": False,
    }
    return pgm + "\n", metadata


def gazebo_world_sdf(track: Track, *, world_name: str | None = None, cones: bool = True) -> str:
    geometry = dashboard_geometry(track, spacing_m=0.5)
    points = "".join(
        f"<point>{sample['center'][0]:.6f} {sample['center'][1]:.6f}</point>"
        for sample in geometry["samples"]
    )
    cone_models = ""
    if cones:
        models = []
        for cone in boundary_cones(track, 3.0):
            color = "0.05 0.15 0.9 1" if cone.semantic_class.value == "blue" else "0.95 0.85 0.05 1"
            models.append(
                f"""<model name="{cone.cone_id}"><static>false</static>
<pose>{cone.x_m:.6f} {cone.y_m:.6f} 0.15 0 0 {cone.yaw_rad:.6f}</pose>
<link name="cone"><inertial><mass>0.4</mass><inertia><ixx>0.003</ixx><iyy>0.003</iyy><izz>0.002</izz></inertia></inertial>
<collision name="collision"><geometry><cone><radius>0.15</radius><length>0.30</length></cone></geometry></collision>
<visual name="visual"><geometry><cone><radius>0.15</radius><length>0.30</length></cone></geometry>
<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link></model>"""
            )
        cone_models = "\n".join(models)
    name = world_name or track.name
    return f"""<?xml version="1.0"?>
<sdf version="1.7">
  <world name="{name}">
    <gravity>0 0 -9.80665</gravity>
    <physics name="deterministic" type="ode"><max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate></physics>
    <scene><ambient>0.45 0.48 0.52 1</ambient><background>0.45 0.65 0.9 1</background><shadows>true</shadows></scene>
    <light name="sun" type="directional"><cast_shadows>true</cast_shadows><pose>0 0 20 0 0 0</pose><diffuse>0.9 0.88 0.82 1</diffuse><direction>-0.4 0.2 -1</direction></light>
    <model name="ground"><static>true</static><link name="ground_link">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>250 250</size></plane></geometry><surface><friction><ode><mu>1.0</mu><mu2>0.95</mu2></ode></friction></surface></collision>
      <visual name="grass"><geometry><plane><normal>0 0 1</normal><size>250 250</size></plane></geometry><material><ambient>0.08 0.28 0.06 1</ambient><diffuse>0.12 0.38 0.08 1</diffuse></material></visual>
    </link></model>
    <model name="track_visual"><static>true</static><link name="track_link">
      <visual name="centreline"><geometry><polyline><height>0.015</height>{points}</polyline></geometry><material><ambient>0.12 0.12 0.13 1</ambient><diffuse>0.18 0.18 0.19 1</diffuse></material></visual>
    </link></model>
    {cone_models}
  </world>
</sdf>
"""


def export_track(source: str | Path, output_dir: str | Path) -> dict[str, Path]:
    track = Track.from_yaml(source)
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    paths = {
        "dashboard": destination / f"{track.name}.json",
        "cones": destination / f"{track.name}.cones.json",
        "world": destination / f"{track.name}.sdf",
        "map": destination / f"{track.name}.pgm",
        "map_yaml": destination / f"{track.name}.map.yaml",
        "raceline": destination / f"{track.name}.raceline.csv",
    }
    paths["dashboard"].write_text(json.dumps(dashboard_geometry(track), indent=2) + "\n", encoding="utf-8")
    paths["cones"].write_text(json.dumps(cone_layout(track), indent=2) + "\n", encoding="utf-8")
    paths["world"].write_text(gazebo_world_sdf(track), encoding="utf-8")
    pgm, metadata = occupancy_map(track)
    paths["map"].write_text(pgm, encoding="ascii")
    paths["map_yaml"].write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")
    geometry = dashboard_geometry(track, spacing_m=0.25)
    paths["raceline"].write_text(
        "s_m,x_m,y_m,z_m,curvature_1pm\n"
        + "\n".join(
            f"{sample['s_m']:.6f},{sample['center'][0]:.6f},{sample['center'][1]:.6f},{sample['center'][2]:.6f},{sample['curvature_1pm']:.9f}"
            for sample in geometry["samples"]
        )
        + "\n",
        encoding="utf-8",
    )
    manifest = destination / "manifest.json"
    manifest.write_text(json.dumps({
        "source": str(source),
        "source_validation": validate_track_source(source),
        "outputs": {name: path.name for name, path in paths.items()},
    }, indent=2, default=list) + "\n", encoding="utf-8")
    paths["manifest"] = manifest
    return paths
