"""Step 6 (D2): assembles the OpenRC GoKart STL parts from ~/Desktop/kart stl
into decimated, scaled, rear-axle-referenced glTF assets for the viewer.

IMPORTANT: this is a 1:5-scale RC hobby model of a DIFFERENT vehicle, not the
real kart's CAD (confirmed in docs/my_track_spec/RECON.md: no real CAD or
dimension-sheet PDF exists anywhere found). This script produces a VISUAL
reference only -- physics, footprints, and clearance checks everywhere else
in this repo keep using the dimension-sheet values from kart/configs/karts/.

The Engine group (8.1M triangles across 10 files, ~400 MB) is excluded: it
would blow the ~150k triangle budget by >50x on its own, and an internal
combustion engine block is not visible from outside a kart's silhouette in
the top-down/orbit viewer this is for. Logged, not hidden -- see
docs/my_track_spec/STL_FIT.md.

Axis convention: the STL parts' native frame was determined empirically (no
authoritative metadata exists), not assumed:
  - "Axle 4.0.stl"'s longest dimension (214 mm) runs along native X, and it
    is the actual rear axle shaft -> native X is lateral (left/right).
  - The overall non-Engine assembly's native Z extent (441 mm) is by far the
    largest and matches the spread of axle-adjacent small parts (bearings,
    spacers) along Z -> native Z is longitudinal (forward/rear).
  - Native Y is the remaining (smallest, 133 mm) extent -> vertical (up).
  - Mapping (forward, left, up) = (native Z, native X, native Y) is
    right-handed given a right-handed native frame (Z x X = Y, confirmed).
This is inference from part geometry, not ground truth -- flagged as an
assumption in STL_FIT.md.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import trimesh

SOURCE_DIR = Path("/home/mark1/Desktop/kart stl ")  # NOTE: trailing space is part of the real directory name
OUTPUT_DIR = Path(__file__).parents[1] / "viewer" / "assets" / "kart"
TARGET_LENGTH_M = 2.400  # dimension sheet, both profiles
TRIANGLE_BUDGET_TOTAL = 150_000

GROUPS = {
    "body": "Body",
    "chassis": "Chassis",
    "rear_axle": "Rear Axle",
    "steering": "Steering",
}
EXCLUDED_GROUP = "Engine"
REAR_AXLE_REFERENCE_PART = "Axle 4.0.stl"

# native (x, y, z) -> output (forward=x, left=y, up=z)
AXIS_PERMUTATION = (2, 0, 1)


def remap_axes(vertices: np.ndarray) -> np.ndarray:
    return vertices[:, AXIS_PERMUTATION]


def load_group(group_dir: Path) -> list[tuple[str, trimesh.Trimesh]]:
    meshes = []
    for path in sorted(group_dir.glob("*.stl")):
        mesh = trimesh.load(path, force="mesh", process=False)
        meshes.append((path.stem, mesh))
    return meshes


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    report: dict = {"groups": {}, "excluded": {}, "axis_mapping": "native(x,y,z) -> output(forward,left,up) = (z,x,y)"}

    all_loaded: dict[str, list[tuple[str, trimesh.Trimesh]]] = {}
    combined_min = np.array([np.inf, np.inf, np.inf])
    combined_max = np.array([-np.inf, -np.inf, -np.inf])
    rear_axle_centre_native = None

    for key, dirname in GROUPS.items():
        group_dir = SOURCE_DIR / dirname
        meshes = load_group(group_dir)
        all_loaded[key] = meshes
        triangle_count = sum(len(m.faces) for _, m in meshes)
        for name, mesh in meshes:
            combined_min = np.minimum(combined_min, mesh.bounds[0])
            combined_max = np.maximum(combined_max, mesh.bounds[1])
            if name == Path(REAR_AXLE_REFERENCE_PART).stem:
                rear_axle_centre_native = (mesh.bounds[0] + mesh.bounds[1]) / 2
        report["groups"][key] = {"source_dir": dirname, "parts": len(meshes), "triangles_raw": triangle_count}

    if rear_axle_centre_native is None:
        raise RuntimeError(f"reference part {REAR_AXLE_REFERENCE_PART!r} not found in Rear Axle group")

    native_size = combined_max - combined_min
    output_size_unscaled = native_size[list(AXIS_PERMUTATION)]  # (forward, left, up) extents, still in native units (assumed mm)
    scale = TARGET_LENGTH_M * 1000.0 / output_size_unscaled[0]  # calibrate uniform scale on the forward (length) extent, native units assumed mm
    report["source_bounding_box_mm"] = {"min": combined_min.tolist(), "max": combined_max.tolist(), "size": native_size.tolist()}
    report["scale_factor"] = scale
    report["scale_calibrated_on"] = "forward (length) extent, assumed native units = mm"

    rear_axle_origin_native = rear_axle_centre_native

    def transform(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        mesh = mesh.copy()
        vertices = remap_axes(mesh.vertices - rear_axle_origin_native) * (scale / 1000.0)
        mesh.vertices = vertices
        return mesh

    # Budget triangles proportionally to each group's raw share, after an
    # initial uniform target; exports as one combined mesh per group.
    total_raw = sum(v["triangles_raw"] for v in report["groups"].values())
    exported_total = 0
    residuals_m = {}
    for key, meshes in all_loaded.items():
        transformed = [transform(m) for _, m in meshes]
        combined = trimesh.util.concatenate(transformed) if len(transformed) > 1 else transformed[0]
        raw_group_triangles = len(combined.faces)
        target = max(100, int(TRIANGLE_BUDGET_TOTAL * (report["groups"][key]["triangles_raw"] / total_raw)))
        if raw_group_triangles > target:
            combined = combined.simplify_quadric_decimation(face_count=target)
        exported_total += len(combined.faces)
        out_path = OUTPUT_DIR / f"{key}.glb"
        combined.export(out_path)
        bbox_size_m = (combined.bounds[1] - combined.bounds[0]).tolist()
        report["groups"][key].update({
            "triangles_exported": len(combined.faces),
            "output_file": str(out_path.relative_to(OUTPUT_DIR.parents[2])),
            "bbox_size_m_forward_left_up": bbox_size_m,
        })

    report["triangles_exported_total"] = exported_total
    report["triangle_budget_total"] = TRIANGLE_BUDGET_TOTAL
    report["within_budget"] = exported_total <= TRIANGLE_BUDGET_TOTAL

    engine_dir = SOURCE_DIR / EXCLUDED_GROUP
    engine_triangles = sum(_stl_triangle_count(p) for p in engine_dir.glob("*.stl"))
    report["excluded"][EXCLUDED_GROUP] = {
        "reason": "would blow the triangle budget by >50x alone; not visible from outside the kart silhouette",
        "triangles_raw": engine_triangles,
        "file_count": len(list(engine_dir.glob("*.stl"))),
    }

    # Residuals against the dimension sheet (width, height) -- length is 0 by
    # construction since the scale was calibrated on it.
    scaled_size_m = output_size_unscaled * (scale / 1000.0)
    report["residuals_vs_dimension_sheet_m"] = {
        "length_m": {"scaled": scaled_size_m[0], "target": TARGET_LENGTH_M, "residual": scaled_size_m[0] - TARGET_LENGTH_M},
        "width_m": {"scaled": scaled_size_m[1], "target": 1.181, "residual": scaled_size_m[1] - 1.181},
        "height_m": {"scaled": scaled_size_m[2], "target": 1.118, "residual": scaled_size_m[2] - 1.118},
    }

    # Proxy front wheels: no dedicated wheel/tyre mesh exists among the
    # Steering parts (only hub/axle/nut/mount hardware) -- confirmed by
    # listing every file in that group, not assumed. Per the fallback this
    # was always allowed to use, synthesize simple cylinders instead, one
    # pair per kart profile (their front tracks differ), sized from the
    # real tyre radius in kart/configs/karts/*.yaml (ASSUMED there, not
    # measured) and positioned at each profile's actual front_track_m/2.
    report["front_wheels"] = {}
    from kart_sim.config import load_kart_config
    karts_dir = OUTPUT_DIR.parents[2] / "configs" / "karts"
    for profile in ("shared", "built"):
        kart_config = load_kart_config(karts_dir / f"{profile}.yaml")
        radius_m = kart_config.tires.radius_m
        width_m = kart_config.tires.width_m
        half_track_m = kart_config.kart.front_track_m / 2
        wheel = trimesh.creation.cylinder(radius=radius_m, height=width_m, sections=24)
        wheel.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))  # cylinder axis -> lateral (y)
        for side, sign in (("left", 1), ("right", -1)):
            positioned = wheel.copy()
            positioned.apply_translation([kart_config.kart.wheelbase_m, sign * half_track_m, radius_m])
            out_path = OUTPUT_DIR / f"front_wheel_{profile}_{side}.glb"
            positioned.export(out_path)
        report["front_wheels"][profile] = {
            "radius_m": radius_m, "width_m": width_m, "half_track_m": half_track_m,
            "note": "proxy cylinder (ASSUMED tyre radius/width from kart_sim config, not measured); "
                    "no separable wheel/tyre mesh exists in the source STL parts",
        }

    (OUTPUT_DIR / "prepare_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


def _stl_triangle_count(path: Path) -> int:
    import struct
    with path.open("rb") as f:
        f.read(80)
        return struct.unpack("<I", f.read(4))[0]


if __name__ == "__main__":
    main()
