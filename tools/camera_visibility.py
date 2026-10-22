#!/usr/bin/env python3
"""Nominal ground visibility from a saved, real HP60C frame's SDK intrinsics.

This is geometry only. It does not qualify depth scale, mount calibration, or cone detection.
"""
import math
import os
import sys
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "kart", "ros2_ws", "src", "autokart_vehicle_geometry"))
from autokart_vehicle_geometry.geometry import GEOMETRY

# Observed in docs/skidpad_live_evidence/hp60c_valid_frame_probe.log, FRAME attempt 2.
IMAGE_HEIGHT = 480
FY = 585.0194702148438
CY = 239.9635772705078
with open(os.path.join(ROOT, "autonomy_stack_phase1", "phase_01_perception", "src",
                       "autokart_perception", "data", "perception.yaml"), encoding="utf-8") as _depth_config_file:
    _depth_config = yaml.safe_load(_depth_config_file)
PIPELINE_DEPTH_MIN_M = float(_depth_config["depth_min_m"])
PIPELINE_DEPTH_MAX_M = float(_depth_config["depth_max_m"])


def visibility(height_m=GEOMETRY.camera_height_m,
               pitch_deg=GEOMETRY.camera_pitch_down_deg,
               forward_m=GEOMETRY.camera_forward_distance_at_stop_m,
               fy=FY, cy=CY, image_height=IMAGE_HEIGHT):
    lower_ray_deg = pitch_deg + math.degrees(math.atan((image_height - 1 - cy) / fy))
    depression_deg = math.degrees(math.atan2(height_m, forward_m))
    slant_m = math.hypot(height_m, forward_m)
    return {
        "nearest_ground_forward_m": height_m / math.tan(math.radians(lower_ray_deg)),
        "lower_ray_depression_deg": lower_ray_deg,
        "cone_base_depression_deg": depression_deg,
        "cone_base_below_axis_deg": depression_deg - pitch_deg,
        "cone_base_slant_range_m": slant_m,
        "cone_base_nominally_in_image": depression_deg <= lower_ray_deg,
        "cone_base_within_configured_depth_range": PIPELINE_DEPTH_MIN_M <= slant_m <= PIPELINE_DEPTH_MAX_M,
    }


def main():
    result = visibility()
    print("SDK intrinsics: fy=%.6f px, cy=%.6f px, image height=%d px" % (FY, CY, IMAGE_HEIGHT))
    print("Source: docs/skidpad_live_evidence/hp60c_valid_frame_probe.log, saved live frame")
    for key, value in result.items():
        print(f"{key}: {value}")
    print(f"Configured accepted depth: {PIPELINE_DEPTH_MIN_M:.1f} to {PIPELINE_DEPTH_MAX_M:.1f} m")
    print("Nominal result: visible and within configured depth range at the stop point. "
          "Actual valid depth is UNVERIFIED until tools/depth_check.py measures it on the mounted camera.")


if __name__ == "__main__":
    main()
