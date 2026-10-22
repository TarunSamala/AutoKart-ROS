"""Steering angle PREVIEW only.

Converts the geometry engine's existing curvature profile (autokart_perception
.curvature.profile_centerline, already computed in pipeline.Pipeline.process)
into a bicycle-model steering angle. This value is never sent to the VCU as an
actuation command: docs/VCU_PROTOCOL.md states steering is "parsed and
ignored" by the current firmware release, and no NEMA34/R86mini driver
interface exists in this harness. It exists for the dashboard and for the
command-preview log only.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class SteeringPreview:
    valid: bool
    steering_angle_rad: float
    curvature_m_inv: float | None
    reason: str


def preview_steering(path_profile, wheelbase_m: float) -> SteeringPreview:
    if not path_profile:
        return SteeringPreview(False, 0.0, None, "NO_PATH_PROFILE")
    sample = next((p for p in path_profile if p.curvature_m_inv is not None), None)
    if sample is None:
        return SteeringPreview(False, 0.0, None, "NO_CURVATURE_SAMPLE")
    angle_rad = math.atan(wheelbase_m * sample.curvature_m_inv)
    return SteeringPreview(True, angle_rad, sample.curvature_m_inv, "OK")
