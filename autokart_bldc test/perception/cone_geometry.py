"""Cone position estimation, bird's-eye graph, and centerline/curvature.

Does not reimplement any geometry. Wraps autokart_perception.pipeline.Pipeline,
which already does: depth validation -> per-detection metric localization ->
Delaunay/Gabriel graph -> left/right boundary estimation -> gate selection ->
centerline generation -> curvature profiling -> a safety assessment
(vehicle_authorized is computed there and is never overridden here).
"""
from __future__ import annotations

import sys
from pathlib import Path

PERCEPTION_SRC = (Path(__file__).resolve().parents[2] /
                   "autonomy_stack_phase1/phase_01_perception/src")
if str(PERCEPTION_SRC) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_SRC))

from autokart_perception.pipeline import Pipeline  # noqa: E402
from autokart_perception.sdk import PerceptionGeometrySDK, default_config  # noqa: E402


class ConeGeometryEngine:
    """One instance per run. Holds the Pipeline's small amount of frame-to-frame state
    (stationary-display filter, latched sensor fault), matching how live_mission uses it.
    """

    def __init__(self, detector, config: dict | None = None, synthetic: bool = False):
        self.sdk = PerceptionGeometrySDK(detector=detector) if config is None else \
            PerceptionGeometrySDK(config, detector=detector)
        self.pipeline = Pipeline(detector, self.sdk.config, synthetic=synthetic)

    def process(self, frame):
        """Returns the full autokart_perception.types.Result for this frame."""
        return self.pipeline.process(frame)

    @staticmethod
    def left_right_cones(result) -> tuple[list, list]:
        """Split result.cones into (left, right) using the pipeline's own boundary
        assignment (result.boundaries.left_ids / right_ids), not a re-derivation.
        """
        boundaries = result.boundaries
        if boundaries is None:
            return [], []
        by_id = {cone.id: cone for cone in result.cones}
        left = [by_id[i] for i in boundaries.left_ids if i in by_id]
        right = [by_id[i] for i in boundaries.right_ids if i in by_id]
        return left, right

    @staticmethod
    def birds_eye_points(result) -> list[dict]:
        """Flat list of {id, label, x_forward_m, y_left_m, distance_m, confidence}
        for every localized cone, for the dashboard's top-down view.
        """
        return [{"id": c.id, "label": c.label, "x_forward_m": c.xy[0], "y_left_m": c.xy[1],
                 "distance_m": c.distance_m, "confidence": c.confidence} for c in result.cones]


def probe(detector) -> dict:
    """Software-only smoke test: construct the engine and run its config validation.
    Does not require a camera frame.
    """
    result = {"test": "geometry_engine_init", "tested": True, "pass": False, "detail": None}
    try:
        engine = ConeGeometryEngine(detector)
        result["pass"] = True
        result["detail"] = {"config_keys": sorted(engine.sdk.config.keys())[:10]}
    except Exception as exc:
        result["detail"] = f"{type(exc).__name__}: {exc}"
    return result
