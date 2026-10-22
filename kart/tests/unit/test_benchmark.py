from pathlib import Path

import pytest

from kart_sim.benchmark import (
    CampaignCase,
    CampaignResult,
    SafetyOracle,
    coverage,
    load_catalog,
    minimize_reproducing_faults,
    pairwise_fault_sets,
    seeded_parameter_samples,
)
from kart_sim.types import SimulationError

ROOT = Path(__file__).parents[2]


def test_catalog_traceability_and_coverage():
    catalog = load_catalog(ROOT / "configs/benchmarks/critical_smoke.yaml")
    assert set(catalog) == {"BRAKE-CABLE-BREAK", "MOTOR-STUCK-TORQUE", "VCU-FREEZE"}
    cases = [CampaignCase("torque", "x", ("MOTOR-STUCK-TORQUE",), 1)]
    result = CampaignResult("torque", True, 0, .1, .2, 3, (), "ok")
    report = coverage(catalog, [result], cases)
    assert report["covered_faults"] == ["MOTOR-STUCK-TORQUE"]


def test_oracle_requires_safe_response_and_detects_emergency_propulsion_violation():
    case = CampaignCase("stuck", "x", ("MOTOR-STUCK-TORQUE",), 1)
    rows = [{"state": {"time_s": 0, "u_mps": 1}, "vcu": {"state": "EMERGENCY_STOP", "faults": ("MOTOR-003",), "final_command": {"drive": 0}}}]
    assert SafetyOracle.evaluate(case, rows).passed
    rows[0]["vcu"]["final_command"]["drive"] = .1
    assert not SafetyOracle.evaluate(case, rows).passed


def test_pairwise_seeded_sampling_and_failure_minimization_are_deterministic():
    assert pairwise_fault_sets(("C", "A", "B")) == (("A", "B"), ("A", "C"), ("B", "C"))
    dimensions = {"speed": (1, 2, 3), "surface": ("dry", "wet")}
    assert seeded_parameter_samples(dimensions, count=3, seed=7) == seeded_parameter_samples(
        dimensions, count=3, seed=7
    )
    minimized = minimize_reproducing_faults(
        ("noise", "stuck_torque", "wet"),
        lambda faults: "stuck_torque" in faults,
    )
    assert minimized == ("stuck_torque",)
