"""Deterministic, risk-prioritized fault campaign benchmark utilities."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from itertools import combinations, product
from random import Random
from typing import Iterable

import yaml

from .scenario import run_scenario
from .types import SimulationError


@dataclass(frozen=True)
class FaultDefinition:
    fault_id: str
    subsystem: str
    severity: str
    injection_layer: str
    expected_diagnostics: tuple[str, ...]
    required_response: str
    fmea_rows: tuple[str, ...]
    safety_requirements: tuple[str, ...]
    physical_meaning: str

    def validate(self) -> None:
        if not self.fault_id or not self.subsystem or not self.physical_meaning:
            raise SimulationError("fault catalog entry is incomplete")
        if self.severity not in {"INFO", "WARNING", "DEGRADED", "CRITICAL"}:
            raise SimulationError("fault catalog severity is invalid")
        if not self.expected_diagnostics or not self.required_response:
            raise SimulationError("critical fault catalog entry lacks diagnostic/response traceability")


@dataclass(frozen=True)
class CampaignCase:
    case_id: str
    scenario: str
    fault_ids: tuple[str, ...]
    seed: int
    tier: str = "fast"


@dataclass(frozen=True)
class CampaignResult:
    case_id: str
    passed: bool
    fault_start_s: float | None
    detection_s: float | None
    safe_state_s: float | None
    maximum_speed_mps: float
    invariant_violations: tuple[str, ...]
    evidence: str


def load_catalog(path: str | Path) -> dict[str, FaultDefinition]:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    catalog: dict[str, FaultDefinition] = {}
    for item in raw["faults"]:
        entry = FaultDefinition(
            item["id"], item["subsystem"], item["severity"], item["injection_layer"],
            tuple(item["expected_diagnostics"]), item["required_response"], tuple(item["fmea_rows"]),
            tuple(item["safety_requirements"]), item["physical_meaning"],
        )
        entry.validate()
        if entry.fault_id in catalog:
            raise SimulationError("fault catalog contains duplicate ID")
        catalog[entry.fault_id] = entry
    return catalog


class SafetyOracle:
    """Evaluates telemetry evidence, not injector metadata, for campaign result."""

    @staticmethod
    def evaluate(case: CampaignCase, rows: list[dict]) -> CampaignResult:
        if not rows:
            return CampaignResult(case.case_id, False, None, None, None, 0.0, ("no_telemetry",), "scenario emitted no telemetry")
        fault_rows = [row for row in rows if row.get("vcu", {}).get("faults")]
        safe_rows = [row for row in rows if row.get("vcu", {}).get("state") in {"EMERGENCY_STOP", "FAULT_LOCKOUT", "SAFE"}]
        maximum_speed = max(abs(float(row.get("state", {}).get("u_mps", 0.0))) for row in rows)
        fault_start = next((float(row["state"]["time_s"]) for row in fault_rows), None)
        safe_time = next((float(row["state"]["time_s"]) for row in safe_rows), None)
        violations: list[str] = []
        for row in rows:
            vcu = row.get("vcu", {})
            if vcu.get("state") == "EMERGENCY_STOP" and vcu.get("final_command", {}).get("drive", 0.0) > 0.0:
                violations.append("positive_propulsion_in_emergency_stop")
        expected = {
            "BRAKE-CABLE-BREAK": "BRAKE-003", "MOTOR-STUCK-TORQUE": "MOTOR-003", "VCU-FREEZE": "SAFETY-WDG",
        }
        present = {fault for row in fault_rows for fault in row.get("vcu", {}).get("faults", ())}
        expected_hit = any(expected[fault] in present for fault in case.fault_ids if fault in expected)
        passed = not violations and (expected_hit or not case.fault_ids) and bool(safe_rows)
        return CampaignResult(case.case_id, passed, fault_start, fault_start, safe_time, maximum_speed, tuple(violations),
                              f"diagnostics={sorted(present)} safe_rows={len(safe_rows)}")


class CampaignRunner:
    def __init__(self, catalog: dict[str, FaultDefinition], output_root: str | Path) -> None:
        self.catalog = catalog
        self.output_root = Path(output_root)

    def run(self, cases: Iterable[CampaignCase], *, resume: bool = False) -> list[CampaignResult]:
        results: list[CampaignResult] = []
        for case in sorted(cases, key=lambda item: item.case_id):
            if any(fault not in self.catalog for fault in case.fault_ids):
                raise SimulationError("campaign references an unknown fault")
            result_path = self.output_root / case.case_id / "benchmark_result.json"
            if resume and result_path.exists():
                payload = json.loads(result_path.read_text(encoding="utf-8"))
                payload["invariant_violations"] = tuple(payload["invariant_violations"])
                results.append(CampaignResult(**payload))
                continue
            scenario_path = Path(case.scenario)
            output = run_scenario(scenario_path, self.output_root / case.case_id)
            rows = [json.loads(line) for line in (output / "telemetry.jsonl").read_text(encoding="utf-8").splitlines()]
            result = SafetyOracle.evaluate(case, rows)
            (self.output_root / case.case_id / "benchmark_result.json").write_text(json.dumps(asdict(result), indent=2) + "\n", encoding="utf-8")
            results.append(result)
        return results


def pairwise_fault_sets(fault_ids: Iterable[str]) -> tuple[tuple[str, str], ...]:
    """Stable pairwise combinations for risk review and case generation."""
    return tuple(combinations(sorted(set(fault_ids)), 2))


def seeded_parameter_samples(
    dimensions: dict[str, tuple[object, ...]],
    *,
    count: int,
    seed: int,
) -> tuple[dict[str, object], ...]:
    if count <= 0 or not dimensions or any(not values for values in dimensions.values()):
        raise SimulationError("random campaign dimensions/count are invalid")
    names = tuple(sorted(dimensions))
    population = [dict(zip(names, values)) for values in product(*(dimensions[name] for name in names))]
    random = Random(seed)
    if count >= len(population):
        random.shuffle(population)
        return tuple(population)
    return tuple(population[index] for index in random.sample(range(len(population)), count))


def minimize_reproducing_faults(
    fault_ids: Iterable[str],
    reproduces: object,
) -> tuple[str, ...]:
    """Deterministic one-minimal reduction for a reproducing fault set."""
    retained = list(dict.fromkeys(fault_ids))
    predicate = reproduces
    if not callable(predicate) or not retained or not predicate(tuple(retained)):
        raise SimulationError("initial fault set does not reproduce the failure")
    changed = True
    while changed:
        changed = False
        for fault in tuple(retained):
            candidate = tuple(item for item in retained if item != fault)
            if candidate and predicate(candidate):
                retained = list(candidate)
                changed = True
                break
    return tuple(retained)


def coverage(catalog: dict[str, FaultDefinition], results: Iterable[CampaignResult], cases: Iterable[CampaignCase]) -> dict[str, object]:
    case_list = list(cases); result_by_id = {result.case_id: result for result in results}
    covered = {fault for case in case_list if result_by_id.get(case.case_id, CampaignResult("", False, None, None, None, 0, (), "")).passed for fault in case.fault_ids}
    return {"catalog_faults": len(catalog), "covered_faults": sorted(covered), "uncovered_faults": sorted(set(catalog) - covered),
            "passed_cases": sum(result.passed for result in result_by_id.values()), "total_cases": len(case_list)}
