"""Data-governed calibration primitives; real-kart validation remains absent."""

from __future__ import annotations

import csv
import hashlib
from dataclasses import dataclass, replace
from enum import Enum
from math import isfinite
from pathlib import Path
from typing import Callable

from .types import SimulationError


@dataclass(frozen=True)
class DatasetMetadata:
    dataset_id: str
    date: str
    kart_configuration: str
    track_surface: str
    software_version: str
    calibration_version: str
    raw_checksum: str
    anomalies: tuple[str, ...] = ()

    def validate(self) -> None:
        required = (
            self.dataset_id,
            self.date,
            self.kart_configuration,
            self.track_surface,
            self.software_version,
            self.calibration_version,
            self.raw_checksum,
        )
        if any(not value for value in required):
            raise SimulationError("dataset metadata is incomplete")


class DatasetSplit(str, Enum):
    CALIBRATION = "CALIBRATION"
    VALIDATION = "VALIDATION"
    HOLDOUT = "HOLDOUT"


class CalibrationSplitRegistry:
    def __init__(self) -> None:
        self._assignments: dict[str, DatasetSplit] = {}

    def assign(self, metadata: DatasetMetadata, split: DatasetSplit) -> None:
        metadata.validate()
        previous = self._assignments.get(metadata.dataset_id)
        if previous is not None and previous != split:
            raise SimulationError("dataset cannot cross calibration/validation/holdout splits")
        self._assignments[metadata.dataset_id] = split

    def split_of(self, dataset_id: str) -> DatasetSplit:
        if dataset_id not in self._assignments:
            raise SimulationError("dataset has no frozen split assignment")
        return self._assignments[dataset_id]


@dataclass(frozen=True)
class Parameter:
    name: str
    unit: str
    nominal: float
    lower: float
    upper: float
    source: str
    confidence: str = "provisional"
    identifiable_experiments: tuple[str, ...] = ()
    dependencies: tuple[str, ...] = ()
    fitted_value: float | None = None
    uncertainty: float | None = None
    validation_status: str = "NOT_EXECUTED_EXTERNAL_DEPENDENCY"

    def validate(self) -> None:
        if not self.name or not self.unit or not self.source:
            raise SimulationError("parameter registry entry is incomplete")
        values = (self.nominal, self.lower, self.upper)
        if not all(isfinite(value) for value in values) or not self.lower <= self.nominal <= self.upper:
            raise SimulationError("parameter nominal/bounds are invalid")


class ParameterRegistry:
    def __init__(self) -> None:
        self._parameters: dict[str, Parameter] = {}

    def add(self, parameter: Parameter) -> None:
        parameter.validate()
        if parameter.name in self._parameters:
            raise SimulationError("duplicate calibration parameter")
        self._parameters[parameter.name] = parameter

    def fit(self, name: str, value: float, uncertainty: float) -> Parameter:
        parameter = self._parameters[name]
        if not parameter.lower <= value <= parameter.upper or uncertainty < 0:
            raise SimulationError("fitted parameter violates physical bounds")
        updated = replace(parameter, fitted_value=value, uncertainty=uncertainty)
        self._parameters[name] = updated
        return updated

    def records(self) -> tuple[Parameter, ...]:
        return tuple(self._parameters[name] for name in sorted(self._parameters))


def checksum(path: str | Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def import_csv(path: str | Path, required: tuple[str, ...]) -> list[dict[str, float]]:
    with Path(path).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or not set(required) <= set(rows[0]):
        raise SimulationError("calibration CSV missing required columns")
    try:
        return [{key: float(value) for key, value in row.items()} for row in rows]
    except (TypeError, ValueError) as error:
        raise SimulationError("calibration CSV contains non-numeric data") from error


def bounded_linear_gain(samples: list[dict[str, float]], x: str, y: str, lower: float, upper: float) -> float:
    denominator = sum(row[x] * row[x] for row in samples)
    if denominator <= 0:
        raise SimulationError("parameter is unidentifiable from supplied experiment")
    return max(lower, min(upper, sum(row[x] * row[y] for row in samples) / denominator))


@dataclass(frozen=True)
class OptimizationResult:
    value: float
    objective: float
    evaluations: int


def bounded_grid_optimize(
    objective: Callable[[float], float],
    lower: float,
    upper: float,
    *,
    points: int = 101,
) -> OptimizationResult:
    if not isfinite(lower) or not isfinite(upper) or lower >= upper or points < 3:
        raise SimulationError("invalid bounded optimizer configuration")
    candidates = [lower + (upper - lower) * index / (points - 1) for index in range(points)]
    evaluated = [(float(objective(value)), value) for value in candidates]
    if any(not isfinite(cost) for cost, _ in evaluated):
        raise SimulationError("optimizer objective returned a non-finite value")
    cost, value = min(evaluated, key=lambda item: (item[0], item[1]))
    return OptimizationResult(value, cost, len(evaluated))


def calibration_report_template(metadata: DatasetMetadata, parameters: ParameterRegistry) -> dict[str, object]:
    metadata.validate()
    return {
        "dataset_id": metadata.dataset_id,
        "raw_checksum": metadata.raw_checksum,
        "parameters": [parameter.__dict__ for parameter in parameters.records()],
        "real_data_validation": "NOT_EXECUTED_EXTERNAL_DEPENDENCY",
        "digital_twin_claim": "NOT_JUSTIFIED",
    }
