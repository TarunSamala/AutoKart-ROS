"""Deterministic VCU safety core with measured stopping and guarded reset."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite, sqrt
from struct import pack

from .communication import crc16_ccitt
from .types import VehicleCommand
from .fault_manager import FaultManager
from .safety_diagnostics import FaultSnapshot, FaultSnapshotBuffer


class VcuState(str, Enum):
    POWER_OFF = "POWER_OFF"
    BOOT = "BOOT"
    SELF_TEST = "SELF_TEST"
    SAFE = "SAFE"
    MANUAL = "MANUAL"
    READY = "READY"
    AUTONOMOUS = "AUTONOMOUS"
    DEGRADED = "DEGRADED"
    CONTROLLED_STOP = "CONTROLLED_STOP"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    FAULT_LOCKOUT = "FAULT_LOCKOUT"
    SERVICE_MODE = "SERVICE_MODE"


class ControlledStopPhase(str, Enum):
    INITIATE = "INITIATE"
    REMOVE_PROPULSION = "REMOVE_PROPULSION"
    STABILIZE = "STABILIZE"
    DECELERATE = "DECELERATE"
    FINAL_STOP = "FINAL_STOP"
    HOLD = "HOLD"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"


@dataclass(frozen=True)
class MotionCommand:
    source: str
    sequence: int
    timestamp_s: float
    valid_until_s: float
    speed_mps: float
    steering_rad: float
    brake: float = 0.0
    estop: bool = False
    checksum: int = 0
    mode: str = "speed_steering"
    acceleration_limit_mps2: float = 3.0
    deceleration_limit_mps2: float = 5.0
    valid: bool = True
    reset_epoch: int = 0

    def crc(self) -> int:
        payload = self.source.encode("utf-8") + b"\0" + self.mode.encode("utf-8") + b"\0" + pack(
            ">Iddddd?dd?Q",
            self.sequence,
            self.timestamp_s,
            self.valid_until_s,
            self.speed_mps,
            self.steering_rad,
            self.brake,
            self.estop,
            self.acceleration_limit_mps2,
            self.deceleration_limit_mps2,
            self.valid,
            self.reset_epoch,
        )
        return crc16_ccitt(payload)

    def signed(self) -> "MotionCommand":
        return MotionCommand(
            self.source,
            self.sequence,
            self.timestamp_s,
            self.valid_until_s,
            self.speed_mps,
            self.steering_rad,
            self.brake,
            self.estop,
            self.crc(),
            self.mode,
            self.acceleration_limit_mps2,
            self.deceleration_limit_mps2,
            self.valid,
            self.reset_epoch,
        )


@dataclass(frozen=True)
class VcuInputs:
    timestamp_s: float
    motion_command: MotionCommand | None
    vehicle_speed_mps: float
    steering_angle_rad: float
    brake_pressure_pa: float
    motor_torque_nm: float
    brake_actuator_position_m: float = 0.0
    independent_healthy: bool = True
    wheel_speeds_mps: tuple[float, ...] = ()
    wheel_feedback_valid: bool = True
    motor_current_a: float = 0.0
    motor_feedback_valid: bool = True
    brake_feedback_valid: bool = True
    steering_feedback_valid: bool = True
    contactor_feedback_closed: bool = True
    feedback_timestamp_s: float | None = None
    steering_secondary_angle_rad: float | None = None
    steering_current_a: float = 0.0
    brake_torque_nm: float = 0.0
    vehicle_acceleration_mps2: float = 0.0
    surface_friction: float = 1.0
    chain_torque_nm: float = 0.0
    motor_speed_feedback_valid: bool = True
    steering_actuator_angle_rad: float | None = None


@dataclass(frozen=True)
class VcuOutputs:
    timestamp_s: float
    state: VcuState
    command: VehicleCommand
    motor_enable: bool
    contactor_open: bool
    controlled_stop_phase: str | None
    emergency_stop_active: bool
    active_faults: tuple[str, ...]
    active_speed_limit_mps: float
    limiting_reason: str


@dataclass(frozen=True)
class ResetRequest:
    source: str
    timestamp_s: float
    valid_until_s: float
    authorized: bool
    repair_acknowledged: bool = True


@dataclass(frozen=True)
class ResetInputs:
    timestamp_s: float
    vehicle_speed_mps: float
    wheel_speeds_mps: tuple[float, ...]
    wheel_feedback_valid: bool
    motor_torque_nm: float
    motor_current_a: float
    motor_enable_removed: bool
    contactor_feedback_closed: bool
    contactor_open_commanded: bool
    physical_estop_active: bool
    remote_estop_active: bool
    independent_safety_healthy: bool
    watchdog_healthy: bool
    steering_primary_valid: bool
    steering_secondary_valid: bool
    steering_disagreement_rad: float
    brake_pressure_pa: float
    brake_pressure_valid: bool
    brake_actuator_valid: bool
    power_valid: bool
    thermal_valid: bool
    cooldown_complete: bool
    active_critical_faults: tuple[str, ...] = ()


@dataclass(frozen=True)
class ResetDecision:
    accepted: bool
    reasons: tuple[str, ...]
    next_state: VcuState
    self_test_required: bool = True


@dataclass(frozen=True)
class AutonomousGuards:
    independent_safety_healthy: bool = True
    estop_released: bool = True
    steering_healthy: bool = True
    brake_healthy: bool = True
    motor_healthy: bool = True
    wheel_speed_valid: bool = True
    power_healthy: bool = True
    required_sensors_healthy: bool = True
    localization_valid: bool = True
    perception_valid: bool = True
    track_projection_valid: bool = True
    command_stream_valid: bool = True

    def failures(self) -> tuple[str, ...]:
        return tuple(name for name, value in self.__dict__.items() if not value)


@dataclass(frozen=True)
class SafetyEnvelope:
    clear_distance_m: float = 100.0
    delay_s: float = 0.15
    guaranteed_decel_mps2: float = 3.0
    uncertainty_m: float = 0.5
    margin_m: float = 0.75
    curvature_1pm: float = 0.0
    lateral_limit_mps2: float = 5.0

    def speed_limit(
        self,
        curvature: float | None = None,
        lateral_limit: float | None = None,
        max_speed: float = 8.0,
    ) -> float:
        curvature = self.curvature_1pm if curvature is None else curvature
        lateral_limit = self.lateral_limit_mps2 if lateral_limit is None else lateral_limit
        d = self.clear_distance_m - self.uncertainty_m - self.margin_m
        clearance = (
            0.0
            if d <= 0.0
            else -self.guaranteed_decel_mps2 * self.delay_s
            + sqrt(
                (self.guaranteed_decel_mps2 * self.delay_s) ** 2
                + 2.0 * self.guaranteed_decel_mps2 * d
            )
        )
        curve = max_speed if abs(curvature) < 1e-9 else sqrt(lateral_limit / abs(curvature))
        return min(max_speed, clearance, curve)

    def stop_distance(self, speed_mps: float) -> float:
        return (
            speed_mps * self.delay_s
            + speed_mps**2 / (2.0 * self.guaranteed_decel_mps2)
            + self.uncertainty_m
            + self.margin_m
        )


class IndependentSafety:
    def __init__(self, timeout_s: float = 0.05) -> None:
        self.timeout_s = timeout_s
        self.last_pulse = -1e9
        self.estop = False
        self.stop_latched = False
        self.fault: str | None = None
        self.motor_isolation_requested = False
        self.brake_fallback_requested = False

    def pulse(self, timestamp_s: float) -> None:
        self.last_pulse = timestamp_s

    def permit(self, timestamp_s: float) -> bool:
        if self.estop:
            self.trip("PHYSICAL_ESTOP")
        elif timestamp_s - self.last_pulse > self.timeout_s:
            self.trip("INDEPENDENT_WATCHDOG")
        return not self.stop_latched

    def trip(self, fault: str) -> None:
        self.stop_latched = True
        self.fault = fault
        self.motor_isolation_requested = True
        self.brake_fallback_requested = True

    def reset(self, *, authorized: bool, source_resolved: bool, vehicle_stopped: bool) -> bool:
        if not (authorized and source_resolved and vehicle_stopped and not self.estop):
            return False
        self.stop_latched = False
        self.fault = None
        self.motor_isolation_requested = False
        self.brake_fallback_requested = False
        return True


class ControlledStopManager:
    """Physical stop state, based only on observable feedback supplied to VCU."""

    def __init__(self, confirmation_s: float = 0.05, timeout_s: float = 5.0) -> None:
        self.confirmation_s = confirmation_s
        self.timeout_s = timeout_s
        self.phase: ControlledStopPhase | None = None
        self.started_s: float | None = None
        self.hold_started_s: float | None = None
        self.rolling_restart_count = 0

    def start(self, timestamp_s: float) -> None:
        if self.phase is None or self.phase in (ControlledStopPhase.COMPLETE, ControlledStopPhase.FAILED):
            self.phase = ControlledStopPhase.INITIATE
            self.started_s = timestamp_s
            self.hold_started_s = None

    def _motion_stopped(self, inputs: VcuInputs) -> bool:
        if inputs.feedback_timestamp_s is not None and inputs.timestamp_s - inputs.feedback_timestamp_s > 0.1:
            return False
        if not (inputs.wheel_feedback_valid and inputs.motor_feedback_valid and inputs.brake_feedback_valid):
            return False
        wheel_stopped = all(abs(speed) < 0.05 for speed in inputs.wheel_speeds_mps)
        # Legacy callers without wheel signals still require the independent vehicle estimate.
        if not inputs.wheel_speeds_mps:
            wheel_stopped = True
        return (
            abs(inputs.vehicle_speed_mps) < 0.05
            and wheel_stopped
            and abs(inputs.motor_torque_nm) < 0.1
            and abs(inputs.motor_current_a) < 1.0
            and inputs.brake_pressure_pa >= 100_000.0
        )

    def update(self, inputs: VcuInputs) -> ControlledStopPhase:
        self.start(inputs.timestamp_s)
        assert self.started_s is not None
        elapsed = inputs.timestamp_s - self.started_s
        if elapsed > self.timeout_s:
            self.phase = ControlledStopPhase.FAILED
            return self.phase
        if self.phase == ControlledStopPhase.INITIATE:
            self.phase = ControlledStopPhase.REMOVE_PROPULSION
        elif self.phase == ControlledStopPhase.REMOVE_PROPULSION and elapsed >= 0.02:
            self.phase = ControlledStopPhase.STABILIZE
        elif self.phase == ControlledStopPhase.STABILIZE and elapsed >= 0.05:
            self.phase = ControlledStopPhase.DECELERATE

        stopped = self._motion_stopped(inputs)
        if stopped:
            if self.phase not in (ControlledStopPhase.FINAL_STOP, ControlledStopPhase.HOLD):
                self.phase = ControlledStopPhase.FINAL_STOP
                self.hold_started_s = inputs.timestamp_s
            elif self.phase == ControlledStopPhase.FINAL_STOP:
                self.phase = ControlledStopPhase.HOLD
            if self.hold_started_s is not None and inputs.timestamp_s - self.hold_started_s >= self.confirmation_s:
                self.phase = ControlledStopPhase.COMPLETE
        elif self.phase in (ControlledStopPhase.FINAL_STOP, ControlledStopPhase.HOLD):
            self.phase = ControlledStopPhase.DECELERATE
            self.hold_started_s = None
            self.rolling_restart_count += 1
        return self.phase


class VCU:
    def __init__(self) -> None:
        self.state = VcuState.POWER_OFF
        self.last_sequence: int | None = None
        self.counter_bits = 8
        self.maximum_sequence_gap = 3
        self.maximum_command_age_s = 0.10
        self.faults: list[str] = []
        self.fault_history: list[str] = []
        self.fault_manager = FaultManager()
        self.fault_snapshots = FaultSnapshotBuffer()
        self.independent = IndependentSafety()
        self.controlled_stop = ControlledStopManager()
        self._steer_error_start: float | None = None
        self._steer_actuator_reference_rad: float | None = None
        self._linkage_error_start: float | None = None
        self._brake_error_start: float | None = None
        self._brake_motion_start: float | None = None
        self._brake_release_start: float | None = None
        self._brake_degraded_start: float | None = None
        self._torque_error_start: float | None = None
        self._traction_loss_start: float | None = None
        self._isolation_error_start: float | None = None
        self._reset_in_progress = False
        self._last_command_steering_rad = 0.0
        self._last_command_time_s: float | None = None
        self._filtered_speed_limit_mps: float | None = None
        self._active_speed_limit_mps: float | None = None
        self._service_entered_s: float | None = None
        self.service_timeout_s = 0.5
        self.reset_epoch = 0

    def _fault(self, code: str, timestamp_s: float = 0.0) -> None:
        if code not in self.faults:
            self.faults.append(code)
        if code not in self.fault_history:
            self.fault_history.append(code)
        emergency = code.startswith(("MOTOR-003", "MOTOR-004", "BRAKE-003", "BRAKE-004", "POWER-004", "SAFETY-", "ESTOP"))
        self.fault_manager.confirm(
            code,
            timestamp_s,
            severity="CRITICAL" if emergency else "HIGH",
            response="EMERGENCY_STOP" if emergency else "CONTROLLED_STOP",
            latched=True,
        )

    def evaluate_reset_request(self, request: ResetRequest, inputs: ResetInputs) -> ResetDecision:
        reasons: list[str] = []
        if not request.authorized or request.source not in {"operator", "service_tool"}:
            reasons.append("unauthorized")
        if request.timestamp_s > inputs.timestamp_s + 0.01 or inputs.timestamp_s > request.valid_until_s:
            reasons.append("reset_request_stale_or_future")
        if abs(inputs.vehicle_speed_mps) > 0.05:
            reasons.append("vehicle_moving")
        if not inputs.wheel_feedback_valid or any(abs(speed) > 0.05 for speed in inputs.wheel_speeds_mps):
            reasons.append("wheel_motion_or_invalid")
        if abs(inputs.motor_torque_nm) > 0.1:
            reasons.append("motor_torque_present")
        if abs(inputs.motor_current_a) > 1.0:
            reasons.append("motor_current_present")
        if not inputs.motor_enable_removed:
            reasons.append("motor_enable_not_removed")
        if inputs.contactor_open_commanded and inputs.contactor_feedback_closed:
            reasons.append("contactor_feedback_inconsistent")
        if inputs.physical_estop_active:
            reasons.append("physical_estop_active")
        if inputs.remote_estop_active:
            reasons.append("remote_estop_active")
        if not inputs.independent_safety_healthy:
            reasons.append("independent_safety_unhealthy")
        if not inputs.watchdog_healthy:
            reasons.append("watchdog_unhealthy")
        if not inputs.steering_primary_valid or not inputs.steering_secondary_valid:
            reasons.append("steering_feedback_invalid")
        if abs(inputs.steering_disagreement_rad) > 0.052:
            reasons.append("steering_disagreement")
        if not inputs.brake_pressure_valid or not inputs.brake_actuator_valid:
            reasons.append("brake_feedback_invalid")
        if inputs.brake_pressure_pa < 100_000.0:
            reasons.append("brake_hold_missing")
        if not inputs.power_valid:
            reasons.append("power_invalid")
        if not inputs.thermal_valid:
            reasons.append("thermal_invalid")
        if not inputs.cooldown_complete:
            reasons.append("cooldown_incomplete")
        if inputs.active_critical_faults and not request.repair_acknowledged:
            reasons.append("repair_not_acknowledged")
        return ResetDecision(not reasons, tuple(reasons), VcuState.SELF_TEST if not reasons else self.state)

    def evaluate_reset(
        self,
        *,
        authorized: bool,
        vehicle_speed_mps: float,
        motor_torque_nm: float,
        estop_active: bool,
        brake_pressure_pa: float,
        feedback_healthy: bool,
    ) -> ResetDecision:
        """Compatibility wrapper around the centralized reset validator."""
        request = ResetRequest("operator", 0.0, 1.0, authorized)
        inputs = ResetInputs(
            0.0,
            vehicle_speed_mps,
            (),
            feedback_healthy,
            motor_torque_nm,
            0.0,
            True,
            False,
            True,
            estop_active,
            False,
            feedback_healthy,
            feedback_healthy,
            feedback_healthy,
            feedback_healthy,
            0.0,
            brake_pressure_pa,
            feedback_healthy,
            feedback_healthy,
            feedback_healthy,
            True,
            True,
        )
        return self.evaluate_reset_request(request, inputs)

    def reset(self, **kwargs: object) -> ResetDecision:
        decision = self.evaluate_reset(**kwargs)
        if decision.accepted:
            self.state = VcuState.SELF_TEST
            self.last_sequence = None
            self.reset_epoch += 1
            self._reset_in_progress = True
            self.fault_manager.reset_latched(True)
            self.faults.clear()
        return decision

    def reset_with_inputs(self, request: ResetRequest, inputs: ResetInputs) -> ResetDecision:
        decision = self.evaluate_reset_request(request, inputs)
        if decision.accepted:
            self.state = VcuState.SELF_TEST
            self.last_sequence = None
            self.reset_epoch += 1
            self._reset_in_progress = True
            self.fault_manager.reset_latched(True)
            self.faults.clear()
        return decision

    def transition(self, to: VcuState, healthy: bool = True) -> bool:
        allowed = {
            (VcuState.POWER_OFF, VcuState.BOOT),
            (VcuState.BOOT, VcuState.SELF_TEST),
            (VcuState.SELF_TEST, VcuState.SAFE),
            (VcuState.SAFE, VcuState.READY),
            (VcuState.FAULT_LOCKOUT, VcuState.SELF_TEST),
            (VcuState.SAFE, VcuState.SERVICE_MODE),
            (VcuState.SERVICE_MODE, VcuState.SAFE),
            (VcuState.AUTONOMOUS, VcuState.DEGRADED),
            (VcuState.DEGRADED, VcuState.SAFE),
        }
        if to in (VcuState.EMERGENCY_STOP, VcuState.CONTROLLED_STOP, VcuState.FAULT_LOCKOUT):
            self.state = to
            if to == VcuState.CONTROLLED_STOP:
                self.controlled_stop = ControlledStopManager(
                    self.controlled_stop.confirmation_s, self.controlled_stop.timeout_s
                )
            return True
        if self.state == VcuState.SELF_TEST and to == VcuState.SAFE and not healthy:
            self.state = VcuState.FAULT_LOCKOUT
            self._fault("SELF-TEST-FAILED")
            return False
        if (self.state, to) in allowed and healthy:
            self.state = to
            if to == VcuState.SERVICE_MODE:
                self._service_entered_s = None
            return True
        self._fault("STATE-INVALID")
        return False

    def request_autonomous(self, guards: AutonomousGuards) -> bool:
        if self.state != VcuState.READY:
            self._fault("STATE-INVALID")
            return False
        failures = guards.failures()
        if failures:
            for failure in failures:
                self._fault(f"AUTO-GUARD-{failure.upper()}")
            return False
        self.state = VcuState.AUTONOMOUS
        return True

    def update(
        self,
        timestamp_s: float,
        command: MotionCommand | None,
        *,
        healthy: bool = True,
        envelope: SafetyEnvelope = SafetyEnvelope(),
    ) -> VehicleCommand:
        if self.independent.estop or not self.independent.permit(timestamp_s):
            self.state = VcuState.EMERGENCY_STOP
            self._fault("SAFETY-WDG")
        if self.state == VcuState.EMERGENCY_STOP:
            return VehicleCommand(brake=1.0)
        invalid_code = None
        if (
            command is None
            or timestamp_s > command.valid_until_s
            or (command is not None and timestamp_s - command.timestamp_s > self.maximum_command_age_s)
        ):
            invalid_code = "COMM-STALE"
        elif command.timestamp_s > timestamp_s + 0.01:
            invalid_code = "COMM-FUTURE"
        elif not all(isfinite(value) for value in (
            command.speed_mps,
            command.steering_rad,
            command.brake,
            command.acceleration_limit_mps2,
            command.deceleration_limit_mps2,
        )):
            invalid_code = "COMM-NUMERIC"
        elif not command.valid:
            invalid_code = "COMM-INVALID"
        elif command.mode not in {"speed_steering", "speed_curvature", "emergency_stop"}:
            invalid_code = "COMM-MODE"
        elif command.acceleration_limit_mps2 <= 0.0 or command.deceleration_limit_mps2 <= 0.0:
            invalid_code = "COMM-LIMIT"
        elif command.reset_epoch != self.reset_epoch:
            invalid_code = "COMM-EPOCH"
        elif command.checksum != command.crc():
            invalid_code = "COMM-CRC"
        elif command.source not in {"scenario", "jetson", "baseline_controller", "manual"}:
            invalid_code = "COMM-AUTH"
        if invalid_code:
            if self.state == VcuState.AUTONOMOUS:
                self.state = VcuState.CONTROLLED_STOP
                self.controlled_stop.start(timestamp_s)
                self._fault(invalid_code)
            return VehicleCommand(brake=0.7)
        if command.estop:
            self.state = VcuState.EMERGENCY_STOP
            self._fault("ESTOP-ACTIVE")
            return VehicleCommand(brake=1.0)
        modulus = 1 << self.counter_bits
        sequence_delta = (
            1
            if self.last_sequence is None
            else (command.sequence - self.last_sequence) % modulus
        )
        if not 1 <= sequence_delta <= self.maximum_sequence_gap:
            self._fault("COMM-SEQUENCE")
            if self.state == VcuState.AUTONOMOUS:
                self.state = VcuState.CONTROLLED_STOP
                self.controlled_stop.start(timestamp_s)
            return VehicleCommand(brake=0.7)
        self.last_sequence = command.sequence
        if self.state not in (VcuState.AUTONOMOUS, VcuState.DEGRADED):
            return VehicleCommand()
        if abs(command.steering_rad) > 0.6:
            self._fault("STEER-LIMIT-001", timestamp_s)
        raw_limit = envelope.speed_limit()
        if self._filtered_speed_limit_mps is None:
            self._filtered_speed_limit_mps = raw_limit
        elif raw_limit < self._filtered_speed_limit_mps:
            self._filtered_speed_limit_mps = raw_limit
        else:
            dt = 0.0 if self._last_command_time_s is None else max(0.0, timestamp_s - self._last_command_time_s)
            self._filtered_speed_limit_mps = min(raw_limit, self._filtered_speed_limit_mps + 0.5 * dt)
        limit = self._filtered_speed_limit_mps
        if self.state == VcuState.DEGRADED:
            limit = min(limit, 2.0)
        self._active_speed_limit_mps = limit
        speed = max(0.0, min(command.speed_mps, limit))
        steering = max(-0.6, min(0.6, command.steering_rad))
        if self._last_command_time_s is not None:
            steering_step = 2.0 * max(0.0, timestamp_s - self._last_command_time_s)
            steering = max(
                self._last_command_steering_rad - steering_step,
                min(self._last_command_steering_rad + steering_step, steering),
            )
        self._last_command_steering_rad = steering
        self._last_command_time_s = timestamp_s
        return VehicleCommand(
            drive=min(1.0, speed / 8.0),
            brake=max(0.0, min(1.0, command.brake)),
            steering_rad=steering,
        )

    def cycle(self, inputs: VcuInputs, envelope: SafetyEnvelope = SafetyEnvelope()) -> VcuOutputs:
        previous_fault_count = len(self.faults)
        if self.state == VcuState.SERVICE_MODE:
            self._service_entered_s = inputs.timestamp_s if self._service_entered_s is None else self._service_entered_s
            if inputs.timestamp_s - self._service_entered_s >= self.service_timeout_s:
                self.state = VcuState.SAFE
                self._fault("SERVICE-TIMEOUT", inputs.timestamp_s)
        if not inputs.independent_healthy:
            self.independent.estop = True
        command = self.update(inputs.timestamp_s, inputs.motion_command, envelope=envelope)

        if (
            self.state == VcuState.AUTONOMOUS
            and inputs.motion_command
            and abs(inputs.motion_command.steering_rad) > 0.12
            and abs(inputs.motion_command.steering_rad - inputs.steering_angle_rad) > 0.18
        ):
            if self._steer_error_start is None:
                self._steer_error_start = inputs.timestamp_s
                self._steer_actuator_reference_rad = (
                    inputs.steering_angle_rad
                    if inputs.steering_actuator_angle_rad is None
                    else inputs.steering_actuator_angle_rad
                )
            if inputs.timestamp_s - self._steer_error_start >= 0.10:
                actuator_angle = (
                    inputs.steering_angle_rad
                    if inputs.steering_actuator_angle_rad is None
                    else inputs.steering_actuator_angle_rad
                )
                actuator_motion = abs(actuator_angle - (self._steer_actuator_reference_rad or 0.0))
                if actuator_motion < 0.005:
                    self.state = VcuState.EMERGENCY_STOP if abs(inputs.vehicle_speed_mps) > 5.0 else VcuState.CONTROLLED_STOP
                    if self.state == VcuState.CONTROLLED_STOP:
                        self.controlled_stop.start(inputs.timestamp_s)
                    self._fault("STEER-001")
                self._steer_error_start = None
                self._steer_actuator_reference_rad = None
        else:
            self._steer_error_start = None
            self._steer_actuator_reference_rad = None
        actuator_angle = (
            inputs.steering_angle_rad
            if inputs.steering_actuator_angle_rad is None
            else inputs.steering_actuator_angle_rad
        )
        linkage_mismatch = (
            self.state == VcuState.AUTONOMOUS
            and inputs.motion_command is not None
            and abs(inputs.motion_command.steering_rad) > 0.12
            and abs(actuator_angle) > 0.1
            and abs(inputs.steering_angle_rad) < 0.01
        )
        if linkage_mismatch:
            self._linkage_error_start = (
                inputs.timestamp_s if self._linkage_error_start is None else self._linkage_error_start
            )
            if inputs.timestamp_s - self._linkage_error_start >= 0.15:
                self.state = VcuState.EMERGENCY_STOP if abs(inputs.vehicle_speed_mps) > 5.0 else VcuState.CONTROLLED_STOP
                if self.state == VcuState.CONTROLLED_STOP:
                    self.controlled_stop.start(inputs.timestamp_s)
                self._fault("STEER-004", inputs.timestamp_s)
        else:
            self._linkage_error_start = None
        secondary = (
            inputs.steering_angle_rad
            if inputs.steering_secondary_angle_rad is None
            else inputs.steering_secondary_angle_rad
        )
        if self.state in (VcuState.AUTONOMOUS, VcuState.CONTROLLED_STOP) and (
            not inputs.steering_feedback_valid
            or abs(inputs.steering_angle_rad - secondary) > 0.052
        ):
            if self.state == VcuState.AUTONOMOUS:
                self.state = VcuState.CONTROLLED_STOP
                self.controlled_stop.start(inputs.timestamp_s)
            self._fault("STEER-002")
        if (
            self.state == VcuState.AUTONOMOUS
            and inputs.motion_command
            and abs(inputs.motion_command.steering_rad) < 0.05
            and abs(inputs.steering_angle_rad) > 0.12
        ):
            self.state = VcuState.EMERGENCY_STOP
            self._fault("STEER-003")

        if (
            self.state == VcuState.AUTONOMOUS
            and inputs.motion_command
            and inputs.motion_command.brake > 0.1
            and inputs.brake_actuator_position_m > 0.001
            and inputs.brake_pressure_pa < 250_000.0
        ):
            self._brake_error_start = inputs.timestamp_s if self._brake_error_start is None else self._brake_error_start
            if inputs.timestamp_s - self._brake_error_start >= 0.15:
                if inputs.brake_torque_nm > 5.0 or inputs.vehicle_acceleration_mps2 < -0.5:
                    self.state = VcuState.CONTROLLED_STOP
                    self.controlled_stop.start(inputs.timestamp_s)
                    self._fault("BRAKE-SENSOR-001")
                else:
                    self.state = VcuState.EMERGENCY_STOP
                    self._fault("BRAKE-003")
                    self._fault("STOPPING_CAPABILITY_INSUFFICIENT")
        else:
            self._brake_error_start = None
        expected_pressure = min(
            8_000_000.0,
            2.0e9 * max(0.0, inputs.brake_actuator_position_m - 0.002),
        )
        partial_transmission = (
            self.state == VcuState.AUTONOMOUS
            and inputs.motion_command is not None
            and inputs.motion_command.brake > 0.1
            and inputs.brake_actuator_position_m > 0.005
            and inputs.brake_pressure_pa >= 250_000.0
            and expected_pressure > 0.0
            and inputs.brake_pressure_pa < 0.4 * expected_pressure
        )
        if partial_transmission:
            self._brake_degraded_start = (
                inputs.timestamp_s if self._brake_degraded_start is None else self._brake_degraded_start
            )
            if inputs.timestamp_s - self._brake_degraded_start >= 0.15:
                self.state = VcuState.DEGRADED
                self._fault("BRAKE-005", inputs.timestamp_s)
        else:
            self._brake_degraded_start = None
        braking_requested = (
            self.state == VcuState.AUTONOMOUS
            and inputs.motion_command is not None
            and inputs.motion_command.brake > 0.1
        )
        if braking_requested and inputs.brake_actuator_position_m < 0.001:
            self._brake_motion_start = inputs.timestamp_s if self._brake_motion_start is None else self._brake_motion_start
            if inputs.timestamp_s - self._brake_motion_start >= 0.10:
                if inputs.brake_pressure_pa > 250_000 or inputs.brake_torque_nm > 5.0:
                    self.state = VcuState.CONTROLLED_STOP
                    self.controlled_stop.start(inputs.timestamp_s)
                    self._fault("BRAKE-SENSOR-002")
                else:
                    self.state = VcuState.EMERGENCY_STOP
                    self._fault("BRAKE-001")
                    self._fault("STOPPING_CAPABILITY_INSUFFICIENT")
        else:
            self._brake_motion_start = None
        release_requested = (
            self.state == VcuState.AUTONOMOUS
            and inputs.motion_command is not None
            and inputs.motion_command.brake < 0.05
            and inputs.brake_pressure_pa > 100_000
        )
        if release_requested:
            self._brake_release_start = inputs.timestamp_s if self._brake_release_start is None else self._brake_release_start
            if inputs.timestamp_s - self._brake_release_start >= 0.10:
                self.state = VcuState.FAULT_LOCKOUT
                self._fault("BRAKE-RELEASE-001")
        else:
            self._brake_release_start = None
        if self.state == VcuState.AUTONOMOUS and command.drive > 0.0 and inputs.brake_pressure_pa > 100_000:
            self.state = VcuState.FAULT_LOCKOUT
            self._fault("BRAKE-PROPULSION-CONFLICT")
        if (
            self.state == VcuState.AUTONOMOUS
            and inputs.motion_command
            and inputs.motion_command.brake > 0.2
            and inputs.brake_pressure_pa > 4_000_000.0
            and inputs.brake_torque_nm < 150.0
        ):
            self.state = VcuState.EMERGENCY_STOP
            self._fault("BRAKE-004")

        if command.drive <= 0.01 and inputs.motor_torque_nm > 0.5:
            self._torque_error_start = inputs.timestamp_s if self._torque_error_start is None else self._torque_error_start
            if inputs.timestamp_s - self._torque_error_start >= 0.05:
                self.state = VcuState.EMERGENCY_STOP
                self._fault("MOTOR-003")
        else:
            self._torque_error_start = None
        if (
            self.state == VcuState.AUTONOMOUS
            and command.drive > 0.1
            and inputs.motor_torque_nm < -0.5
        ):
            self.state = VcuState.EMERGENCY_STOP
            self._fault("MOTOR-004")
        traction_loss = (
            self.state == VcuState.AUTONOMOUS
            and command.drive > 0.1
            and (
                (abs(inputs.motor_torque_nm) < 0.1 and inputs.motor_current_a < 1.0)
                or (inputs.motor_torque_nm > 0.5 and abs(inputs.chain_torque_nm) < 0.1)
            )
        )
        if traction_loss:
            self._traction_loss_start = inputs.timestamp_s if self._traction_loss_start is None else self._traction_loss_start
            if inputs.timestamp_s - self._traction_loss_start >= 0.1:
                self.state = VcuState.CONTROLLED_STOP
                self.controlled_stop.start(inputs.timestamp_s)
                self._fault("MOTOR-005")
        else:
            self._traction_loss_start = None
        if self.state == VcuState.AUTONOMOUS and not inputs.motor_speed_feedback_valid:
            self.state = VcuState.CONTROLLED_STOP
            self.controlled_stop.start(inputs.timestamp_s)
            self._fault("MOTOR-SENSOR-001")

        emergency = self.state == VcuState.EMERGENCY_STOP
        stopping = self.state == VcuState.CONTROLLED_STOP
        phase: ControlledStopPhase | None = None
        if emergency:
            command = VehicleCommand(brake=1.0)
        elif stopping:
            phase = self.controlled_stop.update(inputs)
            command = VehicleCommand(steering_rad=inputs.steering_angle_rad, brake=0.7)
            if phase == ControlledStopPhase.FAILED:
                self.state = VcuState.EMERGENCY_STOP
                self._fault("CONTROLLED_STOP_FAILED")
                self._fault("STOPPING_CAPABILITY_INSUFFICIENT")
                command = VehicleCommand(brake=1.0)
                emergency = True
            elif phase == ControlledStopPhase.COMPLETE:
                self.state = VcuState.SAFE
                command = VehicleCommand(brake=0.7)
        if self.state == VcuState.EMERGENCY_STOP and inputs.contactor_feedback_closed:
            self._isolation_error_start = (
                inputs.timestamp_s if self._isolation_error_start is None else self._isolation_error_start
            )
            if inputs.timestamp_s - self._isolation_error_start >= 0.05:
                self.state = VcuState.FAULT_LOCKOUT
                self._fault("POWER-004")
        else:
            self._isolation_error_start = None
        if self.state == VcuState.FAULT_LOCKOUT:
            command = VehicleCommand(brake=1.0, steering_rad=inputs.steering_angle_rad)
            emergency = False

        motor_enable = not emergency and self.state not in (
            VcuState.POWER_OFF,
            VcuState.FAULT_LOCKOUT,
            VcuState.SAFE,
        )
        contactor_open = emergency or self.state in (VcuState.POWER_OFF, VcuState.FAULT_LOCKOUT)
        invariant_violation = (
            (self.state == VcuState.EMERGENCY_STOP and (command.drive > 0 or motor_enable))
            or (self.state == VcuState.FAULT_LOCKOUT and (command.drive > 0 or motor_enable))
            or (self.state == VcuState.CONTROLLED_STOP and command.drive > 0)
        )
        if invariant_violation:
            self._fault("SAFETY-INVARIANT", inputs.timestamp_s)
            self.state = VcuState.EMERGENCY_STOP
            command = VehicleCommand(brake=1.0)
            motor_enable = False
            contactor_open = True
            emergency = True
        if len(self.faults) > previous_fault_count:
            self.fault_snapshots.append(
                FaultSnapshot(
                    inputs.timestamp_s,
                    self.state.value,
                    tuple(self.faults),
                    inputs.vehicle_speed_mps,
                    inputs.motor_torque_nm,
                    inputs.motor_current_a,
                    inputs.brake_pressure_pa,
                    inputs.steering_angle_rad,
                )
            )
        return VcuOutputs(
            inputs.timestamp_s,
            self.state,
            command,
            motor_enable,
            contactor_open,
            phase.value if phase else None,
            emergency,
            tuple(self.faults),
            self._active_speed_limit_mps if self._active_speed_limit_mps is not None else envelope.speed_limit(),
            "clearance_or_curvature",
        )
