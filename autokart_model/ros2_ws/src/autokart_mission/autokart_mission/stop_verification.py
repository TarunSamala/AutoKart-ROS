"""Evidence-only stationary hold verification; ROS-independent and fail closed.

An axle encoder plus its derived odometry is one measurement, not independent
chassis motion evidence. This verifier never treats a target or ACK as motion.
"""
from __future__ import annotations

from math import isfinite


def provenance(value):
    return {"SYNTHETIC": "SIMULATED", "LIVE_PHYSICAL": "LIVE"}.get(value, value)


class StopVerifier:
    def __init__(self, source, *, hold_s=1.0, max_age_s=0.15,
                 wheel_threshold_mps=0.05, ground_threshold_mps=0.05):
        values = (hold_s, max_age_s, wheel_threshold_mps, ground_threshold_mps)
        if any(not isfinite(v) or v <= 0 for v in values):
            raise ValueError("stop verification limits must be finite and positive")
        self.source = provenance(source)
        self.hold_ns, self.max_age_ns = int(hold_s * 1e9), int(max_age_s * 1e9)
        self.wheel_threshold, self.ground_threshold = wheel_threshold_mps, ground_threshold_mps
        self.first = None
        self.last = None
        self.samples = 0
        self.last_identity = None
        self.result = self._result(False, ["STOP_EVIDENCE_MISSING"])

    def _result(self, verified, reasons, duration=0):
        return {"state": "STOP_VERIFIED" if verified else "STOP_UNVERIFIED",
                "verified": verified, "physical_stop_verified": verified and self.source == "LIVE",
                "source": self.source, "stationary_hold_s": duration / 1e9,
                "required_hold_s": self.hold_ns / 1e9, "distinct_samples": self.samples,
                "reason_codes": reasons}

    def restart_hold(self):
        self.first, self.last, self.samples = None, None, 0
        self.result = self._result(False, ["POST_TRIGGER_HOLD_PENDING"])

    def update(self, evidence, now_ns):
        reasons = []
        if not isinstance(evidence, dict):
            evidence = {}
        if evidence.get("schema_version") != 1:
            reasons.append("STOP_EVIDENCE_SCHEMA_INVALID")
        if provenance(evidence.get("source")) != self.source or self.source not in {"LIVE", "REPLAY", "SIMULATED"}:
            reasons.append("STOP_EVIDENCE_SOURCE_MISMATCH")
        stamp = evidence.get("sample_time_ns")
        seq, boot = evidence.get("sequence"), evidence.get("boot_id")
        if (type(stamp) is not int or stamp <= 0 or not 0 <= now_ns - stamp <= self.max_age_ns):
            reasons.append("STOP_TELEMETRY_STALE_OR_UNTIMED")
        if type(seq) is not int or not 0 <= seq < 2**32 or type(boot) is not int or not 0 <= boot < 2**32:
            reasons.append("STOP_SESSION_OR_SEQUENCE_INVALID")
        for field, reason in (("clock_valid", "STOP_CLOCK_UNVERIFIED"),
                              ("wheel_valid", "ENCODER_INVALID"),
                              ("ground_valid", "GROUND_MOTION_UNAVAILABLE"),
                              ("ground_independent", "GROUND_MOTION_NOT_INDEPENDENT"),
                              ("vcu_valid", "VCU_DISCONNECTED_OR_INVALID"),
                              ("stop_active", "VCU_STOP_NOT_ACTIVE")):
            if evidence.get(field) is not True:
                reasons.append(reason)
        ground_source = evidence.get("ground_speed_source")
        if (not isinstance(ground_source, str) or not ground_source.strip() or
                any(x in ground_source.lower() for x in ("wheel", "encoder", "odom", "command", "target"))):
            reasons.append("GROUND_MOTION_SOURCE_INVALID")
        # Instrumentation must be explicitly declared, never defaulted absent.
        if type(evidence.get("brake_instrumented")) is not bool:
            reasons.append("BRAKE_INSTRUMENTATION_UNKNOWN")
        elif evidence["brake_instrumented"] and not (
                evidence.get("brake_valid") is True and evidence.get("brake_applied") is True):
            reasons.append("BRAKE_FEEDBACK_MISSING_OR_UNAPPLIED")
        if evidence.get("estop_active") is not False:
            reasons.append("ESTOP_ACTIVE_OR_UNKNOWN")
        if not isinstance(evidence.get("faults"), list) or evidence["faults"]:
            reasons.append("BLOCKING_VCU_FAULT_OR_UNKNOWN")
        for field, threshold, reason in (("wheel_speed_mps", self.wheel_threshold, "WHEELS_NOT_STATIONARY"),
                                         ("ground_speed_mps", self.ground_threshold, "CHASSIS_NOT_STATIONARY")):
            value = evidence.get(field)
            if type(value) not in (int, float) or not isfinite(value) or abs(value) > threshold:
                reasons.append(reason)
        identity = (boot, seq, stamp)
        if self.last_identity is not None and identity != self.last_identity:
            previous_boot, previous_seq, previous_stamp = self.last_identity
            if boot != previous_boot:
                self.first, self.samples = None, 0
                reasons.append("STOP_VCU_SESSION_CHANGED")
            elif (type(seq) is int and type(stamp) is int and
                  (seq <= previous_seq or stamp <= previous_stamp)):
                reasons.append("STOP_EVIDENCE_NONMONOTONE")
        if reasons:
            self.first, self.samples = None, 0
            # Keep the watermark so replayed evidence cannot regain a hold.
            if type(seq) is int and type(stamp) is int:
                if self.last_identity is None or boot != self.last_identity[0] or seq > self.last_identity[1]:
                    self.last_identity = identity
            self.result = self._result(False, list(dict.fromkeys(reasons)))
            return self.result
        if identity != self.last_identity:
            if self.last is not None and stamp - self.last > self.max_age_ns:
                self.first, self.samples = None, 0
            if self.first is None:
                self.first = stamp
            self.samples += 1
            self.last = stamp
            self.last_identity = identity
        duration = 0 if self.first is None else stamp - self.first
        verified = duration >= self.hold_ns and self.samples >= 3
        self.result = self._result(verified, [] if verified else ["STATIONARY_HOLD_PENDING"], duration)
        return self.result
