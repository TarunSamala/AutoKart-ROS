"""No hardware: exercise the controlled-run tool against a simulated framed serial link, the same
pattern tools/test_braking_test.py already uses for tools/braking_test.py."""
import csv
import io

import pytest

from autokart_vehicle_bridge.protocol import build_frame, parse_frame
import controlled_run
from controlled_run import (ControlledRunAbort, ControlledRunLink, arm, confirm_stopped,
                            log_row, perform_stage, wait_initial_tel)


class FakePort:
    def __init__(self):
        self.sent = []
        self.incoming = bytearray()

    def write(self, data):
        self.sent.append(parse_frame(data))

    def read(self, _n):
        data = bytes(self.incoming)
        self.incoming.clear()
        return data

    def telemetry(self, *, state=0, fault=0, flags=17, dac_mv=0, en=0, brk=1):
        # fwseq,t_ms,state,fault,flags,en,brk,dac_mv,counts,speed_mmps,steer_raw,brake_raw,ack_seq,link_age_ms
        # counts=-1, speed_mmps=-1: this kart has no axle encoder (always unavailable).
        fields = [1, 20, state, fault, flags, en, brk, dac_mv, -1, -1, 0, 0, 1, 0]
        self.incoming.extend(build_frame("TEL," + ",".join(map(str, fields))))


def writer_to(rows):
    buf = io.StringIO()
    fieldnames = ["run", "hold_s", "t_s", "phase", "state", "dac_mv", "en", "brk", "fault",
                 "encoder_counts", "speed_mmps"]
    writer = csv.DictWriter(buf, fieldnames=fieldnames)
    writer.writeheader()

    class Recording:
        def writerow(self, row):
            rows.append(row)
            writer.writerow(row)
    return Recording()


def test_no_encoder_telemetry_does_not_block_this_tool():
    """The whole point of this tool existing separately from braking_test.py: a kart with
    speed_mmps/counts always -1 must still be usable for a controlled run."""
    fake = FakePort()
    link = ControlledRunLink(fake)
    fake.telemetry(state=1, dac_mv=2000)
    tel = link.require_healthy(require_armed=True)
    assert tel.speed_mmps == -1 and tel.counts == -1


def test_mission_stop_sequence_sends_zero_throttle_full_brake_then_disarm():
    fake = FakePort()
    link = ControlledRunLink(fake)
    link.mission_stop_sequence()
    assert fake.sent[-2].startswith("CMD,")
    assert fake.sent[-2].endswith(",0,0,100")
    assert fake.sent[-1].startswith("DIS,")


def test_wait_initial_tel_requires_safe_start():
    fake = FakePort()
    now = [0.0]
    link = ControlledRunLink(fake, clock=lambda: now[0])
    fake.telemetry(state=0)
    wait_initial_tel(link)
    assert link.tel.state == 0


class _AlwaysSafePort(FakePort):
    """Re-emits fresh SAFE telemetry on every read, so a deliberately-never-arms VCU still looks
    like a live, healthy link (never stale) while the test waits out the ARM timeout."""
    def read(self, n):
        self.telemetry(state=0)
        return super().read(n)


def test_arm_refuses_unless_vcu_reaches_armed(monkeypatch):
    fake = _AlwaysSafePort()
    now = [0.0]
    monkeypatch.setattr(controlled_run.time, "sleep", lambda s: now.__setitem__(0, now[0] + s))
    link = ControlledRunLink(fake, clock=lambda: now[0])
    fake.telemetry(state=0)
    with pytest.raises(ControlledRunAbort, match="did not ARM"):
        arm(link)
    assert any(frame.startswith("ARM,") for frame in fake.sent)


def test_arm_succeeds_when_vcu_reports_armed(monkeypatch):
    class ArmingPort(FakePort):
        def write(self, data):
            super().write(data)
            if parse_frame(data).startswith("ARM,"):
                self.armed = True

        def read(self, n):
            if getattr(self, "armed", False):
                self.telemetry(state=1)
            return super().read(n)

    fake = ArmingPort()
    now = [0.0]
    monkeypatch.setattr(controlled_run.time, "sleep", lambda s: now.__setitem__(0, now[0] + s))
    link = ControlledRunLink(fake, clock=lambda: now[0])
    fake.telemetry(state=0)
    arm(link)
    assert link.tel.state == 1


def test_confirm_stopped_requires_safe_and_en_cleared(monkeypatch):
    now = [0.0]
    monkeypatch.setattr(controlled_run.time, "sleep", lambda s: now.__setitem__(0, now[0] + s))
    fake = FakePort()
    link = ControlledRunLink(fake, clock=lambda: now[0])
    fake.telemetry(state=0, en=0)
    rows = []
    confirm_stopped(link, writer_to(rows), 1, 2.0, 0.0)
    assert rows[-1]["state"] == "SAFE" and rows[-1]["en"] == 0


class _StillArmedPort(FakePort):
    """Re-emits fresh ARMED/EN-set telemetry on every read: the VCU never reports back to SAFE."""
    def read(self, n):
        self.telemetry(state=1, en=1)
        return super().read(n)


def test_confirm_stopped_aborts_if_still_armed_or_en_set_after_timeout(monkeypatch):
    now = [0.0]

    def fake_sleep(seconds):
        now[0] += seconds

    monkeypatch.setattr(controlled_run.time, "sleep", fake_sleep)
    fake = _StillArmedPort()
    link = ControlledRunLink(fake, clock=lambda: now[0])
    fake.telemetry(state=1, en=1)  # still armed and driving; never reports SAFE/EN-off
    with pytest.raises(ControlledRunAbort, match="did not return to SAFE"):
        confirm_stopped(link, writer_to([]), 1, 2.0, 0.0)


def test_logged_rows_report_unavailable_not_fabricated_zero_for_missing_encoder():
    fake = FakePort()
    link = ControlledRunLink(fake)
    fake.telemetry(state=1, dac_mv=2000)
    tel = link.require_healthy()
    rows = []
    log_row(writer_to(rows), 1, 2.0, 0.5, tel, "HOLD")
    assert rows[-1]["encoder_counts"] == "UNAVAILABLE"
    assert rows[-1]["speed_mmps"] == "UNAVAILABLE"


def test_perform_stage_ramps_holds_and_stops_within_budget(monkeypatch):
    class RunPort(FakePort):
        def write(self, data):
            super().write(data)
            text = parse_frame(data)
            if text.startswith("ARM,"):
                self.state = 1
            elif text.startswith("CMD,"):
                _, _, throttle_mv, _, _ = text.split(",")
                self.dac_mv = int(throttle_mv)
            if text.startswith("DIS,"):
                self.state = 0
                self.dac_mv = 0

        def read(self, n):
            self.telemetry(state=getattr(self, "state", 0), dac_mv=getattr(self, "dac_mv", 0),
                           en=1 if getattr(self, "state", 0) == 1 else 0)
            return super().read(n)

    fake = RunPort()
    now = [0.0]
    monkeypatch.setattr(controlled_run.time, "sleep", lambda s: now.__setitem__(0, now[0] + s))
    link = ControlledRunLink(fake, clock=lambda: now[0])
    rows = []
    result = perform_stage(link, writer_to(rows), 1, 2.0)
    assert result == {"run": 1, "hold_s": 2.0, "stopped_cleanly": True}
    assert any(row["phase"] == "RAMP" for row in rows)
    assert any(row["phase"] == "HOLD" for row in rows)
    assert any(row["phase"] == "STOP_CONFIRM" for row in rows)
    assert rows[-1]["state"] == "SAFE"


def test_perform_stage_aborts_on_vcu_fault_during_hold(monkeypatch):
    class FaultingPort(FakePort):
        def write(self, data):
            super().write(data)
            text = parse_frame(data)
            if text.startswith("ARM,"):
                self.state = 1
            elif text.startswith("CMD,"):
                _, _, throttle_mv, _, _ = text.split(",")
                self.dac_mv = int(throttle_mv)
                self.ticks = getattr(self, "ticks", 0) + 1

        def read(self, n):
            fault = 4 if getattr(self, "ticks", 0) > 3 else 0  # DAC_FAIL after the ramp completes
            self.telemetry(state=2 if fault else getattr(self, "state", 0),
                           fault=fault, dac_mv=getattr(self, "dac_mv", 0),
                           en=1 if getattr(self, "state", 0) == 1 else 0)
            return super().read(n)

    fake = FaultingPort()
    now = [0.0]
    monkeypatch.setattr(controlled_run.time, "sleep", lambda s: now.__setitem__(0, now[0] + s))
    link = ControlledRunLink(fake, clock=lambda: now[0])
    with pytest.raises(ControlledRunAbort, match="fault"):
        perform_stage(link, writer_to([]), 1, 2.0)


def test_main_refuses_hold_times_that_are_not_ascending_or_do_not_start_short(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(controlled_run.sys, "argv",
                        ["controlled_run.py", "/dev/fake", "--hold-times", "5", "2", "--out-dir", str(tmp_path)])
    with pytest.raises(SystemExit):
        controlled_run.main()


def test_main_refuses_motion_without_every_typed_confirmation(monkeypatch, tmp_path):
    monkeypatch.setattr(controlled_run.sys, "argv",
                        ["controlled_run.py", "/dev/fake", "--out-dir", str(tmp_path)])
    monkeypatch.setattr("builtins.input", lambda _prompt: "CLEAR20")  # stops at the second prompt
    with pytest.raises(SystemExit, match="No motion requested"):
        controlled_run.main()
