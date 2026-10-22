"""No hardware: exercise the braking tool against a simulated framed serial link."""
import pytest

from autokart_vehicle_bridge.protocol import build_frame, parse_frame
import braking_test
from braking_test import SerialLink, BrakingAbort, stop_and_measure, summarize


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

    def telemetry(self, *, state=0, fault=0, flags=17, speed=0):
        # fwseq,t_ms,state,fault,flags,en,brk,dac_mv,counts,speed_mmps,
        # steer_raw,brake_raw,ack_seq,link_age_ms
        fields = [1, 20, state, fault, flags, 0, 1, 0, 0, speed, 0, 0, 1, 0]
        self.incoming.extend(build_frame("TEL," + ",".join(map(str, fields))))


def test_simulated_serial_link_stops_with_zero_dac_and_brake():
    fake = FakePort()
    now = [0.0]
    link = SerialLink(fake, clock=lambda: now[0])
    fake.telemetry(state=1, speed=1000)
    assert link.require_healthy(require_armed=True).speed_mmps == 1000
    link.send("CMD", 2000, 0)
    link.software_stop()
    assert fake.sent[-2].startswith("CMD,")
    assert fake.sent[-2].endswith(",0,0,100")
    assert fake.sent[-1].startswith("DIS,")


def test_simulated_stop_run_sends_mission_stop_sequence_and_records_trace(monkeypatch):
    class MotionPort(FakePort):
        def __init__(self):
            super().__init__()
            self.speed = 1000
            self.disarmed = False

        def write(self, data):
            super().write(data)
            if parse_frame(data).startswith("DIS,"):
                self.disarmed = True

        def read(self, n):
            if self.disarmed:
                self.speed = max(0, self.speed - 200)
                self.telemetry(state=0, speed=self.speed)
            return super().read(n)

    fake = MotionPort()
    now = [0.0]
    monkeypatch.setattr(braking_test.time, "sleep", lambda seconds: now.__setitem__(0, now[0] + seconds))
    link = SerialLink(fake, clock=lambda: now[0])
    fake.telemetry(state=1, speed=1000)
    link.require_healthy(require_armed=True)
    result = stop_and_measure(link)
    assert result["speed_at_cut_mps"] == 1.0
    assert result["stopping_distance_m"] > 0
    assert result["trace"][-1]["speed_mps"] == 0
    assert fake.sent[0].endswith(",0,0,100")
    assert fake.sent[1].startswith("DIS,")


def test_simulated_serial_link_fails_closed_on_loss_fault_and_estop():
    fake = FakePort()
    now = [0.0]
    link = SerialLink(fake, clock=lambda: now[0])
    fake.telemetry(state=1)
    link.require_healthy(require_armed=True)
    now[0] = 0.26
    with pytest.raises(BrakingAbort, match="stale"):
        link.require_healthy(require_armed=True)
    fake.telemetry(state=2, fault=4)
    with pytest.raises(BrakingAbort, match="fault"):
        link.require_healthy()
    fake.telemetry(state=1, flags=48)  # fitted input, pressed; bit5 set and bit0 clear
    with pytest.raises(BrakingAbort, match="E-stop"):
        link.require_healthy()
    fake.telemetry(state=1, flags=16)  # no input fitted; a floating GPIO cannot block by itself
    assert not link.require_healthy().estop_input_fitted


def test_no_encoder_telemetry_refuses_braking_measurement():
    fake = FakePort()
    link = SerialLink(fake)
    fake.telemetry(speed=-1)
    # The current VCU also reports counts=-1; speed alone must be enough to reject it.
    with pytest.raises(BrakingAbort, match="no axle encoder"):
        link.require_healthy()


def test_final_five_runs_use_minimum_mean_deceleration_with_twenty_percent_derate():
    runs = [{"hold_s": 5, "speed_at_cut_mps": 1.0,
             "mean_deceleration_mps2": a, "stopping_distance_m": 0.5}
            for a in (1.0, 0.9, 0.8, 1.1, 1.2)]
    assert summarize(runs)["a_verified_mps2_suggestion"] == pytest.approx(0.64)
    with pytest.raises(BrakingAbort, match="fewer than five"):
        summarize(runs[:4])
