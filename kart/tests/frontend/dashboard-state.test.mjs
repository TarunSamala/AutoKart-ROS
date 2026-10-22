import assert from "node:assert/strict";
import test from "node:test";

if (!globalThis.CustomEvent) {
  globalThis.CustomEvent = class CustomEvent extends Event {
    constructor(type, options = {}) { super(type); this.detail = options.detail; }
  };
}

const {DashboardStateStore} = await import("../../dashboard/js/state-store.js");

function record(sequence, timestamp, x, yaw = 0, faults = []) {
  return {
    sequence,
    timestamp_s: timestamp,
    valid: true,
    stale: false,
    pose: {x_m: x, y_m: 0, z_m: 0, roll_rad: 0, pitch_rad: 0, yaw_rad: yaw},
    motion: {speed_mps: x, longitudinal_speed_mps: x, lateral_speed_mps: 0},
    track: {s_m: x, lateral_m: 0},
    vcu: {state: faults.length ? "EMERGENCY_STOP" : "AUTONOMOUS", faults},
    subsystems: {},
  };
}

test("bounded telemetry, interpolation, and measured/display timestamps", () => {
  const store = new DashboardStateStore(3);
  for (let index = 0; index < 5; index += 1) store.append(record(index, index, index));
  assert.equal(store.records.length, 3);
  const interpolated = store.selectTime(2.5, true);
  assert.equal(interpolated.pose.x_m, 2.5);
  assert.equal(interpolated.display_interpolated, true);
  assert.equal(interpolated.measured_timestamp_s, 2);
});

test("fault and state transitions are derived from telemetry evidence", () => {
  const store = new DashboardStateStore();
  store.append(record(0, 0, 0));
  store.append(record(1, 1, 1, 0, ["MOTOR-003"]));
  assert.equal(store.faultEvents.length, 1);
  assert.equal(store.faultEvents[0].fault, "MOTOR-003");
  assert.equal(store.stateEvents.at(-1).state, "EMERGENCY_STOP");
});

test("backward data becomes stale and invalid", () => {
  const store = new DashboardStateStore();
  store.append(record(0, 1, 0));
  store.append(record(1, .5, 1));
  assert.equal(store.records.at(-1).stale, true);
  assert.equal(store.records.at(-1).valid, false);
});
