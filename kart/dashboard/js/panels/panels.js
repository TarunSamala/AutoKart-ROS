import {commandText, format} from "../state-store.js";
import {FrictionCircle} from "../plots/friction-circle.js";

const $ = id => document.getElementById(id);
const set = (id, value) => { const element = $(id); if (element) element.textContent = value; };
const finite = value => Number.isFinite(Number(value));
const statusClass = value => String(value ?? "").toLowerCase().replaceAll("_", "-");
const DEG = 180 / Math.PI;

function subsystemFault(name, frame) {
  const direct = frame.subsystems?.[name]?.fault;
  const faults = frame.vcu?.faults ?? [];
  const prefix = name === "motor" ? "MOTOR" : name === "brake" ? "BRAKE" : "STEER";
  return direct || faults.find(fault => String(fault).startsWith(prefix)) || null;
}

export class PanelRenderer {
  constructor(store) {
    this.store = store;
    this.selectedWheel = "front_left";
    this.frictionCircle = new FrictionCircle($("friction-circle"));
    this.setupWheelSelector();
    store.addEventListener("static", event => this.staticData(event.detail));
    store.addEventListener("time", event => this.render(event.detail));
    store.addEventListener("connection", () => this.connection());
  }

  setupWheelSelector() {
    const selector = $("wheel-selector");
    const names = {front_left: "Front left", front_right: "Front right", rear_left: "Rear left", rear_right: "Rear right"};
    for (const [id, label] of Object.entries(names)) {
      const button = document.createElement("button");
      button.type = "button";
      button.dataset.wheel = id;
      button.textContent = label;
      if (id === this.selectedWheel) button.classList.add("active");
      button.addEventListener("click", () => {
        this.selectedWheel = id;
        selector.querySelectorAll("button").forEach(item => item.classList.toggle("active", item === button));
        this.render(this.store.current);
      });
      selector.append(button);
    }
  }

  staticData({manifest, track, kart}) {
    set("scenario-name", manifest.scenario);
    set("track-name", track.name);
    set("run-id", manifest.run_id);
    set("run-mode", manifest.mode.toUpperCase());
    set("scene-fidelity", `TRACK: ${track.placeholder ? "SYNTHETIC TEST TRACK" : "PHASE 5 GEOMETRY"} · KART: ${kart.placeholder ? "PLACEHOLDER MODEL" : kart.model}`);
    $("replay-end").textContent = `${format(manifest.end_time_s, 3)} s`;
  }

  connection() {
    const state = this.store.connection;
    const element = $("connection-state");
    element.textContent = state.toUpperCase();
    element.className = `status-pill ${state === "live" ? "autonomous" : state === "disconnected" ? "emergency-stop" : "neutral"}`;
  }

  render(frame) {
    if (!frame) return;
    const pose = frame.pose;
    const motion = frame.motion;
    const track = frame.track;
    const vcu = frame.vcu ?? {};
    const commands = frame.commands ?? {};
    const final = commands.final ?? {};
    const requested = commands.requested;
    const state = vcu.state ?? "NO DATA";
    const stateCss = statusClass(state);
    set("sim-time", `${format(frame.timestamp_s, 3)} s`);
    set("header-speed", format(motion.speed_mps, 1));
    set("scene-position", `${format(pose.x_m)} / ${format(pose.y_m)} / ${format(pose.z_m)}`);
    set("scene-track-position", `${format(track.s_m)} / ${format(track.lateral_m)}`);
    const vcuElement = $("vcu-state");
    vcuElement.textContent = state;
    vcuElement.className = `status-pill ${stateCss}`;
    const independent = vcu.independent_safety ?? {};
    const independentText = independent.stop_latched ? `SAFETY ${independent.fault ?? "TRIPPED"}` : "SAFETY HEALTHY";
    set("independent-state", independentText);
    $("independent-state").className = `status-pill ${independent.stop_latched ? "emergency-stop" : "neutral"}`;
    const critical = ["EMERGENCY_STOP", "FAULT_LOCKOUT"].includes(state);
    $("global-alert").textContent = critical ? `${state}: ${(vcu.faults ?? []).join(", ") || "critical response active"}` : "Read-only telemetry";

    set("vehicle-speed", format(motion.speed_mps));
    set("target-speed", requested ? format(Math.max(0, Number(requested.drive ?? 0) * 8), 2) : "not recorded");
    set("long-accel", format(motion.longitudinal_acceleration_mps2));
    set("lat-accel", format(motion.lateral_acceleration_mps2));
    set("yaw-rate", format(motion.yaw_rate_radps));
    set("sideslip", format(motion.sideslip_rad * DEG));
    set("heading", format(pose.yaw_rad * DEG, 1));
    set("track-s", format(track.s_m));
    set("lateral-error", format(track.lateral_m));
    set("curvature", format(track.curvature_1pm, 4));
    set("distance-travelled", format(this.store.distanceTravelled()));
    set("vehicle-validity", frame.valid ? (frame.display_interpolated ? "INTERPOLATED DISPLAY" : "VALID MEASUREMENT") : "INVALID");
    set("command-requested", commandText(commands.requested));
    set("command-validated", commandText(commands.validated));
    set("command-limited", commandText(commands.safety_limited));
    set("command-actual", commandText(commands.final));

    this.renderSafety(frame);
    this.renderActuators(frame);
    this.renderTires(frame);
    this.renderPower(frame);
    this.renderRace(frame);
    this.renderCorner(frame);
  }

  renderSafety(frame) {
    const vcu = frame.vcu ?? {};
    const state = vcu.state ?? "—";
    const independent = vcu.independent_safety ?? {};
    set("safety-vcu-state", state);
    set("controlled-stop-phase", vcu.controlled_stop_phase ? `controlled-stop phase: ${vcu.controlled_stop_phase}` : "normal operation");
    $("safety-state-card").classList.toggle("critical", ["EMERGENCY_STOP", "FAULT_LOCKOUT"].includes(state));
    set("safety-independent", independent.stop_latched ? `TRIPPED · ${independent.fault ?? "fault"}` : "HEALTHY");
    set("safety-estop", independent.fault === "PHYSICAL_ESTOP" ? "ACTIVE" : "RELEASED");
    set("safety-motor-enable", vcu.motor_enable ? "ENABLED" : "DISABLED");
    set("safety-contactor", vcu.contactor_open ? "OPEN REQUEST" : "CLOSED PERMITTED");
    set("command-age", finite(vcu.command_age_s) ? `${format(vcu.command_age_s, 3)} s` : "not recorded");
    set("reset-eligibility", ["EMERGENCY_STOP", "FAULT_LOCKOUT"].includes(state) ? "GUARDS REQUIRED" : "NOT REQUESTED");
    const limits = vcu.safety_limits_mps ?? {active: vcu.active_speed_limit_mps};
    const list = $("speed-limits");
    list.replaceChildren();
    const values = Object.entries(limits).filter(([, value]) => finite(value));
    const minimum = Math.min(...values.filter(([name]) => name !== "active").map(([, value]) => Number(value)));
    for (const [name, raw] of values) {
      if (name === "active") continue;
      const value = Number(raw);
      const row = document.createElement("div");
      row.className = `bar-row ${Math.abs(value - minimum) < 1e-6 ? "active" : ""}`;
      row.innerHTML = `<label>${name.replaceAll("_", " ")}</label><span class="bar"><i style="width:${Math.max(1, Math.min(100, value / 8 * 100))}%"></i></span><strong>${format(value, 2)}</strong>`;
      list.append(row);
    }
    const stopping = vcu.stopping_distance ?? {};
    const components = [
      ["delay_m", "#4f8df7"], ["braking_m", "#e6ae45"], ["uncertainty_m", "#a98cf4"], ["margin_m", "#ef5d68"],
    ];
    const stack = $("stopping-distance");
    stack.replaceChildren();
    const total = Math.max(.001, Number(stopping.total_m ?? 0));
    for (const [name, color] of components) {
      const item = document.createElement("i");
      item.title = `${name.replaceAll("_", " ")}: ${format(stopping[name])} m`;
      item.style.width = `${Number(stopping[name] ?? 0) / total * 100}%`;
      item.style.background = color;
      stack.append(item);
    }
    let caption = stack.nextElementSibling;
    if (!caption || caption.tagName !== "SMALL") {
      caption = document.createElement("small");
      stack.after(caption);
    }
    caption.textContent = `total ${format(stopping.total_m)} m / clear ${format(stopping.clear_distance_m)} m`;
    const faults = vcu.faults ?? [];
    set("fault-count", String(faults.length));
    const faultList = $("active-fault-list");
    faultList.replaceChildren();
    if (!faults.length) faultList.innerHTML = `<li class="muted">None</li>`;
    else for (const fault of faults) {
      const item = document.createElement("li"); item.textContent = fault; faultList.append(item);
    }
  }

  renderActuators(frame) {
    const steering = frame.subsystems?.steering ?? {};
    const brake = frame.subsystems?.brake ?? {};
    const motor = frame.subsystems?.motor ?? {};
    const requested = frame.commands?.requested ?? {};
    const final = frame.commands?.final ?? {};
    const steeringFault = subsystemFault("steering", frame);
    const brakeFault = subsystemFault("brake", frame);
    const motorFault = subsystemFault("motor", frame);
    set("actuator-health", steeringFault || brakeFault || motorFault ? "FAULT PRESENT" : "NOMINAL");
    set("steering-fault", steeringFault ?? "HEALTHY");
    set("steer-requested", format(requested.steering_rad, 3));
    set("steer-validated", format(steering.validated_target_rad, 3));
    set("steer-actuator", format(steering.shaft_angle_rad, 3));
    set("steer-linkage", format(steering.linkage_angle_rad, 3));
    set("steer-left", format(steering.left_wheel_angle_rad, 3));
    set("steer-right", format(steering.right_wheel_angle_rad, 3));
    set("steer-error", format(steering.following_error_rad, 3));
    set("steer-current", format(steering.current_a, 1));
    set("brake-fault", brakeFault ?? "HEALTHY");
    set("brake-command", `${format(Number(final.brake ?? 0) * 100, 0)}%`);
    set("brake-position", `${format(Number(brake.actual_position_m ?? 0) * 1000, 1)} mm`);
    set("cable-force", `${format(brake.cable_tension_n, 0)} N`);
    set("brake-pressure", `${format(Number(brake.hydraulic_pressure_pa ?? 0) / 1e6, 2)} MPa`);
    set("brake-torque", `${format(brake.brake_torque_nm, 0)} Nm`);
    set("brake-decel", `${format(-frame.motion.longitudinal_acceleration_mps2, 2)} m/s²`);
    set("motor-fault", motorFault ?? "HEALTHY");
    set("torque-requested", format(motor.requested_torque_nm, 1));
    set("torque-actual", format(motor.actual_torque_nm, 1));
    set("motor-current", format(motor.motor_current_a, 1));
    set("motor-speed", format(motor.motor_speed_radps, 0));
    set("chain-torque", format(motor.chain_torque_nm, 1));
    set("controller-temperature", format(motor.controller_temperature_c, 1));
  }

  renderTires(frame) {
    const fourWheel = frame.subsystems?.four_wheel ?? {};
    const wheel = fourWheel.wheels?.[this.selectedWheel] ?? {};
    const surface = frame.track?.wheel_surfaces?.[this.selectedWheel] ?? {};
    set("inside-rear-state", fourWheel.diagnostics?.inside_rear_unloaded ? "INSIDE REAR UNLOADED" : "ALL CONTACT");
    this.frictionCircle.draw(wheel);
    const metrics = $("wheel-metrics");
    const rows = [
      ["Normal load", format(wheel.vertical_load_n, 0), "N"],
      ["Longitudinal force", format(wheel.longitudinal_force_n, 0), "N"],
      ["Lateral force", format(wheel.lateral_force_n, 0), "N"],
      ["Slip ratio", format(wheel.slip_ratio, 3), ""],
      ["Slip angle", format(Number(wheel.slip_angle_rad ?? 0) * DEG, 2), "deg"],
      ["Utilization", format(Number(wheel.utilisation ?? 0) * 100, 1), "%"],
      ["Wheel speed", format(wheel.angular_speed_radps, 1), "rad/s"],
      ["Surface", surface.material_id ?? "not recorded", ""],
      ["Friction μ", format(surface.friction_longitudinal, 2), ""],
      ["Contact", wheel.contact_valid === false ? "LIFTED" : "VALID", ""],
    ];
    metrics.innerHTML = rows.map(([label, value, unit]) => `<div><label>${label}</label><strong>${value}</strong><small>${unit}</small></div>`).join("");
  }

  renderPower(frame) {
    const battery = frame.subsystems?.battery ?? {};
    const motor = frame.subsystems?.motor ?? {};
    const brake = frame.subsystems?.brake ?? {};
    const domains = frame.subsystems?.power_domains ?? {};
    set("battery-power", `${format(battery.power_w, 0)} W`);
    set("bus-voltage", `${format(battery.bus_voltage_v, 1)} V`);
    set("motor-power", `${format(motor.electrical_power_w, 0)} W`);
    set("mechanical-power", `${format(motor.mechanical_power_w, 0)} W`);
    set("battery-soc", format(Number(battery.soc ?? 0) * 100, 1));
    set("battery-voltage", format(battery.terminal_voltage_v, 1));
    set("battery-current", format(battery.current_a, 1));
    set("contactor-state", battery.contactor_state ?? "—");
    set("steering-voltage", format(domains.steering?.voltage_v, 1));
    set("brake-voltage", format(domains.brake?.voltage_v, 1));
    set("motor-temperature", format(motor.motor_temperature_c, 1));
    set("brake-temperature", format(brake.disc_temperature_c, 1));
    const bad = battery.bms_shutdown || domains.steering?.brownout || domains.brake?.brownout;
    set("power-health", bad ? "FAULT / DERATING" : "NOMINAL");
  }

  renderRace(frame) {
    const race = frame.race_control ?? {};
    const kart = race.kart ?? race.karts?.[race.namespace ?? "/kart_01"] ?? {};
    set("race-session-state", race.session_state ?? "SETUP");
    set("race-lap-count", String(kart.lap_count ?? race.lap_count ?? 0));
    set("race-current-lap", finite(kart.current_lap_s) ? format(kart.current_lap_s, 3) : "—");
    set("race-best-lap", finite(kart.best_lap_s) ? format(kart.best_lap_s, 3) : "—");
    const sectors = kart.sector_times_s ?? race.sector_times_s ?? [];
    set("race-sector", sectors.length ? format(sectors.at(-1), 3) : "—");
    set("race-cone-penalties", String(kart.cone_penalties ?? 0));
    set("race-track-limits", String(kart.track_limit_penalties ?? 0));
    set("race-collisions", String(kart.collisions ?? 0));
    set("race-classification", kart.dnf ? "DNF" : kart.estop ? "E-STOP" : race.session_state === "FINISHED" ? "FINISHED" : "RUNNING");
  }

  renderCorner(frame) {
    const corner = this.store.currentCorner();
    set("corner-id", corner?.id ?? "—");
    const summary = $("corner-summary");
    if (!corner) {
      summary.innerHTML = `<strong>No active corner</strong><span>Current track distance is outside segmented corner geometry.</span>`;
      $("corner-metrics").replaceChildren();
      return;
    }
    summary.innerHTML = `<strong>${corner.id} · ${corner.direction}</strong><span>${corner.classification?.type?.replaceAll("_", " ") ?? "corner"} · geometric apex ${format(corner.geometric_apex_s_m)} m</span>`;
    const phase = frame.track.s_m < corner.geometric_apex_s_m ? "turn-in / approach" : "apex / exit";
    $("corner-metrics").innerHTML = [
      ["Phase", phase, ""], ["Curvature", format(frame.track.curvature_1pm, 4), "1/m"],
      ["Speed", format(frame.motion.speed_mps), "m/s"], ["Lateral accel", format(frame.motion.lateral_acceleration_mps2), "m/s²"],
      ["Steering error", format(frame.subsystems?.steering?.following_error_rad, 3), "rad"],
      ["Inside rear", frame.subsystems?.four_wheel?.diagnostics?.inside_rear_unloaded ? "UNLOADED" : "LOADED", ""],
    ].map(([label, value, unit]) => `<div><label>${label}</label><strong>${value}</strong><small>${unit}</small></div>`).join("");
  }
}
