const COLORS = ["#5bc0d8", "#e6ae45", "#ef5d68", "#4ac087", "#a98cf4"];

const SIGNAL_GROUPS = {
  speed: [
    {name: "actual speed [m/s]", get: r => r.motion.speed_mps},
    {name: "requested target [m/s]", get: r => Math.max(0, Number(r.commands?.requested?.drive ?? 0) * 8)},
    {name: "safety limit [m/s]", get: r => r.vcu?.active_speed_limit_mps},
  ],
  actuators: [
    {name: "steering [rad]", get: r => r.subsystems?.steering?.linkage_angle_rad},
    {name: "brake command [-]", get: r => r.commands?.final?.brake},
    {name: "brake pressure [MPa]", get: r => Number(r.subsystems?.brake?.hydraulic_pressure_pa ?? 0) / 1e6},
    {name: "motor torque [Nm]", get: r => r.subsystems?.motor?.actual_torque_nm},
  ],
  dynamics: [
    {name: "longitudinal accel [m/s²]", get: r => r.motion.longitudinal_acceleration_mps2},
    {name: "lateral accel [m/s²]", get: r => r.motion.lateral_acceleration_mps2},
    {name: "yaw rate [rad/s]", get: r => r.motion.yaw_rate_radps},
    {name: "lateral error [m]", get: r => r.track?.lateral_m},
  ],
  tire: [
    {name: "FL utilization [-]", get: r => r.subsystems?.four_wheel?.wheels?.front_left?.utilisation},
    {name: "FR utilization [-]", get: r => r.subsystems?.four_wheel?.wheels?.front_right?.utilisation},
    {name: "RL utilization [-]", get: r => r.subsystems?.four_wheel?.wheels?.rear_left?.utilisation},
    {name: "RR utilization [-]", get: r => r.subsystems?.four_wheel?.wheels?.rear_right?.utilisation},
  ],
  power: [
    {name: "bus voltage [V]", get: r => r.subsystems?.battery?.bus_voltage_v},
    {name: "battery current [A]", get: r => r.subsystems?.battery?.current_a},
    {name: "motor temp [°C]", get: r => r.subsystems?.motor?.motor_temperature_c},
    {name: "brake temp [°C]", get: r => r.subsystems?.brake?.disc_temperature_c},
  ],
  safety: [
    {name: "active speed limit [m/s]", get: r => r.vcu?.active_speed_limit_mps},
    {name: "stopping distance [m]", get: r => r.vcu?.stopping_distance?.total_m},
    {name: "clear distance [m]", get: r => r.vcu?.stopping_distance?.clear_distance_m},
    {name: "command age [s]", get: r => r.vcu?.command_age_s},
  ],
};

function fitCanvas(canvas) {
  const ratio = Math.min(2, window.devicePixelRatio || 1);
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, rect.width);
  const height = Math.max(1, rect.height);
  if (canvas.width !== Math.floor(width * ratio) || canvas.height !== Math.floor(height * ratio)) {
    canvas.width = Math.floor(width * ratio);
    canvas.height = Math.floor(height * ratio);
  }
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  return {context, width, height};
}

export class TimeSeriesPlot {
  constructor(canvas, store) {
    this.canvas = canvas;
    this.store = store;
    this.group = "speed";
    this.cursorTime = null;
    new ResizeObserver(() => this.draw()).observe(canvas);
    store.addEventListener("time", () => this.draw());
    store.addEventListener("records", () => this.draw());
    canvas.addEventListener("pointermove", event => {
      const rect = canvas.getBoundingClientRect();
      const bounds = this.bounds();
      this.cursorTime = bounds.start + Math.max(0, Math.min(1, (event.clientX - rect.left - 44) / Math.max(1, rect.width - 58))) * (bounds.end - bounds.start);
      this.draw();
    });
    canvas.addEventListener("pointerleave", () => { this.cursorTime = null; this.draw(); });
    canvas.addEventListener("click", event => {
      const rect = canvas.getBoundingClientRect();
      const bounds = this.bounds();
      const time = bounds.start + Math.max(0, Math.min(1, (event.clientX - rect.left - 44) / Math.max(1, rect.width - 58))) * (bounds.end - bounds.start);
      store.selectTime(time);
      this.dispatchSeek?.(time);
    });
  }

  bounds() {
    return {start: this.store.records[0]?.timestamp_s ?? 0, end: this.store.records.at(-1)?.timestamp_s ?? 1};
  }

  setGroup(group) { this.group = group; this.draw(); }

  draw() {
    const {context: ctx, width, height} = fitCanvas(this.canvas);
    ctx.fillStyle = "#0a0f14"; ctx.fillRect(0, 0, width, height);
    const records = this.store.records;
    if (records.length < 2) return;
    const signals = SIGNAL_GROUPS[this.group] ?? SIGNAL_GROUPS.speed;
    const values = signals.flatMap(signal => records.map(record => Number(signal.get(record))).filter(Number.isFinite));
    if (!values.length) return;
    let minimum = Math.min(...values), maximum = Math.max(...values);
    if (Math.abs(maximum - minimum) < 1e-9) { minimum -= 1; maximum += 1; }
    const padding = (maximum - minimum) * .08;
    minimum -= padding; maximum += padding;
    const plot = {left: 44, right: width - 12, top: 17, bottom: height - 19};
    const bounds = this.bounds();
    const x = time => plot.left + (time - bounds.start) / Math.max(1e-9, bounds.end - bounds.start) * (plot.right - plot.left);
    const y = value => plot.bottom - (value - minimum) / (maximum - minimum) * (plot.bottom - plot.top);
    ctx.font = "8px IBM Plex Mono, monospace";
    ctx.textAlign = "right";
    for (let index = 0; index <= 4; index += 1) {
      const value = minimum + (maximum - minimum) * index / 4;
      const py = y(value);
      ctx.strokeStyle = "#1d2832"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(plot.left, py); ctx.lineTo(plot.right, py); ctx.stroke();
      ctx.fillStyle = "#647382"; ctx.fillText(value.toFixed(1), plot.left - 6, py + 3);
    }
    ctx.textAlign = "left";
    signals.forEach((signal, signalIndex) => {
      ctx.beginPath();
      let started = false;
      for (const record of records) {
        const value = Number(signal.get(record));
        if (!Number.isFinite(value)) { started = false; continue; }
        const px = x(record.timestamp_s), py = y(value);
        if (!started) { ctx.moveTo(px, py); started = true; } else ctx.lineTo(px, py);
      }
      ctx.strokeStyle = COLORS[signalIndex % COLORS.length];
      ctx.lineWidth = 1.4;
      ctx.stroke();
      ctx.fillStyle = COLORS[signalIndex % COLORS.length];
      ctx.fillRect(plot.left + signalIndex * 170, 5, 9, 2);
      ctx.fillText(signal.name, plot.left + 13 + signalIndex * 170, 9);
    });
    const current = this.store.current?.timestamp_s;
    if (Number.isFinite(current)) {
      ctx.strokeStyle = "#dce6ec"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(x(current), plot.top); ctx.lineTo(x(current), plot.bottom); ctx.stroke();
    }
    if (Number.isFinite(this.cursorTime)) {
      ctx.strokeStyle = "#6d7c89"; ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.moveTo(x(this.cursorTime), plot.top); ctx.lineTo(x(this.cursorTime), plot.bottom); ctx.stroke();
      ctx.setLineDash([]);
    }
  }

  exportPng() {
    const link = document.createElement("a");
    link.download = `autokart-${this.group}-plot.png`;
    link.href = this.canvas.toDataURL("image/png");
    link.click();
  }
}

export class FaultTimeline {
  constructor(canvas, store, seek) {
    this.canvas = canvas;
    this.store = store;
    this.seek = seek;
    new ResizeObserver(() => this.draw()).observe(canvas);
    store.addEventListener("time", () => this.draw());
    store.addEventListener("records", () => this.draw());
    canvas.addEventListener("click", event => {
      if (!store.records.length) return;
      const rect = canvas.getBoundingClientRect();
      const ratio = (event.clientX - rect.left) / rect.width;
      const start = store.records[0].timestamp_s, end = store.records.at(-1).timestamp_s;
      const target = start + ratio * (end - start);
      const closest = [...store.faultEvents, ...store.stateEvents].sort((a, b) => Math.abs(a.timestamp_s - target) - Math.abs(b.timestamp_s - target))[0];
      this.seek(closest?.timestamp_s ?? target);
    });
  }

  draw() {
    const {context: ctx, width, height} = fitCanvas(this.canvas);
    ctx.fillStyle = "#090e13"; ctx.fillRect(0, 0, width, height);
    if (this.store.records.length < 2) return;
    const start = this.store.records[0].timestamp_s, end = this.store.records.at(-1).timestamp_s;
    const x = time => (time - start) / Math.max(1e-9, end - start) * width;
    ctx.fillStyle = "#1b2832"; ctx.fillRect(0, height / 2 - 2, width, 4);
    for (const event of this.store.stateEvents) {
      ctx.fillStyle = ["EMERGENCY_STOP", "FAULT_LOCKOUT"].includes(event.state) ? "#ef5d68" :
        event.state === "CONTROLLED_STOP" ? "#e6ae45" : "#4f8df7";
      ctx.fillRect(x(event.timestamp_s) - 1, 3, 2, height - 6);
    }
    for (const event of this.store.faultEvents) {
      ctx.fillStyle = "#ff3347";
      ctx.beginPath(); ctx.moveTo(x(event.timestamp_s), 3); ctx.lineTo(x(event.timestamp_s) - 4, 10); ctx.lineTo(x(event.timestamp_s) + 4, 10); ctx.closePath(); ctx.fill();
    }
    const current = this.store.current?.timestamp_s;
    if (Number.isFinite(current)) {
      ctx.strokeStyle = "#f0f4f6"; ctx.beginPath(); ctx.moveTo(x(current), 0); ctx.lineTo(x(current), height); ctx.stroke();
    }
  }
}
