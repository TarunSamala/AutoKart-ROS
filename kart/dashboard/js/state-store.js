const TAU = Math.PI * 2;

function lerp(a, b, t) {
  if (a == null || b == null || !Number.isFinite(a) || !Number.isFinite(b)) return a ?? b ?? null;
  return a + (b - a) * t;
}

function lerpAngle(a, b, t) {
  const delta = ((b - a + Math.PI) % TAU + TAU) % TAU - Math.PI;
  return a + delta * t;
}

function clone(value) {
  return value == null ? value : JSON.parse(JSON.stringify(value));
}

export class DashboardStateStore extends EventTarget {
  constructor(maximumRecords = 20000) {
    super();
    this.maximumRecords = maximumRecords;
    this.records = [];
    this.current = null;
    this.track = null;
    this.kart = null;
    this.manifest = null;
    this.faultEvents = [];
    this.stateEvents = [];
    this.connection = "connecting";
    this.lastReceiveWallMs = performance.now();
    this.measuredTimestamp = null;
    this.displayTimestamp = null;
    this.comparison = [];
  }

  setStaticData({manifest, track, kart}) {
    this.manifest = manifest;
    this.track = track;
    this.kart = kart;
    this.dispatchEvent(new CustomEvent("static", {detail: {manifest, track, kart}}));
  }

  replace(records) {
    this.records = [];
    this.faultEvents = [];
    this.stateEvents = [];
    for (const record of records) this.append(record, false);
    this.current = this.records[0] ?? null;
    this.displayTimestamp = this.current?.timestamp_s ?? 0;
    this.measuredTimestamp = this.displayTimestamp;
    this.dispatchEvent(new CustomEvent("records", {detail: this.records}));
  }

  append(record, notify = true) {
    if (!record || !Number.isFinite(record.timestamp_s)) return false;
    const prior = this.records.at(-1);
    record.stale = Boolean(prior && record.timestamp_s < prior.timestamp_s);
    record.valid = Boolean(record.valid && !record.stale);
    this.records.push(record);
    if (this.records.length > this.maximumRecords) this.records.splice(0, this.records.length - this.maximumRecords);
    const priorFaults = new Set(prior?.vcu?.faults ?? []);
    for (const fault of record.vcu?.faults ?? []) {
      if (!priorFaults.has(fault)) this.faultEvents.push({timestamp_s: record.timestamp_s, fault, record});
    }
    if (record.vcu?.state && record.vcu.state !== prior?.vcu?.state) {
      this.stateEvents.push({timestamp_s: record.timestamp_s, state: record.vcu.state, record});
    }
    this.lastReceiveWallMs = performance.now();
    if (notify) this.dispatchEvent(new CustomEvent("record", {detail: record}));
    return true;
  }

  at(timestampS, interpolate = true) {
    if (!this.records.length) return null;
    if (timestampS <= this.records[0].timestamp_s) return this.records[0];
    if (timestampS >= this.records.at(-1).timestamp_s) return this.records.at(-1);
    let low = 0;
    let high = this.records.length - 1;
    while (low + 1 < high) {
      const mid = (low + high) >> 1;
      if (this.records[mid].timestamp_s <= timestampS) low = mid;
      else high = mid;
    }
    const a = this.records[low];
    const b = this.records[high];
    if (!interpolate || !a.valid || !b.valid || b.timestamp_s <= a.timestamp_s) return a;
    const t = (timestampS - a.timestamp_s) / (b.timestamp_s - a.timestamp_s);
    const output = clone(a);
    output.display_interpolated = true;
    output.measured_timestamp_s = a.timestamp_s;
    output.timestamp_s = timestampS;
    for (const key of ["x_m", "y_m", "z_m", "roll_rad", "pitch_rad"]) {
      output.pose[key] = lerp(a.pose[key], b.pose[key], t);
    }
    output.pose.yaw_rad = lerpAngle(a.pose.yaw_rad, b.pose.yaw_rad, t);
    for (const key of Object.keys(output.motion ?? {})) {
      output.motion[key] = lerp(a.motion?.[key], b.motion?.[key], t);
    }
    output.track.s_m = lerp(a.track?.s_m, b.track?.s_m, t);
    output.track.lateral_m = lerp(a.track?.lateral_m, b.track?.lateral_m, t);
    return output;
  }

  selectTime(timestampS, interpolate = true) {
    this.current = this.at(timestampS, interpolate);
    this.displayTimestamp = timestampS;
    this.measuredTimestamp = this.current?.measured_timestamp_s ?? this.current?.timestamp_s ?? null;
    this.dispatchEvent(new CustomEvent("time", {detail: this.current}));
    return this.current;
  }

  setConnection(state) {
    this.connection = state;
    this.lastReceiveWallMs = performance.now();
    this.dispatchEvent(new CustomEvent("connection", {detail: state}));
  }

  isStale(thresholdMs = 750) {
    return this.connection === "disconnected" ||
      (this.manifest?.mode === "live" && performance.now() - this.lastReceiveWallMs > thresholdMs);
  }

  duration() {
    if (this.records.length < 2) return 0;
    return this.records.at(-1).timestamp_s - this.records[0].timestamp_s;
  }

  distanceTravelled(index = this.records.indexOf(this.current)) {
    let distance = 0;
    const limit = index < 0 ? this.records.length - 1 : index;
    for (let i = 1; i <= limit; i += 1) {
      const a = this.records[i - 1].pose;
      const b = this.records[i].pose;
      distance += Math.hypot(b.x_m - a.x_m, b.y_m - a.y_m, b.z_m - a.z_m);
    }
    return distance;
  }

  currentCorner() {
    const s = this.current?.track?.s_m;
    if (!Number.isFinite(s)) return null;
    return (this.track?.corners ?? []).find(corner => s >= corner.start_s_m && s <= corner.end_s_m) ?? null;
  }
}

export function format(value, digits = 2, fallback = "—") {
  return Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : fallback;
}

export function commandText(command) {
  if (!command) return "not recorded";
  return `D ${format(command.drive, 2)} · B ${format(command.brake, 2)} · δ ${format(command.steering_rad, 3)}`;
}
