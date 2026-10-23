import type { Cone, Gate, MissionFrame, Point } from './schema';
import { candidatePaths, delaunayFromCones, fitCircle, gabrielFilter, gatesFromGabriel, observeCones, MyTrackMission, MyTrackConeMap, myTrackBodyFramePoints, type MyTrackStepResult } from './labAlgorithms';
import { DEFAULT_LAB_SETTINGS, generateScenario, type LabScenario, type LabSettings, type ScenarioName } from './labScenarios';
import { PROFILE_GEOMETRY, type KartProfile } from './kartModel';

// MY_TRACK rear-axle-referenced geometry -- reuses kartModel.ts's
// PROFILE_GEOMETRY directly (per-profile wheelbase/track) rather than a
// second hand-copied constant, so the two can't drift apart. Round 1
// hardcoded the 'shared' numbers regardless of which profile was asked
// for; the mission now actually drives with the requested profile's
// wheelbase, matching the 3D kart model Shared/Built toggle.
function myTrackGeometryFor(profile: KartProfile) {
  const g = PROFILE_GEOMETRY[profile];
  return { wheelbaseM: g.wheelbaseM, frontExtentM: g.frontExtentM, rearOverhangM: g.rearOverhangM, halfWidthM: g.halfWidthM };
}

const DT = 1 / 60;
const WHEELBASE_M = 1.05;
const TARGET_SPEED_MPS = 2.6;
const MAX_STEER_RAD = 0.58;
const SENSOR_FOV_RAD = 2.55;
const wrap = (angle: number) => Math.atan2(Math.sin(angle), Math.cos(angle));

/** Deterministic algorithm lab. FSDS remains the physics authority. */
export class LabSkidpad {
  private scenario: LabScenario;
  private settings: LabSettings;
  private x = 0;
  private y = 0;
  private yaw = 0;
  private speed = 0;
  private steering = 0;
  private progress = 0;
  private elapsed = 0;
  private steps = 0;
  private trail: Point[] = [];
  private running = false;
  private lastWall: number | null = null;
  private accumulator = 0;
  private targetProgress = -Infinity;
  private targetBranch = 'ENTRY';
  private wRight = 0;
  private wLeft = 0;
  private mission: 'RIGHT_LOOPS' | 'LEFT_LOOPS' | 'EXIT' | 'STOP' = 'RIGHT_LOOPS';
  private brakeActive = false;
  private terminalBrake = false;
  private enteredLoop = false;
  private tracked = new Map<string, { cone: Cone; seen: number }>();
  private myTrack: MyTrackMission | null = null;
  private myTrackConeMap: MyTrackConeMap | null = null;
  private myTrackOdometryM = 0;
  private myTrackLastResult: MyTrackStepResult | null = null;
  private myTrackGeometry = myTrackGeometryFor('shared');

  constructor(name: ScenarioName = 'SKIDPAD', settings: LabSettings = DEFAULT_LAB_SETTINGS, kartProfile: KartProfile = 'shared') {
    this.settings = { ...settings };
    this.scenario = generateScenario(name, settings);
    this.myTrackGeometry = myTrackGeometryFor(kartProfile);
    if (this.scenario.name === 'MY_TRACK') { this.myTrack = new MyTrackMission(this.myTrackGeometry); this.myTrackConeMap = new MyTrackConeMap(); }
    this.reset();
  }

  getScenario(): ScenarioName { return this.scenario.name; }
  getSettings(): LabSettings { return { ...this.settings }; }

  reset(): void {
    const start = this.scenario.start;
    this.x = start.x_m; this.y = start.y_m; this.yaw = start.heading;
    this.speed = 0; this.steering = 0; this.progress = 0; this.elapsed = 0; this.steps = 0;
    this.trail = [{ x_m: this.x, y_m: this.y }];
    this.accumulator = 0; this.lastWall = null; this.running = false;
    this.targetProgress = -Infinity; this.targetBranch = 'ENTRY'; this.wRight = 0; this.wLeft = 0; this.mission = 'RIGHT_LOOPS'; this.brakeActive = false; this.terminalBrake = false; this.enteredLoop = false; this.tracked.clear();
    this.myTrackOdometryM = 0; this.myTrackLastResult = null; this.myTrack?.reset(); this.myTrackConeMap?.reset();
  }

  setRunning(running: boolean): void {
    if (this.running === running) return;
    this.running = running;
    this.lastWall = null;
  }

  isRunning(): boolean { return this.running; }

  advanceFrame(wallTimeMs: number): void {
    if (!this.running) return;
    if (this.lastWall == null) { this.lastWall = wallTimeMs; return; }
    this.accumulator += Math.min(0.1, Math.max(0, (wallTimeMs - this.lastWall) / 1000));
    this.lastWall = wallTimeMs;
    while (this.accumulator + 1e-9 >= DT) { this.step(); this.accumulator -= DT; }
  }

  private observe(): Cone[] {
    const range = this.scenario.name === 'BRAKE' ? 26 : 19;
    return observeCones(this.scenario.cones, { x_m: this.x, y_m: this.y, yaw_rad: this.yaw }, range, SENSOR_FOV_RAD, this.settings.dropout, this.settings.seed + Math.floor(this.steps / 6), this.settings.noise);
  }

  private branch(): string {
    return this.mission === 'RIGHT_LOOPS' && !this.enteredLoop ? 'ENTRY' : this.mission;
  }

  private routeProgress(target: Point): number {
    const branch = this.branch();
    if (branch === 'ENTRY' || branch === 'EXIT') return target.y_m;
    if (branch === 'STOP') return -Infinity;
    const right = branch === 'RIGHT_LOOPS';
    const centerX = right ? this.settings.skidpadRadius : -this.settings.skidpadRadius;
    const direction = right ? -1 : 1;
    const current = Math.atan2(this.y, this.x - centerX);
    const angle = Math.atan2(target.y_m, target.x_m - centerX);
    const ahead = (direction * (angle - current) + Math.PI * 4) % (Math.PI * 2);
    const completed = right ? Math.max(0, -this.wRight) : Math.max(0, this.wLeft);
    return completed + ahead / (Math.PI * 2);
  }

  private pipeline(detections: Cone[]) {
    const pose = { x_m: this.x, y_m: this.y, yaw_rad: this.yaw };
    const delaunay = delaunayFromCones(detections);
    const gabriel = gabrielFilter(delaunay.edges, detections);
    const gates = gatesFromGabriel(gabriel, detections, pose);
    const missionGates = this.scenario.name === 'SKIDPAD' ? gates.map((gate) => {
      if (!gate.valid) return gate;
      const x = gate.midpoint.x_m, y = gate.midpoint.y_m, r = this.settings.skidpadRadius;
      const onEntry = Math.abs(x) < this.settings.width && y < -this.settings.junctionLength;
      // Only observed gate midpoints in the central exit corridor may seed
      // the EXIT segment. Side-of-loop Gabriel edges can also have positive Y.
      const onExit = Math.abs(x) < this.settings.width * 0.25 && y > this.settings.junctionLength;
      const onRight = x > this.settings.junctionLength * 0.6 && Math.abs(Math.hypot(x - r, y) - r) < this.settings.width;
      const onLeft = x < -this.settings.junctionLength * 0.6 && Math.abs(Math.hypot(x + r, y) - r) < this.settings.width;
      const allowed = this.mission === 'RIGHT_LOOPS' ? this.enteredLoop ? onRight : onEntry : this.mission === 'LEFT_LOOPS' ? onLeft : onExit;
      return allowed ? gate : { ...gate, valid: false, reason: `MISSION REJECTS ${this.mission}` };
    }) : gates;
    const onEntry = this.scenario.name === 'SKIDPAD' && this.mission === 'RIGHT_LOOPS' && !this.enteredLoop;
    const loop = this.scenario.name !== 'SKIDPAD' || onEntry || this.mission === 'EXIT' || this.mission === 'STOP' ? undefined : this.mission === 'RIGHT_LOOPS' ? { center: { x_m: this.settings.skidpadRadius, y_m: 0 }, direction: -1 as const, radius: this.settings.skidpadRadius } : { center: { x_m: -this.settings.skidpadRadius, y_m: 0 }, direction: 1 as const, radius: this.settings.skidpadRadius };
    const ranked = this.mission === 'STOP' && this.scenario.name === 'SKIDPAD' ? [] : candidatePaths(missionGates, pose, undefined, loop);
    const cursor = this.targetBranch === this.branch() ? this.targetProgress : -Infinity;
    const candidates = this.scenario.name === 'SKIDPAD' ? ranked.filter((candidate) =>
      this.routeProgress(candidate.points[candidate.points.length - 1]) >= cursor - 0.02)
      .map((candidate, index) => ({ ...candidate, selected: index === 0, rank: index + 1 })) : ranked;
    return { delaunay, gabriel, gates: missionGates, candidates, selected: candidates[0] };
  }

  private obstacleMetric() {
    const obstacle = this.scenario.obstacle;
    if (!obstacle) return null;
    const centerDistance = Math.hypot(obstacle.x_m - this.x, obstacle.y_m - this.y);
    if (centerDistance > 26 || Math.abs(wrap(Math.atan2(obstacle.y_m - this.y, obstacle.x_m - this.x) - this.yaw)) > 1.28) return null;
    const distance = Math.max(0, centerDistance - obstacle.radius_m);
    const reaction = this.speed * 0.25, braking = this.speed ** 2 / (2 * 3.2), margin = 1.5;
    const required = reaction + braking + margin;
    return { obstacle, distance, required, reaction, braking, margin, ttc: this.speed > 0.05 ? distance / this.speed : null, trigger: distance <= required || this.brakeActive };
  }

  private fittedLoop(side: 'LEFT' | 'RIGHT') {
    const r = this.settings.skidpadRadius, centerX = side === 'RIGHT' ? r : -r;
    const observed = [...this.tracked.values()].map((item) => item.cone).filter((item) => side === 'RIGHT' ? item.x_m > r * 0.5 : item.x_m < -r * 0.5);
    const outerClass = side === 'RIGHT' ? 'BLUE' : 'YELLOW';
    const ring = (item: Cone, radius: number) => Math.abs(Math.hypot(item.x_m - centerX, item.y_m) - radius) < 0.45;
    const outer = fitCircle(observed.filter((item) => item.class === outerClass && ring(item, r + this.settings.width / 2)));
    const inner = fitCircle(observed.filter((item) => item.class !== outerClass && item.class !== 'ORANGE' && ring(item, r - this.settings.width / 2)));
    if (!outer || !inner) return null;
    const center = { x_m: (outer.center.x_m + inner.center.x_m) / 2, y_m: (outer.center.y_m + inner.center.y_m) / 2 };
    if (Math.abs(Math.abs(center.x_m) - r) > this.settings.width * 2 || Math.abs(center.y_m) > this.settings.width * 2) return null;
    return { center, radius_m: (outer.radius_m + inner.radius_m) / 2 };
  }

  private stepMyTrack(): void {
    if (!this.myTrack) return;
    const detections = this.observe();
    const pose = { x_m: this.x, y_m: this.y, yaw_rad: this.yaw };
    // Feed detections into the persistent world-frame cone map, then build
    // gates from the map's CONFIRMED, majority-voted cones -- not from this
    // frame's raw detections. See MyTrackConeMap's doc comment: this is the
    // fix for round 1's lag through the lateral bend-shifts.
    this.myTrackConeMap?.update(detections, this.elapsed);
    const mapped = this.myTrackConeMap?.confirmed() ?? detections;
    const bodyPoints = myTrackBodyFramePoints(mapped, pose);
    const result = this.myTrack.step(bodyPoints, pose, this.speed, this.myTrackOdometryM, this.elapsed);
    this.myTrackLastResult = result;
    this.steering = result.steeringRad;
    const targetSpeed = result.targetSpeedMps;
    this.speed = result.brake || this.speed > targetSpeed ? Math.max(targetSpeed, this.speed - 3.2 * DT) : Math.min(targetSpeed, this.speed + 1.3 * DT);
    const dX = this.speed * Math.cos(this.yaw) * DT, dY = this.speed * Math.sin(this.yaw) * DT;
    this.x += dX; this.y += dY;
    this.yaw = wrap(this.yaw + (this.speed / this.myTrackGeometry.wheelbaseM) * Math.tan(this.steering) * DT);
    this.myTrackOdometryM += Math.hypot(dX, dY);
    this.elapsed += DT; this.steps += 1;
    if (this.steps % 3 === 0) { this.trail.push({ x_m: this.x, y_m: this.y }); if (this.trail.length > 2400) this.trail.shift(); }
    if (result.state === 'STOPPED') this.running = false;
  }

  private step(): void {
    if (this.scenario.name === 'MY_TRACK') { this.stepMyTrack(); return; }
    if (this.scenario.name === 'SKIDPAD' && this.mission === 'STOP') { this.running = false; return; }
    const plan = this.pipeline(this.observe());
    const target = plan.selected?.points.at(-1) ?? null;
    if (target && this.scenario.name === 'SKIDPAD') {
      const branch = this.branch();
      if (this.targetBranch !== branch) { this.targetBranch = branch; this.targetProgress = -Infinity; }
      this.targetProgress = Math.max(this.targetProgress, this.routeProgress(target));
    }
    if (target) {
      const angle = wrap(Math.atan2(target.y_m - this.y, target.x_m - this.x) - this.yaw);
      const distance = Math.hypot(target.x_m - this.x, target.y_m - this.y);
      this.steering = Math.max(-MAX_STEER_RAD, Math.min(MAX_STEER_RAD, Math.atan2(2 * WHEELBASE_M * Math.sin(angle), Math.max(2.5, distance))));
    }
    if (this.scenario.name === 'SKIDPAD' && !this.enteredLoop && this.y > -2) {
      this.enteredLoop = true; this.targetProgress = -Infinity; this.targetBranch = this.branch();
    }
    if (this.scenario.name === 'SKIDPAD' && this.enteredLoop && this.mission !== 'EXIT' && this.mission !== 'STOP') {
      const centerX = this.mission === 'RIGHT_LOOPS' ? this.settings.skidpadRadius : -this.settings.skidpadRadius;
      const direction = this.mission === 'RIGHT_LOOPS' ? -1 : 1;
      const radius = Math.hypot(this.x - centerX, this.y);
      const bearing = Math.atan2(this.y, this.x - centerX);
      const desired = bearing + direction * Math.PI / 2 + direction * Math.atan((radius - this.settings.skidpadRadius) * 0.35);
      const correction = Math.max(-MAX_STEER_RAD, Math.min(MAX_STEER_RAD, wrap(desired - this.yaw) * 0.65));
      this.steering = target ? this.steering * 0.6 + correction * 0.4 : correction;
    }
    if (this.scenario.name === 'SKIDPAD' && this.mission === 'EXIT') {
      if (!target) this.steering = Math.max(-MAX_STEER_RAD, Math.min(MAX_STEER_RAD, wrap(Math.PI / 2 - this.yaw) * 0.65));
      const stopY = this.settings.exitLength - 1;
      if (this.y >= stopY - this.speed ** 2 / (2 * 3.2) - 0.15) this.terminalBrake = true;
    }
    if (this.obstacleMetric()?.trigger) this.brakeActive = true;
    const targetSpeed = this.scenario.name === 'BRAKE' ? 4.2 : TARGET_SPEED_MPS;
    this.speed = this.brakeActive || this.terminalBrake ? Math.max(0, this.speed - 3.2 * DT) : Math.min(targetSpeed, this.speed + 1.3 * DT);
    const oldX = this.x, oldY = this.y;
    this.x += this.speed * Math.cos(this.yaw) * DT;
    this.y += this.speed * Math.sin(this.yaw) * DT;
    this.yaw = wrap(this.yaw + this.speed / WHEELBASE_M * Math.tan(this.steering) * DT);
    this.elapsed += DT; this.steps += 1;
    if (this.steps % 3 === 0) { this.trail.push({ x_m: this.x, y_m: this.y }); if (this.trail.length > 2400) this.trail.shift(); }
    if (this.scenario.name === 'SKIDPAD') {
      const centerX = this.mission === 'RIGHT_LOOPS' ? this.settings.skidpadRadius : -this.settings.skidpadRadius;
      const before = Math.atan2(oldY, oldX - centerX), after = Math.atan2(this.y, this.x - centerX);
      const radius = Math.hypot(this.x - centerX, this.y);
      if (radius > this.settings.skidpadRadius - this.settings.width && radius < this.settings.skidpadRadius + this.settings.width) {
        if (this.mission === 'RIGHT_LOOPS') this.wRight += wrap(after - before) / (2 * Math.PI);
        if (this.mission === 'LEFT_LOOPS') this.wLeft += wrap(after - before) / (2 * Math.PI);
      }
      const junction = Math.hypot(this.x, this.y) < 2.5;
      if (junction && this.mission === 'RIGHT_LOOPS' && this.wRight <= -1.9) { this.mission = 'LEFT_LOOPS'; this.targetProgress = -Infinity; this.targetBranch = this.branch(); }
      else if (junction && this.mission === 'LEFT_LOOPS' && this.wLeft >= 1.9) { this.mission = 'EXIT'; this.targetProgress = -Infinity; this.targetBranch = this.branch(); }
      else if (this.mission === 'EXIT' && this.terminalBrake && this.speed <= 0.1 && this.y >= this.settings.exitLength - 1.6) {
        this.mission = 'STOP'; this.speed = 0; this.steering = 0; this.running = false;
      }
    }
  }

  private getMyTrackFrame(): MissionFrame {
    const result = this.myTrackLastResult;
    const vehicle = {
      x_m: this.x, y_m: this.y, yaw_rad: this.yaw, speed_mps: this.speed, measured_steering_rad: this.steering,
      requested_steering_rad: this.steps === 0 ? null : this.steering,
      target_speed_mps: this.steps === 0 ? null : result?.targetSpeedMps ?? null,
      brake_command: this.steps === 0 ? null : result?.brake ? 1 : 0,
      brake_feedback: result?.brake ? 1 : 0,
    };
    const detections = this.steps === 0 ? [] : this.observe();
    for (const cone of detections) this.tracked.set(cone.id, { cone, seen: this.elapsed });
    const tracked = [...this.tracked.values()].filter((item) => this.elapsed - item.seen < 0.8).map((item) => ({ ...item.cone, track_age_s: this.elapsed - item.seen }));
    const gates: Gate[] = (result?.gatesList ?? []).map((g, index) => ({ id: `MT${index + 1}`, midpoint: g.midpoint, width_m: g.widthM, valid: true, reason: 'MY_TRACK GATE' }));
    const selectedPath = result?.pathWorldXY ?? [];
    return {
      schema_version: 1, source: 'LAB', run_id: `lab-my_track-${this.settings.seed}`, scenario: 'MY_TRACK', sequence: this.steps, timestamp_s: this.elapsed,
      vehicle,
      perception: { cones: detections, tracked_cones: tracked, ground_truth_cones: this.scenario.cones, sensor_range_m: this.steps === 0 ? undefined : 19, sensor_fov_rad: SENSOR_FOV_RAD, obstacles: [] },
      visualization: { course_cones: this.scenario.cones, trail: this.trail.slice(), special_markers: this.scenario.markers },
      planning: { delaunay_edges: [], gabriel_edges: [], gates, candidate_paths: [], selected_path: selectedPath, lookahead: selectedPath.at(-1) ?? null,
        skidpad: { state: result?.state ?? this.myTrack?.state ?? 'READY' } },
      safety: { state: result?.state === 'STOPPED' ? 'STOPPED' : result?.detour ? 'DETOUR' : 'SANDBOX ONLY', priority: 'LAB_AUTONOMY' },
      metrics: { cross_track_error_m: result?.lateralDeviationM ?? null },
      lateralDeviationM: result?.lateralDeviationM ?? undefined,
    };
  }

  getFrame(): MissionFrame {
    if (this.scenario.name === 'MY_TRACK') return this.getMyTrackFrame();
    const vehicle = { x_m: this.x, y_m: this.y, yaw_rad: this.yaw, speed_mps: this.speed, measured_steering_rad: this.steering, requested_steering_rad: this.steps === 0 ? null : this.steering, target_speed_mps: this.steps === 0 ? null : this.terminalBrake ? 0 : this.scenario.name === 'BRAKE' ? 4.2 : TARGET_SPEED_MPS, brake_command: this.steps === 0 ? null : this.brakeActive || this.terminalBrake ? 1 : 0, brake_feedback: this.brakeActive || this.terminalBrake ? 1 : 0 };
    const detections = this.steps === 0 ? [] : this.observe();
    for (const cone of detections) this.tracked.set(cone.id, { cone, seen: this.elapsed });
    const tracked = [...this.tracked.values()].filter((item) => this.elapsed - item.seen < 0.8).map((item) => ({ ...item.cone, track_age_s: this.elapsed - item.seen }));
    const plan = this.pipeline(detections);
    const stop = this.obstacleMetric();
    const selected = plan.selected;
    const branch = this.branch() === 'RIGHT_LOOPS' ? 'RIGHT' : this.branch() === 'LEFT_LOOPS' ? 'LEFT' : this.branch();
    const nearJunction = Math.hypot(this.x, this.y) < 6;
    const branches = this.scenario.name === 'SKIDPAD' && nearJunction ? (['LEFT', 'RIGHT', 'EXIT'] as const).flatMap((name) => {
      const eligible = plan.gates.filter((gate) => gate.valid || gate.reason?.startsWith('MISSION REJECTS'));
      const target = eligible.filter((gate) => name === 'LEFT' ? gate.midpoint.x_m < -this.settings.junctionLength * 0.6 : name === 'RIGHT' ? gate.midpoint.x_m > this.settings.junctionLength * 0.6 : Math.abs(gate.midpoint.x_m) < this.settings.width && gate.midpoint.y_m > this.settings.junctionLength).sort((a, b) => Math.hypot(a.midpoint.x_m - this.x, a.midpoint.y_m - this.y) - Math.hypot(b.midpoint.x_m - this.x, b.midpoint.y_m - this.y))[0];
      return target ? [{ name, points: [{ x_m: this.x, y_m: this.y }, target.midpoint], valid: name === branch, selected: name === branch, reason: name === branch ? `WINDING TARGET · ${target.id}` : `${branch} REQUIRED · ${target.id}` }] : [];
    }) : undefined;
    const leftFit = this.fittedLoop('LEFT'), rightFit = this.fittedLoop('RIGHT');
    return {
      schema_version: 1, source: 'LAB', run_id: `lab-${this.scenario.name.toLowerCase()}-${this.settings.seed}`, scenario: this.scenario.name, sequence: this.steps, timestamp_s: this.elapsed,
      vehicle,
      perception: { cones: detections, tracked_cones: tracked, ground_truth_cones: this.scenario.cones, sensor_range_m: this.steps === 0 ? undefined : this.scenario.name === 'BRAKE' ? 26 : 19, sensor_fov_rad: SENSOR_FOV_RAD, obstacles: stop ? [{ ...stop.obstacle, distance_m: stop.distance, relative_speed_mps: this.speed }] : [] },
      visualization: { course_cones: this.scenario.cones, trail: this.trail.slice(), special_markers: this.scenario.markers },
      planning: { delaunay_edges: plan.delaunay.edges, delaunay_triangles: plan.delaunay.triangles, gabriel_edges: plan.gabriel, gates: plan.gates, candidate_paths: plan.candidates, selected_path: selected?.points ?? [], lookahead: selected?.points.at(-1) ?? null,
        skidpad: this.scenario.name === 'SKIDPAD' && this.steps > 0 ? { left_center: leftFit?.center, right_center: rightFit?.center, left_radius_m: leftFit?.radius_m, right_radius_m: rightFit?.radius_m, w_left: this.wLeft, w_right: this.wRight, target_winding: 2, state: this.mission, branch, branch_locked: nearJunction, progress_index: Number.isFinite(this.targetProgress) ? this.targetProgress : undefined, progress_unit: branch === 'LEFT' || branch === 'RIGHT' ? 'turns' : 'm along route', reason: `${branch} REQUIRED UNTIL WINDING TARGET`, branches } : undefined },
      safety: { state: this.brakeActive ? 'BRAKE OVERRIDE' : 'SANDBOX ONLY', priority: this.brakeActive ? 'OBSTACLE_BRAKE' : 'LAB_AUTONOMY', obstacle: stop ? { id: stop.obstacle.id, distance_m: stop.distance, relative_speed_mps: this.speed, ttc_s: stop.ttc, reaction_distance_m: stop.reaction, braking_distance_m: stop.braking, safety_margin_m: stop.margin, stopping_distance_m: stop.required, available_distance_m: stop.distance, margin_m: stop.distance - stop.required, brake_trigger: stop.trigger, predicted_stop: { x_m: this.x + Math.cos(this.yaw) * (stop.reaction + stop.braking), y_m: this.y + Math.sin(this.yaw) * (stop.reaction + stop.braking) } } : undefined },
      metrics: { cross_track_error_m: selected?.points.at(-1) ? Math.hypot(this.x - selected.points.at(-1)!.x_m, this.y - selected.points.at(-1)!.y_m) : null },
    };
  }
}
