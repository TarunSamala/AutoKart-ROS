import { clonePreset } from './course';
import { controllerDecision, emptyController, runController } from './controller';
import { emptyAvoidance, KART_HALF_WIDTH, planObstacleAvoidance } from './avoidance';
import {
  calculateMidpoints,
  extractEdges,
  filterTriangles,
  interpolateMidpoints,
  orderMidpoints,
  triangulate,
} from './planner';
import { calculateObstacleDetection, emptySensor, runSensor } from './sensor';
import { integrateBicycle, MAX_BRAKING, PHYSICS_DT, stationaryMovement } from './vehicle';
import type {
  Cone,
  ControllerSnapshot,
  CoursePreset,
  DecisionTrace,
  Edge,
  EditTool,
  HistorySample,
  LoopMetric,
  LoopMetrics,
  LoopName,
  Midpoint,
  Obstacle,
  OrderedMidpoint,
  OrderingCheck,
  PathPoint,
  PlanningSnapshot,
  PlanningTiming,
  Point2D,
  SensorSnapshot,
  Triangle,
  VehicleState,
  AvoidanceSide,
  AvoidanceSnapshot,
} from './types';

const TARGETS: Record<LoopName, number> = {
  physics: 60,
  controller: 50,
  sensor: 20,
  planner: 15,
  ui: 10,
  render: 60,
};

const now = () => performance.now();

function emptyTiming(): PlanningTiming {
  return {
    sensorMs: 0,
    delaunayMs: 0,
    filterMs: 0,
    edgeMs: 0,
    midpointMs: 0,
    orderMs: 0,
    splineMs: 0,
    avoidanceMs: 0,
    controllerMs: 0,
    physicsMs: 0,
    renderMs: 0,
    totalPlanMs: 0,
  };
}

function emptyLoops(): LoopMetrics {
  return Object.fromEntries(
    (Object.keys(TARGETS) as LoopName[]).map((name) => [
      name,
      { targetHz: TARGETS[name], measuredHz: 0, executionMs: 0, cycles: 0 } satisfies LoopMetric,
    ]),
  ) as LoopMetrics;
}

export class AutonomyEngine {
  private presetId = 'oval';
  private cones: Cone[] = [];
  private obstacles: Obstacle[] = [];
  private vehicle!: VehicleState;
  private trail: Point2D[] = [];
  private sensor: SensorSnapshot = emptySensor();
  private delaunayPoints: Cone[] = [];
  private triangles: Triangle[] = [];
  private connectivity: [number, number, number][] = [];
  private corridorPolygon: Point2D[] = [];
  private interiorTriangles: Triangle[] = [];
  private exteriorTriangles: Triangle[] = [];
  private flags: boolean[] = [];
  private uniqueEdges: Edge[] = [];
  private crossTrackEdges: Edge[] = [];
  private midpoints: Midpoint[] = [];
  private orderedMidpoints: OrderedMidpoint[] = [];
  private orderingChecks: OrderingCheck[] = [];
  private cumulativeDistance: number[] = [];
  private segmentDistances: number[] = [];
  private basePath: PathPoint[] = [];
  private path: PathPoint[] = [];
  private avoidance: AvoidanceSnapshot = emptyAvoidance();
  private preferredAvoidanceSide: AvoidanceSide | null = null;
  private controller: ControllerSnapshot = emptyController();
  private movement = stationaryMovement({
    x: 0, y: 0, heading: 0, velocity: 0, acceleration: 0, steeringAngle: 0, yawRate: 0, speedCommand: 0,
  });
  private timing = emptyTiming();
  private loops = emptyLoops();
  private loopEvents: Record<LoopName, number[]> = {
    physics: [], controller: [], sensor: [], planner: [], ui: [], render: [],
  };
  private history: HistorySample[] = [];
  private running = false;
  private speed = 1;
  private simulationTime = 0;
  private frame = 0;
  private planningCycle = 0;
  private controllerCycle = 0;
  private editSequence = 0;
  private uiRevision = 0;
  private lastFrameWallTime: number | null = null;
  private nextPhysics = PHYSICS_DT;
  private nextController = 1 / TARGETS.controller;
  private nextSensor = 1 / TARGETS.sensor;
  private nextPlanner = 1 / TARGETS.planner;
  private lastUiWallTime: number | null = null;

  constructor(presetId = 'oval') {
    this.loadPreset(presetId);
  }

  private measure<T>(work: () => T): [T, number] {
    const start = now();
    const result = work();
    return [result, now() - start];
  }

  private markLoop(name: LoopName, wallTime: number, executionMs: number): void {
    const metric = this.loops[name];
    metric.cycles += 1;
    metric.executionMs = executionMs;
    const events = this.loopEvents[name];
    events.push(wallTime);
    while (events.length > 2 && events[0] < wallTime - 1500) events.shift();
    if (events.length > 1) {
      const span = events[events.length - 1] - events[0];
      if (span > 0) metric.measuredHz = ((events.length - 1) * 1000) / span;
    }
  }

  private runSensorCycle(wallTime: number): void {
    const [result, elapsed] = this.measure(() => runSensor(this.cones, this.obstacles, this.vehicle));
    this.sensor = result;
    this.timing.sensorMs = elapsed;
    this.markLoop('sensor', wallTime, elapsed);
  }

  private runPlannerCycle(wallTime: number): void {
    let start = now();
    const delaunay = triangulate(this.sensor.visibleCones);
    this.timing.delaunayMs = now() - start;

    start = now();
    const filtering = filterTriangles(delaunay.triangles, delaunay.points, this.vehicle);
    this.timing.filterMs = now() - start;

    start = now();
    const edges = extractEdges(filtering.interiorTriangles);
    this.timing.edgeMs = now() - start;

    start = now();
    const midpoints = calculateMidpoints(edges.crossTrackEdges);
    this.timing.midpointMs = now() - start;

    start = now();
    const ordering = orderMidpoints(midpoints, this.vehicle, edges.crossTrackEdges);
    this.timing.orderMs = now() - start;

    start = now();
    const interpolation = interpolateMidpoints(ordering.ordered);
    this.timing.splineMs = now() - start;

    start = now();
    let retainedObstacle = null;
    const previouslyTracked = this.avoidance.active ? this.avoidance.obstacle?.obstacle : null;
    if (previouslyTracked && !this.sensor.visibleObstacles.some((item) => item.obstacle.id === previouslyTracked.id)) {
      const tracked = calculateObstacleDetection(previouslyTracked, this.vehicle, this.sensor.range + 3, Math.PI * 2);
      if (tracked.longitudinal > -1.2) retainedObstacle = { ...tracked, detected: false };
    }
    let avoidance = planObstacleAvoidance(
      interpolation.path,
      this.sensor,
      midpoints,
      edges.crossTrackEdges,
      this.preferredAvoidanceSide,
      retainedObstacle,
    );
    if (avoidance.active && avoidance.obstacle) {
      const selectedCandidate = avoidance.candidates.find((candidate) => candidate.side === avoidance.selectedSide);
      const currentClearance =
        Math.hypot(this.vehicle.x - avoidance.obstacle.obstacle.x, this.vehicle.y - avoidance.obstacle.obstacle.y) -
        avoidance.obstacle.obstacle.radius -
        KART_HALF_WIDTH;
      const brakingDistance = (this.vehicle.velocity * this.vehicle.velocity) / (2 * MAX_BRAKING);
      let phase = avoidance.phase;
      if (!selectedCandidate?.feasible || currentClearance < 0.02) phase = 'BLOCKED';
      else phase = 'MANEUVERING';
      avoidance = { ...avoidance, phase, currentClearance, brakingDistance };
    }
    this.timing.avoidanceMs = now() - start;
    this.preferredAvoidanceSide = avoidance.active ? avoidance.selectedSide : null;
    this.timing.totalPlanMs =
      this.timing.delaunayMs +
      this.timing.filterMs +
      this.timing.edgeMs +
      this.timing.midpointMs +
      this.timing.orderMs +
      this.timing.splineMs +
      this.timing.avoidanceMs;

    this.triangles = filtering.triangles;
    this.delaunayPoints = delaunay.points;
    this.connectivity = delaunay.connectivity;
    this.corridorPolygon = filtering.corridorPolygon;
    this.interiorTriangles = filtering.interiorTriangles;
    this.exteriorTriangles = filtering.exteriorTriangles;
    this.flags = filtering.flags;
    this.uniqueEdges = edges.uniqueEdges;
    this.crossTrackEdges = edges.crossTrackEdges;
    this.midpoints = midpoints;
    this.orderedMidpoints = ordering.ordered;
    this.orderingChecks = ordering.checks;
    this.cumulativeDistance = interpolation.cumulativeDistance;
    this.segmentDistances = interpolation.segmentDistances;
    this.basePath = interpolation.path;
    this.avoidance = avoidance;
    this.path = avoidance.selectedPath;
    this.planningCycle += 1;
    this.markLoop('planner', wallTime, this.timing.totalPlanMs);
  }

  private runControllerCycle(wallTime: number): void {
    const [controller, elapsed] = this.measure(() => runController(this.path, this.sensor, this.vehicle, this.avoidance));
    this.controller = controller;
    this.vehicle = {
      ...this.vehicle,
      steeringAngle: controller.steeringAngle,
      speedCommand: controller.speedCommand,
    };
    this.timing.controllerMs = elapsed;
    this.controllerCycle += 1;
    this.markLoop('controller', wallTime, elapsed);
  }

  private runPhysicsCycle(wallTime: number): void {
    const [movement, elapsed] = this.measure(() => integrateBicycle(this.vehicle, PHYSICS_DT));
    this.movement = movement;
    this.vehicle = movement.newState;
    this.timing.physicsMs = elapsed;
    const lastTrail = this.trail[this.trail.length - 1];
    if (!lastTrail || Math.hypot(lastTrail.x - this.vehicle.x, lastTrail.y - this.vehicle.y) > 0.08) {
      this.trail.push({ x: this.vehicle.x, y: this.vehicle.y });
      if (this.trail.length > 1200) this.trail.shift();
    }
    this.markLoop('physics', wallTime, elapsed);
  }

  private runUiCycle(wallTime: number): void {
    const start = now();
    const historySample: HistorySample = {
      time: this.simulationTime,
      steeringAngle: this.vehicle.steeringAngle,
      crossTrackError: this.controller.crossTrackError,
      velocity: this.vehicle.velocity,
      visibleCones: this.sensor.visibleCones.length,
      plannerMs: this.timing.totalPlanMs,
    };
    if (this.history.at(-1)?.time === this.simulationTime) this.history[this.history.length - 1] = historySample;
    else this.history.push(historySample);
    const cutoff = this.simulationTime - 10;
    while (this.history.length > 1 && this.history[0].time < cutoff) this.history.shift();
    this.uiRevision += 1;
    this.markLoop('ui', wallTime, now() - start);
  }

  private runUntil(targetTime: number, wallTime: number): void {
    const epsilon = 1e-9;
    while (true) {
      const next = Math.min(this.nextPhysics, this.nextController, this.nextSensor, this.nextPlanner);
      if (next > targetTime + epsilon) break;
      this.simulationTime = next;
      if (this.nextSensor <= next + epsilon) {
        this.runSensorCycle(wallTime);
        this.nextSensor += 1 / TARGETS.sensor;
      }
      if (this.nextPlanner <= next + epsilon) {
        this.runPlannerCycle(wallTime);
        this.nextPlanner += 1 / TARGETS.planner;
      }
      if (this.nextController <= next + epsilon) {
        this.runControllerCycle(wallTime);
        this.nextController += 1 / TARGETS.controller;
      }
      if (this.nextPhysics <= next + epsilon) {
        this.runPhysicsCycle(wallTime);
        this.nextPhysics += PHYSICS_DT;
      }
    }
    this.simulationTime = targetTime;
  }

  advanceFrame(wallTime = now()): void {
    this.frame += 1;
    if (this.lastFrameWallTime === null) this.lastFrameWallTime = wallTime;
    const realDelta = Math.min(0.1, Math.max(0, (wallTime - this.lastFrameWallTime) / 1000));
    this.lastFrameWallTime = wallTime;
    if (this.running && realDelta > 0) this.runUntil(this.simulationTime + realDelta * this.speed, wallTime);
    if (this.lastUiWallTime === null || wallTime - this.lastUiWallTime >= 1000 / TARGETS.ui) {
      this.runUiCycle(wallTime);
      this.lastUiWallTime = wallTime;
    }
  }

  recordRenderCycle(wallTime: number, executionMs: number): void {
    this.timing.renderMs = executionMs;
    this.markLoop('render', wallTime, executionMs);
  }

  loadPreset(id: string): void {
    const preset: CoursePreset = clonePreset(id);
    this.presetId = preset.id;
    this.cones = preset.cones;
    this.obstacles = preset.obstacles;
    this.vehicle = { ...preset.start };
    this.trail = [{ x: this.vehicle.x, y: this.vehicle.y }];
    this.running = false;
    this.speed = 1;
    this.simulationTime = 0;
    this.frame = 0;
    this.planningCycle = 0;
    this.controllerCycle = 0;
    this.editSequence = 0;
    this.uiRevision = 0;
    this.lastFrameWallTime = null;
    this.nextPhysics = PHYSICS_DT;
    this.nextController = 1 / TARGETS.controller;
    this.nextSensor = 1 / TARGETS.sensor;
    this.nextPlanner = 1 / TARGETS.planner;
    this.lastUiWallTime = null;
    this.timing = emptyTiming();
    this.loops = emptyLoops();
    this.loopEvents = { physics: [], controller: [], sensor: [], planner: [], ui: [], render: [] };
    this.history = [];
    this.avoidance = emptyAvoidance();
    this.basePath = [];
    this.path = [];
    this.controller = emptyController(this.vehicle);
    this.preferredAvoidanceSide = null;
    const wallTime = now();
    this.runSensorCycle(wallTime);
    this.runPlannerCycle(wallTime);
    this.runControllerCycle(wallTime);
    this.movement = stationaryMovement(this.vehicle);
    this.runUiCycle(wallTime);
  }

  reset(): void {
    this.loadPreset(this.presetId);
  }

  setRunning(running: boolean): void {
    this.running = running;
    this.lastFrameWallTime = null;
    this.uiRevision += 1;
  }

  isRunning(): boolean {
    return this.running;
  }

  setSimulationSpeed(speed: number): void {
    this.speed = speed;
    this.uiRevision += 1;
  }

  stepPlanningCycle(): void {
    if (this.running) return;
    const wallTime = now();
    this.runSensorCycle(wallTime);
    this.runPlannerCycle(wallTime);
    this.runControllerCycle(wallTime);
    this.runUiCycle(wallTime);
  }

  stepTime(dt = 0.02): void {
    if (this.running) return;
    const wallTime = now();
    this.runUntil(this.simulationTime + dt, wallTime);
    this.runUiCycle(wallTime);
  }

  editWorld(tool: EditTool, point: Point2D): void {
    if (tool === 'inspect') return;
    if (tool === 'erase') {
      const nearestCone = this.cones
        .map((item) => ({ item, d: Math.hypot(item.x - point.x, item.y - point.y) }))
        .sort((a, b) => a.d - b.d)[0];
      const nearestObstacle = this.obstacles
        .map((item) => ({ item, d: Math.hypot(item.x - point.x, item.y - point.y) }))
        .sort((a, b) => a.d - b.d)[0];
      if (nearestCone && nearestCone.d < 0.8 && (!nearestObstacle || nearestCone.d <= nearestObstacle.d)) {
        this.cones = this.cones.filter((item) => item.id !== nearestCone.item.id);
      } else if (nearestObstacle && nearestObstacle.d < 1) {
        this.obstacles = this.obstacles.filter((item) => item.id !== nearestObstacle.item.id);
      }
    } else if (tool === 'obstacle') {
      this.editSequence += 1;
      this.obstacles.push({ id: `OU${this.editSequence}`, ...point, radius: 0.45 });
    } else {
      const side = tool === 'blue' ? 'BLUE' : 'YELLOW';
      this.editSequence += 1;
      this.cones.push({ id: `${side[0]}U${this.editSequence}`, side, ...point });
    }
    const wallTime = now();
    this.runSensorCycle(wallTime);
    this.runPlannerCycle(wallTime);
    this.runControllerCycle(wallTime);
    this.runUiCycle(wallTime);
  }

  getUiRevision(): number {
    return this.uiRevision;
  }

  private buildTrace(): DecisionTrace {
    return {
      decision: controllerDecision(this.controller),
      sensorCount: this.sensor.visibleCones.length,
      triangleCount: this.triangles.length,
      interiorCount: this.interiorTriangles.length,
      rejectedCount: this.exteriorTriangles.length,
      crossTrackCount: this.crossTrackEdges.length,
      midpointCount: this.midpoints.length,
      pathCount: this.path.length,
      target: this.controller.target,
      alpha: this.controller.alpha,
      curvature: this.controller.curvature,
      steeringAngle: this.controller.steeringAngle,
      yawRate: this.movement.yawRate,
      newPose: this.movement.newState,
    };
  }

  getSnapshot(): PlanningSnapshot {
    return {
      timestamp: now(),
      simulationTime: this.simulationTime,
      frame: this.frame,
      cycle: this.planningCycle,
      controllerCycle: this.controllerCycle,
      running: this.running,
      simulationSpeed: this.speed,
      vehicle: this.vehicle,
      world: { cones: this.cones, obstacles: this.obstacles, trail: this.trail },
      sensor: this.sensor,
      delaunay: { points: this.delaunayPoints, triangles: this.triangles, connectivity: this.connectivity },
      filtering: {
        interiorTriangles: this.interiorTriangles,
        exteriorTriangles: this.exteriorTriangles,
        flags: this.flags,
        corridorPolygon: this.corridorPolygon,
      },
      edges: { uniqueEdges: this.uniqueEdges, crossTrackEdges: this.crossTrackEdges },
      midpoints: this.midpoints,
      orderedMidpoints: this.orderedMidpoints,
      orderingChecks: this.orderingChecks,
      interpolation: {
        cumulativeDistance: this.cumulativeDistance,
        segmentDistances: this.segmentDistances,
        basePath: this.basePath,
        path: this.path,
      },
      avoidance: this.avoidance,
      controller: this.controller,
      movement: this.movement,
      timing: this.timing,
      loops: this.loops,
      history: this.history,
      trace: this.buildTrace(),
    };
  }
}
