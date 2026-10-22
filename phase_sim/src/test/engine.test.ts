import { describe, expect, it } from 'vitest';
import { AutonomyEngine } from '../engine/engine';
import { COURSE_PRESETS } from '../engine/course';
import { calculateConeDetection } from '../engine/sensor';
import { PHYSICS_DT } from '../engine/vehicle';

describe('real-time autonomy engine', () => {
  it('provides a live Delaunay corridor on every track preset', () => {
    for (const preset of COURSE_PRESETS) {
      const engine = new AutonomyEngine(preset.id);
      const snapshot = engine.getSnapshot();
      expect(snapshot.sensor.visibleCones.length, preset.name).toBeGreaterThanOrEqual(4);
      expect(snapshot.filtering.interiorTriangles.length, preset.name).toBeGreaterThan(0);
      expect(snapshot.edges.crossTrackEdges.length, preset.name).toBeGreaterThan(0);
      expect(snapshot.interpolation.path.length, preset.name).toBeGreaterThan(2);
      engine.setRunning(true);
      const start = performance.now();
      for (let frame = 0; frame <= 720; frame += 1) engine.advanceFrame(start + frame * (1000 / 60));
      const driven = engine.getSnapshot();
      expect(driven.world.trail.length, preset.name).toBeGreaterThan(30);
      expect(driven.controller.stopReason, preset.name).toBeNull();
    }
  });

  it('builds every planning stage from the live sensor result', () => {
    const engine = new AutonomyEngine('oval');
    const snapshot = engine.getSnapshot();

    expect(snapshot.sensor.visibleCones.length).toBeGreaterThanOrEqual(4);
    expect(snapshot.delaunay.points).toBe(snapshot.sensor.visibleCones);
    expect(snapshot.delaunay.triangles.length).toBeGreaterThan(0);
    expect(snapshot.filtering.flags).toHaveLength(snapshot.delaunay.triangles.length);
    expect(snapshot.filtering.interiorTriangles.length + snapshot.filtering.exteriorTriangles.length).toBe(snapshot.delaunay.triangles.length);
    expect(snapshot.edges.uniqueEdges.length).toBeGreaterThan(0);
    expect(snapshot.edges.crossTrackEdges.length).toBeGreaterThan(0);
    expect(snapshot.midpoints.length).toBe(snapshot.edges.crossTrackEdges.length);
    expect(snapshot.orderedMidpoints.length).toBeGreaterThan(1);
    expect(snapshot.interpolation.path.length).toBeGreaterThan(snapshot.orderedMidpoints.length);
    expect(snapshot.controller.target).not.toBeNull();

    const shownDetection = snapshot.sensor.detections.find((item) => item.detected)!;
    expect(shownDetection).toEqual(calculateConeDetection(
      shownDetection.cone,
      snapshot.sensor.vehicleAtCalculation,
      snapshot.sensor.range,
      snapshot.sensor.fov,
    ));
  });

  it('moves only through fixed-step bicycle integration at the scheduled multi-rates', () => {
    const engine = new AutonomyEngine('oval');
    const start = engine.getSnapshot().vehicle;
    engine.setRunning(true);
    const wallStart = performance.now();
    for (let frame = 0; frame <= 3600; frame += 1) engine.advanceFrame(wallStart + frame * (1000 / 60));
    const snapshot = engine.getSnapshot();
    const move = snapshot.movement;

    expect(snapshot.simulationTime).toBeCloseTo(60, 4);
    expect(snapshot.loops.physics.cycles).toBeGreaterThanOrEqual(3599);
    expect(snapshot.loops.controller.cycles).toBeGreaterThanOrEqual(2999);
    expect(snapshot.loops.sensor.cycles).toBeGreaterThanOrEqual(1199);
    expect(snapshot.loops.planner.cycles).toBeGreaterThanOrEqual(899);
    expect(snapshot.vehicle.x).not.toBeCloseTo(start.x, 3);
    expect(move.dt).toBe(PHYSICS_DT);
    expect(move.yawRate).toBeCloseTo((move.oldState.velocity / 1.05) * Math.tan(move.oldState.steeringAngle), 12);
    expect(move.deltaHeading).toBeCloseTo(move.yawRate * move.dt, 12);
    expect(move.newState.x).toBeCloseTo(move.oldState.x + move.deltaX, 12);
    expect(move.newState.y).toBeCloseTo(move.oldState.y + move.deltaY, 12);
    expect(snapshot.vehicle.steeringAngle).toBe(snapshot.controller.steeringAngle);
    expect(snapshot.trace.decision).not.toMatch(/^STOP/);
    expect(snapshot.loops.physics.measuredHz).toBeCloseTo(60, 0);
    expect(snapshot.loops.controller.measuredHz).toBeCloseTo(50, 0);
    expect(snapshot.loops.sensor.measuredHz).toBeCloseTo(20, 0);
    expect(snapshot.loops.planner.measuredHz).toBeCloseTo(15, 0);
  });

  it('freezes all simulation state while paused and supports exact inspection steps', () => {
    const engine = new AutonomyEngine('oval');
    const before = engine.getSnapshot();
    engine.advanceFrame(performance.now() + 5000);
    const frozen = engine.getSnapshot();
    expect(frozen.simulationTime).toBe(before.simulationTime);
    expect(frozen.vehicle).toEqual(before.vehicle);
    expect(frozen.cycle).toBe(before.cycle);

    engine.stepPlanningCycle();
    expect(engine.getSnapshot().cycle).toBe(before.cycle + 1);
    engine.stepTime(0.02);
    expect(engine.getSnapshot().simulationTime).toBeCloseTo(0.02, 12);
  });

  it('scores corridor-width routes and maneuvers continuously without overlap', () => {
    const engine = new AutonomyEngine('obstacle');
    const initial = engine.getSnapshot();
    expect(initial.avoidance.active).toBe(true);
    expect(initial.avoidance.candidates).toHaveLength(2);
    expect(initial.avoidance.candidates.find((candidate) => candidate.side === initial.avoidance.selectedSide)?.feasible).toBe(true);
    expect(initial.controller.stopReason).toBeNull();
    expect(initial.avoidance.phase).toBe('MANEUVERING');
    expect(initial.controller.holdReason).toBeNull();
    expect(initial.controller.speedCommand).toBeGreaterThan(0);
    expect(initial.controller.target).toBe(initial.avoidance.selectedPath[initial.controller.targetIndex]);

    engine.setRunning(true);
    const wallStart = performance.now();
    let maneuverSamples = 0;
    for (let frame = 0; frame <= 420; frame += 1) {
      engine.advanceFrame(wallStart + frame * (1000 / 60));
      const live = engine.getSnapshot();
      if (live.avoidance.active) {
        expect(live.controller.stopReason).toBeNull();
        if (live.controller.avoidancePhase === 'MANEUVERING') {
          maneuverSamples += 1;
          expect(live.controller.speedCommand).toBeGreaterThan(0);
        }
      }
    }
    const snapshot = engine.getSnapshot();
    const obstacle = snapshot.world.obstacles[0];
    const minimumCenterDistance = Math.min(...snapshot.world.trail.map((point) => Math.hypot(point.x - obstacle.x, point.y - obstacle.y)));
    expect(minimumCenterDistance - obstacle.radius - 0.36).toBeGreaterThanOrEqual(0.12);
    expect(maneuverSamples).toBeGreaterThan(100);
    expect(snapshot.simulationTime).toBeCloseTo(7, 4);
    expect(snapshot.vehicle.x).toBeGreaterThan(initial.vehicle.x + 2);
  });

  it('transitions a moving kart directly into a safe nonzero-speed maneuver', () => {
    const engine = new AutonomyEngine('oval');
    engine.setRunning(true);
    const wallStart = performance.now();
    for (let frame = 0; frame <= 180; frame += 1) engine.advanceFrame(wallStart + frame * (1000 / 60));
    const moving = engine.getSnapshot();
    expect(moving.vehicle.velocity).toBeGreaterThan(0.7);
    const obstaclePoint = moving.interpolation.path.find((point) => point.s >= (moving.controller.nearest?.s ?? 0) + 3.5)!;
    engine.editWorld('obstacle', obstaclePoint);
    const maneuver = engine.getSnapshot();
    const obstacle = maneuver.world.obstacles.at(-1)!;
    expect(maneuver.avoidance.phase).toBe('MANEUVERING');
    expect(maneuver.controller.speedCommand).toBeGreaterThan(0);
    expect(maneuver.vehicle.velocity).toBeGreaterThan(0.7);

    let minimumManeuverCommand = maneuver.controller.speedCommand;
    let maneuverObserved = false;
    for (let frame = 181; frame <= 780; frame += 1) {
      engine.advanceFrame(wallStart + frame * (1000 / 60));
      const live = engine.getSnapshot();
      if (live.controller.avoidancePhase === 'MANEUVERING') {
        maneuverObserved = true;
        minimumManeuverCommand = Math.min(minimumManeuverCommand, live.controller.speedCommand);
      }
    }
    const finished = engine.getSnapshot();
    const minimumCenterDistance = Math.min(...finished.world.trail.map((point) => Math.hypot(point.x - obstacle.x, point.y - obstacle.y)));
    expect(maneuverObserved).toBe(true);
    expect(minimumManeuverCommand).toBeGreaterThan(0);
    expect(minimumCenterDistance - obstacle.radius - 0.36).toBeGreaterThanOrEqual(0.1);
  });

  it('sequences staggered visible obstacles instead of blocking on a future maneuver', () => {
    const engine = new AutonomyEngine('oval');
    // Regression coordinates reconstructed from the reported stopped oval state.
    engine.editWorld('obstacle', { x: 3.54, y: 7.51 });
    engine.editWorld('obstacle', { x: 7.95, y: 4.31 });
    engine.setRunning(true);
    const wallStart = performance.now();
    let minimumCommand = Number.POSITIVE_INFINITY;
    for (let frame = 0; frame <= 1800; frame += 1) {
      engine.advanceFrame(wallStart + frame * (1000 / 60));
      const live = engine.getSnapshot();
      if (live.avoidance.active) minimumCommand = Math.min(minimumCommand, live.controller.speedCommand);
      expect(live.controller.holdReason, `t=${live.simulationTime.toFixed(2)}`).toBeNull();
    }
    const snapshot = engine.getSnapshot();
    expect(minimumCommand).toBeGreaterThan(0);
    for (const obstacle of snapshot.world.obstacles) {
      const clearance = Math.min(...snapshot.world.trail.map((point) => Math.hypot(point.x - obstacle.x, point.y - obstacle.y))) - obstacle.radius - 0.36;
      expect(clearance, obstacle.id).toBeGreaterThanOrEqual(0.08);
    }
  });

  it('holds position when multiple obstacles block both width-safe routes', () => {
    const engine = new AutonomyEngine('obstacle');
    engine.editWorld('obstacle', { x: 4, y: -5.39 });
    engine.editWorld('obstacle', { x: 4, y: -7.81 });
    const snapshot = engine.getSnapshot();
    expect(snapshot.avoidance.active).toBe(true);
    expect(snapshot.avoidance.candidates.every((candidate) => !candidate.feasible)).toBe(true);
    expect(snapshot.avoidance.phase).toBe('BLOCKED');
    expect(snapshot.controller.speedCommand).toBe(0);
    expect(snapshot.controller.holdReason).toBe('STOP — NO SAFE OBSTACLE ROUTE');
  });
});
