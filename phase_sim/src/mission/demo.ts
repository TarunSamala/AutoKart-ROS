import type { PlanningSnapshot } from '../engine/types';
import type { Cone, Edge, MissionFrame, Point } from './schema';
import { COURSE_PRESETS, MISSION_DEMO_PRESETS } from '../engine/course';

const point = (item: { x: number; y: number }): Point => ({ x_m: item.x, y_m: item.y });

function observedWinding(trail: Array<{ x: number; y: number }>, cx: number): number {
  let radians = 0;
  for (let index = 1; index < trail.length; index += 1) {
    const a = trail[index - 1], b = trail[index];
    if (Math.hypot(a.x - cx, a.y) > 16 || Math.hypot(b.x - cx, b.y) > 16) continue;
    let delta = Math.atan2(b.y, b.x - cx) - Math.atan2(a.y, a.x - cx);
    if (delta > Math.PI) delta -= 2 * Math.PI;
    if (delta < -Math.PI) delta += 2 * Math.PI;
    radians += delta;
  }
  return radians / (2 * Math.PI);
}

export function gabrielForDemo(edges: Edge[], cones: Cone[]): Edge[] {
  const byId = new Map(cones.map((cone) => [cone.id, cone]));
  return edges.map((edge) => {
    const a = byId.get(edge.a), b = byId.get(edge.b);
    if (!a || !b) return { ...edge, accepted: false, reason: 'MISSING ENDPOINT' };
    const midpoint = { x_m: (a.x_m + b.x_m) / 2, y_m: (a.y_m + b.y_m) / 2 };
    const radiusSq = ((a.x_m - b.x_m) ** 2 + (a.y_m - b.y_m) ** 2) / 4;
    const blocker = cones.find((cone) => cone.id !== a.id && cone.id !== b.id &&
      (cone.x_m - midpoint.x_m) ** 2 + (cone.y_m - midpoint.y_m) ** 2 < radiusSq - 1e-9);
    return { ...edge, accepted: !blocker, blocking_cone_id: blocker?.id, radius_m: Math.sqrt(radiusSq), reason: blocker ? `CONE ${blocker.id} INSIDE DIAMETER DISK` : 'EMPTY DIAMETER DISK' };
  });
}

export function demoFrame(snapshot: PlanningSnapshot, preset: string): MissionFrame {
  const course = [...MISSION_DEMO_PRESETS, ...COURSE_PRESETS].find((item) => item.id === preset) ?? COURSE_PRESETS[0];
  const worldCones = snapshot.world.cones.map((cone) => ({ ...point(cone), id: cone.id, class: cone.side, confidence: 1 }));
  const start = course.start;
  const across = { x: -Math.sin(start.heading) * 1.8, y: Math.cos(start.heading) * 1.8 };
  const specialCones: Cone[] = [-1, 1].map((side) => ({ id: `DEMO_START_${side}`, class: 'ORANGE', confidence: 1, x_m: start.x + across.x * side, y_m: start.y + across.y * side }));
  const cones: Cone[] = snapshot.sensor.visibleCones.map((cone) => {
    const detection = snapshot.sensor.detections.find((item) => item.cone.id === cone.id);
    return { ...point(cone), id: cone.id, class: cone.side, confidence: 1, range_m: detection?.distance, bearing_rad: detection?.bearing };
  });
  const delaunayEdges: Edge[] = snapshot.edges.uniqueEdges.map((edge) => ({ id: edge.id, a: edge.a.id, b: edge.b.id, triangle_ids: edge.triangleIds }));
  const gabrielEdges = gabrielForDemo(delaunayEdges, cones);
  const vehicle = snapshot.vehicle;
  return {
    schema_version: 1, source: 'DEMO', run_id: `demo-${preset}`, scenario: preset, sequence: snapshot.frame, timestamp_s: snapshot.simulationTime,
    vehicle: { ...point(vehicle), yaw_rad: vehicle.heading, speed_mps: vehicle.velocity, measured_steering_rad: vehicle.steeringAngle, requested_steering_rad: snapshot.controller.steeringAngle, target_speed_mps: snapshot.controller.speedCommand, brake_command: snapshot.controller.stopReason || snapshot.avoidance.phase === 'BLOCKED' ? 1 : 0 },
    perception: {
      cones,
      sensor_range_m: snapshot.sensor.range,
      sensor_fov_rad: snapshot.sensor.fov,
      ground_truth_cones: worldCones,
      obstacles: snapshot.sensor.visibleObstacles.map((item) => ({ ...point(item.obstacle), id: item.obstacle.id, radius_m: item.obstacle.radius, distance_m: item.distance })),
    },
    visualization: {
      course_cones: [...worldCones, ...specialCones],
      trail: snapshot.world.trail.map(point),
      special_markers: [{ ...point(course.start), id: 'start', label: 'START' }],
      skidpad: preset === 'mission-skidpad' ? { left_center: { x_m: -10, y_m: 0 }, right_center: { x_m: 10, y_m: 0 }, radius_m: 10, w_left: observedWinding(snapshot.world.trail, -10), w_right: observedWinding(snapshot.world.trail, 10) } : undefined,
    },
    planning: {
      delaunay_edges: delaunayEdges, gabriel_edges: gabrielEdges,
      gates: snapshot.midpoints.map((midpoint) => ({ id: midpoint.id, midpoint: point(midpoint), width_m: Math.hypot(midpoint.blue.x - midpoint.yellow.x, midpoint.blue.y - midpoint.yellow.y), valid: true, confidence: 1 })),
      candidate_paths: snapshot.avoidance.candidates.map((candidate, index) => ({ id: candidate.side, points: candidate.path.map(point), costs: { avoidance_score: candidate.score }, total_cost: candidate.score, rank: index + 1, selected: candidate.side === snapshot.avoidance.selectedSide, reason: candidate.reason })),
      selected_path: snapshot.interpolation.path.map(point), lookahead: snapshot.controller.target ? point(snapshot.controller.target) : null,
    },
    safety: { state: 'DEMO ONLY', priority: snapshot.controller.stopReason || snapshot.avoidance.phase === 'BLOCKED' ? 'CONTROLLED_STOP' : 'NORMAL_AUTONOMY', faults: [], obstacle: snapshot.avoidance.obstacle ? { id: snapshot.avoidance.obstacle.obstacle.id, distance_m: snapshot.avoidance.obstacle.distance, relative_speed_mps: vehicle.velocity, ttc_s: vehicle.velocity > 0.05 ? snapshot.avoidance.obstacle.distance / vehicle.velocity : null, stopping_distance_m: snapshot.avoidance.brakingDistance, available_distance_m: snapshot.avoidance.obstacle.distance, margin_m: snapshot.avoidance.obstacle.distance - snapshot.avoidance.brakingDistance, brake_trigger: snapshot.avoidance.phase === 'BLOCKED', predicted_stop: vehicle.velocity > 0.15 ? { x_m: vehicle.x + Math.cos(vehicle.heading) * snapshot.avoidance.brakingDistance, y_m: vehicle.y + Math.sin(vehicle.heading) * snapshot.avoidance.brakingDistance } : undefined } : undefined },
    metrics: { cross_track_error_m: snapshot.controller.crossTrackError, planner_runtime_ms: snapshot.timing.totalPlanMs, controller_runtime_ms: snapshot.timing.controllerMs },
  };
}
