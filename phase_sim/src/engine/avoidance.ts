import type {
  AvoidanceCandidate,
  AvoidanceSide,
  AvoidanceSnapshot,
  Edge,
  Midpoint,
  PathPoint,
  SensorSnapshot,
} from './types';
import { distance } from './math';

export const KART_HALF_WIDTH = 0.36;
export const OBSTACLE_CLEARANCE = 0.4;
export const BOUNDARY_CLEARANCE = 0.12;
export const MANEUVER_SPAN = 4.2;
export const MANEUVER_HOLD = 1.05;

export function emptyAvoidance(path: PathPoint[] = []): AvoidanceSnapshot {
  return {
    active: false,
    phase: 'CLEAR',
    trackingSource: null,
    obstacle: null,
    obstaclePathIndex: -1,
    obstaclePathS: 0,
    obstacleLateral: 0,
    currentClearance: Number.POSITIVE_INFINITY,
    brakingDistance: 0,
    candidates: [],
    selectedSide: null,
    selectedPath: path,
  };
}

function pathWithOffset(basePath: PathPoint[], obstacleS: number, offset: number): { path: PathPoint[]; maxAppliedOffset: number } {
  let cumulative = 0;
  let maxAppliedOffset = 0;
  const path: PathPoint[] = [];
  for (let index = 0; index < basePath.length; index += 1) {
    const point = basePath[index];
    const previous = basePath[Math.max(0, index - 1)];
    const next = basePath[Math.min(basePath.length - 1, index + 1)];
    const tangentHeading = Math.atan2(next.y - previous.y, next.x - previous.x);
    const distanceFromObstacle = Math.abs(point.s - obstacleS);
    const blend = distanceFromObstacle <= MANEUVER_HOLD
      ? 1
      : distanceFromObstacle < MANEUVER_SPAN
        ? 0.5 * (1 + Math.cos(Math.PI * (distanceFromObstacle - MANEUVER_HOLD) / (MANEUVER_SPAN - MANEUVER_HOLD)))
        : 0;
    const appliedOffset = offset * blend;
    maxAppliedOffset = Math.max(maxAppliedOffset, Math.abs(appliedOffset));
    const shifted = {
      ...point,
      x: point.x - Math.sin(tangentHeading) * appliedOffset,
      y: point.y + Math.cos(tangentHeading) * appliedOffset,
      index,
      s: 0,
    };
    if (path.length > 0) cumulative += distance(path[path.length - 1], shifted);
    shifted.s = cumulative;
    path.push(shifted);
  }
  return { path, maxAppliedOffset };
}

function candidateForSide(
  side: AvoidanceSide,
  basePath: PathPoint[],
  obstaclePathS: number,
  obstacleLateral: number,
  obstacleRadius: number,
  corridorWidth: number,
  preferredSide: AvoidanceSide | null,
  obstacles: SensorSnapshot['visibleObstacles'],
): AvoidanceCandidate {
  const centerClearance = obstacleRadius + KART_HALF_WIDTH + OBSTACLE_CLEARANCE;
  const desiredOffset = side === 'LEFT'
    ? obstacleLateral + centerClearance
    : obstacleLateral - centerClearance;
  const shifted = pathWithOffset(basePath, obstaclePathS, desiredOffset);
  const minimumObstacleClearance = obstacles.length
    ? Math.min(...obstacles.flatMap((tracked) => shifted.path.map((point) =>
      Math.hypot(point.x - tracked.obstacle.x, point.y - tracked.obstacle.y) - tracked.obstacle.radius - KART_HALF_WIDTH,
    )))
    : Number.POSITIVE_INFINITY;
  const minimumBoundaryClearance = corridorWidth / 2 - shifted.maxAppliedOffset - KART_HALF_WIDTH;
  const obstaclePass = minimumObstacleClearance >= OBSTACLE_CLEARANCE - 0.025;
  const boundaryPass = minimumBoundaryClearance >= BOUNDARY_CLEARANCE;
  const sideClearance = side === 'LEFT'
    ? desiredOffset - obstacleLateral
    : obstacleLateral - desiredOffset;
  const directionPass = sideClearance >= centerClearance - 1e-6;
  const feasible = obstaclePass && boundaryPass && directionPass;
  const switchPenalty = preferredSide && preferredSide !== side ? 100 : 0;
  const score =
    (feasible ? 0 : 1000) +
    Math.abs(desiredOffset) * 4 +
    Math.max(0, OBSTACLE_CLEARANCE - minimumObstacleClearance) * 80 +
    Math.max(0, BOUNDARY_CLEARANCE - minimumBoundaryClearance) * 80 +
    switchPenalty -
    Math.min(0.8, minimumObstacleClearance) * 0.5 -
    Math.min(0.8, minimumBoundaryClearance) * 0.5;
  const reason = !directionPass
    ? 'CANDIDATE CANNOT CLEAR THE SELECTED OBSTACLE SIDE'
    : !obstaclePass
      ? 'INSUFFICIENT OBSTACLE CLEARANCE'
      : !boundaryPass
        ? 'INSUFFICIENT CONE-BOUNDARY CLEARANCE'
        : 'OBSTACLE AND BOUNDARY CLEARANCES PASS';
  return {
    side,
    feasible,
    desiredOffset,
    maxAppliedOffset: shifted.maxAppliedOffset,
    corridorWidth,
    minimumObstacleClearance,
    minimumBoundaryClearance,
    requiredObstacleClearance: OBSTACLE_CLEARANCE,
    requiredBoundaryClearance: BOUNDARY_CLEARANCE,
    score,
    reason,
    path: shifted.path,
  };
}

export function planObstacleAvoidance(
  basePath: PathPoint[],
  sensor: SensorSnapshot,
  midpoints: Midpoint[],
  crossTrackEdges: Edge[],
  preferredSide: AvoidanceSide | null,
  retainedObstacle: SensorSnapshot['visibleObstacles'][number] | null = null,
): AvoidanceSnapshot {
  const obstacleInputs = [...sensor.visibleObstacles];
  if (retainedObstacle && !obstacleInputs.some((item) => item.obstacle.id === retainedObstacle.obstacle.id)) {
    obstacleInputs.push(retainedObstacle);
  }
  if (basePath.length < 3 || obstacleInputs.length === 0) return emptyAvoidance(basePath);

  const projected = obstacleInputs.map((obstacle) => {
    let pathIndex = 0;
    let centerDistance = Number.POSITIVE_INFINITY;
    basePath.forEach((point, index) => {
      const currentDistance = Math.hypot(point.x - obstacle.obstacle.x, point.y - obstacle.obstacle.y);
      if (currentDistance < centerDistance) {
        centerDistance = currentDistance;
        pathIndex = index;
      }
    });
    return { obstacle, pathIndex, centerDistance };
  }).sort((a, b) => a.obstacle.longitudinal - b.obstacle.longitudinal);
  const threat = projected.find((item) =>
    item.obstacle.longitudinal > -0.45 &&
    item.centerDistance <= item.obstacle.obstacle.radius + KART_HALF_WIDTH + OBSTACLE_CLEARANCE + 0.35,
  );
  if (!threat) return emptyAvoidance(basePath);

  const closest = basePath[threat.pathIndex];
  const previous = basePath[Math.max(0, threat.pathIndex - 1)];
  const next = basePath[Math.min(basePath.length - 1, threat.pathIndex + 1)];
  const tangentHeading = Math.atan2(next.y - previous.y, next.x - previous.x);
  const obstacleDx = threat.obstacle.obstacle.x - closest.x;
  const obstacleDy = threat.obstacle.obstacle.y - closest.y;
  const obstacleLateral = -Math.sin(tangentHeading) * obstacleDx + Math.cos(tangentHeading) * obstacleDy;
  const nearbyWidths = midpoints
    .map((midpoint) => ({ midpoint, edge: crossTrackEdges.find((edge) => edge.id === midpoint.sourceEdgeId) }))
    .filter((item) => item.edge && distance(item.midpoint, closest) <= MANEUVER_SPAN + 1)
    .map((item) => item.edge!.length);
  const corridorWidth = nearbyWidths.length
    ? Math.min(...nearbyWidths)
    : Math.min(...crossTrackEdges.map((edge) => edge.length));
  if (!Number.isFinite(corridorWidth)) return emptyAvoidance(basePath);

  // Score the maneuver that is physically active now. A later obstacle outside this
  // maneuver window gets its own fresh left/right decision as the 15 Hz planner
  // advances; allowing it to invalidate the unshifted far end of both candidates
  // would stop the kart even though two safe sequential maneuvers exist.
  const localObstacles = projected
    .filter((item) =>
      item.obstacle.obstacle.id === threat.obstacle.obstacle.id ||
      (item.obstacle.longitudinal > -0.45 && Math.abs(basePath[item.pathIndex].s - closest.s) <= MANEUVER_SPAN),
    )
    .map((item) => item.obstacle);

  const candidates = (['LEFT', 'RIGHT'] as AvoidanceSide[]).map((side) => candidateForSide(
    side,
    basePath,
    closest.s,
    obstacleLateral,
    threat.obstacle.obstacle.radius,
    corridorWidth,
    preferredSide,
    localObstacles,
  ));
  const committed = preferredSide ? candidates.find((candidate) => candidate.side === preferredSide && candidate.feasible) : null;
  const selected = committed ?? [...candidates].sort((a, b) => a.score - b.score)[0];
  return {
    active: true,
    phase: 'MANEUVERING',
    trackingSource: threat.obstacle.detected ? 'LIVE SENSOR' : 'COMMITTED TRACK',
    obstacle: threat.obstacle,
    obstaclePathIndex: threat.pathIndex,
    obstaclePathS: closest.s,
    obstacleLateral,
    currentClearance: Number.POSITIVE_INFINITY,
    brakingDistance: 0,
    candidates,
    selectedSide: selected.side,
    selectedPath: selected.path,
  };
}
