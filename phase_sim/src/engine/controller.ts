import type { AvoidanceSnapshot, ControllerSnapshot, DecisionState, PathPoint, SensorSnapshot, VehicleState } from './types';
import { clamp, distance } from './math';

export const WHEELBASE = 1.05;
export const DESIRED_LOOKAHEAD = 1.3;
export const AVOIDANCE_LOOKAHEAD = 0.45;
export const CRUISE_SPEED = 1.35;
export const CORNER_MIN_SPEED = 0.75;
export const MANEUVER_SPEED = 0.9;
export const MAX_STEERING = (32 * Math.PI) / 180;

export function emptyController(vehicle?: VehicleState): ControllerSnapshot {
  const vehicleAtCalculation: VehicleState = vehicle ? { ...vehicle } : {
    x: 0, y: 0, heading: 0, velocity: 0, acceleration: 0, steeringAngle: 0, yawRate: 0, speedCommand: 0,
  };
  return {
    vehicleAtCalculation,
    nearestIndex: -1,
    nearest: null,
    targetIndex: -1,
    target: null,
    desiredLookahead: DESIRED_LOOKAHEAD,
    dx: 0,
    dy: 0,
    longitudinal: 0,
    lateral: 0,
    lookaheadDistance: 0,
    alpha: 0,
    curvature: 0,
    steeringAngle: 0,
    crossTrackError: 0,
    speedCommand: 0,
    stopReason: 'STOP — PATH LOST',
    holdReason: null,
    avoidanceActive: false,
    avoidanceSide: null,
    avoidancePhase: 'CLEAR',
  };
}

export function runController(path: PathPoint[], sensor: SensorSnapshot, vehicle: VehicleState, avoidance?: AvoidanceSnapshot): ControllerSnapshot {
  const desiredLookahead = avoidance?.active ? AVOIDANCE_LOOKAHEAD : DESIRED_LOOKAHEAD;
  const holdReason: DecisionState | null = avoidance?.phase === 'BLOCKED'
    ? 'STOP — NO SAFE OBSTACLE ROUTE'
    : null;
  let stopReason: DecisionState | null = null;
  if (sensor.visibleCones.length < 4 || sensor.blueVisible < 2 || sensor.yellowVisible < 2) {
    stopReason = 'STOP — INSUFFICIENT CONES';
  } else if (path.length === 0) stopReason = 'STOP — NO CORRIDOR';
  else if (path.length < 3) stopReason = 'STOP — PATH LOST';

  if (path.length === 0) return { ...emptyController(vehicle), stopReason: stopReason ?? 'STOP — NO CORRIDOR' };

  let nearestIndex = 0;
  let nearestDistance = Number.POSITIVE_INFINITY;
  for (let i = 0; i < path.length; i += 1) {
    const candidateDistance = distance(vehicle, path[i]);
    if (candidateDistance < nearestDistance) {
      nearestDistance = candidateDistance;
      nearestIndex = i;
    }
  }
  const nearest = path[nearestIndex];
  let targetIndex = nearestIndex;
  const requiredS = nearest.s + desiredLookahead;
  while (targetIndex < path.length - 1 && path[targetIndex].s < requiredS) targetIndex += 1;
  const target = path[targetIndex];
  const dx = target.x - vehicle.x;
  const dy = target.y - vehicle.y;
  const cos = Math.cos(vehicle.heading);
  const sin = Math.sin(vehicle.heading);
  const longitudinal = cos * dx + sin * dy;
  const lateral = -sin * dx + cos * dy;
  const lookaheadDistance = Math.hypot(longitudinal, lateral);
  const alpha = Math.atan2(lateral, longitudinal);
  const curvature = lookaheadDistance > 1e-6 ? (2 * Math.sin(alpha)) / lookaheadDistance : 0;
  const steeringAngle = stopReason ? 0 : clamp(Math.atan(WHEELBASE * curvature), -MAX_STEERING, MAX_STEERING);
  const curvatureSpeed = clamp(CRUISE_SPEED / (1 + 1.6 * Math.abs(curvature)), CORNER_MIN_SPEED, CRUISE_SPEED);
  return {
    vehicleAtCalculation: { ...vehicle },
    nearestIndex,
    nearest,
    targetIndex,
    target,
    desiredLookahead,
    dx,
    dy,
    longitudinal,
    lateral,
    lookaheadDistance,
    alpha,
    curvature,
    steeringAngle,
    crossTrackError: nearestDistance,
    speedCommand: stopReason || holdReason ? 0 : avoidance?.active ? MANEUVER_SPEED : curvatureSpeed,
    stopReason,
    holdReason,
    avoidanceActive: Boolean(avoidance?.active),
    avoidanceSide: avoidance?.selectedSide ?? null,
    avoidancePhase: avoidance?.phase ?? 'CLEAR',
  };
}

export function controllerDecision(controller: ControllerSnapshot): DecisionState {
  if (controller.stopReason) return controller.stopReason;
  if (controller.holdReason) return controller.holdReason;
  if (controller.avoidanceActive && controller.avoidanceSide) return `MANEUVER ${controller.avoidanceSide} — OBSTACLE`;
  const degrees = (controller.steeringAngle * 180) / Math.PI;
  if (Math.abs(degrees) < 1.5) return 'GO STRAIGHT';
  return degrees > 0 ? 'STEER LEFT' : 'STEER RIGHT';
}
