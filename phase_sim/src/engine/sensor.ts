import type { Cone, ConeDetection, Obstacle, ObstacleDetection, SensorSnapshot, VehicleState } from './types';
import { normalizeAngle } from './math';

export const DEFAULT_SENSOR_RANGE = 8;
export const DEFAULT_SENSOR_FOV = (130 * Math.PI) / 180;
export const OBSTACLE_STOP_DISTANCE = 2;

export function calculateConeDetection(
  cone: Cone,
  vehicle: VehicleState,
  range = DEFAULT_SENSOR_RANGE,
  fov = DEFAULT_SENSOR_FOV,
): ConeDetection {
  const dx = cone.x - vehicle.x;
  const dy = cone.y - vehicle.y;
  const cos = Math.cos(vehicle.heading);
  const sin = Math.sin(vehicle.heading);
  const longitudinal = cos * dx + sin * dy;
  const lateral = -sin * dx + cos * dy;
  const distance = Math.hypot(dx, dy);
  const bearing = normalizeAngle(Math.atan2(lateral, longitudinal));
  const rangePass = distance <= range;
  const fovPass = Math.abs(bearing) <= fov / 2;
  const forwardPass = longitudinal > 0;
  return {
    cone,
    dx,
    dy,
    longitudinal,
    lateral,
    distance,
    bearing,
    rangePass,
    fovPass,
    forwardPass,
    detected: rangePass && fovPass && forwardPass,
  };
}

export function calculateObstacleDetection(
  obstacle: Obstacle,
  vehicle: VehicleState,
  range = DEFAULT_SENSOR_RANGE,
  fov = DEFAULT_SENSOR_FOV,
): ObstacleDetection {
  const dx = obstacle.x - vehicle.x;
  const dy = obstacle.y - vehicle.y;
  const cos = Math.cos(vehicle.heading);
  const sin = Math.sin(vehicle.heading);
  const longitudinal = cos * dx + sin * dy;
  const lateral = -sin * dx + cos * dy;
  const distance = Math.hypot(dx, dy) - obstacle.radius;
  const bearing = normalizeAngle(Math.atan2(lateral, longitudinal));
  const detected = distance <= range && Math.abs(bearing) <= fov / 2 && longitudinal > 0;
  const unsafe = detected && distance <= OBSTACLE_STOP_DISTANCE && Math.abs(lateral) <= obstacle.radius + 0.8;
  return { obstacle, dx, dy, longitudinal, lateral, distance, bearing, detected, unsafe };
}

export function runSensor(
  cones: Cone[],
  obstacles: Obstacle[],
  vehicle: VehicleState,
  range = DEFAULT_SENSOR_RANGE,
  fov = DEFAULT_SENSOR_FOV,
): SensorSnapshot {
  const detections = cones.map((cone) => calculateConeDetection(cone, vehicle, range, fov));
  const visibleCones = detections.filter((item) => item.detected).map((item) => item.cone);
  const visibleObstacles = obstacles
    .map((obstacle) => calculateObstacleDetection(obstacle, vehicle, range, fov))
    .filter((item) => item.detected);
  return {
    vehicleAtCalculation: { ...vehicle },
    range,
    fov,
    worldConeCount: cones.length,
    detections,
    visibleCones,
    visibleObstacles,
    blueVisible: visibleCones.filter((cone) => cone.side === 'BLUE').length,
    yellowVisible: visibleCones.filter((cone) => cone.side === 'YELLOW').length,
  };
}

export function emptySensor(coneCount = 0): SensorSnapshot {
  const vehicleAtCalculation: VehicleState = {
    x: 0, y: 0, heading: 0, velocity: 0, acceleration: 0, steeringAngle: 0, yawRate: 0, speedCommand: 0,
  };
  return {
    vehicleAtCalculation,
    range: DEFAULT_SENSOR_RANGE,
    fov: DEFAULT_SENSOR_FOV,
    worldConeCount: coneCount,
    detections: [],
    visibleCones: [],
    visibleObstacles: [],
    blueVisible: 0,
    yellowVisible: 0,
  };
}
