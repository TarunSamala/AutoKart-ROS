import type { MovementSnapshot, VehicleState } from './types';
import { clamp, normalizeAngle } from './math';
import { WHEELBASE } from './controller';

export const PHYSICS_DT = 1 / 60;
const MAX_ACCELERATION = 1.8;
export const MAX_BRAKING = 3.2;

export function stationaryMovement(vehicle: VehicleState): MovementSnapshot {
  return {
    dt: 0,
    oldState: { ...vehicle },
    newState: { ...vehicle },
    yawRate: vehicle.yawRate,
    deltaHeading: 0,
    deltaX: 0,
    deltaY: 0,
  };
}

export function integrateBicycle(vehicle: VehicleState, dt: number): MovementSnapshot {
  const oldState = { ...vehicle };
  const velocityError = vehicle.speedCommand - vehicle.velocity;
  const acceleration = clamp(velocityError * 2.4, -MAX_BRAKING, MAX_ACCELERATION);
  const yawRate = (vehicle.velocity / WHEELBASE) * Math.tan(vehicle.steeringAngle);
  const deltaHeading = yawRate * dt;
  const deltaX = vehicle.velocity * Math.cos(vehicle.heading) * dt;
  const deltaY = vehicle.velocity * Math.sin(vehicle.heading) * dt;
  const newState: VehicleState = {
    ...vehicle,
    x: vehicle.x + deltaX,
    y: vehicle.y + deltaY,
    heading: normalizeAngle(vehicle.heading + deltaHeading),
    velocity: Math.max(0, vehicle.velocity + acceleration * dt),
    acceleration,
    yawRate,
  };
  return { dt, oldState, newState, yawRate, deltaHeading, deltaX, deltaY };
}
