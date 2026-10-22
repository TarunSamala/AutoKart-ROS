import type { Point2D } from './types';

export const TAU = Math.PI * 2;
export const radToDeg = (value: number) => (value * 180) / Math.PI;
export const degToRad = (value: number) => (value * Math.PI) / 180;
export const clamp = (value: number, min: number, max: number) => Math.max(min, Math.min(max, value));

export function normalizeAngle(angle: number): number {
  let result = angle;
  while (result > Math.PI) result -= TAU;
  while (result < -Math.PI) result += TAU;
  return result;
}

export function distance(a: Point2D, b: Point2D): number {
  return Math.hypot(b.x - a.x, b.y - a.y);
}

export function pointInPolygon(point: Point2D, polygon: Point2D[]): boolean {
  if (polygon.length < 3) return false;
  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const xi = polygon[i].x;
    const yi = polygon[i].y;
    const xj = polygon[j].x;
    const yj = polygon[j].y;
    const crosses = yi > point.y !== yj > point.y;
    const xAtY = ((xj - xi) * (point.y - yi)) / (yj - yi || Number.EPSILON) + xi;
    if (crosses && point.x < xAtY) inside = !inside;
  }
  return inside;
}

export function pointToSegmentDistance(point: Point2D, a: Point2D, b: Point2D): number {
  const vx = b.x - a.x;
  const vy = b.y - a.y;
  const lengthSquared = vx * vx + vy * vy;
  if (lengthSquared === 0) return distance(point, a);
  const t = clamp(((point.x - a.x) * vx + (point.y - a.y) * vy) / lengthSquared, 0, 1);
  return Math.hypot(point.x - (a.x + t * vx), point.y - (a.y + t * vy));
}

export function signed(value: number, digits = 2): string {
  return `${value >= 0 ? '+' : ''}${value.toFixed(digits)}`;
}
