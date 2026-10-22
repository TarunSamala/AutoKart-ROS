import { radToDeg } from '../engine/math';

export const fixed = (value: number, digits = 3) => Number.isFinite(value) ? value.toFixed(digits) : '—';
export const meters = (value: number, digits = 3) => `${fixed(value, digits)} m`;
export const seconds = (value: number, digits = 2) => `${fixed(value, digits)} s`;
export const degrees = (value: number, digits = 2) => `${value >= 0 ? '+' : ''}${fixed(radToDeg(value), digits)}°`;
export const radians = (value: number, digits = 3) => `${value >= 0 ? '+' : ''}${fixed(value, digits)}`;
export const pass = (value: boolean) => value ? 'PASS' : 'FAIL';
export const direction = (steering: number) => Math.abs(radToDeg(steering)) < 1.5 ? 'STRAIGHT' : steering > 0 ? 'LEFT' : 'RIGHT';
