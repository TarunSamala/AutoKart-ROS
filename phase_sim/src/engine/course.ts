import type { Cone, CoursePreset, VehicleState } from './types';

const baseVehicle = (x: number, y: number, heading: number): VehicleState => ({
  x,
  y,
  heading,
  velocity: 0,
  acceleration: 0,
  steeringAngle: 0,
  yawRate: 0,
  speedCommand: 0,
});

function ellipseCones(rx: number, ry: number, width: number, count: number, wave = 0): Cone[] {
  const cones: Cone[] = [];
  for (let i = 0; i < count; i += 1) {
    const theta = -Math.PI / 2 + (i / count) * Math.PI * 2;
    const wobble = wave * Math.sin(theta * 3);
    const cx = (rx + wobble) * Math.cos(theta);
    const cy = ry * Math.sin(theta);
    const tx = -rx * Math.sin(theta) + wave * 3 * Math.cos(theta * 3) * Math.cos(theta) - wobble * Math.sin(theta);
    const ty = ry * Math.cos(theta);
    const length = Math.hypot(tx, ty);
    const nx = -ty / length;
    const ny = tx / length;
    cones.push({ id: `B${String(i + 1).padStart(2, '0')}`, side: 'BLUE', x: cx + nx * width / 2, y: cy + ny * width / 2 });
    cones.push({ id: `Y${String(i + 1).padStart(2, '0')}`, side: 'YELLOW', x: cx - nx * width / 2, y: cy - ny * width / 2 });
  }
  return cones;
}

function parametricCones(
  curve: (t: number) => { x: number; y: number },
  width: number,
  count: number,
  phase = -Math.PI / 2,
): Cone[] {
  const centers = Array.from({ length: count }, (_, index) => curve(phase + (index / count) * Math.PI * 2));
  const cones: Cone[] = [];
  centers.forEach((center, index) => {
    const previous = centers[(index - 1 + count) % count];
    const next = centers[(index + 1) % count];
    const tx = next.x - previous.x;
    const ty = next.y - previous.y;
    const length = Math.hypot(tx, ty) || 1;
    const nx = -ty / length;
    const ny = tx / length;
    cones.push({ id: `B${String(index + 1).padStart(2, '0')}`, side: 'BLUE', x: center.x + nx * width / 2, y: center.y + ny * width / 2 });
    cones.push({ id: `Y${String(index + 1).padStart(2, '0')}`, side: 'YELLOW', x: center.x - nx * width / 2, y: center.y - ny * width / 2 });
  });
  return cones;
}

function startOnCurve(curve: (t: number) => { x: number; y: number }, phase = -Math.PI / 2): VehicleState {
  const point = curve(phase);
  const next = curve(phase + 0.001);
  return baseVehicle(point.x, point.y, Math.atan2(next.y - point.y, next.x - point.x));
}

const skidPadCurve = (t: number) => {
  const radius = 8 + 1.6 * Math.cos(2 * t);
  return { x: 1.2 * radius * Math.cos(t), y: 0.72 * radius * Math.sin(t) };
};
const hairpinCurve = (t: number) => ({
  x: 11.5 * Math.sign(Math.cos(t)) * Math.abs(Math.cos(t)) ** 0.62,
  y: 6.2 * Math.sign(Math.sin(t)) * Math.abs(Math.sin(t)) ** 0.62,
});
const dynamicCurve = (t: number) => {
  const radial = 10.9 + 0.7 * Math.sin(3 * t) + 0.28 * Math.sin(5 * t + 0.4);
  return { x: radial * Math.cos(t), y: (7 + 0.28 * Math.cos(2 * t)) * Math.sin(t) };
};

const missionSkidpadCurve = (t: number) => ({ x: 20 * Math.sin(t), y: 11 * Math.sin(2 * t) });
const missionAutocrossCurve = (t: number) => {
  const point = dynamicCurve(t);
  return { x: point.x * 3.1, y: point.y * 3.1 };
};
const brakeCones = (): Cone[] => Array.from({ length: 41 }, (_, index) => [
  { id: `B${index + 1}`, side: 'BLUE' as const, x: index * 2, y: 2.2 },
  { id: `Y${index + 1}`, side: 'YELLOW' as const, x: index * 2, y: -2.2 },
]).flat();

function chicaneCones(): Cone[] {
  const cones: Cone[] = [];
  const count = 58;
  const width = 3.4;
  for (let i = 0; i < count; i += 1) {
    const theta = -Math.PI / 2 + (i / count) * Math.PI * 2;
    const radiusX = 12 + 1.6 * Math.sin(theta * 3);
    const cx = radiusX * Math.cos(theta);
    const cy = 7.3 * Math.sin(theta);
    const epsilon = 0.001;
    const t2 = theta + epsilon;
    const r2 = 12 + 1.6 * Math.sin(t2 * 3);
    const tx = (r2 * Math.cos(t2) - cx) / epsilon;
    const ty = (7.3 * Math.sin(t2) - cy) / epsilon;
    const length = Math.hypot(tx, ty);
    const nx = -ty / length;
    const ny = tx / length;
    cones.push({ id: `B${String(i + 1).padStart(2, '0')}`, side: 'BLUE', x: cx + nx * width / 2, y: cy + ny * width / 2 });
    cones.push({ id: `Y${String(i + 1).padStart(2, '0')}`, side: 'YELLOW', x: cx - nx * width / 2, y: cy - ny * width / 2 });
  }
  return cones;
}

export const COURSE_PRESETS: CoursePreset[] = [
  {
    id: 'oval',
    name: 'Engineering Oval',
    cones: ellipseCones(12, 7, 3.4, 56),
    obstacles: [],
    start: baseVehicle(0, -7, 0),
  },
  {
    id: 'chicane',
    name: 'Technical Chicane',
    cones: chicaneCones(),
    obstacles: [],
    start: baseVehicle(0, -7.3, 0),
  },
  {
    id: 'skidpad',
    name: 'Skid Pad Twin-Lobe',
    cones: parametricCones(skidPadCurve, 3.2, 80),
    obstacles: [],
    start: startOnCurve(skidPadCurve),
  },
  {
    id: 'hairpin',
    name: 'Hairpin Circuit',
    cones: parametricCones(hairpinCurve, 3.4, 68),
    obstacles: [],
    start: startOnCurve(hairpinCurve),
  },
  {
    id: 'dynamic',
    name: 'Dynamic Mixed Circuit',
    cones: parametricCones(dynamicCurve, 3.5, 72),
    obstacles: [],
    start: startOnCurve(dynamicCurve),
  },
  {
    id: 'obstacle',
    name: 'Obstacle Maneuver Test',
    cones: ellipseCones(12, 7, 3.4, 56),
    obstacles: [{ id: 'O1', x: 4, y: -6.6, radius: 0.45 }],
    start: baseVehicle(0, -7, 0),
  },
];

/** Larger display/demo courses. The existing engine remains responsible for all motion and planning. */
export const MISSION_DEMO_PRESETS: CoursePreset[] = [
  { id: 'mission-skidpad', name: 'Skidpad · full figure eight', cones: parametricCones(missionSkidpadCurve, 4.2, 96, -Math.PI / 2), obstacles: [], start: startOnCurve(missionSkidpadCurve) },
  { id: 'mission-autocross', name: 'Autocross · extended circuit', cones: parametricCones(missionAutocrossCurve, 4.2, 112), obstacles: [], start: startOnCurve(missionAutocrossCurve) },
  { id: 'mission-brake', name: 'Brake · straight corridor', cones: brakeCones(), obstacles: [{ id: 'O1', x: 36, y: 0, radius: 1.85 }], start: baseVehicle(2, 0, 0) },
];

export function clonePreset(id: string): CoursePreset {
  const preset = [...COURSE_PRESETS, ...MISSION_DEMO_PRESETS].find((item) => item.id === id) ?? COURSE_PRESETS[0];
  return {
    ...preset,
    cones: preset.cones.map((cone) => ({ ...cone })),
    obstacles: preset.obstacles.map((obstacle) => ({ ...obstacle })),
    start: { ...preset.start },
  };
}
