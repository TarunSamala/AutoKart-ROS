import type { Cone, Point } from './schema';

export type ScenarioName = 'SKIDPAD' | 'AUTOCROSS' | 'BRAKE';
export type LabSettings = { seed: number; width: number; coneSpacing: number; complexity: number; dropout: number; noise: number; length: number; hairpins: number; sBends: number; slaloms: number; skidpadRadius: number; entryLength: number; exitLength: number; junctionLength: number };
export const DEFAULT_LAB_SETTINGS: LabSettings = { seed: 43172, width: 3, coneSpacing: 1, complexity: 0.8, dropout: 0, noise: 0, length: 220, hairpins: 2, sBends: 4, slaloms: 3, skidpadRadius: 8, entryLength: 14, exitLength: 14, junctionLength: 2.4 };
export type LabScenario = { name: ScenarioName; cones: Cone[]; guide: Point[]; start: Point & { heading: number }; obstacle?: Point & { id: string; radius_m: number }; markers: Array<Point & { id: string; label: string }> };
const point = (x: number, y: number): Point => ({ x_m: x, y_m: y });
const cone = (id: string, side: string, p: Point): Cone => ({ id, class: side, confidence: 1, ...p });

function resample(points: Point[], spacing: number): Point[] {
  const output: Point[] = [points[0]];
  let remaining = spacing;
  for (let i = 1; i < points.length; i++) {
    let a = points[i - 1], b = points[i];
    let length = Math.hypot(b.x_m - a.x_m, b.y_m - a.y_m);
    while (length >= remaining) {
      const t = remaining / length;
      a = point(a.x_m + (b.x_m - a.x_m) * t, a.y_m + (b.y_m - a.y_m) * t);
      output.push(a); length = Math.hypot(b.x_m - a.x_m, b.y_m - a.y_m); remaining = spacing;
    }
    remaining -= length;
  }
  return output;
}

function boundaryCones(guide: Point[], spacing: number, widthAt: (i: number) => number): Cone[] {
  const samples = resample(guide, spacing);
  return samples.flatMap((p, i) => {
    const previous = samples[Math.max(0, i - 1)], next = samples[Math.min(samples.length - 1, i + 1)];
    const angle = Math.atan2(next.y_m - previous.y_m, next.x_m - previous.x_m), w = widthAt(i) / 2;
    const nx = -Math.sin(angle) * w, ny = Math.cos(angle) * w;
    return [cone(`B${i}`, 'BLUE', point(p.x_m + nx, p.y_m + ny)), cone(`Y${i}`, 'YELLOW', point(p.x_m - nx, p.y_m - ny))];
  });
}

function contourSegments(sdf: (x: number, y: number) => number, bounds: { minX: number; maxX: number; minY: number; maxY: number }, step = 0.12): Point[][] {
  const nx = Math.ceil((bounds.maxX - bounds.minX) / step), ny = Math.ceil((bounds.maxY - bounds.minY) / step);
  const values = Array.from({ length: nx + 1 }, (_, i) => Array.from({ length: ny + 1 }, (_, j) => sdf(bounds.minX + i * step, bounds.minY + j * step)));
  const links = new Map<string, Set<string>>(), vertices = new Map<string, Point>();
  const hash = (p: Point) => Math.round(p.x_m * 10000) + ':' + Math.round(p.y_m * 10000);
  const add = (a: Point, b: Point) => {
    const ka = hash(a), kb = hash(b);
    if (ka === kb) return;
    vertices.set(ka, a); vertices.set(kb, b);
    if (!links.has(ka)) links.set(ka, new Set());
    if (!links.has(kb)) links.set(kb, new Set());
    links.get(ka)!.add(kb); links.get(kb)!.add(ka);
  };
  for (let i = 0; i < nx; i++) for (let j = 0; j < ny; j++) {
    const x = bounds.minX + i * step, y = bounds.minY + j * step;
    const corners = [point(x, y), point(x + step, y), point(x + step, y + step), point(x, y + step)];
    const v = [values[i][j], values[i + 1][j], values[i + 1][j + 1], values[i][j + 1]];
    const hits: Point[] = [];
    for (let edge = 0; edge < 4; edge++) {
      const next = (edge + 1) % 4;
      if ((v[edge] < 0) === (v[next] < 0)) continue;
      const t = v[edge] / (v[edge] - v[next]);
      hits.push(point(corners[edge].x_m + t * (corners[next].x_m - corners[edge].x_m), corners[edge].y_m + t * (corners[next].y_m - corners[edge].y_m)));
    }
    if (hits.length === 2) add(hits[0], hits[1]);
    else if (hits.length === 4) { add(hits[0], hits[1]); add(hits[2], hits[3]); }
  }
  const contours: Point[][] = [], visited = new Set<string>();
  for (const start of links.keys()) {
    if (visited.has(start)) continue;
    const contour: Point[] = []; let previous = '', current = start;
    for (let count = 0; count < links.size + 1; count++) {
      contour.push(vertices.get(current)!); visited.add(current);
      const next = [...(links.get(current) ?? [])].find((item) => item !== previous && (item === start || !visited.has(item)));
      if (!next || next === start) break;
      previous = current; current = next;
    }
    if (contour.length > 12) contours.push(contour);
  }
  return contours;
}

function skidpad(settings: LabSettings): LabScenario {
  const r = settings.skidpadRadius, half = settings.width / 2, entry = settings.entryLength, exit = settings.exitLength;
  const guide: Point[] = [];
  for (let y = -entry; y <= 0; y += 0.4) guide.push(point(0, y));
  for (let i = 1; i <= 192; i++) { const a = Math.PI - i / 192 * Math.PI * 4; guide.push(point(r + r * Math.cos(a), r * Math.sin(a))); }
  for (let i = 1; i <= 192; i++) { const a = i / 192 * Math.PI * 4; guide.push(point(-r + r * Math.cos(a), r * Math.sin(a))); }
  for (let y = 0.4; y <= exit; y += 0.4) guide.push(point(0, y));
  const distanceToLine = (x: number, y: number) => Math.hypot(x, y - Math.max(-entry, Math.min(exit, y)));
  const sdf = (x: number, y: number) => Math.min(Math.abs(Math.hypot(x + r, y) - r), Math.abs(Math.hypot(x - r, y) - r), distanceToLine(x, y)) - half;
  const contours = contourSegments(sdf, { minX: -2 * r - half - 2, maxX: 2 * r + half + 2, minY: -entry - half - 2, maxY: exit + half + 2 });
  const cones: Cone[] = [];
  for (const contour of contours) {
    const closed = [...contour, contour[0]];
    for (const p of resample(closed, settings.coneSpacing)) {
      if (Math.hypot(p.x_m, p.y_m) < settings.junctionLength) continue;
      if (cones.some((existing) => Math.hypot(existing.x_m - p.x_m, existing.y_m - p.y_m) < Math.min(0.6, settings.coneSpacing * 0.6))) continue;
      const choices = [
        { d: Math.abs(Math.hypot(p.x_m + r, p.y_m) - r), cx: -r, dir: 1 },
        { d: Math.abs(Math.hypot(p.x_m - r, p.y_m) - r), cx: r, dir: -1 },
        { d: distanceToLine(p.x_m, p.y_m), cx: 0, dir: 0 },
      ].sort((a, b) => a.d - b.d);
      const nearest = choices[0];
      let signedLeft: number;
      if (nearest.dir === 0) signedLeft = -p.x_m;
      else {
        const a = Math.atan2(p.y_m, p.x_m - nearest.cx);
        const centerline = point(nearest.cx + r * Math.cos(a), r * Math.sin(a));
        const tangent = a + nearest.dir * Math.PI / 2;
        signedLeft = (p.x_m - centerline.x_m) * -Math.sin(tangent) + (p.y_m - centerline.y_m) * Math.cos(tangent);
      }
      const side = signedLeft >= 0 ? 'BLUE' : 'YELLOW';
      cones.push(cone((side === 'BLUE' ? 'B' : 'Y') + cones.length, side, p));
    }
  }
  const special = [cone('START_B', 'ORANGE', point(-half, -entry)), cone('START_Y', 'ORANGE', point(half, -entry)), cone('FINISH_B', 'ORANGE', point(-half, exit)), cone('FINISH_Y', 'ORANGE', point(half, exit))];
  const left = cones.filter((item) => item.x_m < -0.15 && item.y_m > -entry + 0.5 && item.y_m < exit - 0.5);
  const symmetric = [...left, ...left.map((item, index) => cone('M' + index, item.class === 'BLUE' ? 'YELLOW' : 'BLUE', point(-item.x_m, item.y_m)))];
  const physical = symmetric.filter((item) => special.every((marker) => Math.hypot(marker.x_m - item.x_m, marker.y_m - item.y_m) > 0.7));
  return { name: 'SKIDPAD', cones: [...physical, ...special], guide, start: { ...point(0, -entry - 3), heading: Math.PI / 2 }, markers: [] };
}

function autocross(settings: LabSettings): LabScenario {
  // The seeded centreline is a complete open course: straight, repeated S bends,
  // slaloms, a 180-degree hairpin, a long corner and narrowing/widening zones.
  let state = settings.seed >>> 0;
  const random = () => { state = (Math.imul(state, 1664525) + 1013904223) >>> 0; return state / 4294967296; };
  const phase = random() * Math.PI * 2;
  const route: Point[] = [];
  const length = Math.max(170, settings.length);
  for (let x = 0; x <= length * 0.62; x += 0.4) {
    const straight = x < 28 ? 0 : 1;
    const s = Math.sin((x - 28) * (settings.sBends + 2) * Math.PI / (length * 0.62) + phase) * 5.3 * straight * settings.complexity;
    const slalom = x > 50 && x < length * 0.52 ? Math.sin((x - 50) * (settings.slaloms + 2) * Math.PI * 2 / (length * 0.42)) * 1.8 * settings.complexity : 0;
    route.push(point(x, s + slalom));
  }
  const end = route.at(-1)!;
  for (let i = 1; i <= 110; i++) { const a = -Math.PI / 2 + Math.PI * i / 110; route.push(point(end.x_m + 9 * Math.cos(a), end.y_m + 9 + 9 * Math.sin(a))); }
  for (let x = end.x_m - 0.4; x >= length * 0.1; x -= 0.4) {
    const u = end.x_m - x;
    const bend = Math.sin(u * 0.08 + phase) * 4 * settings.complexity + Math.sin(u * 0.17) * 1.3;
    route.push(point(x, end.y_m + 18 + bend));
  }
  // Additional U turns make the builder's hairpin count change the route,
  // while alternating lanes keep the course open and non-self-intersecting.
  for (let hairpin = 1; hairpin < Math.max(1, settings.hairpins); hairpin++) {
    const start = route.at(-1)!;
    const heading = hairpin % 2 ? -1 : 1;
    for (let i = 1; i <= 110; i++) {
      const a = -Math.PI / 2 + heading * Math.PI * i / 110;
      route.push(point(start.x_m + 9 * Math.cos(a), start.y_m + 9 + 9 * Math.sin(a)));
    }
    const turnEnd = route.at(-1)!;
    const destination = heading < 0 ? length * 0.62 : length * 0.1;
    const direction = destination > turnEnd.x_m ? 1 : -1;
    for (let x = turnEnd.x_m + direction * 0.4; direction * x <= direction * destination; x += direction * 0.4) {
      route.push(point(x, turnEnd.y_m + Math.sin(Math.abs(x - turnEnd.x_m) * 0.07 + phase + hairpin) * 3.5 * settings.complexity));
    }
  }
  const guide = resample(route, 0.55);
  const cones = boundaryCones(guide, settings.coneSpacing, (i) => {
    const fraction = i / Math.max(1, guide.length);
    return settings.width * (fraction > 0.35 && fraction < 0.46 ? 0.75 : fraction > 0.7 && fraction < 0.82 ? 1.35 : 1);
  });
  const start = guide[0], next = guide[1], finish = guide.at(-1)!;
  cones.push(cone('START_L', 'ORANGE', point(start.x_m, start.y_m + settings.width / 2)), cone('START_R', 'ORANGE', point(start.x_m, start.y_m - settings.width / 2)));
  return { name: 'AUTOCROSS', cones, guide, start: { ...start, heading: Math.atan2(next.y_m - start.y_m, next.x_m - start.x_m) }, markers: [{ ...start, id: 'start', label: 'START' }, { ...finish, id: 'finish', label: 'FINISH' }] };
}

function brake(settings: LabSettings): LabScenario {
  const guide = Array.from({ length: 161 }, (_, i) => point(i * 0.5, 0));
  const cones = boundaryCones(guide, settings.coneSpacing, () => Math.max(4.4, settings.width));
  cones.push(cone('START_L', 'ORANGE', point(0, 2.2)), cone('START_R', 'ORANGE', point(0, -2.2)));
  return { name: 'BRAKE', cones, guide, start: { ...point(2, 0), heading: 0 }, obstacle: { ...point(37, 0), id: 'OBSTACLE', radius_m: 1.4 }, markers: [{ ...point(0, 0), id: 'start', label: 'START' }, { ...point(37, 0), id: 'obstacle', label: 'OBSTACLE' }] };
}

export function generateScenario(name: ScenarioName, settings: LabSettings = DEFAULT_LAB_SETTINGS): LabScenario {
  if (name === 'SKIDPAD') return skidpad(settings);
  if (name === 'BRAKE') return brake(settings);
  return autocross(settings);
}
