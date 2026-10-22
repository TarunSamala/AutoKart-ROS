import Delaunator from 'delaunator';
import type { Candidate, Cone, Edge, Gate, Point } from './schema';

const distance = (a: Point, b: Point) => Math.hypot(a.x_m - b.x_m, a.y_m - b.y_m);
const key = (a: Cone, b: Cone) => [a.id, b.id].sort().join(':');

/** Least-squares circle fit from observed cone coordinates; no course truth input. */
export function fitCircle(points: Point[]): { center: Point; radius_m: number } | null {
  if (points.length < 3) return null;
  let sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0, sz = 0, sxz = 0, syz = 0;
  for (const p of points) {
    const x = p.x_m, y = p.y_m, z = x * x + y * y;
    sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y; sz += z; sxz += x * z; syz += y * z;
  }
  const matrix = [[2 * sxx, 2 * sxy, sx, sxz], [2 * sxy, 2 * syy, sy, syz], [2 * sx, 2 * sy, points.length, sz]];
  for (let col = 0; col < 3; col++) {
    let pivot = col;
    for (let row = col + 1; row < 3; row++) if (Math.abs(matrix[row][col]) > Math.abs(matrix[pivot][col])) pivot = row;
    if (Math.abs(matrix[pivot][col]) < 1e-8) return null;
    [matrix[col], matrix[pivot]] = [matrix[pivot], matrix[col]];
    const scale = matrix[col][col];
    for (let k = col; k < 4; k++) matrix[col][k] /= scale;
    for (let row = 0; row < 3; row++) if (row !== col) { const factor = matrix[row][col]; for (let k = col; k < 4; k++) matrix[row][k] -= factor * matrix[col][k]; }
  }
  const cx = matrix[0][3], cy = matrix[1][3];
  const radius_m = points.reduce((sum, p) => sum + Math.hypot(p.x_m - cx, p.y_m - cy), 0) / points.length;
  return Number.isFinite(radius_m) ? { center: { x_m: cx, y_m: cy }, radius_m } : null;
}

export function observeCones(cones: Cone[], vehicle: Point & { yaw_rad: number }, range: number, fov: number, dropout = 0, seed = 0, noise = 0): Cone[] {
  return cones.filter((cone) => {
    const dx = cone.x_m - vehicle.x_m, dy = cone.y_m - vehicle.y_m;
    const bearing = Math.atan2(Math.sin(Math.atan2(dy, dx) - vehicle.yaw_rad), Math.cos(Math.atan2(dy, dx) - vehicle.yaw_rad));
    const hash = ((cone.id.split('').reduce((a, c) => a * 31 + c.charCodeAt(0), seed) >>> 0) % 1000) / 1000;
    return Math.hypot(dx, dy) <= range && Math.abs(bearing) <= fov / 2 && hash >= dropout;
  }).map((cone) => {
    const dx = cone.x_m - vehicle.x_m, dy = cone.y_m - vehicle.y_m;
    const hash = cone.id.split('').reduce((a, c) => Math.imul(a, 33) + c.charCodeAt(0), seed) >>> 0;
    const nx = noise * Math.sin(hash * 0.001 + seed * 0.37), ny = noise * Math.cos(hash * 0.0017 + seed * 0.29);
    return { ...cone, x_m: cone.x_m + nx, y_m: cone.y_m + ny, range_m: Math.hypot(dx, dy), bearing_rad: Math.atan2(Math.sin(Math.atan2(dy, dx) - vehicle.yaw_rad), Math.cos(Math.atan2(dy, dx) - vehicle.yaw_rad)) };
  });
}

export function delaunayFromCones(cones: Cone[]): { edges: Edge[]; triangles: [string, string, string][] } {
  if (cones.length < 3) return { edges: [], triangles: [] };
  const triangulation = Delaunator.from(cones, (p) => p.x_m, (p) => p.y_m);
  const edges = new Map<string, Edge>();
  const triangles: [string, string, string][] = [];
  for (let i = 0; i < triangulation.triangles.length; i += 3) {
    const indices = [triangulation.triangles[i], triangulation.triangles[i + 1], triangulation.triangles[i + 2]];
    const id = `T${triangles.length + 1}`;
    triangles.push(indices.map((index) => cones[index].id) as [string, string, string]);
    for (const [aIndex, bIndex] of [[0, 1], [1, 2], [2, 0]]) {
      const a = cones[indices[aIndex]], b = cones[indices[bIndex]], edgeKey = key(a, b);
      const prior = edges.get(edgeKey);
      if (prior) prior.triangle_ids!.push(id);
      else edges.set(edgeKey, { id: `E${edges.size + 1}`, a: a.id, b: b.id, triangle_ids: [id] });
    }
  }
  return { edges: [...edges.values()], triangles };
}

/** The closed diameter disk must contain no third observed cone (within numerical tolerance). */
export function gabrielFilter(edges: Edge[], cones: Cone[]): Edge[] {
  const byId = new Map(cones.map((cone) => [cone.id, cone]));
  return edges.map((edge) => {
    const a = byId.get(edge.a), b = byId.get(edge.b);
    if (!a || !b) return { ...edge, accepted: false, reason: 'MISSING ENDPOINT' };
    const center = { x_m: (a.x_m + b.x_m) / 2, y_m: (a.y_m + b.y_m) / 2 };
    const radius = distance(a, b) / 2;
    const blocker = cones.find((cone) => cone.id !== a.id && cone.id !== b.id && distance(cone, center) < radius - 1e-7);
    return { ...edge, accepted: !blocker, reason: blocker ? `${blocker.id} INSIDE DIAMETER CIRCLE` : 'EMPTY DIAMETER CIRCLE', blocking_cone_id: blocker?.id, radius_m: radius };
  });
}

export function gatesFromGabriel(edges: Edge[], cones: Cone[], vehicle: Point & { yaw_rad: number }): Gate[] {
  const byId = new Map(cones.map((cone) => [cone.id, cone]));
  return edges.filter((edge) => edge.accepted).flatMap((edge) => {
    const a = byId.get(edge.a), b = byId.get(edge.b);
    if (!a || !b || !['BLUE', 'YELLOW'].includes(a.class) || !['BLUE', 'YELLOW'].includes(b.class) || a.class === b.class) return [];
    const width = distance(a, b);
    const midpoint = { x_m: (a.x_m + b.x_m) / 2, y_m: (a.y_m + b.y_m) / 2 };
    const forward = (midpoint.x_m - vehicle.x_m) * Math.cos(vehicle.yaw_rad) + (midpoint.y_m - vehicle.y_m) * Math.sin(vehicle.yaw_rad);
    return [{ id: `G${edge.id.slice(1)}`, source_edge_id: edge.id, midpoint, width_m: width, heading_rad: Math.atan2(b.y_m - a.y_m, b.x_m - a.x_m) + Math.PI / 2, confidence: Math.min(a.confidence, b.confidence), valid: width >= 2.1 && width <= 6.5 && forward > -2, reason: width < 2.1 || width > 6.5 ? 'WIDTH OUTSIDE CORRIDOR' : forward <= -2 ? 'BEHIND VEHICLE' : 'VALID CROSS-TRACK GATE' }];
  });
}

export function candidatePaths(gates: Gate[], vehicle: Point & { yaw_rad: number }, preferred?: Point, loop?: { center: Point; direction: -1 | 1; radius: number }): Candidate[] {
  const forward = (point: Point) => (point.x_m - vehicle.x_m) * Math.cos(vehicle.yaw_rad) + (point.y_m - vehicle.y_m) * Math.sin(vehicle.yaw_rad);
  const currentAngle = loop ? Math.atan2(vehicle.y_m - loop.center.y_m, vehicle.x_m - loop.center.x_m) : 0;
  const progress = (point: Point) => {
    if (!loop) return 0;
    const angle = Math.atan2(point.y_m - loop.center.y_m, point.x_m - loop.center.x_m);
    const raw = loop.direction * (angle - currentAngle);
    return (raw + Math.PI * 4) % (Math.PI * 2);
  };
  const valid = gates.filter((gate) => gate.valid && (loop ? progress(gate.midpoint) > 0.08 && progress(gate.midpoint) < 1.05 && Math.abs(distance(gate.midpoint, loop.center) - loop.radius) < 2.5 : forward(gate.midpoint) > 0.25));
  const ranked = valid.map((gate) => {
    const dist = distance(vehicle, gate.midpoint);
    const heading = Math.atan2(gate.midpoint.y_m - vehicle.y_m, gate.midpoint.x_m - vehicle.x_m);
    const angle = Math.abs(Math.atan2(Math.sin(heading - vehicle.yaw_rad), Math.cos(heading - vehicle.yaw_rad)));
    const mission = preferred ? distance(gate.midpoint, preferred) * 0.08 : 0;
    const cost = loop ? Math.abs(progress(gate.midpoint) - 0.34) * 3.5 + Math.abs(distance(gate.midpoint, loop.center) - loop.radius) * 2 + 0.1 * dist + angle * 0.3 : 0.25 * dist + 2.8 * angle + mission + Math.abs(gate.width_m - 4.2) * 0.1;
    return { gate, cost, angle, mission };
  }).sort((a, b) => a.cost - b.cost).slice(0, 3);
  return ranked.map((item, index) => ({ id: `P${index + 1}`, points: [{ x_m: vehicle.x_m, y_m: vehicle.y_m }, item.gate.midpoint], costs: { distance: distance(vehicle, item.gate.midpoint), heading: item.angle, mission: item.mission }, total_cost: item.cost, rank: index + 1, selected: index === 0, reason: `GATE ${item.gate.id}` }));
}
