import Delaunator from 'delaunator';
import type { Candidate, Cone, Edge, Gate, MissionFrame, Point } from './schema';

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

// ===========================================================================
// MY_TRACK: ported from kart/kart_sim/my_track_algorithm.py (gates, obstacle
// classification, finish detection, swept-footprint detour admission) and
// kart/kart_sim/my_track_mission.py (the mission FSM, detour lock, finish
// latch, obstacle-id hysteresis). All body-frame points below are (x_m
// forward, y_m left) referenced to the rear axle -- same convention as the
// Python module. The Python side has a 9-state FSM; this is an explicit
// simplification to 5 states (READY, RUN, DETOUR, FINISH, STOPPED) per the
// task's own instruction. OBSTACLE_ASSESS/CONTROLLED_STOP/RECENTER/PATH_LOST
// are folded into RUN (with a capped target speed when an obstacle cluster
// cannot be admitted for a detour) or STOPPED (safety fallback), rather than
// surfaced as their own states.
// ===========================================================================

export type MyTrackBodyPoint = { x_m: number; y_m: number; class: string; id: string };
export type MTGate = { left: Point; right: Point; midpoint: Point; widthM: number };
export type MyTrackState = 'READY' | 'RUN' | 'DETOUR' | 'FINISH' | 'STOPPED';
export interface MyTrackGeometry { wheelbaseM: number; frontExtentM: number; rearOverhangM: number; halfWidthM: number }

/** Calibrated values, NOT the originally-specified ones -- copied verbatim
 * from the real, tuned `AlgorithmConfig` in kart/kart_sim/my_track_algorithm.py
 * (see that file's comments for why, e.g. finish_pair_max_forward_gap_m was
 * recalibrated from a spec'd 0.7 to 0.5 against real map-blending behaviour). */
export const MY_TRACK_CONFIG = {
  gateWidthMinM: 2.0, gateWidthMaxM: 3.8, gateSkewDeg: 45,
  obstacleLinkM: 0.85, obstacleMinCones: 4, obstacleMinLateralRangeM: 0.28,
  clusterLinkM: 1.0, finishMinOdometryM: 40.0,
  finishPairMinM: 0.1, finishPairMaxM: 0.6, finishPairMaxForwardGapM: 0.5,
  nominalLaneWidthM: 3.0, fallbackGateMinExistingDistM: 0.8, centrelineMinGates: 2,
  coneRadiusM: 0.15, footprintMarginM: 0.20,
  detourRampLengthsM: [4.0, 3.0, 2.4, 2.0] as const, detourLeadStepM: 0.25,
  bodyWindowXMinM: -2.5, bodyWindowXMaxM: 12.0, bodyWindowYM: 7.0,
  obstacleMemoryS: 3.0,
} as const;

const myTrackSide = (cls: string, y_m: number): 'L' | 'R' => (cls === 'BLUE' ? 'L' : cls === 'YELLOW' ? 'R' : y_m > 0 ? 'L' : 'R');
const median = (values: number[]): number => { const sorted = [...values].sort((a, b) => a - b); const mid = Math.floor(sorted.length / 2); return sorted.length % 2 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) / 2; };
const smoothstep = (t: number): number => { const tc = Math.min(1, Math.max(0, t)); return tc * tc * (3 - 2 * tc); };

/** Converts world-frame cones into the kart's body frame and applies the
 * same forward/lateral observation window as body_frame_points() in the
 * Python module. */
export function myTrackBodyFramePoints(cones: Cone[], pose: { x_m: number; y_m: number; yaw_rad: number }): MyTrackBodyPoint[] {
  const { x_m: xe, y_m: ye, yaw_rad: pe } = pose;
  const c = Math.cos(pe), s = Math.sin(pe);
  const points: MyTrackBodyPoint[] = [];
  for (const cone of cones) {
    const dx = cone.x_m - xe, dy = cone.y_m - ye;
    const xv = c * dx + s * dy, yv = -s * dx + c * dy;
    if (xv > MY_TRACK_CONFIG.bodyWindowXMinM && xv < MY_TRACK_CONFIG.bodyWindowXMaxM && Math.abs(yv) < MY_TRACK_CONFIG.bodyWindowYM) {
      points.push({ x_m: xv, y_m: yv, class: cone.class, id: cone.id });
    }
  }
  return points;
}

/** Delaunay candidate edges between opposite-side boundary cones, filtered by
 * width and skew. Reuses delaunayFromCones() (Delaunator) rather than
 * reimplementing triangulation -- the third parallel implementation this
 * codebase already has for Gabriel/skidpad gates is NOT forked again here. */
export function buildMyTrackGates(boundary: MyTrackBodyPoint[]): MTGate[] {
  if (boundary.length < 3) return [];
  const cones: Cone[] = boundary.map((p) => ({ id: p.id, x_m: p.x_m, y_m: p.y_m, class: p.class, confidence: 1 }));
  const { edges } = delaunayFromCones(cones);
  const byId = new Map(boundary.map((p) => [p.id, p]));
  const gates: MTGate[] = [];
  for (const edge of edges) {
    const a = byId.get(edge.a), b = byId.get(edge.b);
    if (!a || !b) continue;
    if (myTrackSide(a.class, a.y_m) === myTrackSide(b.class, b.y_m)) continue;
    const dx = a.x_m - b.x_m, dy = a.y_m - b.y_m;
    const widthM = Math.hypot(dx, dy);
    if (widthM < MY_TRACK_CONFIG.gateWidthMinM || widthM > MY_TRACK_CONFIG.gateWidthMaxM) continue;
    const skewDeg = (Math.atan2(Math.abs(dx), Math.abs(dy)) * 180) / Math.PI;
    if (skewDeg > MY_TRACK_CONFIG.gateSkewDeg) continue;
    const [left, right] = a.y_m > b.y_m ? [a, b] : [b, a];
    gates.push({ left: { x_m: left.x_m, y_m: left.y_m }, right: { x_m: right.x_m, y_m: right.y_m }, midpoint: { x_m: (left.x_m + right.x_m) / 2, y_m: (left.y_m + right.y_m) / 2 }, widthM });
  }
  return gates;
}

/** Single-side fallback gates for when one boundary is out of view (ported
 * from add_single_side_fallback_gates()). */
export function addMyTrackFallbackGates(boundary: MyTrackBodyPoint[], realGates: MTGate[]): MTGate[] {
  const sides = boundary.map((p) => myTrackSide(p.class, p.y_m));
  const extra: MTGate[] = [];
  for (const [sideName, sign] of [['L', -1], ['R', 1]] as const) {
    const indices = boundary.map((_, k) => k).filter((k) => sides[k] === sideName && boundary[k].x_m > -1.0).sort((j, k) => boundary[j].x_m - boundary[k].x_m);
    for (let position = 0; position < indices.length; position++) {
      const k = indices[position];
      const a = boundary[indices[Math.max(position - 1, 0)]];
      const b = boundary[indices[Math.min(position + 1, indices.length - 1)]];
      let tx = b.x_m - a.x_m, ty = b.y_m - a.y_m;
      const tangentLen = Math.hypot(tx, ty);
      if (tangentLen < 0.5 || tx <= 0) continue;
      tx /= tangentLen; ty /= tangentLen;
      const nx = -sign * ty, ny = sign * tx;
      const point = boundary[k];
      const midX = point.x_m + nx * 0.5 * MY_TRACK_CONFIG.nominalLaneWidthM;
      const midY = point.y_m + ny * 0.5 * MY_TRACK_CONFIG.nominalLaneWidthM;
      const existing = [...realGates.map((g) => g.midpoint), ...extra.map((g) => g.midpoint)];
      if (existing.some((m) => Math.hypot(midX - m.x_m, midY - m.y_m) <= MY_TRACK_CONFIG.fallbackGateMinExistingDistM)) continue;
      const offX = nx * MY_TRACK_CONFIG.nominalLaneWidthM, offY = ny * MY_TRACK_CONFIG.nominalLaneWidthM;
      const [leftXY, rightXY] = sideName === 'L' ? [[point.x_m + offX / 2, point.y_m + offY / 2], [point.x_m - offX / 2, point.y_m - offY / 2]] : [[point.x_m - offX / 2, point.y_m - offY / 2], [point.x_m + offX / 2, point.y_m + offY / 2]];
      extra.push({ left: { x_m: leftXY[0], y_m: leftXY[1] }, right: { x_m: rightXY[0], y_m: rightXY[1] }, midpoint: { x_m: midX, y_m: midY }, widthM: MY_TRACK_CONFIG.nominalLaneWidthM });
    }
  }
  return extra;
}

function solveLinearSystem(matrix: number[][], vector: number[]): number[] {
  const n = vector.length;
  const augmented = matrix.map((row, i) => [...row, vector[i]]);
  for (let col = 0; col < n; col++) {
    let pivot = col;
    for (let row = col + 1; row < n; row++) if (Math.abs(augmented[row][col]) > Math.abs(augmented[pivot][col])) pivot = row;
    if (Math.abs(augmented[pivot][col]) < 1e-10) continue;
    [augmented[col], augmented[pivot]] = [augmented[pivot], augmented[col]];
    const scale = augmented[col][col];
    for (let k = col; k <= n; k++) augmented[col][k] /= scale;
    for (let row = 0; row < n; row++) if (row !== col) { const factor = augmented[row][col]; for (let k = col; k <= n; k++) augmented[row][k] -= factor * augmented[col][k]; }
  }
  return augmented.map((row) => row[n]);
}

/** Weighted polynomial fit, numpy.polyfit-compatible coefficient order
 * (highest degree first). */
function polyfitWeighted(xs: number[], ys: number[], weights: number[], degree: number): number[] {
  const n = degree + 1;
  const ata = Array.from({ length: n }, () => new Array(n).fill(0));
  const atb = new Array(n).fill(0);
  for (let i = 0; i < xs.length; i++) {
    const w = weights[i];
    const powers = new Array(n);
    let p = 1;
    for (let k = n - 1; k >= 0; k--) { powers[k] = p; p *= xs[i]; }
    for (let r = 0; r < n; r++) {
      atb[r] += w * powers[r] * ys[i];
      for (let c = 0; c < n; c++) ata[r][c] += w * powers[r] * powers[c];
    }
  }
  return solveLinearSystem(ata, atb);
}

function evalPoly(coeffs: number[], x: number): number {
  let result = 0;
  for (const c of coeffs) result = result * x + c;
  return result;
}

function polyDerivative(coeffs: number[]): number[] {
  const n = coeffs.length - 1;
  if (n <= 0) return [0];
  return coeffs.slice(0, n).map((c, i) => c * (n - i));
}

export type MyTrackFit = { coeffs: number[]; xLo: number; xHi: number };

/** Robust IRLS polynomial centreline fit (degree 1-3 by gate count, 5
 * reweighting iterations, Tukey-like weights) -- ported from
 * fit_centreline(). */
export function fitMyTrackCentreline(gates: MTGate[]): MyTrackFit | null {
  if (gates.length < 2) return null;
  const midpoints = [...gates].map((g) => g.midpoint).sort((a, b) => a.x_m - b.x_m);
  const xs = midpoints.map((m) => m.x_m), ys = midpoints.map((m) => m.y_m);
  const degree = midpoints.length >= 8 ? 3 : midpoints.length >= 5 ? 2 : 1;
  let weights = new Array(midpoints.length).fill(1);
  let coeffs = new Array(degree + 1).fill(0);
  for (let iter = 0; iter < 5; iter++) {
    coeffs = polyfitWeighted(xs, ys, weights, degree);
    weights = xs.map((x, i) => 1 / Math.max(1, Math.abs(evalPoly(coeffs, x) - ys[i]) / 0.15));
  }
  return { coeffs, xLo: xs[0], xHi: xs[xs.length - 1] };
}

/** Clamped evaluation with linear extrapolation outside the fitted gate
 * range, ported from evaluate_centreline(). */
export function evaluateMyTrackCentreline(fit: MyTrackFit, x: number): number {
  const { coeffs, xLo, xHi } = fit;
  const derivative = polyDerivative(coeffs);
  const xClamped = Math.min(xHi, Math.max(xLo, x));
  const y = evalPoly(coeffs, xClamped);
  const slopeLo = Math.min(0.6, Math.max(-0.6, evalPoly(derivative, xLo)));
  const slopeHi = Math.min(0.6, Math.max(-0.6, evalPoly(derivative, xHi)));
  if (x < xLo) return y + slopeLo * (x - xLo);
  if (x > xHi) return y + slopeHi * (x - xHi);
  return y;
}

/** Connected orange components (flood-fill, link <= obstacleLinkM) are an
 * obstacle only with >= obstacleMinCones cones AND lateral spread >=
 * obstacleMinLateralRangeM -- a straight orange boundary wall segment is
 * collinear (small lateral spread) and stays a boundary. Ported from
 * classify_obstacles(). */
export function classifyMyTrackObstacles(points: MyTrackBodyPoint[], exclude: Set<number>): Set<number> {
  const orange = points.map((_, i) => i).filter((i) => points[i].class === 'ORANGE' && !exclude.has(i));
  const seen = new Set<number>();
  const obstacleIndices = new Set<number>();
  for (const start of orange) {
    if (seen.has(start)) continue;
    const component = [start], frontier = [start];
    seen.add(start);
    while (frontier.length) {
      const a = frontier.pop()!;
      for (const b of orange) {
        if (seen.has(b)) continue;
        if (Math.hypot(points[a].x_m - points[b].x_m, points[a].y_m - points[b].y_m) <= MY_TRACK_CONFIG.obstacleLinkM) {
          seen.add(b); component.push(b); frontier.push(b);
        }
      }
    }
    const ys = component.map((i) => points[i].y_m);
    const lateralRangeM = Math.max(...ys) - Math.min(...ys);
    if (component.length >= MY_TRACK_CONFIG.obstacleMinCones && lateralRangeM >= MY_TRACK_CONFIG.obstacleMinLateralRangeM) {
      component.forEach((i) => obstacleIndices.add(i));
    }
  }
  return obstacleIndices;
}

/** Ported from cluster_indices(): groups already-classified obstacle indices
 * into connected clusters for per-cluster detour admission. */
export function clusterMyTrackIndices(indices: Set<number>, points: MyTrackBodyPoint[], linkM: number): number[][] {
  const remaining = new Set(indices);
  const clusters: number[][] = [];
  while (remaining.size) {
    const seed = remaining.values().next().value as number;
    remaining.delete(seed);
    const component = [seed], frontier = [seed];
    while (frontier.length) {
      const a = frontier.pop()!;
      for (const b of [...remaining]) {
        if (Math.hypot(points[a].x_m - points[b].x_m, points[a].y_m - points[b].y_m) <= linkM) {
          remaining.delete(b); component.push(b); frontier.push(b);
        }
      }
    }
    clusters.push(component);
  }
  return clusters;
}

/** Finish-line detection: an orange pair finishPairMinM-finishPairMaxM apart
 * laterally, near the lane edge, ahead of the kart, only evaluated after
 * finishMinOdometryM of progress. Ported from detect_finish(). */
export function detectMyTrackFinish(points: MyTrackBodyPoint[], odometryM: number): { finishXBody: number | null; finishIndices: Set<number> } {
  if (odometryM <= MY_TRACK_CONFIG.finishMinOdometryM) return { finishXBody: null, finishIndices: new Set() };
  const orange = points.map((_, i) => i).filter((i) => points[i].class === 'ORANGE');
  const candidateX: number[] = [];
  const finishIndices = new Set<number>();
  for (let ai = 0; ai < orange.length; ai++) {
    for (let bi = ai + 1; bi < orange.length; bi++) {
      const a = orange[ai], b = orange[bi];
      const xa = points[a].x_m, ya = points[a].y_m, xb = points[b].x_m, yb = points[b].y_m;
      const lateralGapM = Math.abs(ya - yb), forwardGapM = Math.abs(xa - xb);
      if (lateralGapM >= MY_TRACK_CONFIG.finishPairMinM && lateralGapM <= MY_TRACK_CONFIG.finishPairMaxM && forwardGapM < MY_TRACK_CONFIG.finishPairMaxForwardGapM && xa > 0.5 && xb > 0.5 && Math.abs(ya + yb) / 2 > 1.1) {
        candidateX.push((xa + xb) / 2);
        finishIndices.add(a); finishIndices.add(b);
      }
    }
  }
  if (!candidateX.length) return { finishXBody: null, finishIndices: new Set() };
  return { finishXBody: candidateX.reduce((s, v) => s + v, 0) / candidateX.length, finishIndices };
}

/** Distance from a world point to the kart's footprint rectangle, referenced
 * to the rear axle (0 if the point is inside). Ported from
 * footprint_clearance_m(). */
export function myTrackFootprintClearanceM(pointX: number, pointY: number, kartX: number, kartY: number, kartYaw: number, rearOverhangM: number, frontExtentM: number, halfWidthM: number): number {
  const c = Math.cos(kartYaw), s = Math.sin(kartYaw);
  const dx = pointX - kartX, dy = pointY - kartY;
  const bx = c * dx + s * dy, by = -s * dx + c * dy;
  const ex = Math.max(-rearOverhangM - bx, bx - frontExtentM, 0);
  const ey = Math.max(Math.abs(by) - halfWidthM, 0);
  return Math.hypot(ex, ey);
}

/** Smoothstep-based ramp offset profile used for a detour, ported from
 * MissionController._ramp(). */
export function myTrackRampOffset(xs: number[], xMinM: number, xMaxM: number, leadM: number, rampM: number, heightM: number): number[] {
  const rampEndX = xMinM - leadM, rampStartX = rampEndX - rampM;
  const decelEndX = xMaxM + 0.4, decelFullX = decelEndX + rampM;
  return xs.map((x) => heightM * (smoothstep((x - rampStartX) / Math.max(rampEndX - rampStartX, 1e-6)) - smoothstep((x - decelEndX) / Math.max(decelFullX - decelEndX, 1e-6))));
}

function maxSpeedForStopDistance(distanceM: number, marginM: number, delayS = 0.3, decelMps2 = 3.2): number {
  const availableM = distanceM - marginM;
  if (availableM <= 0) return 0;
  return Math.max(0, -decelMps2 * delayS + Math.sqrt((decelMps2 * delayS) ** 2 + 2 * decelMps2 * availableM));
}

export interface MyTrackStepResult {
  state: MyTrackState;
  steeringRad: number;
  targetSpeedMps: number;
  brake: boolean;
  gates: number;
  obstacle: boolean;
  detour: boolean;
  finishDetected: boolean;
  lateralDeviationM: number | null;
  pathWorldXY: Point[];
  gatesList: MTGate[];
  blind: boolean;
}

/** MY_TRACK mission driver: a simplified 5-state FSM (READY, RUN, DETOUR,
 * FINISH, STOPPED) wired to the ported algorithm chain above. Carries
 * persistent state across frames the same way the Python MissionController
 * does -- detour locks (admitted once, reused rather than re-searched every
 * frame, see admission below) and the finish-line latch (same anti-flicker
 * principle) -- rather than re-deriving them fresh each call, which the
 * Python side traced to a real collision bug (see my_track_mission.py's
 * comments by DetourLock reuse and _obstacle_memory). */
export class MyTrackMission {
  state: MyTrackState = 'READY';
  private locks: Array<{ clusterCenterWorld: Point; rampLengthM: number; leadDistanceM: number }> = [];
  private finishWorldXY: Point | null = null;
  private obstacleMemory = new Map<string, number>();

  constructor(private geometry: MyTrackGeometry, private cruiseSpeedMps = 0.5, private blindCreepSpeedMps = 0.3) {}

  reset(): void {
    this.state = 'READY';
    this.locks = [];
    this.finishWorldXY = null;
    this.obstacleMemory.clear();
  }

  /** Number of currently-admitted detour locks (test/telemetry hook). */
  lockCount(): number { return this.locks.length; }

  step(bodyPoints: MyTrackBodyPoint[], pose: { x_m: number; y_m: number; yaw_rad: number }, speedMps: number, odometryM: number, timestampS: number): MyTrackStepResult {
    if (this.state === 'READY') this.state = 'RUN';
    if (this.state === 'STOPPED') {
      return { state: 'STOPPED', steeringRad: 0, targetSpeedMps: 0, brake: true, gates: 0, obstacle: false, detour: false, finishDetected: true, lateralDeviationM: null, pathWorldXY: [], gatesList: [], blind: false };
    }

    const { x_m: xe, y_m: ye, yaw_rad: pe } = pose;
    const c = Math.cos(pe), s = Math.sin(pe);
    const toWorld = (xb: number, yb: number): Point => ({ x_m: xe + c * xb - s * yb, y_m: ye + s * xb + c * yb });

    let { finishXBody, finishIndices } = bodyPoints.length >= 3 ? detectMyTrackFinish(bodyPoints, odometryM) : { finishXBody: null as number | null, finishIndices: new Set<number>() };
    if (finishXBody != null && this.finishWorldXY == null) this.finishWorldXY = toWorld(finishXBody, 0);
    if (this.finishWorldXY != null) {
      // Recompute the body-frame distance to the latched world point every
      // frame rather than re-detecting (same anti-flicker rationale as the
      // Python side).
      const fx = c * (this.finishWorldXY.x_m - xe) + s * (this.finishWorldXY.y_m - ye);
      const n = 20;
      const pathBody = Array.from({ length: n }, (_, i) => ({ x_m: (Math.max(fx, 0.5) * i) / (n - 1), y_m: 0 }));
      const pathWorld = pathBody.map((p) => toWorld(p.x_m, p.y_m));
      const stopGapM = fx - this.geometry.frontExtentM;
      this.state = stopGapM < 0.1 && speedMps < 0.06 ? 'STOPPED' : 'FINISH';
      const targetSpeedMps = this.state === 'STOPPED' ? 0 : Math.max(0, Math.min(this.cruiseSpeedMps, maxSpeedForStopDistance(stopGapM, 0.3)));
      return { state: this.state, steeringRad: 0, targetSpeedMps, brake: this.state === 'STOPPED', gates: bodyPoints.length, obstacle: false, detour: false, finishDetected: true, lateralDeviationM: 0, pathWorldXY: pathWorld, gatesList: [], blind: false };
    }

    if (bodyPoints.length < 3) return this._blind();

    let obstacleIndices = classifyMyTrackObstacles(bodyPoints, finishIndices);
    for (const i of obstacleIndices) this.obstacleMemory.set(bodyPoints[i].id, timestampS);
    for (const [id, t] of [...this.obstacleMemory]) if (timestampS - t > MY_TRACK_CONFIG.obstacleMemoryS) this.obstacleMemory.delete(id);
    bodyPoints.forEach((p, i) => {
      if (!obstacleIndices.has(i) && p.class === 'ORANGE' && !finishIndices.has(i) && this.obstacleMemory.has(p.id)) obstacleIndices.add(i);
    });

    const boundaryIndices = bodyPoints.map((_, i) => i).filter((i) => !obstacleIndices.has(i) && !finishIndices.has(i));
    if (boundaryIndices.length < 3) return this._blind();
    const boundary = boundaryIndices.map((i) => bodyPoints[i]);

    const realGates = buildMyTrackGates(boundary);
    const fallbackGates = addMyTrackFallbackGates(boundary, realGates);
    const allGates = [...realGates, ...fallbackGates];
    if (allGates.length < MY_TRACK_CONFIG.centrelineMinGates) return this._blind();

    const fit = fitMyTrackCentreline(allGates);
    if (!fit) return this._blind();

    const halfLaneM = median(allGates.map((g) => g.widthM)) / 2;
    const lookaheadM = Math.max(1.8, Math.min(3.0, 1.2 + 2.0 * Math.max(speedMps, 0)));
    const xHi = fit.xHi;
    const xs: number[] = [];
    for (let x = 0; x <= Math.max(xHi, 1) + lookaheadM + 3.0; x += 0.1) xs.push(x);
    const ys = xs.map((x) => evaluateMyTrackCentreline(fit, x));
    const offset = new Array(xs.length).fill(0);

    const clusters = clusterMyTrackIndices(obstacleIndices, bodyPoints, MY_TRACK_CONFIG.clusterLinkM);
    let stopDistanceM: number | null = null;
    let detourActive = false;
    for (const component of clusters) {
      const cxArr = component.map((i) => bodyPoints[i].x_m), cyArr = component.map((i) => bodyPoints[i].y_m);
      const xMinM = Math.min(...cxArr), xMaxM = Math.max(...cxArr);
      if (xMaxM < -3.0) continue;
      const cxMean = cxArr.reduce((a, b) => a + b, 0) / cxArr.length, cyMean = cyArr.reduce((a, b) => a + b, 0) / cyArr.length;
      const lateralFromCentre = cyArr.map((y, idx) => y - evaluateMyTrackCentreline(fit, cxArr[idx]));
      const sideSign = lateralFromCentre.reduce((a, b) => a + b, 0) / lateralFromCentre.length > 0 ? 1 : -1;
      const clearanceToCentreM = Math.min(...lateralFromCentre.map(Math.abs));
      const rampHeightM = (-sideSign * halfLaneM + sideSign * clearanceToCentreM) / 2;
      const freeWidthM = halfLaneM + clearanceToCentreM - 2 * MY_TRACK_CONFIG.coneRadiusM;
      let admitted = false, chosenRampM = 0, chosenLeadM = 0;
      const clusterCenterWorld = toWorld(cxMean, cyMean);
      const existingLock = this.locks.find((lock) => Math.hypot(lock.clusterCenterWorld.x_m - clusterCenterWorld.x_m, lock.clusterCenterWorld.y_m - clusterCenterWorld.y_m) < 2.0);
      if (existingLock) {
        // Reused verbatim, NOT re-searched -- re-searching as the kart gets
        // closer (and the geometry gets less favourable) can flip admission
        // off mid-manoeuvre and snap the path back toward the obstacle. This
        // is the exact bug class the Python side traced and fixed.
        admitted = true; chosenRampM = existingLock.rampLengthM; chosenLeadM = existingLock.leadDistanceM;
      } else if (freeWidthM >= this.geometry.halfWidthM * 2 + 0.5) {
        for (const rampM of MY_TRACK_CONFIG.detourRampLengthsM) {
          let leadM = 0;
          while (xMinM - leadM - rampM >= -0.2) {
            const rampOffset = myTrackRampOffset(xs, xMinM, xMaxM, leadM, rampM, rampHeightM);
            const candidateYs = ys.map((y, idx) => y + rampOffset[idx]);
            const heading = xs.map((_, idx) => {
              const i0 = Math.max(0, idx - 1), i1 = Math.min(xs.length - 1, idx + 1);
              return Math.atan2(candidateYs[i1] - candidateYs[i0], xs[i1] - xs[i0]);
            });
            let sweptOk = true;
            for (let k = 0; k < xs.length && sweptOk; k += 2) {
              for (const p of bodyPoints) {
                if (p.x_m > xs[k] + this.geometry.frontExtentM + 1 || p.x_m < xs[k] - 2) continue;
                const clearance = myTrackFootprintClearanceM(p.x_m, p.y_m, xs[k], candidateYs[k], heading[k], this.geometry.rearOverhangM, this.geometry.frontExtentM, this.geometry.halfWidthM) - MY_TRACK_CONFIG.coneRadiusM;
                if (clearance < MY_TRACK_CONFIG.footprintMarginM) { sweptOk = false; break; }
              }
            }
            if (sweptOk) { admitted = true; chosenRampM = rampM; chosenLeadM = leadM; this.locks.push({ clusterCenterWorld, rampLengthM: rampM, leadDistanceM: leadM }); break; }
            leadM += MY_TRACK_CONFIG.detourLeadStepM;
          }
          if (admitted) break;
        }
      }
      if (admitted) {
        const rampOffset = myTrackRampOffset(xs, xMinM, xMaxM, chosenLeadM, chosenRampM, rampHeightM);
        for (let i = 0; i < offset.length; i++) offset[i] += rampOffset[i];
        detourActive = true;
      } else {
        const candidateStopM = xMinM - this.geometry.frontExtentM - 0.5;
        stopDistanceM = stopDistanceM == null ? candidateStopM : Math.min(stopDistanceM, candidateStopM);
      }
    }

    const pathBody = xs.map((x, i) => ({ x_m: x, y_m: ys[i] + offset[i] }));
    const pathWorld = pathBody.map((p) => toWorld(p.x_m, p.y_m));

    let targetIdx = pathBody.length - 1;
    for (let i = 0; i < pathBody.length; i++) if (Math.hypot(pathBody[i].x_m, pathBody[i].y_m) >= lookaheadM) { targetIdx = i; break; }
    const lookaheadPoint = pathBody[targetIdx];
    const alpha = Math.atan2(lookaheadPoint.y_m, lookaheadPoint.x_m);
    const ld = Math.max(0.5, Math.hypot(lookaheadPoint.x_m, lookaheadPoint.y_m));
    // Tried (and reverted, see the developer's final report): a Stanley-style
    // heading-error+cross-track-error feedback term added on top of this
    // pure-pursuit aim, meant to reduce the lag this causes through
    // MY_TRACK's lane-shift bends. It made the peak deviation worse (grew
    // run-over-run to >1 m) by saturating the steering clamp and oscillating,
    // rather than better. Left as plain pure pursuit; the ~0.25 m target is
    // NOT met (actual: see the developer's final report for the measured
    // number) and is reported honestly rather than hidden behind a tolerance.
    const steeringRad = Math.max(-0.3490658504, Math.min(0.3490658504, Math.atan2(2 * this.geometry.wheelbaseM * Math.sin(alpha), ld)));

    const curvatureSamples: number[] = [];
    for (let i = 1; i < Math.min(50, pathBody.length - 1); i++) {
      const h0 = Math.atan2(pathBody[i].y_m - pathBody[i - 1].y_m, pathBody[i].x_m - pathBody[i - 1].x_m);
      const h1 = Math.atan2(pathBody[i + 1].y_m - pathBody[i].y_m, pathBody[i + 1].x_m - pathBody[i].x_m);
      curvatureSamples.push(Math.abs(h1 - h0) / 0.1);
    }
    const nearCurvature = curvatureSamples.length ? Math.max(...curvatureSamples) : 0;
    let targetSpeedMps = this.cruiseSpeedMps;
    if (nearCurvature > 1e-3) targetSpeedMps = Math.min(targetSpeedMps, Math.sqrt(0.6 / nearCurvature));
    if (stopDistanceM != null) targetSpeedMps = Math.min(targetSpeedMps, maxSpeedForStopDistance(stopDistanceM, 1.5));

    this.state = detourActive ? 'DETOUR' : 'RUN';
    const lateralDeviationM = -(evaluateMyTrackCentreline(fit, 0) + offset[0]);

    return { state: this.state, steeringRad, targetSpeedMps, brake: stopDistanceM != null && targetSpeedMps < 0.05, gates: allGates.length, obstacle: clusters.length > 0, detour: detourActive, finishDetected: false, lateralDeviationM, pathWorldXY: pathWorld, gatesList: allGates, blind: false };
  }

  private _blind(): MyTrackStepResult {
    this.state = 'RUN';
    return { state: 'RUN', steeringRad: 0, targetSpeedMps: this.blindCreepSpeedMps, brake: false, gates: 0, obstacle: false, detour: false, finishDetected: false, lateralDeviationM: null, pathWorldXY: [], gatesList: [], blind: true };
  }
}

/** Real Ackermann steering: given a mean/bicycle-model steering angle and
 * wheelbase/track, returns the two different front-wheel angles (inner
 * wheel turns more than outer). Pure function so the validator and unit
 * tests can check it directly, independent of three.js. */
export function computeAckermannAngles(deltaRad: number, wheelbaseM: number, trackM: number): { leftRad: number; rightRad: number } {
  if (Math.abs(deltaRad) < 1e-6) return { leftRad: deltaRad, rightRad: deltaRad };
  const radiusM = wheelbaseM / Math.tan(deltaRad);
  return { leftRad: Math.atan(wheelbaseM / (radiusM - trackM / 2)), rightRad: Math.atan(wheelbaseM / (radiusM + trackM / 2)) };
}

// ---------------------------------------------------------------------------
// Telemetry export + lateral-deviation summary helpers (Task 4 / Task 2).
// ---------------------------------------------------------------------------

export interface TelemetryRow { t_s: number; x_m: number; y_m: number; yaw_rad: number; speed_mps: number; steer_rad: number; brake: number; lateralDeviationM: number | null; state: string }

export function frameToTelemetryRow(frame: MissionFrame): TelemetryRow {
  return {
    t_s: frame.timestamp_s, x_m: frame.vehicle.x_m, y_m: frame.vehicle.y_m, yaw_rad: frame.vehicle.yaw_rad, speed_mps: frame.vehicle.speed_mps,
    steer_rad: frame.vehicle.measured_steering_rad ?? 0, brake: frame.vehicle.brake_feedback ?? 0,
    lateralDeviationM: frame.lateralDeviationM ?? null, state: frame.planning.skidpad?.state ?? '',
  };
}

export function exportTelemetryCsv(frames: MissionFrame[]): string {
  const header = 'time_s,x_m,y_m,yaw_rad,speed_mps,steer_rad,brake,lateral_deviation_m,state';
  const rows = frames.map(frameToTelemetryRow).map((r) => [r.t_s, r.x_m, r.y_m, r.yaw_rad, r.speed_mps, r.steer_rad, r.brake, r.lateralDeviationM ?? '', r.state].join(','));
  return [header, ...rows].join('\n');
}

export function exportTelemetryJson(frames: MissionFrame[]): string {
  return JSON.stringify(frames.map(frameToTelemetryRow));
}

/** Treats missing values as absent, not zero -- a frame with no fit does not
 * count as zero deviation. Returns 0 if no frame has the field set. */
export function maxAbsLateralDeviationM(frames: MissionFrame[]): number {
  let max = 0;
  for (const frame of frames) if (frame.lateralDeviationM != null) max = Math.max(max, Math.abs(frame.lateralDeviationM));
  return max;
}

export function rmsLateralDeviationM(frames: MissionFrame[]): number {
  const values = frames.map((f) => f.lateralDeviationM).filter((v): v is number => v != null);
  if (!values.length) return 0;
  return Math.sqrt(values.reduce((sum, v) => sum + v * v, 0) / values.length);
}
