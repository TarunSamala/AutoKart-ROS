import Delaunator from 'delaunator';
import type {
  Cone,
  Edge,
  Midpoint,
  OrderedMidpoint,
  OrderingCheck,
  PathPoint,
  Point2D,
  Triangle,
  VehicleState,
} from './types';
import { distance, normalizeAngle, pointInPolygon } from './math';

export const MIN_TRACK_WIDTH = 2;
export const MAX_TRACK_WIDTH = 5.25;

export interface DelaunayResult {
  points: Cone[];
  connectivity: [number, number, number][];
  triangles: Triangle[];
}

export interface FilterResult {
  triangles: Triangle[];
  interiorTriangles: Triangle[];
  exteriorTriangles: Triangle[];
  flags: boolean[];
  corridorPolygon: Point2D[];
}

export interface EdgeResult {
  uniqueEdges: Edge[];
  crossTrackEdges: Edge[];
}

export interface OrderingResult {
  ordered: OrderedMidpoint[];
  checks: OrderingCheck[];
}

export interface InterpolationResult {
  cumulativeDistance: number[];
  segmentDistances: number[];
  path: PathPoint[];
}

function vehicleLongitudinal(point: Point2D, vehicle: VehicleState): number {
  const dx = point.x - vehicle.x;
  const dy = point.y - vehicle.y;
  return Math.cos(vehicle.heading) * dx + Math.sin(vehicle.heading) * dy;
}

export function triangulate(points: Cone[]): DelaunayResult {
  if (points.length < 3) return { points, connectivity: [], triangles: [] };
  const delaunay = Delaunator.from(points, (point) => point.x, (point) => point.y);
  const connectivity: [number, number, number][] = [];
  const triangles: Triangle[] = [];
  for (let i = 0; i < delaunay.triangles.length; i += 3) {
    const indices: [number, number, number] = [
      delaunay.triangles[i],
      delaunay.triangles[i + 1],
      delaunay.triangles[i + 2],
    ];
    const vertices: [Cone, Cone, Cone] = [points[indices[0]], points[indices[1]], points[indices[2]]];
    const centroid = {
      x: (vertices[0].x + vertices[1].x + vertices[2].x) / 3,
      y: (vertices[0].y + vertices[1].y + vertices[2].y) / 3,
    };
    connectivity.push(indices);
    triangles.push({
      id: `T${triangles.length + 1}`,
      indices,
      vertices,
      centroid,
      interior: false,
      filterReason: 'NOT TESTED',
    });
  }
  return { points, connectivity, triangles };
}

export function buildVisibleCorridor(points: Cone[], vehicle: VehicleState): Point2D[] {
  const byProgress = (a: Cone, b: Cone) => vehicleLongitudinal(a, vehicle) - vehicleLongitudinal(b, vehicle);
  const blue = points.filter((point) => point.side === 'BLUE').sort(byProgress);
  const yellow = points.filter((point) => point.side === 'YELLOW').sort(byProgress);
  if (blue.length < 2 || yellow.length < 2) return [];
  return [...blue, ...yellow.reverse()];
}

export function filterTriangles(
  triangles: Triangle[],
  points: Cone[],
  vehicle: VehicleState,
): FilterResult {
  const corridorPolygon = buildVisibleCorridor(points, vehicle);
  const tested = triangles.map((triangle) => {
    const sideCount = new Set(triangle.vertices.map((vertex) => vertex.side)).size;
    const mixedSides = sideCount === 2;
    const centroidInside = pointInPolygon(triangle.centroid, corridorPolygon);
    const interior = mixedSides && centroidInside;
    let filterReason = 'CENTROID INSIDE VISIBLE CORRIDOR';
    if (!mixedSides) filterReason = 'ALL VERTICES ON SAME BOUNDARY';
    else if (!centroidInside) filterReason = 'CENTROID OUTSIDE VISIBLE CORRIDOR';
    return { ...triangle, interior, filterReason };
  });
  return {
    triangles: tested,
    interiorTriangles: tested.filter((triangle) => triangle.interior),
    exteriorTriangles: tested.filter((triangle) => !triangle.interior),
    flags: tested.map((triangle) => triangle.interior),
    corridorPolygon,
  };
}

export function extractEdges(interiorTriangles: Triangle[]): EdgeResult {
  const edgeMap = new Map<string, Edge>();
  for (const triangle of interiorTriangles) {
    const pairs: [number, number][] = [
      [0, 1],
      [1, 2],
      [2, 0],
    ];
    for (const [start, end] of pairs) {
      const aIndex = triangle.indices[start];
      const bIndex = triangle.indices[end];
      const low = Math.min(aIndex, bIndex);
      const high = Math.max(aIndex, bIndex);
      const key = `${low}:${high}`;
      const existing = edgeMap.get(key);
      if (existing) {
        existing.triangleIds.push(triangle.id);
        continue;
      }
      const a = triangle.vertices[start];
      const b = triangle.vertices[end];
      const length = distance(a, b);
      const crossTrack = a.side !== b.side;
      const widthPass = length >= MIN_TRACK_WIDTH && length <= MAX_TRACK_WIDTH;
      const classification = !crossTrack
        ? 'REJECT — SAME SIDE'
        : widthPass
          ? 'VALID CROSS-TRACK EDGE'
          : 'REJECT — WIDTH';
      edgeMap.set(key, {
        id: '',
        aIndex,
        bIndex,
        a,
        b,
        triangleIds: [triangle.id],
        length,
        crossTrack,
        widthPass,
        classification,
      });
    }
  }
  const uniqueEdges = [...edgeMap.values()]
    .sort((a, b) => `${a.a.id}:${a.b.id}`.localeCompare(`${b.a.id}:${b.b.id}`))
    .map((edge, index) => ({ ...edge, id: `E${index + 1}` }));
  return {
    uniqueEdges,
    crossTrackEdges: uniqueEdges.filter((edge) => edge.crossTrack && edge.widthPass),
  };
}

export function calculateMidpoints(edges: Edge[]): Midpoint[] {
  const midpoints: Midpoint[] = [];
  for (const edge of edges) {
    const blue = edge.a.side === 'BLUE' ? edge.a : edge.b;
    const yellow = edge.a.side === 'YELLOW' ? edge.a : edge.b;
    const midpoint: Midpoint = {
      id: `M${midpoints.length + 1}`,
      sourceEdgeId: edge.id,
      blue,
      yellow,
      x: (blue.x + yellow.x) / 2,
      y: (blue.y + yellow.y) / 2,
    };
    if (!midpoints.some((existing) => distance(existing, midpoint) < 0.12)) midpoints.push(midpoint);
  }
  return midpoints;
}

export function orderMidpoints(midpoints: Midpoint[], vehicle: VehicleState, edges: Edge[]): OrderingResult {
  if (midpoints.length === 0) return { ordered: [], checks: [] };
  const remaining = [...midpoints];
  const ordered: OrderedMidpoint[] = [];
  const allChecks: OrderingCheck[] = [];
  let previous: Point2D = vehicle;
  let previousHeading = vehicle.heading;
  let previousMidpoint: Midpoint | null = null;

  while (remaining.length > 0) {
    const candidates = remaining.map((candidate) => {
      const dx = candidate.x - previous.x;
      const dy = candidate.y - previous.y;
      const heading = Math.atan2(dy, dx);
      const headingChange = normalizeAngle(heading - previousHeading);
      const progress = vehicleLongitudinal(candidate, vehicle);
      const forwardProgress = progress > -0.35 && Math.abs(headingChange) < Math.PI * 0.72;
      const candidateEdge = edges.find((edge) => edge.id === candidate.sourceEdgeId);
      const previousEdge = previousMidpoint
        ? edges.find((edge) => edge.id === previousMidpoint?.sourceEdgeId)
        : null;
      const sharesCone = Boolean(
        candidateEdge &&
          previousEdge &&
          [candidateEdge.a.id, candidateEdge.b.id].some((id) => id === previousEdge.a.id || id === previousEdge.b.id),
      );
      const candidateDistance = distance(previous, candidate);
      const connectivityPass = previousMidpoint === null || sharesCone || candidateDistance < 2.4;
      const score = candidateDistance + Math.abs(headingChange) * 1.7 + (forwardProgress ? 0 : 100) + (connectivityPass ? 0 : 12);
      const check: OrderingCheck = {
        candidateId: candidate.id,
        dx,
        dy,
        forwardProgress,
        distance: candidateDistance,
        headingChange,
        connectivityPass,
        score,
        selected: false,
      };
      return {
        candidate,
        check,
        heading,
      };
    });
    candidates.sort((a, b) => a.check.score - b.check.score);
    const best = candidates[0];
    if (!best || !best.check.forwardProgress || (!best.check.connectivityPass && ordered.length > 0)) break;
    best.check.selected = true;
    allChecks.push(...candidates.map((item) => item.check));
    ordered.push({
      ...best.candidate,
      order: ordered.length + 1,
      distanceFromPrevious: best.check.distance,
      headingFromPrevious: best.heading,
      selectionReason: best.check,
    });
    const index = remaining.findIndex((item) => item.id === best.candidate.id);
    remaining.splice(index, 1);
    previous = best.candidate;
    previousHeading = best.heading;
    previousMidpoint = best.candidate;
  }
  return { ordered, checks: allChecks };
}

function catmullRom(p0: Point2D, p1: Point2D, p2: Point2D, p3: Point2D, t: number): Point2D {
  const t2 = t * t;
  const t3 = t2 * t;
  return {
    x: 0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 + (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3),
    y: 0.5 * ((2 * p1.y) + (-p0.y + p2.y) * t + (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 + (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3),
  };
}

export function interpolateMidpoints(ordered: OrderedMidpoint[]): InterpolationResult {
  const segmentDistances: number[] = [];
  const cumulativeDistance = [0];
  for (let i = 1; i < ordered.length; i += 1) {
    const segment = distance(ordered[i - 1], ordered[i]);
    segmentDistances.push(segment);
    cumulativeDistance.push(cumulativeDistance[cumulativeDistance.length - 1] + segment);
  }
  if (ordered.length === 0) return { cumulativeDistance: [], segmentDistances, path: [] };
  if (ordered.length === 1) {
    return { cumulativeDistance, segmentDistances, path: [{ ...ordered[0], index: 0, s: 0, sourceSegment: 0 }] };
  }
  const path: PathPoint[] = [];
  let pathS = 0;
  for (let segment = 0; segment < ordered.length - 1; segment += 1) {
    const p0 = ordered[Math.max(0, segment - 1)];
    const p1 = ordered[segment];
    const p2 = ordered[segment + 1];
    const p3 = ordered[Math.min(ordered.length - 1, segment + 2)];
    const sampleCount = Math.max(8, Math.ceil(distance(p1, p2) / 0.08));
    for (let j = 0; j < sampleCount; j += 1) {
      const point = catmullRom(p0, p1, p2, p3, j / sampleCount);
      if (path.length > 0) pathS += distance(path[path.length - 1], point);
      path.push({ ...point, index: path.length, s: pathS, sourceSegment: segment });
    }
  }
  const final = ordered[ordered.length - 1];
  if (path.length > 0) pathS += distance(path[path.length - 1], final);
  path.push({ x: final.x, y: final.y, index: path.length, s: pathS, sourceSegment: ordered.length - 2 });
  return { cumulativeDistance, segmentDistances, path };
}
