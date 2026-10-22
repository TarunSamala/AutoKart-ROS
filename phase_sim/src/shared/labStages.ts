/**
 * Algorithm-stage model for the ALGORITHM LAB (/shared).
 *
 * Every number shown here is read from the diagnostic returned by
 * autokart_perception.shared_core.AlgorithmCore.evaluate (via lab_api :8767).
 * The frontend NEVER re-runs geometry: the only derived value is the set
 * difference Delaunay \ Gabriel (edges the Python Gabriel filter removed),
 * which is pure display filtering of two lists the backend already returned.
 */

export type XY = [number, number];
export type Observation = { id: number; label: string; confidence: number; xy: XY; sigma_m?: number; distance_m?: number };
export type TrackedCone = { id: number; label: string; x_m: number; y_m: number; confidence: number; sigma_m: number };
export type Gate = { id?: number; left_id: number; right_id: number; midpoint: XY; width: number; quality: number; status: string };
export type RejectedGate = { left_id: number; right_id: number; reason: string };
export type PathSample = { xy: XY; heading_rad: number; segment_m: number; curvature_m_inv: number | null };
export type Diagnostic = {
  algorithm_version: string;
  configuration_sha256?: string;
  source?: string;
  mission?: string;
  reasons: string[];
  mission_state: { state: string; left_laps?: number; right_laps?: number; winding_progress_turns?: number };
  observations: Observation[];
  tracked_cones?: TrackedCone[];
  geometry: {
    status?: string;
    delaunay_edges: [number, number][];
    delaunay_triangles?: [number, number, number][];
    gabriel_edges: [number, number][];
    boundaries: { left_ids: number[]; right_ids: number[]; status: string; mode?: string; confidence?: number; reason?: string };
    gates: Gate[];
    rejected_candidates?: RejectedGate[];
    centerline: XY[];
    centerline_status?: string;
    curvature_status?: string;
    path_profile?: PathSample[];
    topology_reasons?: string[];
    skidpad_topology: null | { valid: boolean; reason?: string; provenance?: string };
  };
  planning: { path_valid: boolean; path_frame?: string; stopping_distance_m?: number | null; speed_profile_provenance?: string };
  control_candidate: { valid: boolean; target_speed_mps: number; steering_angle_rad: number; target_point_base_m?: XY | null; expiry_ns?: number; published_to_vcu?: boolean };
  vehicle_authorized: boolean;
  sdk_status?: string;
  processing_ms?: number;
};

export type StageId = 'observe' | 'delaunay' | 'gabriel' | 'boundaries' | 'gates' | 'centerline' | 'mission' | 'control';
export type StageState = 'OK' | 'EMPTY' | 'INVALID' | 'N/A' | 'WAITING';

export const LAB_STAGES: ReadonlyArray<{ id: StageId; label: string; module: string; explain: string }> = [
  { id: 'observe', label: 'OBSERVE', module: 'lab_api sensor model → shared_core input validation',
    explain: 'Simulated cones inside the 12 m / ±1.1 rad field of view, in base_link (X forward, Y left). Invalid or duplicate cones are dropped by shared_core.' },
  { id: 'delaunay', label: 'DELAUNAY', module: 'geometry.triangulate (scipy Delaunay)',
    explain: 'Triangulation of every valid observed cone. Needs ≥3 non-collinear points.' },
  { id: 'gabriel', label: 'GABRIEL', module: 'gabriel.filter_gabriel',
    explain: 'Keeps a Delaunay edge only if no other cone lies inside the circle whose diameter is that edge. Red dashed = removed by the filter.' },
  { id: 'boundaries', label: 'BOUNDARIES', module: 'boundaries.estimate_boundaries',
    explain: 'Assigns cones to left (blue) and right (yellow) track boundaries.' },
  { id: 'gates', label: 'GATES', module: 'gates.select_track_gates / candidate_gates',
    explain: 'Left↔right cone pairs that form a crossable gate. Dashed red = candidate rejected (ambiguous / not selected).' },
  { id: 'centerline', label: 'CENTERLINE', module: 'centerline.generate_centerline + curvature.profile_centerline',
    explain: 'Gate midpoints ordered into a local path, then a heading/curvature profile.' },
  { id: 'mission', label: 'MISSION', module: 'shared_core mission logic (skidpad topology / WindingTracker)',
    explain: 'Mission-specific logic. Skidpad ring fitting and lap counting only run when mission = SKIDPAD.' },
  { id: 'control', label: 'CONTROLLER', module: 'kart_sim.autonomy.PurePursuitController',
    explain: 'Pure-Pursuit steering/speed CANDIDATE toward the lookahead target. Diagnostic only: published_to_vcu is always false.' },
];

const key = ([a, b]: [number, number]) => (a < b ? `${a}-${b}` : `${b}-${a}`);

/** Delaunay edges that the Python Gabriel filter did not keep (display-only set difference). */
export function gabrielRejected(diagnostic: Diagnostic | null | undefined): [number, number][] {
  if (!diagnostic) return [];
  const kept = new Set(diagnostic.geometry.gabriel_edges.map(key));
  return diagnostic.geometry.delaunay_edges.filter((edge) => !kept.has(key(edge)));
}

const fmt = (value: number | null | undefined, places = 2) => value == null || !Number.isFinite(value) ? '—' : value.toFixed(places);

export type StageSummary = { state: StageState; count: string; facts: Array<[string, string]> };

/** Summarise one stage purely from backend fields. */
export function stageSummary(diagnostic: Diagnostic | null | undefined, id: StageId): StageSummary {
  if (!diagnostic) return { state: 'WAITING', count: '—', facts: [['STATUS', 'No diagnostic from lab_api yet']] };
  const g = diagnostic.geometry;
  const reasons = diagnostic.reasons;
  switch (id) {
    case 'observe': {
      const n = diagnostic.observations.length;
      return { state: reasons.includes('INVALID_GEOMETRY') && n === 0 ? 'INVALID' : n ? 'OK' : 'EMPTY', count: String(n), facts: [
        ['VALID OBSERVATIONS', String(n)], ['FRAME', 'base_link'], ['SOURCE', diagnostic.source ?? '—'],
        ['TRACKED (odom map)', String(diagnostic.tracked_cones?.length ?? '—')], ['SDK STATUS', diagnostic.sdk_status ?? '—'],
        ['CORE PROCESSING', `${fmt(diagnostic.processing_ms)} ms`]] };
    }
    case 'delaunay': {
      const edges = g.delaunay_edges.length;
      return { state: g.status && g.status !== 'OK' ? 'INVALID' : edges ? 'OK' : 'EMPTY', count: String(edges), facts: [
        ['GRAPH STATUS', g.status ?? '—'], ['EDGES', String(edges)], ['TRIANGLES', String(g.delaunay_triangles?.length ?? '—')]] };
    }
    case 'gabriel': {
      const removed = gabrielRejected(diagnostic).length;
      return { state: g.gabriel_edges.length ? 'OK' : 'EMPTY', count: String(g.gabriel_edges.length), facts: [
        ['KEPT', String(g.gabriel_edges.length)], ['REMOVED BY FILTER', String(removed)], ['INPUT (DELAUNAY)', String(g.delaunay_edges.length)]] };
    }
    case 'boundaries': {
      const b = g.boundaries;
      return { state: b.status === 'CANDIDATE' ? 'OK' : 'INVALID', count: `${b.left_ids.length}/${b.right_ids.length}`, facts: [
        ['STATUS', b.status], ['MODE', b.mode ?? '—'], ['CONFIDENCE', fmt(b.confidence)], ['REASON', b.reason ?? '—'],
        ['LEFT / RIGHT CONES', `${b.left_ids.length} / ${b.right_ids.length}`]] };
    }
    case 'gates': {
      const rejected = g.rejected_candidates ?? [];
      const byReason = rejected.reduce<Record<string, number>>((acc, item) => ({ ...acc, [item.reason]: (acc[item.reason] ?? 0) + 1 }), {});
      const ambiguous = g.gates.filter((gate) => gate.status !== 'CANDIDATE').length;
      return { state: !g.gates.length ? 'EMPTY' : ambiguous ? 'INVALID' : 'OK', count: String(g.gates.length), facts: [
        ['SELECTED GATES', String(g.gates.length)], ['AMBIGUOUS SELECTED', String(ambiguous)], ['REJECTED CANDIDATES', String(rejected.length)],
        ['REJECTION REASONS', Object.entries(byReason).map(([reason, count]) => `${reason}×${count}`).join(', ') || 'none'],
        ['WIDTHS', g.gates.map((gate) => fmt(gate.width, 1)).join(', ') || '—']] };
    }
    case 'centerline': {
      const curvatures = (g.path_profile ?? []).map((sample) => sample.curvature_m_inv).filter((value): value is number => value != null);
      const maxK = curvatures.length ? Math.max(...curvatures.map(Math.abs)) : null;
      return { state: g.centerline_status === 'PROVISIONAL' ? 'OK' : g.centerline.length ? 'INVALID' : 'EMPTY', count: String(g.centerline.length), facts: [
        ['CENTERLINE STATUS', g.centerline_status ?? '—'], ['CURVATURE STATUS', g.curvature_status ?? '—'], ['POINTS', String(g.centerline.length)],
        ['MAX |κ|', maxK == null ? '—' : `${fmt(maxK, 3)} 1/m`], ['TOPOLOGY REASONS', g.topology_reasons?.join(', ') || 'none'],
        ['PATH VALID', String(diagnostic.planning.path_valid)]] };
    }
    case 'mission': {
      const m = diagnostic.mission_state;
      const topology = g.skidpad_topology;
      return { state: diagnostic.mission === 'SKIDPAD' ? (topology?.valid ? 'OK' : 'INVALID') : 'N/A', count: m.state, facts: [
        ['MISSION', diagnostic.mission ?? '—'], ['STATE', m.state],
        ['LAPS L / R', m.left_laps == null ? 'n/a (not SKIDPAD)' : `${m.left_laps} / ${m.right_laps}`],
        ['SKIDPAD TOPOLOGY', topology == null ? 'not evaluated (mission ≠ SKIDPAD)' : topology.valid ? `VALID · ${topology.provenance ?? ''}` : `INVALID · ${topology.reason ?? '—'}`],
        ['STOPPING DISTANCE', diagnostic.planning.stopping_distance_m == null ? '— (BRAKING_TEST only)' : `${fmt(diagnostic.planning.stopping_distance_m)} m`]] };
    }
    case 'control': {
      const c = diagnostic.control_candidate;
      return { state: c.valid ? 'OK' : 'INVALID', count: c.valid ? `${fmt(c.steering_angle_rad, 3)} rad` : 'NONE', facts: [
        ['CANDIDATE VALID', String(c.valid)], ['TARGET SPEED', `${fmt(c.target_speed_mps)} m/s`], ['STEERING', `${fmt(c.steering_angle_rad, 3)} rad`],
        ['LOOKAHEAD (base_link)', c.target_point_base_m ? `${fmt(c.target_point_base_m[0])}, ${fmt(c.target_point_base_m[1])} m` : '—'],
        ['SPEED PROFILE', diagnostic.planning.speed_profile_provenance ?? '—'],
        ['PUBLISHED TO VCU', String(c.published_to_vcu ?? false)], ['VEHICLE AUTHORIZED', String(diagnostic.vehicle_authorized)],
        ['REASONS', reasons.join(', ') || 'none']] };
    }
  }
}

/** Which stages' layers are drawn when `focus` (0-based) is selected; null shows every stage. */
export function stageVisible(focus: number | null, stageIndex: number): boolean {
  return focus == null || stageIndex <= focus;
}
