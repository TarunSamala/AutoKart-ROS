export type Source = 'LAB' | 'SIMULATION' | 'REAL_KART' | 'DEMO';
export type Mode = 'LIVE' | 'PAUSE' | 'STEP' | 'REPLAY';
export type Point = { x_m: number; y_m: number; z_m?: number };
export type Cone = Point & { id: string; class: string; confidence: number; range_m?: number; bearing_rad?: number; track_age_s?: number };
export type Edge = { id: string; a: string; b: string; triangle_ids?: string[]; accepted?: boolean; reason?: string; blocking_cone_id?: string; radius_m?: number };
export type Gate = { id: string; source_edge_id?: string; midpoint: Point; width_m: number; heading_rad?: number; confidence?: number; valid: boolean; reason?: string };
export type Candidate = { id: string; points: Point[]; costs: Record<string, number>; total_cost: number; rank: number; selected: boolean; reason?: string };
export type StageDiagnostic = { source?: string; valid?: boolean; reason_codes?: string[]; sample_time_ns?: number; expires_ns?: number; frame_id?: string; sequence?: number } | null;
export type SafetyEnvelopeState = 'CLEAR' | 'BRAKING' | 'CRITICAL' | 'UNKNOWN';
/** Self-reported by the publishing node's own configuration, never cryptographically verified. */
export type SafetyEnvelopeDataSource = 'LIVE' | 'SIMULATED' | 'REPLAY' | 'UNVERIFIED';
export type SafetyEnvelope = { state: SafetyEnvelopeState; reason: string; valid: boolean; calibration_verified: boolean;
  clear_distance_m: number | null; stopping_distance_m: number | null; guaranteed_deceleration_mps2: number;
  data_source: SafetyEnvelopeDataSource };
export interface MissionFrame {
  schema_version: 1;
  source: Source;
  run_id: string;
  scenario?: string;
  sequence: number;
  timestamp_s: number;
  vehicle: Point & { yaw_rad: number; speed_mps: number; measured_steering_rad?: number | null; requested_steering_rad?: number | null; target_speed_mps?: number | null; brake_command?: number | null; brake_feedback?: number | null };
  perception: { cones: Cone[]; tracked_cones?: Cone[]; ground_truth_cones?: Cone[]; sensor_range_m?: number; sensor_fov_rad?: number; obstacles?: Array<Point & { id: string; radius_m: number; distance_m?: number; relative_speed_mps?: number }> };
  /** Display-only context; never presented as a planner output. */
  visualization?: { course_cones?: Cone[]; course_guide?: Point[]; boundary_left?: Point[]; boundary_right?: Point[]; boundary_status?: string; trail?: Point[]; special_markers?: Array<Point & { id: string; label: string }>; skidpad?: { left_center: Point; right_center: Point; radius_m: number; w_left: number; w_right: number } };
  planning: { delaunay_edges: Edge[]; delaunay_triangles?: [string, string, string][]; gabriel_edges: Edge[]; gates: Gate[]; candidate_paths: Candidate[]; selected_path: Point[]; lookahead?: Point | null; predicted_path?: Point[]; skidpad?: { left_center?: Point; right_center?: Point; left_radius_m?: number; right_radius_m?: number; w_left?: number; w_right?: number; target_winding?: number; state?: string; branch?: string; branch_locked?: boolean; reason?: string; progress_index?: number; progress_unit?: string; branches?: Array<{ name: string; points: Point[]; valid: boolean; selected?: boolean; reason?: string }> } };
  safety: { state?: string; priority?: string; physical_estop?: boolean | null; remote_estop?: boolean | null; lora_heartbeat?: boolean | null; vcu_watchdog?: boolean | null; rpi_heartbeat?: boolean | null; command_fresh?: boolean | null; sensor_fresh?: boolean | null; motor_enable?: boolean | null; faults?: string[]; obstacle?: { id: string; distance_m: number; relative_speed_mps: number; ttc_s: number | null; reaction_distance_m?: number; braking_distance_m?: number; safety_margin_m?: number; stopping_distance_m: number; available_distance_m: number; margin_m: number; brake_trigger: boolean; predicted_stop?: Point };
    /** Front safety envelope monitor (DIAGNOSTIC_ONLY, vehicle_authority=OFF). Read-only; never a command source. */
    envelope?: SafetyEnvelope | null };
  metrics: { cross_track_error_m?: number | null; heading_error_rad?: number | null; planner_runtime_ms?: number | null; controller_runtime_ms?: number | null };
  diagnostics?: { perception?: StageDiagnostic; mapping?: StageDiagnostic; planning?: StageDiagnostic };
  /** Signed metres, perpendicular distance from the rear axle to the currently planned path/centreline; +left/-right of travel. Only populated for MY_TRACK; undefined elsewhere or when no fit is available. See docs/UI_CONTRACT.md. */
  lateralDeviationM?: number;
}

/** All sources, including LAB and replay, feed the same render contract. */
export type WorldFrame = MissionFrame;

export function frameAt(frames: MissionFrame[], time: number): MissionFrame | null {
  if (!frames.length) return null;
  let low = 0, high = frames.length - 1;
  while (low < high) {
    const mid = Math.ceil((low + high) / 2);
    if (frames[mid].timestamp_s <= time) low = mid;
    else high = mid - 1;
  }
  return frames[low].timestamp_s <= time ? frames[low] : null;
}

export function stageFrame(frame: MissionFrame, stage: number): MissionFrame {
  const planning = frame.planning;
  return {
    ...frame,
    vehicle: stage >= 8 ? frame.vehicle : { ...frame.vehicle, requested_steering_rad: null, target_speed_mps: null, brake_command: null },
    planning: {
      ...planning,
      delaunay_edges: stage >= 2 ? planning.delaunay_edges : [],
      gabriel_edges: stage >= 3 ? planning.gabriel_edges : [],
      gates: stage >= 4 ? planning.gates : [],
      candidate_paths: stage >= 5 ? planning.candidate_paths : [],
      selected_path: stage >= 7 ? planning.selected_path : [],
      lookahead: stage >= 8 ? planning.lookahead : null,
      skidpad: stage >= 6 ? planning.skidpad : undefined,
    },
  };
}

export const STAGES = ['PERCEPTION', 'DELAUNAY', 'GABRIEL', 'GATES', 'CANDIDATES', 'MISSION', 'SELECTED PATH', 'CONTROL'] as const;
