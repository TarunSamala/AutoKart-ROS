// Wire types for the read-only AutoKart Vehicle Dashboard. Every field a backend may omit is optional:
// a missing value is rendered as "— (NO DATA)", never as 0.

export type Pt = [number, number];
export type Cone = { id: number; label: string; confidence: number; xy: Pt; distance_m?: number; sigma_m?: number };
export type Detection = { id: number; label: string; confidence: number; box: number[] };
export type Gate = { left_id: number; right_id: number; midpoint: Pt; status?: string; width?: number; quality?: number; id?: number };
export type Provisional = { id: number; label: string; confidence: number; nominal_xy_m: Pt | null; uncertainty_m?: number };

export type Snapshot = {
  schema_version?: number; frame_id: string | null; source?: string; status?: string; reasons?: string[];
  capture_timestamp_ns?: number; processed_timestamp_ns?: number; camera_id?: string; camera_sequence?: number; depth_sequence?: number;
  camera_frame?: string; base_frame?: string; pose_status?: string;
  calibration_id?: string; calibration_status?: string; depth_scale_qualified?: boolean; depth_scale_provenance?: string;
  depth_scale_m_per_raw_candidate?: number; algorithm_version?: string; configuration_sha256?: string;
  intrinsics?: { fx: number; fy: number; cx: number; cy: number };
  rgb_depth_pair?: string; rgb_depth_alignment?: string | null; rgb_depth_timestamp_delta_ms?: number | null;
  depth_stats?: { valid_pixels?: number; invalid_pixels?: number; valid_percent?: number; median_raw?: number | null; raw_unit?: string } | null;
  model?: { name?: string; version?: string; sha256?: string | null; supported_classes?: string[]; qualification?: string };
  detections?: Detection[]; detector_counts?: Record<string, number | null>;
  observations?: Cone[]; candidate_observations?: Cone[]; provisional_3d_observations?: Provisional[];
  rejected_candidates?: { id: number; localization_status: string; box?: number[] }[];
  tracked_cones?: unknown[]; tracking_status?: string;
  camera_motion?: { status: string; matched_points?: number; inlier_count?: number; reprojection_rms_px?: number | null; qualified?: boolean } | null;
  geometry?: {
    status?: string; delaunay_edges?: [number, number][]; delaunay_triangles?: [number, number, number][];
    gabriel_edges?: [number, number][]; gates?: Gate[];
    boundaries?: { left_ids: number[]; right_ids: number[]; mode?: string; status?: string; confidence?: number; reason?: string };
    centerline?: Pt[]; centerline_status?: string; curvature_status?: string; topology_reasons?: string[];
    rejected_candidates?: unknown[]; skidpad_topology?: unknown;
  };
  planning?: { path_valid?: boolean; local_path?: Pt[]; path_frame?: string; speed_profile_provenance?: string; stopping_distance_m?: number | null };
  mission_state?: { state?: string } | null;
  control_candidate?: { valid?: boolean; target_speed_mps?: number; steering_angle_rad?: number; target_point_base_m?: Pt | null; published_to_vcu?: boolean };
  perception?: { status?: string; safety_state?: string; safety_reasons?: string[]; provenance?: string };
  latencies_ms?: Record<string, number | null>;
  stream_metrics?: { camera_callback_fps?: number | null; received_frames?: number; mailbox_overwrites?: number; processed_fps?: number | null; inference_fps?: number | null };
  frame_age_ms?: number | null; last_good_age_ms?: number | null;
  camera_streaming?: boolean; ready?: boolean; vehicle_authorized?: boolean;
};

export type Hop = { name: string; state: string; reason?: string | null; last_error?: string | null; last_error_detail?: string | null; [k: string]: unknown };
export type Pipeline = {
  schema_version?: number;
  service?: { name?: string; mode?: string; source?: string; pid?: number; started_at_ns?: number; uptime_s?: number; camera_owner?: boolean; detector?: string | null; model_sha256?: string | null };
  hops?: Hop[]; overall?: string; vehicle_authorized?: boolean;
};

export type Channel<P = Record<string, unknown>> = { current: boolean; state: 'CURRENT' | 'STALE' | 'MISSING'; age_ms: number | null; payload: P | null };
export type Envelope = { state: string; reason?: string; valid?: boolean; calibration_verified?: boolean; clear_distance_m?: number | null; stopping_distance_m?: number | null; data_source?: string };
export type GatewayStatus = {
  source?: string; mode?: string; run_id?: string; scenario?: string | null; ros_connected?: boolean;
  safety_requests_enabled?: boolean; estop_input_status?: string;
  odometry?: Channel; vcu?: Channel; safety?: Channel; safety_envelope?: Channel<Envelope>;
  stages?: Partial<Record<'perception' | 'mapping' | 'planning', Channel>>; read_only?: boolean; vehicle_authorized?: boolean;
};

// Subset of the gateway WebSocket MissionFrame this dashboard displays.
export type RosFrame = {
  source?: string; run_id?: string; sequence?: number; timestamp_s?: number;
  vehicle?: { x_m?: number; y_m?: number; yaw_rad?: number; speed_mps?: number; measured_steering_rad?: number | null; requested_steering_rad?: number | null; target_speed_mps?: number | null; brake_command?: number | null; brake_feedback?: number | null };
  planning?: { selected_path?: { x_m: number; y_m: number }[]; lookahead?: { x_m: number; y_m: number } | null };
  safety?: Record<string, unknown> & { state?: string; faults?: string[] };
  metrics?: { cross_track_error_m?: number | null; heading_error_rad?: number | null };
};

export type Tone = 'ok' | 'info' | 'warn' | 'alert' | 'fault' | 'idle';
export type DashEvent = { t: number; wall: string; hop: string; from: string; to: string; detail: string };
