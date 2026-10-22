import { useEffect, useRef, useState } from 'react';
import type { MissionFrame, SafetyEnvelope, Source } from './schema';

type Channel = { current: boolean; state: 'CURRENT' | 'STALE' | 'MISSING'; age_ms: number | null; payload: Record<string, unknown> | null };
type EnvelopeChannel = { current: boolean; state: 'CURRENT' | 'STALE' | 'MISSING'; age_ms: number | null; payload: SafetyEnvelope | null };
type GatewayStatus = { source: string; mode: string; safety_requests_enabled?: boolean; ros_connected: boolean; odometry: Channel; vcu: Channel; safety: Channel; safety_envelope: EnvelopeChannel; stages: Record<'perception' | 'mapping' | 'planning', Channel>; vehicle_authorized: false };
type Detection = { id?: number; label?: string; confidence?: number; box?: [number, number, number, number] };
type CameraStatus = { available: boolean; frame_id?: string; image_frame?: string; capture_time_ns?: string; age_ms?: number; width?: number; height?: number; sequence?: number; capture_to_gateway_ms?: number | null; missing_frames?: number; source?: string; valid?: boolean; reason?: string; reason_codes?: string[]; calibration_status?: string; model_sha256?: string; detections?: Detection[]; cone_details?: Array<Record<string, unknown>>; depth_stats?: { valid_percent?: number; raw_unit?: string }; receive_fps?: number | null; processing_ms?: number | null; depth_qualified?: boolean; rgb_timestamp_ms?: number; depth_timestamp_ms?: number; rgb_sequence?: number; depth_sequence?: number; frame_age_ms?: number | null; inference_ms?: number | null; model_version?: string; model_backend?: string; model_stage?: string; safety_state?: string | null; latched_fault?: string | null; calibration_reasons?: string[]; depth_scale_factor?: number | null; depth_scale_verified?: boolean; mount_verified?: boolean; calibration_artifact?: { held_out_error_m?: ErrorStats | null; survey_error_m?: ErrorStats | null; tolerance_m?: number; operator?: string; expires_unix_s?: number } | null };
type ErrorStats = { median?: number; p95?: number; max?: number };

const number = (value: unknown, digits = 1) => typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '—';
const text = (value: unknown) => typeof value === 'string' || typeof value === 'number' || typeof value === 'boolean' ? String(value) : '—';
const errorText = (stats?: ErrorStats | null) => stats ? `p50 ${number(stats.median, 3)} · p95 ${number(stats.p95, 3)} · max ${number(stats.max, 3)} m` : 'NOT MEASURED';
export const calibrationState = (camera: { mount_verified?: boolean; calibration_reasons?: string[] } | null | undefined) =>
  camera?.mount_verified ? 'VERIFIED' : camera?.calibration_reasons?.length ? 'FAILED' : 'UNVERIFIED';
const sourceOf = (source: Source) => source === 'REAL_KART' ? 'LIVE' : 'SIMULATED';

const ENVELOPE_BADGE: Record<string, string> = { CLEAR: '🟢', BRAKING: '🟡', CRITICAL: '🔴', UNKNOWN: '⚪' };
const ENVELOPE_COLOR: Record<string, string> = { CLEAR: '#36e0bf', BRAKING: '#f8d55f', CRITICAL: '#ff6b6b', UNKNOWN: '#7f8fa0' };

/**
 * Schematic, non-metric front-safety-envelope overlay. calibration_verified is
 * always false until a verified camera mount/intrinsics survey exists, so this
 * draws fixed, labeled, dashed bands -- never pixel-accurate ground-plane lines
 * (HP60C_Camera_Geometry_Production_Spec.md Panel F / dashboard contract).
 */
function SafetyEnvelopeOverlay({ envelope, width, height }: { envelope: SafetyEnvelope | null; width: number; height: number }) {
  const state = envelope?.state ?? 'UNKNOWN';
  const dataSource = envelope?.data_source ?? 'UNVERIFIED';
  const color = ENVELOPE_COLOR[state];
  const band = (yFraction: number, halfWidthFraction: number, label: string, active: boolean) => {
    const y = height * yFraction;
    const x1 = width * (0.5 - halfWidthFraction), x2 = width * (0.5 + halfWidthFraction);
    return <g key={label} opacity={active ? 1 : 0.35}>
      <line x1={x1} y1={y} x2={x2} y2={y} stroke={color} strokeWidth={active ? 3 : 1.5} strokeDasharray="10 6" />
      <text x={x2 + 6} y={y + 4} fontSize={Math.max(10, height * 0.028)}>{label}</text>
    </g>;
  };
  return <svg viewBox={`0 0 ${width} ${height}`} preserveAspectRatio="none" className="mc-safety-overlay"
    aria-label={`Front safety envelope ${state}, data source ${dataSource}, geometry unverified, schematic only`}>
    <polygon points={`${width * 0.10},${height * 0.92} ${width * 0.90},${height * 0.92} ${width * 0.66},${height * 0.58} ${width * 0.34},${height * 0.58}`}
      fill={color} fillOpacity={0.08} stroke="none" />
    {band(0.58, 0.16, 'VERIFIED CLEAR (schematic)', state === 'CLEAR')}
    {band(0.75, 0.27, `BRAKING ENVELOPE${envelope?.stopping_distance_m != null ? ` · ${envelope.stopping_distance_m.toFixed(2)} m trigger` : ''}`, state === 'BRAKING')}
    {band(0.92, 0.40, `1.0 m MIN CLEARANCE${envelope?.clear_distance_m != null ? ` · nearest ${envelope.clear_distance_m.toFixed(2)} m` : ''}`, state === 'CRITICAL')}
    <text x={width / 2} y={height * 0.5} textAnchor="middle" fontSize={Math.max(11, height * 0.035)}
      fill="#ffd284" opacity={0.85}>GEOMETRY UNVERIFIED · SCHEMATIC ONLY</text>
    {dataSource !== 'LIVE' && <text x={width / 2} y={height * 0.5 + height * 0.06} textAnchor="middle"
      fontSize={Math.max(11, height * 0.035)} fill="#ff6b6b" opacity={0.9}>DATA SOURCE: {dataSource} · NOT A PHYSICAL SENSOR</text>}
  </svg>;
}

type ReadinessGate = { name: string; status: string; command?: string };
type Readiness = { available: boolean; reason?: string; ready?: boolean; gates?: ReadinessGate[];
  reasons?: string[]; age_s?: number };

export function RosLivePanel({ source, frame, sendDemoDrive, demoDriveRejected, wsConnected }: {
  source: Source; frame: MissionFrame | null;
  sendDemoDrive?: (message: Record<string, unknown>) => boolean;
  demoDriveRejected?: string | null; wsConnected?: boolean;
}) {
  const [status, setStatus] = useState<GatewayStatus | null>(null);
  const [camera, setCamera] = useState<CameraStatus | null>(null);
  const [readiness, setReadiness] = useState<Readiness | null>(null);
  const [error, setError] = useState('CONNECTING');
  const [requestResult, setRequestResult] = useState('');
  const [areaClear, setAreaClear] = useState(false);
  const [driveResult, setDriveResult] = useState('');
  const holdTimer = useRef<number | null>(null);
  const requestSafety = (action: 'arm' | 'stop' | 'reset_fault' | 'confirm_manual_power_cut' |
                         'demo_drive_start' | 'demo_drive_stop') => {
    if (status?.source !== 'REAL_KART' || status.mode !== 'LIVE' || status.safety_requests_enabled !== true) { setRequestResult('Live safety request forwarding unavailable'); return; }
    const socket = new WebSocket(`${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}/mission/ws`);
    socket.onopen = () => { socket.send(JSON.stringify({ type: 'request', action })); setRequestResult(`${action} requested`); window.setTimeout(() => socket.close(), 1000); };
    socket.onerror = () => { setRequestResult('Request failed'); socket.close(); };
    socket.onmessage = (event) => { try { if (JSON.parse(event.data).type === 'request_rejected') setRequestResult(`Request rejected: ${action}`); } catch { /* ignore telemetry */ } };
  };
  useEffect(() => {
    let active = true;
    let busy = false;
    const refresh = async () => {
      if (busy) return;
      busy = true;
      try {
        const [statusResponse, cameraResponse, readinessResponse] = await Promise.all([
          fetch('/mission/api/status', { cache: 'no-store' }),
          fetch('/mission/api/camera', { cache: 'no-store' }),
          fetch('/mission/api/readiness', { cache: 'no-store' }),
        ]);
        if (!statusResponse.ok || !cameraResponse.ok) throw new Error(`HTTP ${statusResponse.status}/${cameraResponse.status}`);
        const [nextStatus, nextCamera] = await Promise.all([statusResponse.json(), cameraResponse.json()]);
        // Readiness is best-effort display only: a failure here never blocks or fabricates the
        // rest of this panel's real gateway/camera data.
        const nextReadiness = readinessResponse.ok ? await readinessResponse.json() : { available: false, reason: `HTTP ${readinessResponse.status}` };
        if (active) { setStatus(nextStatus); setCamera(nextCamera); setReadiness(nextReadiness); setError(''); }
      } catch (reason) {
        if (active) { setStatus(null); setCamera(null); setError(reason instanceof Error ? reason.message : 'GATEWAY_UNAVAILABLE'); }
      } finally { busy = false; }
    };
    void refresh();
    const timer = window.setInterval(() => void refresh(), 500);
    return () => { active = false; window.clearInterval(timer); };
  }, []);
  const upstream = sourceOf(source);
  const sourceMatches = status?.source === source && status.mode === 'LIVE';
  const showCamera = sourceMatches && camera?.available === true && camera.source === upstream;
  const stage = (name: 'perception' | 'mapping' | 'planning') => {
    const value = status?.stages[name];
    const payload = value?.payload;
    const qualified = value?.state === 'CURRENT' && payload?.source === upstream;
    return { value, payload: qualified ? payload : null, state: qualified ? 'CURRENT' : value?.state === 'CURRENT' ? 'SOURCE MISMATCH' : value?.state ?? 'MISSING' };
  };
  const perception = stage('perception'), mapping = stage('mapping'), planning = stage('planning');
  const pipelineCurrent = status?.odometry.current && [perception, mapping, planning].every((item) => item.state === 'CURRENT');
  const planner = planning.payload;
  const decision = planner?.decision;
  const mission = planner?.mission_state as Record<string, unknown> | undefined;
  const mappingGeometry = mapping.payload?.geometry as Record<string, unknown> | undefined;
  const boundaries = mappingGeometry?.boundaries as Record<string, unknown> | undefined;
  const faults = status?.vcu.state === 'CURRENT' && Array.isArray(status.vcu.payload?.faults) ? status.vcu.payload.faults : null;
  const currentFrame = sourceMatches && status?.odometry.state === 'CURRENT' && frame?.source === source ? frame : null;
  // Front safety envelope monitor (DIAGNOSTIC_ONLY, vehicle_authority=OFF): read-only display, consumed as-is, never re-derived here.
  const envelopeChannel = sourceMatches ? status?.safety_envelope : null;
  const envelope = envelopeChannel?.state === 'CURRENT' ? envelopeChannel.payload : null;
  const envelopeState = envelope?.state ?? 'UNKNOWN';
  const envelopeDataSource = envelope?.data_source ?? 'UNVERIFIED';
  // RC-06 audit finding B: a live DDS connection is not proof of a real physical sensor --
  // the publishing node self-reports data_source, so flag anything that isn't a verified
  // match for this dashboard's own selected upstream (LIVE for REAL_KART, SIMULATED for SIMULATION).
  const envelopeNotHardwareValidated = envelope != null && envelopeDataSource !== 'LIVE';
  const envelopeSourceMismatch = envelope != null && envelopeDataSource !== upstream;
  const controllerAvailable = currentFrame?.vehicle.requested_steering_rad != null || currentFrame?.vehicle.target_speed_mps != null;
  const imageUrl = showCamera ? `/mission/api/camera/${camera.frame_id}` : '';
  const detections = showCamera ? camera.detections ?? [] : [];
  const cameraProblem = status && !sourceMatches ? 'GATEWAY_SOURCE_MISMATCH' : camera?.available && camera.source !== upstream ? 'CAMERA_SOURCE_MISMATCH' : camera?.reason === 'CAMERA_PAIR_STALE' || camera?.reason === 'CAMERA_PAIR_MISSING' ? 'CAMERA DISCONNECTED' : camera?.reason ?? (error || 'CAMERA DISCONNECTED');
  return <>
    <aside className="mc-ros-camera" aria-label="Camera and YOLO diagnostics">
      <header><b>{source === 'REAL_KART' ? 'LIVE KART' : 'SIMULATED ROS'}</b><span>{error || (!sourceMatches ? 'SOURCE MISMATCH' : pipelineCurrent ? 'ROS DATA CURRENT' : status.ros_connected ? 'PARTIAL ROS DATA' : 'WAITING FOR ROS')}</span></header>
      <section className="mc-rgb"><h2>RGB CAMERA · {showCamera ? camera.source : 'NO CURRENT FRAME'}</h2>
        {showCamera ? <div className="mc-camera-frame"><img src={`${imageUrl}/rgb`} alt="Timestamped perception RGB" /><svg viewBox={`0 0 ${camera.width} ${camera.height}`} preserveAspectRatio="none" aria-label="YOLO detections">{detections.filter((item) => Array.isArray(item.box) && item.box.length === 4).map((item, index) => { const [x1, y1, x2, y2] = item.box!; return <g key={`${item.id ?? index}-${index}`}><rect x={x1} y={y1} width={Math.max(0, x2 - x1)} height={Math.max(0, y2 - y1)} /><text x={x1 + 3} y={Math.max(13, y1 - 4)}>#{text(item.id)} {text(item.label)} {number(item.confidence, 2)}</text></g>; })}</svg><SafetyEnvelopeOverlay envelope={envelope} width={camera.width ?? 640} height={camera.height ?? 480} /></div> : <p>{cameraProblem}</p>}
        <dl><dt>FRAME / SEQUENCE</dt><dd>{showCamera ? `${camera.frame_id} / ${text(camera.sequence)}` : '—'}</dd><dt>IMAGE FRAME</dt><dd>{showCamera ? text(camera.image_frame) : '—'}</dd><dt>RESOLUTION</dt><dd>{showCamera ? `${camera.width} × ${camera.height}` : '—'}</dd><dt>FPS</dt><dd>{showCamera ? number(camera.receive_fps) : '—'}</dd><dt>MISSING FRAMES</dt><dd>{showCamera ? text(camera.missing_frames) : '—'}</dd><dt>MODEL</dt><dd>{showCamera ? `${text(camera.model_version)} · ${text(camera.model_sha256).slice(0, 12)} · ${text(camera.model_backend)} · ${text(camera.model_stage)}` : 'UNKNOWN'}</dd><dt>INFERENCE</dt><dd>{showCamera ? `${number(camera.inference_ms)} ms` : '—'}</dd><dt>RGB / DEPTH STAMP</dt><dd>{showCamera ? `${text(camera.rgb_timestamp_ms)} / ${text(camera.depth_timestamp_ms)} ms · seq ${text(camera.rgb_sequence)}/${text(camera.depth_sequence)}` : '—'}</dd><dt>LATCHED FAULT</dt><dd className={camera?.latched_fault ? 'mc-alert' : ''}>{showCamera ? camera.latched_fault ? `${camera.latched_fault} · operator reset: ros2 service call /autokart_perception/reset_sensor_fault std_srvs/srv/Trigger` : 'NONE' : '—'}</dd></dl>
        <h2>YOLO26s DETECTIONS</h2>{showCamera && detections.length ? <ul className="mc-detections">{detections.map((item, index) => { const detail = camera.cone_details?.find((entry) => entry.id === item.id); const metricQualified = camera.valid === true && camera.depth_qualified === true && detail?.localization_status === 'METRIC_OK' && Array.isArray(detail.validation_reasons) && detail.validation_reasons.length === 0; return <li key={`${item.id ?? index}-${index}`} className={metricQualified ? 'accepted' : 'rejected'}>#{text(item.id)} {text(item.label)} · {number(item.confidence, 2)} · {metricQualified ? `QUALIFIED · X ${number((detail.ground_xy_m as number[] | undefined)?.[0], 2)} m · Y ${number((detail.ground_xy_m as number[] | undefined)?.[1], 2)} m · ±${number(detail.uncertainty_m, 2)} m · ${number(detail.distance_m, 2)} m range (base_link)` : `REJECTED · ${detail?.localization_status === 'METRIC_OK' ? 'UNVERIFIED METRIC CANDIDATE' : text(detail?.localization_status ?? 'NO DEPTH DETAIL')}${Array.isArray(detail?.validation_reasons) && detail.validation_reasons.length ? ` · ${detail.validation_reasons.join(', ')}` : ''}`}</li>; })}</ul> : <p>{showCamera ? 'NO DETECTIONS IN THIS FRAME' : 'DETECTIONS UNAVAILABLE'}</p>}
      </section>
    </aside>
    <aside className="mc-ros-live" aria-label="ROS live diagnostics">
    {source === 'REAL_KART' && <section className="mc-controls" aria-label="Manual power cut, arming and software stop">
      <p style={{ background: '#714b00', color: '#ffe0a0', padding: 12, fontWeight: 800 }}>E-STOP INPUT: NOT FITTED - MANUAL POWER CUT ONLY</p>
      <button type="button" onClick={() => requestSafety('confirm_manual_power_cut')}>Confirm physical power cut within reach for next ARM</button>
      {/* This button only sends the whitelisted 'arm' request; autokart_vcu_safety alone decides
          whether every precondition holds and may refuse it (surfaced below via request_rejected).
          The dashboard does not compute or display an arm-allowed gate of its own. */}
      <button type="button" onClick={() => requestSafety('arm')}>Request ARM (refused unless every gate and the manual power cut hold)</button>
      <button type="button" onClick={() => requestSafety('reset_fault')}>RESET FAULT</button>
      <button type="button" onClick={() => requestSafety('stop')} style={{ background: '#a32121', color: 'white', fontWeight: 900, padding: 12 }}>SOFTWARE STOP - NOT AN EMERGENCY STOP</button>
      <span role="status">{requestResult}</span>
    </section>}
      <section className="mc-depth"><h2>DEPTH · {showCamera ? camera.source : 'NO CURRENT FRAME'}</h2>
        {showCamera ? <img className="mc-depth-frame" src={`${imageUrl}/depth`} alt="RAW16 depth diagnostic visualization" /> : <p>{cameraProblem}</p>}
        <dl><dt>CAPTURE TIME</dt><dd>{showCamera ? `${camera.capture_time_ns} ns` : '—'}</dd><dt>FRAME AGE</dt><dd>{showCamera ? `${number(camera.age_ms, 0)} ms` : '—'}</dd><dt>CAPTURE TO GATEWAY</dt><dd>{showCamera ? `${number(camera.capture_to_gateway_ms)} ms` : '—'}</dd><dt>PROCESSING</dt><dd>{showCamera ? `${number(camera.processing_ms)} ms` : '—'}</dd><dt>CALIBRATION</dt><dd>{showCamera ? text(camera.calibration_status) : 'UNKNOWN'}</dd><dt>DEPTH UNIT</dt><dd>{showCamera ? `${text(camera.depth_stats?.raw_unit)} · ${number(camera.depth_stats?.valid_percent)}% pixels` : 'UNQUALIFIED'}</dd><dt>QUALIFICATION</dt><dd>{showCamera ? `${camera.valid ? 'QUALIFIED CONES PUBLISHED' : 'DIAGNOSTIC ONLY'} · DEPTH ${camera.depth_qualified ? 'QUALIFIED' : 'UNQUALIFIED'}` : 'NO CAMERA DATA'}</dd><dt>REASONS</dt><dd>{showCamera ? camera.reason_codes?.join(', ') || '—' : '—'}</dd></dl>
      </section>
    <section className="mc-calibration"><h2>CAMERA CALIBRATION · {showCamera ? calibrationState(camera) : 'UNKNOWN'}</h2><dl>
      <dt>CAMERA CALIBRATION</dt><dd className={showCamera && calibrationState(camera) === 'FAILED' ? 'mc-alert' : ''}>{showCamera ? calibrationState(camera) : 'UNKNOWN'}</dd>
      <dt>POSITION ERROR (HELD-OUT)</dt><dd>{showCamera ? errorText(camera.calibration_artifact?.held_out_error_m) : '—'}</dd>
      <dt>POSITION ERROR (SURVEY)</dt><dd>{showCamera ? errorText(camera.calibration_artifact?.survey_error_m) : '—'}{showCamera && camera.calibration_artifact?.tolerance_m != null ? ` · tolerance ${number(camera.calibration_artifact.tolerance_m, 3)} m` : ''}</dd>
      <dt>DEPTH SCALE</dt><dd>{showCamera ? camera.depth_scale_verified ? `VERIFIED · factor ${number(camera.depth_scale_factor, 4)}` : 'UNVERIFIED · vendor RAW16 × 0.001' : '—'}</dd>
      <dt>CAMERA MOUNT</dt><dd>{showCamera ? camera.mount_verified ? 'VERIFIED' : 'UNVERIFIED · nominal operator values' : '—'}</dd>
      <dt>CALIBRATION REASONS</dt><dd>{showCamera ? camera.calibration_reasons?.join(', ') || (camera.mount_verified ? '—' : 'NO VERIFIED ARTIFACT LOADED') : '—'}</dd>
    </dl></section>
    <section className="mc-envelope" aria-label="Front safety envelope monitor, diagnostic only">
      <h2>{ENVELOPE_BADGE[envelopeState]} FRONT SAFETY ENVELOPE · {envelopeState}</h2>
      {envelopeNotHardwareValidated && <p className="mc-alert" role="alert">
        {envelopeDataSource === 'UNVERIFIED' ? 'DATA SOURCE UNVERIFIED' : `DATA SOURCE: ${envelopeDataSource}`} — NOT A PHYSICAL SENSOR READING. Do not treat these distances as hardware-validated.
      </p>}
      <dl>
        <dt>DATA SOURCE</dt><dd className={envelopeNotHardwareValidated ? 'mc-alert' : ''}>
          {envelopeDataSource}{envelopeSourceMismatch ? ` · MISMATCH (dashboard expects ${upstream})` : ''}
        </dd>
        <dt>STATE</dt><dd style={{ color: ENVELOPE_COLOR[envelopeState] }}>{envelopeState}{envelope && !envelope.valid ? ' · INVALID' : ''}</dd>
        <dt>NEAREST OBSTACLE CLEARANCE</dt><dd>{envelope?.clear_distance_m != null ? `${envelope.clear_distance_m.toFixed(2)} m from bumper` : envelopeState === 'CLEAR' ? 'NO OBSTACLE IN CORRIDOR' : '—'}</dd>
        <dt>BRAKING TRIGGER DISTANCE</dt><dd>{envelope?.stopping_distance_m != null ? `${envelope.stopping_distance_m.toFixed(2)} m · provisional, not a measured braking test` : '—'}</dd>
        <dt>VEHICLE SPEED</dt><dd>{currentFrame ? `${number(currentFrame.vehicle.speed_mps, 2)} m/s` : 'UNKNOWN'}</dd>
        <dt>/safety/envelope FRESHNESS</dt><dd className={envelopeChannel?.state !== 'CURRENT' ? 'mc-alert' : ''}>{envelopeChannel ? `${envelopeChannel.state} · ${number(envelopeChannel.age_ms, 0)} ms` : 'UNKNOWN · TOPIC NOT PUBLISHING'}</dd>
        <dt>CALIBRATION STATUS</dt><dd className={envelope && !envelope.calibration_verified ? 'mc-alert' : ''}>{envelope ? envelope.calibration_verified ? 'VERIFIED' : 'UNVERIFIED · schematic overlay only' : 'UNKNOWN'}</dd>
        <dt>REASON</dt><dd>{envelope?.reason ?? '—'}</dd>
        <dt>VEHICLE AUTHORITY</dt><dd>OFF · DIAGNOSTIC_ONLY</dd>
      </dl>
    </section>
    <section className="mc-pipeline"><h2>ROS PIPELINE · READ ONLY</h2><dl>
      <dt>ODOMETRY</dt><dd>{sourceMatches ? `${status.odometry.state}${status.odometry.state === 'MISSING' ? ' · NO /autokart/odom PRODUCER' : ''} · ${number(status.odometry.age_ms, 0)} ms` : 'UNKNOWN'}</dd>
      <dt>GEOMETRY</dt><dd>{mapping.state === 'CURRENT' && mapping.payload?.valid === true ? 'QUALIFIED MAP CURRENT' : 'WAITING FOR QUALIFIED GEOMETRY'}</dd>
      <dt>VCU CONNECTIVITY</dt><dd>{sourceMatches ? status.vcu.state : 'UNKNOWN'}</dd>
      <dt>PERCEPTION</dt><dd>{perception.state} · {text(perception.payload?.frame_id)}</dd>
      <dt>MAPPING / GEOMETRY</dt><dd>{mapping.state} · {text(mapping.payload?.tracked_count)} tracks</dd>
      <dt>BOUNDARIES</dt><dd>{mapping.state === 'CURRENT' ? text(boundaries?.status) : 'UNAVAILABLE'}</dd>
      <dt>PLANNER</dt><dd>{planning.state} · {text(decision)}</dd>
      <dt>ACTIVE BUMPER STOP GAP</dt><dd>{planning.state === 'CURRENT' ? `${number(planning.payload?.stop_gap_from_bumper_m, 2)} m · COMMISSIONING` : 'UNKNOWN'}</dd>
      <dt>RED ZONE</dt><dd>{planning.state === 'CURRENT' ? `${number(planning.payload?.red_zone_m, 2)} m · FRONT BUMPER` : 'UNKNOWN'}</dd>
      <dt>ROUTE VALIDITY</dt><dd>{planner ? text(planner.valid) : 'UNKNOWN'}</dd>
      <dt>MISSION / LAPS</dt><dd>{mission ? `${text(mission.state)} · R${text(mission.right_laps)} L${text(mission.left_laps)}` : 'UNKNOWN'}</dd>
      <dt>POSE / HEADING</dt><dd>{currentFrame ? `${number(currentFrame.vehicle.x_m)}, ${number(currentFrame.vehicle.y_m)} m · ${number(currentFrame.vehicle.yaw_rad)} rad` : 'WAITING FOR ODOMETRY'}</dd>
      <dt>LOOKAHEAD</dt><dd>{currentFrame?.planning.lookahead ? `${number(currentFrame.planning.lookahead.x_m)}, ${number(currentFrame.planning.lookahead.y_m)} m` : 'NO CURRENT TARGET'}</dd>
      <dt>LOOKAHEAD DISTANCE</dt><dd>{currentFrame?.planning.lookahead ? `${number(Math.hypot(currentFrame.planning.lookahead.x_m - currentFrame.vehicle.x_m, currentFrame.planning.lookahead.y_m - currentFrame.vehicle.y_m))} m · GEOMETRIC` : 'UNAVAILABLE'}</dd>
      <dt>CROSS TRACK ERROR</dt><dd>{currentFrame?.metrics.cross_track_error_m != null ? `${number(currentFrame.metrics.cross_track_error_m)} m` : 'UNAVAILABLE'}</dd>
      <dt>CONTROLLER</dt><dd>{controllerAvailable ? 'REQUEST OBSERVED · READ ONLY' : 'CONTROLLER UNAVAILABLE'}</dd>
      <dt>SPEED REQUEST / MEASURED</dt><dd>{currentFrame ? `${number(currentFrame.vehicle.target_speed_mps)} / ${number(currentFrame.vehicle.speed_mps)} m/s` : '—'}</dd>
      <dt>STEER REQUEST / MEASURED</dt><dd>{currentFrame ? `${number(currentFrame.vehicle.requested_steering_rad)} / ${number(currentFrame.vehicle.measured_steering_rad)} rad` : '—'}</dd>
      <dt>BRAKE REQUEST / FEEDBACK</dt><dd>{currentFrame ? `${number(currentFrame.vehicle.brake_command)} / ${number(currentFrame.vehicle.brake_feedback)}` : '—'}</dd>
      <dt>OBSTACLE / STOP</dt><dd>{currentFrame ? `${currentFrame.safety.obstacle?.brake_trigger ? 'BRAKE' : text(currentFrame.safety.state)}` : 'UNKNOWN'}</dd>
      <dt>FAULTS / SAFETY</dt><dd>{faults ? faults.join(', ') || 'NONE REPORTED' : 'UNKNOWN'}</dd>
      <dt>NODE AGE</dt><dd>P {number(perception.value?.age_ms, 0)} / M {number(mapping.value?.age_ms, 0)} / L {number(planning.value?.age_ms, 0)} ms</dd>
      <dt>VEHICLE AUTHORITY</dt><dd>DISABLED</dd>
    </dl></section>
    <section className="mc-readiness" aria-label="Readiness gates (tools/readiness_report.py)">
      <h2>READINESS GATES · {readiness?.available ? (readiness.ready ? 'READY FOR FIRST RUN' : 'NOT READY') : 'NO DATA'}</h2>
      {!readiness?.available && <p>{readiness?.reason === 'READINESS_REPORT_NOT_GENERATED' ?
        'NOT RUN · run: python3 tools/readiness_report.py' : 'NO DATA'}</p>}
      {readiness?.available && <>
        <p>Generated {number(readiness.age_s, 0)} s ago</p>
        <table><thead><tr><th>Gate</th><th>Status</th></tr></thead><tbody>
          {(readiness.gates ?? []).map((gate) => <tr key={gate.name}><td>{gate.name}</td>
            <td className={gate.status === 'PASS' ? '' : 'mc-alert'}>{gate.status}</td></tr>)}
        </tbody></table>
      </>}
    </section>
    {source === 'REAL_KART' && <section className="mc-demo-drive" aria-label="DEMO DRIVE - real motion, not autonomy">
      <h2>DEMO DRIVE — REAL MOTION, NOT AUTONOMY</h2>
      <p>Separate from the camera-guided mission. Firmware enforces the throttle cap regardless of
        this UI. The backend (not just this browser tab) sends the drive heartbeat; losing it,
        losing arm, or an E-stop forces the VCU safe (SV 0 V, BRK applied, EN stop).</p>
      <label><input type="checkbox" checked={areaClear} onChange={(event) => setAreaClear(event.target.checked)} />
        Area clear, physical E-stop reachable</label>
      <div>
        <button type="button" onClick={() => requestSafety('demo_drive_start')} disabled={!areaClear}>
          Arm demo drive</button>
        <button type="button" onClick={() => requestSafety('demo_drive_stop')}>End demo drive</button>
      </div>
      <button type="button"
        disabled={!areaClear || !wsConnected}
        onMouseDown={() => {
          if (!sendDemoDrive) return;
          sendDemoDrive({ action: 'drive', direction: 'forward', duration_s: 2.0 });
          holdTimer.current = window.setInterval(() => sendDemoDrive({ action: 'ping' }), 150);
          setDriveResult('holding');
        }}
        onMouseUp={() => {
          if (holdTimer.current != null) { window.clearInterval(holdTimer.current); holdTimer.current = null; }
          sendDemoDrive?.({ action: 'stop' });
          setDriveResult('released');
        }}
        onMouseLeave={() => {
          if (holdTimer.current != null) { window.clearInterval(holdTimer.current); holdTimer.current = null; }
          sendDemoDrive?.({ action: 'stop' });
        }}>HOLD TO DRIVE (forward, capped, releases to stop)</button>
      <button type="button" style={{ background: '#a32121', color: 'white', fontWeight: 900, padding: 12 }}
        onClick={() => {
          if (holdTimer.current != null) { window.clearInterval(holdTimer.current); holdTimer.current = null; }
          sendDemoDrive?.({ action: 'stop' });
          setDriveResult('stopped');
        }}>STOP DEMO DRIVE (always works)</button>
      <span role="status">{driveResult} {demoDriveRejected ? `· rejected: ${demoDriveRejected}` : ''}</span>
    </section>}
    </aside>
  </>;
}
