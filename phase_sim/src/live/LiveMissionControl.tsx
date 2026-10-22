import { memo, useEffect, useRef, useState } from 'react';
import './liveMissionControl.css';

// Read-only production view of live_mission.py (/api/snapshot + /api/image/<frame>/<rgb|depth>).
// There is deliberately no source/mode switcher and no route that could command a vehicle.

type Point = [number, number];
type Cone = { id: number; label: string; confidence: number; xy: Point; distance_m: number; sigma_m: number };
type Gate = { left_id: number; right_id: number; midpoint: Point; status: string; width?: number; quality?: number; id?: number };
type Snapshot = {
  schema_version: number; frame_id: string | null; source: string; status: string; reasons: string[];
  capture_timestamp_ns?: number; camera_sequence?: number; depth_sequence?: number; camera_id?: string;
  calibration_status?: string; depth_scale_qualified?: boolean; base_frame?: string;
  depth_scale_provenance?: string; rgb_depth_pair?: string; rgb_depth_alignment?: string | null;
  rgb_depth_timestamp_delta_ms?: number; algorithm_version?: string;
  depth_stats?: { valid_pixels: number; invalid_pixels: number; valid_percent: number; median_raw: number | null;
    p5_raw?: number | null; p95_raw?: number | null; raw_unit: string };
  model?: { name: string; supported_classes: string[]; qualification: string };
  detections: { id: number; label: string; confidence: number; box: number[] }[];
  observations: Cone[]; candidate_observations?: Cone[];
  provisional_3d_observations?: { id: number; label: string; confidence: number;
    nominal_xy_m: Point | null; nominal_body_xyz_m: [number, number, number] | null;
    optical_xyz_m: [number, number, number]; uncertainty_m: number;
    ground_contact_status: string; position_kind: string }[];
  rejected_candidates?: { id: number; localization_status: string; box: number[] }[];
  tracked_cones?: unknown[]; tracking_status?: string;
  camera_motion?: { status: string; previous_timestamp_ns: number; current_timestamp_ns: number;
    matched_points: number; inlier_count: number; reprojection_rms_px: number | null;
    qualified: false; frame: 'CAMERA_OPTICAL' } | null;
  geometry: { status?: string; delaunay_edges: [number, number][]; delaunay_triangles?: [number, number, number][];
    gabriel_edges: [number, number][];
    boundaries: { left_ids: number[]; right_ids: number[]; mode?: string; status?: string; confidence?: number; reason?: string };
    gates: Gate[]; centerline: Point[]; centerline_status?: string; curvature_status?: string;
    topology_reasons?: string[] };
  planning: { path_valid: boolean; local_path: Point[]; path_frame?: string; speed_profile_provenance?: string };
  control_candidate: { valid: boolean; target_speed_mps: number; steering_angle_rad: number;
    target_point_base_m?: Point | null; published_to_vcu?: boolean };
  mission_state?: { state?: string };
  perception?: { status?: string; safety_state?: string; safety_reasons?: string[]; provenance?: string;
    diagnostics?: { cone_details?: { id: number; localization_status: string; depth_status: string; distance_m?: number }[] } };
  detector_counts?: { raw_output_rows?: number; above_confidence?: number; valid_boxes?: number;
    after_nms?: number; application_detections?: number; depth_localized_candidates?: number;
    validated_metric?: number };
  stream_metrics?: { camera_callback_fps?: number | null; received_frames?: number;
    mailbox_overwrites?: number; mailbox_capacity?: number; processed_fps?: number | null;
    inference_fps?: number | null };
  latencies_ms?: Record<string, number | null>; last_good_age_ms?: number | null;
  frame_age_ms?: number | null;
  // Typed as boolean (not `false`) so an unexpected `true` from the backend is surfaced, never hidden.
  vehicle_authorized: boolean; ready: boolean; camera_streaming?: boolean; pose_status?: string;
};
type View = { snapshot: Snapshot; displayedFrame: Snapshot | null; rgb: string | null;
  depth: string | null; snapshotReceivedAtMs: number | null };
type Layer = 'cones' | 'candidates' | 'rejected' | 'triangles' | 'delaunay' | 'gabriel' | 'boundaries' | 'gates' | 'centerline' | 'trajectory';
type Tone = 'ok' | 'info' | 'sim' | 'warn' | 'alert' | 'fault' | 'idle';
type Sample = { frame: string; sdkMs: number | null; snapshotMs: number | null; processedFps: number | null;
  ageMs: number | null; cones: number };

const initialLayers: Record<Layer, boolean> = { cones: true, candidates: true, rejected: true, triangles: true,
  delaunay: true, gabriel: true, boundaries: true, gates: true, centerline: true, trajectory: true };
const layerMeta: Record<Layer, { label: string; swatch: string }> = {
  cones: { label: 'Cones', swatch: 'cone' }, candidates: { label: 'Unverified candidates', swatch: 'candidate' },
  rejected: { label: 'Rejected list', swatch: 'rejected' }, triangles: { label: 'Delaunay faces', swatch: 'face' },
  delaunay: { label: 'Delaunay edges', swatch: 'delaunay' }, gabriel: { label: 'Gabriel edges', swatch: 'gabriel' },
  boundaries: { label: 'Boundaries L/R', swatch: 'boundary' }, gates: { label: 'Gates', swatch: 'gate' },
  centerline: { label: 'Centerline', swatch: 'centerline' }, trajectory: { label: 'Path + target', swatch: 'trajectory' },
};
const coneColor: Record<string, string> = { blue: '#4f9dff', yellow: '#f5d04c', orange: '#ff9442',
  large_orange: '#ff6a3d', unknown: '#a8b0ba' };
const HISTORY = 90;
const empty: Snapshot = { schema_version: 1, frame_id: null, source: 'CONNECTING', status: 'WAITING',
  reasons: [], detections: [], observations: [], geometry: { delaunay_edges: [], gabriel_edges: [],
    boundaries: { left_ids: [], right_ids: [] }, gates: [], centerline: [] },
  planning: { path_valid: false, local_path: [] },
  control_candidate: { valid: false, target_speed_mps: 0, steering_angle_rad: 0 },
  vehicle_authorized: false, ready: false };

// Every SDK health value live_mission.py / sdk.py can publish, plus the two the client derives.
const healthTone = (status: string): Tone => ({ VALID: 'ok', DEGRADED: 'warn', AMBIGUOUS: 'warn',
  INVALID_GEOMETRY: 'alert', STALE: 'alert', SENSOR_FAULT: 'fault', WAITING: 'idle' } as Record<string, Tone>)[status] ?? 'fault';
const sourceTone = (source: string): Tone => ({ LIVE: 'ok', REPLAY: 'info', SIMULATED: 'sim',
  CONNECTING: 'idle' } as Record<string, Tone>)[source] ?? 'fault';
const finite = (value: number | null | undefined): value is number => typeof value === 'number' && Number.isFinite(value);

const MetricView = memo(function MetricView({ snapshot, layers }: { snapshot: Snapshot; layers: Record<Layer, boolean> }) {
  const geometry = snapshot.geometry;
  const qualifiedMetricAvailable = snapshot.source !== 'SIMULATED' &&
    snapshot.depth_scale_qualified === true &&
    snapshot.calibration_status === 'MOUNT_VERIFIED';
  // Synthetic SDK coordinates have a known simulated base_link, but do not qualify HP60C depth or mount.
  const syntheticMetricAvailable = snapshot.source === 'SIMULATED' && snapshot.base_frame === 'base_link';
  const graphAvailable = qualifiedMetricAvailable || syntheticMetricAvailable;
  const points = graphAvailable ? [...snapshot.observations.map(cone => cone.xy),
    ...geometry.centerline, ...snapshot.planning.local_path].filter(point =>
    Number.isFinite(point[0]) && Number.isFinite(point[1])) : [];
  // Fixed base_link viewport: observations change position without rescaling the scene.
  const forward = 8, lateral = 6, scale = 50;
  const outsideView = points.filter(([x, y]) => x < 0 || x > forward || Math.abs(y) > lateral).length;
  const xy = ([x, y]: Point): Point => [320 - y * scale, 440 - x * scale];
  const byId = new Map(snapshot.observations.map(cone => [cone.id, cone]));
  const edge = ([a, b]: [number, number], kind: string, index: number) => {
    const ca = byId.get(a), cb = byId.get(b);
    if (!ca || !cb) return null;
    const [x1, y1] = xy(ca.xy), [x2, y2] = xy(cb.xy);
    return <line key={`${kind}-${index}`} x1={x1} y1={y1} x2={x2} y2={y2} className={`live-${kind}`} />;
  };
  const line = (points: Point[]) => points.map(point => xy(point).join(',')).join(' ');
  const boundary = (ids: number[]) => line(ids.map(id => byId.get(id)?.xy).filter((p): p is Point => !!p));
  const arc = (r: number) => { // Range ring as a path (not <circle>) so cones stay the first circles.
    const px = r * scale, [ox, oy] = xy([0, 0]);
    return `M ${ox - px} ${oy} A ${px} ${px} 0 0 1 ${ox + px} ${oy}`;
  };
  const target = snapshot.control_candidate.target_point_base_m;
  return <svg viewBox="0 0 640 480" role="img" aria-label={syntheticMetricAvailable && !qualifiedMetricAvailable ?
    "Bird's-eye simulated SDK geometry; camera calibration unverified" : qualifiedMetricAvailable ?
    "Bird's-eye qualified metric geometry" : "Bird's-eye diagnostic grid; metric geometry unavailable"}>
    <rect width="640" height="480" fill="#071320" />
    {Array.from({ length: Math.ceil(2 * lateral) + 1 }, (_, i) => i - Math.ceil(lateral)).map(y =>
      <g key={`y-${y}`}><line className={y === 0 ? 'live-grid-axis' : 'live-grid-line'} x1={xy([0, y])[0]} y1="0" x2={xy([0, y])[0]} y2="480" />
        {y % 2 === 0 && <text x={xy([0, y])[0]} y="474" textAnchor="middle" className="live-axis-label">{y === 0 ? 'Y 0' : `${y > 0 ? '+' : ''}${y}`}</text>}</g>)}
    {Array.from({ length: Math.ceil(forward) + 1 }, (_, x) =>
      <g key={`x-${x}`}><line className={x === 0 ? 'live-grid-axis' : 'live-grid-line'} x1="0" y1={xy([x, 0])[1]} x2="640" y2={xy([x, 0])[1]} />
        <text x="8" y={Math.max(14, xy([x, 0])[1] - 4)} className="live-axis-label">{x} m</text></g>)}
    {[2, 4, 6, 8].map(r => <path key={`ring-${r}`} d={arc(r)} className="live-range-ring" />)}
    {graphAvailable && layers.triangles && (geometry.delaunay_triangles ?? []).map((tri, i) => {
      const corners = tri.map(id => byId.get(id)?.xy);
      if (corners.some(p => !p)) return null;
      return <polygon key={`tri-${i}`} points={line(corners as Point[])} className="live-face" />;
    })}
    {graphAvailable && layers.boundaries && <><polyline points={boundary(geometry.boundaries.left_ids)} className="live-left" />
      <polyline points={boundary(geometry.boundaries.right_ids)} className="live-right" /></>}
    {graphAvailable && layers.delaunay && geometry.delaunay_edges.map((pair, i) => edge(pair, 'delaunay', i))}
    {graphAvailable && layers.gabriel && geometry.gabriel_edges.map((pair, i) => edge(pair, 'gabriel', i))}
    {graphAvailable && layers.gates && geometry.gates.map((gate, i) => edge([gate.left_id, gate.right_id], 'gate', i))}
    {graphAvailable && layers.gates && geometry.gates.map((gate, i) => {
      if (!Number.isFinite(gate.midpoint?.[0]) || !Number.isFinite(gate.midpoint?.[1])) return null;
      const [mx, my] = xy(gate.midpoint);
      return <g key={`gate-mid-${i}`}><path d={`M ${mx} ${my - 5} L ${mx + 5} ${my} L ${mx} ${my + 5} L ${mx - 5} ${my} Z`} className="live-gate-mid" />
        {finite(gate.width) && <text x={mx + 8} y={my + 14} className="live-gate-label">G{gate.id ?? i} · {gate.width.toFixed(2)} m</text>}</g>;
    })}
    {graphAvailable && layers.centerline && geometry.centerline.length > 1 && <polyline points={line(geometry.centerline)} className="live-centerline" />}
    {graphAvailable && layers.trajectory && snapshot.planning.path_valid && snapshot.planning.local_path.length > 1 &&
      <polyline points={line(snapshot.planning.local_path)} className="live-trajectory" />}
    {graphAvailable && layers.trajectory && snapshot.control_candidate.valid && target && (() => {
      const [ox, oy] = xy([0, 0]), [tx, ty] = xy(target);
      return <line x1={ox} y1={oy} x2={tx} y2={ty} className="live-lookahead" />;
    })()}
    {graphAvailable && layers.cones && snapshot.observations.map(cone => { const [cx, cy] = xy(cone.xy); return <g key={cone.id}>
      <title>#{cone.id} {cone.label} · ({cone.xy[0].toFixed(2)}, {cone.xy[1].toFixed(2)}) m · {(100 * cone.confidence).toFixed(0)}% · σ {cone.sigma_m.toFixed(2)} m</title>
      <circle cx={cx} cy={cy} r={cone.label === 'large_orange' ? 11 : 9} fill={coneColor[cone.label] ?? coneColor.unknown} className="live-cone" />
      <text x={cx + 13} y={cy - 8} className="live-cone-label">#{cone.id}</text></g>; })}
    {graphAvailable && layers.trajectory && snapshot.control_candidate.valid && target && (() => {
      const [cx, cy] = xy(target);
      return <circle cx={cx} cy={cy} r="10" className="live-target" />;
    })()}
    {!graphAvailable && layers.candidates && snapshot.candidate_observations?.map(cone => {
      if (!Number.isFinite(cone.xy[0]) || !Number.isFinite(cone.xy[1])) return null;
      const [cx, cy] = xy(cone.xy);
      return <g key={`candidate-${cone.id}`}><title>#{cone.id} unverified ground candidate ({cone.xy[0].toFixed(2)}, {cone.xy[1].toFixed(2)}) m</title>
        <circle cx={cx} cy={cy} r="9" fill="none" stroke="#ffd284" strokeWidth="2" strokeDasharray="3 3" /></g>;
    })}
    {!graphAvailable && layers.candidates && snapshot.provisional_3d_observations?.map(cone => {
      if (!cone.nominal_xy_m || !Number.isFinite(cone.nominal_xy_m[0]) || !Number.isFinite(cone.nominal_xy_m[1])) return null;
      const [cx, cy] = xy(cone.nominal_xy_m);
      const radius = Math.min(70, Math.max(8, cone.uncertainty_m * scale));
      return <g key={`body-${cone.id}`} aria-label={`Cone ${cone.id} provisional body position; ground contact unavailable`}>
        <title>#{cone.id} provisional body ({cone.nominal_xy_m[0].toFixed(2)}, {cone.nominal_xy_m[1].toFixed(2)}) m</title>
        <circle cx={cx} cy={cy} r={radius} fill="none" stroke="#e9a548" strokeOpacity=".6" strokeDasharray="5 4" />
        <path d={`M ${cx} ${cy - 8} L ${cx + 8} ${cy} L ${cx} ${cy + 8} L ${cx - 8} ${cy} Z`}
          fill="#e9a548" stroke="#583b1f" strokeWidth="2" />
      </g>;
    })}
    <path d="M 320 422 L 310 442 L 330 442 Z" className="live-ego" />
    <text x="336" y="456" className="live-origin-label">{syntheticMetricAvailable ? 'simulated base_link origin' : qualifiedMetricAvailable ? 'base_link origin' : 'camera-ground nominal origin'}</text>
    <g transform="translate(560 452)"><line x1="0" y1="0" x2={scale} y2="0" className="live-scale-bar" />
      <line x1="0" y1="-4" x2="0" y2="4" className="live-scale-bar" /><line x1={scale} y1="-4" x2={scale} y2="4" className="live-scale-bar" />
      <text x={scale / 2} y="-7" textAnchor="middle" className="live-axis-label">1 m</text></g>
    {!graphAvailable && <g><rect x="88" y="37" width="464" height="82" rx="10" fill="#162b3b" fillOpacity=".9" stroke="#a67c35" />
      <text x="320" y="65" textAnchor="middle" fill="#ffd284" fontSize="17" fontWeight="700">VALIDATED METRIC VIEW UNAVAILABLE</text>
      <text x="320" y="88" textAnchor="middle" fill="#c9d8e5" fontSize="12">Depth scale and full camera-to-base_link mount require survey</text>
      <text x="320" y="108" textAnchor="middle" fill="#9fb4c6" fontSize="12">{snapshot.candidate_observations?.length ?? 0} ground candidates · {snapshot.provisional_3d_observations?.length ?? 0} body samples · 0 validated</text></g>}
    {graphAvailable && !snapshot.observations.length && <text x="320" y="235" textAnchor="middle" fill="#8fa4bb" fontSize="17">No {syntheticMetricAvailable ? 'simulated' : 'qualified metric'} cone observations in this frame</text>}
    <desc>{graphAvailable ?
      `${syntheticMetricAvailable && !qualifiedMetricAvailable ? 'SIMULATED SDK' : 'QUALIFIED METRIC'} frame; ${outsideView} points outside view` : 'NOMINAL camera-ground frame'}</desc>
  </svg>;
});

function Spark({ label, unit, values, digits = 1 }: { label: string; unit: string; values: (number | null)[]; digits?: number }) {
  const real = values.filter(finite);
  const latest = values.length ? values[values.length - 1] : null;
  const w = 160, h = 34;
  const lo = real.length ? Math.min(...real) : 0, hi = real.length ? Math.max(...real) : 1;
  const span = hi - lo || Math.max(1, Math.abs(hi));
  const base = hi === lo ? lo - span / 2 : lo;
  // Gaps (null samples) break the line instead of being interpolated.
  const segments: string[] = []; let current: string[] = [];
  values.forEach((value, i) => {
    if (!finite(value)) { if (current.length) segments.push(current.join(' ')); current = []; return; }
    const x = (HISTORY - values.length + i) / (HISTORY - 1) * w; // newest sample pinned to the right edge
    current.push(`${x.toFixed(1)},${(h - 3 - ((value - base) / span) * (h - 6)).toFixed(1)}`);
  });
  if (current.length) segments.push(current.join(' '));
  const summary = real.length ? `latest ${finite(latest) ? latest.toFixed(digits) : '—'} ${unit} · min ${lo.toFixed(digits)} · max ${hi.toFixed(digits)} · ${real.length} frames` : 'no samples';
  return <div className="live-spark" title={`${label}: ${summary}`}>
    <span>{label}</span><strong>{finite(latest) ? `${latest.toFixed(digits)} ${unit}` : '—'}</strong>
    <svg viewBox={`0 0 ${w} ${h}`} preserveAspectRatio="none" aria-label={`${label} history: ${summary}`}>
      {segments.map((points, i) => <polyline key={i} points={points} className="live-spark-line" />)}
    </svg>
    <small>{real.length ? `min ${lo.toFixed(digits)} · max ${hi.toFixed(digits)}` : 'no samples yet'}</small>
  </div>;
}

export function LiveMissionControl() {
  const [view, setView] = useState<View>({ snapshot: empty, displayedFrame: null, rgb: null,
    depth: null, snapshotReceivedAtMs: null });
  const [layers, setLayers] = useState(initialLayers);
  const [paused, setPaused] = useState(false);
  const [token, setToken] = useState(() => sessionStorage.getItem('autokart-view-token') || '');
  const [error, setError] = useState('');
  const [displayStats, setDisplayStats] = useState<{ fps: number | null; latencyMs: number | null }>({
    fps: null, latencyMs: null });
  const [clockMs, setClockMs] = useState(() => performance.now());
  const [history, setHistory] = useState<Sample[]>([]);
  const [hidden, setHidden] = useState(() => typeof document !== 'undefined' && document.visibilityState === 'hidden');
  const viewRef = useRef(view);
  const acceptedTimestamp = useRef(0);
  const paintTimes = useRef<number[]>([]);
  const lastPaintedFrame = useRef<string | null>(null);
  useEffect(() => { viewRef.current = view; }, [view]);
  useEffect(() => {
    const timer = window.setInterval(() => setClockMs(performance.now()), 500);
    const onVisibility = () => setHidden(document.visibilityState === 'hidden');
    document.addEventListener('visibilitychange', onVisibility);
    return () => { window.clearInterval(timer); document.removeEventListener('visibilitychange', onVisibility); };
  }, []);
  useEffect(() => {
    if (paused) return;
    let stopped = false;
    let timer: number;
    let request: AbortController | null = null;
    const retired = new Map<number, string[]>();
    const headers: Record<string, string> = token ? { Authorization: `Bearer ${token}` } : {};
    function retire(urls: (string | null)[]) {
      const liveUrls = urls.filter((url): url is string => !!url);
      if (!liveUrls.length) return;
      const handle = window.setTimeout(() => {
        liveUrls.forEach(url => URL.revokeObjectURL(url));
        retired.delete(handle);
      }, 250);
      retired.set(handle, liveUrls);
    }
    async function refresh() {
      if (stopped) return;
      request = new AbortController();
      const timeout = window.setTimeout(() => request?.abort(), 2000);
      try {
        const response = await fetch('/api/snapshot', { headers, cache: 'no-store', signal: request.signal });
        if (!response.ok) throw new Error(response.status === 401 ? 'View token required' : `HTTP ${response.status}`);
        const snapshot = await response.json() as Snapshot;
        if (stopped) return;
        const receivedAt = performance.now();
        if (!snapshot.frame_id) {
          const old = viewRef.current;
          if (snapshot.source !== 'LIVE') retire([old.rgb, old.depth]);
          setView(current => snapshot.source === 'LIVE' ? { ...current, snapshot } :
            { snapshot, displayedFrame: null, rgb: null, depth: null, snapshotReceivedAtMs: null });
        } else if (snapshot.frame_id === viewRef.current.displayedFrame?.frame_id) {
          setView(current => ({ ...current, snapshot }));
        } else if (snapshot.capture_timestamp_ns != null &&
                   snapshot.capture_timestamp_ns <= acceptedTimestamp.current) {
          setError(`Out-of-order frame ${snapshot.frame_id} ignored`);
          return;
        } else {
          const [rgbResponse, depthResponse] = await Promise.all([
            fetch(`/api/image/${snapshot.frame_id}/rgb`, { headers, cache: 'no-store', signal: request.signal }),
            fetch(`/api/image/${snapshot.frame_id}/depth`, { headers, cache: 'no-store', signal: request.signal }),
          ]);
          if (!rgbResponse.ok || !depthResponse.ok) throw new Error('Matching frame images expired');
          const [rgbBlob, depthBlob] = await Promise.all([rgbResponse.blob(), depthResponse.blob()]);
          if (stopped) return;
          const rgb = URL.createObjectURL(rgbBlob), depth = URL.createObjectURL(depthBlob);
          try {
            await Promise.all([rgb, depth].map(async url => {
              const image = new Image(); image.src = url;
              if (typeof image.decode === 'function') await image.decode();
            }));
          } catch (cause) {
            URL.revokeObjectURL(rgb); URL.revokeObjectURL(depth);
            throw cause;
          }
          if (stopped) { URL.revokeObjectURL(rgb); URL.revokeObjectURL(depth); return; }
          if (snapshot.capture_timestamp_ns != null &&
              snapshot.capture_timestamp_ns <= acceptedTimestamp.current) {
            URL.revokeObjectURL(rgb); URL.revokeObjectURL(depth);
            setError(`Out-of-order frame ${snapshot.frame_id} ignored`);
            return;
          }
          acceptedTimestamp.current = snapshot.capture_timestamp_ns ?? acceptedTimestamp.current;
          const old = viewRef.current;
          setView({ snapshot, displayedFrame: snapshot, rgb, depth,
            snapshotReceivedAtMs: receivedAt });
          retire([old.rgb, old.depth]);
        }
        setError('');
      } catch (cause) { if (!stopped) setError(String(cause)); }
      finally {
        window.clearTimeout(timeout); request = null;
        // A single in-flight request bounds work; poll soon enough to catch each ~10 FPS SDK result.
        if (!stopped) timer = window.setTimeout(refresh, 40);
      }
    }
    void refresh();
    return () => {
      stopped = true; request?.abort(); window.clearTimeout(timer);
      retired.forEach((urls, handle) => {
        window.clearTimeout(handle); urls.forEach(url => URL.revokeObjectURL(url));
      });
      retired.clear();
    };
  }, [paused, token]);
  useEffect(() => {
    const frame = view.displayedFrame;
    if (!frame?.frame_id || frame.frame_id === lastPaintedFrame.current) return;
    // Real per-frame backend values only; nothing is interpolated between frames.
    setHistory(current => current.at(-1)?.frame === frame.frame_id ? current : [...current, {
      frame: frame.frame_id!, sdkMs: frame.latencies_ms?.sdk_total ?? null,
      snapshotMs: frame.latencies_ms?.snapshot_total ?? null,
      processedFps: frame.stream_metrics?.processed_fps ?? null,
      ageMs: frame.frame_age_ms ?? frame.latencies_ms?.capture_to_publish ?? null,
      cones: frame.observations.length + (frame.candidate_observations?.length ?? 0) }].slice(-HISTORY));
    const handle = window.requestAnimationFrame(() => {
      const now = performance.now();
      paintTimes.current = [...paintTimes.current.filter(t => now - t <= 5000), now];
      const times = paintTimes.current;
      const fps = times.length > 1 ? (times.length - 1) * 1000 / (now - times[0]) : null;
      const backendAge = frame.frame_age_ms ?? frame.latencies_ms?.capture_to_publish;
      const latencyMs = backendAge != null && view.snapshotReceivedAtMs != null ?
        backendAge + now - view.snapshotReceivedAtMs : null;
      setDisplayStats({ fps, latencyMs });
      lastPaintedFrame.current = frame.frame_id;
    });
    return () => window.cancelAnimationFrame(handle);
  }, [view.displayedFrame?.frame_id, view.snapshotReceivedAtMs]);
  const snapshot = view.snapshot;
  const display = view.displayedFrame;
  const sourceFault = !snapshot.frame_id;
  const historical = !!display && (paused || !!error || sourceFault || snapshot.status === 'STALE' ||
    snapshot.frame_id !== display.frame_id);
  // `ready` is always false from live_mission.py today (depth scale, mount and pose are unqualified);
  // that is reported as NOT READY, not as a fault of the current frame.
  const isFault = sourceFault || snapshot.status !== 'VALID' || !!error;
  const measuredAgeAtReceipt = display?.frame_age_ms ?? display?.latencies_ms?.capture_to_publish;
  const displayedAgeMs = measuredAgeAtReceipt != null && view.snapshotReceivedAtMs != null ?
    measuredAgeAtReceipt + Math.max(0, clockMs - view.snapshotReceivedAtMs) : null;
  const historicalAge = displayedAgeMs != null && Number.isFinite(displayedAgeMs) ?
    ` · ${(displayedAgeMs / 1000).toFixed(1)} s old` : ' · age unavailable';
  const historicalLabel = paused ? 'PAUSED FRAME' : error ? 'LAST FRAME · UPDATE REJECTED' :
    sourceFault ? 'LAST FRAME · SOURCE STALE' : 'STALE FRAME';
  const ms = (value: number | null | undefined) => finite(value) ? `${value.toFixed(1)} ms` : '—';
  const fps = (value: number | null | undefined) => finite(value) ? `${value.toFixed(1)} FPS` : '—';
  const counts = display?.detector_counts;
  const details = display?.perception?.diagnostics?.cone_details ?? [];
  const cameraMotion = display?.camera_motion;
  const geometry = display?.geometry;
  const exportSnapshot = () => {
    const objectUrl = URL.createObjectURL(new Blob([JSON.stringify(snapshot, null, 2)], { type: 'application/json' }));
    const link = document.createElement('a'); link.href = objectUrl; link.download = `${snapshot.frame_id ?? 'autokart-fault'}.json`;
    link.click(); window.setTimeout(() => URL.revokeObjectURL(objectUrl), 500);
  };
  const connected = snapshot.source !== 'CONNECTING';
  const linkLabel = paused ? 'PAUSED' : error ? 'LINK ERROR' : connected ? 'LINK OK' : 'CONNECTING';
  const linkTone: Tone = paused ? 'idle' : error ? 'fault' : connected ? 'ok' : 'idle';
  const healthLabel = error && !connected ? 'NO DATA' : snapshot.status;
  const readiness = error || snapshot.status === 'STALE' ? 'STALE · NOT READY' : snapshot.ready ? 'QUALIFIED' :
    snapshot.camera_streaming && !sourceFault ? 'STREAMING · UNQUALIFIED' : 'NOT READY';
  const authorityViolation = snapshot.vehicle_authorized !== false || display?.vehicle_authorized === true ||
    snapshot.control_candidate.published_to_vcu === true;
  const reference = display ?? snapshot;
  const blockers = connected ? Array.from(new Set([
    ...(reference.source !== 'LIVE' ? [`${reference.source} SOURCE`] : []),
    ...(reference.depth_scale_qualified !== true ? ['DEPTH SCALE UNQUALIFIED'] : []),
    ...(reference.calibration_status !== 'MOUNT_VERIFIED' ? [reference.calibration_status ?? 'CALIBRATION UNKNOWN'] : []),
    ...(reference.perception?.safety_reasons ?? []),
  ])) : [];
  const faultTone: Tone = paused ? 'idle' : error || sourceFault ? 'fault' : isFault ? healthTone(snapshot.status) : 'info';
  const latencies = display?.latencies_ms ?? {};
  const stageKeys = ['inference', 'depth_localization', 'delaunay', 'gabriel', 'gates', 'centerline_curvature'];
  const stageMax = Math.max(1, ...['sdk_total', ...stageKeys].map(key => latencies[key]).filter(finite));
  const presentClasses = Array.from(new Set((display?.observations ?? []).map(cone => cone.label)));
  return <main className="live-mission">
    <header className="live-header">
      <div className="live-title"><p className="live-eyebrow">AUTOKART · MISSION CONTROL · LIVE</p><h1>Live perception diagnostics</h1>
        <p>One frame · one Python SDK result · read-only · no vehicle commands</p></div>
      <div className="live-badges" aria-label="System state">
        <div className={`live-badge tone-${sourceTone(snapshot.source)}`}><span>SOURCE</span><strong>{snapshot.source}</strong></div>
        <div className={`live-badge tone-${error && !connected ? 'fault' : healthTone(snapshot.status)}`}><span>SDK HEALTH</span><strong>{paused ? 'PAUSED DISPLAY' : healthLabel}</strong></div>
        <div className={`live-badge tone-${snapshot.ready ? 'ok' : 'warn'}`}><span>READINESS</span><strong>{snapshot.ready ? 'QUALIFIED' : 'NOT READY'}</strong></div>
        <div className={`live-badge tone-${linkTone}`}><span>LINK</span><strong><i key={display?.frame_id ?? 'none'} className={`live-pulse ${historical || !display ? 'live-pulse-still' : ''}`} />{linkLabel}</strong></div>
        <div className={`live-badge ${authorityViolation ? 'tone-fault' : 'tone-off'}`}><span>VEHICLE AUTHORITY</span><strong>{authorityViolation ? 'UNEXPECTED · CHECK BACKEND' : 'OFF'}</strong></div>
      </div></header>
    <div className="live-banner">DETECTOR {display?.model?.name ?? snapshot.model?.name ?? 'STARTING'} · {display?.model?.qualification ?? snapshot.model?.qualification ?? 'MODEL STATUS PENDING'} · {display?.source === 'SIMULATED' ? 'SIMULATED GEOMETRY · HP60C CALIBRATION UNVERIFIED' : 'DEPTH SCALE AND CAMERA MOUNT UNVERIFIED'} · VEHICLE AUTHORITY DISABLED</div>
    <div className={`live-fault tone-${faultTone}`} role="status">{paused ? 'PAUSED — historical frame; geometry is not current' :
      error ? `Dashboard connection: ${error}. Displayed images are historical; vehicle motion is invalid.` :
      isFault ? `${snapshot.status}: ${snapshot.reasons.join(', ') || 'no current frame'}. ${sourceFault ? 'Geometry and motion candidates are cleared; any retained view is historical.' : historical ? 'Stale images are diagnostic only; geometry and motion candidates are invalid.' : 'Current images remain visible for diagnosis; vehicle motion is invalid.'}` :
      `Current frame · SDK ${snapshot.status}${snapshot.ready ? '' : ` · not ready: ${blockers.join(', ') || 'backend reports ready=false'}`} · diagnostic display only · vehicle authority OFF`}</div>
    <section className="live-hero">
      <article className="live-panel live-rgb"><h2>01 / Camera RGB <small>{display?.camera_id === 'HP60C' ? 'HP60C' : display?.camera_id ?? 'HP60C'} · {display?.frame_id ?? 'waiting'}</small></h2>
        <div className={`live-image ${historical ? 'live-image-historical' : ''}`}>
          <img src={view.rgb ?? undefined} alt={historical ? 'Historical HP60C RGB with matching cone boxes' : 'Current HP60C RGB with matching cone boxes'} hidden={!view.rgb} />
          {!view.rgb && <span>No current camera image</span>}
          {historical && <span className="live-historical-label">{historicalLabel}{historicalAge}</span>}</div>
        <p>{display?.detections.length ?? 0} {historical ? 'historical' : 'current'} detections · {display?.candidate_observations?.length ?? 0} unqualified ground candidates · {display?.provisional_3d_observations?.length ?? 0} provisional body samples · {ms(display?.latencies_ms?.inference)} inference</p></article>
      <article className="live-panel live-depth"><h2>02 / Depth camera <small>RAW16 · {display?.frame_id ?? 'waiting'}</small></h2>
        <div className={`live-image ${historical ? 'live-image-historical' : ''}`}>
          <img src={view.depth ?? undefined} alt={historical ? 'Historical HP60C depth matched to RGB frame' : 'Current HP60C depth matched to RGB frame'} hidden={!view.depth} />
          {!view.depth && <span>No current depth image</span>}
          {historical && <span className="live-historical-label">{historicalLabel}{historicalAge}</span>}</div>
        <p>{display?.depth_stats?.valid_pixels ?? '—'} valid / {display?.depth_stats?.invalid_pixels ?? '—'} invalid pixels · median {display?.depth_stats?.median_raw ?? '—'} raw (p5 {display?.depth_stats?.p5_raw ?? '—'} · p95 {display?.depth_stats?.p95_raw ?? '—'}) · {display?.rgb_depth_pair ?? 'pairing —'} · metre scale unverified</p></article>
      <article className="live-panel live-bev"><h2>03 / Bird's-eye view <small>{display?.base_frame ?? 'base_link'} · {display?.source === 'SIMULATED' ? 'simulated SDK geometry; no HP60C qualification' : 'validated geometry requires qualified depth and mount'}</small></h2>
        <div className="live-bev-caption">
          <span>{(() => { const g = display ?? snapshot; const syn = g.source === 'SIMULATED' && g.base_frame === 'base_link';
            const q = g.source !== 'SIMULATED' && g.depth_scale_qualified === true && g.calibration_status === 'MOUNT_VERIFIED';
            const pts = (syn || q) ? [...g.observations.map(c => c.xy), ...g.geometry.centerline, ...g.planning.local_path]
              .filter(p => Number.isFinite(p[0]) && Number.isFinite(p[1])) : [];
            const out = pts.filter(([x, y]) => x < 0 || x > 8 || Math.abs(y) > 6).length;
            return syn || q ? `${syn && !q ? 'SIMULATED SDK' : 'QUALIFIED METRIC'} · X forward ↑ · Y left ← · fixed 50 px/m · ${out} points outside view` :
              'NOMINAL camera-ground frame · RAW16 scale and mount unverified'; })()}</span>
          <span className={`live-chip tone-${healthTone(geometry?.status === 'OK' ? 'VALID' : geometry?.status ? 'DEGRADED' : 'WAITING')}`}>GRAPH {geometry?.status ?? '—'}</span>
        </div>
        <div className={`live-bev-frame ${historical ? 'live-image-historical' : ''}`}><MetricView snapshot={display ?? snapshot} layers={layers} />{historical && <span className="live-historical-label">{historicalLabel}{historicalAge} · GEOMETRY INVALID</span>}</div>
        <div className="live-legend" role="group" aria-label="Bird's-eye layers">
          {(Object.keys(layers) as Layer[]).map(layer => <label key={layer} className={layers[layer] ? '' : 'off'}>
            <input type="checkbox" checked={layers[layer]} onChange={() => setLayers(current => ({ ...current, [layer]: !current[layer] }))} />
            <i className={`live-swatch sw-${layerMeta[layer].swatch}`} />{layerMeta[layer].label}</label>)}
          {presentClasses.map(label => <span key={label} className="live-class"><i style={{ background: coneColor[label] ?? coneColor.unknown }} />{label}</span>)}
        </div>
        <p>{display?.geometry.delaunay_edges.length ?? 0} Delaunay · {display?.geometry.gabriel_edges.length ?? 0} Gabriel · {display?.geometry.gates.length ?? 0} gates · {display?.geometry.centerline.length ?? 0} path points · {display?.source === 'SIMULATED' ? `${display.observations.length} simulated cones` : `${display?.detector_counts?.validated_metric ?? 0} validated metric cones`} · capture {display?.capture_timestamp_ns ?? '—'} ns</p>
        {layers.candidates && (display?.candidate_observations?.length ?? 0) > 0 && <p>Unverified ground candidates: {display?.candidate_observations?.map(item => `#${item.id} (${item.xy[0].toFixed(2)}, ${item.xy[1].toFixed(2)}) m ±${item.sigma_m.toFixed(2)} m`).join(' · ')}</p>}
        {layers.candidates && (display?.provisional_3d_observations?.length ?? 0) > 0 && <p>Amber body samples (no ground contact): {display?.provisional_3d_observations?.map(item => `#${item.id} (${item.nominal_xy_m?.[0].toFixed(2) ?? '—'}, ${item.nominal_xy_m?.[1].toFixed(2) ?? '—'}) m ±${item.uncertainty_m.toFixed(2)} m ${item.ground_contact_status}`).join(' · ')}</p>}
        {layers.rejected && (display?.rejected_candidates?.length ?? 0) > 0 && <p>Rejected depth/projection: {display?.rejected_candidates?.map(item => `#${item.id} ${item.localization_status}`).join(' · ')}</p>}</article>
      <section className="live-health" aria-label="Stream health">
        <div><span>DISPLAY FRAME</span><strong>{display?.frame_id ?? '—'}</strong></div>
        <div><span>RGB / DEPTH IDS</span><strong>{display ? `${display.camera_sequence ?? '—'} / ${display.depth_sequence ?? '—'}` : '—'}</strong></div>
        <div><span>FRAME AGE</span><strong>{display ? ms(displayedAgeMs) : '—'}</strong></div>
        <div><span>CAPTURE → PUBLISH</span><strong>{ms(display?.latencies_ms?.capture_to_publish)}</strong></div>
        <div><span>CAPTURE → DISPLAY</span><strong>{ms(displayStats.latencyMs)} estimated</strong></div>
        <div><span>READINESS</span><strong>{readiness}</strong></div>
        <div><span>CAMERA CALLBACK</span><strong>{fps(display?.stream_metrics?.camera_callback_fps)}</strong></div>
        <div><span>PROCESSED / DISPLAY</span><strong>{fps(display?.stream_metrics?.processed_fps)} / {historical || hidden ? '—' : fps(displayStats.fps)}</strong></div>
        <div><span>MAILBOX OVERWRITES</span><strong>{display?.stream_metrics?.mailbox_overwrites ?? '—'} · capacity {display?.stream_metrics?.mailbox_capacity ?? '—'}</strong></div>
        <div><span>VALID RAW DEPTH</span><strong>{display?.depth_stats ? `${display.depth_stats.valid_percent.toFixed(1)}%` : '—'}</strong></div>
      </section>
      <section className="live-sparks" aria-label={`Real-time history of the last ${HISTORY} accepted frames`}>
        <Spark label="SDK TOTAL" unit="ms" values={history.map(s => s.sdkMs)} />
        <Spark label="SNAPSHOT TOTAL" unit="ms" values={history.map(s => s.snapshotMs)} />
        <Spark label="PROCESSED" unit="FPS" values={history.map(s => s.processedFps)} />
        <Spark label="FRAME AGE @ RECEIPT" unit="ms" values={history.map(s => s.ageMs)} />
        <Spark label="CONES IN FRAME" unit="" values={history.map(s => s.cones)} digits={0} />
        {hidden && <p className="live-hidden-note">Tab hidden · browser throttles timers, so display FPS and frame age are not representative</p>}
      </section>
    </section>
    <section className="live-stage-counts" aria-label="Detection stages for the displayed frame">
      <strong>{historical ? 'HISTORICAL FRAME' : 'DISPLAYED FRAME'} · {display?.frame_id ?? 'waiting'}</strong>
      <span>Raw output rows <b>{counts?.raw_output_rows ?? '—'}</b></span>
      <span>Above confidence <b>{counts?.above_confidence ?? '—'}</b></span>
      <span>Valid boxes <b>{counts?.valid_boxes ?? '—'}</b></span>
      <span>{display?.model?.name === 'YOLO26s' ? 'End-to-end selected' : 'After NMS'} <b>{counts?.after_nms ?? '—'}</b></span>
      <span>Published detections <b>{counts?.application_detections ?? display?.detections.length ?? '—'}</b></span>
      <span>Depth candidates <b>{counts?.depth_localized_candidates ?? display?.candidate_observations?.length ?? '—'}</b></span>
      <span>Validated metric <b>{counts?.validated_metric ?? '—'}</b></span>
    </section>
    <section className="live-details">
      <article className="live-panel"><h2>Algorithm trace <small>{historical ? 'historical frame' : 'displayed frame'}</small></h2><dl>
        {Object.entries(latencies).map(([stage, value]) => <div key={stage} className={stageKeys.includes(stage) ? 'live-stage' : ''}>
          <dt>{stage.replaceAll('_', ' ')}</dt>
          {stageKeys.includes(stage) && <span className="live-stage-bar"><i style={{ width: finite(value) ? `${Math.max(2, 100 * value / stageMax)}%` : '0%' }} /></span>}
          <dd>{ms(value)}</dd></div>)}
        <div><dt>Path valid</dt><dd>{String(snapshot.planning.path_valid)}</dd></div>
        <div><dt>Pose</dt><dd>{snapshot.pose_status ?? 'UNAVAILABLE'}</dd></div>
        <div><dt>Camera motion</dt><dd>{cameraMotion ?
          `${cameraMotion.status} · CAMERA_OPTICAL provisional · ${cameraMotion.previous_timestamp_ns} → ${cameraMotion.current_timestamp_ns} ns · ${cameraMotion.inlier_count}/${cameraMotion.matched_points} inliers · ${Number.isFinite(cameraMotion.reprojection_rms_px) ? `${cameraMotion.reprojection_rms_px!.toFixed(2)} px RMS` : 'RMS unavailable'}` :
          'UNAVAILABLE · no odom pose inferred'}</dd></div>
        <div><dt>Pure Pursuit candidate</dt><dd>{snapshot.control_candidate.valid ? `${snapshot.control_candidate.steering_angle_rad.toFixed(3)} rad` : 'UNAVAILABLE'}</dd></div>
        <div><dt>Target speed</dt><dd>{snapshot.control_candidate.valid ? `${snapshot.control_candidate.target_speed_mps.toFixed(2)} m/s · diagnostic` : 'UNAVAILABLE'}</dd></div>
        <div><dt>Published to VCU</dt><dd>{snapshot.control_candidate.published_to_vcu === true ? 'TRUE · UNEXPECTED' : snapshot.control_candidate.published_to_vcu === false ? 'false' : '—'}</dd></div>
        <div><dt>Reasons</dt><dd>{snapshot.reasons.join(', ') || 'none'}</dd></div>
      </dl></article>
      <article className="live-panel"><h2>Geometry state <small>{display?.algorithm_version ?? '—'}</small></h2><dl>
        <div><dt>Graph status</dt><dd>{geometry?.status ?? '—'}</dd></div>
        <div><dt>Boundaries</dt><dd>{geometry?.boundaries.status ?? '—'} · {geometry?.boundaries.mode ?? '—'}{finite(geometry?.boundaries.confidence) ? ` · ${(100 * geometry!.boundaries.confidence!).toFixed(0)}%` : ''}{geometry?.boundaries.reason ? ` · ${geometry.boundaries.reason}` : ''}</dd></div>
        <div><dt>Left / right ids</dt><dd>{geometry ? `${geometry.boundaries.left_ids.join(',') || '—'} / ${geometry.boundaries.right_ids.join(',') || '—'}` : '—'}</dd></div>
        <div><dt>Centerline</dt><dd>{geometry?.centerline_status ?? '—'}</dd></div>
        <div><dt>Curvature</dt><dd>{geometry?.curvature_status ?? '—'}</dd></div>
        <div><dt>Topology reasons</dt><dd>{geometry?.topology_reasons?.join(', ') || (geometry ? 'none' : '—')}</dd></div>
        <div><dt>Perception</dt><dd>{display?.perception?.status ?? '—'} · safety {display?.perception?.safety_state ?? '—'}</dd></div>
        <div><dt>Tracking</dt><dd>{display?.tracking_status ?? '—'}</dd></div>
        {(geometry?.gates ?? []).map((gate, i) => <div key={`gate-${i}`}><dt>Gate G{gate.id ?? i} (#{gate.left_id}↔#{gate.right_id})</dt>
          <dd>{gate.status}{finite(gate.width) ? ` · ${gate.width.toFixed(2)} m` : ''}{finite(gate.quality) ? ` · q ${gate.quality.toFixed(2)}` : ''}</dd></div>)}
      </dl></article>
      <article className="live-panel"><h2>Detection details <small>{display?.frame_id ?? 'waiting'}</small></h2>
        {display?.detections.length ? display.detections.map(detection => <div className="live-detection" key={detection.id}>
          <strong><i className="live-dot" style={{ background: coneColor[detection.label] ?? coneColor.unknown }} />#{detection.id} {detection.label}</strong> · {(100 * detection.confidence).toFixed(0)}% · box {detection.box.join(', ')}
          <span>{(() => { const detail = details.find(item => item.id === detection.id);
            const status = detail?.localization_status === 'METRIC_OK' && !display.depth_scale_qualified ?
              'RAW_SCALE_CANDIDATE' : detail?.localization_status ?? 'depth unavailable';
            return `${detail?.distance_m == null ? 'metric distance unavailable' : `${detail.distance_m.toFixed(2)} m raw-scale candidate · unverified`} · ${status}`;
          })()}</span>
        </div>) : <p>No cone detections in this frame{display?.model?.qualification === 'NO_DETECTOR' ? ' · no detector loaded (simulated scenario injects cones directly)' : ''}.</p>}
        <p className="live-note">Lines update only when a new SDK snapshot arrives; nothing is interpolated between frames. Shaded stale views are historical. Cone IDs are frame-local without a qualified pose tracker.</p></article>
      <section className="live-toolbar" aria-label="Display controls (local display only)"><button onClick={() => setPaused(!paused)}>{paused ? 'Resume display' : 'Pause display'}</button>
        <button onClick={exportSnapshot}>Export current diagnostic</button>
        <label>LAN view token <input type="password" value={token} onChange={event => { setToken(event.target.value);
          sessionStorage.setItem('autokart-view-token', event.target.value); }} placeholder="localhost needs none" /></label>
        <small>Controls affect this browser only. This page has no route that commands the vehicle.</small></section>
    </section>
    <footer>Source {snapshot.source} · display frame {display?.frame_id ?? '—'} · capture {display?.capture_timestamp_ns ?? '—'} ns · calibration {display?.calibration_status ?? 'unavailable'} · depth scale {display?.depth_scale_provenance ?? '—'} · vehicle authority OFF · no telemetry or VCU publication</footer>
  </main>;
}
