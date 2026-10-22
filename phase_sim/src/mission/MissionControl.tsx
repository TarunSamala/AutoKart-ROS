import { useEffect, useMemo, useState } from 'react';
import { MissionScene, type Layer, type Selection } from './MissionScene';
import { STAGES, stageFrame, type MissionFrame } from './schema';
import { SOURCE_LABEL, useMissionData } from './useMissionData';
import { LabNav } from '../shared/LabNav';
import { RosLivePanel } from './RosLivePanel';
import { DEFAULT_LAB_SETTINGS, type LabSettings, type ScenarioName } from './labScenarios';
import './mission.css';

// 'ALGORITHM INDEX' (formerly 'ALGORITHM LAB') is a click-to-inspect list of the current source's
// published elements; renamed so "LAB" only ever refers to the Python ALGORITHM LAB at /shared.
const VIEWS = ['3D WORLD', 'ALGORITHM INDEX', 'SCENARIO BUILDER', 'VEHICLE', 'SAFETY', 'REPLAY', 'BENCHMARK'] as const;
type View = typeof VIEWS[number];
const INITIAL_LAYERS: Record<Layer, boolean> = { groundTruth: true, tracked: true, detections: true, delaunay: true, gabriel: true, gates: true, boundaries: true, candidates: true, selected: true, safety: true, sensor: true };
const fmt = (value: number | null | undefined, places = 2) => value == null || !Number.isFinite(value) ? '—' : value.toFixed(places);

function DiagnosticStrip({ frame }: { frame: MissionFrame | null }) {
  if (!frame?.diagnostics) return null;
  return <div className="mc-status" aria-label="ROS stage diagnostics">{(['perception', 'mapping', 'planning'] as const).map((stage) => {
    const item = frame.diagnostics?.[stage];
    const stale = item?.expires_ns != null && frame.timestamp_s * 1e9 > item.expires_ns;
    const state = !item ? 'UNKNOWN' : stale ? 'STALE' : item.valid ? 'VALID' : 'INVALID';
    const reason = item?.reason_codes?.join(', ') || (item?.valid ? 'stage sample valid' : 'no current reason');
    return <span key={stage} className={state !== 'VALID' ? 'mc-alert' : ''}>{stage.toUpperCase()} {item?.source ?? 'UNKNOWN'} · {state} · {item?.frame_id ?? '—'} · {item?.sample_time_ns ?? '—'} ns · {reason}</span>;
  })}<span>VCU AUTHORITY DISABLED</span></div>;
}

function Detail({ frame, selection }: { frame: MissionFrame | null; selection: Selection }) {
  if (!frame) return <p className="mc-muted">Waiting for a timestamped frame.</p>;
  const cone = [...frame.perception.cones, ...(frame.perception.ground_truth_cones ?? []), ...(frame.visualization?.course_cones ?? [])].find((item) => item.id === selection.id);
  const truth = frame.perception.ground_truth_cones?.find((item) => item.id === selection.id);
  const edge = [...frame.planning.gabriel_edges, ...frame.planning.delaunay_edges].find((item) => item.id === selection.id);
  const gate = frame.planning.gates.find((item) => item.id === selection.id);
  const candidate = frame.planning.candidate_paths.find((item) => item.id === selection.id);
  if (selection.kind === 'cone' && cone) return <dl><dt>ID / CLASS</dt><dd>{cone.id} / {cone.class}</dd><dt>POSITION XY</dt><dd>{fmt(cone.x_m)}, {fmt(cone.y_m)} m</dd><dt>Z</dt><dd>{cone.z_m == null ? 'UNAVAILABLE' : `${fmt(cone.z_m)} m`}</dd><dt>FRAME</dt><dd>{frame.source === 'REAL_KART' || frame.source === 'SIMULATION' ? 'odom' : frame.source}</dd><dt>CONFIDENCE</dt><dd>{fmt(cone.confidence)}</dd><dt>RANGE / BEARING</dt><dd>{fmt(cone.range_m)} m / {fmt(cone.bearing_rad)} rad</dd><dt>TRACK AGE</dt><dd>{fmt(cone.track_age_s)} s</dd><dt>GROUND TRUTH ERROR</dt><dd>{truth && frame.perception.cones.includes(cone) ? `${fmt(Math.hypot(cone.x_m - truth.x_m, cone.y_m - truth.y_m))} m` : '—'}</dd></dl>;
  if (selection.kind === 'edge' && edge) { const a = frame.perception.cones.find((cone) => cone.id === edge.a), b = frame.perception.cones.find((cone) => cone.id === edge.b); const length = a && b ? Math.hypot(a.x_m - b.x_m, a.y_m - b.y_m) : null; return <dl><dt>ENDPOINTS</dt><dd>{edge.a} ↔ {edge.b}</dd><dt>LENGTH / DIAMETER</dt><dd>{fmt(length)} m</dd><dt>TRIANGLES</dt><dd>{edge.triangle_ids?.join(', ') || '—'}</dd><dt>GABRIEL</dt><dd>{edge.accepted === undefined ? 'NO DEBUG' : edge.accepted ? 'ACCEPTED' : 'REJECTED'}</dd><dt>RADIUS</dt><dd>{fmt(edge.radius_m)} m</dd><dt>BLOCKING CONE</dt><dd>{edge.blocking_cone_id ?? '—'}</dd><dt>REASON</dt><dd>{edge.reason ?? '—'}</dd></dl>; }
  if (selection.kind === 'gate' && gate) return <dl><dt>WIDTH</dt><dd>{fmt(gate.width_m)} m</dd><dt>HEADING</dt><dd>{fmt(gate.heading_rad)} rad</dd><dt>CONFIDENCE</dt><dd>{fmt(gate.confidence)}</dd><dt>VALID</dt><dd>{String(gate.valid)}</dd><dt>REASON</dt><dd>{gate.reason ?? '—'}</dd></dl>;
  if (selection.kind === 'candidate' && candidate) return <dl><dt>RANK / STATUS</dt><dd>{candidate.rank} / {candidate.selected ? 'SELECTED' : 'NOT SELECTED'}</dd><dt>TOTAL COST</dt><dd>{fmt(candidate.total_cost)}</dd>{Object.entries(candidate.costs).map(([key, value]) => <div key={key}><dt>{key}</dt><dd>{fmt(value)}</dd></div>)}<dt>REASON</dt><dd>{candidate.reason ?? '—'}</dd></dl>;
  if (selection.kind === 'obstacle') { const obstacle = frame.perception.obstacles?.find((item) => item.id === selection.id); if (obstacle) return <dl><dt>POSITION</dt><dd>{fmt(obstacle.x_m)}, {fmt(obstacle.y_m)} m</dd><dt>RANGE</dt><dd>{fmt(obstacle.distance_m)} m</dd><dt>RADIUS</dt><dd>{fmt(obstacle.radius_m)} m</dd></dl>; }
  return <dl><dt>MEASURED POSE</dt><dd>{fmt(frame.vehicle.x_m)}, {fmt(frame.vehicle.y_m)} m</dd><dt>HEADING</dt><dd>{fmt(frame.vehicle.yaw_rad)} rad</dd><dt>SPEED</dt><dd>{fmt(frame.vehicle.speed_mps)} m/s</dd><dt>STEERING FEEDBACK</dt><dd>{fmt(frame.vehicle.measured_steering_rad)} rad</dd><dt>REQUESTED STEERING</dt><dd>{fmt(frame.vehicle.requested_steering_rad)} rad</dd><dt>CROSS TRACK</dt><dd>{fmt(frame.metrics.cross_track_error_m)} m</dd></dl>;
}

function Plot({ frames, current, field, title, unit }: { frames: MissionFrame[]; current: MissionFrame | null; field: (frame: MissionFrame) => number | null | undefined; title: string; unit: string }) {
  const points = useMemo(() => frames.map((frame) => ({ t: frame.timestamp_s, value: field(frame) })).filter((point): point is { t: number; value: number } => point.value != null && Number.isFinite(point.value)).slice(-600), [frames, field]);
  if (points.length < 2) return <div className="mc-plot"><b>{title}</b><span>NO DATA</span></div>;
  const min = Math.min(...points.map((item) => item.value));
  const max = Math.max(...points.map((item) => item.value));
  const span = Math.max(0.001, max - min);
  const t0 = points[0].t, duration = Math.max(0.001, points[points.length - 1].t - t0);
  const polyline = points.map((item) => `${((item.t - t0) / duration) * 100},${90 - ((item.value - min) / span) * 75}`).join(' ');
  const cursor = current ? ((current.timestamp_s - t0) / duration) * 100 : 100;
  return <div className="mc-plot"><b>{title}</b><strong>{fmt(current ? field(current) : null)} <small>{unit}</small></strong><svg viewBox="0 0 100 100" preserveAspectRatio="none" aria-label={`${title} plot`}><polyline points={polyline} fill="none" stroke="#5cddba" strokeWidth="1.6" vectorEffect="non-scaling-stroke" /><line x1={cursor} x2={cursor} y1="0" y2="100" stroke="#e9c45c" strokeWidth="1" vectorEffect="non-scaling-stroke" /></svg></div>;
}
function AlgorithmIndex({ frame, onSelect }: { frame: MissionFrame | null; onSelect: (item: Selection) => void }) {
  if (!frame) return null;
  const groups: Array<{ label: string; kind: string; items: Array<{ id: string; description: string }> }> = [
    { label: 'DETECTED CONES', kind: 'cone', items: frame.perception.cones.map((cone) => ({ id: cone.id, description: cone.class })) },
    { label: 'DELAUNAY EDGES', kind: 'edge', items: frame.planning.delaunay_edges.map((edge) => ({ id: edge.id, description: `${edge.a} ↔ ${edge.b}` })) },
    { label: 'GABRIEL TESTS', kind: 'edge', items: frame.planning.gabriel_edges.map((edge) => ({ id: edge.id, description: edge.accepted ? 'ACCEPTED' : `REJECTED · ${edge.blocking_cone_id ?? edge.reason ?? '—'}` })) },
    { label: 'GATES', kind: 'gate', items: frame.planning.gates.map((gate) => ({ id: gate.id, description: gate.valid ? `${fmt(gate.width_m)} m` : gate.reason ?? 'REJECTED' })) },
    { label: 'CANDIDATE PATHS', kind: 'candidate', items: frame.planning.candidate_paths.map((candidate) => ({ id: candidate.id, description: `${fmt(candidate.total_cost)} · ${candidate.selected ? 'SELECTED' : 'NOT SELECTED'}` })) },
  ];
  return <section className="mc-card mc-index"><small>CLICK TO INSPECT</small>{groups.map((group) => <div key={group.label}><h3>{group.label}</h3>{group.items.length ? group.items.slice(0, 40).map((item) => <button key={item.id} onClick={() => onSelect({ kind: group.kind, id: item.id })}><b>{item.id}</b><span>{item.description}</span></button>) : <p className="mc-muted">No published data</p>}</div>)}</section>;
}
type PlotSpec = { title: string; unit: string; field: (frame: MissionFrame) => number | null | undefined };
const PLOT_GROUPS: Record<string, PlotSpec[]> = {
  MOTION: [
    { title: 'SPEED', unit: 'm/s', field: (f) => f.vehicle.speed_mps },
    { title: 'TARGET SPEED', unit: 'm/s', field: (f) => f.vehicle.target_speed_mps },
    { title: 'STEERING REQUEST', unit: 'rad', field: (f) => f.vehicle.requested_steering_rad },
    { title: 'STEERING MEASURED', unit: 'rad', field: (f) => f.vehicle.measured_steering_rad },
  ],
  TRACKING: [
    { title: 'STEERING ERROR', unit: 'rad', field: (f) => f.vehicle.requested_steering_rad != null && f.vehicle.measured_steering_rad != null ? f.vehicle.requested_steering_rad - f.vehicle.measured_steering_rad : null },
    { title: 'CROSS TRACK', unit: 'm', field: (f) => f.metrics.cross_track_error_m },
    { title: 'HEADING ERROR', unit: 'rad', field: (f) => f.metrics.heading_error_rad },
    { title: 'W LEFT', unit: 'turns', field: (f) => f.planning.skidpad?.w_left },
    { title: 'W RIGHT', unit: 'turns', field: (f) => f.planning.skidpad?.w_right },
  ],
  SAFETY: [
    { title: 'OBSTACLE DISTANCE', unit: 'm', field: (f) => f.safety.obstacle?.distance_m },
    { title: 'TTC', unit: 's', field: (f) => f.safety.obstacle?.ttc_s },
    { title: 'BRAKE COMMAND', unit: '', field: (f) => f.vehicle.brake_command },
    { title: 'BRAKE FEEDBACK', unit: '', field: (f) => f.vehicle.brake_feedback },
  ],
  RUNTIME: [
    { title: 'PLANNER', unit: 'ms', field: (f) => f.metrics.planner_runtime_ms },
    { title: 'CONTROLLER', unit: 'ms', field: (f) => f.metrics.controller_runtime_ms },
  ],
};

function StepNote({ frame, stage, selection }: { frame: MissionFrame | null; stage: number; selection: Selection }) {
  if (!frame) return <span>Waiting for a frame</span>;
  const p = frame.planning;
  const rejected = p.gabriel_edges.filter((edge) => edge.accepted === false).length;
  const accepted = p.gabriel_edges.filter((edge) => edge.accepted === true).length;
  const selectedEdge = selection.kind === 'edge' ? p.gabriel_edges.find((edge) => edge.id === selection.id) : undefined;
  const messages = [
    `${frame.perception.cones.length} detected cones · FOV ${fmt(frame.perception.sensor_fov_rad, 1)} rad`,
    `${p.delaunay_edges.length} edges connect observed cones`,
    `${p.gabriel_edges.length} tests · ${accepted} accepted · ${rejected} rejected${selectedEdge ? ` · ${selectedEdge.id}: ${selectedEdge.reason ?? (selectedEdge.accepted ? 'ACCEPT' : 'REJECT')}` : ''}`,
    `${p.gates.filter((gate) => gate.valid).length} valid gates`,
    `${p.candidate_paths.length} published candidate paths`,
    p.skidpad ? `${p.skidpad.state ?? 'MISSION'} · Wₗ ${fmt(p.skidpad.w_left)} · Wᵣ ${fmt(p.skidpad.w_right)}` : 'No mission decision published',
    `${p.selected_path.length} selected trajectory points`,
    `Lookahead ${p.lookahead ? `${fmt(p.lookahead.x_m)}, ${fmt(p.lookahead.y_m)} m` : 'not published'} · steering ${fmt(frame.vehicle.requested_steering_rad)} rad`,
  ];
  return <span>{messages[stage - 1]}</span>;
}

function MissionCard({ frame }: { frame: MissionFrame | null }) {
  const live = frame?.planning.skidpad, demo = frame?.visualization?.skidpad;
  if (!live && !demo) return null;
  const left = live?.w_left ?? demo?.w_left, right = live?.w_right ?? demo?.w_right;
  const target = Math.abs(live?.target_winding ?? 2);
  const progress = (value: number | undefined) => Math.min(100, Math.abs(value ?? 0) / target * 100);
  return <section className="mc-floating mc-mission"><small>{live ? 'SKIDPAD MISSION' : 'DEMO · OBSERVED TRAIL'}</small><strong>{live?.state ?? 'WINDING OBSERVATION'}</strong><div className="mc-winding"><div>RIGHT <b>{fmt(right)} turns</b><meter min="0" max="100" value={progress(right)} /></div><div>LEFT <b>{fmt(left)} turns</b><meter min="0" max="100" value={progress(left)} /></div></div>{live?.branches?.length ? <div className="mc-branches">{live.branches.map((branch) => <div key={branch.name}><b>{branch.name}</b><span>{branch.selected ? '✓ SELECTED' : branch.valid ? 'AVAILABLE' : `REJECTED · ${branch.reason ?? '—'}`}</span></div>)}</div> : null}{live ? <p>SEGMENT {live.branch ?? live.state ?? '—'} · WAYPOINT {frame?.planning.lookahead && frame.planning.selected_path.length ? `${frame.planning.selected_path.length - 1} / ${frame.planning.selected_path.length - 1}` : 'UNAVAILABLE'}<br />PROGRESS {frame?.planning.lookahead ? fmt(live.progress_index, 3) : '—'} {frame?.planning.lookahead ? live.progress_unit ?? '—' : ''} · LOCK {live.branch_locked == null ? '—' : live.branch_locked ? 'ACTIVE' : 'OPEN'}<br />{live.reason ?? 'No branch reason published'}</p> : <p>Winding from the visible DEMO trail. No branch decision is available.</p>}</section>;
}

/** One source picker for both headers; labels come from SOURCE_LABEL so the JS sandbox is never called "LAB". */
function SourceOptions({ data, value, className }: { data: ReturnType<typeof useMissionData>; value: string; className?: string }) {
  return <select className={className} aria-label="Data source" value={value} onChange={(event) => { if (event.target.value === 'REPLAY') data.setMode('REPLAY'); else data.setSource(event.target.value as typeof data.source); }}>
    <option value="LAB">{SOURCE_LABEL.LAB}</option>
    <option value="SIMULATION">{SOURCE_LABEL.SIMULATION}</option>
    <option value="REPLAY" disabled={data.frames.length < 2}>REPLAY · RECORDED</option>
    <option value="REAL_KART">{SOURCE_LABEL.REAL_KART}</option>
    {data.source === 'DEMO' && <option value="DEMO">{SOURCE_LABEL.DEMO}</option>}
  </select>;
}

function LabToolbar({ data, onPause, onStep, onNext, onView }: { data: ReturnType<typeof useMissionData>; onPause: () => void; onStep: () => void; onNext: () => void; onView: (view: View) => void }) {
  return <header className="mc-top mc-lab-top">
    <div className="mc-brand"><span>AK</span><div><strong>SKIDPAD SANDBOX</strong><small>CLIENT-SIDE JS · NO BACKEND</small></div></div>
    <div className="mc-lab-controls">
      <label>SOURCE<SourceOptions data={data} value="LAB" /></label>
      <label>SCENARIO<select aria-label="Scenario" value={data.labScenario} onChange={(event) => data.configureLab(event.target.value as ScenarioName)}><option value="SKIDPAD">SKIDPAD</option><option value="AUTOCROSS">AUTOCROSS</option><option value="BRAKE">BRAKE</option></select></label>
      <button className="mc-run" onClick={data.runLab} disabled={data.mode === 'LIVE'}>▶ RUN</button>
      <button onClick={onPause} disabled={data.mode !== 'LIVE'}>⏸ PAUSE</button>
      <button onClick={onStep} disabled={data.mode === 'REPLAY'}>STEP</button>
      <button onClick={onNext} disabled={data.mode !== 'STEP'}>NEXT STAGE</button>
      <button onClick={data.resetLab}>↺ RESET</button>
      <button onClick={() => onView('ALGORITHM INDEX')}>INDEX</button>
      <button onClick={() => onView('SCENARIO BUILDER')}>BUILDER</button>
      <button onClick={() => onView('REPLAY')}>REPLAY</button>
      <strong className="mc-lab-state">{data.status}</strong>
    </div>
    <div className="mc-clock">{fmt(data.current?.timestamp_s, 2)} <small>s</small></div>
  </header>;
}

function LabFooter({ frame, onPlots, plotsOpen, onSave }: { frame: MissionFrame | null; onPlots: () => void; plotsOpen: boolean; onSave: () => void }) {
  return <footer className="mc-bottom mc-lab-bottom"><span>SPEED <b>{fmt(frame?.vehicle.speed_mps)} m/s</b></span><span>STEERING <b>{fmt(frame?.vehicle.measured_steering_rad)} rad</b></span><span>WINDING <b>{fmt(frame?.planning.skidpad?.w_right)} / {fmt(frame?.planning.skidpad?.w_left)}</b></span><span>TTC <b>{fmt(frame?.safety.obstacle?.ttc_s)} s</b></span><span>TIME <b>{fmt(frame?.timestamp_s, 2)} s</b></span><button onClick={onSave}>SAVE RUN</button><button onClick={onPlots}>PLOTS {plotsOpen ? '▾' : '▴'}</button></footer>;
}

function ScenarioBuilder({ scenario, settings, setSettings, onGenerate }: { scenario: ScenarioName; settings: LabSettings; setSettings: (next: LabSettings) => void; onGenerate: (name: ScenarioName, settings: LabSettings) => void }) {
  const [name, setName] = useState<ScenarioName>(scenario);
  const field = (key: keyof LabSettings, label: string, min: number, max: number, step = 1) => <label key={key}>{label}<input aria-label={label} type="number" min={min} max={max} step={step} value={settings[key]} onChange={(event) => setSettings({ ...settings, [key]: Number(event.target.value) })} /></label>;
  return <section className="mc-card mc-builder"><small>SCENARIO BUILDER · JS SANDBOX</small><h2>Generate a course</h2><p>Changes take effect on the world when you press Generate.</p><label>SCENARIO<select value={name} onChange={(event) => setName(event.target.value as ScenarioName)}><option>SKIDPAD</option><option>AUTOCROSS</option><option>BRAKE</option></select></label><div className="mc-builder-fields">{name === 'SKIDPAD' ? <>{field('skidpadRadius', 'Loop radius (m)', 5, 15, 0.5)}{field('width', 'Track width (m)', 2.5, 6, 0.1)}{field('coneSpacing', 'Cone spacing (m)', 0.7, 3, 0.1)}{field('entryLength', 'Entry length (m)', 8, 30, 0.5)}{field('exitLength', 'Exit length (m)', 8, 30, 0.5)}{field('junctionLength', 'Clear junction (m)', 1.5, 5, 0.1)}</> : <>{field('complexity', 'Complexity', 0, 1, 0.05)}{field('length', 'Length (m)', 170, 500, 10)}{field('width', 'Track width (m)', 2.5, 6, 0.1)}{field('hairpins', 'Hairpins', 1, 5)}{field('sBends', 'S bends', 2, 8)}{field('slaloms', 'Slalom sets', 2, 8)}{field('coneSpacing', 'Cone spacing (m)', 1, 4, 0.1)}</>}{field('noise', 'Noise (m)', 0, 0.5, 0.01)}{field('dropout', 'Dropout', 0, 0.5, 0.01)}{field('seed', 'Seed', 0, 999999)}</div><button className="mc-run" onClick={() => onGenerate(name, settings)}>GENERATE</button></section>;
}

export function MissionControl() {
  const data = useMissionData();
  const [view, setView] = useState<View>('3D WORLD');
  const [layers, setLayers] = useState(INITIAL_LAYERS);
  const [camera, setCamera] = useState('algorithm');
  const [selection, setSelection] = useState<Selection>({ kind: 'kart', id: 'vehicle' });
  const [inspectorOpen, setInspectorOpen] = useState(false);
  const [stage, setStage] = useState(1);
  const [animate, setAnimate] = useState(true);
  const [animatedStage, setAnimatedStage] = useState(1);
  const [errorMessage, setErrorMessage] = useState('');
  const [plotGroup, setPlotGroup] = useState('MOTION');
  const [plotsOpen, setPlotsOpen] = useState(false);
  const [layersOpen, setLayersOpen] = useState(false);
  const [builder, setBuilder] = useState<LabSettings>(DEFAULT_LAB_SETTINGS);
  useEffect(() => { document.title = 'AutoKart Sandbox & ROS Viewer'; }, []);
  useEffect(() => {
    if (!animate || data.mode !== 'LIVE') return;
    const timer = window.setInterval(() => setAnimatedStage((current) => current % STAGES.length + 1), 820);
    return () => window.clearInterval(timer);
  }, [animate, data.mode]);
  const activeStage = data.mode === 'STEP' ? stage : animate && data.mode === 'LIVE' ? animatedStage : 0;
  const frame = data.current && (activeStage || (data.source === 'LAB' && data.status === 'READY')) ? stageFrame(data.current, activeStage || 1) : data.current;
  const raw = data.current;
  const isLab = data.source === 'LAB' && data.mode !== 'REPLAY';
  const minTime = data.frames[0]?.timestamp_s ?? 0;
  const maxTime = data.frames[data.frames.length - 1]?.timestamp_s ?? 0;
  const scenario = String(data.manifest.scenario ?? raw?.scenario ?? (data.source === 'DEMO' ? data.demoPreset : '—')).replace('mission-', '').toUpperCase();
  const setLayer = (layer: Layer) => setLayers((current) => ({ ...current, [layer]: !current[layer] }));
  const pause = () => { data.setTime(raw?.timestamp_s ?? data.time); data.setMode('PAUSE'); data.setPlaying(false); };
  const enterStep = () => { data.setTime(raw?.timestamp_s ?? data.time); data.setMode('STEP'); data.setPlaying(false); setStage(1); };
  const choose = (item: Selection) => { setSelection(item); setInspectorOpen(true); };
  const selectView = (next: View) => { setView(next); if (next === 'REPLAY' && data.frames.length) data.setMode('REPLAY'); };
  const obstacle = raw?.safety.obstacle;
  const hasWorld = view === '3D WORLD' || view === 'ALGORITHM INDEX' || view === 'REPLAY';

  return <div className="mission-shell">
    <LabNav current="sandbox" />
    {isLab ? <LabToolbar data={data} onPause={pause} onStep={enterStep} onNext={() => setStage((current) => Math.min(STAGES.length, current + 1))} onView={selectView} /> : <header className="mc-top"><div className="mc-brand"><span>AK</span><div><strong>ROS VIEWER</strong><small>{data.source === 'REAL_KART' ? 'REAL KART · READ-ONLY' : data.source === 'SIMULATION' ? 'ROS SIMULATION · READ-ONLY' : 'RECORDED / SYNTHETIC'}</small></div></div><SourceOptions data={data} value={data.mode === "REPLAY" ? "REPLAY" : data.source} className="mc-top-source" /><nav>{VIEWS.map((name) => <button key={name} className={view === name ? 'selected' : ''} onClick={() => selectView(name)}>{name}</button>)}</nav><div className="mc-clock">{fmt(raw?.timestamp_s, 2)} <small>s</small></div></header>}
    {isLab ? <div className="mc-status mc-lab-status"><span className="mc-sandbox-badge">SKIDPAD SANDBOX — client-side JS (LabSkidpad.ts), simplified kinematics. NOT the Python algorithm core: see <a href="/shared">ALGORITHM LAB</a></span><span>{data.labScenario}</span><span>{data.status}</span><span>MISSION {raw?.planning.skidpad?.state ?? '—'}</span><span className={obstacle?.brake_trigger ? 'mc-alert' : ''}>SAFETY {obstacle?.brake_trigger ? 'BRAKE OVERRIDE' : 'NORMAL'}</span><span>NO BACKEND · NO ROS · VEHICLE AUTHORITY OFF</span></div> : <div className="mc-status"><span className={data.source === 'DEMO' || data.mode === 'REPLAY' ? 'mc-demo' : ''}>{data.mode === 'REPLAY' ? `REPLAY OF ${SOURCE_LABEL[data.source]} · RECORDED, NOT LIVE` : SOURCE_LABEL[data.source]}</span><span>SCENARIO {scenario}</span><span>{data.mode} · {data.status}</span><span>MISSION {raw?.planning.skidpad?.state ?? 'NO DEBUG'}</span><span>SPEED {fmt(raw?.vehicle.speed_mps)} m/s</span><span className={obstacle?.brake_trigger ? 'mc-alert' : ''}>SAFETY {obstacle?.brake_trigger ? 'BRAKE REQUIRED' : raw?.safety.state ?? 'NO DATA'}</span><span>RUN {String(raw?.run_id ?? '—')}</span><span>READ ONLY</span></div>}
    {!isLab && <DiagnosticStrip frame={raw} />}
    {errorMessage && <div className="mc-error" role="alert">{errorMessage}</div>}
    <main className="mc-main"><div className="mc-world">
      <MissionScene frame={frame} layers={layers} cameraMode={camera} selection={selection} onSelect={choose} animationStage={activeStage} />
      {data.source === 'REAL_KART' && data.mode === 'LIVE' && !(raw?.perception.cones.length) && <div className="mc-waiting-geometry" role="status"><b>WAITING FOR QUALIFIED GEOMETRY</b><small>The world view shows only odom-frame data from verified camera calibration, live odometry and a current map. No simulated geometry is substituted.</small></div>}
      <div className="mc-world-top"><div className="mc-world-title"><small>{isLab ? "SANDBOX · CLIENT-SIDE JS WORLD" : view === "ALGORITHM INDEX" ? "ALGORITHM INDEX" : view === "REPLAY" ? "SYNCHRONIZED REPLAY" : data.source === "REAL_KART" ? "3D WORLD · REAL KART (READ-ONLY)" : data.source === "SIMULATION" ? "3D WORLD · ROS SIMULATION" : `3D WORLD · ${SOURCE_LABEL[data.source]}`}</small><h1>{scenario === "—" ? "Waiting for track data" : scenario}</h1></div><div className="mc-world-controls"><button className={animate ? "selected" : ""} onClick={() => setAnimate(!animate)} aria-pressed={animate}>▶ ALGORITHM ANIMATION</button><select value={camera} onChange={(event) => setCamera(event.target.value)} aria-label="Camera mode"><option value="algorithm">ALGORITHM</option><option value="top">TOP</option><option value="chase">CHASE</option><option value="driver">DRIVER</option><option value="free">FREE</option></select><button onClick={() => setLayersOpen(!layersOpen)} aria-expanded={layersOpen}>LAYERS</button></div></div>
      <div className="mc-world-pipeline">{['SEE', 'DELAUNAY', 'GABRIEL', 'GATES', 'CANDIDATES', 'MISSION', 'SELECT', 'CONTROL'].map((name, index) => <div key={name} className={activeStage === index + 1 ? 'active' : ''}><b>{name}</b><small>{index === 0 ? raw?.perception.cones.length ?? '—' : index === 1 ? raw?.planning.delaunay_edges.length ?? '—' : index === 2 ? raw?.planning.gabriel_edges.length ?? '—' : index === 3 ? raw?.planning.gates.length ?? '—' : index === 4 ? raw?.planning.candidate_paths.length ?? '—' : index === 5 ? raw?.planning.skidpad?.state ? 'READY' : '—' : index === 6 ? raw?.planning.selected_path.length ?? '—' : raw?.vehicle.requested_steering_rad == null ? '—' : 'READY'}</small></div>)}</div>
      {activeStage > 0 && <div className="mc-stage-note"><b>{activeStage} · {STAGES[activeStage - 1]}</b><StepNote frame={raw} stage={activeStage} selection={selection} /></div>}
      {layersOpen && <div className="mc-layerbar">{(Object.keys(layers) as Layer[]).map((layer) => <label key={layer}><input type="checkbox" checked={layers[layer]} onChange={() => setLayer(layer)} />{layer.replace(/([A-Z])/g, ' $1').toUpperCase()}</label>)}</div>}
      <div className="mc-legend"><b>VISUAL KEY</b><span><i className="blue" /> BLUE · LEFT</span><span><i className="yellow" /> YELLOW · RIGHT</span><span><i className="orange" /> ORANGE · SPECIAL</span><span><i className="cyan line" /> DELAUNAY</span><span><i className="green line" /> GABRIEL ACCEPT</span><span><i className="red line" /> GABRIEL REJECT</span><span><i className="white line" /> CANDIDATE</span><span><i className="magenta line" /> SELECTED</span>{data.source === "DEMO" && <em>Dim cones show DEMO course context</em>}</div>
      <MissionCard frame={raw} />
      {obstacle && <div className={`mc-floating mc-safety ${obstacle.brake_trigger ? 'braking' : ''}`}><small>OBSTACLE {obstacle.id}</small><strong>{obstacle.brake_trigger ? 'BRAKE OVERRIDE' : 'SAFE · MONITORING'}</strong><p>Speed {fmt(raw?.vehicle.speed_mps)} m/s · obstacle {fmt(obstacle.distance_m)} m<br />TTC {fmt(obstacle.ttc_s)} s · reaction {fmt(obstacle.reaction_distance_m)} m<br />Braking {fmt(obstacle.braking_distance_m)} m · safety {fmt(obstacle.safety_margin_m)} m<br />Required {fmt(obstacle.stopping_distance_m)} m · margin {fmt(obstacle.margin_m)} m</p></div>}
      {inspectorOpen && <section className="mc-floating mc-inspector"><div className="mc-inspector-head"><small>INSPECT · {selection.kind.toUpperCase()}</small><button onClick={() => setInspectorOpen(false)} aria-label="Close inspector">×</button></div><h2>{selection.id}</h2><Detail frame={raw} selection={selection} /></section>}
      {view === 'ALGORITHM INDEX' && <div className="mc-view-drawer"><button className="mc-close" onClick={() => setView('3D WORLD')}>×</button><AlgorithmIndex frame={raw} onSelect={choose} /></div>}
      {!hasWorld && <div className="mc-view-drawer mc-settings"><button className="mc-close" onClick={() => setView('3D WORLD')}>×</button>{view === 'SCENARIO BUILDER' && <ScenarioBuilder scenario={data.labScenario} settings={builder} setSettings={setBuilder} onGenerate={(name, settings) => { data.configureLab(name, settings); setView('3D WORLD'); }} />}
      {view === 'VEHICLE' && <section className="mc-card"><small>VEHICLE FEEDBACK</small><h2>{fmt(raw?.vehicle.speed_mps)} m/s</h2><dl><dt>TARGET SPEED</dt><dd>{fmt(raw?.vehicle.target_speed_mps)} m/s</dd><dt>STEERING REQUEST</dt><dd>{fmt(raw?.vehicle.requested_steering_rad)} rad</dd><dt>STEERING FEEDBACK</dt><dd>{fmt(raw?.vehicle.measured_steering_rad)} rad</dd><dt>BRAKE COMMAND</dt><dd>{fmt(raw?.vehicle.brake_command)}</dd><dt>BRAKE FEEDBACK</dt><dd>{fmt(raw?.vehicle.brake_feedback)}</dd><dt>CROSS TRACK ERROR</dt><dd>{fmt(raw?.metrics.cross_track_error_m)} m</dd><dt>HEADING ERROR</dt><dd>{fmt(raw?.metrics.heading_error_rad)} rad</dd><dt>PLANNER / CONTROL</dt><dd>{fmt(raw?.metrics.planner_runtime_ms)} / {fmt(raw?.metrics.controller_runtime_ms)} ms</dd></dl></section>}
      {view === 'SAFETY' && <section className="mc-card"><small>READ ONLY SAFETY OBSERVATION</small><h2>{raw?.safety.state ?? 'NO SAFETY DATA'}</h2><dl><dt>COMMAND PRIORITY</dt><dd>{raw?.safety.priority ?? '—'}</dd>{([['PHYSICAL E-STOP', raw?.safety.physical_estop], ['REMOTE E-STOP', raw?.safety.remote_estop], ['LORA HEARTBEAT', raw?.safety.lora_heartbeat], ['VCU WATCHDOG', raw?.safety.vcu_watchdog], ['RPI HEARTBEAT', raw?.safety.rpi_heartbeat], ['COMMAND FRESH', raw?.safety.command_fresh], ['SENSOR FRESH', raw?.safety.sensor_fresh], ['MOTOR ENABLE', raw?.safety.motor_enable]] as const).map(([name, value]) => <div key={name}><dt>{name}</dt><dd>{value == null ? '—' : value ? 'YES' : 'NO'}</dd></div>)}<dt>FAULTS</dt><dd>{raw?.safety.faults?.join(', ') || '—'}</dd></dl>{obstacle && <p>OBSTACLE {obstacle.id}: {fmt(obstacle.distance_m)} m · TTC {fmt(obstacle.ttc_s)} s · STOP {fmt(obstacle.stopping_distance_m)} m</p>}</section>}
      {view === 'BENCHMARK' && <section className="mc-card"><small>RUN SUMMARY</small><h2>{String(data.manifest.run_id ?? 'NO RUN')}</h2><dl>{Object.entries(data.manifest).filter(([key]) => key !== 'run_id').map(([key, value]) => <div key={key}><dt>{key.replaceAll('_', ' ').toUpperCase()}</dt><dd>{typeof value === 'object' ? JSON.stringify(value) : String(value)}</dd></div>)}</dl><p className="mc-muted">Missing metrics are not inferred from display samples.</p></section>}</div>}
    </div>{!isLab && data.mode !== 'REPLAY' && data.source !== 'DEMO' && <RosLivePanel source={data.source} frame={raw} />}</main>
    {isLab ? <><LabFooter frame={raw} plotsOpen={plotsOpen} onPlots={() => setPlotsOpen(!plotsOpen)} onSave={data.exportReplay} />{plotsOpen && <div className="mc-telemetry mc-lab-plots"><div className="mc-plot-groups">{Object.keys(PLOT_GROUPS).map((group) => <button key={group} className={plotGroup === group ? 'selected' : ''} onClick={() => setPlotGroup(group)}>{group}</button>)}</div><div className="mc-plots">{PLOT_GROUPS[plotGroup].map((spec) => <Plot key={spec.title} frames={data.frames} current={raw} {...spec} />)}</div></div>}</> : <footer className="mc-bottom"><div className="mc-transport"><button onClick={() => { data.setMode('LIVE'); data.setPlaying(false); }} className={data.mode === 'LIVE' ? 'selected' : ''}>LIVE</button><button onClick={pause} className={data.mode === 'PAUSE' ? 'selected' : ''} disabled={!data.frames.length}>PAUSE</button><button onClick={enterStep} className={data.mode === 'STEP' ? 'selected' : ''} disabled={!data.frames.length}>STEP</button><button onClick={() => setStage((current) => Math.min(STAGES.length, current + 1))} disabled={data.mode !== 'STEP' || stage >= STAGES.length}>NEXT STAGE</button><button onClick={() => { data.setMode('REPLAY'); data.setPlaying(false); }} className={data.mode === 'REPLAY' ? 'selected' : ''} disabled={!data.frames.length}>REPLAY</button><button onClick={() => data.setPlaying(!data.playing)} disabled={data.mode !== 'REPLAY' || data.frames.length < 2}>{data.playing ? 'PAUSE PLAYBACK' : 'PLAY'}</button><button onClick={() => data.openGatewayReplay().catch((error) => setErrorMessage(error.message))}>GATEWAY RUN</button><label className="mc-upload">OPEN FILE<input type="file" accept=".json,.jsonl" onChange={(event) => { const file = event.target.files?.[0]; if (file) data.loadReplay(file).catch((error) => setErrorMessage(error.message)); }} /></label><span>{data.mode === 'STEP' ? `${stage} / ${STAGES.length} · ${STAGES[stage - 1]}` : `${fmt(data.time, 3)} s`}</span><button className={plotsOpen ? 'selected' : ''} onClick={() => setPlotsOpen(!plotsOpen)} aria-expanded={plotsOpen}>TELEMETRY {plotsOpen ? '▾' : '▴'}</button></div><input className="mc-timeline" type="range" min={minTime} max={Math.max(minTime + 0.001, maxTime)} step="0.001" value={Math.min(maxTime, Math.max(minTime, data.time))} disabled={data.mode === 'LIVE' || data.frames.length < 2} onChange={(event) => { data.setPlaying(false); data.setTime(Number(event.target.value)); }} aria-label="Replay timeline" />{plotsOpen && <div className="mc-telemetry"><div className="mc-plot-groups">{Object.keys(PLOT_GROUPS).map((group) => <button key={group} className={plotGroup === group ? 'selected' : ''} onClick={() => setPlotGroup(group)}>{group}</button>)}</div><div className="mc-plots">{PLOT_GROUPS[plotGroup].map((spec) => <Plot key={spec.title} frames={data.frames} current={raw} {...spec} />)}</div></div>}</footer>}
  </div>;
}
