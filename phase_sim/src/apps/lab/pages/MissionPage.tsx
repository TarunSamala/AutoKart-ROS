import { useEffect, useRef, useState } from 'react';
import { MissionScene, type Layer, type Selection } from '../../../mission/MissionScene';
import { RosLivePanel } from '../../../mission/RosLivePanel';
import { STAGES, stageFrame, type MissionFrame } from '../../../mission/schema';
import { SOURCE_LABEL, useMissionData } from '../../../mission/useMissionData';
import type { ScenarioName } from '../../../mission/labScenarios';
import '../../../mission/mission.css';
import { Link } from '../router';
import { useReportProvenance, useRun } from '../runContext';
import { RecordingPicker } from './RecordingPicker';

/*
 * Lab mission view (/mission and /replay).
 *
 * Composition, not a fork: it renders the existing MissionScene (three.js), useMissionData
 * (sandbox engine, ROS websocket, replay decoder) and RosLivePanel unchanged, but supplies
 * its OWN source picker and toolbar. MissionControl.tsx (whose picker includes REAL_KART)
 * is NOT imported anywhere in the Lab bundle.
 *
 * REAL_KART blocking:
 *  1. The only source options are LAB (client sandbox), SIMULATION (Lab sim gateway) and REPLAY.
 *     `selectSource` is typed to those three, so REAL_KART cannot be passed from this file.
 *  2. A recording whose frames are tagged REAL_KART can be replayed (mode REPLAY: no socket,
 *     labelled "replay of a REAL KART recording · not live"). If the hook ever ends up with
 *     source REAL_KART (or DEMO) outside REPLAY mode, the guard effect below immediately
 *     resets it to the sandbox before any live connection is shown.
 *  3. /mission/* is proxied only to the Lab sim gateway (:8775), so even a misrouted socket
 *     would receive SIMULATION data and be rejected as SOURCE MISMATCH by useMissionData.
 */

type LabSource = 'LAB' | 'SIMULATION' | 'REPLAY';
const INITIAL_LAYERS: Record<Layer, boolean> = { groundTruth: true, tracked: true, detections: true, delaunay: true, gabriel: true, gates: true, boundaries: true, candidates: true, selected: true, safety: true, sensor: true };
const RIBBON = ['SEE', 'DELAUNAY', 'GABRIEL', 'GATES', 'CANDIDATES', 'MISSION', 'SELECT', 'CONTROL'] as const;
const fmt = (value: number | null | undefined, places = 2) => value == null || !Number.isFinite(value) ? '—' : value.toFixed(places);
const SOURCE_TEXT: Record<LabSource, string> = { LAB: 'SANDBOX · CLIENT JS', SIMULATION: 'ROS SIMULATION · :8775', REPLAY: 'REPLAY · RECORDED' };

function ribbonCount(frame: MissionFrame | null, index: number): string {
  if (!frame) return '—';
  const p = frame.planning;
  switch (index) {
    case 0: return String(frame.perception.cones.length);
    case 1: return String(p.delaunay_edges.length);
    case 2: return String(p.gabriel_edges.length);
    case 3: return String(p.gates.length);
    case 4: return String(p.candidate_paths.length);
    case 5: return p.skidpad?.state ?? '—';
    case 6: return String(p.selected_path.length);
    default: return frame.vehicle.requested_steering_rad == null ? '—' : `${fmt(frame.vehicle.requested_steering_rad, 3)} rad`;
  }
}

function Inspector({ frame, selection, onClose }: { frame: MissionFrame | null; selection: Selection; onClose: () => void }) {
  const p = frame?.planning;
  const cone = frame && [...frame.perception.cones, ...(frame.perception.ground_truth_cones ?? []), ...(frame.visualization?.course_cones ?? [])].find((item) => item.id === selection.id);
  const edge = p && [...p.gabriel_edges, ...p.delaunay_edges].find((item) => item.id === selection.id);
  const gate = p?.gates.find((item) => item.id === selection.id);
  const candidate = p?.candidate_paths.find((item) => item.id === selection.id);
  let rows: Array<[string, string]> = [];
  if (!frame) rows = [['STATUS', 'No frame']];
  else if (selection.kind === 'cone' && cone) rows = [['CLASS', cone.class], ['POSITION', `${fmt(cone.x_m)}, ${fmt(cone.y_m)} m`], ['CONFIDENCE', fmt(cone.confidence)], ['RANGE / BEARING', `${fmt(cone.range_m)} m / ${fmt(cone.bearing_rad)} rad`]];
  else if (selection.kind === 'edge' && edge) rows = [['ENDPOINTS', `${edge.a} ↔ ${edge.b}`], ['GABRIEL', edge.accepted === undefined ? 'NO DEBUG' : edge.accepted ? 'ACCEPTED' : 'REJECTED'], ['BLOCKING CONE', edge.blocking_cone_id ?? '—'], ['REASON', edge.reason ?? '—']];
  else if (selection.kind === 'gate' && gate) rows = [['WIDTH', `${fmt(gate.width_m)} m`], ['VALID', String(gate.valid)], ['CONFIDENCE', fmt(gate.confidence)], ['REASON', gate.reason ?? '—']];
  else if (selection.kind === 'candidate' && candidate) rows = [['RANK', String(candidate.rank)], ['SELECTED', String(candidate.selected)], ['TOTAL COST', fmt(candidate.total_cost)], ...Object.entries(candidate.costs).map(([key, value]) => [key.toUpperCase(), fmt(value)] as [string, string]), ['REASON', candidate.reason ?? '—']];
  else rows = [['POSE', `${fmt(frame.vehicle.x_m)}, ${fmt(frame.vehicle.y_m)} m`], ['HEADING', `${fmt(frame.vehicle.yaw_rad)} rad`], ['SPEED', `${fmt(frame.vehicle.speed_mps)} m/s`], ['STEERING REQUEST (SIM)', `${fmt(frame.vehicle.requested_steering_rad, 3)} rad`]];
  return <section className="mc-floating mc-inspector" aria-label="Selection inspector">
    <div className="mc-inspector-head"><small>INSPECT · {selection.kind.toUpperCase()}</small><button onClick={onClose} aria-label="Close inspector">×</button></div>
    <h2>{selection.id}</h2>
    <dl>{rows.map(([name, value]) => <div key={name}><dt>{name}</dt><dd>{value}</dd></div>)}</dl>
  </section>;
}

export function MissionPage({ variant }: { variant: 'mission' | 'replay' }) {
  const { recording } = useRun();
  if (variant === 'replay' && !recording) return <ReplayEmpty />;
  return <MissionView variant={variant} />;
}

function ReplayEmpty() {
  useReportProvenance(null, '');
  return <article className="lab-page lab-doc" data-page="replay" aria-labelledby="replay-title">
    <p className="lab-eyebrow">DATA · REPLAY + TIMELINE</p>
    <h1 id="replay-title">Replay and timeline</h1>
    <p className="lab-warn-text" role="status">No recording loaded. Open a MissionFrame <code>.jsonl</code> / <code>.json</code> recording to play, pause, step and seek it through the 3D mission view.</p>
    <RecordingPicker />
    <p className="lab-note">Recordings come from the sandbox (<Link to="/mission">Mission → SAVE RUN</Link>) or any MissionFrame schema_version 1 file. They are held in this tab's memory only.</p>
  </article>;
}

function MissionView({ variant }: { variant: 'mission' | 'replay' }) {
  const data = useMissionData();
  const { run, recording } = useRun();
  const [layers, setLayers] = useState(INITIAL_LAYERS);
  const [layersOpen, setLayersOpen] = useState(false);
  const [camera, setCamera] = useState('algorithm');
  const [selection, setSelection] = useState<Selection>({ kind: 'kart', id: 'vehicle' });
  const [inspectorOpen, setInspectorOpen] = useState(false);
  const [stage, setStage] = useState(1);
  const [replayStep, setReplayStep] = useState(false);
  const [reveal, setReveal] = useState(false);
  const [revealStage, setRevealStage] = useState(1);
  const [error, setError] = useState('');
  const [blockedNotice, setBlockedNotice] = useState('');
  const loadedRecording = useRef<File | null>(null);

  // Apply the run manager's scenario/seed to the sandbox once on entry.
  useEffect(() => {
    if (variant === 'mission' && !(run.mode === 'REPLAY' && recording)) data.configureLab(run.scenario, run.settings);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // Replay the run manager's recording (through the existing useMissionData.loadReplay decoder).
  useEffect(() => {
    const wantReplay = variant === 'replay' || run.mode === 'REPLAY';
    if (!wantReplay || !recording || loadedRecording.current === recording.file) return;
    loadedRecording.current = recording.file;
    data.loadReplay(recording.file).catch((cause: unknown) => setError(cause instanceof Error ? cause.message : String(cause)));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [recording, variant, run.mode]);

  // Guard: a live REAL_KART (or legacy DEMO) source is never allowed outside REPLAY mode in the Lab.
  useEffect(() => {
    if ((data.source === 'REAL_KART' || data.source === 'DEMO') && data.mode !== 'REPLAY') {
      setBlockedNotice(`Source ${data.source} is not available in the Lab; switched back to the sandbox.`);
      data.setSource('LAB');
    }
  }, [data.source, data.mode, data]);

  useEffect(() => {
    if (!reveal || data.mode !== 'LIVE') return;
    const timer = window.setInterval(() => setRevealStage((current) => current % STAGES.length + 1), 820);
    return () => window.clearInterval(timer);
  }, [reveal, data.mode]);

  const replaying = data.mode === 'REPLAY';
  const labSource: LabSource = replaying ? 'REPLAY' : data.source === 'SIMULATION' ? 'SIMULATION' : 'LAB';
  const isSandbox = labSource === 'LAB';
  const raw = data.current;
  const stepping = replaying ? replayStep : data.mode === 'STEP';
  const activeStage = stepping ? stage : reveal && data.mode === 'LIVE' ? revealStage : 0;
  const frame = raw && activeStage ? stageFrame(raw, activeStage) : raw;
  const minTime = data.frames[0]?.timestamp_s ?? 0;
  const maxTime = data.frames[data.frames.length - 1]?.timestamp_s ?? 0;
  const canReplay = !!recording || data.frames.length >= 2;

  const provenanceDetail = replaying
    ? `replay of a ${SOURCE_LABEL[data.source]} recording${recording && loadedRecording.current === recording.file ? ` (${recording.name})` : ' (this session)'} · recorded, not live`
    : labSource === 'SIMULATION' ? 'Lab simulation ROS gateway :8775 (separate ROS domain) · read-only' : `client-side JS sandbox (LabSkidpad) · ${data.labScenario} · seed ${data.labSettings.seed}`;
  // Nothing displayed (e.g. sim gateway offline) -> no active provenance is claimed.
  useReportProvenance(raw ? (replaying ? 'REPLAY' : 'SYNTHETIC') : null, provenanceDetail);

  const selectSource = (next: LabSource) => {
    setError(''); setBlockedNotice(''); setReplayStep(false);
    if (next === 'REPLAY') {
      if (recording) { loadedRecording.current = recording.file; data.loadReplay(recording.file).catch((cause: unknown) => setError(cause instanceof Error ? cause.message : String(cause))); }
      else if (data.frames.length >= 2) { data.setPlaying(false); data.setMode('REPLAY'); }
      return;
    }
    data.setSource(next);
  };
  const pause = () => { data.setTime(raw?.timestamp_s ?? data.time); data.setMode('PAUSE'); data.setPlaying(false); };
  // In replay, stepping stays in REPLAY mode (no socket is ever opened for a recording's source).
  const enterStep = () => {
    data.setTime(raw?.timestamp_s ?? data.time); data.setPlaying(false); setStage(1);
    if (replaying) setReplayStep(true); else data.setMode('STEP');
  };
  const choose = (item: Selection) => { setSelection(item); setInspectorOpen(true); };
  const obstacle = raw?.safety.obstacle;
  const scenarioName = String(data.manifest.scenario ?? raw?.scenario ?? '—').toUpperCase();

  return <div className="lab-page lab-mission mission-shell" data-page={variant}>
    <h1 className="lab-visually-hidden">{variant === 'replay' ? 'Replay and timeline' : 'Mission view'}</h1>
    <header className="mc-top mc-lab-top">
      <div className="mc-brand"><span aria-hidden="true">AK</span><div><strong>{variant === 'replay' ? 'REPLAY + TIMELINE' : 'MISSION VIEW'}</strong><small>SIM / REPLAY SOURCES ONLY</small></div></div>
      <div className="mc-lab-controls">
        <label>SOURCE<select aria-label="Data source" value={labSource} onChange={(event) => selectSource(event.target.value as LabSource)}>
          <option value="LAB">{SOURCE_TEXT.LAB}</option>
          <option value="SIMULATION">{SOURCE_TEXT.SIMULATION}</option>
          <option value="REPLAY" disabled={!canReplay}>{SOURCE_TEXT.REPLAY}{canReplay ? '' : ' (open a recording)'}</option>
        </select></label>
        {isSandbox && <label>SCENARIO<select aria-label="Scenario" value={data.labScenario} onChange={(event) => data.configureLab(event.target.value as ScenarioName, data.labSettings)}>
          <option value="SKIDPAD">SKIDPAD</option><option value="AUTOCROSS">AUTOCROSS</option><option value="BRAKE">BRAKE</option></select></label>}
        {isSandbox && <button className="mc-run" onClick={() => { setReplayStep(false); data.runLab(); }} disabled={data.mode === 'LIVE'}>▶ RUN</button>}
        {isSandbox && <button onClick={pause} disabled={data.mode !== 'LIVE'}>⏸ PAUSE</button>}
        {replaying && <button className="mc-run" onClick={() => { setReplayStep(false); data.setPlaying(!data.playing); }} disabled={data.frames.length < 2}>{data.playing ? '⏸ PAUSE' : '▶ PLAY'}</button>}
        {labSource === 'SIMULATION' && <button onClick={() => { data.setMode('LIVE'); data.setPlaying(false); }} disabled={data.mode === 'LIVE'}>FOLLOW SIM</button>}
        {labSource === 'SIMULATION' && <button onClick={pause} disabled={data.mode !== 'LIVE' || !data.frames.length}>⏸ PAUSE</button>}
        <button onClick={enterStep} disabled={!raw}>STEP</button>
        <button onClick={() => setStage((current) => Math.min(STAGES.length, current + 1))} disabled={!stepping || stage >= STAGES.length}>NEXT STAGE</button>
        {isSandbox && <button onClick={data.resetLab}>↺ RESET</button>}
        <strong className="mc-lab-state" role="status">{data.status}</strong>
      </div>
      <div className="mc-clock">{fmt(raw?.timestamp_s, 2)} <small>s</small></div>
    </header>
    <div className="mc-status mc-lab-status">
      <span className="mc-sandbox-badge">{replaying ? 'REPLAY · RECORDED, NOT LIVE' : labSource === 'SIMULATION' ? 'ROS SIMULATION · LAB DOMAIN · READ-ONLY' : 'SANDBOX · client-side JS (LabSkidpad.ts) · not the Python core — see Core Lab'}</span>
      <span>SCENARIO {scenarioName}</span><span>{data.mode}</span><span>MISSION {raw?.planning.skidpad?.state ?? '—'}</span>
      <span>RUN {String(data.manifest.run_id ?? raw?.run_id ?? '—')}</span>
      <span>CONTROLLER OUTPUT · SIM-ONLY / DIAGNOSTIC · VEHICLE AUTHORITY OFF</span>
    </div>
    {blockedNotice && <div className="mc-error" role="alert">{blockedNotice}</div>}
    {error && <div className="mc-error" role="alert">{error}</div>}
    {replaying && data.source === 'REAL_KART' && <div className="mc-error" role="note">This recording was captured from the REAL KART. It is replayed from the file only — no live connection is opened.</div>}
    <div className="mc-main"><div className="mc-world">
      <MissionScene frame={frame} layers={layers} cameraMode={camera} selection={selection} onSelect={choose} animationStage={activeStage} />
      {!raw && <div className="mc-waiting-geometry" role="status"><b>{labSource === 'SIMULATION' ? 'NO CURRENT SIMULATION FRAME' : variant === 'replay' && !recording ? 'NO RECORDING LOADED' : 'WAITING FOR A FRAME'}</b>
        <small>{labSource === 'SIMULATION' ? `Simulation gateway status: ${data.status}. Nothing is substituted while it is offline.` : variant === 'replay' && !recording ? 'Open a MissionFrame recording below or on Lab Home.' : 'No frame yet.'}</small></div>}
      <div className="mc-world-top"><div className="mc-world-title"><small>{SOURCE_TEXT[labSource]}</small><h2>{scenarioName === '—' ? 'Waiting for track data' : scenarioName}</h2></div>
        <div className="mc-world-controls">
          <button className={reveal ? 'selected' : ''} onClick={() => setReveal(!reveal)} aria-pressed={reveal} title="Display only: progressively reveals the stages of the current frame; it does not run the algorithm">STAGE REVEAL</button>
          <select value={camera} onChange={(event) => setCamera(event.target.value)} aria-label="Camera mode"><option value="algorithm">ALGORITHM</option><option value="top">TOP</option><option value="chase">CHASE</option><option value="driver">DRIVER</option><option value="free">FREE</option></select>
          <button onClick={() => setLayersOpen(!layersOpen)} aria-expanded={layersOpen}>LAYERS</button>
        </div></div>
      <ol className="mc-world-pipeline" aria-label="Algorithm stage ribbon">{RIBBON.map((name, index) => <li key={name} className={activeStage === index + 1 ? 'active' : ''} aria-current={activeStage === index + 1 ? 'step' : undefined}>
        <b>{name}</b><small>{ribbonCount(raw, index)}</small></li>)}</ol>
      {activeStage > 0 && <div className="mc-stage-note"><b>{activeStage} · {STAGES[activeStage - 1]}</b><span>{stepping ? 'Stepping the current frame stage by stage' : 'Stage reveal (display only)'}</span></div>}
      {layersOpen && <div className="mc-layerbar">{(Object.keys(layers) as Layer[]).map((layer) => <label key={layer}><input type="checkbox" checked={layers[layer]} onChange={() => setLayers((current) => ({ ...current, [layer]: !current[layer] }))} />{layer.replace(/([A-Z])/g, ' $1').toUpperCase()}</label>)}</div>}
      <div className="mc-legend"><b>VISUAL KEY</b><span><i className="blue" /> BLUE · LEFT</span><span><i className="yellow" /> YELLOW · RIGHT</span><span><i className="orange" /> ORANGE · SPECIAL</span><span><i className="cyan line" /> DELAUNAY</span><span><i className="green line" /> GABRIEL ACCEPT</span><span><i className="red line" /> GABRIEL REJECT</span><span><i className="white line" /> CANDIDATE</span><span><i className="magenta line" /> SELECTED</span></div>
      {obstacle && <div className={`mc-floating mc-safety ${obstacle.brake_trigger ? 'braking' : ''}`}><small>OBSTACLE {obstacle.id} · SIM</small><strong>{obstacle.brake_trigger ? 'BRAKE (SIM)' : 'MONITORING (SIM)'}</strong><p>Obstacle {fmt(obstacle.distance_m)} m · TTC {fmt(obstacle.ttc_s)} s<br />Required stop {fmt(obstacle.stopping_distance_m)} m · margin {fmt(obstacle.margin_m)} m</p></div>}
      {inspectorOpen && <Inspector frame={raw} selection={selection} onClose={() => setInspectorOpen(false)} />}
    </div>{labSource === 'SIMULATION' && <RosLivePanel source="SIMULATION" frame={raw} />}</div>
    <footer className="mc-bottom">
      <div className="mc-transport">
        <span>SPEED <b>{fmt(raw?.vehicle.speed_mps)} m/s</b></span>
        <span>STEER REQ (SIM) <b>{fmt(raw?.vehicle.requested_steering_rad, 3)} rad</b></span>
        <span>W R/L <b>{fmt(raw?.planning.skidpad?.w_right)} / {fmt(raw?.planning.skidpad?.w_left)}</b></span>
        <span>{stepping ? `${stage} / ${STAGES.length} · ${STAGES[stage - 1]}` : `${fmt(data.time, 3)} s`}</span>
        {isSandbox && <button onClick={data.exportReplay} disabled={data.frames.length < 2}>SAVE RUN (.jsonl)</button>}
        {labSource === 'SIMULATION' && <button onClick={() => data.openGatewayReplay().catch((cause: Error) => setError(cause.message))}>SIM GATEWAY RECORDING</button>}
        {variant === 'replay' && <Link to="/dataset" className="lab-inline-link">Inspect frames in Dataset</Link>}
      </div>
      <input className="mc-timeline" type="range" min={minTime} max={Math.max(minTime + 0.001, maxTime)} step="0.001" value={Math.min(maxTime, Math.max(minTime, data.time))}
        disabled={!replaying || data.frames.length < 2} onChange={(event) => { data.setPlaying(false); setReplayStep(false); data.setTime(Number(event.target.value)); }} aria-label="Replay timeline" />
      {(variant === 'replay' || labSource !== 'SIMULATION') && <div className="lab-mission-picker"><RecordingPicker label="Open recording for replay" /></div>}
    </footer>
  </div>;
}
