import { useCallback, useEffect, useRef, useState } from 'react';
import { LabNav } from './LabNav';
import { LAB_STAGES, gabrielRejected, stageSummary, stageVisible, type Diagnostic, type XY } from './labStages';
import './sharedLab.css';

/*
 * ALGORITHM LAB (/shared).
 * Backend: autokart_perception.lab_api on :8767 (proxied as /api/core/*), which runs the
 * real, tested autokart_perception.shared_core.AlgorithmCore against a kart_sim
 * PlanarBicycleKart. The frontend only draws what the backend returns; it never
 * recomputes geometry or control and never talks to ROS, the camera or a VCU.
 */

type Cone = { id: number; label: string; confidence: number; x_m: number; y_m: number };
export type Snapshot = { source: string; tick: number; cones: Cone[];
  kart: { x_m: number; y_m: number; yaw_rad: number; speed_mps: number };
  odometry: { x_m: number; y_m: number; yaw_rad: number }; diagnostic: Diagnostic | null };
type CoreStatus = { status: string; vehicle_authorized: boolean; algorithm_version: string; configuration_sha256: string };

/** Layer → index of the algorithm stage that produces it (see LAB_STAGES). */
const LAYER_STAGE = {
  'field of view': 0, observations: 0, 'tracked map (odom)': 0,
  delaunay: 1, gabriel: 2, boundaries: 3, gates: 4, centerline: 5, 'lookahead target': 7,
} as const;
type Layer = keyof typeof LAYER_STAGE;
const initialLayers = Object.fromEntries(Object.keys(LAYER_STAGE).map((name) => [name, name !== 'tracked map (odom)'])) as Record<Layer, boolean>;

async function request<T = Snapshot>(path: string, body?: object): Promise<T> {
  const response = await fetch(`/api/core/${path}`, body ? {
    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
  } : { cache: 'no-store' });
  if (!response.ok) throw new Error(`${path}: HTTP ${response.status}`);
  const type = response.headers.get('content-type') ?? '';
  if (!type.includes('application/json')) throw new Error(`${path}: expected JSON from lab_api, got ${type || 'unknown content'}`);
  return response.json() as Promise<T>;
}

const fmt = (value: number | null | undefined, places = 2) => value == null || !Number.isFinite(value) ? '—' : value.toFixed(places);

export function SharedLab() {
  const [snapshot, setSnapshot] = useState<Snapshot | null>(null);
  const [core, setCore] = useState<CoreStatus | null>(null);
  const [running, setRunning] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [layers, setLayers] = useState(initialLayers);
  const [focus, setFocus] = useState<number | null>(null);
  const [animateStages, setAnimateStages] = useState(false);
  const [error, setError] = useState('');
  const busy = useRef(false);
  const [dragging, setDragging] = useState<number | null>(null);
  const refresh = useCallback(async (path: string, body?: object) => {
    if (busy.current) return;
    busy.current = true;
    try { setSnapshot(await request(path, body)); setError(''); }
    catch (cause) { setError(cause instanceof Error ? cause.message : String(cause)); setRunning(false); }
    finally { busy.current = false; }
  }, []);
  useEffect(() => { document.title = 'AutoKart Algorithm Lab'; }, []);
  useEffect(() => {
    void refresh('snapshot');
    request<CoreStatus>('status').then(setCore).catch(() => setCore(null));
  }, [refresh]);
  useEffect(() => {
    if (!running) return;
    const timer = window.setInterval(() => { void refresh('step', {}); }, 120 / speed);
    return () => clearInterval(timer);
  }, [running, speed, refresh]);
  useEffect(() => {
    if (!animateStages) return;
    const timer = window.setInterval(() => setFocus((current) => current == null ? 0 : (current + 1) % LAB_STAGES.length), 900 / speed);
    return () => clearInterval(timer);
  }, [animateStages, speed]);
  const stepStage = (delta: number) => { setAnimateStages(false); setFocus((current) => {
    const next = (current ?? (delta > 0 ? -1 : LAB_STAGES.length)) + delta;
    return Math.max(0, Math.min(LAB_STAGES.length - 1, next));
  }); };
  useEffect(() => {
    const onKey = (event: KeyboardEvent) => {
      if (event.target instanceof HTMLInputElement || event.target instanceof HTMLSelectElement) return;
      if (event.key === 'ArrowRight') stepStage(1);
      else if (event.key === 'ArrowLeft') stepStage(-1);
      else if (event.key === 'Escape') { setAnimateStages(false); setFocus(null); }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, []);

  const diagnostic = snapshot?.diagnostic;
  const pose = snapshot?.odometry ?? { x_m: 0, y_m: 0, yaw_rad: 0 };
  const toWorld = ([x, y]: XY): XY => [
    pose.x_m + Math.cos(pose.yaw_rad) * x - Math.sin(pose.yaw_rad) * y,
    pose.y_m + Math.sin(pose.yaw_rad) * x + Math.cos(pose.yaw_rad) * y,
  ];
  const sx = (x: number) => 70 + x * 22;
  const sy = (y: number) => 280 - y * 22;
  const observations = new Map(diagnostic?.observations.map(item => [item.id, toWorld(item.xy)]) ?? []);
  const boundaryPoints = (ids: number[]) => ids.map(id => observations.get(id)).filter((value): value is XY => value !== undefined)
    .map(value => `${sx(value[0])},${sy(value[1])}`).join(' ');
  const edge = ([a, b]: [number, number], kind: string, index: number) => {
    const start = observations.get(a), end = observations.get(b);
    return start && end ? <line key={`${kind}-${index}`} x1={sx(start[0])} y1={sy(start[1])}
      x2={sx(end[0])} y2={sy(end[1])} className={`shared-${kind}`} /> : null;
  };
  const pointerWorld = (event: React.PointerEvent<SVGSVGElement>) => {
    const bounds = event.currentTarget.getBoundingClientRect();
    const x = (event.clientX - bounds.left) * 1000 / bounds.width;
    const y = (event.clientY - bounds.top) * 560 / bounds.height;
    return { x_m: (x - 70) / 22, y_m: (280 - y) / 22 };
  };
  /** Layer is drawn when its checkbox is on and its stage is at/before the focused stage. */
  const show = (layer: Layer) => layers[layer] && stageVisible(focus, LAYER_STAGE[layer]);
  const stageClass = (layer: Layer) => focus != null && LAYER_STAGE[layer] === focus ? 'stage-focus' : focus != null ? 'stage-dim' : '';
  const focusedStage = focus == null ? null : LAB_STAGES[focus];
  const focusedSummary = focusedStage ? stageSummary(diagnostic, focusedStage.id) : null;
  const target = diagnostic?.control_candidate.target_point_base_m ? toWorld(diagnostic.control_candidate.target_point_base_m) : null;
  const authorityViolation = diagnostic?.vehicle_authorized === true || diagnostic?.control_candidate.published_to_vcu === true || core?.vehicle_authorized === true;

  return <main className="shared-lab">
    <LabNav current="algorithm" />
    <header className="shared-head"><div><b>ALGORITHM LAB</b>
      <span>Real Python <code>autokart_perception.shared_core</code> via <code>lab_api</code> :8767 · source <b className="shared-sim">{snapshot?.source ?? 'SIMULATED'}</b> · kart physics: <code>kart_sim.PlanarBicycleKart</code></span></div>
      <dl className="shared-provenance"><dt>ALGORITHM</dt><dd>{diagnostic?.algorithm_version ?? core?.algorithm_version ?? '—'}</dd>
        <dt>CONFIG SHA</dt><dd title={core?.configuration_sha256}>{(diagnostic?.configuration_sha256 ?? core?.configuration_sha256 ?? '—').slice(0, 12)}</dd>
        <dt>MISSION</dt><dd>{diagnostic?.mission ?? '—'}</dd></dl>
    </header>
    {authorityViolation && <p className="shared-error" role="alert">UNEXPECTED: backend reports vehicle_authorized/published_to_vcu = true. The LAB must never authorize a vehicle — stop and investigate lab_api.</p>}
    <section className="shared-toolbar">
      <button onClick={() => setRunning(!running)}>{running ? 'Pause' : 'Start'}</button>
      <button onClick={() => void refresh('step', {})}>Single frame</button>
      <button onClick={() => { setRunning(false); void refresh('reset', {}); }}>Reset</button>
      <label>Speed <select value={speed} onChange={event => setSpeed(Number(event.target.value))}>
        <option value={0.5}>0.5×</option><option value={1}>1×</option><option value={2}>2×</option><option value={4}>4×</option>
      </select></label>
      <span className="shared-sep" />
      <button onClick={() => stepStage(-1)} disabled={focus === 0} aria-label="Previous stage">◀ Stage</button>
      <button onClick={() => stepStage(1)} disabled={focus === LAB_STAGES.length - 1} aria-label="Next stage">Stage ▶</button>
      <button className={animateStages ? 'selected' : ''} aria-pressed={animateStages} onClick={() => setAnimateStages(!animateStages)}>{animateStages ? 'Stop stage animation' : 'Animate stages'}</button>
      <button className={focus == null ? 'selected' : ''} onClick={() => { setAnimateStages(false); setFocus(null); }}>All stages</button>
      <strong>Vehicle authority: {diagnostic ? (diagnostic.vehicle_authorized ? 'AUTHORIZED (!)' : 'DISABLED') : 'DISABLED (no diagnostic yet)'}</strong>
    </section>
    <ol className="shared-ribbon" aria-label="Algorithm stages">{LAB_STAGES.map((stage, index) => {
      const summary = stageSummary(diagnostic, stage.id);
      return <li key={stage.id}><button className={`state-${summary.state.replace('/', '')} ${focus === index ? 'active' : ''} ${focus != null && index > focus ? 'later' : ''}`}
        aria-pressed={focus === index} onClick={() => { setAnimateStages(false); setFocus(focus === index ? null : index); }}>
        <small>{index + 1}</small><b>{stage.label}</b><span>{summary.count}</span><em>{summary.state}</em></button></li>;
    })}</ol>
    {error && <p className="shared-error" role="alert">LAB API unavailable ({error}). Start it with <code>autokart_apps/run_lab.sh</code> (port 8767). Nothing is simulated in the browser as a fallback.</p>}
    <div className="shared-layout">
      <div className="shared-world">
      <svg viewBox="0 0 1000 560" aria-label="Algorithm Lab world view" onPointerMove={event => {
        if (dragging === null) return;
        const world = pointerWorld(event);
        setSnapshot(current => current && ({ ...current, cones: current.cones.map(cone =>
          cone.id === dragging ? { ...cone, ...world } : cone) }));
      }} onPointerUp={event => {
        if (dragging === null) return;
        const world = pointerWorld(event);
        void refresh('move-cone', { id: dragging, ...world });
        setDragging(null);
      }}>
        <rect width="1000" height="560" fill="#07121d" />
        {Array.from({ length: 23 }, (_, i) => <g key={i} className="shared-grid">
          <line x1={70 + 44 * i - 44} y1="0" x2={70 + 44 * i - 44} y2="560" />
          <line x1="0" y1={16 + 44 * i} x2="1000" y2={16 + 44 * i} />
        </g>)}
        <line x1="0" y1={sy(0)} x2="1000" y2={sy(0)} className="shared-axis" />
        <line x1={sx(0)} y1="0" x2={sx(0)} y2="560" className="shared-axis" />
        {show('field of view') && snapshot && <path className={`shared-fov ${stageClass('field of view')}`} d={`M ${sx(snapshot.kart.x_m)} ${sy(snapshot.kart.y_m)} L ${sx(snapshot.kart.x_m + 12 * Math.cos(snapshot.kart.yaw_rad - 1.1))} ${sy(snapshot.kart.y_m + 12 * Math.sin(snapshot.kart.yaw_rad - 1.1))} A ${12 * 22} ${12 * 22} 0 0 0 ${sx(snapshot.kart.x_m + 12 * Math.cos(snapshot.kart.yaw_rad + 1.1))} ${sy(snapshot.kart.y_m + 12 * Math.sin(snapshot.kart.yaw_rad + 1.1))} Z`} />}
        {show('tracked map (odom)') && <g className={stageClass('tracked map (odom)')}>{diagnostic?.tracked_cones?.map(item => <rect key={`track-${item.id}`} className="shared-tracked"
          x={sx(item.x_m) - 4} y={sy(item.y_m) - 4} width="8" height="8" />)}</g>}
        {show('delaunay') && <g className={stageClass('delaunay')}>
          {diagnostic?.geometry.delaunay_triangles?.map((tri, i) => {
            const points = tri.map(id => observations.get(id));
            return points.every(Boolean) ? <polygon key={`tri-${i}`} className="shared-triangle" points={points.map(p => `${sx(p![0])},${sy(p![1])}`).join(' ')} /> : null;
          })}
          {diagnostic?.geometry.delaunay_edges.map((value, i) => edge(value, 'delaunay', i))}</g>}
        {show('gabriel') && <g className={stageClass('gabriel')}>
          {gabrielRejected(diagnostic).map((value, i) => edge(value, 'gabriel-rejected', i))}
          {diagnostic?.geometry.gabriel_edges.map((value, i) => edge(value, 'gabriel', i))}</g>}
        {show('boundaries') && diagnostic && <g className={stageClass('boundaries')}><polyline className="shared-left-boundary" points={boundaryPoints(diagnostic.geometry.boundaries.left_ids)} /><polyline className="shared-right-boundary" points={boundaryPoints(diagnostic.geometry.boundaries.right_ids)} /></g>}
        {show('gates') && <g className={stageClass('gates')}>
          {diagnostic?.geometry.rejected_candidates?.map((gate, i) => edge([gate.left_id, gate.right_id], 'gate-rejected', i))}
          {diagnostic?.geometry.gates.map((gate, i) => {
            const p = toWorld(gate.midpoint);
            return <g key={`gate-${i}`}>{edge([gate.left_id, gate.right_id], gate.status === 'CANDIDATE' ? 'gate' : 'gate-rejected', i)}
              <circle cx={sx(p[0])} cy={sy(p[1])} r="4" fill={gate.status === 'CANDIDATE' ? '#fff' : '#f88'} /></g>;
          })}</g>}
        {show('centerline') && diagnostic && diagnostic.geometry.centerline.length > 0 && <polyline className={`shared-path ${stageClass('centerline')}`}
          points={diagnostic.geometry.centerline.map(value => { const p = toWorld(value); return `${sx(p[0])},${sy(p[1])}`; }).join(' ')} />}
        {show('lookahead target') && target && snapshot && <g className={stageClass('lookahead target')}>
          <line className="shared-lookahead" x1={sx(pose.x_m)} y1={sy(pose.y_m)} x2={sx(target[0])} y2={sy(target[1])} />
          <circle className="shared-target" cx={sx(target[0])} cy={sy(target[1])} r="7" /></g>}
        {snapshot?.cones.map(cone => <circle key={cone.id} cx={sx(cone.x_m)} cy={sy(cone.y_m)} r="8" className="shared-cone"
          fill={cone.label === 'blue' ? '#399bff' : '#ffd645'} stroke="#fff" strokeWidth="1.5"
          onPointerDown={event => { setDragging(cone.id); event.currentTarget.setPointerCapture(event.pointerId); }} />)}
        {show('observations') && <g className={stageClass('observations')}>{diagnostic?.observations.map(item => {
          const p = observations.get(item.id);
          return p ? <circle key={`observation-${item.id}`} cx={sx(p[0])} cy={sy(p[1])} r="12" className="shared-observation" /> : null;
        })}</g>}
        {snapshot && <g transform={`translate(${sx(pose.x_m)} ${sy(pose.y_m)}) rotate(${-pose.yaw_rad * 180 / Math.PI})`}>
          <path d="M 14 0 L -10 -8 L -10 8 Z" className="shared-odom" /></g>}
        {snapshot && <g transform={`translate(${sx(snapshot.kart.x_m)} ${sy(snapshot.kart.y_m)}) rotate(${-snapshot.kart.yaw_rad * 180 / Math.PI})`}>
          <path d="M 14 0 L -10 -8 L -10 8 Z" fill="#36edc1" /></g>}
        <text x="80" y="35" fill="#9bb2c8" fontSize="12">X forward → · Y left ↑ · grid 2 m · frame odom</text>
        {focusedStage && <text x="80" y="54" fill="#d9cbff" fontSize="13" fontWeight="700">STAGE {focus! + 1} · {focusedStage.label} — later stages hidden</text>}
      </svg>
      <div className="shared-legend"><span><i className="kart" /> kart (simulator truth)</span><span><i className="odom" /> odom pose used by shared_core</span>
        <span><i className="cone" /> cones (drag to move)</span><span><i className="ring" /> observed this frame</span><span><i className="tri" /> Delaunay</span>
        <span><i className="gab" /> Gabriel kept</span><span><i className="rej" /> removed / rejected</span><span><i className="gate" /> gate</span>
        <span><i className="path" /> centerline</span><span><i className="tgt" /> Pure-Pursuit target</span></div>
      </div>
      <aside>
        <h2>{focusedStage ? `Stage ${focus! + 1} · ${focusedStage.label}` : 'All stages'}</h2>
        {focusedStage && focusedSummary ? <div className="shared-stage-detail">
          <p><code>{focusedStage.module}</code></p><p>{focusedStage.explain}</p>
          <dl>{focusedSummary.facts.map(([name, value]) => <div key={name}><dt>{name}</dt><dd>{value}</dd></div>)}</dl>
        </div> : <p>Click a stage in the ribbon, use ◀/▶ (or arrow keys), or “Animate stages” to walk the pipeline. Esc returns to all stages.</p>}
        <h2>Layers</h2>{(Object.keys(layers) as Layer[]).map(name => <label key={name}>
          <input type="checkbox" checked={layers[name]} onChange={() => setLayers(current => ({ ...current, [name]: !current[name] }))} />{name}
          <small> · stage {LAYER_STAGE[name] + 1}</small></label>)}
        <h2>Shared computation</h2>
        <dl><dt>Frame</dt><dd>{snapshot?.tick ?? 0}</dd>
          <dt>Cones / gates</dt><dd>{diagnostic?.observations.length ?? 0} / {diagnostic?.geometry.gates.length ?? 0}</dd>
          <dt>Path valid</dt><dd>{String(diagnostic?.planning.path_valid ?? false)}</dd>
          <dt>Mission state</dt><dd>{diagnostic?.mission_state.state ?? 'waiting'}</dd>
          <dt>Speed (sim)</dt><dd>{fmt(snapshot?.kart.speed_mps)} m/s</dd>
          <dt>Target</dt><dd>{fmt(diagnostic?.control_candidate.target_speed_mps)} m/s</dd>
          <dt>Steering</dt><dd>{fmt(diagnostic?.control_candidate.steering_angle_rad, 3)} rad</dd>
          <dt>Odom drift</dt><dd>{snapshot ? `${fmt(Math.hypot(snapshot.kart.x_m - pose.x_m, snapshot.kart.y_m - pose.y_m), 3)} m` : '—'}</dd>
          <dt>Reasons</dt><dd>{diagnostic?.reasons.join(', ') || 'none'}</dd></dl>
        <p>Drag a cone to change the next simulated observation. All geometry and control values come from the Python service; the browser only filters what to draw.</p>
      </aside>
    </div>
  </main>;
}
