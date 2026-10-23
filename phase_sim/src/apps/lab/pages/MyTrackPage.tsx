import { useEffect, useMemo, useRef, useState } from 'react';
import { useMissionData } from '../../../mission/useMissionData';
import type { MissionFrame } from '../../../mission/schema';
import { MY_TRACK_LAYOUT } from '../../../mission/myTrackLayout';
import { Panel } from '../../../ui/Panel';
import { Stat } from '../../../ui/Stat';
import { Bar } from '../../../ui/Bar';
import { Chip } from '../../../ui/Chip';
import { Toggle } from '../../../ui/Toggle';
import { Plot, type PlotPoint } from '../../../ui/Plot';
import '../../../ui/tokens.css';
import './myTrackPage.css';
import { useReportProvenance } from '../runContext';

/*
 * /mytrack — read-only consumer of useMissionData(), same hook MissionPage.tsx drives.
 * Visually styled after kart/viewer/index.html (the Python replay viewer): light theme,
 * small-caps panel titles, calm grey-green instrument-panel feel. Not a fork of the sim --
 * this page never calls into labAlgorithms.ts/LabSkidpad.ts directly, only the hook.
 */

const CORRIDOR_M = MY_TRACK_LAYOUT.corridor_m; // 3.0
const LENGTH_M = MY_TRACK_LAYOUT.length_m; // 50
const HP60C_FOV_DEG = 64.4;
const HP60C_FOV_RAD = (HP60C_FOV_DEG * Math.PI) / 180;

type Profile = 'shared' | 'built';
const PROFILE_DIMS: Record<Profile, { wheelbase_m: number; width_m: number }> = {
  shared: { wheelbase_m: 1.422, width_m: 1.0648 },
  built: { wheelbase_m: 1.1176, width_m: 1.0668 },
};

const fmt = (value: number | null | undefined, places = 2) => value == null || !Number.isFinite(value) ? 'n/a' : value.toFixed(places);

/** Rotate+translate a world point into the kart's body frame (x forward, y left). */
function toBodyFrame(point: { x_m: number; y_m: number }, origin: { x_m: number; y_m: number; yaw_rad: number }) {
  const dx = point.x_m - origin.x_m;
  const dy = point.y_m - origin.y_m;
  const cos = Math.cos(-origin.yaw_rad);
  const sin = Math.sin(-origin.yaw_rad);
  return { x: dx * cos - dy * sin, y: dx * sin + dy * cos };
}

function coneDotClass(cls: string) {
  const c = cls.toLowerCase();
  if (c.includes('blue')) return 'mt-cone-blue';
  if (c.includes('yellow')) return 'mt-cone-yellow';
  return 'mt-cone-orange';
}

function TrackOverview({ frame, history }: { frame: MissionFrame | null; history: MissionFrame[] }) {
  const cones = frame?.visualization?.course_cones ?? frame?.perception.ground_truth_cones ?? [];
  const vw = 420, vh = 120;
  const scaleX = (vw - 20) / LENGTH_M;
  const scaleY = (vh - 20) / (CORRIDOR_M * 3);
  const scale = Math.min(scaleX, scaleY);
  const px = (x: number) => 10 + x * scale;
  const py = (y: number) => vh / 2 - y * scale;
  const path = frame?.planning.selected_path ?? [];
  const kart = frame?.vehicle;
  return (
    <svg className="mt-svg" viewBox={`0 0 ${vw} ${vh}`} role="img" aria-label="Track overview, top-down">
      <rect x={px(0)} y={py(CORRIDOR_M / 2)} width={LENGTH_M * scale} height={CORRIDOR_M * scale} className="mt-corridor" />
      {history.length > 1 && <polyline className="mt-trail" points={history.map((f) => `${px(f.vehicle.x_m)},${py(f.vehicle.y_m)}`).join(' ')} />}
      {path.length > 1 && <polyline className="mt-path" points={path.map((p) => `${px(p.x_m)},${py(p.y_m)}`).join(' ')} />}
      {cones.map((cone) => <circle key={cone.id} cx={px(cone.x_m)} cy={py(cone.y_m)} r={2.2} className={coneDotClass(cone.class)} />)}
      {kart && <circle cx={px(kart.x_m)} cy={py(kart.y_m)} r={3.5} className="mt-kart-dot" />}
    </svg>
  );
}

function FollowView({ frame, profile }: { frame: MissionFrame | null; profile: Profile }) {
  const vw = 260, vh = 220;
  const scale = 10; // px per metre
  const cx = vw / 2, cy = vh - 30;
  const px = (b: { x: number; y: number }) => cx - b.y * scale;
  const py = (b: { x: number; y: number }) => cy - b.x * scale;
  const dims = PROFILE_DIMS[profile];
  if (!frame) return <div className="mt-empty" role="status">n/a — no frame</div>;
  const cones = (frame.visualization?.course_cones ?? frame.perception.cones).map((cone) => ({ ...cone, body: toBodyFrame(cone, frame.vehicle) })).filter((c) => c.body.x > -2 && c.body.x < 20);
  const path = frame.planning.selected_path.map((p) => toBodyFrame(p, frame.vehicle)).filter((p) => p.x > -1 && p.x < 20);
  const halfW = dims.width_m / 2;
  const nose = dims.wheelbase_m * 0.6;
  const footprint = [
    { x: nose, y: -halfW }, { x: nose, y: halfW }, { x: -dims.wheelbase_m * 0.4, y: halfW }, { x: -dims.wheelbase_m * 0.4, y: -halfW },
  ];
  const wedgeRange = 10;
  const half = HP60C_FOV_RAD / 2;
  const wedgePoints = [{ x: 0, y: 0 }];
  for (let i = 0; i <= 24; i++) { const theta = -half + (i * HP60C_FOV_RAD) / 24; wedgePoints.push({ x: Math.cos(theta) * wedgeRange, y: Math.sin(theta) * wedgeRange }); }
  return (
    <svg className="mt-svg" viewBox={`0 0 ${vw} ${vh}`} role="img" aria-label="Follow view, kart-centred">
      <polygon className="mt-wedge" points={wedgePoints.map((p) => `${px(p)},${py(p)}`).join(' ')} />
      {path.length > 1 && <polyline className="mt-path" points={path.map((p) => `${px(p)},${py(p)}`).join(' ')} />}
      {cones.map((c) => <circle key={c.id} cx={px(c.body)} cy={py(c.body)} r={2.6} className={coneDotClass(c.class)} />)}
      <polygon className="mt-footprint" points={footprint.map((p) => `${px(p)},${py(p)}`).join(' ')} />
    </svg>
  );
}

/** Synthetic HP60C RGB-D projection: cones ahead of the kart projected into a fake 640x480
 * camera frame using a simple pinhole-angle model, since no real per-frame camera detection
 * array is exposed by MissionFrame.perception (checked: perception.cones carries world-ish
 * range/bearing, not image coordinates). Placeholder, clearly labelled as such. */
function Hp60cFrame({ frame }: { frame: MissionFrame | null }) {
  const vw = 320, vh = 240; // scaled 640x480
  if (!frame) return <div className="mt-empty" role="status">n/a — no frame</div>;
  const half = HP60C_FOV_RAD / 2;
  const focal = (vw / 2) / Math.tan(half);
  const boxes = (frame.visualization?.course_cones ?? frame.perception.cones)
    .map((cone) => ({ ...cone, body: toBodyFrame(cone, frame.vehicle) }))
    .filter((c) => c.body.x > 0.3 && Math.abs(Math.atan2(c.body.y, c.body.x)) < half)
    .map((c) => {
      const u = vw / 2 - (c.body.y / c.body.x) * focal;
      const sizePx = Math.max(4, Math.min(40, 10 / c.body.x * focal / 10));
      const v = vh * 0.62;
      return { ...c, u, v, sizePx };
    });
  return (
    <svg className="mt-svg mt-cam" viewBox={`0 0 ${vw} ${vh}`} role="img" aria-label="HP60C RGB-D camera frame, synthetic detections">
      <rect x={0} y={0} width={vw} height={vh} className="mt-cam-bg" />
      <line x1={0} y1={vh * 0.62} x2={vw} y2={vh * 0.62} className="mt-cam-horizon" />
      {boxes.map((b) => (
        <g key={b.id}>
          <rect x={b.u - b.sizePx / 2} y={b.v - b.sizePx} width={b.sizePx} height={b.sizePx} className="mt-cam-box" />
          <text x={b.u} y={b.v + 11} className="mt-cam-label">{b.body.x.toFixed(1)}m</text>
        </g>
      ))}
    </svg>
  );
}

function clearanceM(frame: MissionFrame | null): number | null {
  if (!frame) return null;
  const cones = frame.perception.cones;
  if (!cones.length) return null;
  let min = Infinity;
  for (const cone of cones) { const d = Math.hypot(cone.x_m - frame.vehicle.x_m, cone.y_m - frame.vehicle.y_m); if (d < min) min = d; }
  return Number.isFinite(min) ? min : null;
}

export function MyTrackPage() {
  const data = useMissionData();
  const [profile, setProfile] = useState<Profile>('shared');
  const [speedMult, setSpeedMult] = useState(1);
  const [scrubbing, setScrubbing] = useState(false);
  const configured = useRef(false);

  useEffect(() => {
    if (!configured.current && data.source === 'LAB' && data.labScenario !== 'MY_TRACK') {
      configured.current = true;
      data.configureLab('MY_TRACK', data.labSettings);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  useReportProvenance(data.current ? 'SYNTHETIC' : null, `client-side JS sandbox (LabSkidpad) · MY_TRACK · ${CORRIDOR_M}m corridor / ${LENGTH_M}m`);

  // Local history-scrub playback: advances data.time through the already-recorded frame
  // buffer at `speedMult`x while the live sandbox is paused. Separate from data.runLab(),
  // which still drives the real-time sandbox tick -- LabSkidpad has no timeScale knob to
  // hook into directly, so "speed" here governs scrub-through-history, not sim tick rate.
  const lastTick = useRef(0);
  useEffect(() => {
    if (!scrubbing) return;
    let raf = 0;
    const tick = (now: number) => {
      if (!lastTick.current) lastTick.current = now;
      const dt = (now - lastTick.current) / 1000;
      lastTick.current = now;
      const last = data.frames[data.frames.length - 1]?.timestamp_s ?? 0;
      data.setTime((current) => {
        const next = current + dt * speedMult;
        if (next >= last) { setScrubbing(false); return last; }
        return next;
      });
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => { cancelAnimationFrame(raf); lastTick.current = 0; };
  }, [scrubbing, speedMult, data]);

  const frame = data.current;
  const history = data.frames;
  const minTime = history[0]?.timestamp_s ?? 0;
  const maxTime = history[history.length - 1]?.timestamp_s ?? 0;

  const series = useMemo(() => {
    const speed: PlotPoint[] = [], steer: PlotPoint[] = [], brake: PlotPoint[] = [], deviation: PlotPoint[] = [];
    for (const f of history) {
      speed.push({ t: f.timestamp_s, v: f.vehicle.speed_mps });
      steer.push({ t: f.timestamp_s, v: f.vehicle.requested_steering_rad ?? null });
      brake.push({ t: f.timestamp_s, v: f.vehicle.brake_command ?? null });
      deviation.push({ t: f.timestamp_s, v: f.lateralDeviationM ?? null });
    }
    return { speed, steer, brake, deviation };
  }, [history]);

  const deviationStats = useMemo(() => {
    const values = history.map((f) => f.lateralDeviationM).filter((v): v is number => v != null && Number.isFinite(v));
    if (!values.length) return { max: null, rms: null };
    const max = Math.max(...values.map((v) => Math.abs(v)));
    const rms = Math.sqrt(values.reduce((sum, v) => sum + v * v, 0) / values.length);
    return { max, rms };
  }, [history]);

  const missionState = frame?.planning.skidpad?.state ?? frame?.safety.state ?? 'NO DEBUG';
  const gates = frame?.planning.gates.length ?? 0;
  const mapCones = frame?.perception.ground_truth_cones?.length ?? frame?.perception.cones.length ?? 0;

  return (
    <article className="lab-page mt-page ui-root" data-page="mytrack" aria-labelledby="mt-title">
      <div className="mt-topbar">
        <div className="mt-topbar-titles">
          <p className="lab-eyebrow">MISSION &amp; SIM</p>
          <h1 id="mt-title">MY_TRACK</h1>
          <small className="mt-sub">{CORRIDOR_M}m corridor · {LENGTH_M}m course · sandbox (LabSkidpad.ts), display-only</small>
        </div>
        <Toggle aria-label="Kart dimension profile" value={profile} onChange={setProfile}
          options={[{ value: 'shared', label: 'Shared dims' }, { value: 'built', label: 'Built kart' }]} />
        <button className="mt-btn" onClick={() => { setScrubbing(false); data.mode === 'LIVE' ? data.setMode('PAUSE') : data.runLab(); }}>
          {data.mode === 'LIVE' ? '⏸ Pause' : '▶ Play'}
        </button>
        <label className="mt-select-label">×<select aria-label="Speed multiplier" value={speedMult} onChange={(e) => setSpeedMult(Number(e.target.value))}>
          <option value={1}>1</option><option value={2}>2</option><option value={5}>5</option><option value={10}>10</option>
        </select></label>
        <input className="mt-slider" type="range" aria-label="History scrub" min={minTime} max={Math.max(minTime + 0.001, maxTime)} step="0.001"
          value={Math.min(maxTime, Math.max(minTime, data.time))} disabled={history.length < 2}
          onChange={(e) => { setScrubbing(false); if (data.mode === 'LIVE') data.setMode('PAUSE'); data.setTime(Number(e.target.value)); }} />
        <button className="mt-btn" disabled={history.length < 2 || data.mode === 'LIVE'} onClick={() => setScrubbing((s) => !s)}>{scrubbing ? '⏸' : '▶'} scrub</button>
      </div>

      <div className="mt-grid">
        <Panel title="Track overview" className="mt-span-2"><TrackOverview frame={frame} history={history} /></Panel>
        <Panel title="Follow view" meta={`${profile} · wheelbase ${PROFILE_DIMS[profile].wheelbase_m}m`}><FollowView frame={frame} profile={profile} /></Panel>
        <Panel title="HP60C RGB-D frame" meta="synthetic projection"><Hp60cFrame frame={frame} /></Panel>

        <Panel title="Vehicle and controller" className="mt-span-2">
          <div className="mt-stats">
            <Stat label="Speed" value={fmt(frame?.vehicle.speed_mps)} unit="m/s" />
            <Stat label="Steer req" value={fmt(frame?.vehicle.requested_steering_rad, 3)} unit="rad" />
            <Stat label="Brake cmd" value={fmt(frame?.vehicle.brake_command)} />
            <Stat label="Clearance" value={fmt(clearanceM(frame))} unit="m" />
            <Stat label="Distance" value={fmt(frame?.vehicle.x_m)} unit="m" />
            <Stat label="Gates" value={gates} />
            <Stat label="Map cones" value={mapCones} />
          </div>
          <div className="mt-chips">
            <Chip tone={missionState === 'STOP' ? 'crit' : missionState === 'NO DEBUG' ? 'idle' : 'good'}>MISSION {missionState}</Chip>
            <Chip tone="idle">VEHICLE AUTHORITY OFF · SIM-ONLY OUTPUT</Chip>
          </div>
          <div className="mt-bars">
            <Bar label="Speed" value={frame?.vehicle.speed_mps} min={0} max={3} unit=" m/s" />
            <Bar label="Steer" value={frame?.vehicle.requested_steering_rad} min={-0.6} max={0.6} unit=" rad" />
            <Bar label="Brake" value={frame?.vehicle.brake_command} min={0} max={1} />
          </div>
        </Panel>

        <Panel title="Deviation" meta={frame?.lateralDeviationM != null ? undefined : 'n/a — not yet populated'}>
          <div className="mt-stats">
            <Stat label="Live" value={fmt(frame?.lateralDeviationM)} unit="m" />
            <Stat label="Max |dev|" value={fmt(deviationStats.max)} unit="m" />
            <Stat label="RMS" value={fmt(deviationStats.rms)} unit="m" />
          </div>
          <Plot label="Lateral deviation" unit="m" series={series.deviation} domain={[-1.5, 1.5]} />
        </Panel>

        <Panel title="Speed / steering / brake" className="mt-span-2">
          <div className="mt-plots">
            <Plot label="Speed" unit="m/s" series={series.speed} />
            <Plot label="Steering" unit="rad" series={series.steer} domain={[-0.6, 0.6]} />
            <Plot label="Brake" series={series.brake} domain={[0, 1]} />
          </div>
        </Panel>
      </div>
    </article>
  );
}
