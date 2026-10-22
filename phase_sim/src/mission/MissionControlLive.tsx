import { useState } from 'react';
import { MissionScene, type Layer } from './MissionScene';
import type { MissionFrame } from './schema';
import { useMissionDataLive } from './useMissionDataLive';
import { RosLivePanel } from './RosLivePanel';
import './mission.css';
import './opsLayout.css';

/**
 * Live-only operational Mission Control entry point (route: `/ops`).
 *
 * Unlike `MissionControl.tsx` (the development/simulation tool with a
 * LAB/SIMULATION/REPLAY/REAL_KART switcher), this component is hard-locked
 * to the real ROS2 gateway:
 *  - it calls `useMissionDataLive()`, not `useMissionData()` -- a separate,
 *    minimal hook with no `setSource`, no lab engine, no demo engine, and
 *    no replay/file loader;
 *  - it renders no `<select>` / dropdown / button that can change the data
 *    source, mode to REPLAY, or load a file;
 *  - it never imports `./LabSkidpad`, `../engine/engine`, `./demo`, or
 *    `./labScenarios` -- directly or transitively -- so the production
 *    build of this entry point has no code path, at build time, that can
 *    render simulated/lab/demo data as if it were live.
 *
 * On gateway disconnect `useMissionDataLive()` reports `current: null` /
 * a DISCONNECTED-family status instead of substituting any fabricated
 * frame; this component renders that as an explicit disconnected/stale
 * banner, never as a silent fallback to a fake frame.
 */

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

export function MissionControlLive() {
  const data = useMissionDataLive();
  const [layers] = useState(INITIAL_LAYERS);
  const [camera] = useState('algorithm');
  const frame = data.current;
  const obstacle = frame?.safety.obstacle;
  const connected = data.status === 'CONNECTED' || data.status === 'CURRENT ROS FRAME' || data.status === 'GATEWAY CONNECTED · NO CURRENT ROS FRAME';
  const disconnected = !connected;

  return <div className="mission-shell">
    <header className="mc-top">
      <div className="mc-brand"><span>AK</span><div><strong>MISSION CONTROL · LIVE</strong><small>REAL KART ONLY · READ ONLY</small></div></div>
      <span className="mc-top-source" aria-label="Operating mode">REAL_KART</span>
      <div className="mc-clock">{fmt(frame?.timestamp_s, 2)} <small>s</small></div>
    </header>
    <div className="mc-status">
      <span>REAL_KART</span>
      <span>{data.mode} · {data.status}</span>
      <span className={disconnected ? 'mc-alert' : ''}>{disconnected ? 'DISCONNECTED FROM GATEWAY · NO LIVE FRAME DISPLAYED' : 'GATEWAY CONNECTED'}</span>
      <span>MISSION {frame?.planning.skidpad?.state ?? 'NO DEBUG'}</span>
      <span>SPEED {fmt(frame?.vehicle.speed_mps)} m/s</span>
      <span className={obstacle?.brake_trigger ? 'mc-alert' : ''}>SAFETY {obstacle?.brake_trigger ? 'BRAKE REQUIRED' : frame?.safety.state ?? 'NO DATA'}</span>
      <span>RUN {String(frame?.run_id ?? '—')}</span>
      <span>READ ONLY · VEHICLE AUTHORITY OFF</span>
    </div>
    <DiagnosticStrip frame={frame} />
    <main className="mc-main mc-ops"><div className="mc-world">
      {disconnected
        ? <div className="mc-waiting-geometry" role="status"><b>NO LIVE CONNECTION</b><small>Not connected to the real-kart gateway, or no current frame has arrived. No simulated or cached geometry is substituted here.</small></div>
        : <MissionScene frame={frame} layers={layers} cameraMode={camera} selection={{ kind: 'kart', id: 'vehicle' }} onSelect={() => {}} animationStage={0} />}
      {!disconnected && !(frame?.perception.cones.length) && <div className="mc-waiting-geometry" role="status"><b>WAITING FOR QUALIFIED GEOMETRY</b><small>The world view shows only odom-frame data from verified camera calibration, live odometry and a current map. No simulated geometry is substituted.</small></div>}
      <div className="mc-world-top"><div className="mc-world-title"><small>LIVE KART WORLD</small><h1>{String(frame?.scenario ?? data.manifest.scenario ?? 'Waiting for live track data')}</h1></div></div>
      {obstacle && <div className={`mc-floating mc-safety ${obstacle.brake_trigger ? 'braking' : ''}`}><small>OBSTACLE {obstacle.id}</small><strong>{obstacle.brake_trigger ? 'BRAKE OVERRIDE' : 'SAFE · MONITORING'}</strong><p>Speed {fmt(frame?.vehicle.speed_mps)} m/s · obstacle {fmt(obstacle.distance_m)} m<br />TTC {fmt(obstacle.ttc_s)} s · reaction {fmt(obstacle.reaction_distance_m)} m<br />Braking {fmt(obstacle.braking_distance_m)} m · safety {fmt(obstacle.safety_margin_m)} m<br />Required {fmt(obstacle.stopping_distance_m)} m · margin {fmt(obstacle.margin_m)} m</p></div>}
    </div>
    <RosLivePanel source="REAL_KART" frame={frame} sendDemoDrive={data.sendDemoDrive}
      demoDriveRejected={data.demoDriveRejected} wsConnected={data.wsConnected} />
    </main>
    <footer className="mc-bottom"><div className="mc-transport">
      <button onClick={data.resumeLive} className={data.mode === 'LIVE' ? 'selected' : ''}>LIVE</button>
      <button onClick={data.pause} className={data.mode === 'PAUSE' ? 'selected' : ''} disabled={!data.frames.length}>PAUSE</button>
      <span>{fmt(data.time, 3)} s</span>
    </div></footer>
  </div>;
}
