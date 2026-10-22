import { useMemo, useState } from 'react';
import { useDash } from './data';
import { deriveHealth, toneOf, safetyView } from './derive';
import { age, isNum, NO_DATA, num, pt, txt } from './format';
import { BirdsEye, buildScene, DEFAULT_LAYERS, LAYER_META, type LayerKey } from './BirdsEye';
import { Card, Chip, FrameView, KV, Unavailable } from './ui';
import type { Channel } from './types';

const AUTH = <Chip tone="info" title="vehicle_authority is read-only OFF in this app">VEHICLE AUTHORITY: OFF · DIAGNOSTIC_ONLY</Chip>;

function useScene() {
  const d = useDash();
  const h = deriveHealth(d);
  // No frame (fault / NO_FRAME) snapshots carry empty geometry; offline keeps the last snapshot but the view is frozen and labelled.
  const snap = d.perc.snapshot;
  const scene = useMemo(() => buildScene(snap, h.geo.mode, h.geo.why), [snap, h.geo.mode, h.geo.why]);
  const frozen = !h.animating;
  const status = h.cam.state === 'STALE' ? `STALE ${age(h.cam.ageMs)}` : h.cam.state;
  return { d, h, scene, frozen, status };
}

export function Overview() {
  const { d, h, scene, frozen, status } = useScene();
  const f = d.ros.frame, rosFresh = d.ros.at != null && d.tick - d.ros.at < 1000;
  const s = d.perc.snapshot;
  return <div className="grid">
    <Card title="Provenance strip" wide>
      <div className="chips">{h.chips.map(c => <div key={c.key} className="hopcard"><span className="hoplabel">{c.label}</span><Chip tone={c.tone} title={c.detail}>{c.state}</Chip><small>{c.detail}</small></div>)}</div>
    </Card>
    <Card title="Bird's-eye (concise)" wide={false}>
      <BirdsEye compact scene={scene} layers={DEFAULT_LAYERS} frozen={frozen} frameCounter={d.perc.frames} statusText={status} dataFps={d.perc.fps} />
      {h.geo.mode === 'NONE' && <p className="unavailable"><strong>NO QUALIFIED LIVE GEOMETRY.</strong> No kart pose or track is drawn as measured. {h.geo.why}</p>}
    </Card>
    <Card title="RGB camera">
      <FrameView src={d.perc.rgb} alt="Current RGB frame" detections={s?.detections} overlay />
      <KV rows={[['Frame', txt(d.perc.displayed?.frame_id)], ['Camera frame age', age(h.cam.ageMs)], ['Display rate', num(d.perc.fps, 1, 'Hz')]]} />
    </Card>
    <Card title="Vehicle state" right={AUTH}>
      <KV rows={[
        ['Speed', rosFresh && isNum(f?.vehicle?.speed_mps) ? num(f?.vehicle?.speed_mps, 2, 'm/s') : NO_DATA],
        ['Pose (x, y)', rosFresh && isNum(f?.vehicle?.x_m) ? `${num(f?.vehicle?.x_m, 2)}, ${num(f?.vehicle?.y_m, 2)} m` : NO_DATA],
        ['Mission state', txt(d.perc.snapshot?.mission_state?.state)],
        ['Selected safety state', <Chip key="s" tone={h.safety.tone}>{h.safety.label}</Chip>],
        ['Perception status', txt(s?.status)],
        ['Last perception update', d.perc.okAt != null ? `${age(d.tick - d.perc.okAt)} ago` : NO_DATA],
        ['Last gateway update', d.gw.at != null && d.gw.link === 'ok' ? `${age(d.tick - d.gw.at)} ago` : NO_DATA],
        ['Last ROS frame', d.ros.at != null ? `${age(d.tick - d.ros.at)} ago` : NO_DATA],
      ]} />
      {!rosFresh && <Unavailable what="Kart pose and speed" reason={`No current ROS frame on /mission/ws (WebSocket ${d.ros.transport}; ROS data ${h.ros.state}: ${h.ros.detail}).`} />}
    </Card>
  </div>;
}

export function CameraPage() {
  const d = useDash(); const h = deriveHealth(d);
  const s = d.perc.displayed ?? d.perc.snapshot; const dets = s?.detections ?? [];
  const sdk = d.pipe.data?.hops?.find(x => x.name === 'sdk_capture');
  const drops = sdk?.drops as { sdk_mailbox_overwrites?: number; processing_failures?: number } | undefined;
  const stale = h.cam.state === 'STALE';
  return <div className="grid">
    <Card title="RGB" right={<Chip tone={toneOf(h.cam.state)}>{h.cam.state}</Chip>}>
      <FrameView src={d.perc.rgb} alt="RGB frame from the current snapshot" detections={dets} overlay />
      {stale && <p className="unavailable">STALE: displayed image is not current ({age(h.cam.ageMs)} old).</p>}
    </Card>
    <Card title="Depth (RAW16 visualisation)">
      <FrameView src={d.perc.depth} alt="Depth frame from the current snapshot" />
      <KV rows={[['Raw unit', txt(s?.depth_stats?.raw_unit)], ['Valid pixels', num(s?.depth_stats?.valid_percent, 1, '%')], ['Median raw', num(s?.depth_stats?.median_raw, 0)],
        ['Depth scale', `${txt(s?.depth_scale_m_per_raw_candidate)} m/raw · ${s?.depth_scale_qualified ? 'QUALIFIED' : 'UNQUALIFIED'}`], ['Scale provenance', txt(s?.depth_scale_provenance)]]} />
    </Card>
    <Card title="Frame timing" wide>
      <KV rows={[
        ['RGB / depth frame ID', txt(s?.frame_id)], ['Camera', txt(s?.camera_id)], ['RGB sequence', txt(s?.camera_sequence)], ['Depth sequence', txt(s?.depth_sequence)],
        ['RGB-depth pair', `${txt(s?.rgb_depth_pair)} · alignment ${txt(s?.rgb_depth_alignment)} · Δ ${num(s?.rgb_depth_timestamp_delta_ms, 1, 'ms')}`],
        ['Display rate', num(d.perc.fps, 1, 'Hz')], ['Processed rate', num(s?.stream_metrics?.processed_fps, 1, 'Hz')], ['Camera callback rate', num(s?.stream_metrics?.camera_callback_fps, 1, 'Hz')],
        ['Frame age (now)', age(h.cam.ageMs)], ['Capture → display (est.)', num(d.perc.displayLatencyMs, 0, 'ms')],
        ['Capture → publish', num(s?.latencies_ms?.capture_to_publish, 1, 'ms')], ['Inference', num(s?.latencies_ms?.inference, 1, 'ms')],
        ['Dropped: SDK mailbox overwrites', txt(drops?.sdk_mailbox_overwrites ?? s?.stream_metrics?.mailbox_overwrites)],
        ['Dropped: processing failures', txt(drops?.processing_failures)], ['Skipped by this browser', String(d.perc.clientSkipped)],
        ['Frames displayed', String(d.perc.frames)],
      ]} />
      {d.perc.notice && <p className="unavailable">{d.perc.notice}</p>}
    </Card>
    <Card title={`Detections (${dets.length})`} wide>
      {dets.length ? <table><thead><tr><th>#</th><th>Class</th><th>Confidence</th><th>Box x1,y1,x2,y2 (px)</th></tr></thead>
        <tbody>{dets.map(x => <tr key={x.id}><td>#{x.id}</td><td>{x.label}</td><td>{num(100 * x.confidence, 0, '%')}</td><td className="mono">{x.box.join(', ')}</td></tr>)}</tbody></table>
        : <Unavailable what="Detections" reason={h.cam.fresh ? 'The detector returned no boxes for this frame.' : `No current frame (${h.cam.state}).`} />}
      <KV rows={[['Detector', `${txt(s?.model?.name)} ${txt(s?.model?.version)}`], ['Qualification', txt(s?.model?.qualification)]]} />
    </Card>
  </div>;
}

export function PerceptionPage() {
  const { d, h, scene, frozen, status } = useScene();
  const [layers, setLayers] = useState(DEFAULT_LAYERS);
  const s = d.perc.snapshot; const g = s?.geometry;
  const count: Record<LayerKey, number> = {
    cones: scene.mode === 'NONE' ? 0 : scene.cones.length, candidates: scene.mode === 'NONE' ? scene.cones.length : 0,
    delaunay: scene.edges.filter(e => e.layer === 'delaunay').length, gabriel: scene.edges.filter(e => e.layer === 'gabriel').length,
    gabrielRejected: scene.edges.filter(e => e.layer === 'gabrielRejected').length, boundaries: scene.lines.filter(l => l.layer === 'boundaries').length,
    gates: scene.edges.filter(e => e.layer === 'gate').length, centerline: scene.lines.filter(l => l.layer === 'centerline').length,
    path: scene.lines.filter(l => l.layer === 'path').length, target: scene.target ? 1 : 0,
  };
  const dc = s?.detector_counts;
  return <div className="grid">
    <Card title="Bird's-eye · all algorithm layers" wide right={<Chip tone={toneOf(h.cam.state)}>{h.cam.state}</Chip>}>
      <div className="bev-layout">
        <BirdsEye scene={scene} layers={layers} frozen={frozen} frameCounter={d.perc.frames} statusText={status} dataFps={d.perc.fps} />
        <fieldset className="layers"><legend>Layers</legend>
          {(Object.keys(LAYER_META) as LayerKey[]).map(k => <label key={k}>
            <input type="checkbox" checked={layers[k]} onChange={e => setLayers(l => ({ ...l, [k]: e.target.checked }))} />
            <i style={{ background: LAYER_META[k].color }} aria-hidden /> {LAYER_META[k].label} <b>{count[k]}</b></label>)}
        </fieldset>
      </div>
      {scene.mode === 'NONE' && <p className="unavailable" role="status"><strong>NO QUALIFIED LIVE GEOMETRY.</strong> {h.geo.why || 'No metric geometry'}. Detections exist in the image, but metric cones, Delaunay, Gabriel, gates, centerline and path are withheld until depth scale and camera mount are verified.</p>}
      {scene.mode === 'SIMULATED' && <p className="unavailable"><strong>SIMULATED GEOMETRY.</strong> {h.geo.why}</p>}
    </Card>
    <Card title="Qualification and rejection reasons">
      <KV rows={[['Perception status', txt(s?.status)], ['Reasons', (s?.reasons ?? []).join(', ') || NO_DATA], ['Calibration', txt(s?.calibration_status)],
        ['Depth scale', s?.depth_scale_qualified ? 'QUALIFIED' : 'UNQUALIFIED'], ['Geometry status', txt(g?.status)], ['Topology reasons', (g?.topology_reasons ?? []).join(', ') || NO_DATA],
        ['Boundaries', `${txt(g?.boundaries?.status)} · ${txt(g?.boundaries?.reason)}`], ['Centerline', txt(g?.centerline_status)], ['Curvature', txt(g?.curvature_status)],
        ['Tracking', txt(s?.tracking_status)], ['Camera motion', s?.camera_motion ? `${s.camera_motion.status} (unqualified)` : NO_DATA]]} />
      {(s?.rejected_candidates?.length ?? 0) > 0 ? <table><thead><tr><th>Rejected #</th><th>Reason</th></tr></thead><tbody>
        {s!.rejected_candidates!.map(r => <tr key={r.id}><td>#{r.id}</td><td>{r.localization_status}</td></tr>)}</tbody></table> : <p className="muted">No rejected candidates reported this frame.</p>}
    </Card>
    <Card title="Detector → geometry funnel">
      <KV rows={[['Raw output rows', txt(dc?.raw_output_rows)], ['Above confidence', txt(dc?.above_confidence)], ['After NMS', txt(dc?.after_nms)],
        ['Application detections', txt(dc?.application_detections ?? s?.detections?.length)], ['Depth-localized candidates', txt(dc?.depth_localized_candidates)],
        ['Validated metric cones', txt(dc?.validated_metric ?? s?.observations?.length)], ['Control target', pt(s?.control_candidate?.target_point_base_m)]]} />
    </Card>
  </div>;
}

const ChannelRow = ({ name, ch }: { name: string; ch?: Channel }) => <tr><td>{name}</td>
  <td><Chip tone={ch?.state === 'CURRENT' ? 'ok' : ch?.state === 'STALE' ? 'alert' : 'fault'}>{ch?.state ?? 'MISSING'}</Chip></td><td>{age(ch?.age_ms)}</td></tr>;

export function LocalizationPage() {
  const d = useDash(); const h = deriveHealth(d);
  const f = d.ros.frame; const fresh = d.ros.at != null && d.tick - d.ros.at < 1000;
  const v = f?.vehicle; const s = d.perc.snapshot;
  const state = d.gw.link === 'offline' ? 'GATEWAY_OFFLINE' : fresh ? 'LIVE' : f ? 'STALE' : h.ros.state === 'ROS_OFFLINE' ? 'ROS_OFFLINE' : 'NO_FRAME';
  return <div className="grid">
    <Card title="Odometry pose" right={<Chip tone={toneOf(state)}>{state}</Chip>}>
      {fresh ? <KV rows={[['x', num(v?.x_m, 2, 'm')], ['y', num(v?.y_m, 2, 'm')], ['Yaw', num(v?.yaw_rad, 3, 'rad')], ['Speed', num(v?.speed_mps, 2, 'm/s')], ['Cross-track error', num(f?.metrics?.cross_track_error_m, 2, 'm')], ['Heading error', num(f?.metrics?.heading_error_rad, 3, 'rad')]]} />
        : <><Unavailable what="Odometry pose" reason={f ? `STALE: last /odom frame was ${age(d.ros.at != null ? d.tick - d.ros.at : null)} ago; the old value is not shown as current.` : `${h.ros.detail}.`} />
          {f && <KV rows={[['Last known x (STALE)', num(v?.x_m, 2, 'm'), 'alert'], ['Last known y (STALE)', num(v?.y_m, 2, 'm'), 'alert']]} />}</>}
    </Card>
    <Card title="Sources and quality">
      <table><thead><tr><th>Channel</th><th>State</th><th>Age</th></tr></thead><tbody>
        <ChannelRow name="/odom (gateway channel)" ch={d.gw.status?.odometry} /><ChannelRow name="vcu" ch={d.gw.status?.vcu} /></tbody></table>
      <KV rows={[['Rear encoder', 'NOT REPORTED by gateway (no encoder field in contract)'], ['IMU', 'NOT REPORTED by gateway (no IMU field in contract)'],
        ['Covariance / uncertainty', 'NOT REPORTED (no covariance in ROS frame)'], ['Pose latency', NO_DATA], ['ROS WebSocket', d.ros.transport]]} />
    </Card>
    <Card title="Frames and visual odometry">
      <KV rows={[['Perception camera frame', txt(s?.camera_frame)], ['Perception base frame', txt(s?.base_frame)], ['Perception pose status', txt(s?.pose_status)],
        ['TF chain', 'NOT REPORTED (no TF endpoint in gateway contract)'], ['Camera motion (visual)', s?.camera_motion ? `${s.camera_motion.status} · ${txt(s.camera_motion.inlier_count)}/${txt(s.camera_motion.matched_points)} inliers · RMS ${num(s.camera_motion.reprojection_rms_px, 2, 'px')} · UNQUALIFIED` : NO_DATA]]} />
    </Card>
  </div>;
}

export function MissionPage() {
  const d = useDash(); const s = d.perc.snapshot; const h = deriveHealth(d);
  const plan = d.gw.status?.stages?.planning; const pp = plan?.state === 'CURRENT' ? plan.payload : null;
  const ms = pp?.mission_state as Record<string, unknown> | undefined;
  const path = s?.planning?.local_path ?? [];
  const rosPath = d.ros.frame?.planning?.selected_path ?? [];
  const pathOk = h.cam.fresh && s?.planning?.path_valid === true;
  return <div className="grid">
    <Card title="Mission state" right={<Chip tone="info">DISPLAY ONLY</Chip>}>
      <KV rows={[['Perception mission state', h.cam.fresh ? txt(s?.mission_state?.state) : NO_DATA], ['ROS planner state', ms ? txt(ms.state) : `${plan?.state ?? 'MISSING'} — NO DATA`],
        ['Right / left laps', ms ? `R${txt(ms.right_laps)} L${txt(ms.left_laps)}` : NO_DATA], ['Selected mission (SKIDPAD / BRAKING)', 'NOT REPORTED by backend (no selection field)'],
        ['Mission requests', 'DISABLED · read-only dashboard, no request endpoint']]} />
    </Card>
    <Card title="Planned path">
      {pathOk ? <KV rows={[['Path valid', 'YES (perception planner)'], ['Points', String(path.length)], ['Frame', txt(s?.planning?.path_frame)], ['Speed profile', txt(s?.planning?.speed_profile_provenance)], ['First → last', `${pt(path[0])} → ${pt(path[path.length - 1])}`]]} />
        : <Unavailable what="Perception planned path" reason={h.cam.fresh ? 'No valid path in the current snapshot (geometry unqualified or planner invalid).' : `No current frame (${h.cam.state}).`} />}
      {rosPath.length > 0 && d.ros.at != null && d.tick - d.ros.at < 1000 ? <p>ROS selected path: {rosPath.length} points.</p> : <p className="muted">No current ROS selected path.</p>}
    </Card>
    <Card title="Control candidate (never published)">
      <KV rows={[['Valid', h.cam.fresh ? txt(s?.control_candidate?.valid) : NO_DATA], ['Target speed', h.cam.fresh && s?.control_candidate?.valid ? num(s.control_candidate.target_speed_mps, 2, 'm/s') : NO_DATA],
        ['Steering angle', h.cam.fresh && s?.control_candidate?.valid ? num(s.control_candidate.steering_angle_rad, 3, 'rad') : NO_DATA], ['Target point', pt(s?.control_candidate?.target_point_base_m)],
        ['Published to VCU', txt(s?.control_candidate?.published_to_vcu ?? false)]]} />
    </Card>
  </div>;
}

export function VehiclePage() {
  const d = useDash(); const h = deriveHealth(d);
  const fresh = d.ros.at != null && d.tick - d.ros.at < 1000; const v = d.ros.frame?.vehicle; const sf = d.ros.frame?.safety;
  const vcu = d.gw.status?.vcu; const faults = vcu?.state === 'CURRENT' && Array.isArray(vcu.payload?.faults) ? vcu.payload!.faults as string[] : null;
  const tri = (x: unknown) => (typeof x === 'boolean' ? (x ? 'TRUE' : 'FALSE') : NO_DATA);
  return <div className="grid">
    <Card title="VCU link" right={<Chip tone={vcu?.state === 'CURRENT' ? 'ok' : 'fault'}>{d.gw.link === 'offline' ? 'GATEWAY_OFFLINE' : vcu?.state ?? 'MISSING'}</Chip>}>
      <KV rows={[['Controller link', d.gw.link === 'offline' ? 'UNKNOWN · GATEWAY_OFFLINE' : `${vcu?.state ?? 'MISSING'} · ${age(vcu?.age_ms)}`], ['Faults', faults ? faults.join(', ') || 'NONE REPORTED' : NO_DATA],
        ['Watchdog (vcu_watchdog)', fresh ? tri(sf?.vcu_watchdog) : NO_DATA], ['Command fresh', fresh ? tri(sf?.command_fresh) : NO_DATA], ['Motor enable', fresh ? tri(sf?.motor_enable) : NO_DATA]]} />
    </Card>
    <Card title="Feedback (measured, not commands)">
      {fresh ? <KV rows={[['Speed (measured)', num(v?.speed_mps, 2, 'm/s')], ['Steering (measured)', num(v?.measured_steering_rad, 3, 'rad')], ['Brake feedback', num(v?.brake_feedback, 2)],
        ['Steering requested', num(v?.requested_steering_rad, 3, 'rad')], ['Speed requested', num(v?.target_speed_mps, 2, 'm/s')], ['Brake requested', num(v?.brake_command, 2)]]} />
        : <Unavailable what="Wheel speed, steering and brake feedback" reason={`No current ROS frame (${h.ros.state}). Missing feedback is never shown as 0 or healthy.`} />}
      <KV rows={[['Voltage / temperature', 'NOT INSTRUMENTED / NOT REPORTED by gateway']]} />
    </Card>
  </div>;
}

export function SafetyPage() {
  const d = useDash(); const sv = safetyView(d); const e = sv.envelope;
  const sf = d.ros.frame?.safety; const fresh = d.ros.at != null && d.tick - d.ros.at < 1000;
  const ch = d.gw.status?.safety;
  const tri = (x: unknown) => (!fresh ? NO_DATA : typeof x === 'boolean' ? (x ? 'TRUE' : 'FALSE') : NO_DATA);
  const sp = d.perc.snapshot?.perception;
  return <div className="grid">
    <Card title="Front safety envelope" wide right={AUTH}>
      <div className="big"><Chip tone={sv.tone}>{sv.label}</Chip><span>{sv.reason}</span></div>
      {sv.notes.map(n => <p key={n} className="unavailable" role="alert">{n}</p>)}
      <KV rows={[['/safety/envelope', `${sv.channel} · ${age(sv.ageMs)}`], ['Data source', txt(e?.data_source)], ['Valid', e ? txt(e.valid) : NO_DATA], ['Calibration', e ? (e.calibration_verified ? 'VERIFIED' : 'UNVERIFIED') : NO_DATA],
        ['Nearest clearance', num(e?.clear_distance_m, 2, 'm')], ['Braking trigger distance', num(e?.stopping_distance_m, 2, 'm')]]} />
    </Card>
    <Card title="Fault and hardware flags">
      <KV rows={[['Safety channel', `${ch?.state ?? (d.gw.link === 'offline' ? 'GATEWAY_OFFLINE' : 'MISSING')} · ${age(ch?.age_ms)}`], ['Safety state', fresh ? txt(sf?.state) : NO_DATA],
        ['E-stop input', 'NOT FITTED - MANUAL POWER CUT ONLY', 'warn'], ['Remote E-stop', tri(sf?.remote_estop)], ['LoRa heartbeat', tri(sf?.lora_heartbeat)], ['RPi heartbeat', tri(sf?.rpi_heartbeat)],
        ['Faults', fresh && Array.isArray(sf?.faults) ? (sf!.faults!.join(', ') || 'NONE REPORTED') : NO_DATA], ['Reset / lockout control', 'NONE · this UI cannot reset hardware']]} />
    </Card>
    <Card title="Perception-side safety">
      <KV rows={[['Safety state', txt(sp?.safety_state)], ['Reasons', (sp?.safety_reasons ?? []).join(', ') || NO_DATA], ['ready', txt(d.perc.snapshot?.ready)]]} />
    </Card>
  </div>;
}

export function RosPage() {
  const d = useDash(); const h = deriveHealth(d); const g = d.gw.status;
  return <div className="grid">
    <Card title="ROS gateway" right={<Chip tone={toneOf(h.ros.state)}>{h.ros.state}</Chip>}>
      <p>{h.ros.detail}</p>
      <KV rows={[['Gateway HTTP', d.gw.link === 'ok' ? 'REACHABLE' : d.gw.link === 'offline' ? `GATEWAY_OFFLINE (${d.gw.error})` : 'CONNECTING', d.gw.link === 'offline' ? 'fault' : undefined],
        ['ros_connected', g ? txt(g.ros_connected) : NO_DATA], ['Source / mode', g ? `${txt(g.source)} / ${txt(g.mode)}` : NO_DATA], ['Run / scenario', g ? `${txt(g.run_id)} / ${txt(g.scenario)}` : NO_DATA],
        ['WebSocket', d.ros.transport], ['Frames received (WS)', String(d.ros.frames)], ['ROS domain / QoS', 'NOT REPORTED by gateway']]} />
      {g ? <table><thead><tr><th>Topic channel</th><th>State</th><th>Age</th></tr></thead><tbody>
        <ChannelRow name="odometry" ch={g.odometry} /><ChannelRow name="vcu" ch={g.vcu} /><ChannelRow name="safety" ch={g.safety} /><ChannelRow name="safety_envelope" ch={g.safety_envelope} />
        <ChannelRow name="stage: perception" ch={g.stages?.perception} /><ChannelRow name="stage: mapping" ch={g.stages?.mapping} /><ChannelRow name="stage: planning" ch={g.stages?.planning} /></tbody></table>
        : <Unavailable what="ROS topic health" reason="ROS gateway is not reachable, so topic freshness cannot be determined." />}
    </Card>
    <Card title="Perception pipeline hops" right={<Chip tone={d.pipe.link === 'ok' ? toneOf(d.pipe.data?.overall ?? 'UNKNOWN') : 'idle'}>{d.pipe.link === 'ok' ? d.pipe.data?.overall ?? 'UNKNOWN' : d.pipe.link.toUpperCase()}</Chip>}>
      {d.pipe.link === 'unavailable' && <Unavailable what="Pipeline diagnostics" reason="pipeline diagnostics unavailable from this service version (HTTP 404)." />}
      {d.pipe.link === 'offline' && <Unavailable what="Pipeline diagnostics" reason={`perception service not reachable (${d.pipe.error}).`} />}
      {d.pipe.link === 'connecting' && <p>CONNECTING…</p>}
      {d.pipe.data && <>
        <KV rows={[['Service', `${txt(d.pipe.data.service?.name)} · ${txt(d.pipe.data.service?.mode)} · ${txt(d.pipe.data.service?.source)}`], ['PID / uptime', `${txt(d.pipe.data.service?.pid)} · ${num(d.pipe.data.service?.uptime_s, 0, 's')}`],
          ['Camera SDK owner', txt(d.pipe.data.service?.camera_owner)], ['Detector', txt(d.pipe.data.service?.detector)]]} />
        <table><thead><tr><th>Hop</th><th>State</th><th>Reason / error</th></tr></thead><tbody>
          {(d.pipe.data.hops ?? []).map(x => <tr key={x.name}><td>{x.name}</td><td><Chip tone={toneOf(x.state)}>{x.state}</Chip></td><td>{x.reason ?? x.last_error ?? '—'}</td></tr>)}</tbody></table></>}
    </Card>
  </div>;
}

export function EventsPage() {
  const d = useDash();
  return <div className="grid"><Card title={`State transitions observed (${d.events.length})`} wide>
    <p className="muted">Recorded by this browser session from live status changes. This is not a vehicle log or recording and is lost on reload; times are browser wall-clock.</p>
    {d.events.length ? <table><thead><tr><th>Time (UTC)</th><th>Hop</th><th>From</th><th>To</th><th>Detail</th></tr></thead><tbody>
      {d.events.map((e, i) => <tr key={i}><td className="mono">{e.wall}</td><td>{e.hop}</td><td>{e.from}</td><td><Chip tone={toneOf(e.to.split(' ')[0])}>{e.to}</Chip></td><td>{e.detail}</td></tr>)}</tbody></table>
      : <Unavailable what="Events" reason="No state transitions observed yet in this session." />}
  </Card></div>;
}

export function AboutPage() {
  const d = useDash(); const s = d.perc.snapshot; const sv = d.pipe.data?.service;
  return <div className="grid">
    <Card title="Dashboard"><KV rows={[['App', 'AutoKart Vehicle Dashboard (read-only real-vehicle monitor)'], ['Build mode', import.meta.env.MODE], ['Vehicle authority', 'OFF · no command path in this app'],
      ['Perception endpoint', '/api/snapshot · /api/image/<frame>/<rgb|depth> · /api/v1/diagnostics/pipeline'], ['ROS endpoint', '/mission/api/status · /mission/ws (receive-only)']]} /></Card>
    <Card title="Perception service"><KV rows={[['Service', sv ? `${txt(sv.name)} (${txt(sv.mode)})` : d.pipe.link === 'unavailable' ? 'pipeline route unavailable in this service version' : NO_DATA], ['Snapshot schema', txt(s?.schema_version)],
      ['Algorithm version', txt(s?.algorithm_version)], ['Configuration SHA-256', txt(s?.configuration_sha256)], ['Detector model', `${txt(s?.model?.name)} ${txt(s?.model?.version)}`],
      ['Model SHA-256', txt(sv?.model_sha256 ?? s?.model?.sha256)], ['Device serial', 'NOT REPORTED by service']]} /></Card>
    <Card title="Calibration"><KV rows={[['Calibration ID', txt(s?.calibration_id)], ['Mount status', txt(s?.calibration_status)], ['Depth scale', s?.depth_scale_qualified ? 'QUALIFIED' : 'UNQUALIFIED'],
      ['Scale provenance', txt(s?.depth_scale_provenance)], ['Intrinsics fx/fy/cx/cy', s?.intrinsics ? `${s.intrinsics.fx}/${s.intrinsics.fy}/${s.intrinsics.cx}/${s.intrinsics.cy}` : NO_DATA], ['Settings', 'View-only; none editable']]} /></Card>
  </div>;
}
