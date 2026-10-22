import { useMemo, useState } from 'react';
import type { MissionFrame } from '../../../mission/schema';
import { Link } from '../router';
import { useReportProvenance, useRun } from '../runContext';
import { RecordingPicker } from './RecordingPicker';

const fmt = (value: number | null | undefined, places = 2) => value == null || !Number.isFinite(value) ? '—' : value.toFixed(places);
const PAGE = 50;

export function summarizeRecording(frames: MissionFrame[]) {
  const sources = [...new Set(frames.map((frame) => frame.source))];
  const runs = [...new Set(frames.map((frame) => String(frame.run_id)))];
  let gaps = 0, reversals = 0;
  for (let i = 1; i < frames.length; i++) {
    const delta = frames[i].sequence - frames[i - 1].sequence;
    if (delta > 1) gaps += delta - 1;
    if (delta < 0) reversals += 1;
  }
  const dts = frames.slice(1).map((frame, i) => frame.timestamp_s - frames[i].timestamp_s).filter((dt) => dt > 0).sort((a, b) => a - b);
  const medianDt = dts.length ? dts[Math.floor(dts.length / 2)] : null;
  return { sources, runs, gaps, reversals, start: frames[0]?.timestamp_s ?? null, end: frames[frames.length - 1]?.timestamp_s ?? null, medianDt,
    withDiagnostics: frames.filter((frame) => frame.diagnostics).length };
}

export function DatasetPage() {
  const { recording, clearRecording } = useRun();
  const [page, setPage] = useState(0);
  const [selected, setSelected] = useState(0);
  const frames = recording?.frames ?? [];
  const summary = useMemo(() => summarizeRecording(frames), [frames]);
  useReportProvenance(recording ? 'REPLAY' : null, recording ? `recording ${recording.name} · ${frames.length} frames · recorded, not live` : '');
  const pages = Math.max(1, Math.ceil(frames.length / PAGE));
  const visible = frames.slice(page * PAGE, page * PAGE + PAGE);
  const frame = frames[selected] ?? null;
  return <article className="lab-page lab-doc" data-page="dataset" aria-labelledby="dataset-title">
    <p className="lab-eyebrow">DATA · RECORDING INSPECTOR</p>
    <h1 id="dataset-title">Dataset and camera inspector</h1>
    <section className="lab-card lab-unavailable-inline" aria-label="Camera availability">
      <p data-testid="camera-unavailable"><b>REAL CAMERA: NOT AVAILABLE IN THIS BUILD.</b> The Lab has no route to the camera or live perception service, and MissionFrame recordings carry cone lists and pose, not RGB / depth images. RGB/depth pairing, SDK metadata and frame previews are therefore not shown.</p>
    </section>
    <section className="lab-card" aria-labelledby="dataset-open">
      <h2 id="dataset-open">Recording</h2>
      {recording ? <p><b>{recording.name}</b> · opened {recording.loadedAt} <button className="lab-link-button" onClick={() => { clearRecording(); setPage(0); setSelected(0); }}>Close recording</button></p>
        : <p className="lab-warn-text" role="status">No recording loaded. Recordings are decoded by the existing replay loader (<code>src/mission/replay.ts</code>) and held in this tab only.</p>}
      <RecordingPicker />
    </section>
    {recording && <>
      <section className="lab-card" aria-labelledby="dataset-summary">
        <h2 id="dataset-summary">Summary</h2>
        <dl className="lab-facts lab-facts-wide">
          <div><dt>FRAMES</dt><dd>{frames.length}</dd></div>
          <div><dt>RECORDED SOURCE(S)</dt><dd>{summary.sources.join(', ')}</dd></div>
          <div><dt>RUN ID(S)</dt><dd>{summary.runs.join(', ')}</dd></div>
          <div><dt>TIME SPAN</dt><dd>{fmt(summary.start, 3)} → {fmt(summary.end, 3)} s (recording clock)</dd></div>
          <div><dt>MEDIAN FRAME INTERVAL</dt><dd>{summary.medianDt == null ? '—' : `${fmt(summary.medianDt * 1000, 1)} ms`}</dd></div>
          <div><dt>SEQUENCE GAPS (missing frames)</dt><dd>{summary.gaps}</dd></div>
          <div><dt>SEQUENCE REVERSALS</dt><dd>{summary.reversals}</dd></div>
          <div><dt>FRAMES WITH STAGE DIAGNOSTICS</dt><dd>{summary.withDiagnostics}</dd></div>
        </dl>
        <p className="lab-note">Frames are ordered by timestamp then sequence by the loader; invalid records are dropped. <Link to="/replay">Play this recording in Replay</Link>.</p>
      </section>
      <section className="lab-card" aria-labelledby="dataset-frames">
        <h2 id="dataset-frames">Frames</h2>
        <div className="lab-toolbar" role="toolbar" aria-label="Frame table pages">
          <button onClick={() => setPage(Math.max(0, page - 1))} disabled={page === 0}>Previous {PAGE}</button>
          <span>Page {page + 1} / {pages}</span>
          <button onClick={() => setPage(Math.min(pages - 1, page + 1))} disabled={page >= pages - 1}>Next {PAGE}</button>
        </div>
        <div className="lab-table-wrap" role="region" aria-label="Recorded frames" tabIndex={0}><table className="lab-table">
          <thead><tr>{['#', 'SEQ', 'TIME s', 'SOURCE', 'CONES', 'DELAUNAY', 'GABRIEL', 'GATES', 'CANDIDATES', 'SPEED m/s', 'STEER REQ rad', ''].map((name) => <th key={name || 'action'} scope="col">{name || <span className="lab-visually-hidden">Inspect</span>}</th>)}</tr></thead>
          <tbody>{visible.map((item, i) => { const index = page * PAGE + i; return <tr key={`${item.sequence}-${index}`} aria-selected={index === selected}>
            <td>{index + 1}</td><td>{item.sequence}</td><td>{fmt(item.timestamp_s, 3)}</td><td>{item.source}</td><td>{item.perception.cones.length}</td>
            <td>{item.planning.delaunay_edges.length}</td><td>{item.planning.gabriel_edges.length}</td><td>{item.planning.gates.length}</td><td>{item.planning.candidate_paths.length}</td>
            <td>{fmt(item.vehicle.speed_mps)}</td><td>{fmt(item.vehicle.requested_steering_rad, 3)}</td>
            <td><button className="lab-link-button" onClick={() => setSelected(index)} aria-label={`Inspect frame ${index + 1}`}>Inspect</button></td></tr>; })}</tbody>
        </table></div>
      </section>
      {frame && <section className="lab-card" aria-labelledby="dataset-frame">
        <h2 id="dataset-frame">Frame {selected + 1} · sequence {frame.sequence}</h2>
        <dl className="lab-facts lab-facts-wide">
          <div><dt>TIMESTAMP</dt><dd>{fmt(frame.timestamp_s, 3)} s</dd></div>
          <div><dt>POSE</dt><dd>{fmt(frame.vehicle.x_m)}, {fmt(frame.vehicle.y_m)} m · {fmt(frame.vehicle.yaw_rad, 3)} rad</dd></div>
          <div><dt>SENSOR RANGE / FOV</dt><dd>{fmt(frame.perception.sensor_range_m, 1)} m / {fmt(frame.perception.sensor_fov_rad, 2)} rad</dd></div>
          <div><dt>STAGE DIAGNOSTICS</dt><dd>{frame.diagnostics ? (['perception', 'mapping', 'planning'] as const).map((stage) => `${stage}: ${frame.diagnostics?.[stage]?.valid == null ? '—' : frame.diagnostics?.[stage]?.valid ? 'valid' : 'invalid'}`).join(' · ') : 'not recorded'}</dd></div>
        </dl>
        <div className="lab-table-wrap" role="region" aria-label="Cones in selected frame" tabIndex={0}><table className="lab-table">
          <caption>Detected cones ({frame.perception.cones.length})</caption>
          <thead><tr>{['ID', 'CLASS', 'X m', 'Y m', 'CONF', 'RANGE m', 'BEARING rad'].map((name) => <th key={name} scope="col">{name}</th>)}</tr></thead>
          <tbody>{frame.perception.cones.length ? frame.perception.cones.map((cone) => <tr key={cone.id}><td>{cone.id}</td><td>{cone.class}</td><td>{fmt(cone.x_m)}</td><td>{fmt(cone.y_m)}</td><td>{fmt(cone.confidence)}</td><td>{fmt(cone.range_m)}</td><td>{fmt(cone.bearing_rad, 3)}</td></tr>)
            : <tr><td colSpan={7} className="lab-muted">No detections in this frame.</td></tr>}</tbody>
        </table></div>
      </section>}
    </>}
  </article>;
}
