import { useCallback, useEffect, useRef, useState, type ReactNode } from 'react';
import { gabrielRejected, stageSummary, type Diagnostic, type StageId, type XY } from '../../../shared/labStages';
import { Link } from '../router';
import { useReportProvenance } from '../runContext';
import { labJson, type CoreSnapshot } from '../services';

/**
 * Focused, single-stage views of the CURRENT Python shared_core evaluation (lab_api :8767).
 * Every value comes from the backend diagnostic. The only browser-side computation is the
 * Gabriel diametral-circle cross-check, which is labelled as a display-only cross-check.
 */
export type CoreStage = 'delaunay' | 'gabriel' | 'gates' | 'paths';

const STAGE_META: Record<CoreStage, { title: string; stageIds: StageId[]; intro: string }> = {
  delaunay: { title: 'Delaunay triangulation', stageIds: ['observe', 'delaunay'],
    intro: 'Valid observed cones (base_link: X forward, Y left) and the triangulation shared_core computed from them (geometry.triangulate).' },
  gabriel: { title: 'Gabriel filter', stageIds: ['gabriel'],
    intro: 'A Delaunay edge is kept only if no other cone lies inside the circle whose diameter is that edge (gabriel.filter_gabriel).' },
  gates: { title: 'Gates and corridor', stageIds: ['boundaries', 'gates'],
    intro: 'Left/right boundary assignment and the left↔right cone pairs shared_core selected or rejected as gates.' },
  paths: { title: 'Path and controller candidate', stageIds: ['centerline', 'mission', 'control'],
    intro: 'Gate midpoints ordered into a centerline with heading/curvature, and the Pure-Pursuit candidate. Controller output is sim-only and never published to a VCU.' },
};

const fmt = (value: number | null | undefined, places = 2) => value == null || !Number.isFinite(value) ? '—' : value.toFixed(places);
const K = 40, OX = 40, OY = 200;
const sx = (x: number) => OX + x * K;
const sy = (y: number) => OY - y * K;
const edgeKey = (a: number, b: number) => a < b ? `${a}-${b}` : `${b}-${a}`;

/** Display-only cross-check: observation ids strictly inside the edge's diametral circle. */
export function insideDiametralCircle(a: XY, b: XY, points: Array<{ id: number; xy: XY }>, skip: number[]): number[] {
  const cx = (a[0] + b[0]) / 2, cy = (a[1] + b[1]) / 2;
  const r2 = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) / 4;
  return points.filter((p) => !skip.includes(p.id) && (p.xy[0] - cx) ** 2 + (p.xy[1] - cy) ** 2 < r2 * (1 - 1e-9)).map((p) => p.id);
}

function Table({ caption, head, rows, empty }: { caption: string; head: string[]; rows: ReactNode[][]; empty: string }) {
  return <div className="lab-table-wrap" role="region" aria-label={caption} tabIndex={0}>
    <table className="lab-table"><caption>{caption}</caption>
      <thead><tr>{head.map((name) => <th key={name} scope="col">{name}</th>)}</tr></thead>
      <tbody>{rows.length ? rows.map((row, i) => <tr key={i}>{row.map((cell, j) => <td key={j}>{cell}</td>)}</tr>)
        : <tr><td colSpan={head.length} className="lab-muted">{empty}</td></tr>}</tbody>
    </table>
  </div>;
}

function StageFigure({ stage, diagnostic }: { stage: CoreStage; diagnostic: Diagnostic }) {
  const points = new Map(diagnostic.observations.map((item) => [item.id, item.xy]));
  const line = (a: number, b: number, className: string, key: string) => {
    const p = points.get(a), q = points.get(b);
    return p && q ? <line key={key} x1={sx(p[0])} y1={sy(p[1])} x2={sx(q[0])} y2={sy(q[1])} className={className} /> : null;
  };
  const g = diagnostic.geometry;
  const poly = (ids: number[]) => ids.map((id) => points.get(id)).filter((p): p is XY => !!p).map((p) => `${sx(p[0])},${sy(p[1])}`).join(' ');
  const target = diagnostic.control_candidate.target_point_base_m;
  return <figure className="lab-figure">
    <svg viewBox="0 0 600 400" role="img" aria-label={`${STAGE_META[stage].title} in base_link frame`}>
      <rect width="600" height="400" className="fig-bg" />
      {Array.from({ length: 15 }, (_, i) => <line key={`gx${i}`} x1={sx(i)} x2={sx(i)} y1="0" y2="400" className="fig-grid" />)}
      {Array.from({ length: 11 }, (_, i) => <line key={`gy${i}`} y1={sy(i - 5)} y2={sy(i - 5)} x1="0" x2="600" className="fig-grid" />)}
      <line x1="0" x2="600" y1={OY} y2={OY} className="fig-axis" /><line x1={OX} x2={OX} y1="0" y2="400" className="fig-axis" />
      {stage === 'delaunay' && <>
        {g.delaunay_triangles?.map((tri, i) => {
          const corners = tri.map((id) => points.get(id));
          return corners.every(Boolean) ? <polygon key={`t${i}`} className="fig-triangle" points={corners.map((p) => `${sx(p![0])},${sy(p![1])}`).join(' ')} /> : null;
        })}
        {g.delaunay_edges.map(([a, b], i) => line(a, b, 'fig-delaunay', `d${i}`))}
      </>}
      {stage === 'gabriel' && <>
        {gabrielRejected(diagnostic).map(([a, b], i) => line(a, b, 'fig-rejected', `r${i}`))}
        {g.gabriel_edges.map(([a, b], i) => line(a, b, 'fig-gabriel', `g${i}`))}
      </>}
      {stage === 'gates' && <>
        <polyline points={poly(g.boundaries.left_ids)} className="fig-left" />
        <polyline points={poly(g.boundaries.right_ids)} className="fig-right" />
        {g.rejected_candidates?.map((gate, i) => line(gate.left_id, gate.right_id, 'fig-rejected', `rc${i}`))}
        {g.gates.map((gate, i) => <g key={`gate${i}`}>{line(gate.left_id, gate.right_id, gate.status === 'CANDIDATE' ? 'fig-gate' : 'fig-rejected', `gl${i}`)}
          <circle cx={sx(gate.midpoint[0])} cy={sy(gate.midpoint[1])} r="4" className="fig-mid" /></g>)}
      </>}
      {stage === 'paths' && <>
        {g.centerline.length > 1 && <polyline points={g.centerline.map((p) => `${sx(p[0])},${sy(p[1])}`).join(' ')} className="fig-path" />}
        {target && <><line x1={sx(0)} y1={sy(0)} x2={sx(target[0])} y2={sy(target[1])} className="fig-lookahead" />
          <circle cx={sx(target[0])} cy={sy(target[1])} r="7" className="fig-target" /></>}
      </>}
      {diagnostic.observations.map((item) => <g key={`o${item.id}`}>
        <circle cx={sx(item.xy[0])} cy={sy(item.xy[1])} r="7" className={item.label === 'blue' ? 'fig-cone-blue' : item.label === 'yellow' ? 'fig-cone-yellow' : 'fig-cone-other'} />
        <text x={sx(item.xy[0]) + 9} y={sy(item.xy[1]) - 8} className="fig-label">{item.id}</text>
      </g>)}
      <path d={`M ${sx(0) + 14} ${sy(0)} L ${sx(0) - 10} ${sy(0) - 8} L ${sx(0) - 10} ${sy(0) + 8} Z`} className="fig-kart" />
      <text x="50" y="22" className="fig-caption">base_link · X forward → · Y left ↑ · grid 1 m</text>
    </svg>
    <figcaption>Drawn from the lab_api diagnostic only. Cone numbers are observation ids.</figcaption>
  </figure>;
}

function StageTables({ stage, diagnostic }: { stage: CoreStage; diagnostic: Diagnostic }) {
  const g = diagnostic.geometry;
  const points = new Map(diagnostic.observations.map((item) => [item.id, item.xy]));
  const length = (a: number, b: number) => { const p = points.get(a), q = points.get(b); return p && q ? Math.hypot(p[0] - q[0], p[1] - q[1]) : null; };
  if (stage === 'delaunay') return <>
    <Table caption={`Input vertices (${diagnostic.observations.length} valid observations)`} head={['ID', 'CLASS', 'X m', 'Y m', 'CONF', 'RANGE m']} empty="No valid observations this frame."
      rows={diagnostic.observations.map((o) => [o.id, o.label, fmt(o.xy[0]), fmt(o.xy[1]), fmt(o.confidence), fmt(o.distance_m)])} />
    <Table caption={`Triangles (${g.delaunay_triangles?.length ?? 'not reported'})`} head={['#', 'VERTEX IDS']} empty={g.delaunay_triangles ? 'No triangles (needs ≥3 non-collinear cones).' : 'Backend did not report triangles.'}
      rows={(g.delaunay_triangles ?? []).map((tri, i) => [i + 1, tri.join(' · ')])} />
    <p className="lab-note">Degenerate / duplicate handling: shared_core drops invalid and duplicate cones before triangulation; the diagnostic lists only the cones it kept, so dropped inputs are not shown here.</p>
  </>;
  if (stage === 'gabriel') {
    const kept = new Set(g.gabriel_edges.map(([a, b]) => edgeKey(a, b)));
    const pts = diagnostic.observations.map((o) => ({ id: o.id, xy: o.xy }));
    const rows = g.delaunay_edges.map(([a, b]) => {
      const p = points.get(a), q = points.get(b);
      const inside = p && q ? insideDiametralCircle(p, q, pts, [a, b]) : null;
      const isKept = kept.has(edgeKey(a, b));
      const agrees = inside == null ? null : isKept === (inside.length === 0);
      return [`${a} ↔ ${b}`, fmt(length(a, b)), isKept ? 'KEPT' : 'REMOVED', inside == null ? '—' : inside.length ? inside.join(', ') : 'none',
        agrees == null ? '—' : agrees ? 'agrees' : <b className="lab-warn">DISAGREES</b>];
    });
    return <>
      <Table caption={`Delaunay edges tested (${g.delaunay_edges.length}) · kept ${g.gabriel_edges.length} · removed ${gabrielRejected(diagnostic).length}`}
        head={['EDGE', 'LENGTH m', 'shared_core RESULT', 'CONES INSIDE DIAMETRAL CIRCLE (browser cross-check)', 'CROSS-CHECK']} rows={rows} empty="No Delaunay edges this frame." />
      <p className="lab-note">Rejection reasons are not part of the lab_api diagnostic. The cross-check column is computed in the browser from the same observation coordinates, for display only; shared_core's result is authoritative.</p>
    </>;
  }
  if (stage === 'gates') return <>
    <Table caption={`Selected gates (${g.gates.length})`} head={['LEFT', 'RIGHT', 'WIDTH m', 'QUALITY', 'STATUS', 'MIDPOINT (base_link)']} empty="No gates selected this frame."
      rows={g.gates.map((gate) => [gate.left_id, gate.right_id, fmt(gate.width), fmt(gate.quality), gate.status, `${fmt(gate.midpoint[0])}, ${fmt(gate.midpoint[1])}`])} />
    <Table caption={`Rejected candidate pairs (${g.rejected_candidates?.length ?? 'not reported'})`} head={['LEFT', 'RIGHT', 'REASON']} empty={g.rejected_candidates ? 'No rejected candidates.' : 'Backend did not report rejected candidates.'}
      rows={(g.rejected_candidates ?? []).map((item) => [item.left_id, item.right_id, item.reason])} />
    <p className="lab-note">LEFT / RIGHT are observation ids from this frame (the numbers on the figure), so every gate traces back to its input cones.</p>
  </>;
  const c = diagnostic.control_candidate;
  return <>
    <Table caption={`Centerline profile (${g.path_profile?.length ?? g.centerline.length} samples)`} head={['#', 'X m', 'Y m', 'HEADING rad', 'SEGMENT m', 'CURVATURE 1/m']} empty="No centerline this frame."
      rows={(g.path_profile ?? g.centerline.map((xy) => ({ xy, heading_rad: NaN, segment_m: NaN, curvature_m_inv: null }))).map((s, i) => [i + 1, fmt(s.xy[0]), fmt(s.xy[1]), fmt(s.heading_rad, 3), fmt(s.segment_m), fmt(s.curvature_m_inv, 3)])} />
    <section className="lab-card lab-sim-only" aria-label="Controller candidate, simulation only">
      <h3>Controller candidate · SIM-ONLY / DIAGNOSTIC</h3>
      <dl className="lab-facts">
        <div><dt>VALID</dt><dd>{String(c.valid)}</dd></div>
        <div><dt>TARGET SPEED</dt><dd>{fmt(c.target_speed_mps)} m/s</dd></div>
        <div><dt>STEERING</dt><dd>{fmt(c.steering_angle_rad, 3)} rad</dd></div>
        <div><dt>LOOKAHEAD (base_link)</dt><dd>{c.target_point_base_m ? `${fmt(c.target_point_base_m[0])}, ${fmt(c.target_point_base_m[1])} m` : '—'}</dd></div>
        <div><dt>PUBLISHED TO VCU</dt><dd>{String(c.published_to_vcu ?? false)}</dd></div>
        <div><dt>VEHICLE AUTHORIZED</dt><dd>{String(diagnostic.vehicle_authorized)}</dd></div>
      </dl>
      <p className="lab-note">This drives only the simulated kart inside lab_api. Nothing in the Lab can send it to a vehicle.</p>
    </section>
    <p className="lab-note"><b>Not implemented in this build:</b> path variants and candidate ranking. shared_core returns one centerline per frame, so there are no alternative paths or rejection metrics to compare. The sandbox's candidate paths are on <Link to="/mission">Mission</Link> (client-side JS, not shared_core).</p>
  </>;
}

export function CoreStagePage({ stage }: { stage: CoreStage }) {
  const [snapshot, setSnapshot] = useState<CoreSnapshot | null>(null);
  const [error, setError] = useState('');
  const [busy, setBusy] = useState(false);
  const busyRef = useRef(false);
  const call = useCallback(async (path: string, body?: object) => {
    if (busyRef.current) return;
    busyRef.current = true; setBusy(true);
    try { setSnapshot(await labJson<CoreSnapshot>(`/api/core/${path}`, body)); setError(''); }
    catch (cause) { setError(cause instanceof Error ? cause.message : String(cause)); }
    finally { busyRef.current = false; setBusy(false); }
  }, []);
  useEffect(() => { void call('snapshot'); }, [call]);
  const meta = STAGE_META[stage];
  const diagnostic = snapshot?.diagnostic ?? null;
  useReportProvenance(snapshot ? 'SYNTHETIC' : null, snapshot ? `Python shared_core via lab_api :8767 · frame ${snapshot.tick} · source ${snapshot.source}` : '');
  const authorityViolation = diagnostic?.vehicle_authorized === true || diagnostic?.control_candidate.published_to_vcu === true;
  return <article className="lab-page lab-doc lab-stage" data-page={stage} aria-labelledby="stage-title">
    <p className="lab-eyebrow">ALGORITHM CORE · shared_core via lab_api :8767</p>
    <h1 id="stage-title">{meta.title}</h1>
    <p>{meta.intro}</p>
    <div className="lab-toolbar" role="toolbar" aria-label="Core frame controls">
      <button onClick={() => void call('step', {})} disabled={busy}>Single frame</button>
      <button onClick={() => void call('reset', {})} disabled={busy}>Reset simulation</button>
      <button onClick={() => void call('snapshot')} disabled={busy}>Refresh</button>
      <Link to="/core" className="lab-button-link">Open in Core Lab (drag cones)</Link>
      <span className="lab-toolbar-meta">Frame <b data-testid="core-tick">{snapshot?.tick ?? '—'}</b> · algorithm <code>{diagnostic?.algorithm_version ?? '—'}</code> · config <code title={diagnostic?.configuration_sha256}>{(diagnostic?.configuration_sha256 ?? '—').slice(0, 12)}</code></span>
    </div>
    <p className="lab-note">The CORE simulation is one shared instance: Single frame and Reset here also advance / reset the Core Lab page.</p>
    {authorityViolation && <p className="lab-error" role="alert">UNEXPECTED: backend reports vehicle_authorized / published_to_vcu = true. The Lab must never authorize a vehicle — investigate lab_api.</p>}
    {error && <p className="lab-error" role="alert">Lab CORE API unavailable ({error}). Start <code>autokart_perception.lab_api</code> on :8767. Nothing is computed in the browser as a fallback.</p>}
    {!snapshot && !error && <p role="status">Waiting for lab_api…</p>}
    {snapshot && !diagnostic && <p role="status">lab_api returned a snapshot without a diagnostic yet. Press Single frame.</p>}
    {diagnostic && <div className="lab-stage-layout">
      <StageFigure stage={stage} diagnostic={diagnostic} />
      <aside className="lab-card" aria-label="Stage summary">
        {meta.stageIds.map((id) => { const summary = stageSummary(diagnostic, id); return <section key={id}>
          <h2>{id.toUpperCase()} <small className={`lab-state state-${summary.state.replace('/', '')}`}>{summary.state}</small></h2>
          <dl className="lab-facts">{summary.facts.map(([name, value]) => <div key={name}><dt>{name}</dt><dd>{value}</dd></div>)}</dl>
        </section>; })}
        <p className="lab-note">Reasons: {diagnostic.reasons.join(', ') || 'none'}</p>
      </aside>
    </div>}
    {diagnostic && <StageTables stage={stage} diagnostic={diagnostic} />}
  </article>;
}
