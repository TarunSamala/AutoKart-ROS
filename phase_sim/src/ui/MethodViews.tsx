import type { InspectionSelection, PlanningSnapshot, Point2D } from '../engine/types';
import { fixed } from './format';

function bounds(snapshot: PlanningSnapshot) {
  const points = snapshot.sensor.visibleCones.length ? snapshot.sensor.visibleCones : [snapshot.vehicle];
  const xs = points.map((point) => point.x);
  const ys = points.map((point) => point.y);
  const minX = Math.min(...xs) - 0.6;
  const maxX = Math.max(...xs) + 0.6;
  const minY = Math.min(...ys) - 0.6;
  const maxY = Math.max(...ys) + 0.6;
  return { minX, minY, width: Math.max(1, maxX - minX), height: Math.max(1, maxY - minY) };
}

function points(items: Point2D[]) {
  return items.map((point) => `${point.x},${-point.y}`).join(' ');
}

function MiniView({ snapshot, stage, onSelect }: { snapshot: PlanningSnapshot; stage: 1 | 2 | 3 | 4; onSelect?: (selection: InspectionSelection) => void }) {
  const box = bounds(snapshot);
  const viewBox = `${box.minX} ${-(box.minY + box.height)} ${box.width} ${box.height}`;
  return (
    <svg viewBox={viewBox} preserveAspectRatio="xMidYMid meet" role="img" aria-label={`Interactive planning stage ${stage}`}>
      {stage === 1 && snapshot.delaunay.triangles.map((triangle) => <polygon onClick={() => onSelect?.({ kind: 'triangle', id: triangle.id })} key={triangle.id} points={points(triangle.vertices)} className="mini-triangle all interactive-geometry" />)}
      {stage === 2 && snapshot.filtering.exteriorTriangles.map((triangle) => <polygon onClick={() => onSelect?.({ kind: 'triangle', id: triangle.id })} key={triangle.id} points={points(triangle.vertices)} className="mini-triangle rejected interactive-geometry" />)}
      {stage >= 2 && snapshot.filtering.interiorTriangles.map((triangle) => <polygon onClick={() => onSelect?.({ kind: 'triangle', id: triangle.id })} key={triangle.id} points={points(triangle.vertices)} className="mini-triangle interior interactive-geometry" />)}
      {stage === 3 && snapshot.edges.crossTrackEdges.map((edge) => <line onClick={() => onSelect?.({ kind: 'edge', id: edge.id })} key={edge.id} x1={edge.a.x} y1={-edge.a.y} x2={edge.b.x} y2={-edge.b.y} className="mini-edge interactive-geometry" />)}
      {stage >= 3 && snapshot.midpoints.map((midpoint) => <circle onClick={() => onSelect?.({ kind: 'midpoint', id: midpoint.id })} key={midpoint.id} cx={midpoint.x} cy={-midpoint.y} r={0.09} className="mini-midpoint interactive-geometry" />)}
      {stage === 4 && snapshot.avoidance.active && <polyline points={points(snapshot.interpolation.basePath)} className="mini-path base" />}
      {stage === 4 && <polyline points={points(snapshot.interpolation.path)} className={snapshot.avoidance.active ? 'mini-path selected-route' : 'mini-path'} />}
      {stage === 4 && snapshot.interpolation.path.filter((_, index) => index % 8 === 0).map((point) => <circle onClick={() => onSelect?.({ kind: 'path', id: String(point.index) })} key={point.index} cx={point.x} cy={-point.y} r={0.055} className="mini-path-point interactive-geometry" />)}
      {snapshot.sensor.visibleCones.map((cone) => <circle onClick={() => onSelect?.({ kind: 'cone', id: cone.id })} key={cone.id} cx={cone.x} cy={-cone.y} r={0.08} className={`${cone.side === 'BLUE' ? 'mini-blue' : 'mini-yellow'} interactive-geometry`} />)}
    </svg>
  );
}

export function MethodViews({ snapshot, onSelect }: { snapshot: PlanningSnapshot; onSelect?: (selection: InspectionSelection) => void }) {
  const stages = [
    ['1', 'DELAUNAY', `${snapshot.delaunay.triangles.length} triangles`],
    ['2', 'REMOVE EXTERIOR', `${snapshot.filtering.interiorTriangles.length} retained`],
    ['3', 'MIDPOINTS', `${snapshot.midpoints.length} estimates`],
    ['4', 'INTERPOLATE', `${snapshot.interpolation.path.length} samples`],
  ] as const;
  return (
    <section className="methodology" aria-label="Planning methodology views">
      {stages.map(([number, title, value], index) => <article key={number}>
        <header><span>{number}</span><b>{title}</b><small>{value}</small></header>
        <MiniView snapshot={snapshot} stage={(index + 1) as 1 | 2 | 3 | 4} onSelect={onSelect} />
      </article>)}
    </section>
  );
}

function HistoryGraph({ title, unit, values, times, color, symmetric = false }: { title: string; unit: string; values: number[]; times: number[]; color: string; symmetric?: boolean }) {
  const width = 240;
  const height = 68;
  const pad = 8;
  let min = values.length ? Math.min(...values) : 0;
  let max = values.length ? Math.max(...values) : 1;
  if (symmetric) {
    const magnitude = Math.max(Math.abs(min), Math.abs(max), 0.01);
    min = -magnitude;
    max = magnitude;
  }
  if (max - min < 1e-6) { max += 0.5; min -= 0.5; }
  const minTime = times.length ? times[0] : 0;
  const maxTime = times.length ? times[times.length - 1] : 1;
  const spanTime = Math.max(0.01, maxTime - minTime);
  const plotted = values.map((value, index) => {
    const x = pad + ((times[index] - minTime) / spanTime) * (width - pad * 2);
    const y = pad + (1 - (value - min) / (max - min)) * (height - pad * 2);
    return `${x},${y}`;
  }).join(' ');
  return <article className="history-graph"><header><b>{title}</b><span>{fixed(values.at(-1) ?? 0, 2)} {unit}</span></header><svg viewBox={`0 0 ${width} ${height}`} preserveAspectRatio="none"><line x1={pad} y1={height / 2} x2={width - pad} y2={height / 2} className="graph-grid" /><polyline points={plotted} fill="none" stroke={color} strokeWidth="2" vectorEffect="non-scaling-stroke" /></svg><small>last {fixed(spanTime, 1)} s</small></article>;
}

export function HistoryGraphs({ snapshot }: { snapshot: PlanningSnapshot }) {
  const history = snapshot.history;
  const times = history.map((item) => item.time);
  return <section className="history-grid" aria-label="Rolling engineering history">
    <HistoryGraph title="STEERING ANGLE" unit="deg" times={times} values={history.map((item) => item.steeringAngle * 180 / Math.PI)} color="#ffbd4a" symmetric />
    <HistoryGraph title="PATH TRACKING ERROR" unit="m" times={times} values={history.map((item) => item.crossTrackError)} color="#57d6c4" />
    <HistoryGraph title="VELOCITY" unit="m/s" times={times} values={history.map((item) => item.velocity)} color="#6fb9ff" />
    <HistoryGraph title="VISIBLE CONES" unit="cones" times={times} values={history.map((item) => item.visibleCones)} color="#e9edf0" />
    <HistoryGraph title="PLANNER TIME" unit="ms" times={times} values={history.map((item) => item.plannerMs)} color="#fb7464" />
  </section>;
}
