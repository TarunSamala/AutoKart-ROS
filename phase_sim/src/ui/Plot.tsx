import './plot.css';

export type PlotPoint = { t: number; v: number | null | undefined };

/**
 * Simple legible SVG time-series line plot. Points whose value is null/undefined break the
 * line (rendered as a gap, never interpolated or coerced to 0) so an "n/a" stretch reads as
 * missing data, not as zero.
 */
export function Plot({ label, unit = '', series, height = 72, domain }: {
  label: string;
  unit?: string;
  series: PlotPoint[];
  height?: number;
  /** Fixed y-domain [min, max]; auto-computed from the data when omitted. */
  domain?: [number, number];
}) {
  const width = 280;
  const known = series.filter((p) => p.v != null && Number.isFinite(p.v as number));
  const hasData = known.length > 0;
  const minT = hasData ? series[0].t : 0;
  const maxT = hasData ? series[series.length - 1].t : 1;
  const spanT = Math.max(1e-6, maxT - minT);
  const [minV, maxV] = domain ?? (hasData
    ? [Math.min(...known.map((p) => p.v as number)), Math.max(...known.map((p) => p.v as number))]
    : [0, 1]);
  const spanV = Math.max(1e-6, maxV - minV);
  const pad = 6;
  const x = (t: number) => pad + ((t - minT) / spanT) * (width - 2 * pad);
  const y = (v: number) => height - pad - ((v - minV) / spanV) * (height - 2 * pad);

  // Build one or more disjoint polyline segments, breaking at any null/undefined sample.
  const segments: string[] = [];
  let current: string[] = [];
  for (const p of series) {
    if (p.v == null || !Number.isFinite(p.v as number)) {
      if (current.length > 1) segments.push(current.join(' '));
      current = [];
      continue;
    }
    current.push(`${x(p.t).toFixed(1)},${y(p.v as number).toFixed(1)}`);
  }
  if (current.length > 1) segments.push(current.join(' '));

  const last = known.length ? known[known.length - 1].v as number : null;

  return (
    <div className="ui-plot">
      <div className="ui-plot-head">
        <span className="ui-plot-label">{label}</span>
        <span className="ui-plot-value ui-mono">{last == null ? 'n/a' : `${last.toFixed(2)}${unit}`}</span>
      </div>
      {hasData ? (
        <svg className="ui-plot-svg" viewBox={`0 0 ${width} ${height}`} preserveAspectRatio="none" role="img" aria-label={`${label} over time`}>
          <line x1={pad} y1={height - pad} x2={width - pad} y2={height - pad} className="ui-plot-axis" />
          {segments.map((d, i) => <polyline key={i} points={d} className="ui-plot-line" />)}
        </svg>
      ) : (
        <div className="ui-plot-empty" role="status">n/a — no data yet</div>
      )}
    </div>
  );
}
