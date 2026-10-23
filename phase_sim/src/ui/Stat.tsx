import './stat.css';

/** Big number + unit + optional target/comparison, e.g. speed, steering, clearance readouts. */
export function Stat({ label, value, unit, target, tone = 'normal' }: {
  label: string;
  value: string | number;
  unit?: string;
  target?: string;
  tone?: 'normal' | 'good' | 'warn' | 'crit' | 'idle';
}) {
  return (
    <div className="ui-stat" data-tone={tone}>
      <span className="ui-stat-label">{label}</span>
      <span className="ui-stat-value ui-mono">{value}{unit && <small className="ui-stat-unit">{unit}</small>}</span>
      {target && <span className="ui-stat-target ui-mono">target {target}</span>}
    </div>
  );
}
