import './bar.css';

/** Horizontal gauge bar: fraction of [min,max] filled, with a label and formatted value. */
export function Bar({ label, value, min = 0, max = 1, unit = '', tone = 'normal' }: {
  label: string;
  value: number | null | undefined;
  min?: number;
  max?: number;
  unit?: string;
  tone?: 'normal' | 'good' | 'warn' | 'crit';
}) {
  const known = value != null && Number.isFinite(value);
  const pct = known ? Math.max(0, Math.min(100, ((value! - min) / (max - min || 1)) * 100)) : 0;
  return (
    <div className="ui-bar" data-tone={known ? tone : 'idle'}>
      <div className="ui-bar-row">
        <span className="ui-bar-label">{label}</span>
        <span className="ui-bar-value ui-mono">{known ? `${value!.toFixed(2)}${unit}` : 'n/a'}</span>
      </div>
      <div className="ui-bar-track"><div className="ui-bar-fill" style={{ width: `${pct}%` }} /></div>
    </div>
  );
}
