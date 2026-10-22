export const NO_DATA = '— (NO DATA)';
export const isNum = (v: unknown): v is number => typeof v === 'number' && Number.isFinite(v);

/** Format a number with a unit; missing / non-finite becomes an explicit NO DATA marker, never 0. */
export function num(v: unknown, digits = 1, unit = ''): string {
  return isNum(v) ? `${v.toFixed(digits)}${unit ? ` ${unit}` : ''}` : NO_DATA;
}
export function txt(v: unknown): string {
  if (v === null || v === undefined || v === '') return NO_DATA;
  if (typeof v === 'number' && !Number.isFinite(v)) return NO_DATA;
  if (typeof v === 'boolean') return v ? 'YES' : 'NO';
  if (typeof v === 'object') return JSON.stringify(v);
  return String(v);
}
export function ns(v: unknown): string {
  return isNum(v) ? `${(v / 1e9).toFixed(3)} s (monotonic)` : NO_DATA;
}
export function pt(p: unknown, digits = 2): string {
  return Array.isArray(p) && isNum(p[0]) && isNum(p[1]) ? `(${p[0].toFixed(digits)}, ${p[1].toFixed(digits)}) m` : NO_DATA;
}
export function age(raw: unknown): string {
  if (!isNum(raw)) return NO_DATA;
  const ms = Math.max(0, raw); // timers can land a few ms before the poll that stamped the value
  return ms >= 10_000 ? `${(ms / 1000).toFixed(1)} s` : `${ms.toFixed(0)} ms`;
}
export const wallClock = (d = new Date()) => d.toISOString().slice(11, 23);
